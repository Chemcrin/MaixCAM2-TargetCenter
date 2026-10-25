"""Canny/Otsu proposals -> gradient line refine -> four-side tape evidence -> center.

One search pass. Every proposal gets a cheap geometric prefilter, a narrow and a
wide line refine, and a soft tape score; the best candidate wins. There is no
verify/rescue cascade, no front-view warp and no per-side hard coverage gate.
Tracking refines the previous quadrilateral directly.

The target center is the intersection of the quadrilateral diagonals: a projective
map sends the rectangle's diagonal intersection (its center) to the image diagonal
intersection, so no homography is needed to publish the center.
"""
import time

import cv2
import numpy as np

from .contour_locator import ContourLocator
from .kalman_filter import ConstantVelocityKalman2D
from .rect_geometry import (homography, order_quad, polygon_area, project_points,
                            quad_is_valid, rectangle_corners, sample_batch,
                            side_lengths)


def _quad_center(q):
    """Diagonal intersection of an ordered quadrilateral (projective center)."""
    def line(a, b):
        return np.cross(np.r_[a, 1.], np.r_[b, 1.])
    p = np.cross(line(q[0], q[2]), line(q[1], q[3]))
    return (p[:2] / p[2]).astype(np.float64)


class RectangleCenterDetector:
    """有状态检测器：跨帧记住锁定框与平滑中心。

    搜索 = 两个 proposal 层（Canny/Otsu）→ 几何预筛 → 精修 → 四边胶带证据打分；
    跟踪 = 只在上一帧四边形周围做窄带精修，失败才回退 ROI/全图搜索。
    """

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._frame_shape = None
        self._kernel = np.ones((3, 3), np.uint8)
        # 四边均匀参数 t ∈ (0.12, 0.88)：避开透视下最易糊的角点。
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._layout_cache = {}
        self._physical_cache = {}
        self._offset_cache = {}
        self._search_count = 0
        self._mask_count = 0
        self._proposals = 0
        self.locator = ContourLocator(config)
        self._prev_timestamp = self._timestamp = None
        self._kalman = ConstantVelocityKalman2D(
            getattr(config, "KALMAN_PROCESS_PX", 40.), getattr(config, "KALMAN_MEASURE_PX", .9),
            getattr(config, "KALMAN_RESET_PX", 25.), getattr(config, "KALMAN_MAX_PREDICT_S", .25),
            getattr(config, "KALMAN_VELOCITY_LIMIT_PX_S", 6000.))

    def reset(self):
        self.last = self._smooth = self._track = None
        self._prev_timestamp = self._timestamp = None
        self._kalman.reset()

    def _valid_quad(self, q, shape, margin=1):
        return quad_is_valid(q, shape, margin, getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.))

    # ------------------------------------------------------------- physical layout
    def _layout(self, q, kind):
        """由四边形朝向推断物理长宽比，并给出该 kind 对应的物理矩形四角。"""
        lengths = side_lengths(q)
        horizontal = lengths[0] + lengths[2] > lengths[1] + lengths[3]
        w, h = ((self.cfg.TARGET_LONG_MM, self.cfg.TARGET_SHORT_MM) if horizontal
                else (self.cfg.TARGET_SHORT_MM, self.cfg.TARGET_LONG_MM))
        inset = self.cfg.BLACK_FRAME_MM if kind == "inner" else 0.0
        return w, h, self._physical_rectangle(w, h, inset)

    def _physical_rectangle(self, w, h, inset=0.):
        key = (w, h, inset)
        if key not in self._physical_cache:
            self._physical_cache[key] = rectangle_corners(w, h, inset)
        return self._physical_cache[key]

    # ------------------------------------------------------------- tape evidence
    def _evidence(self, gray, q, kind):
        """四边暗带证据（不设通过/拒绝门槛，门槛留在搜索评分处）。

        返回每边的 dark / inside / contrast / coverage、内外框、mm→px 单应。
        采样点落到图像外时返回 None：这种候选既量不准也不该外推中心。
        """
        if not self._valid_quad(q, gray.shape):
            return None
        w, h, physical_quad = self._layout(q, kind)
        tape = float(self.cfg.BLACK_FRAME_MM)
        if tape <= 0 or 2 * tape >= min(w, h):
            raise ValueError("BLACK_FRAME_MM must be between 0 and half the short side")
        try:
            rect_to_image = homography(physical_quad, q)
            key = (w, h, tape)
            if key not in self._layout_cache:
                offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
                t = self._t
                p = np.empty((4, len(t), 5, 2), np.float64)
                p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
                p[0, :, :, 1] = offsets
                p[1, :, :, 0] = w - offsets
                p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
                p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
                p[2, :, :, 1] = h - offsets
                p[3, :, :, 0] = offsets
                p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
                self._layout_cache[key] = p
            mapped = project_points(rect_to_image, self._layout_cache[key].reshape(-1, 2)).reshape(
                4, len(self._t), 5, 2)
        except (ValueError, np.linalg.LinAlgError):
            return None
        gh, gw = gray.shape
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = sample_batch(gray, mapped).astype(np.float32)
        dark = np.max(samples[:, :, :3], axis=2)
        bright = (samples[:, :, 3] + samples[:, :, 4]) * .5
        delta = bright - dark
        coverage = ((delta >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        rows = [dict(dark=float(np.median(dark[i])), inside=float(np.median(bright[i])),
                     contrast=float(np.median(delta[i])), coverage=float(coverage[i]))
                for i in range(4)]
        outer = q.copy() if kind == 'outer' else project_points(
            rect_to_image, self._physical_rectangle(w, h)).astype(np.float32)
        if kind != 'outer' and not self._valid_quad(outer, gray.shape):
            return None
        inner = q.copy() if kind == 'inner' else project_points(
            rect_to_image, self._physical_rectangle(w, h, tape)).astype(np.float32)
        return dict(quad=q, outer_quad=outer, inner_quad=inner, kind=kind,
                    rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=float(np.median(delta)),
                    min_contrast=float(delta.min()),
                    coverage=coverage, score=float(np.median(delta) + 60 * coverage.mean()))

    # ------------------------------------------------------------- line refine
    def _refine(self, gray, q, kind, radius):
        """在四边窄带内找有向梯度跳变，鲁棒拟合四条直线，交点即亚像素四角。"""
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        radius = int(radius)
        if radius not in self._offset_cache:
            offsets = np.arange(-radius, radius + 1, dtype=np.float32)
            self._offset_cache[radius] = (offsets, np.abs(offsets[1:-1])[None, None, :] * .45)
        offsets, penalty = self._offset_cache[radius]
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        probe_ok = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                    (points[..., 1] >= 1) & (points[..., 1] < gh - 1))
        values = sample_batch(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]
        if kind == "outer":
            gradient *= -1
        edge_ok = probe_ok[:, :, :-2] & probe_ok[:, :, 2:]
        indices = np.argmax(np.where(edge_ok, gradient - penalty, -np.inf), axis=2)
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        # 抛物线插值用**未加距离惩罚**的梯度峰；惩罚只用于挑最近的边。
        left_i = np.maximum(indices - 1, 0)
        right_i = np.minimum(indices + 1, gradient.shape[2] - 1)
        left = np.take_along_axis(gradient, left_i[..., None], axis=2)[..., 0]
        right = np.take_along_axis(gradient, right_i[..., None], axis=2)[..., 0]
        left = np.where(np.take_along_axis(edge_ok, left_i[..., None], axis=2)[..., 0], left, strength)
        right = np.where(np.take_along_axis(edge_ok, right_i[..., None], axis=2)[..., 0], right, strength)
        curvature = left - 2 * strength + right
        sub = np.zeros_like(strength)
        usable = (indices > 0) & (indices < gradient.shape[2] - 1) & (curvature < -1e-3)
        np.divide(.5 * (left - right), curvature, out=sub, where=usable)
        selected = offsets[indices + 1] + np.clip(sub, -.5, .5)
        good_edges = edge_ok.any(axis=2) & (strength >= self.cfg.MIN_EDGE_CONTRAST * .7)
        if good_edges.all():
            medians = np.median(selected, axis=1)
            deviations = np.median(np.abs(selected - medians[:, None]), axis=1)
        else:
            medians = deviations = None
        lines = []
        refitted = 0
        for i in range(4):
            good = good_edges[i].copy()
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                lines.append(np.cross(np.r_[q[i], 1.], np.r_[nxt[i], 1.]))
                continue
            shifts = selected[i, good]
            med = float(medians[i]) if medians is not None else float(np.median(shifts))
            mad = float(deviations[i]) if deviations is not None else float(np.median(np.abs(shifts - med)))
            good &= np.abs(selected[i] - med) <= max(2.5, mad * 3)
            if good.sum() < 5:
                return None
            samples = base[i, good] + selected[i, good, None] * normal[i]
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            residual = np.abs((samples[:, 0] - x) * vy - (samples[:, 1] - y) * vx)
            collinear = residual <= getattr(self.cfg, 'MAX_LINE_RESIDUAL_PX', .8)
            if collinear.sum() >= 5 and not collinear.all():
                vx, vy, x, y = cv2.fitLine(samples[collinear], cv2.DIST_HUBER, 0, .01, .01).reshape(4)
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
            refitted += 1
        if refitted < 4:
            return None
        lines = np.asarray(lines)
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None
        return refined

    # ------------------------------------------------------------- scoring helpers
    @staticmethod
    def _aspect(q):
        lengths = side_lengths(q)
        return float(max((lengths[0] + lengths[2]) * .5, (lengths[1] + lengths[3]) * .5) /
                     max(1., min((lengths[0] + lengths[2]) * .5, (lengths[1] + lengths[3]) * .5)))

    def _edge_support(self, gray, q, band=3):
        """四条边上强梯度的比例；只用于低分候选提前淘汰，不作硬门槛。"""
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 8:
            return 0.
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]
        t = np.linspace(.12, .88, 12, dtype=np.float32)
        base = q[:, None, :] * (1 - t[None, :, None]) + nxt[:, None, :] * t[None, :, None]
        ds = np.arange(-band, band + 1, dtype=np.float32)
        points = base[:, :, None, :] + normal[:, None, None, :] * ds[None, None, :, None]
        values = sample_batch(gray, points).astype(np.float32)
        grad = values[:, :, 2:] - values[:, :, :-2]
        return float((np.abs(grad).max(axis=2) >= 8.).mean())

    def _tape_ratio(self, gray, q, dark, bright):
        """实测暗带宽度 / 18mm 理论宽度；None 表示暗侧一直延伸到扫描上限（暗背景）。"""
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return 0.
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]
        short = float(min((lengths[0] + lengths[2]) * .5, (lengths[1] + lengths[3]) * .5))
        expected = max(2., short * self.cfg.BLACK_FRAME_MM / self.cfg.TARGET_SHORT_MM)
        maxd = max(4, int(expected * 2.6))
        t = np.linspace(.15, .85, 10, dtype=np.float32)
        base = q[:, None, :] * (1 - t[None, :, None]) + nxt[:, None, :] * t[None, :, None]
        ds = np.arange(1, maxd + 1, dtype=np.float32)
        thr = .5 * (float(dark) + float(bright))
        runs = []
        hit = 0
        for sign in (1., -1.):
            points = base[:, :, None, :] + normal[:, None, None, :] * (sign * ds)[None, None, :, None]
            values = sample_batch(gray, points).astype(np.float32)
            if sign > 0:
                pos = values
            else:
                neg = values
        for i in range(4):
            for j in range(pos.shape[1]):
                a, ha = self._run_length(pos[i, j], thr)
                b, hb = self._run_length(neg[i, j], thr)
                runs.append(max(a, b))
                hit += int(ha or hb)
        if not runs:
            return 0.
        if hit >= .5 * len(runs):
            return None
        return float(np.median(runs) / expected)

    @staticmethod
    def _run_length(values, thr):
        dark = values < thr
        if not dark.any():
            return 0., False
        start = int(np.argmax(dark))
        if start > 3:
            return 0., False
        bright_after = values[start + 1:] > thr
        if not bright_after.any():
            return float(len(values) - start), True
        return float(np.argmax(bright_after) + 1), False

    def _sides_ok(self, cand):
        """四边软证据：至少三边暗带明确，剩下的边必须确实很弱而不是一条亮边。"""
        rows = cand['edge_profile']
        good = 0
        for row in rows:
            ok = (row['contrast'] >= self.cfg.MIN_SIDE_CONTRAST and
                  row['dark'] <= self.cfg.MAX_DARK_BRIGHT_RATIO * max(1., row['inside']))
            good += int(ok)
            if not ok and not 0. < row['contrast'] < self.cfg.MAX_WEAK_SIDE_CONTRAST:
                return False
        return good >= self.cfg.MIN_STRONG_SIDES

    # ------------------------------------------------------------- search
    def _proposal_quads(self, gray, roi=None):
        """Canny + Otsu 两层候选；只去掉完全相同的四边形。"""
        out = []
        for level in (ContourLocator.LEVEL_MASK, ContourLocator.LEVEL_EDGE):
            quads = self.locator.locate(gray, roi, level)
            self._mask_count += self.locator.last_stats.get('masks', 0)
            for q, _ in quads:
                if any(np.max(np.linalg.norm(q - old, axis=1)) < 2 for old in out):
                    continue
                out.append(q)
        return out

    def _search(self, gray, roi=None):
        self._search_count += 1
        best = None
        area_img = float(gray.shape[0] * gray.shape[1])
        for q0 in self._proposal_quads(gray, roi):
            self._proposals += 1
            a0 = self._aspect(q0)
            if not self.cfg.MIN_ASPECT <= a0 <= self.cfg.MAX_ASPECT:
                continue
            if abs(polygon_area(q0)) * .5 < area_img * self.cfg.MIN_AREA_RATIO:
                continue
            for kind in ('outer', 'inner'):
                for radius in (self.cfg.REFINE_RADIUS, self.cfg.REFINE_RADIUS_WIDE):
                    q = self._refine(gray, q0, kind, radius)
                    if q is None:
                        continue
                    a = self._aspect(q)
                    area = abs(polygon_area(q)) * .5
                    if not self.cfg.MIN_ASPECT <= a <= self.cfg.MAX_ASPECT:
                        continue
                    if not area_img * self.cfg.MIN_AREA_RATIO <= area <= area_img * self.cfg.MAX_AREA_RATIO:
                        continue
                    cand = self._evidence(gray, q, kind)
                    if cand is None or cand['contrast'] < self.cfg.MIN_EDGE_CONTRAST:
                        continue
                    if not self._sides_ok(cand):
                        continue
                    dark = float(np.median([r['dark'] for r in cand['edge_profile']]))
                    bright = float(np.median([r['inside'] for r in cand['edge_profile']]))
                    if dark > self.cfg.MAX_DARK_BRIGHT_RATIO * max(1., bright):
                        continue
                    tape = self._tape_ratio(gray, q, dark, bright)
                    if tape is not None and tape < self.cfg.MIN_TAPE_RATIO:
                        continue
                    support = self._edge_support(gray, q)
                    if support < self.cfg.SUPPORT_MIN:
                        continue
                    con = min(1.5, max(0., cand['contrast'] / 80.))
                    score = (np.sqrt(area) * np.exp(-((np.log(a / (self.cfg.TARGET_LONG_MM /
                             self.cfg.TARGET_SHORT_MM))) / .45) ** 2) *
                             support ** 2 * (0.10 + float(np.mean(cand['coverage']))) * (0.25 + con))
                    if tape is None and score < self.cfg.SEARCH_MIN_SCORE:
                        continue
                    if best is None or score > best[0]:
                        best = (score, cand)
            if best is not None and best[0] >= self.cfg.SEARCH_EARLY_SCORE:
                break
        return best[1] if best is not None else None

    # ------------------------------------------------------------- center filtering
    def _continuous_time(self, newer, older):
        return (older is not None and np.isfinite([newer, older]).all() and
                0 < newer - older <= self.cfg.TRACK_MAX_GAP_S)

    def _filter_center(self, raw, timestamp):
        if getattr(self.cfg, 'KALMAN_ENABLE', True):
            self._smooth = self._kalman.update(raw, timestamp)
        else:
            alpha = self.cfg.CENTER_ALPHA
            if (self._smooth is None or alpha <= 0 or
                    np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX or
                    not self._continuous_time(timestamp, self._timestamp)):
                self._smooth = raw.copy()
            else:
                self._smooth += alpha * (raw - self._smooth)
        return self._smooth

    # ------------------------------------------------------------- main entry
    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        timestamp = started if timestamp is None else float(timestamp)
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError('process expects a uint8 grayscale image')
        if not np.isfinite(timestamp):
            raise ValueError('non-finite timestamp')
        if self._frame_shape != g.shape:
            self.reset()
            self._frame_shape = g.shape
        self._search_count = self._mask_count = self._proposals = 0
        if g.size == 0:
            self.reset()
            return dict(valid=False, stage='empty_frame', masks=0, search_count=0,
                        proposals=0, detector_ms=0.)
        candidate, stage = None, 'full_search'
        if self._track is not None:
            for kind in (self._track['kind'], 'outer' if self._track['kind'] == 'inner' else 'inner'):
                q = self._refine(g, self._track['quad'], kind, self.cfg.TRACK_RADIUS)
                if q is None:
                    continue
                cand = self._evidence(g, q, kind)
                if cand is not None and self._edge_support(g, q) >= self.cfg.TRACK_SUPPORT_MIN:
                    candidate, stage = cand, 'track'
                    break
            if candidate is None:
                q = self._track['outer_quad']
                span = q.max(axis=0) - q.min(axis=0)
                lo = np.maximum(0, q.min(axis=0) - span * self.cfg.ROI_PADDING - 24).astype(int)
                hi = np.minimum([g.shape[1], g.shape[0]],
                                q.max(axis=0) + span * self.cfg.ROI_PADDING + 24).astype(int)
                candidate = self._search(g, tuple(map(int, (*lo, *(hi - lo)))))
                stage = 'roi_search'
        if candidate is None:
            candidate, stage = self._search(g), 'full_search'
        if candidate is None:
            self._smooth = self._track = None
            self._kalman.reset()
            result = dict(valid=False, stage='no_black_frame')
        else:
            raw = _quad_center(candidate['quad'])
            center = self._filter_center(raw, timestamp)
            self._track, self._timestamp = candidate, timestamp
            result = dict(candidate)
            result.update(valid=True, stage=stage, u=float(center[0]), v=float(center[1]),
                          raw_u=float(raw[0]), raw_v=float(raw[1]),
                          quad=candidate['outer_quad'], detected_quad=candidate['quad'],
                          outer_inferred=candidate['kind'] == 'inner')
        result.update(captured_at=timestamp, image_size=(g.shape[1], g.shape[0]),
                      threshold=None, search_count=self._search_count, masks=self._mask_count,
                      thresholds=self._mask_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter() - started) * 1000)
        self.last = result
        return result


def draw_result(gray, result, show_width=480, top_offset=0):
    """缩小后转 BGR 描边；输出坐标仍是相机原图坐标。

    只画外框、内框和靶心十字——不画四角标签，减少板端绘制耗时。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    if result.get("inner_quad") is not None:
        iq = np.rint(result["inner_quad"] * scale).astype(np.int32)
        cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(canvas, "(%d,%d) %s %.1fms" % (round(result["u"]), round(result["v"]),
                result["stage"], result["detector_ms"]), (8, 22 + top_offset),
                cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
