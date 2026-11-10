"""低开销黑框检测：Otsu/局部阈值四边形提案 + 窄带验证 + 当前帧精修。

候选四角按轮廓环序排列，不强制图像坐标的 TL/TR/BR/BL，因此框旋转后角仍
一一对应。每边黑带的物理宽度参与验证，透视单应性将物理矩形中心映射回图像。
每帧优先检查上一帧的扩展 ROI，失败才扫低分辨率全图；不运行全图 Canny/Hough。
"""
import math
import time

import cv2
import numpy as np


def _area(q):
    return float(cv2.contourArea(np.asarray(q, np.float32).reshape(-1, 1, 2), oriented=True))


def _order_quad(points):
    """循环顺序，不锚定左上角；转动后相邻边仍对应。"""
    p = np.asarray(points, np.float32).reshape(4, 2)
    center = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0]))]
    if _area(p) < 0:
        p = p[::-1]
    return np.roll(p, -int(np.argmin(p[:, 0] + p[:, 1])), axis=0).copy()


def _edge_lengths(q):
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def _rect(w, h, inset=0.0):
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float32)


def project_points(matrix, points):
    p = np.asarray(points, np.float32).reshape(1, -1, 2)
    return cv2.perspectiveTransform(p, np.asarray(matrix, np.float64)).reshape(-1, 2)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    a = np.arange(int(samples), dtype=np.float32) * (2 * np.pi / int(samples))
    return (np.array([width_mm / 2, height_mm / 2], np.float32) +
            float(radius_mm) * np.column_stack((np.cos(a), np.sin(a))).astype(np.float32))


def _line_intersection_center(q):
    """两对角线交点是平面矩形中心的精确透视投影。"""
    p = np.asarray(q, np.float64)
    a, b = p[2] - p[0], p[3] - p[1]
    den = a[0] * b[1] - a[1] * b[0]
    if abs(den) < 1e-8:
        raise ValueError("degenerate diagonal intersection")
    d = p[1] - p[0]
    t = (d[0] * b[1] - d[1] * b[0]) / den
    return p[0] + t * a


def _sample(gray, points):
    p = np.asarray(points, np.float32).reshape(-1, 2)
    return cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(points.shape[:-1])


class RectangleCenterDetector:
    def __init__(self, config):
        self.cfg = config
        self._track = None
        self._smooth = None
        self._frame = 0
        self._kernel = np.ones((3, 3), np.uint8)
        self._sample_t = np.linspace(0.12, 0.88, int(config.EDGE_SAMPLES), dtype=np.float32)
        self._circle_cache = {}
        self.last = None

    def _valid_quad(self, q, shape, allow_border=False):
        q = np.asarray(q, np.float32).reshape(4, 2)
        h, w = shape
        margin = 1 if allow_border else 2
        if (q[:, 0].min() < margin or q[:, 1].min() < margin or
                q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
            return False
        if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
            return False
        e = np.roll(q, -1, axis=0) - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < float(getattr(self.cfg, "MIN_QUAD_SIDE", 14)):
            return False
        cross = np.abs(e[:, 0] * np.roll(e[:, 1], -1) - e[:, 1] * np.roll(e[:, 0], -1))
        if np.min(cross / (lengths * np.roll(lengths, -1) + 1e-6)) < 0.16:
            return False
        return abs(_area(q)) >= float(getattr(self.cfg, "MIN_CONTOUR_AREA", 350))

    def _physical_layouts(self, q, kind):
        lengths = _edge_lengths(q)
        pair_a = (lengths[0] + lengths[2]) * 0.5
        pair_b = (lengths[1] + lengths[3]) * 0.5
        tape = float(self.cfg.BLACK_FRAME_MM)
        short = float(self.cfg.TARGET_SHORT_MM)
        long = float(self.cfg.TARGET_LONG_MM)
        if tape <= 0 or tape * 2 >= short:
            raise ValueError("BLACK_FRAME_MM must be less than half the short side")
        # 两种旋转对应关系都试；强透视使图像中的边长次序可能反转。
        dims = ((long, short), (short, long)) if pair_a >= pair_b else ((short, long), (long, short))
        return dims

    def _profile(self, gray, q, kind, width, height):
        """映射四边上少量点：暗带的两个点都须暗，框内纸面须明显更亮。"""
        tape = float(self.cfg.BLACK_FRAME_MM)
        physical_boundary = _rect(width, height, tape if kind == "inner" else 0.0)
        try:
            h_phys_to_img = cv2.getPerspectiveTransform(physical_boundary, np.asarray(q, np.float32))
        except cv2.error:
            return None
        if not np.all(np.isfinite(h_phys_to_img)):
            return None
        t = self._sample_t
        boundary = physical_boundary
        edges = np.roll(boundary, -1, axis=0) - boundary
        lengths = np.linalg.norm(edges, axis=1)
        if lengths.min() < 1:
            return None
        # Inward unit normals of the ordered physical quad.
        center = np.array([width * 0.5, height * 0.5], np.float32)
        mid = (boundary + np.roll(boundary, -1, axis=0)) * 0.5
        inward = center[None, :] - mid
        inward /= np.maximum(np.linalg.norm(inward, axis=1, keepdims=True), 1e-6)
        base = (boundary[:, None, :] * (1 - t[None, :, None]) +
                np.roll(boundary, -1, axis=0)[:, None, :] * t[None, :, None])
        # For an inner contour the band lies outside the paper. For an outer
        # contour it lies inside the outer edge. The inner paper witness is
        # always farther into the target than the sampled band.
        if kind == "inner":
            dark_offsets = (-0.35 * tape, -0.72 * tape)
            paper_offsets = 4.0
        else:
            dark_offsets = (0.35 * tape, 0.72 * tape)
            paper_offsets = tape + 4.0
        samples_phys = np.empty((4, len(t), 3, 2), np.float32)
        samples_phys[:, :, 0, :] = base + inward[:, None, :] * dark_offsets[0]
        samples_phys[:, :, 1, :] = base + inward[:, None, :] * dark_offsets[1]
        samples_phys[:, :, 2, :] = base + inward[:, None, :] * paper_offsets
        mapped = project_points(h_phys_to_img, samples_phys.reshape(-1, 2)).reshape(samples_phys.shape)
        gh, gw = gray.shape
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        values = _sample(gray, mapped).astype(np.float32)
        # Max of two black-band samples rejects thin monitor bezels whose
        # second sample has already crossed into the bright display area.
        dark = np.maximum(values[:, :, 0], values[:, :, 1])
        bright = values[:, :, 2]
        contrast = bright - dark
        coverage = ((contrast >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                coverage.min() < self.cfg.WEAK_EDGE_COVERAGE):
            return None
        score = float(np.median(contrast) + 60.0 * coverage.mean())
        return dict(matrix=h_phys_to_img, coverage=coverage, score=score,
                    contrast=float(np.median(contrast)), min_contrast=float(contrast.min()))

    def _verify(self, gray, q, kind):
        q = _order_quad(q)
        if not self._valid_quad(q, gray.shape):
            return None
        best = None
        for width, height in self._physical_layouts(q, kind):
            evidence = self._profile(gray, q, kind, width, height)
            if evidence is None:
                continue
            matrix = evidence["matrix"]
            # The selected edge's homography uses full physical-frame
            # coordinates, so it maps both the outer and inner edges directly.
            rect_to_image = matrix
            outer = project_points(rect_to_image, _rect(width, height)).astype(np.float32)
            inner = project_points(rect_to_image, _rect(width, height, float(self.cfg.BLACK_FRAME_MM))).astype(np.float32)
            if not self._valid_quad(outer, gray.shape):
                continue
            item = dict(quad=q, outer_quad=outer, inner_quad=inner, kind=kind,
                        rect_to_image=rect_to_image, width_mm=width, height_mm=height,
                        score=evidence["score"], contrast=evidence["contrast"],
                        min_contrast=evidence["min_contrast"], coverage=evidence["coverage"])
            if best is None or item["score"] > best["score"]:
                best = item
        return best

    def _refine(self, gray, q, kind, radius):
        """Current-frame narrow-band gradient samples -> robust line intersections."""
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        edge = nxt - q
        lengths = np.linalg.norm(edge, axis=1)
        if lengths.min() < 10:
            return None
        center = q.mean(axis=0)
        normals = center[None, :] - (q + nxt) * 0.5
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-6)
        base = q[:, None, :] * (1 - self._sample_t[None, :, None]) + nxt[:, None, :] * self._sample_t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        points = base[:, :, None, :] + normals[:, None, None, :] * offsets[None, None, :, None]
        h, w = gray.shape
        valid = ((points[..., 0] >= 1) & (points[..., 0] < w - 1) &
                 (points[..., 1] >= 1) & (points[..., 1] < h - 1)).all(axis=2)
        values = _sample(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]
        # inner contour: inward is white, outer contour: inward crosses dark tape.
        if kind == "outer":
            gradient *= -1
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.35
        idx = np.argmax(gradient - penalty, axis=2)
        strength = np.take_along_axis(gradient, idx[..., None], axis=2)[..., 0]
        shift = offsets[idx + 1]
        lines = []
        for i in range(4):
            good = valid[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.65)
            if good.sum() < max(5, len(self._sample_t) * self.cfg.WEAK_EDGE_COVERAGE):
                return None
            median = float(np.median(shift[i, good]))
            spread = float(np.median(np.abs(shift[i, good] - median)))
            good &= np.abs(shift[i] - median) <= max(2.5, spread * 3.0)
            if good.sum() < 5:
                return None
            pts = base[i, good] + shift[i, good, None] * normals[i]
            vx, vy, x, y = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            residual = np.abs((pts[:, 0] - x) * vy - (pts[:, 1] - y) * vx)
            if np.median(residual) > self.cfg.MAX_LINE_RESIDUAL_PX:
                return None
            lines.append([-vy, vx, vy * x - vx * y])
        lines = np.asarray(lines, np.float64)
        crossings = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crossings[:, 2]) < 1e-8):
            return None
        refined = (crossings[:, :2] / crossings[:, 2:]).astype(np.float32)
        if np.linalg.norm(refined - q, axis=1).max() > radius * 2.8:
            return None
        return refined if self._valid_quad(refined, gray.shape) else None

    def _candidate_quads(self, mask):
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        h, w = mask.shape
        min_area = max(float(self.cfg.MIN_CONTOUR_AREA), mask.size * self.cfg.MIN_AREA_RATIO)
        eligible = []
        # Area/bounding-box gates precede approximation; cap pathological scenes.
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            if min(bw, bh) < self.cfg.MIN_QUAD_SIDE or bw * bh < min_area:
                continue
            area = abs(float(cv2.contourArea(contour)))
            if min_area <= area <= mask.size * self.cfg.MAX_AREA_RATIO:
                eligible.append((area, contour))
        candidates = []
        # Retrieval order is spatial, not area ordered; select the bounded set
        # of largest connected dark regions before approximation.
        for area, contour in sorted(eligible, key=lambda item: item[0], reverse=True)[:self.cfg.MAX_CANDIDATES]:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.cfg.APPROX_EPSILON * perimeter, True)
            if len(approx) != 4:
                hull = cv2.convexHull(contour)
                approx = cv2.approxPolyDP(hull, self.cfg.APPROX_EPSILON * 1.4 * cv2.arcLength(hull, True), True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            q = _order_quad(approx.reshape(4, 2))
            if not self._valid_quad(q, mask.shape, allow_border=True):
                continue
            candidates.append((q, "outer", area))

        # When black tape touches a dark background, its outer contour merges
        # into the scene and the paper is no longer a hierarchy child. Search
        # the complementary bright mask for the enclosed paper contour. This
        # keeps the same geometry/edge checks while recovering that common case.
        bright = cv2.bitwise_not(mask)
        paper_contours = cv2.findContours(bright, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)[-2]
        paper_eligible = []
        for contour in paper_contours[:self.cfg.MAX_CONTOURS]:
            x, y, bw, bh = cv2.boundingRect(contour)
            if min(bw, bh) < self.cfg.MIN_QUAD_SIDE or bw * bh < min_area:
                continue
            area = abs(float(cv2.contourArea(contour)))
            if min_area <= area <= mask.size * self.cfg.MAX_AREA_RATIO:
                paper_eligible.append((area, contour))
        for area, contour in sorted(paper_eligible, key=lambda item: item[0], reverse=True)[:self.cfg.MAX_CANDIDATES]:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.cfg.APPROX_EPSILON * perimeter, True)
            if len(approx) != 4:
                hull = cv2.convexHull(contour)
                approx = cv2.approxPolyDP(hull, self.cfg.APPROX_EPSILON * 1.4 * cv2.arcLength(hull, True), True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            q = _order_quad(approx.reshape(4, 2))
            if not self._valid_quad(q, mask.shape, allow_border=True):
                continue
            candidates.append((q, "inner", area))

        # A closed hole and the complementary bright component often describe
        # the same edge; keep one representation to bound verification work.
        unique = []
        for candidate in sorted(candidates, key=lambda item: item[2], reverse=True):
            q, kind, area = candidate
            duplicate = any(np.mean(np.linalg.norm(q - old[0], axis=1)) < 5.0
                            and kind == old[1] for old in unique)
            if not duplicate:
                unique.append(candidate)
        return sorted(unique, key=lambda c: c[2], reverse=True)[:self.cfg.MAX_CANDIDATES]

    def _scan(self, gray, roi=None):
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        if crop.size == 0:
            return None
        scale = min(1.0, float(self.cfg.SEARCH_WIDTH) / w)
        sw, sh = max(32, round(w * scale)), max(24, round(h * scale))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        xy_scale = np.array([w / float(sw), h / float(sh)], np.float32)
        origin = np.array([x, y], np.float32)
        # One global threshold first; local threshold only if this scan has no
        # confirmed target. No threshold sweep or full-frame Hough search.
        otsu, _ = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        modes = (("otsu", int(otsu)), ("adaptive", None))
        for mode, threshold in modes:
            if mode == "otsu":
                _, mask = cv2.threshold(small, threshold, 255, cv2.THRESH_BINARY_INV)
            else:
                block = int(self.cfg.ADAPTIVE_BLOCK_SIZE)
                mask = cv2.adaptiveThreshold(small, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, block, self.cfg.ADAPTIVE_C)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            self._search_count += 1
            if mode == "otsu":
                self._threshold_count += 1
            for q, kind, _area in self._candidate_quads(mask):
                q = q * xy_scale + origin
                verified = self._verify(gray, q, kind)
                if verified is None:
                    continue
                refined = self._refine(gray, q, kind, self.cfg.REFINE_RADIUS)
                if refined is not None:
                    precise = self._verify(gray, refined, kind)
                    if precise is not None:
                        verified = precise
                verified["threshold"] = threshold if mode == "otsu" else "adaptive"
                return verified
        return None

    def _search_roi(self, gray):
        if self._track is None:
            return None
        q = self._track["outer_quad"]
        lo, hi = q.min(axis=0), q.max(axis=0)
        pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, self.cfg.ROI_MIN_PADDING)
        lo = np.maximum(0, np.floor(lo - pad)).astype(int)
        hi = np.minimum([gray.shape[1], gray.shape[0]], np.ceil(hi + pad)).astype(int)
        if np.any(hi - lo < self.cfg.MIN_ROI_SIZE):
            return None
        return int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1])

    def _empty(self, started, stage):
        result = dict(valid=False, stage=stage, search_count=self._search_count,
                      thresholds=self._threshold_count,
                      detector_ms=(time.perf_counter() - started) * 1000)
        self.last = result
        return result

    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        self._frame += 1
        self._search_count = self._threshold_count = 0
        candidate, stage = None, "search"
        if self._track is not None:
            old_q, kind = self._track["quad"], self._track["kind"]
            q = self._refine(g, old_q, kind, self.cfg.TRACK_RADIUS)
            if q is not None:
                candidate = self._verify(g, q, kind)
            if candidate is not None:
                stage = "track"
            else:
                # Predictive ROI then one full-frame reacquisition on the same
                # current frame. No stale corners or previous laser data escape.
                roi = self._search_roi(g)
                if roi is not None:
                    candidate = self._scan(g, roi)
                    if candidate is not None:
                        stage = "roi_search"
        if candidate is None:
            candidate = self._scan(g)
            if candidate is not None:
                stage = "full_search"
        if candidate is None:
            self._track = self._smooth = None
            return self._empty(started, "no_black_frame")

        self._track = candidate
        width, height = candidate["width_mm"], candidate["height_mm"]
        raw = _line_intersection_center(candidate["quad"])
        alpha = float(self.cfg.CENTER_ALPHA)
        if self._smooth is None or alpha <= 0 or np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX:
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
        key = (width, height, self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
        if key not in self._circle_cache:
            self._circle_cache[key] = standard_circle(width, height, self.cfg.CIRCLE_RADIUS_MM,
                                                       self.cfg.CIRCLE_SAMPLES)
        # Circle is for the existing debug/API contract; center itself is the
        # exact projective diagonal intersection and needs no per-frame warp.
        rect_to_image = candidate["rect_to_image"]
        result = dict(candidate)
        result.update(valid=True, u=float(center[0]), v=float(center[1]),
                      raw_u=float(raw[0]), raw_v=float(raw[1]),
                      quad=candidate["outer_quad"], detected_quad=candidate["quad"],
                      outer_inferred=candidate["kind"] == "inner",
                      homography=np.linalg.inv(rect_to_image),
                      circle_points=project_points(rect_to_image, self._circle_cache[key]),
                      stage=stage, search_count=self._search_count,
                      thresholds=self._threshold_count,
                      detector_ms=(time.perf_counter() - started) * 1000)
        self.last = result
        return result


def draw_result(gray, result, show_width=480):
    """绘制识别框与顶点；返回预览图，不改动检测坐标。"""
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))),
                       interpolation=cv2.INTER_AREA)
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(p[0] + 5, canvas.shape[1] - width - 1)),
               max(12, min(p[1] - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36,
                    (255, 255, 0), 1, cv2.LINE_AA)
    center = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, center, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(canvas, "(%d,%d) %s %.1fms" %
                (round(result["u"]), round(result["v"]), result["stage"], result["detector_ms"]),
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
