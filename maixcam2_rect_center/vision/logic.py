"""③ 逻辑层：流程编排、状态机、对外接口 —— `main.py` 的业务本体就在这里。

四层之一，装"决定该做什么"的全部代码，**一行副作用都不做**：

    结果契约   fail_result / finish_result
    检测器     RectangleCenterDetector（环-洞结构 + ROI 跟踪）
               LaserSpotDetector（光斑读数）
    闭环决策   GimbalAim（BOOT/COARSE/FINE/SETTLE/HOLD/FAULT）
               VisionControl（识别 → 偏差 → 决策；命令只填进 `self.commands`）
    主循环     `_capture_loop`（三条时间轴调度）/ `run`（装配）/ `_release`（按序释放）
    命令行入口 见根目录 `main.py`；对外脚本接口见根目录 `target_api.py`

依赖方向：**本层 import 轮询层、算法层、执行层**，反向一律禁止。
本层不直接产生副作用：

    绘制 → `output.draw_result` / `output.draw_preview` / `output.Preview`
    串口 → `output.stop_motion` / `output.apply_commands`

`link.move` / `link.stop` 在整份代码里只允许出现在执行层，逻辑层只产出
`{'yaw': (ccw, pulses, rpm) | None, 'pitch': ...}` 这样的决策值。

时序约定（不可改）：`stop_motion()` 返回 False 时**必须立刻终止**本轮决策链，
不能继续下发。"先停后走"与 ACK/E2 看门狗都依赖这一点。
"""
import math
import os
import threading
import time

import cv2
import numpy as np

from .algo import (deviation, edge_lengths, order_quad, profile_samples,
                   project_points, quad_diagonal_center, rect_quad,
                   refine_sample_points, sample_points, valid_quad)
from .output import (Preview, TouchButtons, aligned_frame, apply_commands, display_result,
                    draw_preview, perf_line, stop_motion)
from .poll import FramePoller, FrameRate, LatestFrameWorker, to_gray_frame
from .zdt import StepperLink, estimate_move_seconds, speed_profile

BUILD_ID = "1.11.1-2026-11-23"

# 主循环单轮超过该毫秒数（不含 show）即判为"慢"，打一次归因提示。
MAIN_LOOP_SLOW_MS = 66.0

# 结果阶段词汇（检测器写入 result["stage"]）
STAGE_PLANE = "plane_frame"
STAGE_NONE = "no_black_frame"


# ================================================================== 结果契约
def fail_result(stage, fail_stage, reason):
    """统一的失败结果。`fail_stage` 是走过的层，`reason` 是首次失败原因。"""
    return dict(valid=False, stage=stage, fail_stage=fail_stage, fail_reason=reason)


def finish_result(result, timestamp, image_size, started):
    """补齐时间戳、图像尺寸、耗时。"""
    result.update(captured_at=timestamp, image_size=image_size,
                  detector_ms=(time.perf_counter() - started) * 1000.0)
    return result


def _poly_area_py(quad):
    """四点多边形面积（纯 Python：板上每次 numpy 调用约 1ms，四点不值得付）。"""
    total = 0.0
    n = len(quad)
    for i in range(n):
        x1, y1 = float(quad[i][0]), float(quad[i][1])
        x2, y2 = float(quad[(i + 1) % n][0]), float(quad[(i + 1) % n][1])
        total += x1 * y2 - x2 * y1
    return abs(total) * 0.5


def _aspect_py(quad):
    """四边形长边/短边（>=1），纯 Python。仅作日志与显示的标量指标。"""
    n = len(quad)
    lengths = [math.hypot(float(quad[(i + 1) % n][0]) - float(quad[i][0]),
                          float(quad[(i + 1) % n][1]) - float(quad[i][1])) for i in range(n)]
    a = (lengths[0] + lengths[2]) * 0.5
    b = (lengths[1] + lengths[3]) * 0.5
    lo, hi = min(a, b), max(a, b)
    return float(hi / lo) if lo > 1e-6 else 0.0


# ================================================================== 编排
class RectangleCenterDetector:
    """黑框检测 + ROI 跟踪：低分辨率四边形提案 → 窄带黑带验证 → 当前帧四边精修。

    三段各自的理由（每一条都对应板上实测）：

    - **提案**：在 1/2 尺度上整幅一次 Otsu（无确认目标才退化到自适应阈值），
      连通域 + 多边形逼近出四边形。不用全图 Canny/Hough：板端一次 cv2 调用约 1ms，
      Hough 的参数扫描是几十次调用，帧率直接被吃掉。
    - **验证**：把物理矩形（纸面 + 胶带）按真实带宽向内、向外各取两点，
      要求"两点都暗、纸面一点明显更亮"，且四条边里至少三条覆盖达标。
      带宽因此进入判据 —— 细边框、印刷细环、洞洞板、门框在结构上就过不来。
    - **精修**：当前帧沿每条边取一排法线灰度，取梯度极值拟直线再求交，
      把提案阶段的降采样误差压到亚像素；精修不通过就退回提案值，不拿单帧失败换掉锁。
    - **中心取内框（纸边）对角线交点**：胶带上下不等宽时外框中心会偏出半个带宽，
      纸边不会。透视下每条边仍是直线，四线交角是几何精确解，不需要逐帧 warp。

    锁定后只在上帧框附近的 ROI 里重复同一套判据，失败才回全图重搜。
    """

    def __init__(self, config):
        self.cfg = config
        self.search_width = float(getattr(config, "SEARCH_WIDTH", 320))
        self.edge_samples = int(getattr(config, "EDGE_SAMPLES", 24))
        self.refine_radius = int(getattr(config, "REFINE_RADIUS", 6))
        self.track_radius = int(getattr(config, "TRACK_RADIUS", 12))
        self.min_contour_area = float(getattr(config, "MIN_CONTOUR_AREA", 350.0))
        self.min_quad_side = float(getattr(config, "MIN_QUAD_SIDE", 14.0))
        self.min_area_ratio = float(getattr(config, "MIN_AREA_RATIO", 0.025))
        self.max_area_ratio = float(getattr(config, "MAX_AREA_RATIO", 0.92))
        self.max_candidates = int(getattr(config, "MAX_CANDIDATES", 8))
        self.max_contours = int(getattr(config, "MAX_CONTOURS", 128))
        self.approx_epsilon = float(getattr(config, "APPROX_EPSILON", 0.025))
        self.max_line_residual = float(getattr(config, "MAX_LINE_RESIDUAL_PX", 2.0))
        self.adaptive_block = int(getattr(config, "ADAPTIVE_BLOCK_SIZE", 31))
        self.adaptive_c = float(getattr(config, "ADAPTIVE_C", 8.0))
        self.min_edge_coverage = float(getattr(config, "MIN_EDGE_COVERAGE", 0.55))
        self.weak_edge_coverage = float(getattr(config, "WEAK_EDGE_COVERAGE", 0.25))
        self.max_dark_ratio = float(getattr(config, "MAX_DARK_BRIGHT_RATIO", 0.72))
        self.min_edge_contrast = float(getattr(config, "MIN_EDGE_CONTRAST", 20.0))
        self.tape_mm = float(getattr(config, "BLACK_FRAME_MM", 18.0))
        self.aspect_min = float(getattr(config, "RECT_ASPECT_MIN", 1.05))
        self.aspect_max = float(getattr(config, "RECT_ASPECT_MAX", 3.40))
        self.short_mm = float(getattr(config, "TARGET_SHORT_MM", 210.0))
        self.long_mm = float(getattr(config, "TARGET_LONG_MM", 297.0))
        self.model_mm = tuple(getattr(config, "RECT_MODEL_MM", (297.0, 210.0)))
        self.roi_padding = float(getattr(config, "ROI_PADDING", 0.20))
        self.roi_min_padding = float(getattr(config, "ROI_MIN_PADDING", 24.0))
        self.min_roi_size = int(getattr(config, "MIN_ROI_SIZE", 32))
        self.center_alpha = float(getattr(config, "CENTER_ALPHA", 0.70))
        self.filter_reset = float(getattr(config, "FILTER_RESET_PX", 25.0))
        self.keep_stages = bool(getattr(config, "RECT_KEEP_STAGES", False))
        if self.tape_mm <= 0 or self.tape_mm * 2 >= self.short_mm:
            raise ValueError("BLACK_FRAME_MM must be less than half the short side")
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._sample_t = np.linspace(0.12, 0.88, max(6, self.edge_samples), dtype=np.float32)
        self._profile_cache = {}
        self._offset_cache = {}
        self.last_stages = {}
        self.last = None
        self.reset()

    def reset(self):
        """清空跟踪与平滑状态。换靶/重开一题/独立图片之间必须调用。"""
        self._track = None
        self._smooth = None
        self._frame = 0
        self._search_count = 0
        self._threshold_count = 0
        self._proposals = 0
        self._timing = dict(prep=0.0, comp=0.0, quad=0.0)

    # ------------------------------------------------------------- 内部工具
    def _valid_quad(self, q, shape, allow_border=False):
        return valid_quad(q, shape, self.min_quad_side, self.min_contour_area,
                          allow_border=allow_border)

    def _physical_layouts(self, q):
        """纸面横放还是竖放：两种对应关系都试，强透视会让图像边长次序反转。"""
        lengths = edge_lengths(q)
        pair_a = (lengths[0] + lengths[2]) * 0.5
        pair_b = (lengths[1] + lengths[3]) * 0.5
        short, long = self.short_mm, self.long_mm
        if pair_a >= pair_b:
            return ((long, short), (short, long))
        return ((short, long), (long, short))

    def _profile(self, gray, q, kind, width, height):
        """映射四边上的少量点：暗带两个采样点都须暗，框内的纸面须明显更亮。"""
        tape = self.tape_mm
        physical_boundary = rect_quad(width, height, tape if kind == "inner" else 0.0)
        try:
            phys_to_img = cv2.getPerspectiveTransform(physical_boundary, np.asarray(q, np.float32))
        except cv2.error:
            return None
        if not np.all(np.isfinite(phys_to_img)):
            return None
        key = (kind, width, height, tape)
        samples_phys = self._profile_cache.get(key)
        if samples_phys is None:
            samples_phys = profile_samples(kind, width, height, tape, self._sample_t)
            if samples_phys is None:
                return None
            self._profile_cache[key] = samples_phys
        mapped = project_points(phys_to_img, samples_phys.reshape(-1, 2)).reshape(samples_phys.shape)
        gh, gw = gray.shape
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        values = sample_points(gray, mapped).astype(np.float32)
        # 暗带两点取最大值：显示器窄边框的第二点可能已经跨进亮屏，取大值即判它不达标。
        dark = np.maximum(values[:, :, 0], values[:, :, 1])
        bright = values[:, :, 2]
        contrast = bright - dark
        coverage = ((contrast >= self.min_edge_contrast) &
                    (dark <= bright * self.max_dark_ratio)).mean(axis=1)
        if (np.count_nonzero(coverage >= self.min_edge_coverage) < 3 or
                coverage.min() < self.weak_edge_coverage):
            return None
        score = float(np.median(contrast) + 60.0 * coverage.mean())
        return dict(matrix=phys_to_img, coverage=coverage, score=score,
                    contrast=float(np.median(contrast)),
                    min_contrast=float(contrast.min()))

    def _verify(self, gray, q, kind):
        """把提案四边形当作物理解释一遍：带宽、亮度、覆盖率、内外框都自洽才算通过。"""
        q = order_quad(q)
        if not self._valid_quad(q, gray.shape):
            return None
        # 正方框（显示器、窗格）在这一步出局：靶纸的外框/纸边都不是正方形。
        # 这道门与带宽门互补 —— 带宽门管"暗带厚不厚"，它管"框像不像靶纸"。
        if not (self.aspect_min <= _aspect_py(q) <= self.aspect_max):
            return None
        best = None
        for width, height in self._physical_layouts(q):
            evidence = self._profile(gray, q, kind, width, height)
            if evidence is None:
                continue
            matrix = evidence["matrix"]
            outer = project_points(matrix, rect_quad(width, height)).astype(np.float32)
            inner = project_points(matrix, rect_quad(width, height, self.tape_mm)).astype(np.float32)
            if not self._valid_quad(outer, gray.shape):
                continue
            item = dict(quad=q, outer_quad=outer, inner_quad=inner, kind=kind,
                        rect_to_image=matrix, width_mm=width, height_mm=height,
                        score=evidence["score"], contrast=evidence["contrast"],
                        min_contrast=evidence["min_contrast"], coverage=evidence["coverage"])
            if best is None or item["score"] > best["score"]:
                best = item
        return best

    def _refine(self, gray, q, kind, radius):
        """当前帧窄带梯度采样 → 每边鲁棒拟线 → 相邻求交，得到亚像素四角。"""
        q = np.asarray(q, np.float32)
        if edge_lengths(q).min() < 10:
            return None
        points, base, normals, offsets = refine_sample_points(q, self._sample_t, radius)
        h, w = gray.shape
        valid = ((points[..., 0] >= 1) & (points[..., 0] < w - 1) &
                 (points[..., 1] >= 1) & (points[..., 1] < h - 1)).all(axis=2)
        values = sample_points(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]
        # 内框：向内是亮纸面（梯度为正）；外框：向内先跨过暗带（梯度为负）。
        if kind == "outer":
            gradient *= -1
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.35
        idx = np.argmax(gradient - penalty, axis=2)
        strength = np.take_along_axis(gradient, idx[..., None], axis=2)[..., 0]
        shift = offsets[idx + 1]
        min_points = max(5, int(len(self._sample_t) * self.weak_edge_coverage))
        lines = []
        for i in range(4):
            good = valid[i] & (strength[i] >= self.min_edge_contrast * 0.65)
            if good.sum() < min_points:
                return None
            median = float(np.median(shift[i, good]))
            spread = float(np.median(np.abs(shift[i, good] - median)))
            good &= np.abs(shift[i] - median) <= max(2.5, spread * 3.0)
            if good.sum() < 5:
                return None
            pts = base[i, good] + shift[i, good, None] * normals[i]
            vx, vy, x, y = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            residual = np.abs((pts[:, 0] - x) * vy - (pts[:, 1] - y) * vx)
            if np.median(residual) > self.max_line_residual:
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
        """暗块外轮廓（outer）+ 亮面补集轮廓（inner）都做提案，去重后按面积取前 N。"""
        h, w = mask.shape
        min_area = max(self.min_contour_area, mask.size * self.min_area_ratio)
        max_area = mask.size * self.max_area_ratio
        candidates = []
        for source, kind, limit in ((mask, "outer", None), (cv2.bitwise_not(mask), "inner", self.max_contours)):
            contours = cv2.findContours(source, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if limit is not None:
                contours = contours[:limit]
            eligible = []
            for contour in contours:
                _x, _y, bw, bh = cv2.boundingRect(contour)
                if min(bw, bh) < self.min_quad_side or bw * bh < min_area:
                    continue
                area = abs(float(cv2.contourArea(contour)))
                if min_area <= area <= max_area:
                    eligible.append((area, contour))
            # 取面积最大的若干个再做多边形逼近：检索顺序是空间顺序，不是面积顺序。
            for area, contour in sorted(eligible, key=lambda item: item[0], reverse=True)[:self.max_candidates]:
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, self.approx_epsilon * perimeter, True)
                if len(approx) != 4:
                    hull = cv2.convexHull(contour)
                    approx = cv2.approxPolyDP(hull, self.approx_epsilon * 1.4 * cv2.arcLength(hull, True), True)
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue
                q = order_quad(approx.reshape(4, 2))
                if not self._valid_quad(q, mask.shape, allow_border=True):
                    continue
                candidates.append((q, kind, area))
        # 闭合洞与亮面补集常常描述同一条边，留一个表示即可，避免重复验证。
        unique = []
        for candidate in sorted(candidates, key=lambda item: item[2], reverse=True):
            q, kind, _area = candidate
            duplicate = any(np.mean(np.linalg.norm(q - old[0], axis=1)) < 5.0 and kind == old[1]
                            for old in unique)
            if not duplicate:
                unique.append(candidate)
        return sorted(unique, key=lambda item: item[2], reverse=True)[:self.max_candidates]

    def _scan(self, gray, roi=None):
        """在 roi（或整幅）里做一次"降采样 → 阈值 → 提案 → 验证 → 精修"。"""
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        if crop.size == 0:
            return None
        started = time.perf_counter()
        scale = min(1.0, self.search_width / w)
        sw, sh = max(32, round(w * scale)), max(24, round(h * scale))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        xy_scale = np.array([w / float(sw), h / float(sh)], np.float32)
        origin = np.array([x, y], np.float32)
        # 先全局 Otsu；本次扫描没有确认目标才退化到局部阈值。不做阈值扫描、不做全图 Hough。
        otsu, otsu_mask = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        self._timing["prep"] += (time.perf_counter() - started) * 1000.0
        for mode in ("otsu", "adaptive"):
            started = time.perf_counter()
            if mode == "otsu":
                mask = otsu_mask
                threshold = int(otsu)
            else:
                block = self.adaptive_block + (1 - self.adaptive_block % 2)
                mask = cv2.adaptiveThreshold(small, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, block, self.adaptive_c)
                threshold = "adaptive"
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            self._threshold_count += 1
            self._search_count += 1
            if self.keep_stages:
                self.last_stages = dict(binary=mask)
            proposals = self._candidate_quads(mask)
            self._proposals += len(proposals)
            self._timing["prep"] += (time.perf_counter() - started) * 1000.0
            for q, kind, _area in proposals:
                q = q * xy_scale + origin
                started = time.perf_counter()
                verified = self._verify(gray, q, kind)
                self._timing["comp"] += (time.perf_counter() - started) * 1000.0
                if verified is None:
                    continue
                started = time.perf_counter()
                refined = self._refine(gray, q, kind, self.refine_radius)
                if refined is not None:
                    precise = self._verify(gray, refined, kind)
                    if precise is not None:
                        verified = precise
                self._timing["quad"] += (time.perf_counter() - started) * 1000.0
                verified["threshold"] = threshold
                return verified
        return None

    def _search_roi(self, gray):
        """上帧外框外扩一圈的搜索窗；窗太小说明框已贴边，直接回全图。"""
        if self._track is None:
            return None
        q = np.asarray(self._track["outer_quad"], np.float32)
        lo, hi = q.min(axis=0), q.max(axis=0)
        pad = np.maximum((hi - lo) * self.roi_padding, self.roi_min_padding)
        lo = np.maximum(0, np.floor(lo - pad)).astype(int)
        hi = np.minimum([gray.shape[1], gray.shape[0]], np.ceil(hi + pad)).astype(int)
        if np.any(hi - lo < self.min_roi_size):
            return None
        return int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1])

    def _fail_reason(self):
        """no_proposal = 画面里连候选块都没有；all_filtered = 有候选但带宽/亮度/覆盖率不过。"""
        return "all_filtered" if self._proposals else "no_proposal"

    # ------------------------------------------------------------- 对外入口
    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        ts = started if timestamp is None else float(timestamp)
        if not np.isfinite(ts):
            raise ValueError("non-finite timestamp")
        g = to_gray_frame(gray)
        self._frame += 1
        self._search_count = self._threshold_count = self._proposals = 0
        self._timing = dict(prep=0.0, comp=0.0, quad=0.0)
        candidate, search_mode = None, None
        if self._track is not None and g.size:
            old_q, kind = self._track["detected_quad"], self._track["kind"]
            q = self._refine(g, old_q, kind, self.track_radius)
            if q is not None:
                candidate = self._verify(g, q, kind)
            if candidate is not None:
                search_mode = "track"
            else:
                # 预测 ROI 内重搜一次；同一帧内完成，不拿上一帧的角点冒充本帧测量。
                roi = self._search_roi(g)
                if roi is not None:
                    candidate = self._scan(g, roi)
                    if candidate is not None:
                        search_mode = "roi_search"
        if candidate is None and g.size:
            candidate = self._scan(g)
            if candidate is not None:
                search_mode = "full_search"
        if candidate is None:
            self._track = self._smooth = None
            result = dict(valid=False, stage=STAGE_NONE, fail_stage="detect",
                          fail_reason=self._fail_reason(), center=None)
        else:
            candidate["detected_quad"] = candidate["quad"]
            self._track = candidate
            raw = quad_diagonal_center(candidate["inner_quad"])
            if raw is None:
                self._smooth = None
                result = dict(valid=False, stage=STAGE_NONE, fail_stage="detect",
                              fail_reason="degenerate", center=None)
            else:
                alpha = self.center_alpha
                if (self._smooth is None or alpha <= 0 or
                        math.hypot(raw[0] - self._smooth[0], raw[1] - self._smooth[1]) > self.filter_reset):
                    smooth = raw
                else:
                    smooth = (self._smooth[0] + alpha * (raw[0] - self._smooth[0]),
                              self._smooth[1] + alpha * (raw[1] - self._smooth[1]))
                self._smooth = smooth
                result = self._make_result(candidate, raw, smooth, search_mode)
        result.update(masks=1, proposals=self._proposals, search_count=self._search_count,
                      threshold=self._threshold_count, reject=dict(proposals=self._proposals),
                      t_ms=dict(self._timing), build=BUILD_ID)
        self.last = finish_result(result, ts,
                                  (int(g.shape[1]), int(g.shape[0])) if g.size else (0, 0),
                                  started)
        return self.last

    def _make_result(self, cand, raw, smooth, search_mode):
        """把验证通过的候选翻译成主轨结果契约（下游只认这里的字段）。

        逐帧不做单应反变换、不投影响应圆点：中心已经是内框对角线交点的几何解，
        每帧多付两次 numpy 调用（板端各约 1ms）换不到精度。
        """
        outer, inner = cand["outer_quad"], cand["inner_quad"]
        bx = outer[:, 0]
        by = outer[:, 1]
        return dict(valid=True, stage=STAGE_PLANE, search_mode=search_mode,
                    u=float(smooth[0]), v=float(smooth[1]),
                    raw_u=float(raw[0]), raw_v=float(raw[1]),
                    quad=outer, inner_quad=inner, detected_quad=cand["quad"],
                    outer_inferred=cand["kind"] == "inner", center_ref="inner",
                    bbox=(int(bx.min()), int(by.min()),
                          int(bx.max() - bx.min()), int(by.max() - by.min())),
                    area=_poly_area_py(outer), aspect=_aspect_py(inner),
                    band=dict(paper=cand["contrast"], contrast=cand["contrast"]),
                    confidence=float(np.mean(cand["coverage"])),
                    width_mm=float(self.model_mm[0]), height_mm=float(self.model_mm[1]),
                    rect_to_image=cand["rect_to_image"],
                    partial=False, hold=False, lost=0)


# ================================================================== 光斑
class LaserSpotDetector:
    """标称位置附近找最亮点，亚像素质心 + 限频 + 跳变门限。

    为什么只搜一个小 ROI：相机与激光笔刚性固连（同装在俯仰平台上），光斑在图像里的
    位置近似固定 —— 2026-10-07 现场一帧实测约 (294, 202)（640x480），背景灰度 145、
    峰值 177。因此不必全图找"最亮的东西"（那会把白墙高光、灯光、反光都算进去）。

    两种取位方式（`LASER_MODE`），云台侧拿到的 (u,v) 语义完全相同：
        "fixed"（默认） 光斑位置写死 = (LASER_U_NOMINAL, LASER_V_NOMINAL)，
            全程不做任何图像检测；每帧唯一的运算是"黑框中心 − 该常量"的两次减法。
            之所以可行：相机与激光刚性同转 → 光斑位置本来就是常量。
        "detect"        每 LASER_INTERVAL_S 在标称位附近 ROI 实测一次（自校准）。
            只在需要重新标定 U/V、或想验证"写死的值还准不准"时临时启用。

    输出契约（dict）：
        found    bool    本次是否有可信光斑（未更新时间沿用的仍是它）
        updated  bool    本次是否真的重算了（False = 限频沿用 / fixed 模式）
        u, v     float   光斑位置，相机原图坐标。**调用方必须先看 found**
        peak/bg  float   ROI 最大灰度 / 背景灰度
        area     int     超过阈值的像素数
        reason   str     原因码：ok / fixed / no_peak / too_big / too_small / jumped / roi
        ms       float   本次计算耗时（毫秒），未更新时为 0
    """

    def __init__(self, config):
        self.cfg = config
        self.mode = str(getattr(config, "LASER_MODE", "detect")).strip().lower()
        self.nominal = (float(config.LASER_U_NOMINAL), float(config.LASER_V_NOMINAL))
        self.interval = float(getattr(config, "LASER_INTERVAL_S", 0.30))
        self.alpha = float(getattr(config, "LASER_SMOOTH_ALPHA", 0.0))
        self.max_jump = float(getattr(config, "LASER_MAX_JUMP_PX", 40.0))
        self.last = dict(found=False, updated=False, u=0.0, v=0.0, peak=0.0, bg=0.0,
                         area=0, reason="init", ms=0.0)
        self._t = None
        self._uv = None

    def _detect(self, g):
        """在标称位附近的方形 ROI 内做一次"阈值 + 加权质心"，返回 (u,v)。

        判据链（任一不满足即判无光斑，并带 reason 便于排障）：
            峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)  → 否则 no_peak
            亮点像素数在 [LASER_MIN_PIXELS, LASER_MAX_PIXELS] 内    → 过大 too_big / 过小 too_small
        质心用 (灰度−阈值) 加权，越亮的像素权重越大 → 亚像素级、抗噪。
        """
        cfg = self.cfg
        H, W = g.shape[:2]
        cx, cy = float(cfg.LASER_U_NOMINAL), float(cfg.LASER_V_NOMINAL)
        r = int(cfg.LASER_ROI_RADIUS)
        # ROI = 标称位 ± r，并夹取到图像内（标称值决定"往哪儿看"，不决定结果）。
        x0 = max(0, int(cx - r))
        y0 = max(0, int(cy - r))
        x1 = min(W, int(cx + r) + 1)
        y1 = min(H, int(cy + r) + 1)
        if x1 - x0 < 5 or y1 - y0 < 5:
            return dict(found=False, reason="roi", peak=0.0, bg=0.0, area=0, u=cx, v=cy)
        roi = g[y0:y1, x0:x1]
        # 背景用 1/4 子采样中位数：抗光斑/噪点污染，又不必对 16k 像素排序。
        bg = float(np.median(roi[::4, ::4]))
        peak = float(roi.max())
        thr = max(float(cfg.LASER_MIN_PEAK), bg + float(cfg.LASER_MIN_CONTRAST))
        if peak < thr:
            return dict(found=False, reason="no_peak", peak=peak, bg=bg, area=0, u=cx, v=cy)
        mask = roi >= thr
        area = int(mask.sum())
        if area > int(cfg.LASER_MAX_PIXELS):
            # 大片高光/反光，不是激光点（激光点在 165cm 处只有几个像素）。
            return dict(found=False, reason="too_big", peak=peak, bg=bg, area=area, u=cx, v=cy)
        if area < int(cfg.LASER_MIN_PIXELS):
            return dict(found=False, reason="too_small", peak=peak, bg=bg, area=area, u=cx, v=cy)
        ys, xs = np.nonzero(mask)
        w = roi[ys, xs].astype(np.float32) - thr
        total = float(w.sum())
        if total <= 0:
            return dict(found=False, reason="flat", peak=peak, bg=bg, area=area, u=cx, v=cy)
        # 加权质心：权重 = 灰度−阈值；坐标加 ROI 原点 (x0,y0) 回到全图坐标。
        return dict(found=True, reason="ok", peak=peak, bg=bg, area=area,
                    u=x0 + float((xs * w).sum() / total), v=y0 + float((ys * w).sum() / total))

    @staticmethod
    def _fixed_dict(nominal):
        return dict(found=True, updated=False, u=nominal[0], v=nominal[1],
                    peak=0.0, bg=0.0, area=0, reason="fixed", ms=0.0)

    def static_state(self):
        """fixed 模式的常量结果：光斑位置写死，不做任何图像运算。"""
        return self._fixed_dict(self.nominal)

    def process(self, gray, now=None):
        """每帧调用；fixed 模式恒返回标定常量，detect 模式内部按 LASER_INTERVAL_S 限频。

        限频逻辑：距上次实测不足 interval 时**直接沿用上次结果**（updated=False, ms=0），
        因此每帧调用的代价近似为零；真正重算时依次做跳变门限与平滑。
        """
        if self.mode == "fixed":
            return self.static_state()
        now = time.perf_counter() if now is None else float(now)
        if self._t is not None and (now - self._t) < self.interval:
            out = dict(self.last)                 # 沿用上次：位置不变，仅标记未更新
            out["updated"] = False
            out["ms"] = 0.0
            return out
        self._t = now
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        started = time.perf_counter()
        res = self._detect(g)
        res["ms"] = (time.perf_counter() - started) * 1000.0
        res["updated"] = True
        if res["found"] and self._uv is not None and self.max_jump > 0:
            if abs(res["u"] - self._uv[0]) > self.max_jump or abs(res["v"] - self._uv[1]) > self.max_jump:
                # 单次跳太远：判为误检（高光/干扰物），保留上次位置。
                keep = dict(res)
                keep.update(found=True, reason="jumped", u=self._uv[0], v=self._uv[1])
                self.last = keep
                return keep
        if res["found"]:
            if self._uv is not None and 0.0 < self.alpha < 1.0:
                # 位置一阶低通（仅 detect 模式）；alpha=0 时关闭平滑，直接用最新质心。
                self._uv = (self._uv[0] + self.alpha * (res["u"] - self._uv[0]),
                            self._uv[1] + self.alpha * (res["v"] - self._uv[1]))
            else:
                self._uv = (res["u"], res["v"])
            res["u"], res["v"] = self._uv
        self.last = res
        return res



# ================================================================== 闭环决策
# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令


class GimbalAim:
    """基于像素误差的两轴增量比例控制器 + 状态机。

    - 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 `deviation`）；
    - 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅；
      速度按误差排（远快近慢）、起停斜坡随指令下发，驱动器自己走完加/减速；
    - 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / FAULT``；
    - 保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
      （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）、发散看护（``DIVERGE_GUARD``）。

    本类**保持纯 Python（不 import maix）**，可在 PC 上离线仿真。
    控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
    """

    def __init__(self, cfg, controller=None):
        self.cfg = cfg
        self.controller = controller
        self.state = BOOT
        self.fault_reason = None
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self.last = {"state": self.state, "dx": None, "dy": None,
                     "cmds": {"yaw": None, "pitch": None}}
        self._jump = {"yaw": 0, "pitch": 0}
        self._lost = 0
        self._busy_until = {"yaw": -1e9, "pitch": -1e9}
        self._capture_after = {"yaw": -1e9, "pitch": -1e9}
        self._direction = {"yaw": 0., "pitch": 0.}
        self._last_measurement_time = None
        self._stops = []
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}

    # ------------------------------------------------------------- 外部接口
    def fault(self, reason):
        """进入 FAULT：不再发任何指令，由上层决定是否复位。"""
        self.state = FAULT
        self.fault_reason = str(reason)
        print("[aim] FAULT: %s" % self.fault_reason)
        return self.state

    def reset(self):
        self.__init__(self.cfg, self.controller)
        self.state = BOOT
        self.fault_reason = None
        self._lost = 0
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        return self.state

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD）。"""
        now = float(now)
        self._stops = []
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid:
            return self._on_lost(now)
        dx, dy = float(result["dx"]), float(result["dy"])
        captured = float(result.get("captured_at", now))
        if not all(map(math.isfinite, (dx, dy, now, captured))) or captured > now:
            return self._on_lost(now)
        if self._last_measurement_time is not None:
            if captured <= self._last_measurement_time:
                return self._emit(dx, dy, {"yaw": None, "pitch": None})
            if captured-self._last_measurement_time > getattr(self.cfg, "AIM_HISTORY_MAX_GAP_S", .30):
                self._jump = {"yaw": 0, "pitch": 0}
                self._guard_error = {"yaw": None, "pitch": None}
                self._diverge = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = self._captured_at = captured
        self._lost = 0
        for axis, error in (("yaw", dx), ("pitch", dy)):
            if self._busy_until[axis] > now and (abs(error) <= self._deadband() or error*self._direction[axis] < 0):
                self._stops.append(axis)
                self._busy_until[axis] = now
                self._capture_after[axis] = now
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            self.state = SETTLE
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        self.state = COARSE if (coarse_px > 0 and max(abs(dx), abs(dy)) > coarse_px) else FINE
        cmds = {"yaw": self._axis_command("yaw", dx, now),
                "pitch": self._axis_command("pitch", dy, now)}
        return self._emit(dx, dy, cmds)

    def status_line(self):
        def fmt(cmd):
            if not cmd:
                return "-"
            return ("CCW" if cmd[0] else "CW") + "/%dp" % cmd[1]
        return "[aim] state=%s dx=%s dy=%s yaw=%s pit=%s lost=%d" % (
            self.last.get("state", self.state), self.last.get("dx"), self.last.get("dy"),
            fmt(self.last.get("cmds", {}).get("yaw")),
            fmt(self.last.get("cmds", {}).get("pitch")), self._lost)

    # ------------------------------------------------------------- 内部实现
    def _emit(self, dx, dy, cmds):
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds, "stops": list(self._stops)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _on_lost(self, now):
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}
        self._lost += 1
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _mix(self, error):
        fine, coarse = self._deadband(), float(getattr(self.cfg, "COARSE_PX", 24.))
        if not getattr(self.cfg, "AIM_PROGRESSIVE", True):
            return float(abs(error) > coarse)
        return max(0., min(1., (abs(error)-fine)/max(.001, coarse-fine)))

    def _gain(self, axis, error):
        fine = float(getattr(self.cfg, "GAIN_"+axis.upper(), .3))
        coarse = float(getattr(self.cfg, "GAIN_COARSE", .65))
        if not 0 < fine <= 1 or not 0 < coarse <= 1:
            raise ValueError("gain must be in (0, 1]")
        return fine + self._mix(error)*(coarse-fine)

    def _speed(self, error):
        """远快近慢：目标角速度随误差线性上升，上下限夹住。

        单位是**电机轴**度/秒；有减速比时输出端按比例更慢。这里只管"想多快"，
        真正的 rpm/acc 换算在驱动层 `zdt.speed_profile`。
        """
        per_px = float(getattr(self.cfg, "AIM_DEG_S_PER_PX", 0.5))
        low = float(getattr(self.cfg, "AIM_MIN_DEG_S", 8.0))
        high = float(getattr(self.cfg, "AIM_MAX_DEG_S", 60.0))
        return max(low, min(high, abs(float(error))*per_px))

    def mark_stopped(self, now, axis=None):
        for name in ("yaw", "pitch") if axis is None else (axis,):
            self._busy_until[name] = now
            self._capture_after[name] = now
            self._direction[name] = 0.

    def _axis_command(self, axis, error, now):
        cfg = self.cfg
        if self.state == FAULT:
            return None
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                return None
        else:
            self._jump[axis] = 0
        if abs(error) <= self._deadband():
            return None
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval:
            return None
        if getattr(cfg, "AIM_WAIT_MOTION", True) and (now < self._busy_until[axis] or self._captured_at <= max(self._busy_until[axis], self._capture_after[axis])):
            return None
        if getattr(cfg, "DIVERGE_GUARD", False):
            previous = self._guard_error[axis]
            if previous is not None:
                grew = abs(error) > abs(previous)*getattr(cfg, "DIVERGE_RATIO", 1.15)+getattr(cfg, "DIVERGE_MIN_PX", 4.)
                self._diverge[axis] = self._diverge[axis]+1 if grew and error*previous > 0 else 0
                if self._diverge[axis] >= getattr(cfg, "DIVERGE_CONFIRM_FRAMES", 6):
                    self.fault("diverge:"+axis)
                    return None
            self._guard_error[axis] = error
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or float(output) == 0:
                return None
            pulses = min(max_pulses, max(1, int(round(abs(float(output))))))
            ccw = float(output) * sign > 0
        else:
            ppp = float(getattr(cfg, "PULSE_PER_PX_YAW" if axis == "yaw"
                                else "PULSE_PER_PX_PITCH", 3.67))
            if not math.isfinite(ppp) or ppp <= 0:
                self.fault("invalid_scale:"+axis)
                return None
            pulses = min(max_pulses, max(1, int(round(abs(error) * ppp * self._gain(axis, error)))))
            ccw = error * sign > 0
        if not math.isfinite(sign) or abs(sign) != 1:
            self.fault("invalid_direction:"+axis)
            return None
        self.last_command_time[axis] = now
        # 速度按"误差大走得快、误差小走得慢"排，斜坡时间固定；驱动器自己完成
        # 加速—匀速—减速，一条指令就是一次完整动作，不会一顿一顿。
        rpm, acc = speed_profile(self._speed(error),
                                 float(getattr(cfg, "AIM_RAMP_S", 0.12)))
        self._direction[axis] = error
        self._busy_until[axis] = now+estimate_move_seconds(pulses, rpm, acc, getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))+getattr(cfg, "AIM_MOTION_MARGIN_S", .03)
        return ccw, pulses, rpm, acc



# ================================================================== 串口唯一所有者
class VisionControl:
    """识别 → 偏差 → 决策 → 下发。串口的唯一所有者（识别线程内调用）。

    职责边界：本类只做**决策**，任何 `link.move` / `link.stop` 都经 `apply_commands`
    与 `stop_motion` 走执行层；本类不直接写串口。
    """

    def __init__(self, cfg, link=None, aim=None, clock=None):
        self.cfg, self.link, self.aim = cfg, link, aim
        self._clock = clock if clock is not None else time
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == 'fixed'
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True
        self.control_source = 'vision'
        self.commands = {'yaw': None, 'pitch': None}
        self._action_lock = threading.Lock()
        self._requested_action = None
        self._action = None
        self._action_steps = ()
        self._action_index = 0
        self._action_ready_at = 0.0
        self._manual_hold = not getattr(cfg, 'AUTO_TRACK_ON_BOOT', True)

    def request_action(self, name):
        """May be called from the camera/touch thread; UART work stays in this worker."""
        if name not in ('search', 'self_test', 'stop'):
            raise ValueError('unknown gimbal action: ' + str(name))
        with self._action_lock:
            self._requested_action = name

    def _take_action(self):
        with self._action_lock:
            name, self._requested_action = self._requested_action, None
        return name

    def _start_action(self, name, now):
        if not self._stop_motion(now):
            return
        self._action = None
        if name == 'stop':
            self._manual_hold = True
            self.aim.update({'aim_valid': False}, now=now)
            return
        self._manual_hold = False
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', False):
            print('[gimbal] %s ignored: GIMBAL_CALIBRATED=False' % name)
            return
        ppr = int(self.cfg.MOTOR_PULSES_PER_REV)
        angle = float(self.cfg.SEARCH_ANGLE_DEG if name == 'search' else self.cfg.SELF_TEST_ANGLE_DEG)
        pulses = round(ppr * angle / 360.0)
        if ppr <= 0 or not 0 < angle <= 60 or pulses <= 0:
            self.aim.fault('invalid_action_angle')
            return
        if name == 'search':
            self._action_steps = (('yaw', True, pulses), ('yaw', False, 2*pulses),
                                  ('yaw', True, pulses))
            self.detector.reset()
        else:
            self._action_steps = (('yaw', True, pulses), ('yaw', False, 2*pulses),
                                  ('yaw', True, pulses), ('pitch', True, pulses),
                                  ('pitch', False, 2*pulses), ('pitch', True, pulses))
        self._action = name
        self._action_index = 0
        self._action_ready_at = now
        self.aim.update({'aim_valid': False}, now=now)
        print('[gimbal] %s started, angle=%.1f deg' % (name, angle))

    def _advance_action(self, result, now):
        if self._action == 'search' and result and result.get('valid') and not result.get('hold'):
            if not self._stop_motion(now):
                return
            self._action = None
            self.aim.reset()
            print('[gimbal] target acquired; tracking')
            return
        if now < self._action_ready_at:
            return
        if self._action_index >= len(self._action_steps):
            print('[gimbal] %s complete' % self._action)
            self._action = None
            self._manual_hold = True
            return
        axis, ccw, pulses = self._action_steps[self._action_index]
        rpm, acc = int(self.cfg.MANUAL_RPM), int(self.cfg.MANUAL_ACC)
        if not 0 < rpm <= 5000 or not 0 < acc <= 255:
            self.aim.fault('invalid_manual_speed')
            self._stop_motion(now)
            return
        ok, _, applied = apply_commands(self.link, self.aim,
                                         {axis: (ccw, pulses, rpm, acc)}, now, acc)
        if not ok:
            self._action = None
            return
        self.commands.update(applied)
        self._action_index += 1
        self._action_ready_at = (now + estimate_move_seconds(pulses, rpm, acc,
                                 self.cfg.MOTOR_PULSES_PER_REV) + self.cfg.MANUAL_SETTLE_S)

    def process(self, gray, captured_at):
        started = self._clock.perf_counter()
        result = self.detector.process(gray, timestamp=captured_at)
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, captured_at)
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result.update(laser_u=self.laser_state['u'], laser_v=self.laser_state['v'], dx=dv[0], dy=dv[1])
        result['aim_valid'] = dv is not None
        output = self.accept_result(result, captured_at)
        return dict(result=output, detector_result=result,
                    laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected-started)*1000, status=self.status())

    def accept_result(self, result, captured_at, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        captured_at = float(captured_at)
        fresh = (math.isfinite(now) and math.isfinite(captured_at) and
                 0 <= now-captured_at <= self.cfg.RESULT_MAX_AGE_S and captured_at > self.last_capture)
        output = dict(result) if fresh else dict(valid=False, aim_valid=False, stage='stale')
        output.update(captured_at=captured_at, age_s=now-captured_at)
        if fresh:
            self.last_capture = captured_at
        self._control(output, now, new_measurement=fresh)
        output.update(aim_state=self.aim.state if self.aim else None, cmds=dict(self.commands),
                      control_source=self.control_source)
        return output

    def idle(self):
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        if self.aim is not None and self.aim.state == 'FAULT':
            return 'FAULT'
        return self._action.upper() if self._action else (self.aim.state if self.aim else 'vision')

    def _stop_motion(self, now, axis=None):
        """停机（实现在执行层 `stop_motion`）。False = 停机失败，调用方必须终止决策链。"""
        return stop_motion(self.link, self.aim, now, axis)

    def _control(self, result, now, new_measurement):
        self.commands = {'yaw': None, 'pitch': None}
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        # FAULT before poll: even E2 re-enable is forbidden once motion is faulted.
        if aim.state == 'FAULT':
            return
        feedback = link.poll(now)
        if feedback and feedback.get('fault'):
            self._stop_motion(now)
            aim.fault('link:%s' % feedback.get('reason'))
            return
        requested = self._take_action()
        if requested is not None:
            self._start_action(requested, now)
            if aim.state == 'FAULT':
                return
        if self._action is not None:
            self._advance_action(result if new_measurement else None, now)
            return
        if self._manual_hold:
            aim.update({'aim_valid': False}, now=now)
            return
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', True):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
            aim.update({'aim_valid': False}, now=now)
            return
        if not new_measurement:
            if (result is not None and result.get('stage') == 'stale' or now-self.last_capture > self.cfg.RESULT_MAX_AGE_S) and not self.stale_stopped:
                self._stop_motion(now)
                aim.update({'aim_valid': False}, now=now)
                self.stale_stopped = True
            return
        if not result or not result.get('aim_valid'):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        if aim.state == 'FAULT':
            return
        output = aim.update(result or {'aim_valid': False}, now=now)
        if aim.state == 'FAULT':
            self._stop_motion(now)
            return
        for axis in output.get('stops', ()):
            if not self._stop_motion(now, axis):
                return
        # 顺序（stops 先、cmds 后）与失败路径是不可变的时序约定。
        ok, _failed, applied = apply_commands(link, aim, output['cmds'], now, self.cfg.ACC_TRACK)
        if not ok:
            return
        self.commands.update(applied)


# ================================================================== 主循环
def _capture_loop(cfg, clock, app, image, state, build_id):
    cam, worker, controller, preview = (state[k] for k in ('camera', 'worker', 'controller', 'preview'))
    capture_rate = FrameRate()
    last_log = clock.perf_counter()
    read_times, draw_times, loop_times = [], [], []
    poller = FramePoller(cam, image, getattr(cfg, 'LENS_CORR', False),
                         getattr(cfg, 'LENS_STRENGTH', 1.5), clock)
    buttons = state.get('buttons')
    while not app.need_exit():
        started = clock.perf_counter()
        if buttons is not None:
            action = buttons.poll(started)
            if action is not None:
                controller.request_action(action)
        gray, captured, read_ms = poller.poll()
        if worker.error is not None:
            raise RuntimeError('vision worker failed: ' + worker.error)
        if gray is None:
            if not cfg.VISION_ASYNC:
                controller.idle()
            continue
        read_times.append(read_ms)
        capture_rate.record(captured)
        worker.submit(gray, captured)
        now = clock.perf_counter()
        snapshot, fps, _ = worker.snapshot(now)
        if worker.error is not None:
            raise RuntimeError('vision worker failed: ' + worker.error)
        result, laser, age = display_result(snapshot, now, cfg.RESULT_MAX_AGE_S)
        if preview.due(now):
            t_draw = clock.perf_counter()
            try:
                draw_preview(cfg, preview, aligned_frame(snapshot, gray, result), result,
                              laser, age, fps, snapshot['status'] if snapshot else 'waiting', now)
            except Exception as exc:
                # 绘制/转换错误属于显示，绝不能升级成电机错误。
                preview.enabled = False
                print('[preview] render disabled: %s' % exc)
            draw_times.append((clock.perf_counter()-t_draw)*1000)
        loop_times.append((clock.perf_counter()-started)*1000)
        now = clock.perf_counter()
        if now-last_log >= cfg.LOG_INTERVAL:
            capture_fps, _ = capture_rate.values(now)
            display_fps, low1 = preview.rate.values(now)
            avg, p95, worst, n = preview.take_show_stats()
            mean = lambda values: float(np.mean(values)) if values else 0.
            loop_avg = mean(loop_times)
            detector_result = snapshot.get('detector_result', snapshot['result']) if snapshot else {}
            print(perf_line(build_id, capture_fps, display_fps, low1, fps, result,
                             mean(read_times), mean(draw_times), loop_avg, avg, p95, worst, n, age,
                             preview.shown, preview.dropped, worker.dropped,
                             detector_result=detector_result))
            metrics = worker.metrics(now)
            print('[throughput] valid_fps=%.1f valid_low1=%.1f worker_p95=%.1fms' %
                  (metrics['valid_fps'], metrics['valid_low1'], metrics['worker_p95_ms']))
            if state['aim'] is not None:
                print(state['aim'].status_line())
            if p95 > 100:
                preview.warn_slow_once(p95)
            if loop_avg > MAIN_LOOP_SLOW_MS:
                print('[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大' % loop_avg)
            last_log = now
            read_times.clear(); draw_times.clear(); loop_times.clear()


def _release(state):
    errors = []
    def close(resource):
        if resource is not None and hasattr(resource, 'close'):
            try:
                resource.close()
            except Exception as exc:
                errors.append(exc)
    worker = state.get('worker')
    close(worker)
    if worker is None or worker.thread is None or not worker.thread.is_alive():
        link = state.get('link')
        if link is not None:
            # 退出停机也走执行层入口：`stop_motion` 内部已捕获异常并回报 False。
            if stop_motion(link, None, time.perf_counter()) is False:
                errors.append(RuntimeError('stop on exit failed'))
        close(link)
    close(state.get('preview'))
    close(state.get('camera'))
    if errors:
        raise RuntimeError('cleanup: ' + '; '.join(map(str, errors)))


def run():
    # 兜底：绕过 main.py 直接调 run()（单文件版/其它入口）时同样关掉内置图传。
    os.environ.setdefault("MAIXVISION", "0")
    from config import Config
    cfg, clock = Config, time
    state = dict(camera=None, link=None, aim=None, worker=None, controller=None, preview=None)
    try:
        from maix import app, camera, image
        cv2.setNumThreads(cfg.OPENCV_THREADS)
        cam = state['camera'] = camera.Camera(cfg.CAMERA_W, cfg.CAMERA_H,
                                             image.Format.FMT_GRAYSCALE,
                                             fps=cfg.CAMERA_FPS, buff_num=cfg.CAMERA_BUFFERS)
        cam.skip_frames(cfg.SKIP_FRAMES)
        if cfg.ENABLE_GIMBAL:
            link = state['link'] = StepperLink(cfg, clock=clock)
            aim = state['aim'] = GimbalAim(cfg)
            if link.enable() is False:
                aim.fault('enable_failed')
            print('[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d' %
                  (cfg.YAW_DEV, cfg.ADDR_YAW, cfg.PITCH_DEV, cfg.ADDR_PITCH, cfg.ACC_TRACK))
        preview = state['preview'] = Preview(cfg, clock=clock)
        if getattr(cfg, 'ENABLE_TOUCH_BUTTONS', False) and preview.enabled:
            try:
                state['buttons'] = TouchButtons(preview.width, preview.height)
                preview.buttons = state['buttons']
            except Exception as exc:
                print('[touch] unavailable: %s' % exc)
        controller = state['controller'] = VisionControl(cfg, state['link'], state['aim'], clock=clock)
        worker = state['worker'] = LatestFrameWorker(controller.process, controller.idle,
                                                    cfg.VISION_ASYNC, clock=clock)
        print('[rect-center] %s search_width=%d tape=%.0fmm edge=%d refine=%d track_radius=%d '
              'display_width=%d display_fps=%d' %
              (BUILD_ID, cfg.SEARCH_WIDTH, cfg.BLACK_FRAME_MM, cfg.EDGE_SAMPLES,
               cfg.REFINE_RADIUS, cfg.TRACK_RADIUS, preview.render_width, cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC, preview.render_width, preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC, cfg.RESULT_MAX_AGE_S*1000))
        _capture_loop(cfg, clock, app, image, state, BUILD_ID)
    finally:
        _release(state)
