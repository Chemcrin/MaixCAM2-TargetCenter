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
import time

import cv2
import numpy as np

from .algo import (auto_level, close_mask, components, dark_mask, deviation,
                   downscale, hole_quad, outer_from_inner, prefilter_boxes,
                   quad_diagonal_center, ring_hole)
from .output import (Preview, aligned_frame, apply_commands, display_result,
                    draw_preview, estimate_move_seconds, perf_line, stop_motion,
                    StepperLink)
from .poll import FramePoller, FrameRate, LatestFrameWorker, to_gray_frame

BUILD_ID = "2026-11-09-quad-track-v10"

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


# ================================================================== 编排
class RectangleCenterDetector:
    """黑框检测 + 跨帧跟踪（环-洞链路）。跟踪态每帧只有十来次图像调用。

    为什么这样组织（每一条都是板上实测出来的）：

    - **降采样 2 倍**再找块：像素少四倍，胶带环在 1/2 图上依旧连续；
    - **自适应阈值**（90 分位 × 0.62）：压在胶带与纸面之间，曝光变了阈值跟着变；
    - **连通域**而不是 findContours：一次调用拿到所有块，省掉几百次 Python 级调用；
    - **环里必须有洞**：洞就是纸面，洞的轮廓就是纸边矩形 —— 洞洞板（许多小洞）、
      门框立柱（没有洞）在 `ring_hole` 被结构性排除，这是 v9 误锁的解药；
    - **中心 = 洞四边拟线 → 四线交角 → 对角线交点**：透视几何精确解，
      纸面旋转、斜视（梯形）都准；不用单应、不用逐边剖面扫描（那是 v7 的 190ms/候选）；
    - **锁定后只在 ROI 里找** + 速度外推 + 短暂续命：画面晃一下、某帧阈值没抓住，
      按上一帧速度外推继续输出，不再立刻掉进 HOLD（v7 单帧失败即停机的解药）。
    """

    def __init__(self, config):
        self.cfg = config
        # 参数全部允许缺省，config 未同步时也不至于抛 AttributeError。
        self.scale = int(getattr(config, "RECT_DOWNSCALE", 2))
        self.level_ratio = float(getattr(config, "RECT_LEVEL_RATIO", 0.62))
        self.level_floor = float(getattr(config, "RECT_LEVEL_FLOOR", 45.0))
        self.level_ceil = float(getattr(config, "RECT_LEVEL_CEIL", 160.0))
        self.close_size = int(getattr(config, "RECT_CLOSE_SIZE", 3))
        self.min_area = float(getattr(config, "RECT_MIN_AREA", 1200.0))
        self.max_area_ratio = float(getattr(config, "RECT_MAX_AREA_RATIO", 0.55))
        self.min_side = float(getattr(config, "RECT_MIN_SIDE_PX", 24.0))
        self.bbox_lo = float(getattr(config, "RECT_BBOX_ASPECT_MIN", 0.45))
        self.bbox_hi = float(getattr(config, "RECT_BBOX_ASPECT_MAX", 3.4))
        self.min_fill = float(getattr(config, "RECT_MIN_FILL", 0.04))
        self.max_fill = float(getattr(config, "RECT_MAX_FILL", 0.75))
        self.max_candidates = int(getattr(config, "RECT_MAX_CANDIDATES", 2))
        self.min_aspect = float(getattr(config, "RECT_MIN_ASPECT", 1.05))
        self.max_aspect = float(getattr(config, "RECT_MAX_ASPECT", 3.2))
        self.hole_min = float(getattr(config, "RECT_HOLE_MIN_PX", 120.0))
        self.hole_max = float(getattr(config, "RECT_HOLE_MAX_PX", 400.0))
        self.band_cover = float(getattr(config, "RECT_BAND_COVER", 0.60))
        self.min_contrast = float(getattr(config, "RECT_BAND_MIN_CONTRAST", 25.0))
        self.model = tuple(getattr(config, "RECT_MODEL_MM", (261.0, 174.0)))
        self.track_expand = float(getattr(config, "RECT_TRACK_EXPAND", 1.35))
        self.track_margin = float(getattr(config, "RECT_TRACK_MARGIN", 10.0))
        self.track_max_side = float(getattr(config, "RECT_TRACK_MAX_SIDE", 240.0))
        self.size_tol = float(getattr(config, "RECT_TRACK_SIZE_TOL", 0.25))
        self.gate = float(getattr(config, "RECT_TRACK_GATE", 0.45))
        self.hold = int(getattr(config, "RECT_HOLD_FRAMES", 6))
        self.smooth = float(getattr(config, "RECT_SMOOTH", 0.55))
        self.hold_max = float(getattr(config, "RECT_HOLD_MAX_PX", 90.0))
        self.max_speed = float(getattr(config, "RECT_MAX_SPEED", 2500.0))
        self.keep_stages = bool(getattr(config, "RECT_KEEP_STAGES", False))
        self.measure_margin = 8.0        # 搜索态全分辨率精测的外扩边距
        self.last = None
        self.last_stages = {}
        self.reset()

    def reset(self):
        """清空跟踪状态。换靶/重开一题时调用；无历史帧时等价于"从头开始搜框"。"""
        self.level = None
        self.center = None          # 平滑后的中心（全分辨率）
        self.measured = None        # 最近一次实测中心
        self.velocity = (0.0, 0.0)  # px/s
        self.box = None             # 最近一次环 bbox（全分辨率）
        self.ref = (0.0, 0.0)       # 锁定时的框尺寸，跟踪时作为尺寸基准
        self.inner = None           # 最近一次内框四角（续命帧平移用）
        self.outer = None           # 最近一次外框四角
        self.band_w = 0.0           # 胶带带宽估计（外框外推用）
        self.confidence = 0.0
        self.lost = 0
        self.tracking = False
        self._last_ts = None
        self._hit_ts = None

    # ------------------------------------------------------------- 内部工具
    def _level_for(self, img):
        return auto_level(img, 2.0, 90.0, self.level_ratio,
                          self.level_floor, self.level_ceil)

    def _predict(self, ts):
        """按速度外推当前位置；限幅防止甩飞。"""
        if self.center is None:
            return None
        dt = 0.0 if self._last_ts is None else float(ts) - self._last_ts
        dt = 0.0 if dt < 0.0 else (0.25 if dt > 0.25 else dt)
        px = self.center[0] + self.velocity[0] * dt
        py = self.center[1] + self.velocity[1] * dt
        lo_x, hi_x = self.center[0] - self.hold_max, self.center[0] + self.hold_max
        lo_y, hi_y = self.center[1] - self.hold_max, self.center[1] + self.hold_max
        return (lo_x if px < lo_x else (hi_x if px > hi_x else px),
                lo_y if py < lo_y else (hi_y if py > hi_y else py))

    @staticmethod
    def _fail_reason(stats):
        """按淘汰的层给失败原因，和 [perf] 的 det_fail 口径一致。"""
        if not stats.get("components"):
            return "no_proposal"
        if not stats.get("prefiltered") and not stats.get("structural"):
            return "all_area"
        return "all_filtered"

    # ------------------------------------------------------------- 全分辨率精测
    def _measure(self, g, box, stats, timings=None):
        """候选块 → 全分辨率裁剪 → 环-洞结构 → 洞四边拟线 → 对角线交点。

        搜索降采样只负责"块在哪里"，量测在全分辨率做：保证重捕获那一刻
        的中心精度，也让带宽估计（画外框用）来自最锐利的边。
        阈值由 ring_hole 在裁剪图内 Otsu 自定，不做全局假设。
        """
        t1 = time.perf_counter()
        rh = ring_hole(g, box, self.measure_margin, self.hole_min, self.hole_max,
                       self.min_aspect, self.max_aspect, self.band_cover,
                       self.min_contrast, True, self.close_size, False)
        if rh is None:
            stats["structural"] = stats.get("structural", 0) + 1
            return None
        t2 = time.perf_counter()
        quad, hits = hole_quad(rh["hole"], rh["rect"])
        if quad is None:
            stats["structural"] = stats.get("structural", 0) + 1
            return None
        ox, oy = rh["origin"]
        quad = [(px + ox, py + oy) for px, py in quad]
        center = quad_diagonal_center(quad)
        if center is None:
            stats["structural"] = stats.get("structural", 0) + 1
            return None
        band_w = rh["band_w"]
        outer = outer_from_inner(quad, band_w)
        rb = rh["box"]
        t3 = time.perf_counter()
        if timings is not None:
            timings["comp"] = (t2 - t1) * 1000.0
            timings["quad"] = (t3 - t2) * 1000.0
        return dict(center=center, box=(rb[0] + ox, rb[1] + oy, rb[2], rb[3]),
                    inner=quad, outer=outer, band_w=band_w,
                    paper=rh.get("paper"), contrast=rh.get("contrast"),
                    level=rh.get("level"),
                    area=float(rh["hole_area"]), aspect=float(rh["aspect"]), hits=hits)

    # ------------------------------------------------------------- 全图搜框
    def _search(self, g, ts, stats, timings):
        t0 = time.perf_counter()
        s = self.scale
        small = downscale(g, s)
        level = self._level_for(small)
        self.level = level
        mask = close_mask(dark_mask(small, level), self.close_size)
        comps, _cent = components(mask)
        stats["components"] = int(comps.shape[0])
        cands = prefilter_boxes(comps, float(mask.shape[0] * mask.shape[1]),
                                self.min_area / float(s * s), self.max_area_ratio,
                                self.min_side / float(s), self.bbox_lo, self.bbox_hi,
                                self.min_fill, self.max_fill)
        stats["prefiltered"] = len(cands)
        timings["prep"] = (time.perf_counter() - t0) * 1000.0
        if self.keep_stages:
            self.last_stages = dict(binary=mask)
        if not cands:
            return None
        for _box_area, x, y, w, h in cands[:self.max_candidates]:
            det = self._measure(g, (x * s, y * s, w * s, h * s), stats, timings)
            if det is not None:
                det["reacquire"] = True
                return det
        return None

    # ------------------------------------------------------------- ROI 跟踪
    def _track(self, g, ts, timings):
        box = self.box
        pred = self._predict(ts)
        if box is None or pred is None:
            return None
        t0 = time.perf_counter()
        s = self.scale
        h, w = g.shape[:2]
        hw = box[2] * self.track_expand * 0.5 + self.track_margin
        hh = box[3] * self.track_expand * 0.5 + self.track_margin
        x0, y0 = int(pred[0] - hw), int(pred[1] - hh)
        x1, y1 = int(pred[0] + hw), int(pred[1] + hh)
        x0, y0 = (0 if x0 < 0 else x0), (0 if y0 < 0 else y0)
        x1, y1 = (w if x1 > w else x1), (h if y1 > h else y1)
        if x1 - x0 < 24 or y1 - y0 < 24:
            return None
        # 跟踪时的降采样按 ROI 尺寸自适应：ROI 不超限就全分辨率量测（与搜索态
        # 同口径，重捕获零跳变）；超限才升档，把送进轮廓层的像素量钉住。
        span = max(x1 - x0, y1 - y0)
        s = 1
        while span / s > self.track_max_side:
            s += 1
        small = downscale(g[y0:y1, x0:x1], s)
        timings["prep"] = (time.perf_counter() - t0) * 1000.0
        t1 = time.perf_counter()
        # ROI 里直接找环-洞（不再过连通域预筛）：最大的顶层轮廓就是环，
        # 判据放宽（免亮度门、边比下限放到 1.0）防运动模糊误杀。
        # 阈值由 ring_hole 在 ROI 内 Otsu 自定，与搜索态同一条链路。
        rh = ring_hole(small, (0, 0, small.shape[1], small.shape[0]), 0.0,
                       self.hole_min / s, self.hole_max / s, 1.0, 3.5,
                       self.band_cover, 0.0, False, self.close_size, self.keep_stages)
        if rh is None:
            return None
        t2 = time.perf_counter()
        quad, hits = hole_quad(rh["hole"], rh["rect"])
        if quad is None:
            return None
        # ROI 原点是原图坐标，只有块内坐标才乘缩放系数（乘错一次就差半个画面）。
        quad = [(x0 + px * s, y0 + py * s) for px, py in quad]
        center = quad_diagonal_center(quad)
        if center is None:
            return None
        rb = rh["box"]
        fbox = (x0 + rb[0] * s, y0 + rb[1] * s, rb[2] * s, rb[3] * s)
        # 尺寸门：和"锁定时的尺寸"比。逐帧比较会一步步缩到更小的连通域
        # （实测 250x180 -> 114x112 -> 50x52，最后锁到里面的圆环上）。
        ref_w = max(8.0, self.ref[0])
        ref_h = max(8.0, self.ref[1])
        if abs(fbox[2] - ref_w) > self.size_tol * ref_w or \
                abs(fbox[3] - ref_h) > self.size_tol * ref_h:
            return None
        # 跟踪门：实测中心离预测太远说明锁到了别的东西（或上帧预测跑飞），当帧作废。
        gate = self.gate * max(box[2], box[3])
        if abs(center[0] - pred[0]) > gate or abs(center[1] - pred[1]) > gate:
            return None
        band_w = rh["band_w"] * float(s)
        outer = outer_from_inner(quad, band_w)
        t3 = time.perf_counter()
        timings["comp"] = (t2 - t1) * 1000.0
        timings["quad"] = (t3 - t2) * 1000.0
        if self.keep_stages and rh.get("mask") is not None:
            self.last_stages = dict(binary=rh["mask"])
        return dict(center=center, box=fbox, inner=quad, outer=outer, band_w=band_w,
                    paper=None, contrast=None, level=rh.get("level"),
                    area=float(rh["hole_area"]) * s * s,
                    aspect=float(rh["aspect"]), hits=hits,
                    proposals=int(rh.get("contours", 0)))

    # ------------------------------------------------------------- 状态推进
    def _on_hit(self, det, ts):
        meas = det["center"]
        # 速度只在帧间隔足够长时更新：连拍两帧 dt≈0.2ms，除出来是天文数字，
        # 外推会把 ROI 甩到画面另一头（实测会锁到墙上的别的东西）。
        if self.measured is not None and self._hit_ts is not None:
            dt = ts - self._hit_ts
            if 0.004 <= dt <= 0.25:
                a = 0.55
                vx = self.velocity[0] * (1.0 - a) + (meas[0] - self.measured[0]) / dt * a
                vy = self.velocity[1] * (1.0 - a) + (meas[1] - self.measured[1]) / dt * a
                speed = (vx * vx + vy * vy) ** 0.5
                if speed > self.max_speed:
                    vx *= self.max_speed / speed
                    vy *= self.max_speed / speed
                self.velocity = (vx, vy)
        if self.center is None or self.lost > 0:
            self.center = meas                      # 重新捕获：直接对齐，不做低通
        else:
            k = self.smooth
            self.center = (self.center[0] + k * (meas[0] - self.center[0]),
                           self.center[1] + k * (meas[1] - self.center[1]))
        self.measured = meas
        self.box = det["box"]
        self.inner = det.get("inner") or self.inner
        self.outer = det.get("outer") or self.outer
        self.band_w = float(det.get("band_w") or self.band_w or 0.0)
        if det.get("contrast") is not None:
            self.confidence = min(1.0, float(det["contrast"]) / 60.0)
        if det.get("reacquire"):
            self.ref = (float(det["box"][2]), float(det["box"][3]))
        else:
            # 参考尺寸只允许慢速跟随（每帧 <=2%），这样"车在靠近"能跟上，
            # "跳到别的小块上"跟不上。
            self.ref = (self.ref[0] + max(-0.02 * self.ref[0], min(0.02 * self.ref[0],
                                                                 det["box"][2] - self.ref[0])),
                        self.ref[1] + max(-0.02 * self.ref[1], min(0.02 * self.ref[1],
                                                                 det["box"][3] - self.ref[1])))
        self.lost = 0
        self.tracking = True
        self._hit_ts = ts
        self._last_ts = ts
        self.level = det.get("level") or self.level
        return self._make_result(det, hold=False)

    def _on_missing(self, ts, stats):
        if self.center is None or self.box is None or self.lost >= self.hold:
            self.tracking = False
            return fail_result(STAGE_NONE, "plane", self._fail_reason(stats))
        self.lost += 1
        self.velocity = (self.velocity[0] * 0.7, self.velocity[1] * 0.7)
        pred = self._predict(ts)
        self.center = pred
        self._last_ts = ts
        # 续命帧的框沿预测平移（纯 Python）：图传上不至于钉在旧位置显假锁。
        dx = pred[0] - (self.measured[0] if self.measured else pred[0])
        dy = pred[1] - (self.measured[1] if self.measured else pred[1])
        inner = [(px + dx, py + dy) for px, py in self.inner] if self.inner else None
        outer = [(px + dx, py + dy) for px, py in self.outer] if self.outer else None
        det = dict(center=pred, box=self.box, inner=inner, outer=outer,
                   area=0.0, aspect=0.0, hits=0, contrast=None, paper=None)
        return self._make_result(det, hold=True)

    def _make_result(self, det, hold):
        box = det["box"]
        cx, cy = self.center
        conf = self.confidence
        if hold:
            conf = max(0.2, conf * (1.0 - self.lost / float(max(1, self.hold))))
        return dict(valid=True, stage=STAGE_PLANE,
                    u=float(cx), v=float(cy),
                    raw_u=float(det["center"][0]), raw_v=float(det["center"][1]),
                    quad=det.get("outer"), inner_quad=det.get("inner"),
                    detected_quad=det.get("outer"), outer_inferred=False,
                    center_ref="inner", circle=None,
                    area=float(det.get("area") or 0.0),
                    aspect=float(det.get("aspect") or 0.0),
                    bbox=(int(box[0]), int(box[1]), int(box[2]), int(box[3])),
                    partial=bool(hold), hold=bool(hold), lost=int(self.lost),
                    refined_edges=int(det.get("hits") or 0), weak_edges=0, clipped_edges=0,
                    rect_to_image=None, circle_points=None,
                    band=dict(paper=det.get("paper"), contrast=det.get("contrast")),
                    width_mm=float(self.model[0]), height_mm=float(self.model[1]),
                    confidence=float(conf))

    def _detect(self, g, ts):
        timings = {}
        stats = dict(components=0, prefiltered=0, structural=0)
        det = None
        if self.tracking:
            det = self._track(g, ts, timings)
        if det is None:
            det = self._search(g, ts, stats, timings)
        if det is not None and det.get("proposals") is not None:
            stats["components"] = det["proposals"]
        result = self._on_hit(det, ts) if det is not None else self._on_missing(ts, stats)
        return result, stats, timings

    # ------------------------------------------------------------- 对外入口
    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        ts = started if timestamp is None else float(timestamp)
        if not np.isfinite(ts):
            raise ValueError("non-finite timestamp")
        g = to_gray_frame(gray)
        if g.size == 0:
            result, stats, timings = fail_result("empty_frame", "none", "empty"), \
                dict(components=0, prefiltered=0, structural=0), {}
        else:
            result, stats, timings = self._detect(g, ts)
        result.update(masks=1, proposals=stats.get("components", 0), search_count=1,
                      reject=dict(stats), threshold=self.level, build=BUILD_ID)
        result["t_ms"] = timings
        self.last = finish_result(result, ts,
                                  (int(g.shape[1]), int(g.shape[0])) if g.size else (0, 0),
                                  started)
        return self.last


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
    - 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅，``ACC_TRACK`` 曲线加减速；
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

    def _rpm(self, error=0):
        fine, coarse = self.cfg.RPM_FINE, self.cfg.RPM_COARSE
        return max(1, round(fine+self._mix(error)*(coarse-fine)))

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
        rpm = self._rpm(error)
        self._direction[axis] = error
        self._busy_until[axis] = now+estimate_move_seconds(pulses, rpm, getattr(cfg, "ACC_TRACK", 80), getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))+getattr(cfg, "AIM_MOTION_MARGIN_S", .03)
        return ccw, pulses, rpm



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
        return self.aim.state if self.aim else 'vision'

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
    while not app.need_exit():
        started = clock.perf_counter()
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
        controller = state['controller'] = VisionControl(cfg, state['link'], state['aim'], clock=clock)
        worker = state['worker'] = LatestFrameWorker(controller.process, controller.idle,
                                                    cfg.VISION_ASYNC, clock=clock)
        print('[rect-center] %s scale=%d level=%.2f close=%d hold=%d gate=%.2f '
              'display_width=%d display_fps=%d' %
              (BUILD_ID, cfg.RECT_DOWNSCALE, cfg.RECT_LEVEL_RATIO, cfg.RECT_CLOSE_SIZE,
               cfg.RECT_HOLD_FRAMES, cfg.RECT_TRACK_GATE,
               preview.render_width, cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC, preview.render_width, preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC, cfg.RESULT_MAX_AGE_S*1000))
        _capture_loop(cfg, clock, app, image, state, BUILD_ID)
    finally:
        _release(state)
