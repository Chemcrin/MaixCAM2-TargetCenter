"""靶心识别检测器：**Canny 轮廓定位 → 四边暗带验证 → 亚像素精修 → 单应解中心**。

================================================================================
几何原理（为什么靶心能"算"出来）
================================================================================
靶纸四周贴 1.8cm 黑胶带形成一个矩形环。相机斜看时该矩形在图像里是透视投影，
但**两条对角线的交点**在任何透视变换下都映射到对应四边形的对角线交点 —— 这是
**射影不变量**。于是：

    检测四边形四角 → 解 mm↔px 单应 → 把物理矩形中心 (w/2, h/2) 投影回图像 = 靶心 (u,v)

换一种说法就是题目要求的四步，二者**数学上完全等价**：

    四边形 → 透视变换成正视图 → 取正视图中心 → 逆透视反投影回原图

本模块把后一种说法也**真的做了一遍**（`warp_front_view` + `_front_view_center`），
得到一个与解析解**互相独立**的中心估计，用于交叉校验（两者不一致时置
`front_disagree=True`，提示几何/标定异常）。所以"正视图"不是装饰，是第二只眼睛。

================================================================================
单帧处理链
================================================================================
    ① 候选提议  `ContourLocator`  —— Canny → findContours → 四边形筛选（**低分辨率**）
    ② 四边验证  `_verify`         —— 从四边向里采"胶带暗带 + 内侧亮面"，覆盖率为真伪判据
    ③ 亚像素精修 `_refine`        —— 当前帧四条灰度窄带 → 有向跳变 → fitLine → 四角交点
    ④ 中心解算  `_center`         —— 单应把物理矩形中心投回原图（+ 正视图剖面交叉校验）
    ⑤ 时序滤波  `ConstantVelocityKalman2D` —— 抑制抖动、给下一帧提供速度预测

四级进入方式（越靠前越省算力；`process` 按上一帧是否锁定依次降级）：
    track       已锁定 → 只沿四边窄带精修（不跑 Canny/轮廓/掩膜，稳态 ~1ms）
    roi_search  刚失锁 → 在上一帧外框外扩 ROI_PADDING 的窗口内定位一次
    full_search 彻底丢失 → 全图低分辨率定位（1 张掩膜，必要时第 2 张）

掩膜纪律（v0.8 重构的核心）
--------------------------------------------------------------------------------
旧实现每帧要建 `Otsu + 8 档 DARK_THRESHOLDS` 共 9~10 张掩膜，每张都做
`threshold + morphologyEx + findContours`，再对每个候选做全分辨率四边采样 ——
板端实测单帧 ~312ms、detect_fps=3.2、结果 age>200ms 全被判 stale，闭环被锁死
（见 `板端识别问题整理_2026-10-13.md`）。
现在改为 **Canny 一级（1 张掩膜）+ Otsu 兜底一级（仅当第一级没找到四边齐备的候选）**，
且阈值化/形态学/轮廓树都在 `SEARCH_WIDTH` 的低分辨率图上完成，只有"验证/精修/解中心"
才回到原始分辨率 —— 这正是"减少掩膜数量、控制该项运行时长"的落点。
"""
import time

import cv2
import numpy as np

from .contour_locator import LEVELS, ContourLocator
from .kalman_filter import ConstantVelocityKalman2D
from .rect_geometry import (front_view_size, homography, homography_8x8,
                            inverse_map_points, line_through, order_quad, polygon_area,
                            project_points, quad_from_contour, quad_is_valid,
                            rectangle_corners, sample_batch, side_lengths, standard_circle,
                            warp_front_view)

# ---- 兼容别名（旧名字在 tools/diagnose_detect.py 等外部工具里被直接 import）--------
_polygon_area = polygon_area
_order_quad = order_quad
_side_lengths = side_lengths
_homography = homography
_rectangle = rectangle_corners
_sample = sample_batch
_line_through = line_through

__all__ = ["RectangleCenterDetector", "draw_result", "homography_8x8", "project_points",
           "inverse_map_points", "standard_circle", "quad_from_contour", "quad_is_valid",
           "_homography", "_rectangle", "_sample", "_polygon_area", "_order_quad",
           "_side_lengths", "_line_through"]


def _midline_mm(profile, extent_mm):
    """一维剖面里"两侧暗带（胶带）内沿之间的中线"，单位 mm；不可信时返回 None。

    先按分位数取暗/亮两档再取中值作阈值（避免个别反光或污点把 min/max 拉飞），
    然后**只在前/后 1/3 内**找暗带内沿（胶带恒在靶纸边缘，中段出现暗点属于印刷/阴影），
    最后对穿越点做线性插值得到亚像素位置。返回 None 表示"这个剖面没有可用的胶带暗带"。
    """
    prof = np.asarray(profile, np.float64).ravel()
    if prof.size < 12:
        return None
    dark, bright = float(np.percentile(prof, 8)), float(np.percentile(prof, 92))
    if bright - dark < 8.0:                      # 对比度太低：没有可见的暗带
        return None
    threshold = 0.5 * (dark + bright)
    scale = float(extent_mm) / prof.size          # mm/px；正视图一个像素对应多少 mm

    def inner_edge(forward):
        third = max(2, prof.size // 3)
        segment = prof[:third] if forward else prof[-third:]
        below = np.nonzero(segment <= threshold)[0]
        if below.size == 0:
            return None
        index = int(below[-1]) if forward else int(below[0]) + (prof.size - third)
        neighbour = index + 1 if forward else index - 1
        if not 0 <= neighbour < prof.size:
            return None
        step = prof[neighbour] - prof[index]
        if abs(step) < 1e-9:
            return float(index)
        # 从低于阈值到高于阈值的线性插值穿越点（亚像素）
        return float(index) + (threshold - prof[index]) / step

    left, right = inner_edge(True), inner_edge(False)
    if left is None or right is None or right <= left:
        return None
    return ((left + right) * 0.5 + 0.5) * scale    # +0.5：像素 i 的中心在 i+0.5 处


class RectangleCenterDetector:
    """有状态检测器：跨帧记住锁定框(_track)、中心滤波器与运动历史。

    状态字段：
        _track       上一帧验证通过的外框（dict 候选）；None = 未锁定
        _prev_quad   再上一帧的候选四角（运动预测用，仅同 kind 连续帧才建立）
        _lost        连续未检出帧数（>0 = 刚失锁，禁用预测，避免用旧速度外推）
        _kalman      中心卡尔曼滤波器（CV 模型，抑抖 + 提供速度预测）
        _smooth      上一帧输出中心（KALMAN_ENABLE=False 时退化为 EMA 口径）
        _locked_th   不再使用（保留字段以免外部代码 AttributeError）；见 README 迁移说明
        _roi_retry   失锁后是否还允许一次"ROI 搜索"（用一次即置 False，防反复局部搜）
    每帧重置的临时量：_search_count / _mask_count / _proposals。
    """

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._prev_quad = None                                 # 上上帧的候选四角（运动预测用）
        self._track_timestamp = self._prev_timestamp = None
        self._frame_shape = None
        self._lost = 0                                         # 连续未检出帧数（>0 时不预测）
        self._locked_th = None
        self._frame = 0
        self._locator = ContourLocator(config)
        # 四边上均匀分布的参数 t ∈ (0.12, 0.88)：避开四角（角点附近因透视最易模糊）。
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._circle_cache = {}                                # 正视面圆周采样点缓存（按尺存）
        self._search_count = 0
        self._mask_count = 0
        self._proposals = 0
        self._roi_retry = True
        self._kalman = self._make_kalman(config)

    @staticmethod
    def _make_kalman(config):
        if not bool(getattr(config, "KALMAN_ENABLE", True)):
            return None
        return ConstantVelocityKalman2D(
            process_px=float(getattr(config, "KALMAN_PROCESS_PX", 260.0)),
            measure_px=float(getattr(config, "KALMAN_MEASURE_PX", 0.9)),
            reset_px=float(getattr(config, "KALMAN_RESET_PX",
                                   getattr(config, "FILTER_RESET_PX", 25.0))),
            max_predict_s=float(getattr(config, "KALMAN_MAX_PREDICT_S", 0.25)),
            velocity_limit_px_s=float(getattr(config, "KALMAN_VELOCITY_LIMIT_PX_S", 6000.0)))

    def reset(self):
        """清空图像坐标与运动历史；换分辨率、断流后不能复用旧框。"""
        self.last = self._smooth = self._track = self._prev_quad = None
        self._track_timestamp = self._prev_timestamp = None
        self._frame_shape = None
        self._lost = self._frame = 0
        self._locked_th = None
        self._roi_retry = True
        if self._kalman is not None:
            self._kalman.reset()
        return self

    # ------------------------------------------------------------------ 小工具
    def _continuous_time(self, timestamp):
        """两次采图时刻是否连续（可用于运动预测/滤波外推）。None 一律视为不连续。"""
        if timestamp is None or self._track_timestamp is None:
            return timestamp is None and self._track_timestamp is None
        gap = timestamp - self._track_timestamp
        return 0 < gap <= float(getattr(self.cfg, "PREDICT_MAX_GAP_S", 0.25))

    def _valid_quad(self, q, shape, margin=1):
        """四边形体检（几何判据集中在 rect_geometry.quad_is_valid，便于单测）。"""
        return quad_is_valid(q, shape, margin=margin,
                             min_edge=float(getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.0)))

    def _layout(self, q, kind):
        """由四边形朝向推断物理长宽比，并给出该 kind 对应的物理矩形四角。

        horizontal：两条"横向边长之和" > 两条"纵向边长之和" → 长边水平 → w=长边(mm)。
        kind=="inner" 时物理矩形向内缩进一个胶带宽度 BLACK_FRAME_MM（对应内框角点）。
        """
        lengths = side_lengths(q)
        horizontal = lengths[0] + lengths[2] > lengths[1] + lengths[3]
        w, h = ((self.cfg.TARGET_LONG_MM, self.cfg.TARGET_SHORT_MM) if horizontal
                else (self.cfg.TARGET_SHORT_MM, self.cfg.TARGET_LONG_MM))
        inset = self.cfg.BLACK_FRAME_MM if kind == "inner" else 0.0
        return w, h, rectangle_corners(w, h, inset)

    # ------------------------------------------------------------------ ② 四边验证
    def _verify(self, gray, q, kind):
        """在正视面选取四边稀疏采样点并投影，不生成整幅 warp 图。

        每条边检查胶带厚度方向的暗带覆盖率与内侧亮区对比，允许局部反光；
        背景可以是黑色，不再强制外侧为白色。普通纸边及实心黑块会失败。
        """
        if not self._valid_quad(q, gray.shape):
            return None
        w, h, physical_quad = self._layout(q, kind)
        tape = float(self.cfg.BLACK_FRAME_MM)
        if tape <= 0 or 2 * tape >= min(w, h):
            raise ValueError("BLACK_FRAME_MM must be between 0 and half the short side")
        try:
            # 求 mm → px 单应：把物理矩形映射到图像候选四边形 q。
            rect_to_image = homography(physical_quad, q)
            t = self._t
            # 沿每条边的法向采 5 个偏移：0.20/0.40/0.60×胶带宽 = 黑带内部三点（应为暗），
            # 1.22/1.45×胶带宽 = 越过内沿的亮面两点（应为亮）。暗亮对比即边证据。
            offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
            p = np.empty((4, len(t), 5, 2), np.float64)
            # 四条边分别参数化：(边0 上/边1 右/边2 下/边3 左)，每条边沿 t 铺采样点。
            p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
            p[0, :, :, 1] = offsets
            p[1, :, :, 0] = w - offsets
            p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
            p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
            p[2, :, :, 1] = h - offsets
            p[3, :, :, 0] = offsets
            p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
            mapped = project_points(rect_to_image, p.reshape(-1, 2)).reshape(p.shape)
        except (ValueError, np.linalg.LinAlgError):
            return None
        gh, gw = gray.shape
        # 任何采样点落到图像外 → 该候选不可信（中心也无法可靠外推）。
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = sample_batch(gray, mapped).astype(np.float32)
        # 每条边、每个 t：暗带取三点最大值（黑带里最亮的那个仍应暗），亮面取两点中位数。
        dark = np.max(samples[:, :, :3], axis=2)
        bright = np.median(samples[:, :, 3:], axis=2)
        delta = bright - dark
        # "该证据成立" = 对比度达标 且 暗带确实比亮面暗（比值约束，排除两侧都亮的纸边）。
        coverage = ((delta >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        edge_contrast = np.median(delta, axis=1)
        # 至少三边的大部分采样成立，且对比度中位达标。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                np.median(edge_contrast) < self.cfg.MIN_EDGE_CONTRAST):
            return None
        # ★ 允许**一条边**大面积缺失：实机上"缺一个角的黑框"是常态 —— 胶带在角落断掉、
        #    强反光把某一段胶带洗白、或被物体遮挡。旧规则一律要求四边覆盖率 ≥ WEAK_EDGE_COVERAGE，
        #    实测（tools/diagnose_detect.py）会把唯一正确的候选拒掉，表现即"基本识别不到"。
        #    放宽的边界很清楚：只有当**另外三条边都达到 STRONG_EDGE_COVERAGE** 时才豁免，
        #    所以"凭一条亮带猜框"依然被拒。
        if (coverage.min() < self.cfg.WEAK_EDGE_COVERAGE and
                np.sort(coverage)[::-1][:3].min() < self.cfg.STRONG_EDGE_COVERAGE):
            return None
        # ★ 逐边剖面的中位数**一次向量化**算完：这里原先对每边单独调 4 次 np.median，
        #   一轮验证要 15 次 np.median（单次调用开销 ~30µs）—— 剖析显示它占了 _verify
        #   六成时间，而它只是**诊断字段**。向量化后同样信息、调用次数降到 3 次。
        rows = [dict(dark=float(d), inside=float(b), contrast=float(c), coverage=float(v))
                for d, b, c, v in zip(np.median(dark, axis=1), np.median(bright, axis=1),
                                      edge_contrast, coverage)]
        outer = project_points(rect_to_image, rectangle_corners(w, h)).astype(np.float32)
        if not self._valid_quad(outer, gray.shape):
            return None
        coverage = np.asarray(coverage, np.float64)
        contrast = float(np.median(edge_contrast))
        # 返回候选：外框(图像)、内框(胶带内沿)、mm→px 单应、四边剖面、质量分。
        # score  = 中位对比度 + 60×平均覆盖率（历史口径，外部工具在用）。
        # rank   = score + 40×**最小**覆盖率 —— 多候选择优用的排序键：把"缺一条边"的候选
        #          压到"四边齐备"的候选之后。实机教训（板端真机帧）：一个顶边落在纸面上的
        #          假候选靠"一条边缺失"豁免通过验证，score 与真候选只差 3 分，而 rank 差 40 分
        #          ⇒ 用 rank 择优可以稳定选出真框，用 score 则可能选错。
        return dict(quad=q, outer_quad=outer,
                    inner_quad=project_points(rect_to_image, rectangle_corners(w, h, tape)).astype(np.float32),
                    kind=kind, rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=contrast,
                    min_contrast=float(edge_contrast.min()), coverage=coverage,
                    score=float(contrast + 60 * coverage.mean()),
                    rank=float(contrast + 60 * coverage.mean() + 40 * coverage.min()))

    def _four_edges_complete(self, candidate):
        """四边齐备（没有靠"缺边豁免"通过）→ 可以直接锁定，不必再跑兜底级别。"""
        return (float(np.asarray(candidate["coverage"]).min()) >= self.cfg.MIN_EDGE_COVERAGE and
                float(candidate["contrast"]) >= self.cfg.MIN_EDGE_CONTRAST)

    # ------------------------------------------------------------------ ③ 亚像素精修
    def _refine(self, gray, q, kind, radius):
        """当前帧四条灰度窄带 → 有方向的边缘跳变 → fitLine → 四角交点。

        与 _verify 的区别：_verify 只判"像不像"，_refine 才真正**重新定位**每条边。
        步骤：沿四边已有点 (base) 的法向 ±radius 采样灰度 → 求有向梯度 → 每列取
        "最近预测位置"的跳变（offset 越小越优先，防止误锁到内侧印刷圆或另一侧胶带边）
        → 用 fitLine 拟合四条直线 → 相邻直线两两求交 = 亚像素四角。
        精修结果偏离原 q 超过 radius*2.8 或四角体检不过 → 判失败（返回 None）。

        ★ 容错（实机必需）：某条边胶带缺失/被反光洗白时，该边**沿用输入四边形的边**，
          只要另有至少三条边真拟合过就继续。旧实现任一条边证据不足就整框返回 None，
          实机"缺一个角"时会让窄带跟踪每帧都失败。
        """
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]   # 各边外/内法向
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        # 每条边、每个 t、每个法向偏移 → 采样点网格 (4, len(t), 2r+1, 2)。
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        inside = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                  (points[..., 1] >= 1) & (points[..., 1] < gh - 1)).all(axis=2)
        values = sample_batch(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]                    # 中心差分近似梯度
        if kind == "outer":
            gradient *= -1                                                 # 外框要求暗→亮方向相反
        # 近预测边缘优先，避免捕获靶纸内部印刷圆或另一侧的胶带边缘。
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.45              # 偏离预测位置的惩罚
        indices = np.argmax(gradient - penalty, axis=2)                    # 每股取最强跳变
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        selected = offsets[indices + 1]
        lines = []
        refitted = 0
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)   # 有效采样
            shifts = selected[i, good]
            # 该边证据不足（胶带缺失/反光洗白）→ **沿用输入四边形的这条边**（=预测位置），
            # 而不是放弃整框精修：一条边没证据是实机常态，放弃会让帧率与锁定一起塌。
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                lines.append(line_through(q[i], q[(i + 1) % 4]))
                continue
            med = float(np.median(shifts))
            # 去掉偏离中位数过大的离群跳变（反光/杂物），再确认仍够点。
            good &= np.abs(selected[i] - med) <= max(2.5, float(np.median(np.abs(shifts - med))) * 3)
            if good.sum() < 5:
                lines.append(line_through(q[i], q[(i + 1) % 4]))
                continue
            samples = base[i, good] + selected[i, good, None] * normal[i]
            # 用鲁棒 Huber 回归拟合该边直线；fitLine 返回 (vx, vy, x0, y0)。
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            # 直线 ax+by+c=0。
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
            refitted += 1
        if refitted < 3:
            return None                                                    # 至少三条边真拟合过
        lines = np.asarray(lines)
        # 相邻两条直线（roll 1 使其错位一位）叉积求交点：交点齐次坐标 = 两直线系数叉积。
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None                                                    # 有平行边 → 退化
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None                                                    # 跑太远或形状不合法
        return refined

    # ------------------------------------------------------------------ 候选受理
    def _accept(self, gray, q, kind):
        """**轻量受理**：只做四边验证（失败时试宽半径救援），**不做精修**。

        关键在于把"筛选"与"精修"分开：一帧里可能有十几个候选，但最终只会用到一两个。
        旧实现给每个候选都跑一遍"窄带精修 + 再验证"，单帧在板端帧捕获时做了 14 次精修
        （剖析：精修占 41% 时间）—— 绝大多数白做。现在验证只负责"像不像"，
        只有**最终选中的那个**才进 `_finalize` 做亚像素精修。
        """
        verified = self._verify(gray, q, kind)
        if verified is not None:
            return verified
        return self._rescue(gray, q, kind)

    def _finalize(self, gray, candidate, iterations):
        """对最终选中的候选做亚像素精修，**迭代到不动点**，并重新验证。

        为什么迭代：搜索帧的候选离真边可能有几个像素，只精修一轮会停在半路，于是
        "首帧"与"第二帧跟踪"的中心差 ~1px（实机表现：刚锁上时中心轻轻跳一下）。
        迭代到不动点后，两条路径收敛到同一位置（实测 1.14px → 0.11px）。
        """
        kind = candidate["kind"]
        base = candidate["quad"]
        best = candidate
        for _ in range(max(1, int(iterations))):
            refined = self._refine(gray, base, kind, self.cfg.REFINE_RADIUS)
            if refined is None:
                break
            precise = self._verify(gray, refined, kind)
            if precise is None:
                break
            moved = float(np.linalg.norm(np.asarray(refined, np.float64) -
                                        np.asarray(base, np.float64), axis=1).max())
            best, base = precise, refined
            if moved < 0.2:                              # 已到不动点
                break
        return best

    def _rescue(self, gray, q, kind):
        """★ 宽半径兜底精修：候选可能整体偏离真边，`_verify` 的固定小偏移会采到纸面。

        实机实测（2026-10-13）：掩膜给出的候选顶边比胶带外沿低约 14px，而 `_verify` 只从边
        向里采 0.2~0.6×BLACK_FRAME_MM（≈2~6px）⇒ 直接采到纸面、把纸当成"暗带"而误拒；
        窄半径（REFINE_RADIUS / TRACK_RADIUS）都纠不动这种十几 px 的偏差，
        于是画面里黑框清清楚楚却永远 no_black_frame。
        救援必须**确证落在真胶带上**才采纳：① 中位对比度 ≥ RECOVER_MIN_CONTRAST；
        ② **四边暗带灰度的最大值**必须够黑（≤ RECOVER_MAX_DARK）—— 真胶带四边都是黑的，
        而"纸边/褶皱"这类假边界总有一两条边只是中灰。
        """
        wide = float(getattr(self.cfg, "REFINE_RADIUS_WIDE", 0.0))
        if wide <= self.cfg.REFINE_RADIUS:
            return None
        refit = self._refine(gray, q, kind, wide)
        if refit is None:
            return None
        recovered = self._verify(gray, refit, kind)
        if recovered is None:
            return None
        darkest = max(row["dark"] for row in recovered["edge_profile"])
        if (recovered["contrast"] >= float(getattr(self.cfg, "RECOVER_MIN_CONTRAST", 80.0)) and
                darkest <= float(getattr(self.cfg, "RECOVER_MAX_DARK", 100.0))):
            return recovered
        return None

    # ------------------------------------------------------------------ ① 搜索
    def _search(self, gray, roi=None):
        """低分辨率提议 → 全分辨率验证；**单帧内跑完全部级别**（重捕获恒为 1 帧）。

        与旧实现的三点差别：
          · 提议来自 `ContourLocator`（Canny 边缘，必要时补一张 Otsu 掩膜），
            不再是"9~10 档阈值逐档 findContours"；
          · 同一级别内**验证完所有候选再择优**（`rank`，见 `_verify`），
            而不是"第一个通过就返回" —— 假候选靠"缺一条边豁免"抢先通过是实机出现过的
            真实误锁（板端真机帧中心偏 17px，而真候选 rank 高 40 分）；
          · **四边齐备**的候选立即锁定早退（普通场景只花 1 次验证 + 1 次精修），
            靠"缺边豁免/宽半径救援"通过的候选才继续扫完整个级别、比 rank 后再精修。
        `SEARCH_EXHAUSTIVE=True`（默认，用户明确要求的稳定性口径）时候选数不截断；
        False 时按 `SEARCH_MAX_PROPOSALS` 截断 —— 预算在 ROI 与全图搜索之间**共享**。
        """
        self._search_count += 1
        cap = None if bool(getattr(self.cfg, "SEARCH_EXHAUSTIVE", True)) \
            else int(self.cfg.SEARCH_MAX_PROPOSALS)
        iterations = max(1, int(getattr(self.cfg, "REFINE_ITERATIONS", 2)))
        best, best_rank = None, None
        for level in LEVELS:
            proposals = self._locator.locate(gray, roi, level)
            stats = self._locator.last_stats
            self._mask_count += int(stats["masks"])
            for q, kind in proposals:
                if cap is not None and self._proposals >= cap:
                    return self._finalize(gray, best, iterations) if best else None
                self._proposals += 1
                candidate = self._accept(gray, q, kind)
                if candidate is None:
                    continue
                if self._four_edges_complete(candidate):
                    return self._finalize(gray, candidate, 1)     # 四边齐备：立即锁定
                if best is None or candidate["rank"] > best_rank:
                    best, best_rank = candidate, candidate["rank"]
        return self._finalize(gray, best, iterations) if best else None

    # ------------------------------------------------------------------ ④ 中心解算
    def front_view(self, gray, result, short_px=None):
        """把已检出的框**透视变换成正视图**（正视 = 矩形，无透视畸变）。

        返回 (front_view, page_to_image, mm_to_front)；任何一步退化都返回 (None, None, None)。
        正视图一律按**整张物理靶纸**(w×h) 建立（用 `outer_quad` 求单应，与 kind 无关），
        因此无论锁定的是内框还是外框，胶带暗带恒落在正视图四周。
        用途：现场可视化（`tools/diagnose_detect.py --front out.png`）、以及
        `front_view_center` 的独立中心校验。
        """
        if not result or not result.get("valid"):
            return None, None, None
        short_px = int(getattr(self.cfg, "FRONT_VIEW_SHORT_PX", 0) if short_px is None else short_px)
        if short_px <= 0:
            return None, None, None
        w, h = result["width_mm"], result["height_mm"]
        try:
            page_to_image = homography(rectangle_corners(w, h), result["outer_quad"])
            view, mm_to_front = warp_front_view(gray, page_to_image, w, h, short_px)
        except (ValueError, np.linalg.LinAlgError):
            return None, None, None
        if view is None or view.size == 0:
            return None, None, None
        return view, page_to_image, mm_to_front

    def front_view_center(self, gray, result, short_px=None):
        """在**正视图里重新取中心**，再逆透视反投影回原图；失败返回 None。

        与 `process` 里的解析解（"单应把物理中心投回去"）是**两条独立的算法**：
            · 解析解建立在**四角几何**上 —— 四角一旦拟合偏了，中心就跟着偏；
            · 本方法建立在**灰度对称性**上 —— 对单个角点误差不敏感，但对光照不均敏感。
        两者接近说明几何自洽（`tools/test_real_scene.py` 用它锁住 `warp_front_view` 的
        矩阵方向：方向反了会得到"整片背景"，剖面法立刻失效）。
        ⚠ 本方法精度受正视图分辨率限制（128px 短边时典型偏差 3~4px，256px 时约 1px），
          因此**不参与**逐帧中心输出 —— 热路径用解析解，本方法供诊断/校验。
        """
        view, page_to_image, _ = self.front_view(gray, result, short_px)
        if view is None:
            return None
        rows, cols = view.shape
        # 只看中间一半行/列：避开垂直方向的胶带暗带，让剖面里只剩"两侧暗带 + 中间亮区"。
        band = slice(rows // 4, max(rows // 4 + 1, 3 * rows // 4))
        half = slice(cols // 4, max(cols // 4 + 1, 3 * cols // 4))
        cx = _midline_mm(view[band, :].mean(axis=0), result["width_mm"])
        cy = _midline_mm(view[:, half].mean(axis=1), result["height_mm"])
        if cx is None or cy is None:
            return None
        try:
            point = project_points(page_to_image, [[cx, cy]])[0]
        except (ValueError, np.linalg.LinAlgError):
            return None
        return np.asarray(point, np.float64)

    # ------------------------------------------------------------------ 运动预测
    def _motion_delta(self, timestamp=None):
        """上两帧四角的**逐角位移**（图像坐标）；无历史或刚失锁则 None。

        逐角位移天然同时包含平移、缩放与旋转，直接加在"上一帧四角"上即为匀速外推。
        只在**连续两帧都成功**（_lost == 0）时可用：失锁后速度已不可信，宁可退回原地精修。
        位移限幅 PREDICT_MAX_PX 防止噪点/误检把预测甩到画面外。
        """
        if not getattr(self.cfg, "ENABLE_MOTION_PREDICT", True):
            return None
        if self._prev_quad is None or self._track is None or self._lost:
            return None
        delta = np.asarray(self._track["quad"], np.float32) - np.asarray(self._prev_quad, np.float32)
        if timestamp is not None:
            if self._prev_timestamp is None or not self._continuous_time(timestamp):
                return None
            interval = self._track_timestamp - self._prev_timestamp
            if interval <= 0:
                return None
            # 最新帧线程会丢弃旧帧；按采图间隔外推，不能把每次回调当作等间隔视频帧。
            delta = delta * ((timestamp - self._track_timestamp) / interval)
        step = float(np.linalg.norm(delta, axis=1).max())
        cap = float(getattr(self.cfg, "PREDICT_MAX_PX", 60.0))
        if cap > 0 and step > cap:
            delta = delta * (cap / step)
        return delta

    # ------------------------------------------------------------------ 主入口
    def process(self, gray, timestamp=None):
        """单帧入口：按"track → roi_search → full_search"三级降级找框，输出中心与结果 dict。"""
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        if min(g.shape) < 2:
            self.reset()
            self.last = dict(valid=False, stage="empty_frame", search_count=0,
                             masks=0, proposals=0, detector_ms=0.0)
            return self.last
        timestamp = None if timestamp is None else float(timestamp)
        if timestamp is not None and not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self._frame_shape != g.shape:
            self.reset()
            self._frame_shape = g.shape
        if self._track is not None and not self._continuous_time(timestamp):
            self._prev_quad = self._prev_timestamp = self._smooth = None
            if self._kalman is not None:
                self._kalman.reset()
        self._frame += 1
        self._search_count = self._mask_count = self._proposals = 0
        candidate, stage = None, "search"
        # ---- ① track：上一帧已锁定 → 先用**运动预测位置**、再用上一帧位置做窄带精修 ----
        # 预测位置让窄带跟上快速移动（不必靠加大 TRACK_RADIUS 牺牲慢速稳定性）；
        # 预测失准（突然停止/变向）时自动回落到上一帧位置，两级都试。
        if self._track is not None:
            kind = self._track["kind"]
            guesses = []
            delta = self._motion_delta(timestamp)
            if delta is not None:
                guesses.append(self._track["quad"] + delta)
            guesses.append(self._track["quad"])
            for guess in guesses:
                q = self._refine(g, guess, kind, self.cfg.TRACK_RADIUS)
                if q is None:
                    continue
                verified = self._verify(g, q, kind)
                if verified is not None:
                    candidate, stage = verified, "track"
                    break
        # ---- ② roi_search：刚失锁 → 在上一帧外框（含预测位移）外扩的窗口内搜一次 ----
        if candidate is None and self._track is not None and self._roi_retry:
            q = np.asarray(self._track["outer_quad"], np.float32)
            delta = self._motion_delta(timestamp)
            if delta is not None:
                q = q + delta                                  # 窗口中心跟上预测位移
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, 24)         # 外扩量（至少 24px）
            size = np.array([g.shape[1], g.shape[0]])
            lo = np.clip(np.floor(lo - pad), 0, size).astype(int)
            hi = np.clip(np.ceil(hi + pad), 0, size).astype(int)
            if np.all(hi - lo >= 2):
                candidate = self._search(g, (int(lo[0]), int(lo[1]),
                                             int(hi[0] - lo[0]), int(hi[1] - lo[1])))
            stage = "roi_search"
            self._roi_retry = False                                        # 只允许一次 ROI 搜索
        # ★ v0.6.4 口径保留：跟踪与 ROI 都失败后**必须**再跑一次完整全图搜索。
        #   旧守卫写作 `(not SEARCH_SPLIT_FRAMES and self._proposals < SEARCH_MAX_PROPOSALS)`：
        #   只要前面的 ROI 搜索把候选预算烧光，本帧就**跳过**全图搜索、直接判 no_black_frame
        #   —— 画面里黑框再清楚也照样"丢一帧"。这正是 v1–v3 没有、而后续版本引入的回归。
        exhaustive = bool(getattr(self.cfg, "SEARCH_EXHAUSTIVE", True))
        if (candidate is None and
                (exhaustive or self._search_count == 0 or
                 self._proposals < self.cfg.SEARCH_MAX_PROPOSALS)):
            # ---- ③ full_search：彻底丢失 → 全图低分辨率定位（1~2 张掩膜） ----
            candidate = self._search(g)
            stage = "full_search"
        if candidate is None:
            # 三路都没找到：清空平滑中心与预测历史，返回无效结果（不沿用旧框，避免"粘住"）。
            self._smooth = None
            self._lost += 1
            self._prev_quad = None
            self._prev_timestamp = None
            self.last = dict(valid=False, stage="no_black_frame", search_count=self._search_count,
                             masks=self._mask_count, proposals=self._proposals,
                             detector_ms=(time.perf_counter() - started) * 1000)
            return self.last
        # ---- 找到：锁定、清进度、算中心 ----
        # 预测历史只在"连续两帧都成功"时可用：上上帧 = 旧 _track，失锁后清空避免用旧速度外推。
        self._prev_quad = None
        self._prev_timestamp = None
        if (self._track is not None and not self._lost and
                self._track["kind"] == candidate["kind"] and self._continuous_time(timestamp)):
            # 重搜会重新选“左上角”，旋转跨过该分界时编号可能循环移位。
            # 只匹配同一种内/外框的物理角，避免把胶带厚度或编号切换当作速度。
            previous = np.asarray(self._track["quad"], np.float32)
            if stage == "track":
                self._prev_quad = previous                 # 窄带精修保留编号，稳态无需匹配
            else:
                matches = np.stack([np.roll(previous, shift, axis=0) for shift in range(4)])
                errors = np.sum((matches - candidate["quad"]) ** 2, axis=(1, 2))
                self._prev_quad = matches[int(np.argmin(errors))]
            self._prev_timestamp = self._track_timestamp
        self._track = candidate
        self._track_timestamp = timestamp
        self._lost = 0
        self._roi_retry = True
        w, h = candidate["width_mm"], candidate["height_mm"]
        matrix = candidate["rect_to_image"]
        # ★ 中心 = 正视面中心 (w/2, h/2) 经 mm→px 单应**逆投影**回原图。
        #   （正视图取中心再逆透视反投影，与这一步数学等价；见 `_front_view_center` 的交叉校验。）
        # ★ 中心 = 正视面中心 (w/2, h/2) 经 mm→px 单应**逆投影**回原图 ——
        #   这就是"透视变换成正视图 → 取正视图中心 → 逆透视反投影"的闭式等价写法：
        #   正视图中心恒在 (w/2, h/2)，把它经 rect_to_image 投回原图即得，且**没有重采样误差**。
        #   （重采样版本见 front_view()/front_view_center()，供可视化与独立校验。）
        raw = project_points(matrix, [[w / 2, h / 2]])[0]
        center = self._filter_center(raw, timestamp)
        # 圆周采样点（仅可视化/交叉校验用）按尺寸缓存，避免每帧重算。
        key = (w, h, self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
        if key not in self._circle_cache:
            self._circle_cache[key] = standard_circle(*key)
        result = dict(candidate)
        result.update(valid=True, u=float(center[0]), v=float(center[1]),
                      raw_u=float(raw[0]), raw_v=float(raw[1]),
                      quad=candidate["outer_quad"], detected_quad=candidate["quad"],
                      outer_inferred=candidate["kind"] == "inner",
                      circle_points=project_points(matrix, self._circle_cache[key]),
                      stage=stage, threshold=self._locked_th, search_count=self._search_count,
                      masks=self._mask_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter() - started) * 1000)
        self.last = result
        return result

    # ------------------------------------------------------------------ ⑤ 时序滤波
    def _filter_center(self, raw, timestamp):
        """把逐帧中心送进卡尔曼滤波；返回给显示/常规使用的平滑中心。

        ★ 时钟口径：`timestamp` 与上一帧**不连续**（首帧 / 回退 / 断流 / 传 None）时，
          滤波器重新播种 —— 平滑输出 == 原始测量。这样"时钟不连续必须丢弃旧速度"
          这条既有约束（tools/test_tracking.test_clock_discontinuity）同时作用于滤波器，
          不会出现"检测已判不连续、滤波还在拿旧速度外推"的两套口径。
        """
        raw = np.asarray(raw, np.float64).reshape(2)
        kalman = self._kalman
        if kalman is not None:
            if timestamp is None or kalman.t is None:
                kalman.seed(raw, timestamp)
                estimate = raw
            else:
                estimate = kalman.update(raw, timestamp)
                if estimate is None:
                    estimate = raw
            self._smooth = np.asarray(estimate, np.float64).copy()
            return self._smooth
        # KALMAN_ENABLE=False：退回一阶 EMA（旧口径，保留以便现场对照）。
        alpha = float(self.cfg.CENTER_ALPHA)
        if (self._smooth is None or alpha <= 0 or
                np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX):
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
        return center

    # ------------------------------------------------------------------ 速度/预测查询
    def velocity(self):
        """当前中心速度 (vx, vy) px/s；滤波关闭或无状态时为 None。"""
        return None if self._kalman is None else self._kalman.velocity()

    def predict_center(self, timestamp):
        """按运动学模型外推中心位置（不改状态）；用于给 ROI/窄带中心提供先验。"""
        return None if self._kalman is None else self._kalman.predict_at(timestamp)


def draw_result(gray, result, show_width=480, top_offset=0):
    """缩小后再转 BGR 和描边，输出坐标仍为相机原图坐标。

    top_offset：屏幕上方的文字整体下移量（像素）。激光读数面板占用左上角时由
    main.py 传 vision.laser_spot.PANEL_H，避免两处文字叠在一起。

    绘制元素（均按 scale 换算到缩小图坐标，避免在原始分辨率上画）：
        · 绿色外框 quad、青色内框 inner_quad
        · 洋红四角圆点 + "P1..P4 (x,y)" 标签（原图坐标）
        · 蓝色靶心十字 + "(u,v) stage 耗时" 文字
        · 黄色小十字 = 正视图剖面中心 `front_u/v`（与蓝色十字不等时说明两法不一致）
    返回 BGR 画布，交由 Preview.render 再 letterbox 到屏幕画布。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])        # 原图 → 缩小图 的比例
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        # 无效：只提示一行红字，不画任何框（过期结果已在 display_result 降级）。
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)          # 外框（缩小图坐标）
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)   # 内框（缩小图坐标）
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)               # 洋红实心点
        # 标签用**原图坐标**（x,y），便于与日志/回算对照；位置做边界夹取防出画。
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(int(p[0]) + 5, canvas.shape[1] - tw - 1)),
               max(12, min(int(p[1]) - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 0), 1, cv2.LINE_AA)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)   # 蓝色靶心十字
    cv2.putText(canvas, "(%d,%d) %s %.1fms" % (round(result["u"]), round(result["v"]),
                result["stage"], result["detector_ms"]), (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
