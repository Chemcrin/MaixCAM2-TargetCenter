"""候选定位器：**Canny → findContours → 四边形筛选**，只在低分辨率上跑。

职责边界（与 `rectangle_center` 的分工）
--------------------------------------------------------------------------------
本模块只回答一个问题："画面里哪些地方**可能是**靶纸黑框的四条边？"
它**不判真伪**：输出的每个候选都必须经 `rectangle_center._verify` 的四边暗带 + 内侧亮区
校验才算数。这样"提议"与"裁决"解耦，换提议策略不会动摇精度与误检防线。

两级提议（掩膜数量受控）
--------------------------------------------------------------------------------
    level 0 `edge`  : `Canny` 边缘图（**1 张掩膜**）→ 轮廓 → 四边形。
    level 1 `mask`  : 仅当 level 0 一个都没验过时，才补一张 Otsu 暗区掩膜
                      （用轮廓树层级直接定内/外框，省掉"两种假设都试"的开销）。

旧实现是"Otsu + 8 档 `DARK_THRESHOLDS` 逐档阈值化 × 每档 findContours"共 9~10 张掩膜，
板端实测单帧 ~312ms（detect_fps=3.2，见 板端识别问题整理_2026-10-13.md）。
改成"1 张（必要时 2 张）"后，最贵的那部分（阈值化 + 形态学 + 轮廓树，都在低分辨率上）
直接降了一个数量级。

为什么 Canny 边缘图的轮廓**能**近似出黑框矩形（而不是一堆碎线段）
--------------------------------------------------------------------------------
Canny 输出的是 1px 细线：胶带带的外沿与内沿各是一条**闭合细环**，环的轮廓正好包住整个
矩形内部，`approxPolyDP` 一把就降到 4 点。即使胶带环在角落断开（实机常态），带的外沿/
内沿/两端缺口会共同围成一个**闭合的细带**，其轮廓"中空程度"高（面积 ≪ 凸包面积），
于是 `quad_from_contour` 的凸包兜底被启用，缺口被补上 → 仍能得到完整矩形。
真伪始终由 `_verify` 裁决，所以"凸包把阴影拼成矩形"这类情况不会变成误检。

坐标口径
--------------------------------------------------------------------------------
搜索在缩小图上做（`SEARCH_WIDTH`），**返回的四边形已换算回原图坐标**（含 ROI 偏移），
调用方拿到的就是可直接 `_verify` / `_refine` 的原图坐标。
"""
import time

import cv2
import numpy as np

from .rect_geometry import quad_from_contour, quad_is_valid

# 提议级别：0 = Canny 边缘（默认，1 张掩膜）；1 = Otsu 暗区掩膜（兜底，仅第一级全失败时）
LEVEL_EDGE, LEVEL_MASK = 0, 1
LEVELS = (LEVEL_EDGE, LEVEL_MASK)


class ContourLocator:
    """低分辨率候选提议器（无跨帧状态；每帧独立、可重复调用）。"""

    def __init__(self, config):
        self.cfg = config
        close_px = int(getattr(config, "EDGE_CLOSE_PX", 3))
        self._kernel = None if close_px <= 1 else cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_px, close_px))
        self.last_stats = dict(masks=0, contours=0, quads=0, ms=0.0, level=None)

    # ------------------------------------------------------------------ 对外
    def locate(self, gray, roi=None, level=LEVEL_EDGE):
        """在 `roi`（默认全图）内提议候选四边形。

        返回 ``[(quad(float32 (4,2), 原图坐标), kind), ...]``，**已按优先级排序**：
        同一个四边形的 "inner" 假设排在 "outer" 之前（亮面孔比与背景粘连的黑环更稳定）。
        """
        started = time.perf_counter()
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        # 搜索恒在低分辨率：宽超过 SEARCH_WIDTH 就等比缩小（ROI 搜索时 w 小 → 不缩）。
        scale = min(1.0, float(self.cfg.SEARCH_WIDTH) / float(w))
        sw = max(32, int(round(w * scale)))
        sh = max(24, int(round(h * scale)))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        sxy = np.array([w / float(sw), h / float(sh)], np.float32)
        offset = np.array([x, y], np.float32)

        masks, contours_n = self._masks(small, level)
        quads, seen = [], []
        for mask in masks:
            for q_small in self._quads_from_mask(mask, small.shape):
                q = q_small * sxy + offset
                if not quad_is_valid(q, gray.shape,
                                     min_edge=float(getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.0))):
                    continue
                key = np.rint(q).astype(np.int32).tolist()
                if key in seen:                       # 同一个框被多条轮廓重复提及时只留一次
                    continue
                seen.append(key)
                quads.append(q)
                if len(quads) >= int(getattr(self.cfg, "MAX_CANDIDATES", 8)):
                    break
            if len(quads) >= int(getattr(self.cfg, "MAX_CANDIDATES", 8)):
                break
        proposals = []
        for q in quads:
            for kind in ("inner", "outer"):
                proposals.append((q, kind))
        self.last_stats = dict(masks=len(masks), contours=contours_n, quads=len(quads),
                               ms=(time.perf_counter() - started) * 1000.0, level=level)
        return proposals

    # ------------------------------------------------------------------ 掩膜
    def _masks(self, small, level):
        """生成该级别需要的掩膜（edge 级 1 张；mask 级 1 张）。返回 (掩膜列表, 轮廓数)。"""
        if level == LEVEL_EDGE:
            edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
            if self._kernel is not None:
                # 3px 闭运算把 Canny 常见的一两个像素断裂接上，避免轮廓碎成多段；
                # 这一步只在低分辨率图（≤320 宽）上做，代价可忽略。
                edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, self._kernel)
            return [edges], -1
        _, mask = cv2.threshold(small, 0, 255,
                                cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)   # 暗区为前景
        if self._kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        return [mask], -1

    # ------------------------------------------------------------------ 轮廓筛选
    def _quads_from_mask(self, mask, shape):
        """轮廓 → 四边形，按"像不像一个够大的框"廉价预筛后交给 `quad_from_contour`。

        ★ 命中足够多的四边形就**立刻停止扫轮廓**：`quad_from_contour` 内部要做
          epsilon 扫描（最多 5 次 `approxPolyDP`，环形带还要再来一遍凸包），单次不便宜；
          而调用方只会用前 `MAX_CANDIDATES` 个候选。不限次数的版本在"边缘密"的画面上
          会把预算全花在后面的碎轮廓上（实测是搜索路径的头号开销来源）。
        """
        found = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = found[-2]
        if not contours:
            return []
        h, w = shape
        area_frame = float(w * h)
        limit_scan = int(getattr(self.cfg, "SEARCH_MAX_CONTOURS", 16))
        want = int(getattr(self.cfg, "MAX_CANDIDATES", 8))
        ids = sorted(range(len(contours)), key=lambda i: abs(cv2.contourArea(contours[i])),
                     reverse=True)[:limit_scan]
        out, min_ratio = [], float(self.cfg.MIN_AREA_RATIO)
        max_ratio = float(self.cfg.MAX_AREA_RATIO)
        min_side = max(8, int(0.04 * min(w, h)))
        for i in ids:
            if len(out) >= want:
                break
            cnt = contours[i]
            bx, by, bw, bh = cv2.boundingRect(cnt)
            if min(bw, bh) < min_side:
                continue                                        # 又小又扁：不是框
            area = abs(cv2.contourArea(cnt))
            # 边缘图的细环轮廓"面积"就是环内空腔面积，与实心掩膜的框面积口径一致。
            if not area_frame * min_ratio <= area <= area_frame * max_ratio:
                continue
            q = quad_from_contour(cnt, self.cfg.APPROX_EPSILON, self.cfg.HULL_FILL_RATIO)
            if q is not None:
                out.append(q)
        return out
