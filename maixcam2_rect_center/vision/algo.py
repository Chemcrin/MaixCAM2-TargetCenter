"""② 算法层：纯函数 —— 输出只由入参决定，无状态、无 I/O、不读 config、不打日志。

四层之一，只有三类内容：

    几何  signed_area / order_quad / line_cross / quad_diagonal_center
    图像  downscale / auto_level / dark_mask / components / frame_candidates
          paper_rect / rect_quad / nearest_component
    偏差  deviation（靶心 − 光斑，纯算术）

四条刻意的设计约束（属于本层，不要在别处放宽）：

1. **只认"四周被暗带围住的矩形"**：面积 → 长宽比 → 填充率 → 四边压暗带。
   不做图案识别（纸面同心圆不参与），因此"亮面被暗边包住"的东西（屏幕、窗框）
   在形状上可能被当成靶纸 —— 这是本方案的已知边界。
2. **每次调用只做几次 cv2/numpy 运算**：板上一次 Python 级调用约 1ms，
   本层的调用次数直接等于单帧耗时。禁止在这里写"逐轮廓 / 逐采样点"的循环。
3. **中心取"框心向外扫四条线"的中点**，不取外接框中心 —— 阴影会让暗带在一侧变宽，
   外接框中心会系统性偏移（实测 6~8px），过框心的弦中点则没有这个问题。
4. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "signed_area", "order_quad", "line_cross", "quad_diagonal_center",
    "downscale", "auto_level", "dark_mask", "components", "frame_candidates",
    "nearest_component", "paper_rect", "rect_quad", "box_center", "deviation",
]

# ================================================================== 几何
def signed_area(p):
    """带符号面积的 2 倍：>0 逆时针。用于把四角规整成统一方向。"""
    p = np.asarray(p, np.float64)
    return float(p[0, 0] * (p[1, 1] - p[3, 1]) + p[1, 0] * (p[2, 1] - p[0, 1]) +
                 p[2, 0] * (p[3, 1] - p[1, 1]) + p[3, 0] * (p[0, 1] - p[2, 1]))


def order_quad(points):
    """四角规整为 [左上, 右上, 右下, 左下]（逆时针）。"""
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]
    if signed_area(p) < 0:
        p = p[::-1]
    i = int(np.argmin(p.sum(axis=1)))
    return np.concatenate((p[i:], p[:i]))


def line_cross(a, b, c, d):
    """线段 ab 与 cd 所在直线的交点；平行或退化返回 None。"""
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    cx, cy = float(c[0]), float(c[1])
    dx, dy = float(d[0]), float(d[1])
    den = (ax - bx) * (cy - dy) - (ay - by) * (cx - dx)
    if abs(den) < 1e-9:
        return None
    t = ((ax - cx) * (cy - dy) - (ay - cy) * (cx - dx)) / den
    return ax + t * (bx - ax), ay + t * (by - ay)


def quad_diagonal_center(quad):
    """对角线交点 = 透视下的矩形中心。退化时返回 None，由调用方判失败。"""
    q = np.asarray(quad, np.float32)
    p = line_cross(q[0], q[2], q[1], q[3])
    if p is None or not np.isfinite(p).all():
        return None
    return p


def rect_quad(left, top, right, bottom):
    """轴对齐矩形 → 四角（顺序与 order_quad 一致）。"""
    return np.array([[left, top], [right, top], [right, bottom], [left, bottom]], np.float32)


def box_center(box):
    """(x, y, w, h) → 中心点。"""
    return (float(box[0]) + float(box[2]) * 0.5, float(box[1]) + float(box[3]) * 0.5)


# ================================================================== 图像
def downscale(gray, factor):
    """整数倍降采样；factor<=1 原样返回。

    用 INTER_AREA（2×2 均值）而不是隔点取样：同样的调用次数，均值把反光噪点
    压掉一半，胶带环更连续。降采样是"减少像素"的唯一手段，也是本链路提速的第一步。
    """
    f = int(factor)
    if f <= 1:
        return gray
    h, w = gray.shape[:2]
    return cv2.resize(gray, (max(8, w // f), max(8, h // f)), interpolation=cv2.INTER_AREA)


def auto_level(small, lo_pct=2.0, hi_pct=90.0, ratio=0.62, floor=45.0, ceil=160.0):
    """自适应二值阈值：压在"胶带（暗）"与"纸面（亮）"之间。

    取 90 分位当亮面基准，阈值 = ratio × 它：实测纸面约 190、胶带约 50，
    0.62×190≈118 正好卡在中间；纸面随曝光变暗时阈值同步下移，不会失效。
    分位数只在 1/16 的像素上算（步长 4），两次 numpy 调用就够。
    """
    sub = small[::4, ::4]
    lo = float(np.percentile(sub, lo_pct))
    hi = float(np.percentile(sub, hi_pct))
    return float(np.clip(ratio * hi, max(float(floor), lo + 15.0), float(ceil)))


def dark_mask(small, level):
    """暗于 level 的像素置 255。胶带、门缝、阴影都进来，靠后面的形状门筛掉。"""
    return cv2.threshold(small, float(level), 255, cv2.THRESH_BINARY_INV)[1]


def components(mask):
    """连通域统计：一次调用拿到所有块的 (x, y, w, h, 面积) 与质心。

    比 findContours + 逐轮廓 boundingRect 快一个数量级 —— 板上每次 Python 级调用
    约 1ms，几百个轮廓就是几百毫秒，而这里只有一次。
    """
    n, _lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return np.zeros((0, 5), np.int32), np.zeros((0, 2), np.float64)
    return stats[1:], cent[1:]


def frame_candidates(stats, cent, mask, min_area, max_area_ratio, min_aspect,
                     max_aspect, min_fill, max_fill, min_edge, probe=6):
    """从连通域里挑"四周被暗带围住的矩形"，返回第一个通过全部判据的候选。

    四道门：面积 → 长宽比 → 填充率 → **四条边都压着暗带**。
    最后一道是关键：画面里"框住靶心的稀疏暗结构"（窗棂、柜门缝、桌沿）外接框
    也含靶心，但它没有四条边；胶带环四条边都有暗带，缺一个角也不影响
    （缺角只让那一条边的暗率下降，不会归零）。
    """
    h, w = mask.shape[:2]
    total = float(h * w)
    if stats.size == 0:
        return []
    sx = stats[:, 0].astype(np.float64)
    sy = stats[:, 1].astype(np.float64)
    bw = stats[:, 2].astype(np.float64)
    bh = stats[:, 3].astype(np.float64)
    area = stats[:, 4].astype(np.float64)
    box = bw * bh
    short = np.minimum(bw, bh)
    aspect = np.maximum(bw, bh) / np.maximum(1.0, short)
    fill = area / np.maximum(1.0, box)
    keep = ((short >= 14.0) & (area >= float(min_area)) &
            (box <= total * float(max_area_ratio)) &
            (aspect >= float(min_aspect)) & (aspect <= float(max_aspect)) &
            (fill >= float(min_fill)) & (fill <= float(max_fill)))
    idx = np.flatnonzero(keep)
    if idx.size == 0:
        return []
    idx = idx[np.argsort(-box[idx])][:int(probe)]
    for i in idx:
        x0, y0 = int(sx[i]), int(sy[i])
        w0, h0 = int(bw[i]), int(bh[i])
        band = 3 if min(w0, h0) > 6 else 1
        edge = min(float(mask[y0:y0 + band, x0:x0 + w0].mean()),
                   float(mask[y0 + h0 - band:y0 + h0, x0:x0 + w0].mean()),
                   float(mask[y0:y0 + h0, x0:x0 + band].mean()),
                   float(mask[y0:y0 + h0, x0 + w0 - band:x0 + w0].mean())) / 255.0
        if edge < float(min_edge):
            continue
        return [dict(x=x0, y=y0, w=w0, h=h0, area=float(area[i]),
                     aspect=float(aspect[i]), fill=float(fill[i]), edge=edge)]
    return []


def nearest_component(stats, cent, target, ref_w, ref_h, size_tol=0.55, dist_ratio=1.2):
    """在连通域里挑离预测中心最近、尺寸与上一帧相当的那块；没有则 None。

    跟踪时用它：ROI 里通常只有胶带环这一块尺寸对得上，其它碎块要么太小、
    要么离预测中心太远。尺寸容差 ±55% 覆盖"车在靠近/远离"的情况。
    """
    if stats.size == 0:
        return None
    dx = cent[:, 0] - float(target[0])
    dy = cent[:, 1] - float(target[1])
    dist = np.hypot(dx, dy)
    wx = stats[:, 2] / max(1.0, float(ref_w))
    hy = stats[:, 3] / max(1.0, float(ref_h))
    ok = (wx > 1.0 - float(size_tol)) & (wx < 1.0 + float(size_tol)) & \
         (hy > 1.0 - float(size_tol)) & (hy < 1.0 + float(size_tol))
    ok &= dist < float(dist_ratio) * max(float(ref_w), float(ref_h))
    if not np.any(ok):
        return None
    idx = np.flatnonzero(ok)
    k = int(idx[np.argmin(dist[idx])])
    return dict(x=int(stats[k, 0]), y=int(stats[k, 1]),
                w=int(stats[k, 2]), h=int(stats[k, 3]),
                area=float(stats[k, 4]), cx=float(cent[k, 0]), cy=float(cent[k, 1]))


def paper_rect(gray, box, level, rounds=1):
    """框心向外扫四条线，返回纸边 (left, top, right, bottom)；扫不到返回 None。

    为什么从里往外扫：胶带环的外接框总是偏的（下缘常被阴影撑宽，实测偏 6~8px），
    而纸面一定在框内。从框心向四个方向找第一个"亮→暗"的跨越，取左右/上下的中点 ——
    对中心对称图形，过任意点的水平弦中点仍在中心线上，所以一轮就足以把偏差消掉。
    """
    x, y, w, h = int(box[0]), int(box[1]), int(box[2]), int(box[3])
    if w < 12 or h < 12:
        return None
    cx, cy = x + w // 2, y + h // 2
    best = None
    for _ in range(max(1, int(rounds))):
        i, j = cx - x, cy - y
        # 每个方向取相邻三条线的最小值再扫：胶带反光会让单独一行整段变亮，
        # 三行取最小等于"只要有任意一行是暗的就算暗"，能跨过 1~2px 的断口。
        row = gray[max(y, cy - 1):min(y + h, cy + 2), x:x + w].min(axis=0)
        col = gray[y:y + h, max(x, cx - 1):min(x + w, cx + 2)].min(axis=1)
        r = np.flatnonzero(row[i:] < level)
        l = np.flatnonzero(row[:i + 1][::-1] < level)
        d = np.flatnonzero(col[j:] < level)
        u = np.flatnonzero(col[:j + 1][::-1] < level)
        if not (r.size and l.size and d.size and u.size):
            break                     # 本轮扫不到（通常会撞到框边），保留上一轮的结果
        left = x + i - int(l[0])
        right = x + i + int(r[0])
        top = y + j - int(u[0])
        bottom = y + j + int(d[0])
        if right - left < 12 or bottom - top < 12:
            break
        best = (left, top, right, bottom)
        ncx, ncy = (left + right) // 2, (top + bottom) // 2
        if abs(ncx - cx) <= 1 and abs(ncy - cy) <= 1:
            break
        cx, cy = ncx, ncy
    return best


# ================================================================== 偏差
def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定：dx = u靶心 − u光斑；电机方向由各轴 DIR_SIGN 映射，像素误差为正不代表
    固定的电机方向。激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])
    else:
        tx, ty = float(result["u"]), float(result["v"])
    lu, lv = float(laser['u']), float(laser['v'])
    if not all(map(math.isfinite, (tx, ty, lu, lv))):
        return None
    size = result.get('image_size')
    if size is not None and not (0 <= lu < size[0] and 0 <= lv < size[1]):
        return None
    return (tx - lu, ty - lv)
