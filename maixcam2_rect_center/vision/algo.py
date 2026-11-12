"""② 算法层：纯函数 —— 输出只由入参决定，无状态、无 I/O、不读 config、不打日志。

四层之一，只有三类内容：

    几何  line_cross / quad_diagonal_center / order_quad / project_points
          rect_quad / poly_area / edge_lengths / valid_quad
    采样  sample_points / profile_samples / refine_sample_points
    偏差  deviation（靶心 − 光斑，纯算术）

链路核心（为什么这样组织，每一条都对应板上的真实教训）：

1. **板上一次 Python 级 numpy/cv2 调用 ≈ 1ms，单帧耗时 ≈ 调用次数**（不是像素数）。
   热路径只允许整批一次的大图 cv2 调用（缩放/阈值/形态学/轮廓/remap），
   以及 `tolist()` 之后的小数组纯 Python 循环（求交/取中位/判门）。
2. **黑框是"一定物理宽度的暗带围出一块亮面"**。`profile_samples` 把物理矩形边
   按真实带宽（`BLACK_FRAME_MM`）向内外各取两点：暗带两点都须暗、纸面一点须明显更亮。
   带宽因此进入判据，细边框（显示器窄边、印刷细环）过不了；洞洞板、门框这类
   "没有闭合暗带"的结构也在覆盖率门上出局。
3. **中心取纸边（内框）对角线交点**。胶带上下不等宽时外框中心会偏出半个带宽，
   纸边不会；透视下每条边仍是直线，四线交角是几何精确解，不需要逐帧 warp。
4. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "line_cross", "quad_diagonal_center", "poly_area", "edge_lengths", "order_quad",
    "project_points", "rect_quad", "valid_quad",
    "sample_points", "profile_samples", "refine_sample_points",
    "deviation",
]


# ================================================================== 几何
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
    """对角线交点 = 透视下的矩形中心。退化时返回 None，由调用方判失败。

    纯 Python（板上每次 numpy 调用 ~1ms，四点算术不值得付这个钱）。
    """
    q = [(float(p[0]), float(p[1])) for p in quad]
    if len(q) != 4:
        return None
    p = line_cross(q[0], q[2], q[1], q[3])
    if p is None or not (math.isfinite(p[0]) and math.isfinite(p[1])):
        return None
    return p


def poly_area(quad):
    """有向面积（图像坐标 y 向下时，顺时针为负）。用于判四边形绕向。"""
    return float(cv2.contourArea(np.asarray(quad, np.float32).reshape(-1, 1, 2), oriented=True))


def edge_lengths(quad):
    """四条边的长度，按顶点循环顺序返回。"""
    q = np.asarray(quad, np.float32).reshape(4, 2)
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def order_quad(points):
    """把四点整理成循环顺序，并把 x+y 最小的点转到首位。

    不强制图像的 TL/TR/BR/BL —— 框转过 45° 后"左上"不再是物理角点；
    只保证四点在环上相邻，后续 edges[i] 与 edges[i+1] 一定是物理相邻边。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    center = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0]))]
    if poly_area(p) < 0:
        p = p[::-1]
    return np.roll(p, -int(np.argmin(p[:, 0] + p[:, 1])), axis=0).copy()


def project_points(matrix, points):
    """单应映射一批点；输入输出都是 (N,2)。"""
    p = np.asarray(points, np.float32).reshape(1, -1, 2)
    return cv2.perspectiveTransform(p, np.asarray(matrix, np.float64)).reshape(-1, 2)


def rect_quad(width, height, inset=0.0):
    """物理矩形的四角（mm），inset 为向内收缩量；顺序 TL/TR/BR/BL。"""
    return np.array([[inset, inset], [width - inset, inset],
                     [width - inset, height - inset], [inset, height - inset]], np.float32)


def valid_quad(quad, shape, min_side, min_area, allow_border=False, min_sine=0.16):
    """四边形合法性门：不出画、凸、最短边够长、相邻边不共线、面积够大。

    min_sine 是相邻边夹角的正弦下限（0.16 ≈ 9°），挡"几乎退化成一条线"的提案。
    阈值由调用方传入，本层不读 config。
    """
    q = np.asarray(quad, np.float32).reshape(4, 2)
    h, w = shape[0], shape[1]
    margin = 1 if allow_border else 2
    if (q[:, 0].min() < margin or q[:, 1].min() < margin or
            q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
        return False
    if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
        return False
    e = np.roll(q, -1, axis=0) - q
    lengths = np.linalg.norm(e, axis=1)
    if lengths.min() < float(min_side):
        return False
    cross = np.abs(e[:, 0] * np.roll(e[:, 1], -1) - e[:, 1] * np.roll(e[:, 0], -1))
    if np.min(cross / (lengths * np.roll(lengths, -1) + 1e-6)) < float(min_sine):
        return False
    return abs(poly_area(q)) >= float(min_area)


# ================================================================== 采样
def sample_points(gray, points):
    """按亚像素坐标批量采样灰度（一次 remap 取全部点，避免 Python 级逐点索引）。"""
    p = np.asarray(points, np.float32).reshape(-1, 2)
    return cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(points.shape[:-1])


def profile_samples(kind, width, height, tape, t, paper_inset=4.0):
    """构造"暗带两点 + 纸面一点"的物理采样坐标，返回 (4边, len(t), 3, 2)。

    kind="inner"：入参是纸边（内框）四角，暗带在纸面外侧，故暗带采样沿外法线；
    kind="outer"：入参是胶带外沿四角，暗带在内侧，纸面见证点再往内让一个带宽。
    两种情形下"纸面见证点"永远比暗带点更深地落在靶纸内部，判据方向因此一致。

    纯几何、可缓存：同一 (kind, 物理尺寸, 带宽) 的采样模板整段会话只算一次。
    """
    boundary = rect_quad(width, height, tape if kind == "inner" else 0.0)
    edges = np.roll(boundary, -1, axis=0) - boundary
    if np.linalg.norm(edges, axis=1).min() < 1:
        return None
    center = np.array([width * 0.5, height * 0.5], np.float32)
    mid = (boundary + np.roll(boundary, -1, axis=0)) * 0.5
    inward = center[None, :] - mid
    inward /= np.maximum(np.linalg.norm(inward, axis=1, keepdims=True), 1e-6)
    base = (boundary[:, None, :] * (1 - t[None, :, None]) +
            np.roll(boundary, -1, axis=0)[:, None, :] * t[None, :, None])
    if kind == "inner":
        dark_offsets = (-0.35 * tape, -0.72 * tape)   # 负号 = 沿外法线离开纸面
        paper_offsets = float(paper_inset)
    else:
        dark_offsets = (0.35 * tape, 0.72 * tape)
        paper_offsets = float(tape) + float(paper_inset)
    out = np.empty((4, len(t), 3, 2), np.float32)
    out[:, :, 0, :] = base + inward[:, None, :] * dark_offsets[0]
    out[:, :, 1, :] = base + inward[:, None, :] * dark_offsets[1]
    out[:, :, 2, :] = base + inward[:, None, :] * paper_offsets
    return out


def refine_sample_points(quad, t, radius):
    """构造沿边法线的窄带采样点，返回 (points, base, normals, offsets)。

    points 形如 (4边, len(t), 2*radius+1, 2)：每条边每个采样位置取一排灰度，
    梯度极值即亚像素边位置；base/normals 供调用方把位移还原成图像坐标。
    """
    q = np.asarray(quad, np.float32).reshape(4, 2)
    nxt = np.roll(q, -1, axis=0)
    center = q.mean(axis=0)
    normals = center[None, :] - (q + nxt) * 0.5
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-6)
    base = q[:, None, :] * (1 - t[None, :, None]) + nxt[:, None, :] * t[None, :, None]
    offsets = np.arange(-radius, radius + 1, dtype=np.float32)
    points = base[:, :, None, :] + normals[:, None, None, :] * offsets[None, None, :, None]
    return points, base, normals, offsets


# ================================================================== 偏差
def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定：dx = u靶心 − u光斑，与后续串口直控云台同一套符号：
    电机方向由各轴 DIR_SIGN 映射；像素误差为正不代表固定的电机方向。

    use_raw：闭环取这一档（True）—— 取 result["raw_u"]/["raw_v"]。
    result["u"]/["v"] 是显示用的平滑中心；闭环用 raw 是为了不引入相位滞后。
    激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])   # 闭环档：未平滑中心
    else:
        tx, ty = float(result["u"]), float(result["v"])           # 显示档：识别输出中心
    lu, lv = float(laser['u']), float(laser['v'])
    if not all(map(math.isfinite, (tx, ty, lu, lv))):
        return None
    size = result.get('image_size')
    if size is not None and not (0 <= lu < size[0] and 0 <= lv < size[1]):
        return None
    return (tx - lu, ty - lv)       # dx=靶心−光斑, dy 同理
