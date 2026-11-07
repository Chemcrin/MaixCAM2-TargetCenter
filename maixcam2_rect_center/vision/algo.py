"""② 算法层：纯函数 —— 输出只由入参决定，无状态、无 I/O、不读 config、不打日志。

四层之一，只有三类内容：

    几何  signed_area / order_quad / line_cross / quad_diagonal_center
    图像  blur_frame / binarize_frame / find_quad_candidates / select_largest
    偏差  deviation（靶心 − 光斑，纯算术）

三条刻意的设计约束（属于本层，不要在别处放宽）：

1. **判据只有三层**：面积 → 四边形近似 → 长宽比。不做胶带暗带证据、亚像素精修、
   单应反算。画面里出现更大的深色矩形时会锁错，这是本方案的已知边界。
2. **中心取四点的对角线交点**，不取正立外接矩形中心 —— 透视下无偏。
3. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "signed_area", "order_quad", "line_cross", "quad_diagonal_center",
    "blur_frame", "binarize_frame", "find_quad_candidates", "select_largest",
    "deviation",
]

# ================================================================== 纯函数
# ------------------------------------------------------------------ 几何
def signed_area(p):
    """带符号面积的 2 倍：>0 逆时针。用于把四角规整成统一方向。"""
    p = np.asarray(p, np.float64)
    return float(p[0, 0] * (p[1, 1] - p[3, 1]) + p[1, 0] * (p[2, 1] - p[0, 1]) +
                 p[2, 0] * (p[3, 1] - p[1, 1]) + p[3, 0] * (p[0, 1] - p[2, 1]))


def order_quad(points):
    """四角规整为 [左上, 右上, 右下, 左下]（逆时针）。

    顺序固定是对角线配对的前提：只有这样 q0-q2 与 q1-q3 才是对角线而不是相邻边。
    """
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


# ------------------------------------------------------------------ 预处理
def blur_frame(gray, kernel):
    """高斯模糊。kernel 为 None 或任一分量为 0/负数时原样返回（关闭模糊）。"""
    k = kernel
    if not k or k[0] <= 0 or k[1] <= 0:
        return gray
    return cv2.GaussianBlur(gray, tuple(k), 0)


def binarize_frame(blur, block_size, c_const):
    """自适应反色：比局部均值暗 c_const 以上的像素变白，黑胶带因此成为白色环带。

    block_size 必须是 >=3 的奇数，否则 OpenCV 直接抛错——在这里先拦，报错更明确。
    """
    if block_size < 3 or block_size % 2 == 0:
        raise ValueError("RECT_BLOCK_SIZE must be an odd integer >= 3")
    return cv2.adaptiveThreshold(
        blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,
        int(block_size), float(c_const))


# ------------------------------------------------------------------ 三层过滤
def find_quad_candidates(binary, min_area, max_area_ratio, eps_ratio,
                         min_aspect, max_aspect, max_candidates=32):
    """面积 / 四边形 / 长宽比三层过滤。返回 (候选列表, 各层淘汰计数)。

    只负责"合不合格"，不负责"选哪个"——择优在 select_largest。
    每个淘汰分支先记账再 continue，于是有一条可自检的等式：
        contours = area + quad + aspect + candidates
    """
    h, w = binary.shape[:2]
    total = float(h * w)
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    stats = dict(contours=len(contours), area=0, quad=0, aspect=0, candidates=0)
    out = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > total * max_area_ratio:
            stats["area"] += 1
            continue
        peri = cv2.arcLength(cnt, True)
        if peri <= 0:
            stats["quad"] += 1
            continue
        approx = cv2.approxPolyDP(cnt, eps_ratio * peri, True)
        if len(approx) != 4:
            stats["quad"] += 1
            continue
        x, y, bw, bh = cv2.boundingRect(approx)
        if bh <= 0 or bw <= 0:
            stats["quad"] += 1
            continue
        # 正立外接框的宽高比。前提是靶纸横放：竖放会落到 1/1.414 附近被这里拒掉，
        # 大角度旋转也会让比值趋近 1。现场若出现这两种情况，改判据而不是抬参数。
        aspect = bw / float(bh)
        if aspect < min_aspect or aspect > max_aspect:
            stats["aspect"] += 1
            continue
        out.append(dict(area=float(area), bbox=(x, y, bw, bh), aspect=float(aspect),
                        approx=approx.reshape(4, 2)))
        stats["candidates"] += 1
        if len(out) >= max_candidates:
            break
    return out, stats


def select_largest(candidates):
    """在通过三层的候选里取面积最大者；空列表返回 None。"""
    return max(candidates, key=lambda c: c["area"]) if candidates else None


# ================================================================== 结果组装


def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定：dx = u靶心 − u光斑，与后续串口直控云台同一套符号：
    电机方向由各轴 DIR_SIGN 映射；像素误差为正不代表固定的电机方向。

    use_raw：闭环取这一档（True）—— 取 result["raw_u"]/["raw_v"]。
    result["u"]/["v"] 是识别输出中心；本链路无滤波，两者当前恒等，
    保留双口径是为了将来加平滑时调用方不必改（平滑值会给闭环带来相位滞后）。
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
