"""② 算法层：纯函数 —— 输出只由入参决定，无状态、无 I/O、不读 config、不打日志。

四层之一，只有三类内容：

    几何  signed_area / order_quad / line_cross / quad_diagonal_center
    图像  blur_frame / binarize_frame / find_quad_candidates / select_largest
          find_concentric（同心圆，独立成链）
    偏差  deviation（靶心 − 光斑，纯算术）

四条刻意的设计约束（属于本层，不要在别处放宽）：

1. **四边形判据只有三层**：面积 → 四边形近似 → 长宽比。不做胶带暗带证据、亚像素精修、
   单应反算。画面里出现更大的深色矩形时会锁错，这是本方案的已知边界。
2. **同心圆是并行证据，不是第四层判据**：`find_concentric` 走 RETR_LIST 独立成链，
   命中即直接给出圆心；它不放宽上面三层，三层也不因它改变。
3. **中心取四点的对角线交点**，不取正立外接矩形中心 —— 透视下无偏；同心圆路径取圆心。
4. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "signed_area", "order_quad", "line_cross", "quad_diagonal_center",
    "blur_frame", "binarize_frame", "find_quad_candidates", "select_largest",
    "find_concentric", "deviation",
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


# ------------------------------------------------------------------ 同心圆
def find_concentric(binary, min_radius, max_radius_ratio,
                    min_circularity=0.80, min_rings=2, min_outer_radius=20.0,
                    close_kernel=3, max_contours=240):
    """在二值图里找一组同心圆，返回 (u, v, r_max, rings) 或 None。

    为什么需要这条独立路径：三条四边形判据要求靶纸四周存在**闭合暗环**，
    没贴胶带、或胶带被高光/反光打断时整张靶纸连候选都进不了；而画面上任何
    "被暗边围住的横放矩形"（墙、柜门、门框）反而全部通过，取最大面积就锁错。
    同心圆是靶纸独有的证据，且**圆心就是靶心本身**，不依赖长宽比前提。

    判据链（任一不满足即整组作废）：
        单圈圆度 4πA/P² >= min_circularity   —— 区分圆与矩形（正方形仅 0.785）
        半径去重：同一环的内沿/外沿半径只差线宽，合并成一个 ring
        圆心聚簇：圆心相距 <= 10% 半径的归为同一组
        簇内不同半径数 >= min_rings、r_max/r_min >= 1.5、r_max >= min_outer_radius

    为什么要先做闭运算：印在纸上的圆环只有 1~2px 宽，二值化后常被抗锯齿切成
    若干小弧段；弧段不闭合，`contourArea` 退化成"线宽×弧长"，圆度接近 0，
    整圈就此消失。3×3 闭运算能把 1~2px 的断口补上，代价是半径略胖 1~2px，
    对"取圆心"没有影响。

    min_rings 取 2 而不是靶纸的圈数：真实帧里常只有最外圈 + 一圈内环是闭合的
    （`real_scene_corner_gap.png` 实测 2 组半径、圆度 0.89/0.91），
    要求 3 圈会把真靶纸一起拒掉。误报由 r_max/r_min >= 1.5 与 min_outer_radius 兜。

    不做拟合、不做亚像素：`minEnclosingCircle` 的圆心在像素级已足够，
    后续偏差与闭环都按像素工作，亚像素对 165cm 处 0.67px 的靶心没有意义。
    """
    h, w = binary.shape[:2]
    max_r = float(max_radius_ratio) * min(h, w)
    src = binary
    if int(close_kernel) >= 3:
        k = int(close_kernel) | 1                     # 闭运算核必须是奇数
        src = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,
                               np.ones((k, k), np.uint8))
    contours = cv2.findContours(src, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    rings = []
    for cnt in contours[:int(max_contours)]:
        area = cv2.contourArea(cnt)
        if area < 12.0:
            continue
        peri = cv2.arcLength(cnt, True)
        if peri <= 0:
            continue
        if 4.0 * math.pi * area / (peri * peri) < min_circularity:
            continue
        (cx, cy), r = cv2.minEnclosingCircle(cnt)
        if r < float(min_radius) or r > max_r:
            continue
        rings.append((float(cx), float(cy), float(r)))
    if len(rings) < int(min_rings):
        return None

    # 按圆心聚簇：同一组同心圆的各圈圆心高度重合，不同位置的圆不会互吞。
    clusters = []
    for cx, cy, r in rings:
        for cl in clusters:
            if math.hypot(cx - cl["x"], cy - cl["y"]) <= max(4.0, 0.10 * cl["r"]):
                n = cl["n"]
                cl["x"] = (cl["x"] * n + cx) / (n + 1)
                cl["y"] = (cl["y"] * n + cy) / (n + 1)
                cl["r"] = max(cl["r"], r)
                cl["n"] = n + 1
                cl["radii"].append(r)
                break
        else:
            clusters.append(dict(x=cx, y=cy, r=r, n=1, radii=[r]))

    best = None
    for cl in clusters:
        uniq = []
        for r in sorted(cl["radii"]):
            if not uniq or r - uniq[-1] > max(3.0, 0.09 * uniq[-1]):
                uniq.append(r)
        if len(uniq) < int(min_rings):
            continue
        if uniq[-1] < float(min_outer_radius):
            continue
        if uniq[-1] / uniq[0] < 1.5:
            continue
        # 圈数越多、最外圈越大越可信；同分时留大的那组。
        score = len(uniq) * uniq[-1]
        if best is None or score > best[0]:
            best = (score, cl["x"], cl["y"], uniq[-1], len(uniq))
    return None if best is None else (best[1], best[2], best[3], best[4])


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
