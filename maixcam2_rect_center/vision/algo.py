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
    "bilinear", "frame_edge_points", "fit_line", "quad_from_lines", "refine_frame",
    "solve_homography", "apply_homography", "invert_homography", "plane_quad",
    "sample_polar", "ring_report", "quad_mix", "band_evidence",
    "find_frame_candidates",
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


# ------------------------------------------------------------------ 灰度采样
def bilinear(gray, xs, ys):
    """双线性采样，越界夹到边缘。xs/ys 可为标量或任意形状数组。"""
    g = gray.astype(np.float32, copy=False)
    h, w = g.shape[:2]
    x = np.clip(np.asarray(xs, np.float32), 0.0, w - 2.0)
    y = np.clip(np.asarray(ys, np.float32), 0.0, h - 2.0)
    x0 = np.floor(x).astype(np.int32)
    y0 = np.floor(y).astype(np.int32)
    fx = x - x0
    fy = y - y0
    return (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x0 + 1] * fx * (1 - fy) +
            g[y0 + 1, x0] * (1 - fx) * fy + g[y0 + 1, x0 + 1] * fx * fy)


def _runs(flags):
    """连续 True 的区间：返回 (起点数组, 终点数组[不含])。"""
    f = np.asarray(flags, np.int8)
    d = np.diff(np.concatenate([[0], f, [0]]))
    return np.flatnonzero(d == 1), np.flatnonzero(d == -1)


def _crossing(prof, i, level, offs):
    """prof[i-1] → prof[i] 之间 level 的穿越位置，按 offs 插值成实际偏移。"""
    if i <= 0 or i >= prof.size:
        return float(offs[min(max(i, 0), offs.size - 1)])
    a, b = float(prof[i - 1]), float(prof[i])
    t = 0.0 if abs(b - a) < 1e-6 else float(np.clip((level - a) / (b - a), 0.0, 1.0))
    return float(offs[i - 1] + t * (offs[i] - offs[i - 1]))


def frame_edge_points(gray, a, b, center, samples=14, span=6.0, inward=30.0,
                      rise=10.0, step=0.5):
    """沿一条边 a→b 采样，返回 (外沿点, 内沿点) 两组亚像素坐标。

    判据用的是"胶带是一段连续暗带"，而不是"哪里有梯度"：
      外沿 = 从外向内第一次出现、并能持续 3 个采样的暗区起点；
      内沿 = 从外沿继续向内，暗区结束、回到纸面亮度的位置。
    这样胶带内部的噪点、阴影边缘、以及**遮挡物**（比纸暗但远不如胶带黑）
    都不会被当成边界——灰斑那类东西的"暗"和胶带的"暗"不在一个量级。

    没有可判定的暗带（被完全遮挡 / 该处不是胶带）就跳过这个采样点；
    调用方按"每条边至少 4 点"判成败，缺的角由相邻两条直线求交补回来。
    """
    a = np.asarray(a, np.float64)
    b = np.asarray(b, np.float64)
    c = np.asarray(center, np.float64)
    d = b - a
    length = float(np.hypot(d[0], d[1]))
    if length < 8.0:
        return np.empty((0, 2)), np.empty((0, 2))
    d = d / length
    nrm = np.array([-d[1], d[0]])
    if float(nrm @ ((a + b) * 0.5 - c)) < 0.0:
        nrm = -nrm
    out_pts, in_pts = [], []
    # 沿外法线从 +span（框外）走到 -inward（框内）：先遇到暗带起点，再遇到暗带终点。
    offs = np.arange(span, -inward - 1e-9, -step)
    zero = int(round(span / step))
    ts = np.linspace(0.15, 0.85, int(samples))
    base = a[None, :] + ts[:, None] * (b - a)[None, :]
    gx = base[:, 0:1] + nrm[0] * offs[None, :]
    gy = base[:, 1:2] + nrm[1] * offs[None, :]
    prof = bilinear(gray, gx, gy)                 # (samples, len(offs)) 一次取完
    lo = np.percentile(prof, 10.0, axis=1)
    hi = np.percentile(prof, 90.0, axis=1)
    for k in range(prof.shape[0]):
        if hi[k] - lo[k] < float(rise):
            continue
        row = prof[k]
        level = lo[k] + 0.40 * (hi[k] - lo[k])
        starts, ends = _runs(row < level)
        if starts.size == 0:
            continue
        j = int(np.argmin(np.abs(starts - zero)))
        s0, s1 = int(starts[j]), int(ends[j])
        if s1 - s0 < 4 or s1 >= row.size - 2:
            continue                     # 暗带太窄，或延伸到窗口末端（内沿不在窗内）
        pos_out = _crossing(row, s0, level, offs) if s0 > 0 else float(offs[0])
        pos_in = _crossing(row, s1, level, offs)
        if pos_out - pos_in < 2.0:
            continue
        dark_mean = float(row[s0:s1].mean())
        bright_mean = float(row[s1:].mean())
        if bright_mean < 40.0 or dark_mean > 0.60 * bright_mean:
            continue                     # 遮挡物/阴影：暗得不够"黑"，不是胶带
        out_pts.append(base[k] + nrm * pos_out)
        in_pts.append(base[k] + nrm * pos_in)
    return (np.asarray(out_pts, np.float64).reshape(-1, 2),
            np.asarray(in_pts, np.float64).reshape(-1, 2))


def fit_line(points):
    """总体最小二乘直线拟合，返回过 (p0) 与方向 p1 的两点式；点不足返回 None。"""
    p = np.asarray(points, np.float32).reshape(-1, 2)
    if p.shape[0] < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(p, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    if not np.isfinite((vx, vy, x0, y0)).all() or (abs(vx) < 1e-9 and abs(vy) < 1e-9):
        return None
    return (float(x0), float(y0)), (float(x0 + vx), float(y0 + vy))


def quad_from_lines(lines):
    """四条边线两两求交 → 四角。第 i 个角 = 第 i-1 条边与第 i 条边的交点。"""
    if lines is None or len(lines) != 4 or any(l is None for l in lines):
        return None
    pts = []
    for i in range(4):
        p = line_cross(lines[i - 1][0], lines[i - 1][1], lines[i][0], lines[i][1])
        if p is None:
            return None
        pts.append(p)
    q = np.asarray(pts, np.float32)
    if not np.isfinite(q).all():
        return None
    return order_quad(q)


def refine_frame(gray, quad, samples=14, span=6.0, inward=30.0, rise=10.0, step=0.5):
    """粗四边形 → (外框四角, 内框四角, 内沿命中点数)。任一框凑不齐四边返回 None。"""
    q = order_quad(quad)
    c = q.mean(axis=0)
    outer_lines, inner_lines = [], []
    inner_hits = 0
    for i in range(4):
        po, pi = frame_edge_points(gray, q[i], q[(i + 1) % 4], c,
                                   samples, span, inward, rise, step)
        inner_hits += int(pi.shape[0])
        lo, li = fit_line(po), fit_line(pi)
        if lo is not None:
            outer_lines.append(lo)
        if li is not None:
            inner_lines.append(li)
    return quad_from_lines(outer_lines), quad_from_lines(inner_lines), inner_hits


# ------------------------------------------------------------------ 单应
def _normalize_points(points):
    """Hartley 归一化：重心到原点、平均距离到 √2。返回 (T, T⁻¹, 归一化点)。

    不做这一步，8×8 的条件数会到 1e5 量级，像素坐标的舍入误差被放大成可见偏差。
    """
    p = np.asarray(points, np.float64).reshape(-1, 2)
    center = p.mean(axis=0)
    dist = float(np.sqrt(((p - center) ** 2).sum(axis=1)).mean())
    scale = math.sqrt(2.0) / max(dist, 1e-9)
    t = np.array([[scale, 0.0, -scale * center[0]],
                  [0.0, scale, -scale * center[1]],
                  [0.0, 0.0, 1.0]], np.float64)
    t_inv = np.array([[1.0 / scale, 0.0, center[0]],
                      [0.0, 1.0 / scale, center[1]],
                      [0.0, 0.0, 1.0]], np.float64)
    return t, t_inv, (p - center) * scale


def _solve8(a, b):
    """8×8 高斯消元（列主元）。a、b 不被改动；奇异返回 None。"""
    m = np.concatenate([np.asarray(a, np.float64), np.asarray(b, np.float64).reshape(8, 1)],
                       axis=1)
    for col in range(8):
        pivot = int(np.argmax(np.abs(m[col:, col]))) + col
        if abs(m[pivot, col]) < 1e-12:
            return None
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]
        m[col] /= m[col, col]
        for row in range(8):
            if row != col and m[row, col] != 0.0:
                m[row] -= m[row, col] * m[col]
    return m[:, 8]


def solve_homography(src, dst):
    """图像点 src → 平面点 dst 的单应矩阵 (3×3)。

    8 个自由度：每对点给 2 个方程，4 对点正好 8×8，高斯消元；多于 4 对时
    正规方程 AᵀA h = Aᵀb 仍是 8×8，同一套消元，白捡最小二乘。
    两侧都先归一化，最后把归一化变换剥掉。
    """
    s = np.asarray(src, np.float64).reshape(-1, 2)
    d = np.asarray(dst, np.float64).reshape(-1, 2)
    n = int(min(len(s), len(d)))
    if n < 4:
        return None
    s, d = s[:n], d[:n]
    ts, ts_inv, sn = _normalize_points(s)
    td, td_inv, dn = _normalize_points(d)
    rows = np.zeros((2 * n, 8), np.float64)
    rhs = np.zeros(2 * n, np.float64)
    for k in range(n):
        u, v = sn[k]
        x, y = dn[k]
        rows[2 * k] = (u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v)
        rows[2 * k + 1] = (0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v)
        rhs[2 * k] = x
        rhs[2 * k + 1] = y
    h = _solve8(rows, rhs) if n == 4 else _solve8(rows.T @ rows, rows.T @ rhs)
    if h is None:
        return None
    hn = np.array([[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]], np.float64)
    mat = td_inv @ hn @ ts
    if not np.isfinite(mat).all() or abs(mat[2, 2]) < 1e-12:
        return None
    return mat / mat[2, 2]


def apply_homography(mat, points):
    """把 (N,2) 点按 3×3 单应投影过去。"""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    q = np.concatenate([p, np.ones((p.shape[0], 1))], axis=1) @ np.asarray(mat, np.float64).T
    w = q[:, 2:3]
    return q[:, :2] / np.where(np.abs(w) < 1e-12, 1e-12, w)


def invert_homography(mat):
    """3×3 单应的解析逆（伴随矩阵 / 行列式）。奇异返回 None。"""
    m = np.asarray(mat, np.float64)
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        return None
    inv = np.array([[e * i - f * h, c * h - b * i, b * f - c * e],
                    [f * g - d * i, a * i - c * g, c * d - a * f],
                    [d * h - e * g, b * g - a * h, a * e - b * d]], np.float64) / det
    return inv / inv[2, 2] if abs(inv[2, 2]) > 1e-12 else inv


def plane_quad(width, height):
    """纸面模型矩形（中心在原点、y 向下），顺序与 order_quad 一致：[左上,右上,右下,左下]。"""
    hw, hh = float(width) * 0.5, float(height) * 0.5
    return np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], np.float32)


# ------------------------------------------------------------------ 极坐标采样
def sample_polar(gray, plane_to_image, n_theta, n_r, r_min, r_max):
    """在平面坐标里按同心圆采样，返回 (半径数组, 灰度矩阵 [n_r, n_theta])。

    圆必须在平面上才是等角度的；直接在图像上画圆会被透视压成椭圆。
    这里不生成正视图像，只把采样点逆变换回原图取值 —— 省掉整幅 warp。
    """
    n_r = max(2, int(n_r))
    n_theta = max(4, int(n_theta))
    step = (float(r_max) - float(r_min)) / (n_r - 1)
    radii = float(r_min) + step * np.arange(n_r)
    ang = np.linspace(0.0, 2.0 * math.pi, n_theta, endpoint=False)
    rr, aa = np.meshgrid(radii, ang, indexing='ij')
    pts = np.stack([rr.ravel() * np.cos(aa.ravel()),
                    rr.ravel() * np.sin(aa.ravel())], axis=1)
    img = apply_homography(plane_to_image, pts).reshape(n_r, n_theta, 2)
    return radii, bilinear(gray, img[..., 0], img[..., 1])


def ring_report(radii, samples, min_contrast=6.0, smooth=9):
    """从极坐标采样读同心环：条数、半径、对比度、间距一致性。

    先减自身滑动平均（高通），把"纸面整体明暗"和"胶带厚边"压掉，只留 1~2px 的细环；
    再找局部极小并抛物线插值。**只做校验**：这里输出的半径不参与靶心定位，
    因此不需要全局搜索圆心，也就不存在"拟合残差很小但位置错了"的问题。
    """
    r = np.asarray(radii, np.float64)
    prof = np.asarray(samples, np.float64).mean(axis=1)
    if r.size < 12 or prof.size != r.size:
        return dict(count=0, radii=[], contrasts=[], spacing_cv=1.0, contrast=0.0)
    k = int(max(3, int(smooth) | 1))
    k = min(k, r.size - 2)
    base = np.convolve(prof, np.ones(k) / float(k), mode='same')
    hp = prof - base
    peaks = []
    for i in range(2, prof.size - 2):
        if hp[i] < hp[i - 1] and hp[i] <= hp[i + 1] and -hp[i] >= float(min_contrast):
            a, b, c = hp[i - 1], hp[i], hp[i + 1]
            den = a - 2.0 * b + c
            off = 0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0
            step = float(r[1] - r[0])
            peaks.append((float(r[i] + np.clip(off, -1.0, 1.0) * step), float(-b)))
    if not peaks:
        return dict(count=0, radii=[], contrasts=[], spacing_cv=1.0, contrast=0.0)
    hits = [p[0] for p in peaks]
    contrasts = [p[1] for p in peaks]
    cv = 1.0
    if len(hits) >= 3:
        gaps = np.diff(hits)
        mean = float(gaps.mean())
        cv = float(gaps.std() / mean) if mean > 1e-9 else 1.0
    return dict(count=len(hits), radii=hits, contrasts=contrasts,
                spacing_cv=cv, contrast=float(max(contrasts)))


# ------------------------------------------------------------------ 胶带带候选
def quad_mix(outer, inner, t):
    """两框对应角做线性混合：t=0 外框、t=1 内框、0.5 带中线（近似）。"""
    o = np.asarray(outer, np.float32).reshape(4, 2)
    i = np.asarray(inner, np.float32).reshape(4, 2)
    return o * (1.0 - float(t)) + i * float(t)


def band_evidence(gray, outer, inner):
    """胶带带证据：带中线灰度 vs 纸面内区灰度，返回 (带灰, 纸灰, 纸-带)。"""
    band = quad_mix(outer, inner, 0.5)
    pts = []
    for i in range(4):
        for t in (0.25, 0.5, 0.75):
            pts.append(band[i] + t * (band[(i + 1) % 4] - band[i]))
    pts = np.asarray(pts, np.float32)
    band_g = float(bilinear(gray, pts[:, 0], pts[:, 1]).mean())
    paper = quad_mix(outer, inner, 1.0)
    c = paper.mean(axis=0)
    v = paper[2] - paper[0]
    probe = c + np.array([[0.0, 0.0], [v[0] * 0.2, 0.0], [-v[0] * 0.2, 0.0],
                          [0.0, v[1] * 0.2], [0.0, -v[1] * 0.2]], np.float32)
    paper_g = float(bilinear(gray, probe[:, 0], probe[:, 1]).mean())
    return band_g, paper_g, paper_g - band_g


def find_frame_candidates(gray, binary, min_area, max_area_ratio, min_aspect, max_aspect,
                          min_contrast, samples=14, span=6.0, inward=30.0, rise=10.0,
                          step=0.5, max_candidates=8):
    """从二值图提"胶带带"候选：轮廓 → 最小外接矩形 → 内外框精修 → 灰度对比度打分。

    为什么用最小外接矩形而不是 approxPolyDP：胶带内角是圆的、边常被遮挡，
    多边形逼近永远凑不出 4 个顶点（实测两档真机帧 eps 1%~8% 全为 0）；
    外接矩形只要求"四个方向上取到极值点"，缺一个角依然成立。
    记账等式：contours = area + shape + aspect + frame + candidates
    """
    h, w = binary.shape[:2]
    total = float(h * w)
    contours = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    stats = dict(contours=len(contours), area=0, shape=0, aspect=0, frame=0, candidates=0)
    prelim = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < float(min_area) or area > total * float(max_area_ratio):
            stats["area"] += 1
            continue
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), ang = rect
        if min(rw, rh) < 12.0:
            stats["shape"] += 1
            continue
        aspect = max(rw, rh) / max(1.0, min(rw, rh))
        if aspect < float(min_aspect) or aspect > float(max_aspect):
            stats["aspect"] += 1
            continue
        prelim.append((float(area), aspect, rect))
    # 精修是唯一的重活：按面积取前若干个，避免病态画面把预算耗在小矩形上。
    prelim.sort(key=lambda p: -p[0])
    out = []
    for area, aspect, rect in prelim[:int(max_candidates)]:
        outer, inner, hits = refine_frame(gray, cv2.boxPoints(rect), samples, span,
                                          inward, rise, step)
        if outer is None or inner is None:
            stats["frame"] += 1
            continue
        outer_area = abs(float(cv2.contourArea(outer.astype(np.float32))))
        inner_area = abs(float(cv2.contourArea(inner.astype(np.float32))))
        if outer_area <= 1.0 or not (0.25 <= inner_area / outer_area <= 0.95):
            stats["frame"] += 1
            continue
        band_g, paper_g, contrast = band_evidence(gray, outer, inner)
        if contrast < float(min_contrast):
            stats["frame"] += 1
            continue
        x, y, bw, bh = cv2.boundingRect(outer.astype(np.float32))
        out.append(dict(outer=outer, inner=inner, score=float(contrast), area=area,
                        aspect=float(aspect), bbox=(x, y, bw, bh),
                        band_gray=band_g, paper_gray=paper_g, inner_hits=int(hits)))
        stats["candidates"] += 1
    out.sort(key=lambda c: c["score"], reverse=True)
    return out[:int(max_candidates)], stats


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
