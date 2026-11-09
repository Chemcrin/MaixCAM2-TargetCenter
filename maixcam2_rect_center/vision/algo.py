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
    "blur_frame", "edge_map", "deviation",
    "bilinear", "frame_edge_points", "fit_line", "quad_from_lines", "refine_frame",
    "solve_homography", "apply_homography", "invert_homography", "plane_quad",
    "quad_mix", "band_evidence",
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


def edge_map(gray, low, high, close_kernel):
    """Canny 边缘 + 闭运算：把细边连成能围出区域的轮廓。

    阈值就是"过滤纸面图案"的旋钮：调高，浅色的印刷环直接消失，只剩胶带那种硬边；
    调低，环会变成一堆小碎轮廓——它们面积小，后面按面积一筛就没了。
    high <= 0 表示按中位数自动定阈值，适应曝光变化。
    """
    src = gray.astype(np.uint8, copy=False)
    if high <= 0:
        med = float(np.median(src))
        low = int(max(10.0, 0.66 * med))
        high = int(min(255.0, 1.33 * med))
    e = cv2.Canny(src, int(low), int(high))
    k = int(close_kernel)
    if k >= 3:
        k |= 1
        e = cv2.morphologyEx(e, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    return e


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


def frame_edge_points(gray, a, b, center, samples=10, span=6.0, inward=30.0,
                      rise=10.0, step=0.5):
    """沿一条边 a→b 采样，返回 (外沿点, 内沿点) 两组亚像素坐标。

    判据是"胶带是一段连续暗带"，不是"哪里有梯度"：
      外沿 = 从框外往框内走，第一段能持续 4 个采样的暗区起点；
      内沿 = 这段暗区结束、回到纸面亮度的位置。
    遮挡物（灰斑、阴影）比纸暗但远不如胶带黑，会被"暗带均值 < 0.6×亮区均值"挡掉。

    实现上整批算：一次取 (samples × 剖面长度) 的灰度矩阵，再对行做向量化判断。
    早期版本逐行 numpy 调用，在板上因为调用开销把单帧拖到 1.6 秒——这里必须整批。
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

    offs = np.arange(span, -inward - 1e-9, -step)
    m = int(offs.size)
    zero = int(round(span / step))
    ts = np.linspace(0.15, 0.85, int(samples))
    base = a[None, :] + ts[:, None] * (b - a)[None, :]
    gx = base[:, 0:1] + nrm[0] * offs[None, :]
    gy = base[:, 1:2] + nrm[1] * offs[None, :]
    prof = bilinear(gray, gx, gy)                 # (samples, m) 一次取完
    n = prof.shape[0]

    lo = prof.min(axis=1)
    hi = prof.max(axis=1)
    ok = (hi - lo) >= float(rise)
    if not np.any(ok):
        return np.empty((0, 2)), np.empty((0, 2))
    level = lo + 0.40 * (hi - lo)

    dark = prof < level[:, None]
    pad = np.zeros((n, 1), bool)
    edge = np.diff(np.concatenate([pad, dark, pad], axis=1).astype(np.int8), axis=1)
    started = edge == 1
    ended = edge == -1
    cols = np.arange(edge.shape[1])
    rows = np.arange(n)

    cost = np.where(started, np.abs(cols[None, :] - zero), 10 ** 6)
    s0 = cost.argmin(axis=1)
    valid = cost[rows, s0] < 10 ** 6
    ecost = np.where(ended & (cols[None, :] > s0[:, None]), cols[None, :], 10 ** 6)
    s1 = ecost.argmin(axis=1)
    valid &= ecost[rows, s1] < 10 ** 6
    valid &= (s0 < m - 2) & (s1 < m - 2)
    valid &= (s1 - s0) >= 4
    valid &= ok
    if not np.any(valid):
        return np.empty((0, 2)), np.empty((0, 2))

    r = np.flatnonzero(valid)
    pv = prof[r]
    lv = level[r]
    s0v, s1v = s0[r], s1[r]
    rr = np.arange(r.size)

    i0 = np.clip(s0v - 1, 0, m - 1)
    a0, b0 = pv[rr, i0], pv[rr, s0v]
    den0 = b0 - a0
    t0 = np.where(np.abs(den0) > 1e-6,
                  np.clip((lv - a0) / np.where(np.abs(den0) > 1e-6, den0, 1.0), 0.0, 1.0), 0.0)
    pos_out = offs[i0] + t0 * (offs[s0v] - offs[i0])
    pos_out = np.where(s0v > 0, pos_out, float(offs[0]))

    i1 = np.clip(s1v - 1, 0, m - 1)
    a1, b1 = pv[rr, i1], pv[rr, s1v]
    den1 = b1 - a1
    t1 = np.where(np.abs(den1) > 1e-6,
                  np.clip((lv - a1) / np.where(np.abs(den1) > 1e-6, den1, 1.0), 0.0, 1.0), 0.0)
    pos_in = offs[i1] + t1 * (offs[s1v] - offs[i1])

    csum = np.concatenate([np.zeros((r.size, 1)), np.cumsum(pv, axis=1)], axis=1)
    dark_cnt = np.maximum(1, s1v - s0v)
    bright_cnt = np.maximum(1, m - s1v)
    dark_mean = (csum[rr, s1v] - csum[rr, s0v]) / dark_cnt
    bright_mean = (csum[rr, m] - csum[rr, s1v]) / bright_cnt

    keep = (bright_mean >= 40.0) & (dark_mean <= 0.60 * bright_mean) & ((pos_out - pos_in) >= 2.0)
    if not np.any(keep):
        return np.empty((0, 2)), np.empty((0, 2))
    sel = base[r][keep]
    out_pts = sel + nrm[None, :] * pos_out[keep][:, None]
    in_pts = sel + nrm[None, :] * pos_in[keep][:, None]
    return out_pts, in_pts


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


def find_frame_candidates(gray, edges, min_area, max_area_ratio, min_aspect, max_aspect,
                          min_contrast, samples=14, span=6.0, inward=30.0, rise=10.0,
                          step=0.5, max_candidates=8):
    """从二值图提"胶带带"候选：轮廓 → 最小外接矩形 → 内外框精修 → 灰度对比度打分。

    为什么用最小外接矩形而不是 approxPolyDP：胶带内角是圆的、边常被遮挡，
    多边形逼近永远凑不出 4 个顶点（实测两档真机帧 eps 1%~8% 全为 0）；
    外接矩形只要求"四个方向上取到极值点"，缺一个角依然成立。
    记账等式：contours = area + shape + aspect + frame + candidates
    """
    h, w = edges.shape[:2]
    total = float(h * w)
    contours = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
    stats = dict(contours=len(contours), area=0, shape=0, aspect=0, frame=0,
                 candidates=0, capped=0)
    # 第一遍只做最便宜的 boundingRect。边缘图上轮廓可达 300+，
    # 每个都算 minAreaRect 会在板上把单帧拖到接近一秒。
    prelim = []
    for cnt in contours:
        bx, by, bw, bh = cv2.boundingRect(cnt)
        box_area = float(bw) * float(bh)
        if min(bw, bh) < 12 or box_area < float(min_area) or box_area > total * float(max_area_ratio):
            stats["area"] += 1
            continue
        prelim.append((box_area, cnt))
    prelim.sort(key=lambda p: -p[0])
    stats["capped"] = max(0, len(prelim) - int(max_candidates))
    out = []
    for box_area, cnt in prelim[:int(max_candidates)]:
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), ang = rect
        if min(rw, rh) < 12.0:
            stats["shape"] += 1
            continue
        aspect = max(rw, rh) / max(1.0, min(rw, rh))
        if aspect < float(min_aspect) or aspect > float(max_aspect):
            stats["aspect"] += 1
            continue
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
        out.append(dict(outer=outer, inner=inner, score=float(contrast), area=box_area,
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
