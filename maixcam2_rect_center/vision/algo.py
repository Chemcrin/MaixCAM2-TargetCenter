"""② 算法层：纯函数 —— 输出只由入参决定，无状态、无 I/O、不读 config、不打日志。

四层之一，只有三类内容：

    几何  line_cross / quad_diagonal_center（对角线交点，纯 Python）
    图像  downscale / auto_level / dark_mask / close_mask / components
          prefilter_boxes / ring_hole / hole_quad / outer_from_inner
    偏差  deviation（靶心 − 光斑，纯算术）

链路核心（为什么这样组织，每一条都对应板上的真实教训）：

1. **板上一次 Python 级 numpy/cv2 调用 ≈ 1ms，单帧耗时 ≈ 调用次数**（不是像素数）。
   所以热路径只有两类写法：整批一次的大图 cv2 调用（阈值/闭运算/连通域/轮廓），
   以及 `tolist()` 之后的小数组**纯 Python** 循环（选块/投影/拟线/求交）。
   v9 实测：小数组的向量化 numpy 与循环同价，却多付每次 ~1ms 的调用开销。
2. **黑胶带环 = 一块暗带围出一个亮洞**。`ring_hole` 用 RETR_CCOMP 的两层层级
   拿"环外边界 + 洞（纸面）"：洞洞板（许多小洞）、门框立柱（没有洞）被结构性排除；
   洞里的印刷图案是环的**兄弟**轮廓而非子轮廓，天然不参与判据。
3. **中心 = 洞四边拟线 → 四线交角 → 对角线交点**。矩形（及其透视像）的对角线交点
   在任意视角下都对应真靶心：正放、旋转、斜视（梯形）共用同一条链路，无需单应。
   轮廓用 CHAIN_APPROX_NONE 取密集边点，保证每条边有足够的点做最小二乘。
4. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "line_cross", "quad_diagonal_center",
    "downscale", "auto_level", "dark_mask", "close_mask", "components",
    "prefilter_boxes", "ring_hole", "hole_quad", "outer_from_inner",
    "deviation",
]

# ================================================================== 纯函数
# ------------------------------------------------------------------ 几何
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


# ------------------------------------------------------------------ 预处理
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

    取 90 分位当亮面基准，阈值 = ratio × 它：纸面随曝光变暗时阈值同步下移，不会失效。
    分位数只在 1/16 的像素上算（步长 4），两次 numpy 调用就够。
    """
    sub = small[::4, ::4]
    lo = float(np.percentile(sub, lo_pct))
    hi = float(np.percentile(sub, hi_pct))
    return float(np.clip(ratio * hi, max(float(floor), lo + 15.0), float(ceil)))


def dark_mask(small, level):
    """暗于 level 的像素置 255。胶带、门缝、阴影都进来，靠后面的结构门筛掉。"""
    return cv2.threshold(small, float(level), 255, cv2.THRESH_BINARY_INV)[1]


_CLOSE_KERNELS = {}


def close_mask(mask, size):
    """闭运算：把运动模糊造成的细小断带桥接起来（size<3 时原样返回）。

    核缓存进模块级 dict：核只跟 size 有关，逐帧重建纯浪费一次调用。
    """
    k = int(size)
    if k < 3:
        return mask
    k |= 1
    kernel = _CLOSE_KERNELS.get(k)
    if kernel is None:
        kernel = np.ones((k, k), np.uint8)
        _CLOSE_KERNELS[k] = kernel
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def components(mask):
    """连通域统计：一次调用拿到所有块的 (x, y, w, h, 面积) 与质心。

    比 findContours + 逐轮廓 boundingRect 快一个数量级 —— 板上每次 Python 级调用
    约 1ms，几百个轮廓就是几百毫秒，而这里只有一次。
    """
    n, _lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return np.zeros((0, 5), np.int32), np.zeros((0, 2), np.float64)
    return stats[1:], cent[1:]


# ------------------------------------------------------------------ 候选预筛
def prefilter_boxes(stats, total, min_area, max_area_ratio, min_side,
                    bbox_lo, bbox_hi, min_fill, max_fill):
    """连通域 stats (N,5) → 通过预筛的候选 [(bbox面积, x, y, w, h), ...] 按面积降序。

    **先预筛再排序**是真机帧的生死顺序：暗背景大块（fill~0.6）、立柱（fill~0.88）
    的 bbox 面积都比靶环大，不先按形状筛掉，候选名额就被它们占光。
    预筛窗刻意宽松：bbox 边比下限到 0.45——纸面旋转 45° 时 bbox 接近正方形，
    这里误杀就再没有翻身机会；真正的形状判据在 ring_hole 的洞边比门（旋转不变）。

    stats.tolist() 一次调用 + 纯 Python 循环：小数组向量化 numpy 每次调用 ~1ms，
    循环反而便宜。
    """
    limit = float(total) * float(max_area_ratio)
    min_area = float(min_area)
    min_side = float(min_side)
    out = []
    for x, y, w, h, area in stats.tolist():
        w = int(w)
        h = int(h)
        short = w if w < h else h
        if short < min_side:
            continue
        box_area = float(w) * float(h)
        if box_area < min_area or box_area > limit:
            continue
        long_ = w if w > h else h
        aspect = long_ / short
        if aspect < float(bbox_lo) or aspect > float(bbox_hi):
            continue
        fill = float(area) / box_area
        if fill < float(min_fill) or fill > float(max_fill):
            continue
        out.append((box_area, int(x), int(y), w, h))
    out.sort(key=lambda t: -t[0])
    return out


# ------------------------------------------------------------------ 环-洞结构
def ring_hole(gray, box, margin, min_hole, max_hole, aspect_lo, aspect_hi,
              band_cover, min_contrast, want_brightness=True, close_size=3,
              want_mask=False):
    """在 gray 的 box±margin 裁剪图里找「暗带环 + 它围出的亮洞」，结构判据全在这里。

    阈值在裁剪图内做 **Otsu 自动分割**（暗带|亮面天然双峰），不做任何全局假设：
    真机帧实测过"胶带灰度 118 比全局分位阈值 116 还亮"的翻车——纸面不一定是最亮的，
    分位数会跟着背景 composition 漂。Otsu 在候选的小图里永远面对同一道二选一：
    暗的胶带 vs 亮的面，两峰离得远时分得准；两峰糊在一起时结构门自然拒绝。

    层级语义（RETR_CCOMP 只有两层）：
        顶层（parent==-1）  = 各暗块的**外边界**；
        第二层（某顶层的 child）= 该暗块围出的**洞**。
        洞里的印刷图案/红点是又一层暗块 → 仍是**顶层**，是环的兄弟而非儿子，
        所以"环的直接子轮廓"恰好就是纸洞，不会被洞内图案干扰。

    判据（任一不过即返回 None，调用方拿到的就是"这块不是靶纸"）：
        a. 环 = 顶层中 bbox 面积最大者（裁剪图本来就是候选块的位置）；
        b. 环的直接子轮廓中面积最大者 = 洞。真机教训：暗框常与暗色背景连成
           mega-blob（边框 35、墙 75，都低于阈值），此时 blob 的 bbox 远大于洞
           ——所以判据不拿 bbox 作参照，靠下面 c/d 的绝对尺寸与洞周暗带封闭性；
        c. 洞 minAreaRect：长边 ∈ [min_hole, max_hole]（定距下靶纸像素尺寸已知，
           挡门框这类巨型假框与印刷环这类微型假框）、边比 ∈ [aspect_lo, aspect_hi]
           （下限杀正方形/圆洞，上限给透视压缩留余量；旋转不变）；
        d. 洞周暗带封闭性：洞边界向外一圈（厚度 ≈ 0.06×洞短边）必须基本是暗的
           （暗占比 ≥ band_cover）。胶带环、显示器边框都过；印刷圆环线（3px 细线）
           和"亮区被细丝勉强围住"的结构过不去；
        e. want_brightness 时：洞内灰度均值 − Otsu 阈值 ≥ min_contrast（洞必须是亮纸面）。

    返回 dict（坐标系 = 裁剪图局部，origin 带回全图偏移）：
        hole      洞轮廓点（纯 Python list，供 hole_quad 拟线）
        rect      洞的 minAreaRect ((cx,cy),(w,h),angle)
        aspect    洞边比（长/短）
        hole_area / outer_area / hole_perim   面积与周长（带宽 = (外-洞)/周长）
        paper / contrast    洞内均值灰度与（均值 − Otsu 阈值），want_brightness=False 时为 None
        level     本块实际使用的 Otsu 阈值（诊断用）
        box       环外边界的 bbox（裁剪图局部）
        origin    裁剪原点（全图坐标）
        mask      want_mask=True 时返回二值图（调试用）
    """
    h, w = gray.shape[:2]
    x, y, bw, bh = (int(v) for v in box)
    m = int(margin)
    x0 = max(0, x - m)
    y0 = max(0, y - m)
    x1 = min(w, x + bw + m)
    y1 = min(h, y + bh + m)
    if x1 - x0 < 16 or y1 - y0 < 16:
        return None
    crop = gray[y0:y1, x0:x1]
    otsu, mask = cv2.threshold(crop, 0, 255,
                               cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    level = float(otsu)
    mask = close_mask(mask, close_size)
    res = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    contours, hierarchy = res[-2], res[-1]
    if hierarchy is None or not contours:
        return None
    hier = hierarchy[0].tolist()
    # 环 = 顶层中 bbox 面积最大者。杂乱场景顶层轮廓可达上百个，逐个
    # boundingRect 就是上百次调用；先用轮廓点数（=周长，O(1) 的 len）粗排
    # 取前 8，再只对这 8 个量 bbox —— 周长与块大小强相关，粗排足够保真。
    tops = [i for i, row in enumerate(hier) if row[3] == -1]
    if not tops:
        return None
    if len(tops) > 8:
        tops.sort(key=lambda i: -len(contours[i]))
        tops = tops[:8]
    ring_i = -1
    ring_box = None
    ring_bb = -1.0
    for i in tops:
        bx, by, bw2, bh2 = cv2.boundingRect(contours[i])
        area_bb = float(bw2) * float(bh2)
        if area_bb > ring_bb:
            ring_bb = area_bb
            ring_i = i
            ring_box = (bx, by, bw2, bh2)
    if ring_i < 0:
        return None
    hole_i = -1
    hole_area = 0.0
    j = hier[ring_i][2]
    while j != -1:
        a = abs(float(cv2.contourArea(contours[j])))
        if a > hole_area:
            hole_area = a
            hole_i = j
        j = hier[j][0]
    if hole_i < 0:
        return None
    hole = contours[hole_i]
    rect = cv2.minAreaRect(hole)
    (_rcx, _rcy), (rw, rh), _ang = rect
    long_ = max(rw, rh)
    short_ = min(rw, rh)
    if short_ <= 0 or long_ < float(min_hole) or long_ > float(max_hole):
        return None
    aspect = long_ / short_
    if aspect < float(aspect_lo) or aspect > float(aspect_hi):
        return None
    outer_area = abs(float(cv2.contourArea(contours[ring_i])))
    if outer_area <= 1.0:
        return None
    # 洞周暗带封闭性：洞向外一圈（厚度 ≈ 0.06×洞短边）必须基本是暗的。
    # 洞的存在本身只保证"紧贴边界是暗的"（它被暗块围出来），细丝围亮区、
    # 印刷细环这类结构靠这一道拦下；mega-blob（框连着暗墙）不受影响，
    # 因为检查只看洞的近邻，与 blob 多大无关。
    # 膨胀只在洞 bbox±k 的邻域里做：全幅 640×480 上开大核膨胀要几十毫秒。
    expand = max(3.0, 0.06 * short_)
    k = int(2.0 * expand) | 1
    kernel = _CLOSE_KERNELS.get(k)
    if kernel is None:
        kernel = np.ones((k, k), np.uint8)
        _CLOSE_KERNELS[k] = kernel
    bx, by, bw2, bh2 = cv2.boundingRect(hole)
    gx0 = max(0, bx - k)
    gy0 = max(0, by - k)
    gx1 = min(crop.shape[1], bx + bw2 + k)
    gy1 = min(crop.shape[0], by + bh2 + k)
    sub = mask[gy0:gy1, gx0:gx1]
    hole_mask = np.zeros(sub.shape, np.uint8)
    shifted = hole - np.array([gx0, gy0], np.int32)
    cv2.fillPoly(hole_mask, [shifted], 255)
    band = cv2.dilate(hole_mask, kernel)
    cv2.fillPoly(band, [shifted], 0)
    if float(cv2.mean(sub, band)[0]) / 255.0 < float(band_cover):
        return None
    paper = None
    contrast = None
    if want_brightness:
        # 洞内亮度：直接在洞邻域子图上求均值（hole_mask 已是子图坐标）
        paper = float(cv2.mean(crop[gy0:gy1, gx0:gx1], hole_mask)[0])
        contrast = paper - level
        if contrast < float(min_contrast):
            return None
    band_w = min((outer_area - hole_area) / max(float(cv2.arcLength(hole, True)), 1.0),
                 0.12 * short_)
    out = dict(hole=hole.reshape(-1, 2).tolist(), rect=rect, aspect=float(aspect),
               hole_area=hole_area, outer_area=outer_area, band_w=float(band_w),
               paper=paper, contrast=(None if contrast is None else float(contrast)),
               level=level, box=ring_box, origin=(x0, y0), contours=len(contours))
    if want_mask:
        out["mask"] = mask
    return out


# ------------------------------------------------------------------ 洞 → 四角
def _fit_line_dep(pts, trim_px=3.0, min_pts=3):
    """闭式最小二乘拟线：x = a·y + b（x=法向坐标，y=沿边坐标）。

    先整批拟一次，剔除法向残差 > trim_px 的外点（反光/毛边）再拟一次即收敛。
    点数不足或退化（沿边坐标全相同）返回 None；成功返回 (a, b, mean_resid)，
    mean_resid = 收敛后那批点的平均法向残差 —— "洞是不是规整四边形"的判据。
    """
    a = b = 0.0
    for _ in range(2):
        if len(pts) < min_pts:
            return None
        n = float(len(pts))
        sx = sy = sxx = syy = sxy = 0.0
        for xx, yy in pts:
            sx += xx
            sy += yy
            sxx += xx * xx
            syy += yy * yy
            sxy += xx * yy
        den = n * syy - sy * sy
        if abs(den) < 1e-9:
            return None
        a = (n * sxy - sx * sy) / den
        b = (sx - a * sy) / n
        kept = [p for p in pts if abs(p[0] - (a * p[1] + b)) <= trim_px]
        if len(kept) == len(pts):
            resid = sum(abs(p[0] - (a * p[1] + b)) for p in pts) / len(pts)
            return (a, b, resid)
        pts = kept
    resid = sum(abs(p[0] - (a * p[1] + b)) for p in pts) / len(pts)
    return (a, b, resid) if len(pts) >= min_pts else None


def _cross_uv(line_u, line_v):
    """du = a·dv + b 与 dv = c·du + d 两线求交，返回 (du, dv)；近平行返回 None。"""
    a, b = line_u[0], line_u[1]
    c, d = line_v[0], line_v[1]
    den = 1.0 - a * c
    if abs(den) < 1e-9:
        return None
    du = (a * d + b) / den
    return du, c * du + d


def _rect_quad_from(rect):
    """minAreaRect 的四角（环序，对角线 = 0-2 / 1-3）。仅作 hole_quad 的回退。"""
    return cv2.boxPoints(rect).tolist()


def _poly2_area(quad):
    """鞋带公式算 2 倍有符号面积（纯 Python），用于凸性/退化防护。"""
    total = 0.0
    n = len(quad)
    for i in range(n):
        x0, y0 = quad[i]
        x1, y1 = quad[(i + 1) % n]
        total += float(x0) * float(y1) - float(x1) * float(y0)
    return total


def hole_quad(points, rect, min_pts=8, keep=0.75, hole_area=None):
    """洞轮廓点 + minAreaRect → 内框四角（环序，对角线 = 0-2 / 1-3）。

    **全纯 Python**：热路径上小数组的 numpy 调用与循环同价，但每次多付 ~1ms。

    步骤：
      1) 把点投影到矩形两轴 (du, dv)（u 轴 = minAreaRect 的 angle 方向）；
      2) 按 |du|/hu 与 |dv|/hv 的支配关系分四条边 —— 对角线平分角把圆角弧
         自然分到两侧，随后每边只留沿边中段（|along| ≤ keep × 半长），
         圆角与角点被丢弃，剩下的全是直边点；
      3) 每边闭式最小二乘拟线（du = a·dv + b 型，斜边也不会除零）；
      4) 相邻两边求交得四角，映射回图像坐标。
    透视下每条边仍是直线，所以四线交角是精确解 —— 斜看成梯形也准。

    任一边点数不足 / 拟线失败 / 四角退化 → 回退 minAreaRect 四角
    （旋转精确、透视近似，比失败强）。返回 (quad, hits)：hits = 精拟边数
    （4 = 全精拟，0 = 回退）。
    """
    (cx, cy), (rw, rh), ang = rect
    cx = float(cx)
    cy = float(cy)
    a = math.radians(float(ang))
    ux, uy = math.cos(a), math.sin(a)
    vx, vy = -math.sin(a), math.cos(a)
    hu = max(0.5 * float(rw), 1e-3)
    hv = max(0.5 * float(rh), 1e-3)
    # CHAIN_APPROX_NONE 的洞轮廓可达上千点；投影循环是纯 Python，先抽稀到
    # ≤600 点（切片零调用），拟线精度损失亚像素级，循环时间减半以上。
    if len(points) > 600:
        step = len(points) // 600 + 1
        points = points[::step]
    sides = ([], [], [], [])          # 0:+u 1:-u 2:+v 3:-v
    for x, y in points:
        dx = float(x) - cx
        dy = float(y) - cy
        du = dx * ux + dy * uy
        dv = dx * vx + dy * vy
        if abs(du) / hu >= abs(dv) / hv:
            sides[0 if du > 0 else 1].append((du, dv))
        else:
            sides[2 if dv > 0 else 3].append((du, dv))
    lines = []
    for idx, pts in enumerate(sides):
        if idx < 2:                    # +u / -u 边：沿边 = dv，法向 = du
            pts = [(du, dv) for du, dv in pts if abs(dv) <= keep * hv]
            line = _fit_line_dep(pts, min_pts=min_pts)
        else:                          # +v / -v 边：沿边 = du，法向 = dv
            pts = [(dv, du) for du, dv in pts if abs(du) <= keep * hu]
            line = _fit_line_dep(pts, min_pts=min_pts)
        lines.append(line)
    quad = None
    if all(ln is not None for ln in lines):
        quad = []
        for ui, vi in ((0, 3), (0, 2), (1, 2), (1, 3)):
            p = _cross_uv(lines[ui], lines[vi])
            if p is None:
                quad = None
                break
            du, dv = p
            quad.append((cx + du * ux + dv * vx, cy + du * uy + dv * vy))
        if quad is not None:
            ok = abs(_poly2_area(quad)) >= 64.0
            if ok:
                for i in range(4):
                    x0, y0 = quad[i]
                    x1, y1 = quad[(i + 1) % 4]
                    if math.hypot(x1 - x0, y1 - y0) < 8.0:
                        ok = False
                        break
            # 规整性：四条边的平均拟合残差 / 短边 必须很小 —— 真纸洞是干净
            # 矩形（残差亚像素级）；柜门/墙缝围出的不规则亮区，拟线残差爆炸。
            # 纯几何判据，不含任何图案识别。
            if ok:
                wsum = rsum = 0.0
                for idx, ln in enumerate(lines):
                    n = float(len(sides[idx]))
                    wsum += n
                    rsum += ln[2] * n
                if rsum / max(wsum, 1.0) / (2.0 * min(hu, hv)) > 0.03:
                    ok = False
            # 凸性：四边叉积同号才算凸四边形
            if ok:
                sign = None
                for i in range(4):
                    ax, ay = quad[i]
                    bx, by = quad[(i + 1) % 4]
                    cx2, cy2 = quad[(i + 2) % 4]
                    c1x, c1y = bx - ax, by - ay
                    c2x, c2y = cx2 - bx, cy2 - by
                    cr = c1x * c2y - c1y * c2x
                    if abs(cr) < 1e-6:
                        ok = False
                        break
                    if sign is None:
                        sign = cr > 0
                    elif (cr > 0) != sign:
                        ok = False
                        break
            if not ok:
                quad = None
    if quad is None:
        # 回退也要过"洞像不像矩形"的门：洞面积填不满自己的 minAreaRect
        # （< 0.55）说明洞本身是 L 形/不规则亮区，回退四边形只会画出假框。
        # 真纸洞（含透视）填充率 0.85+。
        if hole_area is not None and \
                float(hole_area) < 0.55 * (2.0 * hu) * (2.0 * hv):
            return None, 0
        return _rect_quad_from(rect), 0
    return quad, 4


def outer_from_inner(quad, band_w):
    """内框四角沿"外角平分线"外推带宽 → 外框四角（纯 Python，零图像调用）。

    只用于画图与记账，不进任何判据：外框受胶带不等宽影响，中心会偏出
    半个带宽，所以靶心永远取内框的对角线交点。
    """
    quad = [(float(p[0]), float(p[1])) for p in quad]
    band_w = float(band_w)
    if band_w <= 0.0 or len(quad) != 4:
        return list(quad)
    out = []
    for i in range(4):
        px, py = quad[i]
        qx, qy = quad[(i - 1) % 4]
        rx, ry = quad[(i + 1) % 4]
        e1x, e1y = px - qx, py - qy
        e2x, e2y = rx - px, ry - py
        l1 = math.hypot(e1x, e1y)
        l2 = math.hypot(e2x, e2y)
        if l1 < 1e-6 or l2 < 1e-6:
            return list(quad)
        u1x, u1y = e1x / l1, e1y / l1
        u2x, u2y = e2x / l2, e2y / l2
        bx = -(u1x + u2x)
        by = -(u1y + u2y)
        bl = math.hypot(bx, by)
        if bl < 1e-6:
            return list(quad)
        bx /= bl
        by /= bl
        # 内角 θ = (−u1) 与 u2 的夹角；沿平分线外推 d = band_w / sin(θ/2)
        # 时两条边都恰好外移 band_w（直角时 d = band_w·√2）。
        cos_t = -(u1x * u2x + u1y * u2y)
        cos_t = max(-1.0, min(1.0, cos_t))
        sin_half = math.sqrt(max(1e-6, (1.0 - cos_t) * 0.5))
        d = band_w / sin_half
        out.append((px + bx * d, py + by * d))
    return out


# ================================================================== 偏差
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
