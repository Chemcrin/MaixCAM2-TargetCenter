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
    "downscale", "auto_level", "dark_mask", "adaptive_mask", "components",
    "frame_candidates",
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


def adaptive_mask(small, block=31, c=12.0):
    """自适应二值：比"局部均值 − C"暗的像素置 255。

    为什么不用全局阈值：框体的绝对灰度随曝光漂（白墙场景 ~110，暗室 ~50），
    全局阈值永远在"框没变暗"和"背景影子被算进来"之间摇摆。
    自适应只回答"这块是不是比它周围暗"—— 框夹在亮纸和亮墙之间，天然成立；
    大片均匀的暗背景局部均值和它一样，不会被算成暗。
    block 必须大于框带宽（半分辨率下框带 ~10px，取 31），否则框会被均值吃掉。
    """
    k = int(block) | 1
    return cv2.adaptiveThreshold(small, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY_INV, k, float(c))


def components(mask):
    """连通域统计：一次调用拿到所有块的 (x, y, w, h, 面积) 与质心。

    比 findContours + 逐轮廓 boundingRect 快一个数量级 —— 板上每次 Python 级调用
    约 1ms，几百个轮廓就是几百毫秒，而这里只有一次。
    """
    n, _lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return np.zeros((0, 5), np.int32), np.zeros((0, 2), np.float64)
    return stats[1:], cent[1:]


def frame_candidates(stats, cent, mask, gray, level, min_area, max_area_ratio,
                     min_aspect, max_aspect, min_fill, max_fill, min_contrast,
                     rounds, probe=8):
    """挑"能从框心扫出一个内框、且框内明显比框体亮"的连通域。

    三道门：形状（面积/长宽比/填充率） → **能扫出内框** → **框内比框体亮**。
      · "能扫出内框"：从框心向四个方向找第一个"亮→暗"的跨越（见 `paper_rect`）。
        这一步对倾斜免疫——只要结构是"暗带围住一块亮区"，扫线必然四向命中；
        而窗棂、柜门缝、洞洞板这种稀疏暗结构扫不出一个占外接框大半的内框。
        ★ 早期用"外接框四条边压暗带"当判据，靶纸一倾斜（约 10°）就整帧漏检，
        因为斜框的外接框边缘大部分是亮背景。换成扫内框后不再依赖正置假设。
      · "框内比框体亮"：区分"白纸压在深色板/胶带上"与"同灰度的暗背景"。
        对比度取 内框灰度 − 框体灰度（框体=该连通域的暗像素均值），旋转无关。
    返回**所有被扫描过的候选**（带 ok 标记与各项实测值），由调用方决定如何排序 ——
    排序规则属于业务，不属于算法层。
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
    out = []
    for i in idx:
        x0, y0 = int(sx[i]), int(sy[i])
        w0, h0 = int(bw[i]), int(bh[i])
        inner = paper_rect(gray, (x0, y0, w0, h0), level, rounds)
        if inner is None:
            continue                     # 扫不出内框 → 不是"暗带围住亮区"的结构
        ix0, iy0, ix1, iy1 = inner
        rw = (ix1 - ix0) / max(1.0, float(w0))
        rh = (iy1 - iy0) / max(1.0, float(h0))
        if not (0.40 <= rw <= 0.98 and 0.40 <= rh <= 0.98):
            continue                     # 内框太小/太偏：扫到的是内部某个暗点
        crop = gray[y0:y0 + h0, x0:x0 + w0]
        body = crop[mask[y0:y0 + h0, x0:x0 + w0] > 0]
        comp_mean = float(body.mean()) if body.size else float(crop.mean())
        paper = float(gray[iy0:iy1, ix0:ix1].mean())
        contrast = max(0.0, paper - comp_mean)
        ok = contrast >= float(min_contrast)
        out.append(dict(x=x0, y=y0, w=w0, h=h0, area=float(area[i]),
                        aspect=float(aspect[i]), fill=float(fill[i]),
                        edge=min(rw, rh), paper=paper, body=comp_mean,
                        contrast=contrast, ok=ok))
    return out



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


def _outer_dark(seg, level):
    """一维剖面里最外圈暗游程的起点（相对 seg 起点的偏移）；没有暗则 None。

    seg 从框心出发向外。暗游程按"相邻索引差 > 2"分组（1~2px 的反光断口不算新游程），
    最后一组就是最靠外的暗带 —— 对"印刷环嵌在纸面里"的结构，它必然是框体本身，
    其起点即纸边。
    """
    idx = np.flatnonzero(seg < level)
    if idx.size == 0:
        return None
    if idx.size == 1:
        return int(idx[0])
    gap = np.flatnonzero(np.diff(idx) > 2)
    return int(idx[gap[-1] + 1]) if gap.size else int(idx[0])


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
        # 沿每个方向取"最外圈暗带的内侧边界"= 纸边。
        # 不能取第一个暗点：纸面上印的同心环也是暗的，第一下就撞在环上，
        # 内框会缩成一个点；也不能取最后一个暗像素：那会跑到框的外沿。
        # 正确做法是按暗游程分组，取最后一组（最靠外的那圈）的起点。
        # 某个方向完全扫不到暗（纸边贴着外接框）就退回用外接框那条边：
        # 缺一角不该让整帧失败，缺的那条边本来就在外接框上。
        # 比较电平用这条线自己的亮暗中点（p10/p90 的均值），不依赖全局阈值 ——
        # 框的绝对灰度随曝光漂，但"框比纸暗"这件事不变。
        lv_r = 0.5 * (float(np.percentile(row, 10)) + float(np.percentile(row, 90)))
        lv_c = 0.5 * (float(np.percentile(col, 10)) + float(np.percentile(col, 90)))
        step = _outer_dark(row[i:], lv_r)
        right = x + w - 1 if step is None else x + i + step
        step = _outer_dark(row[:i + 1][::-1], lv_r)
        left = x if step is None else x + i - step
        step = _outer_dark(col[j:], lv_c)
        bottom = y + h - 1 if step is None else y + j + step
        step = _outer_dark(col[:j + 1][::-1], lv_c)
        top = y if step is None else y + j - step
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
