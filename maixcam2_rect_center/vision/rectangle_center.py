"""OpenCV 黑框识别：自适应反色二值化 → 轮廓（含内孔）→ 四层过滤 → 四点对角线交点。

组织方式：每个小功能都是**模块级函数**，输入输出只依赖参数，不读类状态；
`RectangleCenterDetector` 只做两件事——保存参数、按顺序调用它们。
这样任何一步都能单独跑、单独测（`diagnose_detect.py` 就是直接调下面这些函数）。

    to_gray_frame      校验并去掉灰度哑轴
    blur_frame         高斯模糊（可关）
    binarize_frame     自适应反色，黑胶带 → 白环
    find_quad_candidates  面积 / 四边形 / 长宽比三层，返回候选 + 各层淘汰计数
    band_score         胶带暗带证据：边界外侧是否显著暗于内侧
    select_best        在通过者里择优（有时序先验时优先靠近上一帧的那个）
    refine_quad        沿四边做直线拟合求交，把角点从"多边形顶点"提升到亚像素
    locate_result      候选 → 结果 dict（含失败归因）
    quad_diagonal_center  四点 → 对角线交点
    draw_result        结果 → BGR 画布

四条刻意的设计约束：

1. **轮廓取内孔**：`RETR_CCOMP` 同时拿到黑框环围出的纸面边界。黑框与桌面、窗框
   这类暗结构相连时，最外层轮廓会被整块吞掉，而纸面孔洞始终是闭合的——这是
   "明明看得见却检不出"的主因。
2. **形状判据用四边形自身的对边长度比**，不用正立外接框。外接框在旋转时趋近正方形、
   竖放时落到 0.7，这两种现场都会出现；对边比与朝向无关。
3. **第四层是胶带证据**：边界内侧亮、外侧暗的采样占比。它挡的是"更大的深色矩形"
   （窗框、屏幕、桌沿）——那些边界两侧亮度关系不成立。
4. **跨帧只用于择优与复锁**，不改变单帧判据：有上一帧时优先选离它近的候选，
   丢失超过一定帧数就退回纯面积择优，避免被旧位置困死。

分阶段调试：`RECT_KEEP_STAGES` 置真后，`process` 把中间结果留在 `last_stages`
（`blur` / `binary`），供 `tools/run_pc.py --stage` 导出。
"""
import time
import traceback

import cv2
import numpy as np

STAGE_OK = "full_search"
STAGE_TRACK = "track"
STAGE_NONE = "no_black_frame"


# ------------------------------------------------------------------ 几何小工具
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


def quad_side_aspect(quad):
    """四边形自身的长宽比：对边平均长度之比。

    与正立外接框的比值不同，它不随旋转变小、也不因横竖放而翻转：
    同一张 A4 无论怎么转、怎么放，都稳定落在 1.414 附近（透视会带来偏移）。
    退化（最短边 < 1px）返回 0，由调用方当作不合格。
    """
    q = np.asarray(quad, np.float64).reshape(4, 2)
    e = [float(np.linalg.norm(q[(i + 1) % 4] - q[i])) for i in range(4)]
    long_side = (e[0] + e[2]) * 0.5
    short_side = (e[1] + e[3]) * 0.5
    if min(long_side, short_side) < 1.0:
        return 0.0
    return max(long_side, short_side) / min(long_side, short_side)


def quad_side_lengths(quad):
    """四条边的长度（按输入顺序，相邻边连续）。"""
    q = np.asarray(quad, np.float64).reshape(4, 2)
    return [float(np.linalg.norm(q[(i + 1) % 4] - q[i])) for i in range(4)]


# ------------------------------------------------------------------ 预处理
def to_gray_frame(gray):
    """校验输入并去掉 (H,W,1) 哑轴。返回 (H,W) uint8。

    只做格式规整，不做任何增强——真正的灰度转换在 camera_source.to_gray。
    """
    g = np.asarray(gray)
    if g.ndim == 3 and g.shape[2] == 1:
        g = g[:, :, 0]
    if g.ndim != 2 or g.dtype != np.uint8:
        raise ValueError("process expects a uint8 grayscale image")
    return g


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


# ------------------------------------------------------------------ 三层形状过滤
def find_quad_candidates(binary, min_area, max_area_ratio, eps_ratio,
                         min_aspect, max_aspect, max_candidates=32,
                         min_fill=0.55):
    """面积 / 四边形 / 长宽比三层过滤。返回 (候选列表, 各层淘汰计数)。

    轮廓用 `RETR_CCOMP`：外轮廓与内孔一起遍历。黑框环围出的纸面是一个内孔，
    它不受外部暗结构连通的影响，是靶纸最稳定的边界。

    形状有两条路：多边形近似正好四点就用它（透视四边形，最准）；近似不出四点
    时退一步用**最小外接矩形**——胶带环有一处缺口、纸面漏气时轮廓不再是闭合四边形，
    但外接矩形仍贴着纸边，填充度足够高就采信。两条路都记 `kind`，
    下游的暗带证据会把"框住杂物"的那些挡掉。

    只负责"合不合格"，不负责"选哪个"——择优在 select_best。
    每个淘汰分支先记账再 continue，于是有一条可自检的等式：
        contours = area + quad + aspect + candidates
    """
    h, w = binary.shape[:2]
    total = float(h * w)
    contours = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[-2]
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
        if len(approx) == 4:
            quad = order_quad(approx.reshape(4, 2))
            kind = "poly"
        else:
            rect = cv2.minAreaRect(cnt)
            rw, rh = rect[1]
            if min(rw, rh) < 8.0 or rw * rh <= 0:
                stats["quad"] += 1
                continue
            fill = area / float(rw * rh)
            if fill < min_fill:
                stats["quad"] += 1
                continue
            quad = order_quad(cv2.boxPoints(rect))
            kind = "rect"
        aspect = quad_side_aspect(quad)
        if aspect < min_aspect or aspect > max_aspect:
            stats["aspect"] += 1
            continue
        out.append(dict(area=float(area), bbox=cv2.boundingRect(quad.astype(np.int32)),
                        aspect=float(aspect), quad=quad, contour=cnt, kind=kind))
        stats["candidates"] += 1
        if len(out) >= max_candidates:
            break
    return out, stats


# ------------------------------------------------------------------ 第四层：胶带证据
def band_edges(gray, quad, offset=7.0, min_contrast=10.0, per_edge=9):
    """逐条边统计"外侧显著暗于内侧"的采样占比，返回长度 4 的列表。

    无效（采样点全部越界或边长太短）的边记 None，由调用方决定怎么算。
    单独给出每条边，是为了能要求"至少三条边同时成立"——靶纸四周都有胶带，
    而屏幕里的一块深色窗口往往只有两三条边侧面恰好更暗。
    """
    q = np.asarray(quad, np.float64).reshape(4, 2)
    h, w = gray.shape[:2]
    c = q.mean(axis=0)
    per = []
    for i in range(4):
        p, r = q[i], q[(i + 1) % 4]
        e = r - p
        length = float(np.hypot(e[0], e[1]))
        if length < 8.0:
            per.append(None)
            continue
        n = np.array([-e[1], e[0]]) / length
        if float(np.dot(n, (p + r) * 0.5 - c)) < 0.0:
            n = -n
        hits = 0
        total = 0
        for t in np.linspace(0.12, 0.88, per_edge):
            base = p + t * e
            out_pt = base + n * offset
            in_pt = base - n * offset
            if not (0.0 <= out_pt[0] < w - 1 and 0.0 <= out_pt[1] < h - 1 and
                    0.0 <= in_pt[0] < w - 1 and 0.0 <= in_pt[1] < h - 1):
                continue
            total += 1
            outer = int(gray[int(out_pt[1]), int(out_pt[0])])
            inner = int(gray[int(in_pt[1]), int(in_pt[0])])
            if outer + min_contrast < inner:
                hits += 1
        per.append(hits / float(total) if total >= 3 else None)
    return per


def band_score(gray, quad, offset=7.0, min_contrast=10.0, per_edge=9):
    """边界外侧显著暗于内侧的采样点占比，范围 0~1。

    靶纸的黑胶带在纸面之外一圈：以纸边（内沿）为界，内侧是白纸、外侧是黑胶带，
    每条边的采样都能满足"外暗内亮"。窗框、屏幕、桌沿的边界两侧没有这种关系，
    得分会明显偏低——这是拦"更大的深色矩形抢锁"的那一层。

    越界的采样点直接跳过（不计入分母），有效采样不足时返回 0，
    表示"这一帧给不出证据"，由调用方按不合格处理而不是当作通过。
    """
    per = [x for x in band_edges(gray, quad, offset, min_contrast, per_edge)
           if x is not None]
    if not per:
        return 0.0
    return float(sum(per) / len(per))


def band_edge_floor(gray, quad, offset=7.0, min_contrast=10.0, per_edge=9):
    """第三条最好的边的得分。用来要求"至少三条边同时成立"。"""
    per = sorted((x for x in band_edges(gray, quad, offset, min_contrast, per_edge)
                  if x is not None), reverse=True)
    if len(per) < 3:
        return 0.0
    return float(per[2])


def band_thickness(gray, quad, probe_ratio=0.15, samples=3, rise_step=18.0):
    """边界外侧那条暗带有多厚，按边长归一。返回 (中位数, 最小, 最大, 有效边数)。

    18mm 胶带贴在 210mm 宽的纸上，厚度占边长约 8%；显示器边框、窗框、桌沿的
    边框厚度占比通常只有 1%~2%。`band_score` 只能证明"外侧比内侧暗"，
    笔记本屏幕同样满足；这一项才把"胶带"和"细边框"分开。

    **边界用灰度回升界定，不用绝对灰度阈值**：从四边形边线向外取灰度剖面，
    先找最暗处，再从那里往外找第一次显著回升，回升点就是暗带外沿。
    这样量出的是"从纸边到胶带外沿"的物理宽度，与整幅画面的曝光无关——
    上一版用绝对灰度上限，画面一亮胶带灰度越线，四条边同时量不出厚度，整帧判负。

    返回最小/最大值是为了"四边同宽"判据：同一卷胶带贴出来的四条边宽度接近，
    而显示器上、左、右、下四条边框的宽度常常相差几倍。有效边数不足 3 时
    调用方不应使用一致性判据（一条边被遮挡是常态）。
    """
    q = np.asarray(quad, np.float64).reshape(4, 2)
    h, w = gray.shape[:2]
    centre = q.mean(axis=0)
    short_side = min(quad_side_lengths(q))
    max_probe = max(10, int(probe_ratio * short_side))
    ratios = []
    for i in range(4):
        p, r = q[i], q[(i + 1) % 4]
        e = r - p
        length = float(np.hypot(e[0], e[1]))
        if length < 20.0:
            continue
        u = e / length
        n = np.array([-u[1], u[0]])
        if float(np.dot(n, (p + r) * 0.5 - centre)) < 0.0:
            n = -n
        per_edge = []
        for t in np.linspace(0.3, 0.7, samples):
            base = p + t * e
            profile = []
            for k in range(0, max_probe + 1):
                pt = base + n * k
                if not (0.0 <= pt[0] < w - 1 and 0.0 <= pt[1] < h - 1):
                    break
                profile.append(float(gray[int(pt[1]), int(pt[0])]))
            if len(profile) < 5:
                continue
            # 内侧参考：边线往里 2px，用来判断这条边外侧是不是真的更暗
            inner = base - n * 2.0
            if not (0.0 <= inner[0] < w - 1 and 0.0 <= inner[1] < h - 1):
                continue
            ref = float(gray[int(inner[1]), int(inner[0])])
            if ref < 20.0:
                continue
            kmin = int(np.argmin(profile[1:])) + 1
            vmin = profile[kmin]
            # 最暗处必须在探测窗口靠边界的那一段，且明显低于内侧，
            # 否则说明这条边外面根本没有暗带。上限按窗口比例给，胶带宽时
            # 最暗点本来就会落在窗口深处，卡太死会把近距靶纸判成无效边。
            if kmin > max(3, max_probe * 4 // 5):
                continue
            if vmin > ref - 8.0:
                continue
            end = None
            for k in range(kmin + 1, len(profile)):
                if profile[k] - profile[k - 1] >= rise_step:
                    end = k
                    break
            if end is None:
                # 没有回升：外侧一直是暗的（贴墙/贴暗背景）。这时暗带宽度
                # 无法界定，按探测上限算——只要它够宽，后面的比值门仍能过。
                end = len(profile)
            if end - 1 < 2:
                continue
            per_edge.append((end - 1) / length)
        if per_edge:
            ratios.append(float(np.median(per_edge)))
    if not ratios:
        return 0.0, 0.0, 0.0, 0
    return (float(np.median(ratios)), float(np.min(ratios)),
            float(np.max(ratios)), len(ratios))


def inner_mean(gray, quad):
    """四边形内部（外接框中央一半）的平均灰度。

    靶纸内部是白纸，即使现场偏暗也明显亮于黑胶带；显示器里的黑窗口、
    暗设备面板这类"暗区里的暗框"边界同样满足"外暗内亮"，但内部均值很低。
    这一项是最便宜的排除手段，只取一块 numpy 切片的均值，不做掩膜。
    """
    q = np.asarray(quad, np.float32).reshape(4, 2)
    x, y, w, h = cv2.boundingRect(q)
    H, W = gray.shape[:2]
    x0, x1 = max(0, x + w // 4), min(W, x + 3 * w // 4)
    y0, y1 = max(0, y + h // 4), min(H, y + 3 * h // 4)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return 0.0
    return float(gray[y0:y1, x0:x1].mean())


def filter_band(gray, candidates, min_score, offset, min_contrast,
                min_ratio=0.0, probe_ratio=0.15, min_uniform=0.0,
                min_inner=0.0, min_edge=0.0, rise_step=18.0, limit=0, debug=False):
    """按暗带得分、暗带厚度与四边厚度一致性筛候选，就地写入 band 相关字段。

    四项：得分证明"外侧确实更暗"；第三条边也成立说明四周都有胶带；
    厚度证明"暗得够宽，是胶带不是边框"；一致性证明"四边同宽"——
    显示器上、左、右、下四条边框常常差几倍。

    `limit > 0` 时只对面积最大的那些候选做证据计算：每一项都要沿边采样，
    候选多时是整帧耗时的大头，而真正的靶纸几乎总在面积前列。
    """
    pool = candidates
    if limit > 0 and len(pool) > limit:
        pool = sorted(pool, key=lambda c: c["area"], reverse=True)[:limit]
    dropped = len(candidates) - len(pool)
    kept = []
    diag = []
    for cand in pool:
        quad = cand["quad"]
        edges = [x for x in band_edges(gray, quad, offset, min_contrast) if x is not None]
        if not edges:
            continue
        score = float(sum(edges) / len(edges))
        cand["band"] = float(score)
        if score < min_score:
            continue
        if min_edge > 0.0:
            top = sorted(edges, reverse=True)
            if len(top) < 3 or top[2] < min_edge:
                continue
        if min_inner > 0.0 and inner_mean(gray, quad) < min_inner:
            continue
        if min_ratio > 0.0:
            median, low, high, edges = band_thickness(gray, quad, probe_ratio,
                                                      rise_step=rise_step)
            if median < min_ratio:
                continue
            # 一致性只在三条边以上才判：一条边被手或画面边缘遮住是常态，
            # 拿两条边的比值去要求"四边同宽"只会把正常帧判死。
            if edges >= 3 and min_uniform > 0.0 and high > 0.0:
                if low / high < min_uniform:
                    continue
            cand["band_ratio"] = float(median)
            cand["band_edges"] = int(edges)
        kept.append(cand)
    if debug:
        for cand in pool:
            diag.append("a=%d asp=%.2f s=%.2f r=%.3f e=%s i=%.0f" % (
                cand["area"], cand["aspect"], cand.get("band", 0.0),
                cand.get("band_ratio", 0.0), cand.get("band_edges", "-"),
                inner_mean(gray, cand["quad"])))
    return kept, len(candidates) - len(kept) + dropped, diag


# ------------------------------------------------------------------ 择优
def select_best(candidates, prior_center=None, prior_radius=0.0):
    """选出一个候选。

    没有先验时取面积最大者；有先验时**先**在半径内挑"最近且最大"的那个
    （距离平方除以面积取最小：目标不会瞬移，尺寸也接近），半径内一个都没有
    才退回面积最大。这样"丢锁后重新捕获"依然靠全图候选，不会被旧位置困死
    （先验由调用方按丢失帧数决定是否给）。
    """
    if not candidates:
        return None
    if prior_center is not None:
        cx, cy = float(prior_center[0]), float(prior_center[1])
        near = []
        for cand in candidates:
            dx = float(cand["quad"][:, 0].mean()) - cx
            dy = float(cand["quad"][:, 1].mean()) - cy
            if dx * dx + dy * dy <= prior_radius ** 2:
                near.append(((dx * dx + dy * dy) / max(1.0, cand["area"]), cand))
        if near:
            return min(near, key=lambda item: item[0])[1]
    return max(candidates, key=lambda c: c["area"])


# ------------------------------------------------------------------ 角点精修
def refine_quad(quad, contour, band=3.0, min_points=8, max_shift=4.0):
    """沿四条边各拟合一条直线再求交点，得到亚像素角点。

    `approxPolyDP` 的顶点落在轮廓的离散采样点上，逐帧随噪声跳动——这是"四个端点
    框不稳"的直接来源。把每条边附近的轮廓点做最小二乘拟合、再求相邻直线交点，
    顶点就由整条边的走向决定，抖动被平均掉。

    任何一条边凑不够点、或精修结果位移超过 `max_shift`，就原样返回输入四边形：
    宁可给粗角点，也不要给跑偏的角点。
    """
    src = np.asarray(quad, np.float64).reshape(4, 2)
    pts = np.asarray(contour, np.float64).reshape(-1, 2)
    if len(pts) < 12:
        return src.astype(np.float32)
    lines = []
    for i in range(4):
        p, r = src[i], src[(i + 1) % 4]
        e = r - p
        length = float(np.hypot(e[0], e[1]))
        if length < 12.0:
            return src.astype(np.float32)
        u = e / length
        d = pts - p
        t = d @ u / length
        dist = np.abs(d[:, 0] * (-u[1]) + d[:, 1] * u[0])
        sel = (t > 0.15) & (t < 0.85) & (dist < band)
        if int(sel.sum()) < min_points:
            return src.astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(pts[sel].astype(np.float32), cv2.DIST_L2,
                                     0, 0.01, 0.01).ravel()
        lines.append(((float(x0), float(y0)),
                      (float(x0) + float(vx) * 100.0, float(y0) + float(vy) * 100.0)))
    refined = []
    for i in range(4):
        cross = line_cross(lines[(i - 1) % 4][0], lines[(i - 1) % 4][1],
                           lines[i][0], lines[i][1])
        if cross is None or not np.isfinite(cross).all():
            return src.astype(np.float32)
        refined.append(cross)
    refined = np.asarray(refined, np.float64)
    if float(np.abs(refined - src).max()) > max_shift:
        return src.astype(np.float32)
    return order_quad(refined).astype(np.float32)


def refine_edges_1d(gray, quad, radius=12.0, samples=16, min_step=6.0, max_shift=6.0):
    """以上一帧的四边为基准，沿各边法向做一维灰度搜索，重新定位四条边。

    靶纸的黑胶带在纸面之外一圈，所以沿外法向灰度是"由亮转暗"：每条边取若干采样点、
    在 ±`radius` 内找最陡的下降处，再用这些点拟合直线、四线求交。

    比全图重搜便宜一个数量级；更关键的是每条边由十几个跳变点共同决定，
    顶点是连续变化的——锁定之后四个端点不再逐帧跳动。

    任一条边凑不够跳变点、或整体位移超过 `max_shift` 就返回 None，
    由调用方回退全图搜索（不拿陈旧位置硬撑）。
    """
    src = np.asarray(quad, np.float64).reshape(4, 2)
    h, w = gray.shape[:2]
    centre = src.mean(axis=0)
    offsets = np.arange(-radius, radius + 1.0)
    lines = []
    for i in range(4):
        p, r = src[i], src[(i + 1) % 4]
        e = r - p
        length = float(np.hypot(e[0], e[1]))
        if length < 20.0:
            return None
        u = e / length
        n = np.array([-u[1], u[0]])
        if float(np.dot(n, (p + r) * 0.5 - centre)) < 0.0:
            n = -n
        pts = []
        for t in np.linspace(0.12, 0.88, samples):
            base = p + t * e
            xs = base[0] + n[0] * offsets
            ys = base[1] + n[1] * offsets
            inside = (xs >= 0) & (xs < w - 1) & (ys >= 0) & (ys < h - 1)
            if int(inside.sum()) < len(offsets) * 2 // 3:
                continue
            vals = gray[ys[inside].astype(np.int32),
                        xs[inside].astype(np.int32)].astype(np.float64)
            drop = vals[1:] - vals[:-1]
            if drop.size == 0:
                continue
            k = int(np.argmin(drop))
            if drop[k] > -min_step:
                continue
            pts.append(base + n * (offsets[inside][k] + 0.5))
        if len(pts) < max(4, samples // 2):
            return None
        vx, vy, x0, y0 = cv2.fitLine(np.asarray(pts, np.float32), cv2.DIST_L2,
                                     0, 0.01, 0.01).ravel()
        lines.append(((float(x0), float(y0)),
                      (float(x0) + float(vx) * 100.0, float(y0) + float(vy) * 100.0)))
    out = []
    for i in range(4):
        cross = line_cross(lines[(i - 1) % 4][0], lines[(i - 1) % 4][1],
                           lines[i][0], lines[i][1])
        if cross is None or not np.isfinite(cross).all():
            return None
        out.append(cross)
    out = np.asarray(out, np.float64)
    if float(np.abs(out - src).max()) > max_shift:
        return None
    return order_quad(out).astype(np.float32)


def band_offset_for(quad, scale=0.03, lo=3.0, hi=10.0):
    """按目标大小给出暗带采样距离：远靶用近距、近靶用远距，避免采到胶带外面。"""
    lens = quad_side_lengths(quad)
    return float(min(hi, max(lo, scale * min(lens))))


def quad_payload(quad, band=0.0, area=None):
    """把一组四角包成与全图候选同构的 dict，便于共用成功结果组装。"""
    q = np.asarray(quad, np.float32).reshape(4, 2)
    return dict(quad=q,
                area=float(area if area is not None else abs(cv2.contourArea(q))),
                aspect=quad_side_aspect(q),
                bbox=cv2.boundingRect(q.astype(np.int32)),
                band=float(band))


# ------------------------------------------------------------------ 结果组装
def fail_result(stage, fail_stage, reason):
    """统一的失败结果。`fail_stage` 是走过的层，`reason` 是首次失败原因。"""
    return dict(valid=False, stage=stage, fail_stage=fail_stage, fail_reason=reason)


def success_result(best, stage=STAGE_OK):
    """胜出候选 → 成功结果。四点共线（对角线无交点）时退化为失败，不拿外接框中心凑数。"""
    quad = order_quad(best["quad"])
    center = quad_diagonal_center(quad)
    if center is None:
        return fail_result(STAGE_NONE, "full", "degenerate")
    return dict(valid=True, stage=stage,
                u=float(center[0]), v=float(center[1]),
                raw_u=float(center[0]), raw_v=float(center[1]),
                quad=quad,
                # 新链路只认一个框，不区分胶带内外沿；给 None 让绘图层跳过。
                inner_quad=None, detected_quad=quad,
                outer_inferred=False,
                area=best["area"], aspect=best["aspect"], bbox=best["bbox"],
                band=float(best.get("band", 0.0)),
                partial=False, refined_edges=4, weak_edges=0, clipped_edges=0,
                rect_to_image=None, circle_points=None,
                width_mm=None, height_mm=None)


def locate_result(best, stats):
    """候选与统计 → 结果 dict。失败原因按淘汰层归因，供 [perf] 直接读出。"""
    if best is not None:
        return success_result(best)
    if stats["contours"] == 0:
        reason = "no_proposal"
    elif stats["area"] == stats["contours"]:
        reason = "all_area"
    elif stats.get("band", 0) and stats["candidates"] == stats.get("band", 0):
        reason = "all_band"
    else:
        reason = "all_filtered"
    return fail_result(STAGE_NONE, "full", reason)


def finish_result(result, timestamp, image_size, started):
    """补齐时间戳、图像尺寸、耗时。"""
    result.update(captured_at=timestamp, image_size=image_size,
                  detector_ms=(time.perf_counter() - started) * 1000.0)
    return result


# ------------------------------------------------------------------ 编排
class RectangleCenterDetector:
    """只保存参数、维护跨帧先验并编排上面那些函数；本身不做像素运算。"""

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self.last_stages = {}
        self.last_stats = {}
        # 参数全部允许缺省，config 未同步时也不至于抛 AttributeError。
        self.min_area = float(getattr(config, "RECT_MIN_AREA", 300))
        self.max_area_ratio = float(getattr(config, "RECT_MAX_AREA_RATIO", 0.92))
        self.eps_ratio = float(getattr(config, "RECT_APPROX_EPS", 0.02))
        self.min_aspect = float(getattr(config, "RECT_MIN_ASPECT", 1.05))
        self.max_aspect = float(getattr(config, "RECT_MAX_ASPECT", 1.95))
        self.block_size = int(getattr(config, "RECT_BLOCK_SIZE", 31))
        self.c_const = float(getattr(config, "RECT_C", 8.0))
        self.kernel = tuple(getattr(config, "RECT_GAUSS_KERNEL", (5, 5)))
        self.max_candidates = int(getattr(config, "RECT_MAX_CANDIDATES", 32))
        self.keep_stages = bool(getattr(config, "RECT_KEEP_STAGES", False))
        self.min_band = float(getattr(config, "RECT_MIN_BAND_SCORE", 0.55))
        self.min_band_ratio = float(getattr(config, "RECT_MIN_BAND_RATIO", 0.02))
        self.min_band_uniform = float(getattr(config, "RECT_BAND_MIN_UNIFORM", 0.40))
        self.min_inner_ratio = float(getattr(config, "RECT_MIN_INNER_RATIO", 0.60))
        self.inner_floor = float(getattr(config, "RECT_MIN_INNER_FLOOR", 45.0))
        self.min_edge_score = float(getattr(config, "RECT_MIN_EDGE_SCORE", 0.60))
        self.band_probe_ratio = float(getattr(config, "RECT_BAND_PROBE_RATIO", 0.15))
        self.band_rise_step = float(getattr(config, "RECT_BAND_RISE_STEP", 18.0))
        self.band_limit = int(getattr(config, "RECT_BAND_CANDIDATES", 12))
        self.debug_band = bool(getattr(config, "RECT_DEBUG_BAND", False))
        self.band_offset = float(getattr(config, "RECT_BAND_OFFSET_PX", 7.0))
        self.band_contrast = float(getattr(config, "RECT_BAND_CONTRAST", 10.0))
        self.refine_edges = bool(getattr(config, "RECT_EDGE_REFINE", True))
        self.edge_band = float(getattr(config, "RECT_EDGE_BAND_PX", 3.0))
        self.edge_min_points = int(getattr(config, "RECT_EDGE_MIN_POINTS", 8))
        self.edge_max_shift = float(getattr(config, "RECT_EDGE_MAX_SHIFT_PX", 4.0))
        self.prior_frames = int(getattr(config, "RECT_PRIOR_FRAMES", 15))
        self.prior_radius = float(getattr(config, "RECT_PRIOR_RADIUS_PX", 140.0))
        self.track_radius = float(getattr(config, "RECT_TRACK_RADIUS_PX", 12.0))
        self.track_samples = int(getattr(config, "RECT_TRACK_SAMPLES", 16))
        self.track_min_step = float(getattr(config, "RECT_TRACK_MIN_STEP", 6.0))
        self.track_max_shift = float(getattr(config, "RECT_TRACK_MAX_SHIFT_PX", 6.0))
        self.track_max_frames = int(getattr(config, "RECT_TRACK_MAX_FRAMES", 40))
        self._prior_center = None
        self._prior_lost = 0
        self._last_quad = None
        self._track_frames = 0
        self._fault_logged = False

    def reset(self):
        """清空跨帧状态。换靶/换场景时调用，避免拿旧位置去择优或精修。"""
        self.last = None
        self.last_stages = {}
        self.last_stats = {}
        self._prior_center = None
        self._prior_lost = 0
        self._last_quad = None
        self._track_frames = 0

    def _prior(self):
        """当前帧可用的位置先验；丢失太久就返回 None，退回纯面积择优。"""
        if self._prior_center is None or self._prior_lost > self.prior_frames:
            return None
        return self._prior_center, self.prior_radius

    def _try_track(self, gray):
        """用上一帧的四边做窄带精修。返回成功结果或 None。"""
        if self._last_quad is None or self._prior_lost > self.prior_frames:
            return None
        if self.track_max_frames and self._track_frames >= self.track_max_frames:
            return None  # 定期回全图，阻止精修误差累积
        quad = refine_edges_1d(gray, self._last_quad, self.track_radius,
                               self.track_samples, self.track_min_step,
                               self.track_max_shift)
        if quad is None:
            return None
        offset = band_offset_for(quad, 0.03, 3.0, max(3.0, self.band_offset))
        if band_score(gray, quad, offset, self.band_contrast) < self.min_band:
            return None
        median, low, high, edges = band_thickness(gray, quad, self.band_probe_ratio,
                                                  rise_step=self.band_rise_step)
        if median < self.min_band_ratio:
            return None
        if edges >= 3 and high > 0.0 and low / high < self.min_band_uniform:
            return None
        return quad

    def _process_frame(self, g):
        """跑一帧：先窄带精修，失败再全图搜索。返回 (结果, 统计)。"""
        stats = dict(contours=0, area=0, quad=0, aspect=0, band=0, candidates=0)
        tracked = self._try_track(g)
        if tracked is not None:
            score = band_score(g, tracked, band_offset_for(tracked, 0.03, 3.0,
                                                           max(3.0, self.band_offset)),
                               self.band_contrast)
            result = success_result(quad_payload(tracked, score), STAGE_TRACK)
            self._track_frames += 1
            stats["track"] = 1
            return result, stats

        blur = blur_frame(g, self.kernel)
        binary = binarize_frame(blur, self.block_size, self.c_const)
        candidates, stats = find_quad_candidates(
            binary, self.min_area, self.max_area_ratio, self.eps_ratio,
            self.min_aspect, self.max_aspect, self.max_candidates)
        # 框内亮度门按"全图均值"的相对比例给：现场偏暗时整幅一起暗，
        # 固定绝对灰度会把正常靶纸判死；只有明显暗于画面整体才可能是暗面板。
        inner_floor = max(self.inner_floor, self.min_inner_ratio * float(g.mean()))
        candidates, stats["band"], diag = filter_band(
            g, candidates, self.min_band, self.band_offset, self.band_contrast,
            self.min_band_ratio, self.band_probe_ratio,
            self.min_band_uniform, inner_floor,
            self.min_edge_score, self.band_rise_step, self.band_limit,
            self.debug_band)
        prior = self._prior()
        best = select_best(candidates, *(prior if prior else (None, 0.0)))
        if best is not None and self.refine_edges:
            best = dict(best)
            best["quad"] = refine_quad(best["quad"], best["contour"],
                                       self.edge_band, self.edge_min_points,
                                       self.edge_max_shift)
        result = locate_result(best, stats)
        if result.get("valid"):
            self._track_frames = 0
        elif self.debug_band and diag:
            # 只在失败帧里带：给板端 [perf] 的 det_fail 附上前几个候选的判据读数，
            # 一眼能看出是"外侧不暗"还是"暗带太窄"还是"框内太黑"。
            result["fail_reason"] = "%s|%s" % (result.get("fail_reason", "-"), " ".join(diag[:2]))
        if self.keep_stages:
            self.last_stages = dict(blur=blur, binary=binary)
        return result, stats

    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        ts = started if timestamp is None else float(timestamp)
        if not np.isfinite(ts):
            raise ValueError("non-finite timestamp")

        g = to_gray_frame(gray)
        if g.size == 0:
            result = fail_result("empty_frame", "none", "empty")
            stats = dict(contours=0, area=0, quad=0, aspect=0, band=0, candidates=0)
        else:
            try:
                result, stats = self._process_frame(g)
            except Exception as exc:
                # 板上任何一个 OpenCV 调用不支持、形状不匹配，都会让识别线程直接退出，
                # 上层只看到"一直识别不到"而没有任何线索。这里降级成失败结果、
                # 把异常类型留在 fail_reason 里（[perf] 的 det_fail 能直接读出来），
                # 并只打印一次完整栈，避免每帧刷屏。
                if not self._fault_logged:
                    self._fault_logged = True
                    traceback.print_exc()
                result = fail_result(STAGE_NONE, "exception", type(exc).__name__)
                stats = dict(contours=0, area=0, quad=0, aspect=0, band=0, candidates=0)

        result.update(masks=1, proposals=stats["contours"], search_count=1,
                      reject=dict(stats), threshold=None)
        if result.get("valid"):
            self._prior_center = (result["u"], result["v"])
            self._prior_lost = 0
            self._last_quad = np.asarray(result["quad"], np.float32)
        else:
            self._prior_lost += 1
            if self._prior_lost > self.prior_frames:
                self._last_quad = None
                self._track_frames = 0
        self.last_stats = stats
        self.last = finish_result(result, ts, (int(g.shape[1]), int(g.shape[0]))
                                  if g.size else (0, 0), started)
        return self.last


# ------------------------------------------------------------------ 绘制
def draw_result(gray, result, show_width=480, top_offset=0):
    """把一帧结果画成 BGR 画布。坐标一律按 scale 换算，文字仍报原图坐标。

    只画三个元素：绿色四边形、黄色外接框、蓝色靶心十字。外接框保留是为了
    直观看"框选得对不对"；长宽比按四边形对边比算。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / max(1, g.shape[1]))
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

    if not result.get("valid"):
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas

    q = np.rint(np.asarray(result["quad"], np.float32) * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    for x, y in q:
        cv2.circle(canvas, (int(x), int(y)), 3, (255, 0, 255), -1)

    bbox = result.get("bbox")
    if bbox:
        x, y, bw, bh = bbox
        cv2.rectangle(canvas, (int(round(x * scale)), int(round(y * scale))),
                      (int(round((x + bw) * scale)), int(round((y + bh) * scale))),
                      (0, 220, 255), 1)

    c = (int(round(result["u"] * scale)), int(round(result["v"] * scale)))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(canvas, "(%d,%d) area=%d aspect=%.2f band=%.2f %.1fms" %
                (round(result["u"]), round(result["v"]), int(result.get("area", 0)),
                 float(result.get("aspect", 0.)), float(result.get("band", 0.)),
                 result.get("detector_ms", 0.)),
                (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
