"""OpenCV 黑框识别：自适应反色二值化 → 轮廓 → 三层过滤 → 四点对角线交点。

组织方式：每个小功能都是**模块级函数**，输入输出只依赖参数，不读类状态；
`RectangleCenterDetector` 只做两件事——保存参数、按顺序调用它们。
这样任何一步都能单独跑、单独测（`diagnose_detect.py` 就是直接调下面这些函数）。

    to_gray_frame      校验并去掉灰度哑轴
    blur_frame         高斯模糊（可关）
    binarize_frame     自适应反色，黑胶带 → 白环
    find_quad_candidates  三层过滤，返回候选 + 各层淘汰计数
    select_largest     在通过者里取面积最大
    locate_result      候选 → 结果 dict（含失败归因）
    quad_diagonal_center  四点 → 对角线交点
    draw_result        结果 → BGR 画布

三条刻意的设计约束：

1. **无跨帧状态**：每帧独立全图搜索，同一张图必然得到同一结果。
2. **判据只有三层**：面积 → 四边形近似 → 长宽比。不做胶带暗带证据、亚像素精修、
   单应反算。画面里出现更大的深色矩形时会锁错，这是本方案的已知边界。
3. **中心取四点的对角线交点**，不取正立外接矩形中心。

分阶段调试：`RECT_KEEP_STAGES` 置真后，`process` 把中间结果留在 `last_stages`
（`blur` / `binary`），供 `tools/run_pc.py --stage` 导出。
"""
import time

import cv2
import numpy as np

STAGE_OK = "full_search"
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


# ------------------------------------------------------------------ 结果组装
def fail_result(stage, fail_stage, reason):
    """统一的失败结果。`fail_stage` 是走过的层，`reason` 是首次失败原因。"""
    return dict(valid=False, stage=stage, fail_stage=fail_stage, fail_reason=reason)


def success_result(best):
    """胜出候选 → 成功结果。四点共线（对角线无交点）时退化为失败，不拿外接框中心凑数。"""
    quad = order_quad(best["approx"])
    center = quad_diagonal_center(quad)
    if center is None:
        return fail_result(STAGE_NONE, "full", "degenerate")
    return dict(valid=True, stage=STAGE_OK,
                u=float(center[0]), v=float(center[1]),
                raw_u=float(center[0]), raw_v=float(center[1]),
                quad=quad,
                # 新链路只认一个框，不区分胶带内外沿；给 None 让绘图层跳过。
                inner_quad=None, detected_quad=quad, outer_inferred=False,
                area=best["area"], aspect=best["aspect"], bbox=best["bbox"],
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
    """只保存参数并编排上面那些函数；本身不做像素运算。"""

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self.last_stages = {}
        self.last_stats = {}
        # 参数全部允许缺省，config 未同步时也不至于抛 AttributeError。
        self.min_area = float(getattr(config, "RECT_MIN_AREA", 300))
        self.max_area_ratio = float(getattr(config, "RECT_MAX_AREA_RATIO", 0.92))
        self.eps_ratio = float(getattr(config, "RECT_APPROX_EPS", 0.02))
        self.min_aspect = float(getattr(config, "RECT_MIN_ASPECT", 1.1))
        self.max_aspect = float(getattr(config, "RECT_MAX_ASPECT", 1.8))
        self.block_size = int(getattr(config, "RECT_BLOCK_SIZE", 31))
        self.c_const = float(getattr(config, "RECT_C", 8.0))
        self.kernel = tuple(getattr(config, "RECT_GAUSS_KERNEL", (5, 5)))
        self.max_candidates = int(getattr(config, "RECT_MAX_CANDIDATES", 32))
        self.keep_stages = bool(getattr(config, "RECT_KEEP_STAGES", False))

    def reset(self):
        """接口兼容用。本检测器无跨帧状态，清空只影响调试缓存。"""
        self.last = None
        self.last_stages = {}
        self.last_stats = {}

    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        ts = started if timestamp is None else float(timestamp)
        if not np.isfinite(ts):
            raise ValueError("non-finite timestamp")

        g = to_gray_frame(gray)
        if g.size == 0:
            result = fail_result("empty_frame", "none", "empty")
            stats = dict(contours=0, area=0, quad=0, aspect=0, candidates=0)
        else:
            blur = blur_frame(g, self.kernel)
            binary = binarize_frame(blur, self.block_size, self.c_const)
            candidates, stats = find_quad_candidates(
                binary, self.min_area, self.max_area_ratio, self.eps_ratio,
                self.min_aspect, self.max_aspect, self.max_candidates)
            result = locate_result(select_largest(candidates), stats)
            if self.keep_stages:
                self.last_stages = dict(blur=blur, binary=binary)

        result.update(masks=1, proposals=stats["contours"], search_count=1,
                      reject=dict(stats), threshold=None)
        self.last_stats = stats
        self.last = finish_result(result, ts, (int(g.shape[1]), int(g.shape[0]))
                                  if g.size else (0, 0), started)
        return self.last


# ------------------------------------------------------------------ 绘制
def draw_result(gray, result, show_width=480, top_offset=0):
    """把一帧结果画成 BGR 画布。坐标一律按 scale 换算，文字仍报原图坐标。

    只画三个元素：绿色四边形、黄色外接框、蓝色靶心十字。外接框保留是为了
    直观看"框选得对不对"；长宽比按正立外接框算。
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
    cv2.putText(canvas, "(%d,%d) area=%d aspect=%.2f %.1fms" %
                (round(result["u"]), round(result["v"]), int(result.get("area", 0)),
                 float(result.get("aspect", 0.)), result.get("detector_ms", 0.)),
                (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    return canvas
