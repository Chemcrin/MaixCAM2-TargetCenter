"""局部光斑定位、靶心像素偏差与箭头；不执行云台控制。

坐标属于送入检测器的完整灰度图（启用 lens_corr 时是校正后的图）。
截图中的箭头只能给出初值，必须在实际工作距离、同一图像设置下校准。
"""
import math

import cv2
import numpy as np


def reference_point(frame_shape, config):
    height, width = frame_shape[:2]
    ref_w, ref_h = config.LASER_REFERENCE_SIZE
    laser_x, laser_y = config.LASER_PIXEL
    if min(width, height, ref_w, ref_h) <= 0:
        raise ValueError("激光参考分辨率和当前图像尺寸必须大于零")
    if (not all(math.isfinite(float(v)) for v in (laser_x, laser_y)) or
            not (0 <= laser_x < ref_w and 0 <= laser_y < ref_h)):
        raise ValueError("LASER_PIXEL 必须位于 LASER_REFERENCE_SIZE 图像内")
    return float(laser_x) * width / ref_w, float(laser_y) * height / ref_h


class LaserSpotDetector:
    """仅在参考位置附近约 64×64 ROI 查找小而孤立的亮斑。

    去局部背景后找亮连通域，排除长线、大反光、图像边缘和竞争亮点。
    本帧未检测到可靠光斑立即返回无效；不回填旧帧坐标。
    灰度下无法区分所有点状反光，结果是亮斑测量而非颜色鉴别。
    """
    def __init__(self, config):
        self.cfg = config

    def process(self, gray):
        g = np.asarray(gray).squeeze()
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("光斑检测需要 uint8 灰度图")
        lx, ly = reference_point(g.shape, self.cfg)
        radius = int(self.cfg.LASER_SEARCH_RADIUS)
        if radius < 10:
            raise ValueError("LASER_SEARCH_RADIUS 至少为 10")
        h, w = g.shape
        x0, y0 = max(0, int(lx) - radius), max(0, int(ly) - radius)
        x1, y1 = min(w, int(lx) + radius + 1), min(h, int(ly) + radius + 1)
        roi = g[y0:y1, x0:x1]
        empty = dict(valid=False, u=None, v=None, contrast=0.0, stage="missing")
        if min(roi.shape) < 15:
            return empty
        # float32 避免 uint8 相减溢出。仅 ROI 模糊，不增加全图扫描。
        values = roi.astype(np.float32)
        residual = values - cv2.GaussianBlur(values, (15, 15), 0)
        mask = (residual >= self.cfg.LASER_MIN_CONTRAST).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        candidates = []
        for i in range(1, count):
            x, y, bw, bh, area = map(int, stats[i])
            if not (self.cfg.LASER_MIN_AREA <= area <= self.cfg.LASER_MAX_AREA):
                continue
            if (x < 4 or y < 4 or x + bw >= roi.shape[1] - 4 or
                    y + bh >= roi.shape[0] - 4 or max(bw, bh) > self.cfg.LASER_MAX_DIAMETER or
                    max(bw, bh) / max(1, min(bw, bh)) > 2.5):
                continue
            yy, xx = np.nonzero(labels[y:y+bh, x:x+bw] == i)
            xx, yy = xx + x, yy + y
            weights = residual[yy, xx]
            cx, cy = float(np.average(xx, weights=weights)), float(np.average(yy, weights=weights))
            peak = float(values[yy, xx].max())
            # 检查四周局部背景；线条/亮面边界一侧也亮，不是独立小光点。
            ring_radius = max(4, int(math.ceil(max(bw, bh) / 2.0)) + 3)
            angles = np.arange(16) * (2.0 * np.pi / 16)
            rx = np.rint(cx + ring_radius * np.cos(angles)).astype(int)
            ry = np.rint(cy + ring_radius * np.sin(angles)).astype(int)
            if rx.min() < 0 or ry.min() < 0 or rx.max() >= roi.shape[1] or ry.max() >= roi.shape[0]:
                continue
            ring = values[ry, rx]
            contrast = peak - float(np.median(ring))
            if (contrast < self.cfg.LASER_MIN_CONTRAST or
                    float((peak - ring >= self.cfg.LASER_MIN_CONTRAST).mean()) < 0.75):
                continue
            u, v = cx + x0, cy + y0
            distance = math.hypot(u - lx, v - ly)
            score = float(weights.max()) - 0.20 * distance
            candidates.append(dict(valid=True, u=u, v=v, contrast=contrast,
                                   stage="measured", score=score, area=area))
        candidates.sort(key=lambda c: c["score"], reverse=True)
        if not candidates:
            return empty
        if (len(candidates) > 1 and candidates[1]["score"] >= candidates[0]["score"] * 0.8):
            empty["stage"] = "ambiguous"
            return empty
        return candidates[0]


def add_aiming(result, frame_shape, config, spot=None):
    """原地补充本帧数据。dx>0 向右，dy>0 向下；失锁清空目标偏差。"""
    # 仅适用于等视场图像缩放；改变裁剪/镜头校正/镜像/相机模式时须重新标定。
    ref_x, ref_y = reference_point(frame_shape, config)
    if config.LASER_MODE == "fixed":
        laser_valid, lx, ly, source = True, ref_x, ref_y, "fixed"
    elif config.LASER_MODE == "detect":
        laser_valid = bool(spot and spot.get("valid"))
        lx, ly = (float(spot["u"]), float(spot["v"])) if laser_valid else (None, None)
        source = spot.get("stage", "missing") if spot else "missing"
    else:
        raise ValueError("LASER_MODE 必须为 detect 或 fixed")
    calibrated = bool(config.LASER_CALIBRATED)
    target_valid = bool(result.get("valid"))
    if target_valid:
        target_valid = all(math.isfinite(float(result.get(k, math.nan))) for k in ("u", "v"))
    aim_valid = target_valid and laser_valid
    dx = float(result["u"]) - lx if aim_valid else None
    dy = float(result["v"]) - ly if aim_valid else None
    result.update(laser_u=lx, laser_v=ly, laser_valid=laser_valid,
                  laser_reference=(ref_x, ref_y), laser_source=source,
                  laser_contrast=float(spot.get("contrast", 0.0)) if spot else 0.0,
                  laser_calibrated=calibrated, aim_valid=aim_valid,
                  dx=dx, dy=dy, distance_px=math.hypot(dx, dy) if aim_valid else None)
    return result


def pack_aiming(result, protocol="aim"):
    """返回一帧 ASCII 包。aim 扩展协议与旧 center 协议显式区分。"""
    if protocol == "center":
        if result.get("valid"):
            return ("@[%d,%d]%%\r\n" % (round(result["u"]), round(result["v"]))).encode("ascii")
        return b"@[invalid]%\r\n"
    if protocol != "aim":
        raise ValueError("UART_PROTOCOL 必须为 aim 或 center")
    valid = bool(result.get("aim_valid"))
    lx, ly = (round(result["laser_u"]), round(result["laser_v"])) if result["laser_valid"] else (0, 0)
    tx, ty = (round(result["u"]), round(result["v"])) if result.get("valid") else (0, 0)
    # 在量化后求差，保证整数包中 dx=tx-lx、dy=ty-ly 严格成立。
    dx, dy = (tx - lx, ty - ly) if valid else (0, 0)
    return ("@A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d%%\r\n" %
            (int(valid), tx, ty, lx, ly, dx, dy,
             int(bool(result["laser_calibrated"])), int(result["laser_valid"]),
             int(bool(result.get("valid"))))).encode("ascii")


def draw_aiming(canvas, result, frame_shape):
    """在已有 BGR 预览上叠加激光点和 L→T 箭头；只在显示帧调用。"""
    height, width = frame_shape[:2]
    show_h, show_w = canvas.shape[:2]
    sx, sy = show_w / float(width), show_h / float(height)
    laser_xy = (result["laser_u"], result["laser_v"]) if result["laser_valid"] else result["laser_reference"]
    laser = (int(round(laser_xy[0] * sx)), int(round(laser_xy[1] * sy)))
    laser = (min(show_w - 1, laser[0]), min(show_h - 1, laser[1]))
    if result["aim_valid"]:
        target = (int(round(result["u"] * sx)), int(round(result["v"] * sy)))
        if target != laser:
            cv2.arrowedLine(canvas, laser, target, (0, 180, 255), 2,
                            cv2.LINE_AA, tipLength=0.16)
        cv2.drawMarker(canvas, target, (255, 0, 0), cv2.MARKER_CROSS, 12, 2)
    color = (0, 0, 255) if result["laser_valid"] else (128, 128, 128)
    cv2.circle(canvas, laser, 5, color, 2, cv2.LINE_AA)
    cv2.drawMarker(canvas, laser, color, cv2.MARKER_CROSS, 9, 1)
    cv2.putText(canvas, "L", (max(0, min(show_w - 14, laser[0] + 8)),
                            max(12, min(show_h - 5, laser[1] - 8))),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    label = "SPOT" if result["laser_source"] == "measured" else "FIXED REF"
    lines = (["L=(%.1f,%.1f) %s" % (result["laser_u"], result["laser_v"], label)]
             if result["laser_valid"] else ["LASER %s" % result["laser_source"].upper()])
    if result["aim_valid"]:
        lines.append("dX=%+.1f dY=%+.1f px" % (result["dx"], result["dy"]))
    else:
        lines.append("dX=-- dY=-- %s" % ("NO LASER" if not result["laser_valid"] else "NO TARGET"))
    for y, text in zip((show_h - 50, show_h - 32), lines):
        cv2.putText(canvas, text, (8, max(12, y)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.43, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, text, (8, max(12, y)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.43, (0, 230, 255), 1, cv2.LINE_AA)
    return canvas
