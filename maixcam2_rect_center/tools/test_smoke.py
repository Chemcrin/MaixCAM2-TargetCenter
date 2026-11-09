"""PC 冒烟回归：验证四层链路（阈值 → 环-洞结构 → 四线交角 → 对角线交点 → 偏差）没被改坏。

不依赖任何图片固件，全部用合成图，因此可以进无人值守的门禁。
真机场景（反光、贴边、遮挡）不在本用例覆盖范围，上板另行确认。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from target_api import CenterTracker, detect_black_frame       # noqa: E402
from vision.algo import deviation                              # noqa: E402

FAILS = []


def check(label, ok, detail=""):
    print("  %s %s%s" % ("OK  " if ok else "FAIL", label, ("  <- " + detail) if detail and not ok else ""))
    if not ok:
        FAILS.append(label)


def draw_frame(cx, cy, w, h, thickness=8, rings=4, background=235, ink=40,
               size=(640, 480), angle=0.0, warp=None):
    """画一张白底黑框 + 同心环的合成靶纸；框中心即期望靶心。

    rings=0 用来造"亮面被暗边包住但没有同心环"的反例（笔记本屏幕、窗框那一类）。
    angle：整图绕 (cx, cy) 旋转（真值中心不变，测旋转不变性）。
    warp：3×3 单应，整图透视变换（真值 = H × 中心，测透视正确性）。
    borderValue 必须是背景色，否则变换后的黑边本身就是一圈假候选。
    """
    img = np.full((size[1], size[0]), background, np.uint8)
    cv2.rectangle(img, (cx - w // 2, cy - h // 2), (cx + w // 2, cy + h // 2), ink, thickness)
    if rings > 0:
        r0 = 0.42 * min(w, h)
        for i in range(rings):
            cv2.circle(img, (cx, cy), int(r0 * (1 - 0.22 * i)), ink, 2, cv2.LINE_AA)
    if angle:
        M = cv2.getRotationMatrix2D((float(cx), float(cy)), float(angle), 1.0)
        img = cv2.warpAffine(img, M, size, flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=(float(background),))
    if warp is not None:
        img = cv2.warpPerspective(img, np.asarray(warp, np.float64), size,
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=(float(background),))
    return img


def _center_error(res, truth):
    return max(abs(res.u - truth[0]), abs(res.v - truth[1]))


def main():
    print("[smoke] 1. 正样本：240x170 黑框，中心应落在 (320, 240) 附近")
    res = detect_black_frame(draw_frame(320, 240, 240, 170))
    check("valid", bool(res.valid), repr(res))
    if res.valid:
        err = _center_error(res, (320.0, 240.0))
        check("中心误差 <= 2px", err <= 2.0, "err=%.2fpx center=(%.1f, %.1f)" % (err, res.u, res.v))
        check("四点齐全", res.get("quad") is not None and len(res["quad"]) == 4)

    print("[smoke] 1b. 旋转不变：30°/45° 旋转后中心不动")
    for deg in (30.0, 45.0):
        res = detect_black_frame(draw_frame(320, 240, 240, 170, angle=deg))
        check("%g° valid" % deg, bool(res.valid), repr(res))
        if res.valid:
            err = _center_error(res, (320.0, 240.0))
            check("%g° 中心误差 <= 3px" % deg, err <= 3.0,
                  "err=%.2fpx center=(%.1f, %.1f)" % (err, res.u, res.v))

    print("[smoke] 1c. 透视正确：梯形（顶边缩 25%）中心 = H×(320,240)")
    src = np.float32([[200, 155], [440, 155], [440, 325], [200, 325]])
    dst = np.float32([[230, 155], [410, 155], [440, 325], [200, 325]])   # 顶边缩 25%
    H = cv2.getPerspectiveTransform(src, dst)
    truth = H @ np.array([320.0, 240.0, 1.0])
    truth = (truth[0] / truth[2], truth[1] / truth[2])
    res = detect_black_frame(draw_frame(320, 240, 240, 170, warp=H))
    check("透视 valid", bool(res.valid), repr(res))
    if res.valid:
        err = _center_error(res, truth)
        check("透视中心误差 <= 3px", err <= 3.0,
              "err=%.2fpx center=(%.1f, %.1f) truth=(%.1f, %.1f)" % (err, res.u, res.v, truth[0], truth[1]))

    print("[smoke] 2. 负样本：空白画面应判 no_proposal，不得伪检出")
    res = detect_black_frame(np.full((480, 640), 235, np.uint8))
    check("invalid", not res.valid, repr(res))
    check("归因 no_proposal", res.get("fail_reason") == "no_proposal", str(res.get("fail_reason")))

    print("[smoke] 3. 边界：正方形被洞边比门拒绝（rings=0，隔离出形状门本身）")
    res = detect_black_frame(draw_frame(320, 240, 200, 200, rings=0))
    check("invalid", not res.valid, repr(res))
    check("归因 all_filtered", res.get("fail_reason") == "all_filtered", str(res.get("fail_reason")))

    print("[smoke] 3b. 结构负样本：洞洞板/门框/纯同心环都不得检出")
    # 洞洞板：一大块暗板上打亮孔（fill 很高 → 预筛杀）
    slab = np.full((480, 640), 235, np.uint8)
    cv2.rectangle(slab, (120, 90), (520, 390), 40, -1)
    for gy in range(130, 390, 40):
        for gx in range(160, 520, 40):
            cv2.circle(slab, (gx, gy), 8, 235, -1)
    res = detect_black_frame(slab)
    check("洞洞板 invalid", not res.valid, str(res.get("fail_reason")))
    # 门框：不闭合的粗 L 形暗带（没有洞 → 结构门杀）
    door = np.full((480, 640), 235, np.uint8)
    cv2.rectangle(door, (140, 80), (180, 400), 40, -1)
    cv2.rectangle(door, (140, 80), (420, 120), 40, -1)
    res = detect_black_frame(door)
    check("门框 invalid", not res.valid, str(res.get("fail_reason")))
    # 纯同心环（无外框）：洞是圆的 → 洞边比门杀
    rings_only = np.full((480, 640), 235, np.uint8)
    for r in (80, 60, 40):
        cv2.circle(rings_only, (320, 240), r, 40, 4, cv2.LINE_AA)
    res = detect_black_frame(rings_only)
    check("纯同心环 invalid", not res.valid, str(res.get("fail_reason")))

    print("[smoke] 4. 偏差符号：dx=靶心-光斑，右/下为正")
    laser = dict(found=True, u=300.0, v=200.0)
    target = dict(valid=True, u=320.0, v=240.0, raw_u=320.0, raw_v=240.0, image_size=(640, 480))
    dv = deviation(laser, target, use_raw=True)
    check("dx=+20 dy=+40", dv is not None and abs(dv[0] - 20.0) < 1e-6 and abs(dv[1] - 40.0) < 1e-6, str(dv))
    check("光斑丢失返回 None", deviation(dict(found=False, u=0.0, v=0.0), target) is None)

    print("[smoke] 5. 追踪器：连续三帧稳定 + reset 后可继续使用")
    tracker = CenterTracker()
    frame = draw_frame(320, 240, 240, 170)
    hits = [tracker.track(frame) for _ in range(3)]
    check("三帧全部 valid", all(r.valid for r in hits), str([r.valid for r in hits]))
    drift = max(abs(r.u - hits[0].u) + abs(r.v - hits[0].v) for r in hits) if hits[0].valid else 1e9
    check("位置稳定（漂移 < 1px）", drift < 1.0, "drift=%.3fpx" % drift)
    check("reset 后仍可检出", bool(tracker.reset().track(frame).valid))

    print("[smoke] %s（失败 %d 项）" % ("PASS" if not FAILS else "FAIL", len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
