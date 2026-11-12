"""PC 冒烟回归：验证四层链路（提案 → 带宽验证 → 四边精修 → 内框交点 → 偏差）没被改坏。

不依赖任何图片固件，全部用合成图，因此可以进无人值守的门禁。
合成图的**黑带宽度按物理比例绘制**（`BLACK_FRAME_MM` / 纸面长边 × 框像素长边）——
带宽是判据的一部分，随手画的细边框本来就应该被拒，不能拿来当正样本。
真机场景（反光、贴边、遮挡）不在本用例覆盖范围，上板另行确认。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from target_api import CenterTracker, detect_black_frame       # noqa: E402
from vision.algo import deviation, quad_diagonal_center        # noqa: E402

FAILS = []
TAPE_MM = 18.0
PAPER_LONG_MM = 297.0


def check(label, ok, detail=""):
    print("  %s %s%s" % ("OK  " if ok else "FAIL", label, ("  <- " + detail) if detail and not ok else ""))
    if not ok:
        FAILS.append(label)


def tape_px(box_long_px):
    """按物理比例算黑带像素宽度：框长边 px 对应纸面长边 297mm。"""
    return max(3, int(round(TAPE_MM * box_long_px / PAPER_LONG_MM)))


def draw_frame(cx, cy, w, h, thickness=None, rings=4, background=235, ink=40,
               size=(640, 480), angle=0.0, warp=None):
    """画一张白底黑框 + 同心环的合成靶纸；框中心即期望靶心。

    rings=0 用来造"亮面被暗边包住但没有同心环"的变体。
    angle：整图绕 (cx, cy) 旋转（真值中心不变，测旋转不变性）。
    warp：3×3 单应，整图透视变换（真值 = H × 中心，测透视正确性）。
    borderValue 必须是背景色，否则变换后的黑边本身就是一圈假候选。
    """
    if thickness is None:
        thickness = tape_px(max(w, h))
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
    print("[smoke] 1. 正样本：240x170 黑框（带宽按物理比例），中心应落在 (320, 240) 附近")
    res = detect_black_frame(draw_frame(320, 240, 240, 170))
    check("valid", bool(res.valid), repr(res))
    if res.valid:
        err = _center_error(res, (320.0, 240.0))
        check("中心误差 <= 2px", err <= 2.0, "err=%.2fpx center=(%.1f, %.1f)" % (err, res.u, res.v))
        check("四点齐全", res.get("quad") is not None and len(res["quad"]) == 4)
        # 中心语义：必须是纸边（内框）对角线交点，不能退化成外框中心。
        inner_c = quad_diagonal_center(res["inner_quad"])
        check("中心=内框对角线交点",
              inner_c is not None and max(abs(inner_c[0] - res.u), abs(inner_c[1] - res.v)) <= 0.5,
              "inner=%s center=(%.1f,%.1f)" % (inner_c, res.u, res.v))
        ow = float(np.linalg.norm(np.asarray(res["quad"][0]) - np.asarray(res["quad"][1])))
        iw = float(np.linalg.norm(np.asarray(res["inner_quad"][0]) - np.asarray(res["inner_quad"][1])))
        check("内框比外框小一个带宽", ow - iw > 5.0, "outer=%.1f inner=%.1f" % (ow, iw))

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

    print("[smoke] 3. 带宽门：细边框（3px，远窄于 18mm 胶带）必须拒检")
    res = detect_black_frame(draw_frame(320, 240, 240, 170, thickness=3))
    check("invalid", not res.valid, repr(res))
    check("归因 all_filtered", res.get("fail_reason") == "all_filtered", str(res.get("fail_reason")))

    print("[smoke] 3b. 形状门：正方形框（显示器/窗格那类）必须拒检")
    res = detect_black_frame(draw_frame(320, 240, 200, 200, rings=0))
    check("invalid", not res.valid, repr(res))

    print("[smoke] 3c. 结构负样本：洞洞板/门框/纯同心环都不得检出")
    slab = np.full((480, 640), 235, np.uint8)
    cv2.rectangle(slab, (120, 90), (520, 390), 40, -1)
    for gy in range(130, 390, 40):
        for gx in range(160, 520, 40):
            cv2.circle(slab, (gx, gy), 8, 235, -1)
    res = detect_black_frame(slab)
    check("洞洞板 invalid", not res.valid, str(res.get("fail_reason")))
    door = np.full((480, 640), 235, np.uint8)
    cv2.rectangle(door, (140, 80), (180, 400), 40, -1)
    cv2.rectangle(door, (140, 80), (420, 120), 40, -1)
    res = detect_black_frame(door)
    check("门框 invalid", not res.valid, str(res.get("fail_reason")))
    rings_only = np.full((480, 640), 235, np.uint8)
    for r in (80, 60, 40):
        cv2.circle(rings_only, (320, 240), r, 40, 4, cv2.LINE_AA)
    res = detect_black_frame(rings_only)
    check("纯同心环 invalid", not res.valid, str(res.get("fail_reason")))

    print("[smoke] 3d. 遮挡鲁棒：角部被亮块盖住仍应锁定（现场手/夹具挡角很常见）")
    occluded = draw_frame(320, 240, 240, 170)
    cv2.rectangle(occluded, (195, 150), (250, 190), 235, -1)
    res = detect_black_frame(occluded)
    check("遮挡后 valid", bool(res.valid), repr(res))
    if res.valid:
        check("遮挡后中心误差 <= 3px", _center_error(res, (320.0, 240.0)) <= 3.0,
              "center=(%.1f, %.1f)" % (res.u, res.v))

    print("[smoke] 3e. 出画：靶纸大半移出画面必须拒检（没有完整框就没有真值）")
    clipped = np.full((480, 640), 235, np.uint8)
    th = tape_px(max(240, 170))
    cv2.rectangle(clipped, (-60, 155), (180, 325), 40, th)
    res = detect_black_frame(clipped)
    check("大半出画 invalid", not res.valid, str(res.get("fail_reason")))

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
    check("第二帧起走跟踪态", hits[-1].get("search_mode") == "track", str(hits[-1].get("search_mode")))
    check("reset 后仍可检出", bool(tracker.reset().track(frame).valid))

    print("[smoke] %s（失败 %d 项）" % ("PASS" if not FAILS else "FAIL", len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
