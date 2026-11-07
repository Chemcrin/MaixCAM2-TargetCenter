"""PC 冒烟回归：验证四层链路（二值化 → 三层过滤 → 对角线中心 → 偏差）没被改坏。

不依赖任何图片固件，全部用合成图，因此可以进无人值守的门禁。
真机场景（反光、贴边、大角度）不在本用例覆盖范围，上板另行确认。
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


def draw_frame(cx, cy, w, h, thickness=8, background=235, ink=40, size=(640, 480)):
    """画一张白底黑框的合成靶纸；框中心即期望靶心。"""
    img = np.full((size[1], size[0]), background, np.uint8)
    cv2.rectangle(img, (cx - w // 2, cy - h // 2), (cx + w // 2, cy + h // 2), ink, thickness)
    return img


def main():
    print("[smoke] 1. 正样本：240x170 黑框，中心应落在 (320, 240) 附近")
    res = detect_black_frame(draw_frame(320, 240, 240, 170))
    check("valid", bool(res.valid), repr(res))
    if res.valid:
        err = max(abs(res.u - 320.0), abs(res.v - 240.0))
        check("中心误差 <= 2px", err <= 2.0, "err=%.2fpx center=(%.1f, %.1f)" % (err, res.u, res.v))
        check("四点齐全", res.get("quad") is not None and len(res["quad"]) == 4)

    print("[smoke] 2. 负样本：空白画面应判 no_proposal，不得伪检出")
    res = detect_black_frame(np.full((480, 640), 235, np.uint8))
    check("invalid", not res.valid, repr(res))
    check("归因 no_proposal", res.get("fail_reason") == "no_proposal", str(res.get("fail_reason")))

    print("[smoke] 3. 边界：正方形被长宽比门拒绝（竖放/大角度倾斜的已知边界）")
    res = detect_black_frame(draw_frame(320, 240, 200, 200))
    check("invalid", not res.valid, repr(res))
    check("归因 all_filtered", res.get("fail_reason") == "all_filtered", str(res.get("fail_reason")))

    print("[smoke] 4. 偏差符号：dx=靶心-光斑，右/下为正")
    laser = dict(found=True, u=300.0, v=200.0)
    target = dict(valid=True, u=320.0, v=240.0, raw_u=320.0, raw_v=240.0, image_size=(640, 480))
    dv = deviation(laser, target, use_raw=True)
    check("dx=+20 dy=+40", dv is not None and abs(dv[0] - 20.0) < 1e-6 and abs(dv[1] - 40.0) < 1e-6, str(dv))
    check("光斑丢失返回 None", deviation(dict(found=False, u=0.0, v=0.0), target) is None)

    print("[smoke] 5. 追踪器：连续三帧稳定，reset 后可继续使用")
    tracker = CenterTracker()
    frame = draw_frame(320, 240, 240, 170)
    hits = [tracker.track(frame) for _ in range(3)]
    check("三帧全部 valid", all(r.valid for r in hits), str([r.valid for r in hits]))
    check("位置不漂移", max(abs(r.u - hits[0].u) + abs(r.v - hits[0].v) for r in hits) < 1e-6)
    check("reset 后仍可检出", bool(tracker.reset().track(frame).valid))

    print("[smoke] %s（失败 %d 项）" % ("PASS" if not FAILS else "FAIL", len(FAILS)))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
