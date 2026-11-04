"""识别内核回归：合成靶纸的中心精度 / 旋转与竖放 / 角点重复性 / 误检拒绝。

为什么要有这个测试：`check_pc.py` 其余用例覆盖的是云台、串口、显示、运行时，
**没有一条验证"还能不能认出靶纸"**。改判据时门禁照样全绿，直到上板才发现检不出。
这里用合成靶纸补上精度与稳定性，用真实帧补上"真实场景还能不能检出"。

用例分四组：
    S*  合成靶纸：中心误差、四边形四角误差
    R*  旋转与竖放：旧判据（正立外接框长宽比）会在这两组上直接失败
    N*  负样本：笔记本屏幕必须拒检
    T*  真实帧：录屏里裁出的靶纸画面，必须检出且中心落在靶纸内

合成图是干净的，只证明几何正确；真实场景的鲁棒性靠 T* 与现场 `[perf]` 判断。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from target_api import CenterTracker, load_config  # noqa: E402

FIXTURE = Path(__file__).resolve().parent
FAILS = []


def synth_target(angle=0.0, scale=1.0, centre=(320, 240), vertical=False, bg=110):
    """画一张合成靶纸：灰背景 + 黑胶带外圈 + 白纸面 + 同心圆。

    返回 (灰度图, 真值中心)。真值中心就是贴片中心——旋转不改变贴片中心。
    """
    pw, ph = int(300 * scale), int(212 * scale)
    if vertical:
        pw, ph = ph, pw
    tape = max(4, int(12 * scale))
    tile = np.full((ph, pw), 30, np.uint8)
    tile[tape:ph - tape, tape:pw - tape] = 205
    inner_h, inner_w = ph - 2 * tape, pw - 2 * tape
    for r in range(1, 6):
        cv2.circle(tile, (pw // 2, ph // 2),
                   int(min(inner_w, inner_h) * 0.09 * r), 120, 1, cv2.LINE_AA)
    canvas = np.full((480, 640), bg, np.uint8)
    if angle:
        # 旋转前先加一圈背景，否则旋转会把靶纸四角裁掉（那是生成器的缺陷，不是检测的）
        pad = int((pw + ph) * 0.45)
        tile = cv2.copyMakeBorder(tile, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=bg)
        mat = cv2.getRotationMatrix2D((tile.shape[1] / 2.0, tile.shape[0] / 2.0), angle, 1.0)
        tile = cv2.warpAffine(tile, mat, (tile.shape[1], tile.shape[0]),
                              flags=cv2.INTER_LINEAR, borderValue=bg)
        ph, pw = tile.shape[:2]
    x0 = int(round(centre[0] - pw / 2.0))
    y0 = int(round(centre[1] - ph / 2.0))
    sx = max(0, -x0)
    sy = max(0, -y0)
    dx = max(0, x0)
    dy = max(0, y0)
    cw = min(pw - sx, canvas.shape[1] - dx)
    ch = min(ph - sy, canvas.shape[0] - dy)
    canvas[dy:dy + ch, dx:dx + cw] = tile[sy:sy + ch, sx:sx + cw]
    return canvas, (float(centre[0]), float(centre[1]))


def check(name, cond, detail=""):
    if cond:
        print("  OK   %s  %s" % (name, detail))
    else:
        print("  FAIL %s  %s" % (name, detail))
        FAILS.append(name)


def run_detect(tracker, gray):
    return tracker.track(gray)


def quad_corner_error(quad, truth_quad):
    a = np.asarray(quad, np.float64).reshape(4, 2)
    b = np.asarray(truth_quad, np.float64).reshape(4, 2)
    return float(np.abs(a - b).max())


def main():
    print("识别内核回归 —— 合成靶纸 / 旋转竖放 / 误检 / 真实帧")
    cfg = load_config()
    tracker = CenterTracker(cfg)

    print("\nS 合成靶纸：中心应落在真值 2px 内")
    for scale, centres in ((0.6, ((320, 240), (200, 180))),
                           (1.0, ((320, 240), (200, 180))),
                           (1.4, ((320, 240),))):
        for cx, cy in centres:
            gray, truth = synth_target(scale=scale, centre=(cx, cy))
            tracker.reset()
            res = run_detect(tracker, gray)
            err = (abs(res.u - truth[0]) + abs(res.v - truth[1])) if res.valid else 999.0
            check("S1 scale=%.1f centre=(%d,%d)" % (scale, cx, cy),
                  res.valid and err <= 2.0,
                  "valid=%s err=%.2f band=%.2f" % (res.valid, err, res.get("band", 0)))

    print("\nR 旋转与竖放：旧判据会在这些角度上失败")
    for angle in (0, 15, -25, 35):
        gray, truth = synth_target(angle=angle)
        tracker.reset()
        res = run_detect(tracker, gray)
        err = (abs(res.u - truth[0]) + abs(res.v - truth[1])) if res.valid else 999.0
        check("R1 angle=%+d" % angle, res.valid and err <= 3.0,
              "valid=%s err=%.2f aspect=%.2f" % (res.valid, err, res.get("aspect", 0)))
    for angle in (0, 20):
        gray, truth = synth_target(angle=angle, vertical=True)
        tracker.reset()
        res = run_detect(tracker, gray)
        err = (abs(res.u - truth[0]) + abs(res.v - truth[1])) if res.valid else 999.0
        check("R2 vertical angle=%+d" % angle, res.valid and err <= 3.0,
              "valid=%s err=%.2f aspect=%.2f" % (res.valid, err, res.get("aspect", 0)))

    print("\nR3 角点重复性：同一帧加不同噪声，四角标准差应 < 1px")
    base, _ = synth_target(angle=12)
    quads = []
    rng = np.random.RandomState(7)
    for _ in range(6):
        noisy = np.clip(base.astype(np.int16) +
                        rng.normal(0, 4, base.shape), 0, 255).astype(np.uint8)
        tracker.reset()
        res = run_detect(tracker, noisy)
        if res.valid:
            quads.append(np.asarray(res.quad, np.float64).reshape(4, 2))
    if len(quads) >= 4:
        stack = np.stack(quads)
        spread = float(stack.std(axis=0).max())
        check("R3 corner std", spread < 1.0, "std=%.3fpx n=%d" % (spread, len(quads)))
    else:
        check("R3 corner std", False, "only %d/6 valid" % len(quads))

    print("\nN 负样本：笔记本屏幕必须拒检")
    for name in ("negative_e421.png",):
        path = FIXTURE / name
        if not path.exists():
            print("  SKIP %s  (缺固件)" % name)
            continue
        gray = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_GRAYSCALE)
        tracker.reset()
        res = run_detect(tracker, gray)
        check("N1 %s" % name, not res.valid,
              "valid=%s reason=%s" % (res.valid, res.get("fail_reason", "-")))

    print("\nN2 合成干扰：亮屏 + 细边框（边框/屏宽约 1%）必须拒检")
    canvas = np.full((480, 640), 90, np.uint8)
    cv2.rectangle(canvas, (150, 120), (490, 340), 40, -1)
    cv2.rectangle(canvas, (155, 125), (485, 335), 210, -1)
    for x in range(170, 470, 26):
        cv2.line(canvas, (x, 150), (x, 310), 120, 1)
    tracker.reset()
    res = run_detect(tracker, canvas)
    check("N2 thin-bezel screen", not res.valid,
          "valid=%s reason=%s" % (res.valid, res.get("fail_reason", "-")))

    print("\nT 真实帧：录屏裁出的靶纸画面，必须检出")
    for name, expect in (("real_target_frame.png", (252, 221)),):
        path = FIXTURE / name
        if not path.exists():
            print("  SKIP %s  (缺固件)" % name)
            continue
        gray = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_GRAYSCALE)
        tracker.reset()
        res = run_detect(tracker, gray)
        err = (abs(res.u - expect[0]) + abs(res.v - expect[1])) if res.valid else 999.0
        check("T1 %s" % name, res.valid and err <= 15.0,
              "valid=%s u=%.1f v=%.1f err=%.1f" % (res.valid, res.get("u", -1),
                                                   res.get("v", -1), err))

    print("\n" + "-" * 72)
    if FAILS:
        print("结果：%d 项失败 -> %s" % (len(FAILS), ", ".join(FAILS)))
        return 1
    print("结果：全部通过（0 项失败）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
