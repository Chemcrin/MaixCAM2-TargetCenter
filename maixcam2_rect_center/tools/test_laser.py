"""激光光斑读数回归（fixed 口径）：python -B tools/test_laser.py

光斑位置写死、不做图像检测，检查项：

  T0 常量状态：found=True / reason="fixed" / u,v = 标定值 / 零耗时
  T1 deviation() 符号约定：dx = u靶心 − u光斑，无效输入返回 None
  T2 raw 口径：use_raw=True 取 raw_u/raw_v
  T3 叠加绘制不越界，且确实画了面板
  T4 旧检测接口（LASER_MODE / process）确已移除
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from vision.laser_spot import FixedLaser, PANEL_H, deviation, draw_laser_overlay

SPOT = (Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL)
FAILED = []


def check(name, ok, detail=""):
    print("  %-4s %s%s" % ("OK" if ok else "FAIL", name, ("  " + detail) if detail else ""))
    if not ok:
        FAILED.append(name)


def main():
    print("=" * 72)
    print("激光光斑读数回归（fixed）  nominal=%s" % (SPOT,))
    print("=" * 72)

    # ---------------- T0 常量状态
    lz = FixedLaser(Config)
    st = lz.static_state()
    check("T0 常量状态 found/reason/u/v",
          st["found"] and st["reason"] == "fixed" and not st["updated"]
          and (st["u"], st["v"]) == SPOT, "u=%.0f v=%.0f" % (st["u"], st["v"]))
    check("T0' static_state() 返回共享常量（两次调用同一份）", lz.static_state() is st)

    # ---------------- T1 deviation 符号约定
    laser = dict(found=True, u=100.0, v=200.0)
    dv = deviation(laser, dict(valid=True, u=130.0, v=180.0))
    check("T1 deviation = 靶心-光斑", dv == (30.0, -20.0), "dv=%s" % (dv,))
    check("T1' 无靶纸时返回 None", deviation(laser, dict(valid=False)) is None)

    # ---------------- T2 raw 口径
    res = dict(valid=True, u=150.0, v=210.0, raw_u=130.0, raw_v=180.0)
    dv_raw = deviation(laser, res, use_raw=True)
    check("T2 use_raw=True 取 raw 中心（闭环口径）", dv_raw == (30.0, -20.0), "dv=%s" % (dv_raw,))

    # ---------------- T3 叠加绘制
    import cv2
    canvas = np.zeros((360, 480, 3), np.uint8)
    h = draw_laser_overlay(canvas, dict(found=True, u=SPOT[0], v=SPOT[1]),
                           dict(valid=True, u=SPOT[0] + 30, v=SPOT[1] - 20), 0.75)
    panel = canvas[0:PANEL_H, 0:200]
    check("T3 面板高度/绘制", h == PANEL_H and panel.sum() > 0,
          "h=%d 面板非零像素=%d" % (h, int((panel.sum(axis=2) > 0).sum())))
    canvas2 = np.zeros((360, 480, 3), np.uint8)
    h2 = draw_laser_overlay(canvas2, None, dict(valid=False), 0.75)
    check("T3' laser=None 时只返回 0 不绘制", h2 == 0 and canvas2.sum() == 0)

    # ---------------- T4 旧检测接口确已移除
    import vision.laser_spot as mod
    check("T4 检测接口已清理（无 process/_detect/LASER_MODE 依赖）",
          not hasattr(mod, "LaserSpotDetector") and not hasattr(FixedLaser, "process")
          and not hasattr(Config, "LASER_MODE") and not hasattr(Config, "LASER_ROI_RADIUS"))

    print("-" * 72)
    print("结果：%s（%d 项失败）" % ("全部通过" if not FAILED else "有失败", len(FAILED)))
    for name in FAILED:
        print("   失败：%s" % name)
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
