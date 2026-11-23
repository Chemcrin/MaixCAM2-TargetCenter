"""云台开环标定（板端运行）——量出 PULSE_PER_PX 与 DIR_SIGN。

为什么需要它：闭环把"像素误差"换算成"脉冲数"，靠的就是 `PULSE_PER_PX`
（1 像素差要发几个脉冲）。这个值取决于镜头视场、细分、减速比，算不出来，
只能实测。方向符号 `DIR_SIGN_*` 同理，接反了会越追越偏。

做法（每轴四步，全程自动）：

    1. 记住靶心当前位置；
    2. 朝正方向走 N 个脉冲，等停稳，再量靶心 → 得到"正走位移"；
    3. 反向走 N（回到起点附近）→ 顺便量回程误差，即齿隙；
    4. 再反向走 N → 得到"反走位移"；最后正向走 N 回起点。

    PULSE_PER_PX = 2N / (|正走位移| + |反走位移|)
    DIR_SIGN     = +1（正走让坐标变小）/ -1（正走让坐标变大）

跑法：MaixVision 打开 `dist/calib_gimbal.py` →「运行当前文件」。
      也可在模块化工程里把本文件当入口，但要用「运行项目」。

⚠ 运行前把靶纸完整放进画面。画面左上角会实时显示识别状态。
⚠ 探针幅度小（约 5~10°）、速度慢（30°/s），不会甩头；中途丢目标会原路退回。
⚠ 结果只打印在控制台，不会自动改 config.py —— 确认后手工填回。
"""
import os

# 关掉 MaixCDK 内置图传：标定脚本自己建本地屏，避免"连接未就绪"时 show() 忙等 500ms
os.environ.setdefault("MAIXVISION", "0")

import time

import cv2
import numpy as np

from config import Config as cfg
from vision.logic import RectangleCenterDetector
from vision.zdt import StepperLink, estimate_move_seconds

PROBE_PULSES = 80          # 起始探针脉冲数（3200 细分下约 9°）
PROBE_DEG_S = 30.0         # 探针速度：慢，肉眼能跟
PROBE_RAMP_S = 0.15        # 起停斜坡
SETTLE_S = 0.45            # 停稳后再量（齿隙回弹要时间）
MEASURE_FRAMES = 4         # 每次量几帧取平均
VALID_FRAMES = 6           # 起始锁定需要连续几帧
MAX_SPREAD_PX = 3.0        # 这几帧的中心最大离散度
WorstPxPerPulse = 4.0      # 安全余量：假设 1 脉冲最多走 4px，防止探针把靶纸推出画面


def _mean(points):
    return (sum(p[0] for p in points)/len(points), sum(p[1] for p in points)/len(points))


def _spread(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return max(max(xs)-min(xs), max(ys)-min(ys))


class Viewer:
    """相机 + 本地屏 + 标注。只做标定用，不复用主程序的显示线程。"""

    def __init__(self):
        from maix import camera, display, image
        self.image = image
        self.cam = camera.Camera(cfg.CAMERA_W, cfg.CAMERA_H,
                                 image.Format.FMT_GRAYSCALE, fps=30)
        self.cam.skip_frames(10)
        self.disp = display.Display()
        self.n = 0

    def read(self):
        img = self.cam.read()
        gray = self.image.image2cv(img, ensure_bgr=False, copy=False)
        gray = np.asarray(gray)
        if gray.ndim == 3:
            gray = gray[:, :, 0]
        return np.ascontiguousarray(gray)

    def show(self, gray, res, line1="", line2=""):
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        ok = bool(res and res.get("valid"))
        if ok:
            u, v = float(res["u"]), float(res["v"])
            cv2.drawMarker(canvas, (int(round(u)), int(round(v))), (255, 0, 0),
                           cv2.MARKER_CROSS, 18, 2)
            cv2.putText(canvas, "target (%d,%d)" % (round(u), round(v)),
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
        else:
            reason = (res or {}).get("fail_reason") or (res or {}).get("stage") or "-"
            cv2.putText(canvas, "NO TARGET  " + str(reason), (8, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
        if line1:
            cv2.putText(canvas, line1, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, .45,
                        (0, 255, 255), 1)
        if line2:
            cv2.putText(canvas, line2, (8, 74), cv2.FONT_HERSHEY_SIMPLEX, .45,
                        (255, 220, 0), 1)
        self.disp.show(self.image.cv2image(canvas, bgr=True, copy=True))
        self.n += 1


def _grab(viewer, det):
    gray = viewer.read()
    return gray, det.process(gray, timestamp=time.time())


def _wait_target(viewer, det, app):
    """等画面里出现稳定黑框，返回 (u, v)；超时/用户退出返回 None。"""
    print("[calib] 等待目标：把靶纸完整放进画面")
    points, last_log = [], 0.0
    while not app.need_exit():
        gray, res = _grab(viewer, det)
        viewer.show(gray, res, "waiting for target", "keep the whole frame in view")
        if res.get("valid") and not res.get("hold"):
            points.append((float(res["u"]), float(res["v"])))
            if len(points) > VALID_FRAMES:
                points.pop(0)
            if len(points) == VALID_FRAMES and _spread(points) <= MAX_SPREAD_PX:
                center = _mean(points)
                print("[calib] 锁定目标 u=%.1f v=%.1f" % center)
                return center
        else:
            points = []
        if time.time()-last_log > 1.0:
            last_log = time.time()
            print("[calib] 还没看到黑框（%s）" %
                  (res.get("fail_reason") or res.get("stage") or "-"))
    return None


def _measure_center(viewer, det, app, timeout=1.5):
    """连量几帧取平均；超时或丢目标返回 None。"""
    points = []
    end = time.time()+timeout
    while time.time() < end and not app.need_exit():
        gray, res = _grab(viewer, det)
        viewer.show(gray, res, "measuring", "%d/%d" % (len(points), MEASURE_FRAMES))
        if res.get("valid") and not res.get("hold"):
            points.append((float(res["u"]), float(res["v"])))
            if len(points) >= MEASURE_FRAMES:
                return _mean(points)
    return None


def _move_wait(link, axis, ccw, pulses):
    ok, rpm, acc = link.move_smooth(axis, ccw, pulses, PROBE_DEG_S, PROBE_RAMP_S)
    if not ok:
        print("[calib] %s 下发失败" % axis)
        return False
    wait = estimate_move_seconds(pulses, rpm, acc,
                                 getattr(cfg, "MOTOR_PULSES_PER_REV", 3200)) + SETTLE_S
    time.sleep(max(0.05, wait))
    return True


def _choose_probe(center, axis):
    """按靶心到画面边缘的距离定探针幅度，避免把靶纸推出画面。"""
    u, v = center
    margin = min(u, cfg.CAMERA_W-u) if axis == "yaw" else min(v, cfg.CAMERA_H-v)
    limit = max(12, int(margin*0.6/WorstPxPerPulse))
    return max(8, min(PROBE_PULSES, limit))


def _probe_axis(viewer, det, link, app, axis, center):
    """一轴四步探针。返回 (pulse_per_px, dir_sign, backlash_px, d_plus, d_minus) 或 None。"""
    comp = 0 if axis == "yaw" else 1
    n = _choose_probe(center, axis)
    print("[calib] %s 探针 %d 脉冲（约 %.1f°）" % (axis, n, n*360.0/cfg.MOTOR_PULSES_PER_REV))
    start = center

    if not _move_wait(link, axis, True, n):
        return None
    plus = _measure_center(viewer, det, app)
    if plus is None:
        print("[calib] %s 正走后丢目标，原路退回" % axis)
        _move_wait(link, axis, False, n)
        return None

    if not _move_wait(link, axis, False, n):
        return None
    back = _measure_center(viewer, det, app)

    if not _move_wait(link, axis, False, n):
        return None
    minus = _measure_center(viewer, det, app)
    if minus is None:
        print("[calib] %s 反走后丢目标，回到起点" % axis)
        _move_wait(link, axis, True, n)
        return None

    _move_wait(link, axis, True, n)          # 回起点

    d_plus = plus[comp]-start[comp]
    d_minus = minus[comp]-start[comp]
    backlash = abs(back[comp]-start[comp]) if back is not None else float("nan")
    travel = abs(d_plus)+abs(d_minus)
    if travel < 6.0:
        print("[calib] %s 位移太小（%.1fpx），结果不可信；换更大的靶或更近的距离重试" %
              (axis, travel))
        return None
    k = 2.0*n/travel
    sign = 1 if d_plus < 0 else -1
    print("[calib] %s  n=%d  Δ+=%+.1fpx  Δ-=%+.1fpx  回程差=%.1fpx" %
          (axis, n, d_plus, d_minus, backlash))
    print("[calib] %s  PULSE_PER_PX=%.4f  DIR_SIGN=%+d" % (axis, k, sign))
    return k, sign, backlash, d_plus, d_minus


def _fov_hint(k):
    """用标定值反推水平视场，做一次常识校验（40° 左右才像这颗镜头）。"""
    px_per_pulse = 1.0/max(1e-6, k)
    px_per_deg = px_per_pulse/(360.0/cfg.MOTOR_PULSES_PER_REV)
    f_px = px_per_deg*180.0/np.pi
    return 2.0*np.degrees(np.arctan((cfg.CAMERA_W/2.0)/max(1.0, f_px)))


def main():
    from maix import app
    viewer = Viewer()
    det = RectangleCenterDetector(cfg)
    link = StepperLink(cfg)
    print("[calib] 串口: yaw=%s(ID%d) pitch=%s(ID%d)" %
          (cfg.YAW_DEV, cfg.ADDR_YAW, cfg.PITCH_DEV, cfg.ADDR_PITCH))
    if not link.enable():
        print("[calib] 使能失败：检查串口/驱动器供电。中止。")
        link.close()
        return
    result = {}
    try:
        center = _wait_target(viewer, det, app)
        if center is None:
            print("[calib] 没有拿到稳定目标，中止")
            return
        for axis in ("yaw", "pitch"):
            got = _probe_axis(viewer, det, link, app, axis, center)
            if got is not None:
                result[axis] = got
            # 一轴测完，重新锁一次中心，避免另一轴从偏移位置起测
            again = _measure_center(viewer, det, app, timeout=2.0)
            if again is None:
                print("[calib] 重新锁定失败，中止")
                break
            center = again
        print("")
        print("=" * 62)
        print(" 标定结果（确认后手工填回 config.py）")
        print("=" * 62)
        if "yaw" in result:
            k, sign, back, dp, dm = result["yaw"]
            print("  PULSE_PER_PX_YAW = %.4f" % k)
            print("  DIR_SIGN_YAW     = %+d" % sign)
            print("  # 回程差 %.1fpx；反推水平视场约 %.0f°" % (back, _fov_hint(k)))
        else:
            print("  YAW 未测出，保持原值")
        if "pitch" in result:
            k, sign, back, dp, dm = result["pitch"]
            print("  PULSE_PER_PX_PITCH = %.4f" % k)
            print("  DIR_SIGN_PITCH     = %+d" % sign)
            print("  # 回程差 %.1fpx；反推水平视场约 %.0f°" % (back, _fov_hint(k)))
        else:
            print("  PITCH 未测出，保持原值")
        print("=" * 62)
        print("[calib] 提示：视场应在 30~90°；超出说明细分/减速比与假设不符。")
    finally:
        link.stop()
        link.close()
        print("[calib] 电机已停，串口已关")


if __name__ == "__main__":
    main()
