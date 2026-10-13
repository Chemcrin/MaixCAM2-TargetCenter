"""MaixCAM2 B 轨入口；应用生命周期由 TrackingApplication 管理。

main() 只负责构建号和运行。识别/闭环独占工作线程中的 UART；触摸仅投递
SELF-TEST/SEARCH 请求。上电不自动自检，CALIBRATE_AXIS 非空时只做开环标定。

模块：app_runtime 装配与退出释放，vision_worker 最新帧识别，aim_control
调度视觉/搜索/IMU，preview 本地屏与触摸，maixvision_stream 独立进程图传。
rectangle_center/laser_spot 产生测量，stepper_link/gimbal_aim 执行有限动作。
rate_meter/telemetry 分别记录采图、识别和显示帧率。

在 MaixVision 运行完整项目；运行当前文件请用生成的 dist/main_single.py。
旧私有别名保留供现有工具调用；配置和标定步骤见 README 与硬件标定文档。
"""

import os
import sys
import time

import cv2
import numpy as np


def _root():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return os.getcwd()


ROOT = _root()
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.rectangle_center import draw_result
from vision.laser_spot import draw_laser_overlay, PANEL_H
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import probe_imu
from vision.gimbal_calibrate import GimbalCalibrator
from vision.rate_meter import FrameRate
from vision.camera_source import to_gray
from vision.vision_worker import LatestFrameWorker
from vision.preview import (Preview, TouchButtons, display_result,
                            apply_maixvision_stream_policy)
from vision.aim_control import VisionControl, send_uart
from target_api import TrackingApplication
from vision.telemetry import format_perf_line, format_slow_loop, MAIN_LOOP_SLOW_MS

BUILD_ID = "2026-10-13-canny-kalman-v0.8.0"

# ---- 兼容别名（v0.4.6 模块化后保留旧私有名）--------------------------------
# tools/test_runtime_performance.py 直接 `from main import (_FrameRate, _LatestVision,
# _Preview, _VisionControl, _display_result, _apply_maixvision_stream_policy)`，
# test_main_self_test.py 通过 `import main as entry` 跑真实 main()。保留下列别名后，
# 两个入口测试**零改动**即可通过（实现已下沉到 vision/ 各模块）。
_FrameRate = FrameRate
_LatestVision = LatestFrameWorker
_Preview = Preview
_Touch = TouchButtons
_VisionControl = VisionControl
_display_result = display_result
_to_gray = to_gray
_send_uart = send_uart
_apply_maixvision_stream_policy = apply_maixvision_stream_policy


def main():
    """硬件装配、按钮、线程与清理由可复用应用 API 管理。"""
    return TrackingApplication(Config, BUILD_ID, clock=time,
                               controller_factory=VisionControl).run()


def _pc_hint(exc):
    """PC 上没有 MaixPy SDK（`maix`）时的指引：把"跑不了"变成"知道该跑什么"。"""
    return (
        "\n[pc] 本入口需要 MaixCAM2 板载 SDK（import maix），PC 上无法运行：%s\n"
        "[pc] 在 Windows / Linux 桌面上验证识别，请改用：\n"
        "       \"D:/Python311/python.exe\" -B tools/check_pc.py                 # 全部离线检查\n"
        "       \"D:/Python311/python.exe\" -B tools/run_pc.py --images <图片/目录>\n"
        "       \"D:/Python311/python.exe\" -B tools/diagnose_detect.py <图片>     # 逐边诊断为何没检出\n"
        "     板端运行：MaixVision 打开本目录选「运行项目」，或运行 dist/main_single.py。\n"
        % exc)


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as exc:
        if getattr(exc, "name", "") == "maix":
            raise SystemExit(_pc_hint(exc))
        raise
