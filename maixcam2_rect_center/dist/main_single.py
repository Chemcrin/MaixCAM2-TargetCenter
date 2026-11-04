#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_single.py · 25E 靶心识别（轨道 B）—— 单文件自包含版
================================================================================
★ 本文件由 tools/build_single.py 自动生成，请勿手改！
    改参数 -> config.py ；改算法 -> vision/*.py ；改流程 -> main.py
    改完在本目录重新执行：  python tools/build_single.py

用途：在 MaixVision 里点「运行当前文件」即可运行（不必用「运行项目」）。
      行为与模块化工程完全一致，输出也就完全一致。

生成时间：2026-11-04 12:23:27
    config.py                    sha1=c6ffc424c54a
    main.py                      sha1=1f359a58abc8
    target_api.py                sha1=c37f98868fe9
    vision/__init__.py           sha1=abc847cb4264
    vision/aim_control.py        sha1=6dab67c57421
    vision/app_runtime.py        sha1=405c903a4834
    vision/camera_source.py      sha1=b398fedefe6e
    vision/gimbal_aim.py         sha1=543b2d46a1e8
    vision/gimbal_calibrate.py   sha1=c7db6a254e40
    vision/gimbal_search.py      sha1=fc5c09964c19
    vision/gimbal_self_test.py   sha1=1094ca64bdc0
    vision/imu_heading.py        sha1=a1193ada2173
    vision/laser_spot.py         sha1=a9ff385db303
    vision/maixvision_stream.py  sha1=c30d784f26a0
    vision/preview.py            sha1=bae51cd6c3a7
    vision/rate_meter.py         sha1=6f2a054c97a0
    vision/rectangle_center.py   sha1=09f8c5551270
    vision/stepper_link.py       sha1=35bbfea2f5af
    vision/telemetry.py          sha1=a6627b2dc0a7
    vision/vision_worker.py      sha1=fcf416a9f29a
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': 'c6ffc424c54a7d37f48577fd173689de96c66b88', 'vision/rate_meter.py': '6f2a054c97a00470409a59de9dea42b0bff9c6ba', 'vision/camera_source.py': 'b398fedefe6eaf931d372abe00ba267f7d9697b5', 'vision/vision_worker.py': 'fcf416a9f29ab849861c59c314dc318f070278eb', 'vision/telemetry.py': 'a6627b2dc0a741030c33ab8c5d41e80df84d4dc9', 'vision/maixvision_stream.py': 'c30d784f26a0da91756e9a63f71ba795fdda7a8d', 'vision/preview.py': 'bae51cd6c3a7332bd2006b381efb68c258cf2b88', 'vision/rectangle_center.py': '09f8c5551270685632435bbc7ab44fd51912e6b1', 'vision/laser_spot.py': 'a9ff385db303d1699de76129bf036c508fe44702', 'vision/stepper_link.py': '35bbfea2f5afeed77dd7fa22a853f17c3a3d6982', 'vision/imu_heading.py': 'a1193ada2173f113452781bcebc5a5e5b8745db3', 'vision/gimbal_self_test.py': '1094ca64bdc09f087f659b197c1085767a050952', 'vision/gimbal_aim.py': '543b2d46a1e86c861e0ff4c7f4e481950b8a8242', 'vision/gimbal_search.py': 'fc5c09964c192bd9c68e7e837dc3dc37b9bc4579', 'vision/aim_control.py': '6dab67c574216816d723822c82d4a37e4e7fb881', 'vision/gimbal_calibrate.py': 'c7db6a254e404cadea5c1ddd892afe7ae8834aa3', 'vision/app_runtime.py': '405c903a483496880743275fae8c37d123c2459f', 'vision/__init__.py': 'abc847cb4264bf79d47c0dae40f584bf4baba64f', 'target_api.py': 'c37f98868fe9e96d2153587371aee8c2cd7b2bec', 'main.py': '1f359a58abc837a6abbd8d5f59a4fab2325b818c'}

# --------------------------------------------------------------------------
# ① 各模块源码原样嵌入，运行时装进 sys.modules（保持原 import 语义）
# --------------------------------------------------------------------------
_MOD_SRC = {}

# ===== config  (config.py) =====
_MOD_SRC['config'] = r'''
class Config:
    # ---- 相机 / 显示 ----
    CAMERA_W = 640
    CAMERA_H = 480
    CAMERA_FPS = 40          # 25ms 采图周期，给 30+fps 识别/显示留余量
    CAMERA_BUFFERS = 2
    SKIP_FRAMES = 10
    # 软件畸变校正默认关闭：未按本镜头标定会**引入**畸变（2026-10-10 实测定案）
    LENS_CORR = False
    LENS_STRENGTH = 1.5
    SHOW = True
    # 0 = 用屏幕原始宽度（推荐）：640×480 画布与相机同尺寸，驱动层不再缩放，触摸坐标 1:1
    SHOW_WIDTH = 0
    SHOW_FPS = 40
    SHOW_JPEG_QUALITY = 70   # 只影响 MaixVision 图传，检测始终用原始灰度图
    DISABLE_MAIXVISION_STREAM = False
    VISION_ASYNC = True      # 后台识别，忙时只保留最新帧，不积压旧图
    SHOW_ASYNC = True        # 独立显示线程：show() 慢只掉预览帧，不拖识别/串口/闭环
    RESULT_MAX_AGE_S = 0.20  # 以采图时刻计龄；过期结果不画框、不下发指令
    UART_RX_MAX_BYTES = 256
    UART_RX_BUFFER_MAX_BYTES = 1024
    ENABLE_TOUCH_BUTTON = True   # 屏幕右上角 SELF-TEST 按钮（点一下才自检）
    OPENCV_THREADS = 1
    LOG_INTERVAL = 2.0

    # ---- 双 ZDT 云台链路（轴映射 2026-10-09 实测定案，见 AGENTS.md §2）----
    #   yaw   = UART2 / B0,B1   / /dev/ttyS2 / 驱动器 ID=2
    #   pitch = UART0 / A28,A29 / /dev/ttyS0 / 驱动器 ID=1
    ENABLE_GIMBAL = True
    YAW_DEV, YAW_BAUD = "/dev/ttyS2", 115200
    YAW_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
    PITCH_DEV, PITCH_BAUD = "/dev/ttyS0", 115200
    # ⚠ U0 必须显式 pinmap（空表静默不通）；ttyS0 还是系统日志口，故开机清 RX
    PITCH_PINS = {"A28": "UART0_TX", "A29": "UART0_RX"}
    ADDR_YAW, ADDR_PITCH = 2, 1
    PINMAP_STRICT = False    # 失败只告警，避免把真故障藏成"打不开"
    FLUSH_RX_ON_START = True
    # 闭环方向符号：两轴正方向均 CCW（yaw CCW=左 / pitch CCW=上）。
    # 相机与激光同装云台 ⇒ 云台转过去时靶心在画面里反向移动，故取 -1。
    # ⚠ 未经标定：上电若"越追越偏"，把对应项翻成 +1。
    DIR_SIGN_YAW = -1
    DIR_SIGN_PITCH = -1
    PULSE_PER_PX_YAW = 3.67       # ★ 主参数，必须用实机开环标定替换
    PULSE_PER_PX_PITCH = 3.67
    GAIN_YAW = 0.5                # 被 AIM_PROFILE 覆盖，实际值见文件末尾 profiles
    GAIN_PITCH = 0.5              # 同上
    DEADBAND_PX = 1.0             # 每轴死区；应 ≥1.5× 实测回差
    MAX_STEP_PULSES = 360         # 同上，被 profile 覆盖
    RPM_FINE = 300                # 同上，被 profile 覆盖
    # 曲线加减速档位：0 = 瞬时启停（勿用）；≠0 时每 (256-acc)*50us 变 1RPM，越小越柔
    ACC_DEFAULT = 80
    COMMAND_INTERVAL_S = 0.08

    # ---- 上电自检 ----
    GIMBAL_SELF_TEST = True
    SELF_TEST_YAW_DEG = 15.0
    SELF_TEST_PITCH_DEG = 30.0
    SELF_TEST_PULSES = 400        # >0 时按固定脉冲走，不依赖"脉冲/度"标定
    YAW_PULSES_PER_DEG = 0.0
    PITCH_PULSES_PER_DEG = 0.0
    YAW_POSITIVE_CCW = True
    PITCH_POSITIVE_CCW = True
    SELF_TEST_RPM = 900
    SELF_TEST_SETTLE_S = 0.35
    MOTOR_PULSES_PER_REV = 3200
    GIMBAL_GEAR_RATIO = 0.0       # 未知留 0：自检不猜齿轮比

    # ---- 闭环口径 ----
    # True = 用 raw_u/raw_v 算 dx/dy：新链路无滤波，两者恒等；保留该开关以兼容闭环代码
    AIM_USE_RAW = True
    JUMP_REJECT_PX = 400.0        # 离群拒收：先当误检丢掉
    JUMP_CONFIRM_FRAMES = 3       # 连续这么多帧才采信跳变
    COARSE_PX = 24.0              # |e| 超过它切粗调
    GAIN_COARSE = 0.9             # 被 profile 覆盖；必须 ≤1，否则增量闭环不收敛
    RPM_COARSE = 1200             # 被 profile 覆盖

    # ---- 链路健康 ----
    ENABLE_ACK_WATCHDOG = True
    E2_REENABLE = True            # 收到 E2（未使能/堵转）自动补发一次 0xF3
    E2_WINDOW_S = 2.0
    E2_FAULT_COUNT = 10
    RX_TIMEOUT_MS = 300           # 只在收到过确认帧后生效；0 = 关闭
    RX_TIMEOUT_LIMIT = 20

    # ---- 失锁重捕获（默认关）----
    ENABLE_SCAN = False
    LOST_FRAMES = 15
    SCAN_STEP_DEG = 1.5
    SCAN_LIMIT_DEG = 15.0
    SCAN_STEP_PULSES = 40
    SCAN_LIMIT_PULSES = 400
    SCAN_RPM = 60
    SCAN_STEP_INTERVAL_S = 0.15

    # ---- 开环标定（一次性；标完把 CALIBRATE_AXIS 置回 ""）----
    CALIBRATE_AXIS = ""
    CALIBRATE_PULSES = 2000
    CALIBRATE_RPM = 300
    CALIBRATE_SAMPLES = 5
    CALIBRATE_SETTLE_S = 1.0
    CALIBRATE_TIMEOUT_S = 20.0

    # 旧版纯视觉 ASCII 输出，仅 ENABLE_GIMBAL=False 的降级路径使用
    ENABLE_UART = False
    UART_PORT = "/dev/ttyS0"
    UART_BAUD = 115200

    # ---- 识别（自适应反色二值化 → 四层过滤）----
    # 调参顺序：先 RECT_BLOCK_SIZE / RECT_C 让纸面成块、胶带成环，再 RECT_MIN_AREA 去碎片，
    # 再 RECT_APPROX_EPS 稳四边形，再 RECT_MIN/MAX_ASPECT 收紧形状，最后 RECT_MIN_BAND_SCORE
    # 压误检（它挡窗框/屏幕/桌沿）；全部调完再看 RECT_PRIOR_* 的复锁行为。
    RECT_MIN_AREA = 300           # 最小轮廓面积（px2）；先小后大
    RECT_MAX_AREA_RATIO = 0.92    # 面积上限（占整幅比例）：挡“锁到整面墙/门框”
    RECT_APPROX_EPS = 0.02        # 多边形近似精度（占周长比例）
    RECT_MIN_ASPECT = 1.05        # 四边形对边长度比下限；与横竖放、旋转无关
    RECT_MAX_ASPECT = 1.95        # 上限；目标约 1.414，透视会把比值推开
    RECT_GAUSS_KERNEL = (5, 5)    # 高斯核；(0, 0) = 跳过模糊
    RECT_BLOCK_SIZE = 41          # 自适应阈值窗口，必须为 >=3 的奇数
    RECT_C = 8.0                  # 自适应阈值常数：调小补断边，调大压噪点
    RECT_MAX_CANDIDATES = 32      # 候选上限，防病态画面拖慢
    # 第四层：胶带暗带证据。边界外侧要比内侧暗这么多才算一条边成立。
    RECT_MIN_BAND_SCORE = 0.55    # 成立边的占比下限；0 = 关闭该层
    RECT_BAND_OFFSET_PX = 7.0     # 内外采样点离边界的距离；应小于胶带可见厚度
    RECT_BAND_CONTRAST = 10.0     # 灰度差门限；现场偏暗时先降它再降占比门
    RECT_MIN_BAND_RATIO = 0.045   # 暗带厚度 / 边长 下限：胶带约 0.08~0.09，显示器边框约 0.01~0.02
    RECT_BAND_MIN_UNIFORM = 0.40  # 四边厚度 min/max 下限（≥3 条边有效才判）
    RECT_MIN_INNER_RATIO = 0.60   # 框内均值 / 全图均值 的下限：挡“暗设备面板/黑窗口”这类暗框
    RECT_MIN_INNER_FLOOR = 45.0   # 上面那个比例判据的绝对兜底，防止全图极暗时门限过低
    RECT_MIN_EDGE_SCORE = 0.60    # 至少三条边的暗带占比要达到这个值（第四条可被遮挡）
    RECT_BAND_RISE_STEP = 18.0    # 暗带外沿的灰度回升门限：相邻像素涨这么多就算出了胶带
    RECT_BAND_PROBE_RATIO = 0.15  # 向外最多探测短边的这个比例
    RECT_BAND_CANDIDATES = 12     # 只对面积最大的这些候选做暗带证据（整帧耗时大头）
    RECT_DEBUG_BAND = False       # 真：失败帧的 det_fail 里附候选判据读数（板端定位用）

    # 角点精修：沿边拟合直线再求交，抑制顶点逐帧跳动
    RECT_EDGE_REFINE = True
    RECT_EDGE_BAND_PX = 3.0       # 参与拟合的轮廓点到边线的最大距离
    RECT_EDGE_MIN_POINTS = 8      # 每条边至少要这么多点，否则放弃精修
    RECT_EDGE_MAX_SHIFT_PX = 4.0  # 精修位移上限，超过就退回粗角点
    # 跨帧先验：只影响择优与复锁，不改单帧判据
    RECT_PRIOR_FRAMES = 15        # 丢失超过这么多帧就放弃位置先验
    RECT_PRIOR_RADIUS_PX = 140.0  # 先验半径；候选中心落在圈内才优先
    # 锁定后的窄带精修：沿四条边的法向搜索灰阶跃变，比全图重搜便宜且端点连续
    RECT_TRACK_RADIUS_PX = 12.0   # 每条边向内外各搜这么多像素
    RECT_TRACK_SAMPLES = 16       # 每条边的采样点数（够拟合一条直线）
    RECT_TRACK_MIN_STEP = 6.0     # 单步灰度下降门限；偏暗场景先降它
    RECT_TRACK_MAX_SHIFT_PX = 14.0 # 精修位移上限，超过就判这一帧跟踪失败（应 >= 帧间最大位移）
    RECT_TRACK_MAX_FRAMES = 40    # 连续跟踪这么多帧后强制回一次全图，防漂移
    RECT_KEEP_STAGES = False      # 真：把 blur/binary 留在 last_stages，供 --stage 导出
    RECT_DEBUG_STAGE = "result"   # none / blur / binary / result

    # ---- 激光光斑读数（相机与激光固连，光斑位置近似固定）----
    # "fixed"：不检测，每帧只算"黑框中心 − 标定常量"，零开销（默认）
    # "detect"：每 LASER_INTERVAL_S 实测一次，用于重标 U/V
    ENABLE_LASER = True
    LASER_U_NOMINAL = 294
    LASER_V_NOMINAL = 202
    LASER_ROI_RADIUS = 64
    LASER_MODE = "fixed"
    LASER_MIN_PEAK = 140.0
    LASER_MIN_CONTRAST = 20.0
    LASER_MIN_PIXELS = 2
    LASER_MAX_PIXELS = 160        # 面积上限挡白墙高光/灯罩反光
    LASER_INTERVAL_S = 0.30
    LASER_MAX_JUMP_PX = 40.0      # 单次更新跳变上限，0 = 关闭
    LASER_SMOOTH_ALPHA = 0.60

    # ---- 瞄准调度 ----
    AIM_PROFILE = "smooth"        # 见文件末尾 profiles：覆盖 6 个增益/转速/限幅
    AIM_PROGRESSIVE = True
    ACC_TRACK = 80
    AIM_WAIT_MOTION = True
    AIM_MOTION_MARGIN_S = 0.03
    AIM_HISTORY_MAX_GAP_S = 0.30
    DIVERGE_GUARD = True          # 发散兜底停机
    DIVERGE_CONFIRM_FRAMES = 6
    DIVERGE_RATIO = 1.15
    DIVERGE_MIN_PX = 4.0
    GIMBAL_CALIBRATED = True
    CALIBRATE_VISUAL_TIMEOUT_S = 3.0
    SELF_TEST_TIMEOUT_S = 10.0
    ENABLE_SEARCH_BUTTON = True
    SEARCH_YAW_DEG = SEARCH_PITCH_DEG = 45.0
    SEARCH_RPM = SEARCH_ACC = 60
    SEARCH_SETTLE_S = 0.10
    SEARCH_TIMEOUT_S = 60.0
    SEARCH_PULSES = 400
    SEARCH_USE_IMU = True
    SEARCH_IMU_MARGIN_DEG = 3.0

    # ---- IMU 前馈（默认关）----
    ENABLE_IMU_FEEDFORWARD = False
    IMU_YAW_AXIS = (0, 0, 1)
    IMU_PITCH_AXIS = (0, -1, 0)
    IMU_MAX_GAP_S = 0.20
    IMU_MAX_AGE_S = 0.20
    IMU_SAMPLE_HZ = 208
    IMU_MOUNT = "gimbal"
    IMU_BIAS_CALIBRATE_S = 1.0
    IMU_BIAS_MAX_STD_DPS = 0.25
    IMU_BIAS_MAX_DPS = 2.0
    IMU_HOLD_MAX_S = 0.25

    # ---- MaixVision 图传桥 ----
    SHOW_STREAM_HOST = "127.0.0.1"
    SHOW_STREAM_PORT = 7899
    SHOW_STREAM_WIDTH = 480
    SHOW_STREAM_SLOW_MS = 300
    SHOW_STREAM_RETRY_S = 2.0
    SHOW_STREAM_RETRY_MAX_S = 15.0


def apply_aim_profile(cfg):
    """Only these six values belong to a profile; unknown names fail at startup."""
    profiles = {"smooth": (.30, .30, .65, 60, 150, 80),
                "responsive": (.50, .50, .90, 120, 400, 360)}
    name = cfg.AIM_PROFILE
    if name not in profiles:
        raise ValueError("unknown AIM_PROFILE: %s" % name)
    for key, value in zip(("GAIN_YAW", "GAIN_PITCH", "GAIN_COARSE", "RPM_FINE", "RPM_COARSE", "MAX_STEP_PULSES"), profiles[name]):
        setattr(cfg, key, value)
    return cfg


apply_aim_profile(Config)
'''

# ===== vision.rate_meter  (vision/rate_meter.py) =====
_MOD_SRC['vision.rate_meter'] = r'''
"""滑动窗口帧率计（从 main.py 原样搬迁，v0.4.6 模块化）。

本模块无第三方依赖、无 I/O，可在 PC 上单独单测（tools/test_runtime_performance.py 的 test_real_fps）。
"""
from collections import deque

import numpy as np


class FrameRate:
    """滑动窗口帧率计。按"成功完成的事件时刻"计时；返回 (平均fps, low1)。

    本类被三处复用（各自独立实例，口径不同、切勿混读）：
        capture_rate —— 主循环 cam.read() 速率   → [perf] capture=
        preview.rate —— 显示 show() 完成速率      → [perf] disp= / 屏幕文字 disp
        worker.rate  —— 识别帧处理完成速率        → [perf] detect_fps=

    low1 = "最慢 1% 帧间隔的倒数"：用于暴露**卡顿**——平均值好看但偶发长停顿（如 JPEG 图传
    抖动）时，fps 可能仍高，而 low1 会被那几帧长间隔拉低，是比平均值更诚实的体感指标。
    """

    def __init__(self):
        self.total = 0
        self._times = deque(maxlen=120)   # 只留最近 120 个时刻，足够覆盖 2s 窗口

    def record(self, now):
        self.total += 1
        self._times.append(float(now))

    def values(self, now):
        ts = list(self._times)
        if len(ts) < 2:
            return 0.0, 0.0
        # 只看最近两秒内的时刻：统计窗口滚动，旧的读数随时间长自然淡出。
        # 关键：把"距上一个事件已过去的时间(stall)"也算作一次间隔 —— 否则一旦卡住
        # （长时间没有新事件），窗口里只剩早先的高速样本，会把上次的快读数永久挂在屏幕上。
        recent = [t for t in ts if t >= float(now) - 2.0]
        if len(recent) < 2:
            return 0.0, 0.0
        gaps = np.diff(recent)
        stall = float(now) - recent[-1]
        if stall > gaps.max():
            gaps = np.append(gaps, stall)          # 把"当前这段停顿"并入间隔序列
        fps = (len(recent) - 1) / max(1e-6, float(now) - recent[0])   # 平均帧率
        slow = np.sort(gaps)[-max(1, int(np.ceil(len(gaps) * 0.01))):]  # 最慢的 1% 间隔
        return fps, 1.0 / max(1e-6, float(np.mean(slow)))              # low1
'''

# ===== vision.camera_source  (vision/camera_source.py) =====
_MOD_SRC['vision.camera_source'] = r'''
"""相机取帧辅助：MaixPy 帧 → 灰度 ndarray（从 main.py 原样搬迁，v0.4.6 模块化）。

`to_gray` 与 maix 运行时无关（只依赖 numpy / cv2 与调用方传入的 image 模块），
因此可在 PC 上以假 image 模块单测。相机对象的创建仍留在入口 main()（见 AGENTS.md 装配约定）。
"""
import cv2


def to_gray(img, image_module):
    """MaixPy 相机帧 → cv2 灰度 ndarray（H×W，uint8）。

    优先走 `image2cv(..., copy=False)` **零拷贝**借用相机缓冲（不额外占内存、最快），
    因此返回的数组在下一次 cam.read() 前必须用完；跨线程传递前务必先 copy（见 LatestFrameWorker.submit）。
    按通道数兜底：单通道直接取、3 通道按 RGB2GRAY、4 通道按 RGBA2GRAY；
    都不能识别时退回 copy=True 的 BGR 再转灰度。
    """
    a = image_module.image2cv(img, ensure_bgr=False, copy=False)
    if a.ndim == 2:
        return a
    if a.ndim == 3 and a.shape[2] == 1:
        return a[:, :, 0]
    if a.ndim == 3 and a.shape[2] == 3:
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    if a.ndim == 3 and a.shape[2] == 4:
        return cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    b = image_module.image2cv(img, ensure_bgr=True, copy=True)
    if b.ndim == 2:
        return b
    return cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
'''

# ===== vision.vision_worker  (vision/vision_worker.py) =====
_MOD_SRC['vision.vision_worker'] = r'''
"""识别工作线程 + 最新帧槽位（从 main.py 原样搬迁，v0.4.6 模块化）。

线程模型（时间轴 B）：主线程 submit() 最新帧 → 识别线程取走处理 → 结果写回最新槽位，
主线程 snapshot() 非阻塞取回。**最新优先、丢弃旧帧**是刻意设计（见类 docstring）。

★ clock 注入：构造可传 `clock`（默认标准库 time）。入口 main() 会把自身模块的 time
传进来，从而让 tools/test_main_self_test.py 的假时钟（patch main.time）能穿透到本模块，
使端到端用例在模拟时间轴下可复现。不传时行为与搬迁前完全一致（真 time）。
"""
import threading
import time
from collections import deque

import numpy as np

from .rate_meter import FrameRate


class LatestFrameWorker:
    """单识别线程 + 两个"最新槽位"：一个待处理帧槽(_pending)、一个结果槽(_latest)。

    线程模型（时间轴 B 的实现，见 main.py 模块开头总纲）：
        · 主线程（生产者）：submit() 往 _pending 放**最新一帧**的副本；
        · 识别线程（消费者）：取走 _pending → callback() 处理 → 结果写 _latest；
        · 主线程（读者）：snapshot() 无锁等待地取回 _latest（可能滞后于当前摄像头帧）。

    "最新优先、丢弃旧帧"是刻意的：识别跟不上时保留最新画面才有意义，积压旧帧只会让
    追踪越追越偏。Camera / Display / 触摸这些**必须独占主线程**的资源一概不跨线程。
    """

    def __init__(self, callback, idle=None, asynchronous=True, clock=None):
        self.callback, self.idle = callback, idle
        self._clock = clock if clock is not None else time
        self._condition = threading.Condition()   # 保护 _pending / _latest / _stop
        self._pending = self._latest = None
        self._stop = False
        self.error = None
        self._snapshot_cache = (None, 0., 0.)
        self.dropped = 0                          # 因来不及处理而被覆盖的帧数（丢帧计数）
        self.rate = FrameRate()                   # 识别帧率（detect_fps 口径）
        self.valid_rate = FrameRate()
        self._durations = deque(maxlen=120)
        self.thread = None
        if asynchronous:
            # daemon=True：进程退出时线程自动结束，不必显式 join 也能安全收摊。
            self.thread = threading.Thread(target=self._run, name="rect-vision", daemon=True)
            self.thread.start()

    def submit(self, gray, captured_at):
        # ★ 必须拷贝：image2cv(copy=False) 借用的相机内部缓冲会被**下一次 read 覆写**，
        #   识别线程还没处理完就可能已被改写 → 后台线程必须持有自己的一份灰度图。
        item = (np.array(gray, dtype=np.uint8, order="C", copy=True), float(captured_at))
        if self.thread is None:
            self._process(item)          # 同步模式（VISION_ASYNC=False）：主线程内直接算
            return
        with self._condition:
            if self._stop:
                return
            if self._pending is not None:
                self.dropped += 1        # 上一帧还没被取走就被覆盖 → 记一次丢帧
            self._pending = item
            self._condition.notify()     # 唤醒识别线程

    def _process(self, item):
        started = self._clock.perf_counter()
        snapshot = self.callback(*item)  # callback = VisionControl.process（识别+偏差+闭环）
        # Keep the owned source buffer with its result: old geometry must never
        # be overlaid on a newer moving target image.
        snapshot = dict(snapshot, frame=item[0])
        completed = self._clock.perf_counter()
        with self._condition:
            self._latest = snapshot
            self.rate.record(completed)
            if snapshot.get('detector_result',snapshot.get('result',{})).get('valid'):
                self.valid_rate.record(completed)
            self._durations.append((completed-started)*1000)

    def _run(self):
        """识别线程主体：循环取最新帧处理；无帧时走 idle() 维持串口保活。"""
        try:
            while True:
                with self._condition:
                    if self._pending is None and not self._stop:
                        # 20ms 超时轮询（而非无限 wait），保证 stop 能被及时看到。
                        self._condition.wait(timeout=0.02)
                    if self._stop:
                        return
                    item, self._pending = self._pending, None
                if item is not None:
                    self._process(item)
                elif self.idle is not None:
                    # 有帧就处理、没帧就保活：空闲时继续收 ACK/推进按钮自检，但不重发旧误差。
                    self.idle()
        except Exception as exc:
            # 识别线程异常不能静默：记下原因，主循环下一轮会读到 error 并抛出终止。
            with self._condition:
                self.error = repr(exc)
                self._latest = None
            print("[vision] worker failed: %s" % exc)

    def snapshot(self, now):
        """取回最近一次识别结果 + 识别帧率（主线程调用，非阻塞）。"""
        if not self._condition.acquire(False):
            return self._snapshot_cache
        try:
            fps, low1 = self.rate.values(now)
            self._snapshot_cache = self._latest, fps, low1
            return self._snapshot_cache
        finally:
            self._condition.release()

    def close(self):
        """停线程；必须等它真正退出——UART 归识别线程独占，未退干净就动硬件会打架。"""
        with self._condition:
            self._stop = True
            self._pending = None
            self._condition.notify()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                # 线程没停就动 UART/相机是竞态；宁可抛错让上层看见，也不静默降级。
                raise RuntimeError("vision worker did not stop; UART remains owned by worker")

    def metrics(self, now):
        """按需取有效识别吞吐和识别+控制P95；不在逐帧热路径计算分位数。"""
        with self._condition:
            fps, low1 = self.valid_rate.values(now)
            durations = list(self._durations)
        return dict(valid_fps=fps,valid_low1=low1,
                    worker_p95_ms=float(np.percentile(durations,95)) if durations else 0.)
'''

# ===== vision.telemetry  (vision/telemetry.py) =====
_MOD_SRC['vision.telemetry'] = r'''
"""周期日志组装（从 main.py 抽出，v0.4.6 模块化）。

只做字符串拼装，不碰任何硬件/时间源 —— 便于单独审阅"日志口径"是否与代码一致。
★ 日志文案逐字冻结（tools/test_main_self_test.test_perf_log_branch_formatting 会校验字段），
  改这里等于改板端日志格式，务必同步 README 的读法说明。
"""
# 主循环单轮超过该毫秒数（不含 show）即判为"慢"，打一次归因提示。
MAIN_LOOP_SLOW_MS = 66.0


def format_perf_line(build_id, capture_fps, display_fps, display_low1, vision_fps,
                     result, read_avg, draw_avg, loop_avg,
                     show_avg, show_p95, show_max, show_n,
                     age_s, shown, drop_show, drop_vision, detector_result=None):
    """拼装 [perf] 行。

    读法：[perf] 里 capture / disp / detect_fps 是三路**不同**口径 ——
      "眼睛看到的刷新率"看 disp，"识别吞吐"看 detect_fps，别互相换算。
      read/draw/loop 属于**主循环**，show 属于**显示线程** —— 分开看就能一眼判定
      "到底谁在吃时间"（这正是 1.5fps 归因的关键）。
    """
    det = result if detector_result is None else detector_result
    # 失败帧才带 fail_stage / fail_reason：det_fail=track+roi+full/rejected
    #   -> "这一帧试过追踪、ROI 重搜、全图重搜，候选都被拒"
    #   -> track_refine = 只走了追踪、精修就失败了；no_proposal = 搜索层零提案
    fail_stage = det.get("fail_stage")
    fail_tag = "-" if not fail_stage else "%s/%s" % (fail_stage, det.get("fail_reason", "-"))
    return ("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
            "valid=%s stage=%s | read=%.1f draw=%.1f loop=%.1f ms | "
            "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
            "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
            (build_id, capture_fps, display_fps, display_low1, vision_fps,
             result.get("valid", False), result.get("stage"),
             read_avg, draw_avg, loop_avg,
             show_avg, show_p95, show_max, show_n,
             age_s * 1000, shown, drop_show, drop_vision)) + (
            " det_valid=%s det_stage=%s det_ms=%.1f det_masks=%d det_fail=%s det_props=%d" %
            (det.get("valid", False), det.get("stage"), det.get("detector_ms", 0.),
             det.get("masks", 0), fail_tag, det.get("proposals", 0)))


def format_slow_loop(loop_avg):
    """主循环偏慢（且此数**不含 show**）时的归因提示。"""
    return ("[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大；"
            "若 show 也恒定 ~505ms 则是图传忙等抢 CPU" % loop_avg)
'''

# ===== vision.maixvision_stream  (vision/maixvision_stream.py) =====
_MOD_SRC['vision.maixvision_stream'] = r'''
"""MaixVision protocol-0 CLIENT; JPEG/WebSocket run in a fresh process, no maix.

Protocol source: Sipeed/MaixCDK components/vision/src/maix_image_trans.cpp.
The parent only copies into one mmap slot under a nonblocking flock. The child
reopens the inherited fd: flock on a shared open-file-description cannot exclude
its parent, even though it is in another process.
"""
import base64
import hashlib
import mmap
import os
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time
try:
    import fcntl
except ImportError:  # Windows PC tools can import the application without POSIX I/O.
    fcntl = None

HEADER = 128
MAGIC = b'\xac\xbe\xcb\xca'
MAX_MESSAGE = 4*1024*1024
# C-level translation avoids a Python XOR loop for every JPEG byte (40fps).
_XOR_TABLES = tuple(bytes(v ^ key for v in range(256)) for key in range(256))


def packet(message, payload=b''):
    frame = MAGIC+struct.pack('<I', len(payload)+3)+bytes((0, message))+payload
    return frame+bytes((sum(frame)&255,))


def unpack_packet(frame):
    if len(frame) < 11 or frame[:4] != MAGIC or struct.unpack_from('<I', frame, 4)[0] != len(frame)-8 or frame[8] != 0 or sum(frame[:-1])&255 != frame[-1]:
        raise ValueError('invalid protocol-0 packet')
    return frame[9], frame[10:-1]


class WebSocketClient:
    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=.5)
        self.buffer = bytearray()
        self.fragments = bytearray()
        self.fragment_opcode = None
        try:
            key = base64.b64encode(os.urandom(16)).decode('ascii')
            request = ('GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % (host, port, key))
            self.socket.sendall(request.encode('ascii'))
            while b'\r\n\r\n' not in self.buffer:
                data = self.socket.recv(4096)
                if not data or len(self.buffer) > 16384:
                    raise OSError('invalid WebSocket handshake')
                self.buffer.extend(data)
            headers, rest = bytes(self.buffer).split(b'\r\n\r\n', 1)
            self.buffer = bytearray(rest)
            lines = headers.decode('ascii').split('\r\n')
            fields = {line.split(':',1)[0].lower(): line.split(':',1)[1] for line in lines[1:] if ':' in line}
            expected = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            if ' 101 ' not in lines[0] or fields.get('sec-websocket-accept', '').strip() != expected:
                raise OSError('WebSocket upgrade rejected')
        except BaseException:
            self.close()
            raise

    def send(self, payload, opcode=2):
        mask = os.urandom(4)
        size = len(payload)
        head = bytes((0x80|opcode, 0x80|size)) if size < 126 else (bytes((0x80|opcode, 0x80|126))+struct.pack('>H', size) if size < 65536 else bytes((0x80|opcode, 0x80|127))+struct.pack('>Q', size))
        encoded = bytearray(payload)
        for i in range(4):
            encoded[i::4] = encoded[i::4].translate(_XOR_TABLES[mask[i]])
        self.socket.sendall(head+mask+encoded)

    def receive(self, timeout=.01):
        if select.select([self.socket], [], [], timeout)[0]:
            data = self.socket.recv(65536)
            if not data:
                raise OSError('WebSocket closed')
            self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 2:
            b0, b1 = self.buffer[:2]
            opcode, size, offset = b0&15, b1&127, 2
            if b0 & 0x70 or b1 & 0x80:
                raise ValueError('unsupported WebSocket frame')
            if size in (126, 127):
                count = 2 if size == 126 else 8
                if len(self.buffer) < offset+count:
                    break
                size = int.from_bytes(self.buffer[offset:offset+count], 'big')
                offset += count
            if size > MAX_MESSAGE:
                raise ValueError('WebSocket message too large')
            if len(self.buffer) < offset+size:
                break
            payload = bytes(self.buffer[offset:offset+size])
            del self.buffer[:offset+size]
            if opcode == 8:
                raise OSError('WebSocket close')
            if opcode == 9:
                self.send(payload, 10)
                continue
            if opcode == 10:
                continue
            if opcode in (1, 2):
                if self.fragment_opcode is not None:
                    raise ValueError('interleaved WebSocket message')
                self.fragment_opcode = opcode
            elif opcode != 0 or self.fragment_opcode is None:
                raise ValueError('invalid continuation')
            self.fragments.extend(payload)
            if len(self.fragments) > MAX_MESSAGE:
                raise ValueError('WebSocket fragments too large')
            if b0 & 0x80:
                if self.fragment_opcode == 2:
                    messages.append(bytes(self.fragments))
                self.fragments.clear()
                self.fragment_opcode = None
        return messages

    def close(self):
        self.socket.close()


class MaixVisionStream:
    def __init__(self, cfg, width, height, clock=None):
        self.cfg, self.width, self.height = cfg, int(width), int(height)
        self.clock = clock if clock is not None else time
        self.process = self.mapping = self.file = None
        self.sequence = self.dropped = 0
        self._metrics_at = self.clock.perf_counter()
        self._sent_last = 0
        self._metrics_cache = dict(sent=0,fps=0.,encode_ms=0.,dropped=0)
        self.enabled = False
        if fcntl is None:
            print('[stream] POSIX flock unavailable; local display only')
            return
        try:
            self.file = tempfile.TemporaryFile()
            size = HEADER+self.width*self.height*3
            self.file.truncate(size)
            self.mapping = mmap.mmap(self.file.fileno(), size)
            # __source__ is supplied by build_single for virtual modules.
            source = getattr(sys.modules.get(__name__), '__source__', None)
            if source is None:
                with open(__file__, encoding='utf-8') as handle:
                    source = handle.read()
            command = source+'\n_child(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]))\n'
            self.process = subprocess.Popen([sys.executable, '-c', command, str(self.file.fileno()), str(self.width), str(self.height),
                getattr(cfg, 'SHOW_STREAM_HOST', '127.0.0.1'), str(getattr(cfg, 'SHOW_STREAM_PORT', 7899)),
                str(getattr(cfg, 'SHOW_STREAM_WIDTH', 480)), str(getattr(cfg, 'SHOW_JPEG_QUALITY', 70)),
                str(getattr(cfg, 'SHOW_STREAM_RETRY_S', 2.)), str(getattr(cfg, 'SHOW_STREAM_RETRY_MAX_S', 15.))],
                pass_fds=(self.file.fileno(),))
            self.enabled = True
        except Exception as exc:
            print('[stream] disabled: %s' % exc)
            self.close()

    def submit(self, canvas):
        if not self.enabled or self.process.poll() is not None:
            return False
        if canvas.shape != (self.height, self.width, 3) or str(canvas.dtype) != 'uint8':
            raise ValueError('stream frame shape/dtype mismatch')
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.dropped += 1
            return False
        try:
            self.sequence += 1
            self.mapping[HEADER:] = canvas.tobytes()
            struct.pack_into('<QII', self.mapping, 0, self.sequence, self.width, self.height)
        finally:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)
        return True

    def close(self):
        self.enabled = False
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.)
            self.process = None
        if self.mapping is not None:
            self.mapping.close()
            self.mapping = None
        if self.file is not None:
            self.file.close()
            self.file = None

    def metrics(self):
        """实际发送成功计数（不把投递到mmap的帧当作已图传）。"""
        if not self.enabled:
            return dict(self._metrics_cache)
        now = self.clock.perf_counter()
        try:
            fcntl.flock(self.file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            return dict(self._metrics_cache)
        try:
            sent, sent_at, encode_ms = struct.unpack_from('<Qdd',self.mapping,32)
        finally:
            fcntl.flock(self.file.fileno(),fcntl.LOCK_UN)
        elapsed = now-self._metrics_at
        if elapsed >= .1:
            self._metrics_cache = dict(sent=sent,fps=(sent-self._sent_last)/elapsed,
                                       encode_ms=encode_ms,dropped=self.dropped)
            self._metrics_at,self._sent_last = now,sent
        return dict(self._metrics_cache)


def _child(inherited_fd, width, height, host, port, stream_width, quality, retry, retry_max):
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    try:
        os.nice(5)  # CPU饱和时优先让识别和采图运行。
    except OSError:
        pass
    fd = os.open('/proc/self/fd/%d' % inherited_fd, os.O_RDWR)
    os.close(inherited_fd)
    memory = mmap.mmap(fd, HEADER+width*height*3)
    last = sent = 0
    delay = retry
    while True:
        client = None
        try:
            client = WebSocketClient(host, port)
            client.send(packet(1, b'code'))
            accepted, fmt, deadline = False, 1, time.monotonic()+10
            while True:
                for message in client.receive(.01):
                    ident, payload = unpack_packet(message)
                    if ident == 2:
                        if payload != b'\x01':
                            raise OSError('MaixVision registration rejected')
                        accepted, delay = True, retry
                    elif ident == 14 and len(payload) == 1:
                        value = payload[0]
                        if value <= 2:
                            fmt = value
                        client.send(packet(15, bytes((int(value <= 2), value))))
                    elif ident == 21 and len(payload) == 1:
                        quality = max(1, min(100, payload[0]))
                        client.send(packet(22, bytes((1, quality))))
                if not accepted:
                    if time.monotonic() > deadline:
                        raise OSError('MaixVision registration timeout')
                    continue
                if fmt == 0:
                    continue
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:
                    continue
                try:
                    sequence, w, h = struct.unpack_from('<QII', memory, 0)
                    raw = bytes(memory[HEADER:]) if sequence != last and sequence and (w,h)==(width,height) else None
                finally:
                    fcntl.flock(fd, fcntl.LOCK_UN)
                if raw is None:
                    continue
                last = sequence
                encoding_started = time.monotonic()
                frame = np.frombuffer(raw, np.uint8).reshape(height, width, 3)
                if 0 < stream_width < width:
                    frame = cv2.resize(frame, (stream_width, max(1, round(height*stream_width/width))), interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode('.jpg' if fmt == 1 else '.png', frame,
                                          [cv2.IMWRITE_JPEG_QUALITY, quality] if fmt == 1 else [])
                if ok:
                    encode_ms = (time.monotonic()-encoding_started)*1000
                    client.send(packet(6, bytes((fmt,))+encoded.tobytes()))
                    sent += 1
                    try:
                        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    except BlockingIOError:
                        continue
                    try:
                        struct.pack_into('<Qdd',memory,32,sent,time.monotonic(),encode_ms)
                    finally:
                        fcntl.flock(fd,fcntl.LOCK_UN)
        except (OSError, ValueError) as exc:
            print('[stream] reconnect: %s' % exc, flush=True)
        finally:
            if client is not None:
                client.close()
        time.sleep(delay)
        delay = min(retry_max, max(.05, delay*2))
'''

# ===== vision.preview  (vision/preview.py) =====
_MOD_SRC['vision.preview'] = r'''
"""显示层：标注画面 →（letterbox）画布 → display.show()，附屏幕可点击按钮（从 main.py 搬迁，v0.4.6）。

时间轴 C 的实现（见 main.py 模块开头总纲）：
    · Preview      —— 显示线程 + 画布池；主循环 render() 投递即返回，show() 慢也只慢预览。
    · TouchButtons —— 触摸按下沿 → 按钮命中。触摸坐标是屏幕坐标，与画布 1:1。
    · display_result()            —— 把识别线程的最新快照翻译成"这一帧该画什么"（过期降级）。
    · apply_maixvision_stream_policy() —— 建 Display 前掐断 MaixVision 图传（拆 500ms 忙等）。

★ clock 注入：构造可传 `clock`（默认标准库 time）。入口 main() 把自身 time 传进来，
  使 tools/test_main_self_test.py 的假时钟可穿透（同步显示路径在模拟时间轴下可复现）。
★ 隔离原则：显示层任何失败一律只打日志并置 Preview.enabled=False，绝不影响识别/串口/闭环。
"""
import os
import math
import threading
import time

import cv2
import numpy as np

from .rate_meter import FrameRate


def display_result(snapshot, now, max_age):
    """把"识别线程的最新快照"翻译成"这一帧显示该画什么"。

    过期定位不能被画成当前帧的有效定位（否则屏幕上框会"粘"在旧位置，误以为还锁着）；
    但视频原帧继续刷新，画面不因此冻住。返回 (result, laser, age)：
        age > max_age → 用 {valid:False, stage:"stale"} 占位，只保留 laser（面板照常显示）
    """
    if snapshot is None:
        return {"valid": False, "stage": "waiting"}, None, 0.0
    age = now - snapshot["captured_at"]
    if not math.isfinite(age) or not 0 <= age <= max_age:
        return {"valid": False, "stage": "stale"}, snapshot["laser"], age
    return snapshot["result"], snapshot["laser"], age


def aligned_frame(snapshot, live_gray, result):
    """有效标注使用对应采集帧；失效/过期时继续显示实时原图。"""
    if snapshot is not None and result.get('valid'):
        return snapshot.get('frame',live_gray)
    return live_gray


def apply_maixvision_stream_policy(cfg):
    """Compatibility hook: stream policy is now local to Display construction."""
    return False


def _local_display_call(callback):
    previous = os.environ.get("MAIXVISION")
    try:
        os.environ["MAIXVISION"] = "0"
        return callback()
    finally:
        if previous is None:
            os.environ.pop("MAIXVISION", None)
        else:
            os.environ["MAIXVISION"] = previous


class Preview:
    """显示层：标注画面 →（letterbox）画布 → `display.show()`，附屏幕可点击按钮。

    ★ 线程模型（时间轴 C，v0.4.3 起）
        主循环（生产者）  `render(view, ...)` —— letterbox + 叠按钮，把画布塞进"待上屏槽"就返回；
        显示线程（消费者） `_show_loop()`     —— 取最新画布 → `show()` → 画布还回池子。
        画布池固定 2 张：显示线程占 1 张、主线程画 1 张；两张都被占用时 `due()` 直接返回 False
        （丢帧，不排队、不等待）。因此 **show() 再慢也只表现为预览掉帧**，绝不拖慢主循环、
        识别线程与云台闭环 —— 这正是"把识别拆出去"之外还必须做的一步（见模块开头总纲）。
        `Config.SHOW_ASYNC=False` 时退回旧行为（主循环内同步 `show()`）。

    为什么画布做成"屏幕尺寸"：MaixPy 触摸返回的是**屏幕坐标**，只有画布尺寸 == 屏幕尺寸，
    触摸坐标才能与画布/按钮坐标 **1:1** 对应，按钮命中判定才可靠（MaixPy「触摸屏使用」示例即如此）。
    标注图按等比例 letterbox 居中，不拉伸变形。
    `Config.SHOW=False` 时不建 Display；无显示 / 建失败 / show 失败只打日志并停用，绝不影响识别与闭环。

    两级尺寸务必分清（`render()` 里同时出现，最易混淆）：
        · display.width/height  —— 物理屏幕尺寸，也是**触摸坐标**所在坐标系（按钮按此定义）
        · render_width/height   —— 实际送去 show() 的**画布**尺寸（SHOW_WIDTH 限制）
    默认 `SHOW_WIDTH=0` ⇒ 画布 == 屏幕：`show()` 不再缩放、触摸不再换算；相机 640×480 与
    MaixCAM2 屏幕 640×480 宽高比一致时，letterbox 也退化成零开销（不 resize、不刷黑边）。
    """

    def __init__(self, cfg, clock=None):
        self.cfg = cfg
        self._clock = clock if clock is not None else time
        self.stream = None
        self._flash = {}
        self.display = None
        self.width = self.height = 0            # 物理屏幕尺寸（触摸坐标系）
        self.render_width = self.render_height = 0
        self.enabled = False                    # 显示是否可用；任何一步失败都置 False 并停用
        self.buttons = {}                       # name -> (x0,y0,x1,y1)，屏幕坐标
        self._button_draw = []                  # 预算好的按钮绘制参数（画布坐标），渲染时不再换算
        self.shown = 0                          # 已成功 show() 的帧数（disp 口径）
        self.dropped = 0                        # 因显示线程忙而未画的帧数（丢帧，不排队）
        self.fps = 0.0
        self.low1 = 0.0
        self.rate = FrameRate()                 # 显示帧率（disp 口径，与识别帧率分开）
        self._period = 1.0 / max(1, int(getattr(cfg, "SHOW_FPS", 15)))   # 显示节拍周期
        self._t_show = -1e9                     # 下一次该画的时刻（固定时间轴，见 due/render）
        self._canvas = None                     # 同步模式的单张画布（异步模式走 _free 池）
        self._show_times = []                   # 本日志窗口内的 show() 耗时样本（ms）
        self._show_max = 0.0
        self.probe_ms = -1.0                    # 开机 show() 探针耗时（判定显示链路是否健康）
        self._slow_warned = False               # "显示侧慢"告警只打一次，避免每 2s 刷屏
        # ---- 显示线程状态（时间轴 C）----
        self.async_show = bool(getattr(cfg, "SHOW_ASYNC", True))
        self._cond = threading.Condition()      # 保护 _free / _pending / _pool / _stop
        self._free = []                         # 空闲画布池
        self._pending = None                    # 待上屏画布（只留最新一张）
        self._pool = 0                          # 已分配画布数（上限 2，由 due() 保证）
        self._stop = False
        self.thread = None
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口/头部模式，画面不流转）")
            return
        try:
            from maix import display
            self.display = _local_display_call(display.Display)
            self.width = int(self.display.width())
            self.height = int(self.display.height())
            # 画布宽度：SHOW_WIDTH<=0 ⇒ 用屏幕原始宽度（画布＝屏幕 ⇒ show() 零缩放）。
            cap = int(getattr(cfg, "SHOW_WIDTH", 0) or 0)
            self.render_width = self.width if cap <= 0 else min(self.width, cap)
            self.render_height = max(1, int(round(self.height * self.render_width / self.width)))
            if hasattr(display, "set_trans_image_quality"):
                display.set_trans_image_quality(int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)))
            self.enabled = True
            print("[preview] mode=%s screen=%dx%d canvas=%dx%d show_fps=%d jpeg=%d maixvision=%s"
                  % ("display-thread" if self.async_show else "sync-inprocess",
                     self.width, self.height, self.render_width, self.render_height,
                     int(getattr(cfg, "SHOW_FPS", 30)), int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)),
                     "on" if os.environ.get("MAIXVISION") == "1" else "off"))
            # ★ 开机探针：立刻量一次 show() 真实耗时。本地屏典型 1~5ms；若命中 MaixCDK 的
            #   "MaixVision 图传未就绪"分支，send_image() 会在 C++ 里**忙等 500ms 并空转 CPU**
            #   （见 MaixCDK components/vision/src/maix_image_trans.cpp），表现为 show 恒定 ~505ms、
            #   并连带把相机/识别/闭环一起拖慢。这条必须当场说清，不能只报一个 p95 数字。
            self.probe_ms = self._probe_show()
            if not bool(getattr(cfg, "DISABLE_MAIXVISION_STREAM", False)) and os.environ.get("MAIXVISION") == "1":
                from .maixvision_stream import MaixVisionStream
                self.stream = MaixVisionStream(cfg, self.render_width, self.render_height, clock=self._clock)
            if self.async_show:
                self.thread = threading.Thread(target=self._show_loop, name="display", daemon=True)
                self.thread.start()
        except Exception as exc:
            self.enabled = False
            # 建不了显示不算错误：板子仍照常识别 + 跑闭环，只是没有画面。
            print("[preview] 显示初始化失败（不影响识别/闭环，仅无预览）: %s" % exc)

    # ------------------------------------------------------------------ 按钮
    def add_button(self, name, x0, y0, x1, y1):
        """注册一个按钮（**屏幕坐标**）。几何与文本宽度在此**预计算一次**，渲染时不再换算。"""
        self.buttons[name] = (int(x0), int(y0), int(x1), int(y1))
        # 屏幕坐标 → 画布坐标（默认 SHOW_WIDTH=0 时二者相同，比例恒为 1）
        sx = self.render_width / float(self.width) if self.width else 1.0
        sy = self.render_height / float(self.height) if self.height else 1.0
        bx0, bx1 = int(round(x0 * sx)), int(round(x1 * sx))
        by0, by1 = int(round(y0 * sy)), int(round(y1 * sy))
        # 用 ASCII 标签：OpenCV 的 Hershey 字体不支持中文，画中文会变问号/方块。
        label = "SELF-TEST" if name == "selftest" else name.upper()
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0][0]
        pos = (bx0 + max(4, (bx1 - bx0 - tw) // 2), by0 + (by1 - by0) // 2 + 7)   # 水平居中
        self._button_draw.append((name, bx0, by0, bx1, by1, label, pos))

    def flash_button(self, name, now=None, hold=.25):
        self._flash[name] = (self._clock.perf_counter() if now is None else now)+hold

    def hit(self, x, y):
        """触摸点 (x, y)（屏幕坐标）命中的按钮名；未命中返回 None。"""
        for name, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    # ------------------------------------------------------------------ 渲染
    def due(self, now):
        """是否到了显示节拍**且**有画布可用；显示线程忙时返回 False（丢帧，绝不等待）。"""
        if not self.enabled or float(now) < self._t_show:
            return False
        if not self.async_show:
            return True
        with self._cond:
            if self._free or self._pool < 2:
                return True
        self.dropped += 1        # 节拍到了但两张画布都占用中 → 本帧不画（丢帧而非排队）
        return False

    def _take_canvas(self):
        """取一张画布：同步模式复用固定那张；异步模式从池里取，池空则新分配（上限由 due 保证）。"""
        need = (self.render_height, self.render_width)
        if not self.async_show:
            if self._canvas is None or self._canvas.shape[:2] != need:
                self._canvas = np.empty((need[0], need[1], 3), np.uint8)
            return self._canvas
        with self._cond:
            if self._free:
                return self._free.pop()
            if self._pool >= 2:
                return None
            self._pool += 1
        return np.empty((need[0], need[1], 3), np.uint8)

    def render(self, view, now, status=""):
        """把标注后的 `view`（任意尺寸）letterbox 到画布、叠按钮与状态，然后交给显示线程。

        步骤（对应模块总纲时间轴 A 的第 ⑦ 步）：
            1) 推进节拍 _t_show（固定时间轴，见下）
            2) 算 letterbox 缩放与居中偏移 (scale/ox/oy)，把 view 放进画布
            3) 叠按钮/状态条
            4) 交显示 —— 异步：投递后**立即返回**；同步：当场 `show()`
        """
        if not self.enabled:
            return
        now = float(now)
        # ★ 固定时间轴限频：节拍由"上一次预期时刻 + 周期"推进，而不是"show 完成后再等一周期"。
        #   好处：即使某次 show() 慢，也不会让后续节拍越拖越后（避免累积漂移）；
        #   已经错过的节拍用 skipped 一次性跳过，只画最新画面。
        if self._t_show < 0:
            self._t_show = now + self._period
        else:
            skipped = max(1, int((now - self._t_show) / self._period) + 1)
            self._t_show += skipped * self._period
        canvas = self._take_canvas()
        if canvas is None:
            self.dropped += 1
            return
        # ---- letterbox：等比缩放 + 居中，两侧留黑边，绝不拉伸（保持几何不变形）----
        vh, vw = view.shape[:2]
        rw, rh = self.render_width, self.render_height
        scale = min(rw / float(vw), rh / float(vh))         # 取两轴较小比例 → 完整装入
        nw, nh = max(1, int(round(vw * scale))), max(1, int(round(vh * scale)))
        ox, oy = (rw - nw) // 2, (rh - nh) // 2             # 居中偏移
        if (nw, nh) != (rw, rh):
            canvas[:] = 0                                   # 只有真存在黑边时才需要刷黑背景
        canvas[oy:oy + nh, ox:ox + nw] = (
            view if (nw, nh) == (vw, vh) else cv2.resize(view, (nw, nh)))
        self._draw_buttons(canvas, status)
        if self.stream is not None:
            try:
                self.stream.submit(canvas)
            except Exception as exc:
                print("[stream] submit failed: %s" % exc)
                self.stream.close()
                self.stream = None
        #                   # 按钮与状态条画在画布之上
        if self.async_show:
            self._publish(canvas)                            # 投递即返回：show 交给显示线程
        else:
            self._do_show(canvas)

    def _publish(self, canvas):
        """把画布放进"待上屏槽"（只留最新一张）并唤醒显示线程；主循环不等它。"""
        with self._cond:
            if self._stop:
                return
            if self._pending is not None:
                self._free.append(self._pending)             # 上一张还没上屏 → 退回池（丢帧）
                self.dropped += 1
            self._pending = canvas
            self._cond.notify()

    def _show_loop(self):
        """显示线程主体（时间轴 C）：取最新画布 → show() → 画布还池。**慢也只慢自己**。"""
        try:
            from maix import image as maix_image
        except Exception as exc:
            self.enabled = False
            print("[preview] 显示线程无法导入 maix.image，退化为无预览: %s" % exc)
            return
        while True:
            with self._cond:
                if self._pending is None and not self._stop:
                    self._cond.wait(timeout=0.02)            # 20ms 轮询，保证 stop 能被及时看到
                if self._stop:
                    return
                canvas, self._pending = self._pending, None
            if canvas is None:
                continue
            if not self._do_show(canvas, maix_image):
                return                                       # show 已失败并停用显示 → 线程退出
            with self._cond:
                self._free.append(canvas)

    def _probe_show(self):
        """开机量一次 show() 耗时，判定显示链路是否健康；返回毫秒（探针失败返回 -1）。

        判据为何取 300ms：本地屏幕刷新官方数据是 **1~5ms**；而 MaixCDK 的
        `ImageTrans::send_image()` 在「处于 MaixVision 模式、但图传连接未就绪（handle->init 为假）」
        时，会 `while(!init) { if(ticks - t > 500) return ERR_NOT_READY; }` ——
        **忙等 500ms 且空转 CPU**。这正是本项目实测
        `show avg/p95/max=505.8/508.4/509.1ms`、`disp≈1.9fps` 且相机与识别一起变慢的根因：
        500ms × 约 2 次/秒 ≈ **烧掉半个 CPU 核**。
        """
        try:
            from maix import image as maix_image
            frame = np.zeros((self.render_height, self.render_width, 3), np.uint8)
            t0 = self._clock.perf_counter()
            self.display.show(maix_image.cv2image(frame, bgr=True, copy=True))
            ms = (self._clock.perf_counter() - t0) * 1000.0
        except Exception as exc:
            print("[preview] show 探针失败（显示可能不可用）: %s" % exc)
            self.enabled = False
            return -1.0
        if ms >= float(getattr(self.cfg, "SHOW_STREAM_SLOW_MS", 300.)):
            print("[preview] show 探针 %.0fms：本地显示偏慢；检查 SDK 是否复用旧图传对象。" % ms)
            print("[preview] 对策①：确认 Display 构造时 MAIXVISION=0；本应用已临时隔离并恢复会话。")
            print("[preview] 对策②：从设备本地启动，对照本地屏 show 耗时。")
            print("[preview] 对策③：检查 VO/屏幕驱动；独立图传连接与编码不在 show 线程。")
        else:
            print("[preview] show 探针 %.1fms —— 显示链路健康（本地屏典型 1~5ms）" % ms)
        return ms

    def warn_slow_once(self, p95):
        """显示侧变慢只提示一次（每 2s 窗口重复报会刷屏）；点明最可能的根因。"""
        if self._slow_warned:
            return
        self._slow_warned = True
        print("[preview] show slow: p95=%.0fms；检查本地屏驱动和 SDK 的图传对象缓存。"
              "图传进程独立，预览丢帧不会积压识别结果。" % p95)

    def _do_show(self, canvas, maix_image=None):
        """真正调用 display.show()，记耗时/帧率；失败即停用显示（不影响识别/闭环）。返回是否成功。"""
        t0 = self._clock.perf_counter()
        try:
            if maix_image is None:
                from maix import image as maix_image
            # bgr=True：cv2 是 BGR 序；copy=True：交给显示的必须是独立缓冲。
            self.display.show(maix_image.cv2image(canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return False
        ms = (self._clock.perf_counter() - t0) * 1000.0
        self.shown += 1
        completed = self._clock.perf_counter()
        self.rate.record(completed)
        self.fps, self.low1 = self.rate.values(completed)
        with self._cond:
            self._show_times.append(ms)
            if ms > self._show_max:
                self._show_max = ms
        return True

    def take_show_stats(self):
        """取走并清空本日志窗口的 show() 耗时样本 → (avg, p95, max, n)，单位 ms。"""
        with self._cond:
            times, worst = self._show_times, self._show_max
            self._show_times, self._show_max = [], 0.0
        if not times:
            return 0.0, 0.0, worst, 0
        arr = np.asarray(times, dtype=np.float64)
        return float(arr.mean()), float(np.percentile(arr, 95)), worst, len(times)

    def _draw_buttons(self, canvas, status=""):
        # 按钮几何/文本宽度已在 add_button() 预算好，这里只画（不再每帧 getTextSize/换算）。
        for (name, x0, y0, x1, y1, label, pos) in self._button_draw:
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 110, 235) if self._clock.perf_counter() < self._flash.get(name, -1e9) else (46, 150, 60), -1)   # 填充
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (235, 235, 235), 1)  # 描边
            cv2.putText(canvas, label, pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        if status:
            # 左下角状态串：self-test i/n、calibrate <axis>、或 aim 状态机名。
            cv2.putText(canvas, status, (8, self.render_height - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    def close(self):
        """先停显示线程（等它把当前一次 show 做完）再关 Display —— 顺序反了会并发操作显示。"""
        if self.async_show:
            with self._cond:
                self._stop = True
                self._pending = None
                self._cond.notify()
            if self.thread is not None:
                self.thread.join(timeout=2.0)
                if self.thread.is_alive():
                    raise RuntimeError("display thread did not stop")
        if self.stream is not None:
            self.stream.close()
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass


class TouchButtons:
    """触摸屏输入：把点击映射到 `Preview` 的按钮（**按下沿**触发，防连点重复启动）。

    `maix.touchscreen.TouchScreen().read()` 立即返回 `(x, y, pressed)`（MaixPy v4）。
    无触摸屏 / 未启用 / 无显示时自动禁用，只打日志。
    坐标口径：read() 返回的是**屏幕坐标**，与按钮注册坐标一致（画布=屏幕尺寸的前提）。
    """

    MAX_HOLD_S = 2.0

    def __init__(self, cfg, clock=None):
        self._clock = clock if clock is not None else time
        self._pressed_at = -1e9
        self.ts = None
        self._down = False                 # 上一帧按下状态，用于识别"按下沿"
        if not bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            print("[touch] 触摸按钮已关闭（ENABLE_TOUCH_BUTTON=False）")
            return
        try:
            from maix import touchscreen
            self.ts = touchscreen.TouchScreen()
            print("[touch] 触摸屏就绪：屏幕按钮可点击")
        except Exception as exc:
            print("[touch] 触摸不可用（按钮禁用）: %s" % exc)

    def poll(self, preview):
        """返回本帧**新按下**的按钮名（按下沿）；无则 None。"""
        if self.ts is None or preview is None or not preview.enabled:
            return None
        try:
            x, y, pressed = self.ts.read()
        except Exception as exc:
            self._down = False
            print("[touch] read failed: %s" % exc)
            return None
        now = self._clock.perf_counter()
        if self._down and now-self._pressed_at > self.MAX_HOLD_S:
            self._down = False
        if pressed:
            if not self._down:             # 只有"上一帧未按、这一帧按下"才算一次点击
                self._down = True
                self._pressed_at = now
                name = preview.hit(int(x), int(y))
                print("[touch] x=%d y=%d hit=%s" % (x, y, name or "none"))
                if name:
                    preview.flash_button(name, now)
                return name
            return None                    # 按住不放：不重复返回，避免反复触发
        self._down = False                 # 松开，允许下一次按下沿
        return None
'''

# ===== vision.rectangle_center  (vision/rectangle_center.py) =====
_MOD_SRC['vision.rectangle_center'] = r'''
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
'''

# ===== vision.laser_spot  (vision/laser_spot.py) =====
_MOD_SRC['vision.laser_spot'] = r'''
"""激光光斑读数：标称位置附近的小 ROI 内找最亮点，输出亚像素质心与到靶心偏差。

为什么只搜一个小 ROI：相机与激光笔刚性固连（同装在俯仰平台上），光斑在图像里的
位置近似固定 —— 2026-10-07 现场一帧实测约 (294, 202)（640x480），背景灰度 145、
峰值 177。因此不必全图找"最亮的东西"（那会把白墙高光、灯光、反光都算进去），
只要在标称位置周围 LASER_ROI_RADIUS 像素内做一次阈值比较 + 几十个像素的质心，
代价可忽略。标称值只决定"往哪儿看"，找到的真实位置仍然逐次更新。

为什么要限频：即使单次只有零点几毫秒，也不该塞进每帧关键路径。process() 内部
按 LASER_INTERVAL_S 限频（默认 0.30s ≈ 每秒 3 次），两次更新之间直接沿用上次
结果（updated=False），所以显示仍然是每帧画的 —— 画面流畅度不受影响。

两种取位方式（`LASER_MODE`），云台侧拿到的 (u,v) 语义完全相同，只是来源不同：
    "fixed"（默认、当前方案） 光斑位置写死 = (LASER_U_NOMINAL, LASER_V_NOMINAL)，
        全程不做任何图像检测；每帧唯一的运算是"黑框中心 − 该常量"的两次减法。
        之所以可行：相机与激光刚性同转 → 光斑在图像里的位置本来就是常量（需求文档 §7.1）。
    "detect"        每 LASER_INTERVAL_S 在标称位附近 ROI 实测一次（自校准，吸收漂移）。
        只在需要重新标定 U/V、或想验证"写死的值还准不准"时临时启用。

输出契约（dict）：
    found    bool    本次是否有可信光斑（未更新时间沿用的仍是它）
    updated  bool    本次是否真的重算了（False = 限频沿用 / fixed 模式）
    u, v     float   光斑位置，相机原图坐标。**调用方必须先看 found**：found=False 时
                     u/v 只是标称值的占位，绝不能当测量结果用（deviation() 已代查）。
    peak     float   ROI 最大灰度；bg 为 ROI 背景灰度（隔行子采样中位数）
    area     int     超过阈值的像素数
    reason   str     原因码：ok / fixed / no_peak / too_big / too_small / jumped / roi
    ms       float   本次计算耗时（毫秒），未更新时为 0
"""
import math
import time

import cv2
import numpy as np

_PANEL_BG = (32, 32, 32)
PANEL_LINE = 18
PANEL_H = 2 * PANEL_LINE + 10   # 面板高度（显示画布坐标）；main 用它把框文字整体下移
_TEXT_C1 = (255, 255, 0)     # BGR 青：光斑状态行
_TEXT_C2 = (120, 255, 120)   # BGR 浅绿：偏差行（有效）
_TEXT_C3 = (120, 120, 255)   # BGR 浅红：偏差行（无靶纸）
_MARK = (0, 0, 255)          # BGR 红：光斑标记（跟激光同色）
_LINK = (0, 160, 255)        # BGR 橙：光斑->靶心连线


class LaserSpotDetector:
    """标称位置附近找最亮点，亚像素质心 + 限频 + 跳变门限。"""

    def __init__(self, config):
        self.cfg = config
        self.mode = str(getattr(config, "LASER_MODE", "detect")).strip().lower()
        self.nominal = (float(config.LASER_U_NOMINAL), float(config.LASER_V_NOMINAL))
        self.interval = float(getattr(config, "LASER_INTERVAL_S", 0.30))
        self.alpha = float(getattr(config, "LASER_SMOOTH_ALPHA", 0.0))
        self.max_jump = float(getattr(config, "LASER_MAX_JUMP_PX", 40.0))
        self.last = dict(found=False, updated=False, u=0.0, v=0.0, peak=0.0, bg=0.0,
                         area=0, reason="init", ms=0.0)
        self._t = None
        self._uv = None

    # ------------------------------------------------------------------ 内部
    def _detect(self, g):
        """在标称位附近的方形 ROI 内做一次"阈值 + 加权质心"，返回 (u,v)。

        判据链（任一不满足即判无光斑，并带 reason 便于排障）：
            峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)  → 否则 no_peak
            亮点像素数在 [LASER_MIN_PIXELS, LASER_MAX_PIXELS] 内    → 过大 too_big（挡高光）/过小 too_small
        质心用 (灰度−阈值) 加权，越亮的像素权重越大 → 亚像素级、抗噪。
        """
        cfg = self.cfg
        H, W = g.shape[:2]
        cx, cy = float(cfg.LASER_U_NOMINAL), float(cfg.LASER_V_NOMINAL)
        r = int(cfg.LASER_ROI_RADIUS)
        # ROI = 标称位 ± r，并夹取到图像内（标称值决定"往哪儿看"，不决定结果）。
        x0 = max(0, int(cx - r))
        y0 = max(0, int(cy - r))
        x1 = min(W, int(cx + r) + 1)
        y1 = min(H, int(cy + r) + 1)
        if x1 - x0 < 5 or y1 - y0 < 5:
            return dict(found=False, reason="roi", peak=0.0, bg=0.0, area=0, u=cx, v=cy)
        roi = g[y0:y1, x0:x1]
        # 背景用 1/4 子采样中位数：抗光斑/噪点污染，又不必对 16k 像素排序。
        bg = float(np.median(roi[::4, ::4]))
        peak = float(roi.max())
        thr = max(float(cfg.LASER_MIN_PEAK), bg + float(cfg.LASER_MIN_CONTRAST))
        if peak < thr:
            return dict(found=False, reason="no_peak", peak=peak, bg=bg, area=0, u=cx, v=cy)
        mask = roi >= thr
        area = int(mask.sum())
        if area > int(cfg.LASER_MAX_PIXELS):
            # 大片高光/反光，不是激光点（激光点在 165cm 处只有几个像素）。
            return dict(found=False, reason="too_big", peak=peak, bg=bg, area=area, u=cx, v=cy)
        if area < int(cfg.LASER_MIN_PIXELS):
            return dict(found=False, reason="too_small", peak=peak, bg=bg, area=area, u=cx, v=cy)
        ys, xs = np.nonzero(mask)
        w = roi[ys, xs].astype(np.float32) - thr
        total = float(w.sum())
        if total <= 0:
            return dict(found=False, reason="flat", peak=peak, bg=bg, area=area, u=cx, v=cy)
        # 加权质心：权重 = 灰度−阈值；坐标加 ROI 原点 (x0,y0) 回到全图坐标。
        return dict(found=True, reason="ok", peak=peak, bg=bg, area=area,
                    u=x0 + float((xs * w).sum() / total), v=y0 + float((ys * w).sum() / total))

    # ------------------------------------------------------------------ 对外
    @staticmethod
    def _fixed_dict(nominal):
        return dict(found=True, updated=False, u=nominal[0], v=nominal[1],
                    peak=0.0, bg=0.0, area=0, reason="fixed", ms=0.0)

    def static_state(self):
        """fixed 模式的常量结果：光斑位置写死，不做任何图像运算。

        main.py 在 fixed 模式下只在开机取一次这个 dict，之后每帧复用；每帧唯一的
        运算是 deviation() 里的两次减法（黑框中心 − 该常量）。
        """
        return self._fixed_dict(self.nominal)

    def process(self, gray, now=None):
        """每帧调用；fixed 模式恒返回标定常量，detect 模式内部按 LASER_INTERVAL_S 限频。

        限频逻辑：距上次实测不足 interval 时**直接沿用上次结果**（updated=False, ms=0），
        因此每帧调用的代价近似为零；真正重算时依次做跳变门限与平滑。
        """
        if self.mode == "fixed":
            return self.static_state()
        now = time.perf_counter() if now is None else float(now)
        if self._t is not None and (now - self._t) < self.interval:
            out = dict(self.last)                 # 沿用上次：位置不变，仅标记未更新
            out["updated"] = False
            out["ms"] = 0.0
            return out
        self._t = now
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        started = time.perf_counter()
        res = self._detect(g)
        res["ms"] = (time.perf_counter() - started) * 1000.0
        res["updated"] = True
        if res["found"] and self._uv is not None and self.max_jump > 0:
            if abs(res["u"] - self._uv[0]) > self.max_jump or abs(res["v"] - self._uv[1]) > self.max_jump:
                # 单次跳太远：判为误检（高光/干扰物），保留上次位置。
                keep = dict(res)
                keep.update(found=True, reason="jumped", u=self._uv[0], v=self._uv[1])
                self.last = keep
                return keep
        if res["found"]:
            if self._uv is not None and 0.0 < self.alpha < 1.0:
                # 位置一阶低通（仅 detect 模式）；alpha=0 时关闭平滑，直接用最新质心。
                self._uv = (self._uv[0] + self.alpha * (res["u"] - self._uv[0]),
                            self._uv[1] + self.alpha * (res["v"] - self._uv[1]))
            else:
                self._uv = (res["u"], res["v"])
            res["u"], res["v"] = self._uv
        self.last = res
        return res


def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定取自需求文档 §7.2（dx = u靶心 − u光斑），与后续串口直控云台同一套符号：
    电机方向由各轴 DIR_SIGN 映射；像素误差为正不代表固定的电机方向。

    use_raw：闭环取这一档（True）—— 取 result["raw_u"]/["raw_v"]。
    result["u"]/["v"] 是识别输出中心；新链路无滤波，两者当前恒等，
    保留双口径是为了将来加平滑时调用方不必改（平滑值会给闭环带来相位滞后）。
    激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])   # 闭环档：未平滑中心
    else:
        tx, ty = float(result["u"]), float(result["v"])           # 显示档：识别输出中心
    lu, lv = float(laser['u']), float(laser['v'])
    if not all(map(math.isfinite,(tx,ty,lu,lv))):
        return None
    size = result.get('image_size')
    if size is not None and not (0 <= lu < size[0] and 0 <= lv < size[1]):
        return None
    return (tx - lu, ty - lv)       # dx=靶心−光斑, dy 同理


def draw_laser_overlay(canvas, laser, result=None, scale=1.0, use_raw=False):
    """在显示画布左上角画激光读数面板 + 光斑标记 + 到靶心的连线。

    use_raw 与 deviation() 保持一致：True 时连线终点与 dx/dy 都用未平滑的 raw 中心，
    屏幕上看到的就是控制律真正在用的那个点。

    scale：**原图坐标 → 画布坐标** 的比例（main.py 传 render_width/原图宽）。
    本函数内部所有坐标（光斑 p、连线终点 c）都要乘 scale，才能与 draw_result 画的框对齐。
    **注意**：面板本身（左上角矩形与文字）用画布固定像素坐标，不随 scale 缩放。

    返回面板占用的高度（像素，显示画布坐标），调用方据此把 draw_result 的文字
    整体下移，避免和本面板重叠。
    """
    if not laser:
        return 0
    found = bool(laser.get("found"))
    # 第 1 行：光斑位置与来源（fixed=标定常量 / 实测 peak,area / 丢失原因）。
    if laser.get("reason") == "fixed":
        line1 = "LASER (%d,%d) [fixed/calib]" % (round(laser["u"]), round(laser["v"]))
    elif found:
        line1 = "LASER (%d,%d) peak=%d area=%d" % (
            round(laser["u"]), round(laser["v"]), round(laser["peak"]), laser.get("area", 0))
    else:
        line1 = "LASER -- [%s] peak=%d bg=%d" % (
            laser.get("reason", "?"), round(laser.get("peak", 0.0)), round(laser.get("bg", 0.0)))
    # 第 2 行：光斑→靶心偏差（用同一 use_raw 口径，与闭环一致）。
    dv = deviation(laser, result, use_raw=use_raw)
    if dv is not None:
        line2 = "D-CENTER dx=%+.1f dy=%+.1f px" % (dv[0], dv[1])
        line2_color = _TEXT_C2                       # 浅绿：有效
    elif result is not None and result.get("valid"):
        line2 = "D-CENTER -- (laser lost)"
        line2_color = _TEXT_C3                       # 浅红：有靶心无光斑
    else:
        line2 = "D-CENTER -- (no black rect)"
        line2_color = _TEXT_C3                       # 浅红：无靶心
    # 面板宽度按两行文字较宽者自适应（固定画布像素，不随 scale 缩放）。
    w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0] for t in (line1, line2))
    cv2.rectangle(canvas, (2, 2), (min(canvas.shape[1] - 1, w + 14), PANEL_H - 2), _PANEL_BG, -1)
    cv2.putText(canvas, line1, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, _TEXT_C1, 1, cv2.LINE_AA)
    cv2.putText(canvas, line2, (8, 18 + PANEL_LINE), cv2.FONT_HERSHEY_SIMPLEX, .45, line2_color, 1, cv2.LINE_AA)
    if not found:
        return PANEL_H                              # 无光斑：只留面板，不画标记/连线
    # 光斑标记：原图坐标 × scale → 画布坐标。
    p = (int(round(laser["u"] * scale)), int(round(laser["v"] * scale)))
    if result is not None and result.get("valid"):
        # 连线终点 = 靶心，同样按 use_raw 选 raw 或平滑中心，再 × scale。
        if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
            c = (int(round(float(result["raw_u"]) * scale)), int(round(float(result["raw_v"]) * scale)))
        else:
            c = (int(round(float(result["u"]) * scale)), int(round(float(result["v"]) * scale)))
        cv2.line(canvas, p, c, _LINK, 1, cv2.LINE_AA)   # 橙色连线：光斑 → 靶心
    cv2.circle(canvas, p, 7, _MARK, 1, cv2.LINE_AA)     # 红色圆圈
    cv2.drawMarker(canvas, p, _MARK, cv2.MARKER_CROSS, 5, 1, cv2.LINE_AA)   # 红十字
    return PANEL_H
'''

# ===== vision.stepper_link  (vision/stepper_link.py) =====
_MOD_SRC['vision.stepper_link'] = r'''
"""ZDT Emm_V5.0 双串口链路（协议帧构造 + 非阻塞收发 + 链路健康）。

本模块可以在 PC 上导入；只有创建 StepperLink 时才会导入 MaixPy。
协议使用固定 0x6B 校验、大端参数、相对位置命令（`MaixCAM2串口直控云台需求文档.md` §6）。

帧构造是纯函数（``cmd_*``），返回帧解析也做成纯函数 + 可注入的 ``_ingest``，
这样即使没有 MaixPy 也能在 PC 上逐字节回归。

返回帧：[地址][功能码][状态][校验]，状态 `0x02`=成功、`0xE2`=未使能/堵转、
`0x00 0xEE`=错误命令（见 §6.1）。
"""

import time
import math

CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2


def _frame(addr, func, data=()):
    return bytes([int(addr) & 0xFF, int(func) & 0xFF] +
                 [int(x) & 0xFF for x in data] + [CR])


def cmd_enable(addr, enable=True, sync=False):
    return _frame(addr, 0xF3, (0xAB, 1 if enable else 0, 1 if sync else 0))


def cmd_speed(addr, ccw, rpm, acc=0, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    return _frame(addr, 0xF6, (1 if ccw else 0, rpm >> 8, rpm,
                               int(acc), 1 if sync else 0))


def cmd_position(addr, ccw, rpm, pulses, acc=0, absolute=False, sync=False):
    rpm = max(0, min(5000, int(rpm)))
    pulses = max(0, min(0xFFFFFFFF, int(abs(pulses))))
    return _frame(addr, 0xFD, (
        1 if ccw else 0, rpm >> 8, rpm, int(acc),
        pulses >> 24, pulses >> 16, pulses >> 8, pulses,
        1 if absolute else 0, 1 if sync else 0))


def cmd_stop_now(addr, sync=False):
    return _frame(addr, 0xFE, (0x98, 1 if sync else 0))


def cmd_sync_motion(addr=0):
    return _frame(addr, 0xFF, (0x66,))


def cmd_read(addr, func):
    return _frame(addr, func, ())


def frame_status(frame):
    """返回帧第 3 字节 = 命令状态；帧太短返回 None。"""
    return int(frame[2]) if frame is not None and len(frame) >= 3 else None


def _readback_pin(pinmap, pin):
    """回读引脚当前实际功能（AGENTS.md §6.4：'某轴不通'的第一手证据）。失败返回 '?'。"""
    try:
        return pinmap.get_pin_function(pin)
    except Exception:
        return "?"


class StepperLink:
    """两条独立 UART 链路；任一轴打开失败不会阻塞另一轴。"""

    def __init__(self, cfg, clock=None):
        self._clock = clock if clock is not None else time
        self.cfg = cfg
        self._ports = {}
        self._rxbuf = {"yaw": bytearray(), "pitch": bytearray()}
        self._stats = {"tx": 0, "rx": 0, "ok": 0, "e2": 0, "err": 0,
                       "timeout": 0, "yaw": True, "pitch": True}
        self._last_status = {"yaw": None, "pitch": None}
        self._last_cmd = {"yaw": 0.0, "pitch": 0.0}
        self._last_ack = {"yaw": 0.0, "pitch": 0.0}
        self._pending_ack_since = {"yaw": None, "pitch": None}
        # ★ ACK 看门狗必须**按轴独立武装**（2026-10-18 板端实锤）：
        #   原先用全局 `_stats["ok"] > 0` 武装，只要 yaw 会回帧，pitch 就被一并
        #   要求回帧；而 pitch 的驱动器 Response=None（或未接线/ID 不符）时永远
        #   不回帧 → 累计 20 次超时 → `FAULT: link:ack_timeout:pitch`，整机锁死，
        #   连一条运动指令都发不出去。从未回过帧的轴不应据此判障。
        self._ack_seen = {"yaw": False, "pitch": False}
        self._no_ack_warned = {"yaw": False, "pitch": False}
        self._timeout_count = {"yaw": 0, "pitch": 0}
        self._e2_times = []
        self.fault_reason = None
        try:
            from maix import uart, pinmap
        except Exception as exc:
            self._stats["err"] += 1
            self._stats["yaw"] = self._stats["pitch"] = False
            self._init_error = exc
            return
        # ★ 实测轴映射定案（AGENTS.md §2）：yaw=UART2/B0B1/ttyS2/ID2，pitch=UART0/A28A29/ttyS0/ID1。
        #   两轴都要显式 pinmap（U0 走空表会静默不通）；ID 跟着"电机"走，不跟着端口。
        pin_sets = {
            "yaw": getattr(cfg, "YAW_PINS", {"B0": "UART2_TX", "B1": "UART2_RX"}),
            "pitch": getattr(cfg, "PITCH_PINS", {"A28": "UART0_TX", "A29": "UART0_RX"}),
        }
        devices = {"yaw": (getattr(cfg, "YAW_DEV", "/dev/ttyS2"),
                             getattr(cfg, "YAW_BAUD", 115200)),
                   "pitch": (getattr(cfg, "PITCH_DEV", "/dev/ttyS0"),
                               getattr(cfg, "PITCH_BAUD", 115200))}
        strict = bool(getattr(cfg, "PINMAP_STRICT", False))
        flush_rx = bool(getattr(cfg, "FLUSH_RX_ON_START", False))
        for axis in ("yaw", "pitch"):
            try:
                for pin, func in pin_sets[axis].items():
                    try:
                        result = pinmap.set_pin_function(pin, func)
                        failed = (result is not None and hasattr(result, "value")
                                  and result.value != 0)
                    except Exception as exc:              # 无该引脚 / 固件不支持
                        result, failed = exc, True
                    if failed:
                        message = "pinmap %s=%s failed: %s" % (pin, func, result)
                        if strict:
                            raise RuntimeError(message)
                        # 非严格模式：只告警，仍尝试打开 UART —— 否则真故障会被藏成"打不开"。
                        print("[gimbal] WARN " + message)
                    else:
                        print("[gimbal] pinmap %s=%s ok (readback=%s)" %
                              (pin, func, _readback_pin(pinmap, pin)))
                self._ports[axis] = uart.UART(*devices[axis])
                if flush_rx:
                    # U0(pitch) 是系统日志口：开机先丢掉内核启动日志/终端回显等 RX 残留。
                    self._flush_rx(axis)
            except Exception as exc:
                self._stats[axis] = False
                self._stats["err"] += 1
                print("[gimbal] %s UART open failed: %s" % (axis, exc))

    # ------------------------------------------------------------- 发送
    def _flush_rx(self, axis):
        """清空该轴串口开机前的 RX 残留（U0=系统日志口，会混内核启动日志/终端回显）。"""
        port = self._ports.get(axis)
        if port is None:
            return
        try:
            available = int(port.available(0))
            if available > 0:
                port.read(min(available, int(getattr(self.cfg, "UART_RX_MAX_BYTES", 256))), timeout=0)
        except Exception:
            pass
        self._rxbuf[axis] = bytearray()

    def _write(self, axis, payload, now=None):
        port = self._ports.get(axis)
        if port is None:
            return False
        try:
            n = port.write(payload)
            if n is not None and int(n) != len(payload):
                raise IOError("write returned %s" % n)
            self._stats["tx"] += 1
            self._last_cmd[axis] = self._clock.perf_counter() if now is None else float(now)
            pending = getattr(self, "_pending_ack_since", None)
            if pending is None:
                pending = self._pending_ack_since = {"yaw": None, "pitch": None}
            if pending[axis] is None:
                pending[axis] = self._last_cmd[axis]
            return True
        except Exception as exc:
            self._stats["err"] += 1
            print("[gimbal] %s TX failed: %s" % (axis, exc))
            return False

    def enable(self, axis=None, now=None):
        axes = ("yaw", "pitch") if axis is None else (axis,)
        ok = True
        for name in axes:
            ok = self._write(name, cmd_enable(self._addr(name)), now=now) and ok
        return ok

    def move(self, axis, ccw, pulses, rpm, acc=0, now=None):
        if axis not in ("yaw", "pitch"):
            raise ValueError("axis must be yaw or pitch")
        if int(pulses) <= 0:
            return False
        return self._write(axis, cmd_position(self._addr(axis), ccw, rpm, pulses, acc=acc),
                           now=now)

    def speed(self, axis, ccw, rpm, acc=0, now=None):
        return self._write(axis, cmd_speed(self._addr(axis), ccw, rpm, acc=acc), now=now)

    def stop(self, axis=None, now=None):
        axes = ("yaw", "pitch") if axis is None else (axis,)
        ok = True
        for name in axes:
            ok = self._write(name, cmd_stop_now(self._addr(name)), now=now) and ok
        return ok

    def _addr(self, axis):
        return int(getattr(self.cfg, "ADDR_YAW" if axis == "yaw" else "ADDR_PITCH",
                           1 if axis == "yaw" else 2))

    # ------------------------------------------------------------- 接收
    def _drain(self):
        """非阻塞读走两条串口现有字节，切出完整返回帧。"""
        frames = []
        for axis, port in tuple(self._ports.items()):
            try:
                available = int(port.available(0))
                if available > 0:
                    data = port.read(min(available, int(getattr(self.cfg, "UART_RX_MAX_BYTES", 256))), timeout=0)
                    if data:
                        self._rxbuf[axis].extend(bytes(data))
                buf = self._rxbuf[axis]
                limit = max(4, int(getattr(self.cfg, "UART_RX_BUFFER_MAX_BYTES", 1024)))
                if len(buf) > limit:
                    del buf[:-limit]
                while len(buf) >= 4:
                    if buf[0] != self._addr(axis):
                        del buf[0]
                        continue
                    end = buf.find(bytes([CR]), 1)
                    if end < 0:
                        break
                    if end < 3:
                        del buf[0]
                        continue
                    frames.append((axis, bytes(buf[:end + 1])))
                    del buf[:end + 1]
                    self._stats["rx"] += 1
            except Exception:
                self._stats["err"] += 1
        return frames

    def _ingest(self, frames, now):
        """解析返回帧：统计成功/E2、E2 自动补使能、E2 洪水与 ACK 超时判障。

        返回 dict(ok, e2, reenabled, fault, reason)。本方法不碰 MaixPy，可直接单测。
        """
        now = float(now)
        if not hasattr(self, "_pending_ack_since"):
            self._pending_ack_since = {"yaw": None, "pitch": None}
        if not hasattr(self, "_ack_seen"):
            # 兼容用 __new__ 构造的轻量实例（测试/离线解析），与上面的 _pending 同口径。
            self._ack_seen = {"yaw": False, "pitch": False}
            self._no_ack_warned = {"yaw": False, "pitch": False}
        out = {"frames": list(frames), "ok": 0, "e2": 0,
               "reenabled": [], "fault": False, "reason": None}
        e2_axes = []
        for axis, frame in frames:
            status = frame_status(frame)
            self._last_status[axis] = status
            if status is None:
                continue
            # 凡是能解析出状态的帧都证明链路活着 -> 刷新该轴 ACK 时刻。
            self._last_ack[axis] = now
            self._ack_seen[axis] = True
            self._pending_ack_since[axis] = None
            self._timeout_count[axis] = 0
            if status == STATUS_OK:
                out["ok"] += 1
                self._stats["ok"] += 1
            elif status == STATUS_E2:
                out["e2"] += 1
                self._stats["e2"] += 1
                e2_axes.append(axis)
                self._e2_times.append(now)
        if not bool(getattr(self.cfg, "ENABLE_ACK_WATCHDOG", True)):
            return out
        if e2_axes and bool(getattr(self.cfg, "E2_REENABLE", True)):
            for axis in sorted(set(e2_axes)):
                if self.enable(axis, now=now):
                    out["reenabled"].append(axis)
        window = float(getattr(self.cfg, "E2_WINDOW_S", 2.0))
        self._e2_times = [t for t in self._e2_times if now - t <= window]
        limit = int(getattr(self.cfg, "E2_FAULT_COUNT", 10))
        if limit > 0 and len(self._e2_times) > limit:
            out["fault"] = True
            out["reason"] = "e2_flood(%d in %.1fs)" % (len(self._e2_times), window)
            self.fault_reason = out["reason"]
            return out
        # ACK 硬超时：**按轴独立**启用 —— 只有该轴自己回过确认帧
        # （Response=Receive/Reached/Both）才要求它继续回帧；从未回过帧的轴
        # （Response=None / 未接线 / ID 不符）只告警一次、不再据此判障，
        # 否则另一轴正常也会把整机拖进 FAULT。
        timeout_ms = float(getattr(self.cfg, "RX_TIMEOUT_MS", 0) or 0)
        if timeout_ms > 0:
            for axis in self._ports:
                if not self._ack_seen[axis]:
                    if (self._last_cmd[axis] > self._last_ack[axis]
                            and not self._no_ack_warned[axis]):
                        self._no_ack_warned[axis] = True
                        out["no_ack"] = axis
                        print("[gimbal] WARN %s 从未返回确认帧：驱动器 Response 可能不是 "
                              "Receive/Reached/Both；该轴 ACK 看门狗已停用（不影响发令，"
                              "但无法确认到位）" % axis)
                    self._timeout_count[axis] = 0
                    continue
                pending = self._pending_ack_since.get(axis)
                if pending is None and self._last_cmd[axis] > self._last_ack[axis]:
                    pending = self._last_cmd[axis]
                if pending is None or now - pending <= timeout_ms / 1000.0:
                    self._timeout_count[axis] = 0
                    continue
                if self._last_ack[axis] >= self._last_cmd[axis]:
                    self._timeout_count[axis] = 0
                    continue
                self._timeout_count[axis] += 1
                self._stats["timeout"] += 1
                if self._timeout_count[axis] > int(getattr(self.cfg, "RX_TIMEOUT_LIMIT", 20)):
                    out["fault"] = True
                    out["reason"] = "ack_timeout:%s" % axis
                    self.fault_reason = out["reason"]
                    return out
        return out

    def poll(self, now=None):
        """非阻塞读两条串口并处理反馈；返回 `_ingest` 的 dict（含 frames）。"""
        now = self._clock.perf_counter() if now is None else float(now)
        return self._ingest(self._drain(), now)

    def stats(self):
        return dict(self._stats)

    def close(self):
        for port in tuple(self._ports.values()):
            try:
                port.close()
            except Exception:
                pass
        self._ports.clear()


def estimate_move_seconds(pulses, rpm, acc=0, pulses_per_rev=3200):
    values = tuple(map(float, (pulses, rpm, acc, pulses_per_rev)))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite motion parameters")
    pulses, rpm, acc, ppr = values
    if ppr <= 0 or rpm <= 0 or not 0 <= acc <= 255:
        raise ValueError("invalid motion parameters")
    pulses, rpm = abs(pulses), min(5000., rpm)
    speed = rpm*ppr/60
    if acc == 0:
        return pulses/speed
    ramp = (256-acc)*50e-6
    acceleration = ppr/(60*ramp)
    ramp_seconds = rpm*ramp
    ramp_distance = speed*ramp_seconds
    if pulses <= ramp_distance:
        return 2*math.sqrt(pulses/acceleration)
    return 2*ramp_seconds+(pulses-ramp_distance)/speed
'''

# ===== vision.imu_heading  (vision/imu_heading.py) =====
_MOD_SRC['vision.imu_heading'] = r'''
"""Relative gyro heading, never an absolute compass. Native I/O has one owner."""
import math
import threading
import time
from collections import deque


def _mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def _unit(v):
    n = math.sqrt(sum(x*x for x in v))
    if not math.isfinite(n) or abs(n-1) > 1e-6:
        raise ValueError('IMU axes must be unit vectors')
    return tuple(v)


def open_imu():
    from maix.ext_dev import imu
    return imu.IMU('lsm6dsowtr', mode=imu.Mode.DUAL,
                   gyro_scale=imu.GyroScale.GYRO_SCALE_500DPS, gyro_odr=imu.GyroOdr.GYRO_ODR_208, block=False)


def read_gyro(device):
    raw = device.read()
    if raw is None or len(raw) != 7 or not any(raw):
        return None  # DUAL seven-value all-zero sample means not ready, not zero rotation.
    gyro = tuple(float(v) for v in raw[3:6])
    return gyro if all(map(math.isfinite, gyro)) else None


class GyroHeading:
    def __init__(self, cfg, clock=None):
        self.cfg, self.clock = cfg, clock if clock is not None else time
        self.yaw_axis = _unit(getattr(cfg, 'IMU_YAW_AXIS', (0, 0, 1)))
        self.pitch_axis = _unit(getattr(cfg, 'IMU_PITCH_AXIS', (0, -1, 0)))
        if abs(sum(a*b for a,b in zip(self.yaw_axis, self.pitch_axis))) > 1e-6:
            raise ValueError('IMU axes must be orthogonal')
        self.q = (1., 0., 0., 0.)
        self.previous = self.timestamp = None
        self.bias = (0., 0., 0.)
        self.history = deque(maxlen=512)
        self.reference = None
        self.epoch = 0
        self.bias_state = 'IDLE'
        self._bias_samples = []
        self._bias_started = None

    def reset_reference(self):
        self.reference = None

    def start_bias_calibration(self, now=None):
        # Caller must stop both motors and ensure the base is stationary.
        self._bias_started = self.clock.perf_counter() if now is None else float(now)
        self._bias_samples = []
        self.bias_state = 'CALIBRATING'
        self.reset_reference()

    def _advance(self, gyro, dt):
        rates = [(a+b)*.5*math.pi/180 for a,b in zip(self.previous, gyro)]
        speed = math.sqrt(sum(v*v for v in rates))
        if speed:
            angle = speed*dt/2
            dq = (math.cos(angle), *(v*math.sin(angle)/speed for v in rates))
            q = _mul(self.q, dq)
            norm = math.sqrt(sum(v*v for v in q))
            self.q = tuple(v/norm for v in q)

    def _angles(self):
        # Rotate the mounting-frame forward vector, then project onto left/up.
        yaw, pitch = self.yaw_axis, self.pitch_axis
        forward = (yaw[1]*pitch[2]-yaw[2]*pitch[1], yaw[2]*pitch[0]-yaw[0]*pitch[2], yaw[0]*pitch[1]-yaw[1]*pitch[0])
        vector = _mul(_mul(self.q, (0., *forward)), (self.q[0], *(-v for v in self.q[1:])))[1:]
        dot = lambda a,b: sum(x*y for x,y in zip(a,b))
        return math.degrees(math.atan2(-dot(vector, pitch), dot(vector, forward))), math.degrees(math.asin(max(-1., min(1., dot(vector, yaw)))))

    def update(self, gyro, now=None):
        now = self.clock.perf_counter() if now is None else float(now)
        if gyro is None:
            return None
        raw = tuple(map(float, gyro))
        if len(raw) != 3 or not all(map(math.isfinite, (*raw, now))):
            raise ValueError('non-finite gyro sample')
        if self.bias_state == 'CALIBRATING':
            self._bias_samples.append(raw)
            if now-self._bias_started >= getattr(self.cfg, 'IMU_BIAS_CALIBRATE_S', 1.):
                samples = self._bias_samples
                mean = tuple(sum(v[i] for v in samples)/len(samples) for i in range(3))
                std = tuple(math.sqrt(sum((v[i]-mean[i])**2 for v in samples)/len(samples)) for i in range(3))
                good = len(samples) >= 10 and max(std) <= getattr(self.cfg, 'IMU_BIAS_MAX_STD_DPS', .25) and max(map(abs, mean)) <= getattr(self.cfg, 'IMU_BIAS_MAX_DPS', 2.)
                self.bias_state = 'READY' if good else 'REJECTED'
                if good:
                    self.bias = mean
                self.timestamp = None
        gyro = tuple(a-b for a,b in zip(raw, self.bias))
        dt = None if self.timestamp is None else now-self.timestamp
        if dt is None or not 0 < dt <= getattr(self.cfg, 'IMU_MAX_GAP_S', .2):
            self.q = (1., 0., 0., 0.)
            self.history.clear()
            self.reset_reference()
            self.epoch += 1
        else:
            self._advance(gyro, dt)
        self.previous, self.timestamp = gyro, now
        yaw, pitch = self._angles()
        sample = dict(yaw=yaw, pitch=pitch, captured_at=now, epoch=self.epoch, bias_state=self.bias_state)
        self.history.append(sample)
        return dict(sample)

    def sample_at(self, timestamp):
        # Never use a future sample to explain an already captured camera frame.
        for sample in reversed(self.history):
            if sample['captured_at'] <= timestamp:
                if timestamp-sample['captured_at'] <= getattr(self.cfg, 'IMU_MAX_AGE_S', .2):
                    return dict(sample)
                break
        return None

    def lock_reference(self, now, yaw_offset_deg=0., pitch_offset_deg=0., gimbal_angles=None):
        sample = self.sample_at(now)
        if sample is None or self.bias_state == 'CALIBRATING':
            return False
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') == 'base':
            if gimbal_angles is None or not all(map(math.isfinite, gimbal_angles)):
                return False
            yaw_offset_deg += gimbal_angles[0]
            pitch_offset_deg += gimbal_angles[1]
        self.reference = (sample['yaw']+yaw_offset_deg, sample['pitch']+pitch_offset_deg, self.epoch)
        return True

    def correction(self, now, gimbal_angles=None):
        sample = self.sample_at(now)
        if sample is None or self.reference is None or self.reference[2] != self.epoch:
            return None
        yaw, pitch = sample['yaw'], sample['pitch']
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') == 'base':
            if gimbal_angles is None:
                return None
            yaw += gimbal_angles[0]
            pitch += gimbal_angles[1]
        return ((self.reference[0]-yaw+180)%360-180, self.reference[1]-pitch)


class HeadingFeedforward:
    def __init__(self, cfg, clock=None, device=None, device_factory=None):
        self.cfg, self.clock = cfg, clock if clock is not None else time
        self.heading = GyroHeading(cfg, clock)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bias_request = threading.Event()
        self._latest = None
        self.error = None
        self.device = device
        self.device_factory = device_factory or open_imu
        self.thread = threading.Thread(target=self._run, name='imu-heading', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if self.device is None:
                self.device = self.device_factory()
            while not self._stop.is_set():
                raw = read_gyro(self.device)  # Native I/O never holds the shared state lock.
                now = self.clock.perf_counter()
                with self._lock:
                    if self._bias_request.is_set():
                        self._bias_request.clear()
                        self.heading.start_bias_calibration(now)
                    if raw is not None:
                        self._latest = self.heading.update(raw, now)
                self._stop.wait(min(.02, 1/max(1, getattr(self.cfg, 'IMU_SAMPLE_HZ', 208))))
        except Exception as exc:
            self.error = str(exc)
        finally:
            # Only the owner thread may destroy a device with a native read in flight.
            if self.device is not None:
                close = getattr(self.device, 'close', None)
                if close:
                    close()

    def sample_at(self, now):
        if not self._lock.acquire(False):
            return None
        try:
            return self.heading.sample_at(now)
        finally:
            self._lock.release()

    def snapshot(self, now=None):
        return self.sample_at(self.clock.perf_counter() if now is None else now)

    def lock_reference(self, now, yaw_offset_deg=0., pitch_offset_deg=0., gimbal_angles=None):
        if not self._lock.acquire(False):
            return False
        try:
            return self.heading.lock_reference(now, yaw_offset_deg, pitch_offset_deg, gimbal_angles)
        finally:
            self._lock.release()

    def correction(self, now, gimbal_angles=None):
        if not self._lock.acquire(False):
            return None
        try:
            return self.heading.correction(now, gimbal_angles)
        finally:
            self._lock.release()

    def reset_reference(self):
        with self._lock:
            self.heading.reset_reference()

    def start_bias_calibration(self, now=None):
        self._bias_request.set()

    def close(self):
        self._stop.set()
        self.thread.join(.10)
        if self.thread.is_alive():
            raise RuntimeError('IMU reader still running')
'''

# ===== vision.gimbal_self_test  (vision/gimbal_self_test.py) =====
_MOD_SRC['vision.gimbal_self_test'] = r'''
"""上电云台小幅自检：yaw ±15°，pitch ±30°，最后回到启动姿态。

两种幅度口径（`config.SELF_TEST_PULSES` 决定用哪种）：
    · >0 = **固定脉冲**（推荐，见 AGENTS.md §3.2）：不依赖"脉冲/度"标定即可动作，
      所以可由屏幕按钮直接触发；顺序与角度模式一致（yaw 左→右、pitch 上→下，各回到起点）。
    · ==0 = 按 `SELF_TEST_*_DEG × *_PULSES_PER_DEG` 换算角度；未标定（两者均 0）则跳过。
"""

from .stepper_link import estimate_move_seconds


class GimbalSelfTest:
    """非阻塞自检状态机。角度通过实测的电机脉冲/输出轴度数换算。"""

    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.state = "IDLE"
        self.index = 0
        self.started = float(now)
        self.deadline = float(now)
        self.sequence = []
        self.reason = None
        self._prepare()

    def _prepare(self):
        if not bool(getattr(self.cfg, "GIMBAL_SELF_TEST", True)):
            self.state, self.reason = "DONE", "disabled"
            return
        # ---- 固定脉冲模式（不依赖标定；屏幕按钮触发走这条）----
        fixed = int(getattr(self.cfg, "SELF_TEST_PULSES", 0) or 0)
        if fixed > 0:
            yaw_ccw = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
            pitch_ccw = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
            self.sequence = [
                ("yaw", yaw_ccw, fixed), ("yaw", not yaw_ccw, 2 * fixed), ("yaw", yaw_ccw, fixed),
                ("pitch", pitch_ccw, fixed), ("pitch", not pitch_ccw, 2 * fixed), ("pitch", pitch_ccw, fixed),
            ]
            self.state = "RUNNING"
            print("[self-test] 固定脉冲模式 pulses=%d rpm=%d" %
                  (fixed, int(getattr(self.cfg, "SELF_TEST_RPM", 900))))
            return
        yaw_ppd = float(getattr(self.cfg, "YAW_PULSES_PER_DEG", 0.0))
        pitch_ppd = float(getattr(self.cfg, "PITCH_PULSES_PER_DEG", 0.0))
        # 未做"脉冲/度"标定时，允许用已知减速比推导：脉冲/度 = 每圈脉冲 × 减速比 / 360。
        # 减速比未知（0）则保持跳过 —— 不用错误比例去撞限位。
        ratio = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0))
        if ratio > 0:
            per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
            derived = per_rev * ratio / 360.0
            if yaw_ppd <= 0:
                yaw_ppd = derived
            if pitch_ppd <= 0:
                pitch_ppd = derived
        if yaw_ppd <= 0 or pitch_ppd <= 0:
            self.state, self.reason = "SKIPPED", "pulse_per_degree_not_calibrated"
            print("[self-test] skipped: set YAW_PULSES_PER_DEG / PITCH_PULSES_PER_DEG")
            print("[self-test]           或用 GIMBAL_GEAR_RATIO 提供减速比，或先用 CALIBRATE_AXIS 实测")
            return
        yaw_angle = abs(float(getattr(self.cfg, "SELF_TEST_YAW_DEG", 15.0)))
        pitch_angle = abs(float(getattr(self.cfg, "SELF_TEST_PITCH_DEG", 30.0)))
        yaw_ccw_positive = bool(getattr(self.cfg, "YAW_POSITIVE_CCW", False))
        pitch_ccw_positive = bool(getattr(self.cfg, "PITCH_POSITIVE_CCW", False))
        self.sequence = [
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("yaw", not yaw_ccw_positive, 2 * yaw_angle * yaw_ppd),
            ("yaw", yaw_ccw_positive, yaw_angle * yaw_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
            ("pitch", not pitch_ccw_positive, 2 * pitch_angle * pitch_ppd),
            ("pitch", pitch_ccw_positive, pitch_angle * pitch_ppd),
        ]
        self.state = "RUNNING"

    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now):
        """每帧调用；到达估算运动时间后发下一段，不阻塞视觉主循环。"""
        if self.done:
            return self.state
        now = float(now)
        if now-self.started > getattr(self.cfg, "SELF_TEST_TIMEOUT_S", 10.):
            self.state, self.reason = "FAULT", "timeout"
            self.link.stop()
            return self.state
        if now < self.deadline:
            return self.state
        if self.index >= len(self.sequence):
            self.state = "DONE"
            print("[self-test] complete; returned to startup pose")
            return self.state
        axis, ccw, raw_pulses = self.sequence[self.index]
        pulses = int(round(raw_pulses))
        if pulses <= 0:
            self.state, self.reason = "FAULT", "invalid_pulse_count"
            self.link.stop(axis)
            return self.state
        rpm = int(getattr(self.cfg, "SELF_TEST_RPM", 900))
        if not self.link.move(axis, ccw, pulses, rpm,
                              int(getattr(self.cfg, "ACC_DEFAULT", 0))):
            self.state, self.reason = "FAULT", "uart_write_failed"
            self.link.stop(axis)
            print("[self-test] fault: %s command failed" % axis)
            return self.state
        pulses_per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
        move_s = estimate_move_seconds(pulses, rpm, getattr(self.cfg, "ACC_DEFAULT", 80), pulses_per_rev)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.35))
        self.deadline = now + move_s + settle_s
        self.index += 1
        print("[self-test] %s %s %dp rpm=%d step=%d/%d" %
              (axis, "CCW" if ccw else "CW", pulses, rpm,
               self.index, len(self.sequence)))
        return self.state


def probe_imu():
    """Probe the official DUAL interface; release the device in the same thread."""
    device = None
    try:
        from .imu_heading import open_imu, read_gyro
        device = open_imu()
        return dict(available=True, method="maix.ext_dev.imu", sample=read_gyro(device), error=None)
    except Exception as exc:
        return dict(available=False, method="maix.ext_dev.imu", sample=None, error=str(exc))
    finally:
        if device is not None and hasattr(device, "close"):
            device.close()
'''

# ===== vision.gimbal_aim  (vision/gimbal_aim.py) =====
_MOD_SRC['vision.gimbal_aim'] = r'''
"""基于像素误差的两轴增量比例控制器 + 状态机。

对照 `MaixCAM2串口直控云台需求文档.md` §7 实现：

- 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 ``laser_spot.deviation``）；
- 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅，``ACC_TRACK`` 曲线加减速；
- 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / SCAN / FAULT``；
- 三重保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
  （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）。

本模块**保持纯 Python（不 import maix）**，可在 PC 上离线仿真与回归。
控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
"""

import math
from .stepper_link import estimate_move_seconds

# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
SCAN = "SCAN"        # 失锁后慢扫重捕获（需 ENABLE_SCAN）
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令


class GimbalAim:
    def __init__(self, cfg, controller=None):
        self.cfg = cfg
        self.controller = controller
        self.state = BOOT
        self.fault_reason = None
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self.last = {"state": self.state, "dx": None, "dy": None,
                     "cmds": {"yaw": None, "pitch": None}}
        self._jump = {"yaw": 0, "pitch": 0}
        self._lost = 0
        self._settle = 0
        self._scan_dir = 1
        self._scan_pos = 0
        self._busy_until = {"yaw": -1e9, "pitch": -1e9}
        self._capture_after = {"yaw": -1e9, "pitch": -1e9}
        self._direction = {"yaw": 0., "pitch": 0.}
        self._last_measurement_time = None
        self._stops = []
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}

    # ------------------------------------------------------------- 外部接口
    def set_controller(self, controller):
        """设置后续 PID/前馈控制器；传入 None 恢复内置比例律。"""
        self.controller = controller

    def fault(self, reason):
        """进入 FAULT：不再发任何指令，由上层决定是否复位。"""
        self.state = FAULT
        self.fault_reason = str(reason)
        print("[aim] FAULT: %s" % self.fault_reason)
        return self.state

    def reset(self):
        self.__init__(self.cfg, self.controller)
        self.state = BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        return self.state

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD / SCAN）。"""
        now = float(now)
        self._stops = []
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid:
            return self._on_lost(now)
        dx, dy = float(result["dx"]), float(result["dy"])
        captured = float(result.get("captured_at", now))
        if not all(map(math.isfinite, (dx, dy, now, captured))) or captured > now:
            return self._on_lost(now)
        if self._last_measurement_time is not None:
            if captured <= self._last_measurement_time:
                return self._emit(dx, dy, {"yaw": None, "pitch": None})
            if captured-self._last_measurement_time > getattr(self.cfg, "AIM_HISTORY_MAX_GAP_S", .30):
                self._jump = {"yaw": 0, "pitch": 0}
                self._guard_error = {"yaw": None, "pitch": None}
                self._diverge = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = self._captured_at = captured
        self._lost = 0
        self._scan_pos = 0
        for axis, error in (("yaw", dx), ("pitch", dy)):
            if self._busy_until[axis] > now and (abs(error) <= self._deadband() or error*self._direction[axis] < 0):
                self._stops.append(axis)
                self._busy_until[axis] = now
                self._capture_after[axis] = now
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            self._settle += 1
            self.state = SETTLE
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        self._settle = 0
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        self.state = COARSE if (coarse_px > 0 and max(abs(dx), abs(dy)) > coarse_px) else FINE
        cmds = {"yaw": self._axis_command("yaw", dx, now),
                "pitch": self._axis_command("pitch", dy, now)}
        return self._emit(dx, dy, cmds)

    def status_line(self):
        def fmt(cmd):
            if not cmd:
                return "-"
            return ("CCW" if cmd[0] else "CW") + "/%dp" % cmd[1]
        return "[aim] state=%s dx=%s dy=%s yaw=%s pit=%s lost=%d" % (
            self.last.get("state", self.state), self.last.get("dx"), self.last.get("dy"),
            fmt(self.last.get("cmds", {}).get("yaw")),
            fmt(self.last.get("cmds", {}).get("pitch")), self._lost)

    # ------------------------------------------------------------- 内部实现
    def _emit(self, dx, dy, cmds):
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds, "stops": list(self._stops)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _on_lost(self, now):
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}
        self._lost += 1
        self._settle = 0
        scan_on = bool(getattr(self.cfg, "ENABLE_SCAN", False))
        if scan_on and self._lost >= int(getattr(self.cfg, "LOST_FRAMES", 15)):
            self.state = SCAN
            cmd = self._scan_command("yaw", now)
            return self._emit(None, None, {"yaw": cmd, "pitch": None})
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _mix(self, error):
        fine, coarse = self._deadband(), float(getattr(self.cfg, "COARSE_PX", 24.))
        if not getattr(self.cfg, "AIM_PROGRESSIVE", True):
            return float(abs(error) > coarse)
        return max(0., min(1., (abs(error)-fine)/max(.001, coarse-fine)))

    def _gain(self, axis, error):
        fine = float(getattr(self.cfg, "GAIN_"+axis.upper(), .3))
        coarse = float(getattr(self.cfg, "GAIN_COARSE", .65))
        if not 0 < fine <= 1 or not 0 < coarse <= 1:
            raise ValueError("gain must be in (0, 1]")
        return fine + self._mix(error)*(coarse-fine)

    def _rpm(self, error=0):
        fine, coarse = self.cfg.RPM_FINE, self.cfg.RPM_COARSE
        return max(1, round(fine+self._mix(error)*(coarse-fine)))

    def mark_stopped(self, now, axis=None):
        for name in ("yaw", "pitch") if axis is None else (axis,):
            self._busy_until[name] = now
            self._capture_after[name] = now
            self._direction[name] = 0.

    def _axis_command(self, axis, error, now):
        cfg = self.cfg
        if self.state == FAULT:
            return None
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                return None
        else:
            self._jump[axis] = 0
        if abs(error) <= self._deadband():
            return None
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval:
            return None
        if getattr(cfg, "AIM_WAIT_MOTION", True) and (now < self._busy_until[axis] or self._captured_at <= max(self._busy_until[axis], self._capture_after[axis])):
            return None
        if getattr(cfg, "DIVERGE_GUARD", False):
            previous = self._guard_error[axis]
            if previous is not None:
                grew = abs(error) > abs(previous)*getattr(cfg, "DIVERGE_RATIO", 1.15)+getattr(cfg, "DIVERGE_MIN_PX", 4.)
                self._diverge[axis] = self._diverge[axis]+1 if grew and error*previous > 0 else 0
                if self._diverge[axis] >= getattr(cfg, "DIVERGE_CONFIRM_FRAMES", 6):
                    self.fault("diverge:"+axis)
                    return None
            self._guard_error[axis] = error
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or float(output) == 0:
                return None
            pulses = min(max_pulses, max(1, int(round(abs(float(output))))))
            ccw = float(output) * sign > 0
        else:
            ppp = float(getattr(cfg, "PULSE_PER_PX_YAW" if axis == "yaw"
                                else "PULSE_PER_PX_PITCH", 3.67))
            if not math.isfinite(ppp) or ppp <= 0:
                self.fault("invalid_scale:"+axis)
                return None
            pulses = min(max_pulses, max(1, int(round(abs(error) * ppp * self._gain(axis, error)))))
            ccw = error * sign > 0
        if not math.isfinite(sign) or abs(sign) != 1:
            self.fault("invalid_direction:"+axis)
            return None
        self.last_command_time[axis] = now
        rpm = self._rpm(error)
        self._direction[axis] = error
        self._busy_until[axis] = now+estimate_move_seconds(pulses, rpm, getattr(cfg, "ACC_TRACK", 80), getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))+getattr(cfg, "AIM_MOTION_MARGIN_S", .03)
        return ccw, pulses, rpm

    def _scan_command(self, axis, now):
        cfg = self.cfg
        if (getattr(cfg, "AIM_WAIT_MOTION", True) and now < self._busy_until[axis]) or now - self.last_command_time[axis] < float(getattr(cfg, "SCAN_STEP_INTERVAL_S", 0.15)):
            return None
        ppd = float(getattr(cfg, "YAW_PULSES_PER_DEG", 0.0))
        if ppd > 0:
            step = max(1, int(round(float(getattr(cfg, "SCAN_STEP_DEG", 1.5)) * ppd)))
            limit = max(step, int(round(float(getattr(cfg, "SCAN_LIMIT_DEG", 15.0)) * ppd)))
        else:
            step = max(1, int(getattr(cfg, "SCAN_STEP_PULSES", 40)))
            limit = max(step, int(getattr(cfg, "SCAN_LIMIT_PULSES", 400)))
        if self._scan_pos >= limit:
            self._scan_dir = -1
        elif self._scan_pos <= -limit:
            self._scan_dir = 1
        sign = float(getattr(cfg, "DIR_SIGN_YAW", 1))
        ccw = (self._scan_dir * sign) > 0
        self._scan_pos += self._scan_dir * step
        self.last_command_time[axis] = now
        rpm = int(getattr(cfg, "SCAN_RPM", 60))
        self._busy_until[axis] = now+estimate_move_seconds(step, rpm, getattr(cfg, "ACC_TRACK", 80), getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))
        return ccw, step, rpm
'''

# ===== vision.gimbal_search  (vision/gimbal_search.py) =====
_MOD_SRC['vision.gimbal_search'] = r'''
"""Button-triggered nonblocking search. Cancellation stops in place, never homes."""
import math
from .stepper_link import estimate_move_seconds


class SearchSweep:
    def __init__(self, cfg, link, now=0., heading=None):
        self.cfg, self.link, self.heading = cfg, link, heading
        self.started_at = float(now)
        self.state, self.reason = 'SEARCH', None
        self.index, self.deadline = 0, float(now)
        self.sequence = []
        self.segment = None
        self.measurements = []
        for axis in ('yaw', 'pitch'):
            limit = abs(float(getattr(cfg, 'SEARCH_'+axis.upper()+'_DEG', 45.)))
            ppd = float(getattr(cfg, axis.upper()+'_PULSES_PER_DEG', 0.))
            pulses = max(1, round(limit*ppd)) if ppd > 0 else max(1, int(getattr(cfg, 'SEARCH_PULSES', 400)))
            positive = bool(getattr(cfg, axis.upper()+'_POSITIVE_CCW', True))
            for sign, factor in ((1, 1), (-1, 2), (1, 1)):
                self.sequence.append((axis, positive if sign > 0 else not positive, pulses*factor, sign*factor*limit))

    @property
    def done(self):
        return self.state != 'SEARCH'

    def _sample(self, now):
        return self.heading.sample_at(now) if self.heading is not None else None

    def _stop(self, now, state, reason):
        ok = self.link.stop(now=now)
        self.state, self.reason = (state, reason) if ok is not False else ('FAULT', 'stop:all')
        return self.state

    def cancel(self, now):
        return self._stop(now, 'CANCELLED', 'button')

    def _target_is_fresh(self, result, now):
        if not result or not result.get('valid') or result.get('held') or result.get('predicted'):
            return False
        captured = result.get('captured_at', float('-inf'))
        return math.isfinite(captured) and self.started_at <= captured <= now and now-captured <= self.cfg.RESULT_MAX_AGE_S

    def update(self, now, result=None):
        if self.done:
            return self.state
        if self._target_is_fresh(result, now):
            return self._stop(now, 'FOUND', 'visual')
        if now-self.started_at >= getattr(self.cfg, 'SEARCH_TIMEOUT_S', 60.):
            return self._stop(now, 'DONE', 'timeout')
        sample = self._sample(now)
        if self.segment is not None:
            axis, pulses, angle, initial, sent = self.segment
            measured = None
            if sample is not None and initial is not None and sample.get('epoch') == initial.get('epoch'):
                measured = (sample[axis]-initial[axis]+180)%360-180
            early = (getattr(self.cfg, 'SEARCH_USE_IMU', True) and measured is not None and measured*angle > 0 and abs(measured) >= abs(angle)-getattr(self.cfg, 'SEARCH_IMU_MARGIN_DEG', 3.))
            if now < self.deadline and not early:
                return self.state
            if early and self.link.stop(axis, now=now) is False:
                return self._stop(now, 'FAULT', 'stop:'+axis)
            if measured is not None and abs(measured) > 1:
                value = pulses/abs(measured)
                # If stopped early, requested pulses are an upper bound, not measured travel.
                self.measurements.append(dict(axis=axis, pulses_per_deg=value, upper_bound=early))
                print('[search] %s pulses/degree%s=%.3f' % (axis, ' upper-bound' if early else '', value))
            self.segment = None
            self.deadline = now+getattr(self.cfg, 'SEARCH_SETTLE_S', .1)
            return self.state
        if now < self.deadline:
            return self.state
        if self.index >= len(self.sequence):
            self.state, self.reason = 'DONE', 'sequence_complete'
            return self.state
        axis, ccw, pulses, angle = self.sequence[self.index]
        rpm, acc = getattr(self.cfg, 'SEARCH_RPM', 60), getattr(self.cfg, 'SEARCH_ACC', 60)
        if self.link.move(axis, ccw, pulses, rpm, acc, now=now) is False:
            return self._stop(now, 'FAULT', 'tx:'+axis)
        self.segment = axis, pulses, angle, sample, now
        self.deadline = now+estimate_move_seconds(pulses, rpm, acc, self.cfg.MOTOR_PULSES_PER_REV)
        self.index += 1
        return self.state
'''

# ===== vision.aim_control  (vision/aim_control.py) =====
_MOD_SRC['vision.aim_control'] = r'''
"""Single owner of UART, priority arbitration, and captured-frame freshness gates."""
import math
import threading
import time
from .gimbal_self_test import GimbalSelfTest
from .gimbal_search import SearchSweep
from .laser_spot import LaserSpotDetector, deviation
from .rectangle_center import RectangleCenterDetector


def send_uart(uart, result):
    if uart is not None:
        payload = '@[%d,%d]%%\r\n' % (round(result['u']), round(result['v'])) if result.get('valid') else '@[invalid]%%\r\n'
        uart.write(payload.encode('ascii'))


class VisionControl:
    def __init__(self, cfg, link, aim, calibrator=None, uart=None, clock=None, heading=None):
        self.cfg, self.link, self.aim, self.calibrator, self.uart = cfg, link, aim, calibrator, uart
        self._clock = clock if clock is not None else time
        self.heading = heading
        self.self_test = self.search = None
        self.request_test = threading.Event()
        self.request_search = threading.Event()
        self.request_bias = threading.Event()
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == 'fixed'
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True
        self.notice = ''
        self.control_source = 'vision'
        self.commands = {'yaw': None, 'pitch': None}
        self._bias_active = False
        self._auto_axis = None

    def process(self, gray, captured_at):
        started = self._clock.perf_counter()
        result = self.detector.process(gray, timestamp=captured_at)
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, captured_at)
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result.update(laser_u=self.laser_state['u'], laser_v=self.laser_state['v'], dx=dv[0], dy=dv[1])
        result['aim_valid'] = dv is not None
        output = self.accept_result(result, captured_at)
        return dict(result=output, detector_result=result,
                    laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected-started)*1000, status=self.status())

    def accept_result(self, result, captured_at, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        captured_at = float(captured_at)
        fresh = (math.isfinite(now) and math.isfinite(captured_at) and
                 0 <= now-captured_at <= self.cfg.RESULT_MAX_AGE_S and captured_at > self.last_capture)
        output = dict(result) if fresh else dict(valid=False, aim_valid=False, stage='stale')
        output.update(captured_at=captured_at, age_s=now-captured_at)
        if fresh:
            self.last_capture = captured_at
        send_uart(self.uart, output)
        self._control(output, now, new_measurement=fresh)
        output.update(aim_state=self.aim.state if self.aim else None, cmds=dict(self.commands),
                      control_source=self.control_source, search_state=self.search.state if self.search else None)
        return output

    def idle(self):
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        if self.aim is not None and self.aim.state == 'FAULT':
            return 'FAULT'
        if self.self_test is not None and not self.self_test.done:
            return 'self-test %d/%d' % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return 'calibrate '+self.calibrator.axis
        if self.search is not None and not self.search.done:
            return 'SEARCH %d/%d' % (self.search.index, len(self.search.sequence))
        if self.control_source == 'imu_hold':
            return 'IMU HOLD '+self.aim.state
        return self.notice or (self.aim.state if self.aim else 'vision')

    def _stop_motion(self, now, axis=None):
        if self.link is None:
            return True
        try:
            ok = self.link.stop(now=now) if axis is None else self.link.stop(axis, now=now)
        except Exception:
            ok = False
        if ok is False:
            if axis is not None:
                try:
                    self.link.stop('pitch' if axis == 'yaw' else 'yaw', now=now)
                except Exception:
                    pass
            self.aim.fault('stop:'+(axis or 'all'))
            return False
        if hasattr(self.aim, 'mark_stopped'):
            self.aim.mark_stopped(now, axis)
        return True

    def _notice(self, text):
        self.notice = text
        print('[ui] '+text)

    def _degree_scales(self):
        if not getattr(self.cfg, 'ENABLE_IMU_FEEDFORWARD', False) or self.heading is None:
            return None
        # A base-mounted sensor needs measured gimbal angles; requested pulses are not feedback.
        if getattr(self.cfg, 'IMU_MOUNT', 'gimbal') != 'gimbal':
            return None
        scales = []
        for axis in ('yaw', 'pitch'):
            ppd = float(getattr(self.cfg, axis.upper()+'_PULSES_PER_DEG', 0))
            ppp = float(getattr(self.cfg, 'PULSE_PER_PX_'+axis.upper(), 0))
            sign = float(getattr(self.cfg, 'DIR_SIGN_'+axis.upper(), 0))
            if not all(map(math.isfinite, (ppd, ppp, sign))) or min(ppd, ppp) <= 0 or abs(sign) != 1:
                return None
            scales.append(ppd/(ppp*sign))
        return scales

    def _with_heading(self, result, now, new_measurement):
        self.control_source = 'vision'
        scales = self._degree_scales()
        if scales is None or (result is not None and result.get('stage') == 'stale'):
            return result
        if new_measurement and result and result.get('aim_valid'):
            captured = result['captured_at']
            at_capture, current = self.heading.sample_at(captured), self.heading.sample_at(now)
            if at_capture is not None and current is not None and at_capture.get('epoch') == current.get('epoch'):
                self.heading.lock_reference(captured, result['dx']/scales[0], result['dy']/scales[1])
                result = dict(result)
                # Add ONLY motion since exposure; adding absolute error double counts rotation.
                result['dx'] -= ((current['yaw']-at_capture['yaw']+180)%360-180)*scales[0]
                result['dy'] -= (current['pitch']-at_capture['pitch'])*scales[1]
                self.control_source = 'vision_imu'
            return result
        if now-self.last_capture <= getattr(self.cfg, 'IMU_HOLD_MAX_S', .25):
            correction, sample = self.heading.correction(now), self.heading.sample_at(now)
            if correction is not None and sample is not None:
                self.control_source = 'imu_hold'
                return dict(aim_valid=True, dx=correction[0]*scales[0], dy=correction[1]*scales[1], captured_at=sample['captured_at'])
        return result

    def _control(self, result, now, new_measurement):
        self.commands = {'yaw': None, 'pitch': None}
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        # FAULT before poll: even E2 re-enable is forbidden once motion is faulted.
        if aim.state == 'FAULT':
            self.request_test.clear()
            self.request_search.clear()
            return
        feedback = link.poll(now)
        if feedback and feedback.get('fault'):
            self._stop_motion(now)
            aim.fault('link:%s' % feedback.get('reason'))
            return
        busy_cal = self.calibrator is not None and not self.calibrator.done
        busy_test = self.self_test is not None and not self.self_test.done
        busy_search = self.search is not None and not self.search.done
        if self.request_test.is_set():
            self.request_test.clear()
            if busy_cal or busy_test or busy_search or self._bias_active or not self.cfg.GIMBAL_SELF_TEST:
                self._notice('SELF-TEST rejected: busy or disabled')
            elif self._stop_motion(now):
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                self._notice('收到点击 -> 启动上电自检 SELF-TEST')
                busy_test = not self.self_test.done
        if self.request_search.is_set():
            self.request_search.clear()
            if busy_search:
                self.search.cancel(now)
                self._stop_motion(now)
                if self.search.state == 'FAULT':
                    aim.fault(self.search.reason)
                self._notice('SEARCH cancelled')
                busy_search = False
            elif busy_cal or busy_test or self._bias_active or not getattr(self.cfg, 'ENABLE_SEARCH_BUTTON', True):
                self._notice('SEARCH rejected: busy or disabled')
            elif self._stop_motion(now):
                self.search = SearchSweep(self.cfg, link, now, self.heading)
                self._notice('SEARCH started')
                busy_search = True
        if self.request_bias.is_set():
            self.request_bias.clear()
            if self.heading is None or busy_cal or busy_test or busy_search:
                self._notice('IMU bias rejected: unavailable or busy')
            elif self._stop_motion(now):
                self.heading.start_bias_calibration(now)
                self._bias_active = True
                self._bias_started = now
                self._notice('IMU bias: keep base stationary')
        if aim.state == 'FAULT':
            return
        if self._bias_active:
            sample = self.heading.sample_at(now)
            if sample and sample.get('bias_state') in ('READY', 'REJECTED'):
                self._bias_active = False
                self._notice('IMU bias '+sample['bias_state'])
            elif now-self._bias_started > getattr(self.cfg, 'IMU_BIAS_CALIBRATE_S', 1.)+1:
                self._bias_active = False
                self._notice('IMU bias timeout')
            return
        if busy_cal:
            self.calibrator.update(now, result if new_measurement else None)
            if self.calibrator.done:
                if self.calibrator.state == 'FAULT' and (self.calibrator.reason or '').startswith(('tx:', 'stop:', 'uart_')):
                    self._stop_motion(now)
                    aim.fault(self.calibrator.reason)
                elif self.calibrator.state == 'DONE' and self._auto_axis == 'yaw':
                    from .gimbal_calibrate import GimbalCalibrator
                    cfg = type('PitchCalibration', (self.cfg,), {'CALIBRATE_AXIS': 'pitch'})
                    self.calibrator = GimbalCalibrator(cfg, link, now)
                    self._auto_axis = 'pitch'
            return
        if busy_test:
            self.self_test.update(now)
            if self.self_test.state == 'FAULT':
                self._stop_motion(now)
                aim.fault('self-test:'+str(self.self_test.reason))
            return
        if busy_search:
            self.search.update(now, result if new_measurement else None)
            if self.search.state == 'FAULT':
                aim.fault(self.search.reason)
            elif self.search.done:
                self._stop_motion(now)
                self._notice('SEARCH '+self.search.state)
            return
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', True):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
            aim.update({'aim_valid': False}, now=now)
            return
        result = self._with_heading(result, now, new_measurement)
        if not new_measurement and self.control_source != 'imu_hold':
            if (result is not None and result.get('stage') == 'stale' or now-self.last_capture > self.cfg.RESULT_MAX_AGE_S) and not self.stale_stopped:
                self._stop_motion(now)
                aim.update({'aim_valid': False}, now=now)
                self.stale_stopped = True
            return
        if not result or not result.get('aim_valid'):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        if aim.state == 'FAULT':
            return
        output = aim.update(result or {'aim_valid': False}, now=now)
        if aim.state == 'FAULT':
            self._stop_motion(now)
            return
        for axis in output.get('stops', ()):
            if not self._stop_motion(now, axis):
                return
        for axis, command in output['cmds'].items():
            if command is not None:
                ccw, pulses, rpm = command
                if link.move(axis, ccw, pulses, rpm, self.cfg.ACC_TRACK, now=now) is False:
                    self._stop_motion(now)
                    aim.fault('tx:'+axis)
                    return
                self.commands[axis] = command

    def close(self):
        if self.heading is not None:
            self.heading.close()
'''

# ===== vision.gimbal_calibrate  (vision/gimbal_calibrate.py) =====
_MOD_SRC['vision.gimbal_calibrate'] = r'''
"""开环标定（一次性）：在图像上实测 PULSE_PER_PX / DIR_SIGN / 交叉项 / 回差。

为什么必须有这一步：闭环能否收敛、方向是否正确，全取决于 ``PULSE_PER_PX_*`` 与
``DIR_SIGN_*``（需求文档 §7.2/§7.3）。理论换算要数齿数、查细分、量安装角，任一错就全错；
本模块直接在图像上实测 —— **"走 N 个脉冲，看靶心在图像里动了多少像素"**，一次性把
齿轮比、细分、焦距、安装角全部吸收掉。

流程（非阻塞状态机，与主循环共用链路、不阻塞视觉）::

    静置取基线 (dx0,dy0)
      -> 正方向走 N 脉冲 -> 静置 -> 测 (dx1,dy1)
      -> 反方向走 N 脉冲回位 -> 静置 -> 测 (dx2,dy2)

输出：
    PULSE_PER_PX = N / |Δ|；DIR_SIGN（走正方向后 |误差| 反而变大 ⇒ 符号反 ⇒ -1）；
    交叉项 C = Δdy/Δdx（判相机横滚）；回差估计 |Δ2−Δ0| / 2（决定 DEADBAND 下限）。

标定期 main 会跳过闭环（见 ``CALIBRATE_AXIS`` 非空）。标完把打印出的数值填进
``config.py``，再把 ``CALIBRATE_AXIS`` 置回 ``""``，重新上电即进入正常跟踪。

保持纯 Python（不 import maix），可在 PC 上用假链路回归。
"""

import math
from .stepper_link import estimate_move_seconds


class GimbalCalibrator:
    def __init__(self, cfg, link, now=0.0):
        self.cfg = cfg
        self.link = link
        self.axis = str(getattr(cfg, "CALIBRATE_AXIS", "")).strip().lower()
        self.pulses = int(getattr(cfg, "CALIBRATE_PULSES", 2000))
        self.need = max(2, int(getattr(cfg, "CALIBRATE_SAMPLES", 5)))
        self.samples = []
        self.base = self.out = self.back = None
        self.report = None
        self.reason = None
        self.last_visual = self.started = float(now)
        self.deadline = float(now) + float(getattr(cfg, "CALIBRATE_SETTLE_S", 1.0))
        self.limit_s = float(getattr(cfg, "CALIBRATE_TIMEOUT_S", 20.0))
        self.state = "BASELINE"
        if self.axis not in ("yaw", "pitch") or self.pulses <= 0:
            self.state, self.reason = "SKIPPED", "CALIBRATE_AXIS unset or invalid"
        else:
            print("[calib] start axis=%s pulses=%d samples=%d rpm=%d" %
                  (self.axis, self.pulses, self.need, int(getattr(cfg, "CALIBRATE_RPM", 300))))

    # ------------------------------------------------------------- 接口
    @property
    def done(self):
        return self.state in ("DONE", "SKIPPED", "FAULT")

    def update(self, now, result=None):
        """每帧调用；到达估算运动时间后推进状态机，不阻塞主循环。"""
        now = float(now)
        if self.done:
            return self.state
        if now - self.started > self.limit_s:
            return self._fault("timeout_no_valid_aim(%.1fs)" % self.limit_s)
        if result and result.get("aim_valid"):
            self.last_visual = now
        elif now-self.last_visual > getattr(self.cfg, "CALIBRATE_VISUAL_TIMEOUT_S", 3.):
            return self._fault("visual_timeout")
        if now < self.deadline:
            return self.state
        dv = None
        if result and result.get("aim_valid") and result.get("dx") is not None:
            dv = (float(result["dx"]), float(result["dy"]))
            if not all(map(math.isfinite, dv)):
                return self._fault("non_finite_measurement")
        if self.state == "BASELINE":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.base = self._avg()
            print("[calib] baseline  dx=%+.2f dy=%+.2f" % self.base)
            if not self._send(ccw=True):
                return self.state
            self.state = "MOVE_OUT"
            self.deadline = now + self._move_seconds()
        elif self.state == "MOVE_OUT":
            self.samples = []
            self.state = "MEASURE_OUT"
        elif self.state == "MEASURE_OUT":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.out = self._avg()
            print("[calib] after +%dp dx=%+.2f dy=%+.2f" % (self.pulses, self.out[0], self.out[1]))
            if not self._send(ccw=False):
                return self.state
            self.state = "MOVE_BACK"
            self.deadline = now + self._move_seconds()
        elif self.state == "MOVE_BACK":
            self.samples = []
            self.state = "MEASURE_BACK"
        elif self.state == "MEASURE_BACK":
            if dv is None:
                return self.state
            self.samples.append(dv)
            if len(self.samples) < self.need:
                return self.state
            self.back = self._avg()
            print("[calib] after -%dp dx=%+.2f dy=%+.2f" % (self.pulses, self.back[0], self.back[1]))
            self._finish()
        return self.state

    # ------------------------------------------------------------- 内部
    def _idx(self):
        return 0 if self.axis == "yaw" else 1

    def _avg(self):
        n = float(len(self.samples))
        return (sum(s[0] for s in self.samples) / n,
                sum(s[1] for s in self.samples) / n)

    def _move_seconds(self):
        rpm = max(1, int(getattr(self.cfg, "CALIBRATE_RPM", 300)))
        ppr = max(1.0, float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200)))
        travel = estimate_move_seconds(self.pulses, rpm, getattr(self.cfg, "ACC_DEFAULT", 80), ppr)
        settle = float(getattr(self.cfg, "CALIBRATE_SETTLE_S", 1.0))
        return travel + settle

    def _send(self, ccw):
        ok = self.link.move(self.axis, ccw, self.pulses,
                            int(getattr(self.cfg, "CALIBRATE_RPM", 300)),
                            int(getattr(self.cfg, "ACC_DEFAULT", 0)))
        if not ok:
            self._fault("uart_write_failed")
        return bool(ok)

    def _fault(self, reason):
        self.state, self.reason = "FAULT", str(reason)
        if self.link.stop(self.axis) is False:
            self.reason = "stop:"+self.axis
        print("[calib] fault: %s" % self.reason)
        return self.state

    def _finish(self):
        i = self._idx()
        delta = self.out[i] - self.base[i]
        cross = (self.out[1] - self.base[1]) if i == 0 else (self.out[0] - self.base[0])
        if abs(delta) < 0.5:
            return self._fault("no_motion(%.2fpx after %dp)" % (delta, self.pulses))
        ppp = self.pulses / abs(delta)
        sign = 1 if delta < 0 else -1
        c = cross / delta
        residual = self.back[i] - self.base[i]
        backlash = abs(residual) / 2.0
        upper = self.axis.upper()
        self.report = dict(axis=self.axis, pulse_per_px=ppp, dir_sign=sign,
                           cross=c, backlash_px=backlash, residual_px=residual)
        print("[calib] ---------------- 结果（填进 config.py）----------------")
        print("[calib]   PULSE_PER_PX_%s = %.2f" % (upper, ppp))
        print("[calib]   DIR_SIGN_%s = %d" % (upper, sign))
        print("[calib]   交叉项 C=%.3f %s" % (
            c, "→ |C|>0.25，相机横滚没装正，优先机械调整" if abs(c) > 0.25 else "(正常)"))
        print("[calib]   回差估计 ≈ %.2fpx（DEADBAND_PX 建议 ≥ 1.5×该值，下限 1.5px）" % backlash)
        print("[calib]   换轴标定请把 CALIBRATE_AXIS 改成另一轴重跑；标完置回 \"\"")
        self.state = "DONE"
        return self.state
'''

# ===== vision.app_runtime  (vision/app_runtime.py) =====
_MOD_SRC['vision.app_runtime'] = r'''
"""Application assembly, three independent time axes, and ordered hardware release."""
import time
import cv2
import numpy as np
from .camera_source import to_gray
from .rate_meter import FrameRate
from .rectangle_center import draw_result
from .laser_spot import draw_laser_overlay, PANEL_H
from .preview import display_result, aligned_frame
from .telemetry import format_perf_line, format_slow_loop, MAIN_LOOP_SLOW_MS

_active = []


def _assert_idle():
    live = [r for r in _active if getattr(r, 'thread', None) is not None and r.thread.is_alive()]
    _active[:] = live
    if live:
        raise RuntimeError('previous worker/display/IMU thread is still running')


def _configure_touch(cfg, preview, touch_class, clock):
    if not preview.enabled or not getattr(cfg, 'ENABLE_TOUCH_BUTTON', True):
        return None
    w,h = preview.width, preview.height
    margin = min(8,max(0,min(w,h)//8))
    gap = max(1,min(8,h//12))
    count = 2 if getattr(cfg,'ENABLE_SEARCH_BUTTON',True) else 1
    bw = min(max(70,int(w*.30)),w-2*margin)
    bh = min(max(34,int(h*.20)),(h-2*margin-gap*(count-1))//count)
    if bh < 3 or bw < 3:
        return None
    preview.add_button('selftest',w-bw-margin,margin,w-margin,margin+bh)
    if count == 2:
        y=margin+bh+gap
        preview.add_button('search',w-bw-margin,y,w-margin,y+bh)
    return touch_class(cfg,clock=clock)


def _draw_preview(cfg, preview, gray, result, laser, age, fps, status, now):
    view=draw_result(gray,result,preview.render_width,top_offset=PANEL_H if laser else 0)
    if laser is not None:
        draw_laser_overlay(view,laser,result,min(1.,preview.render_width/float(gray.shape[1])),use_raw=cfg.AIM_USE_RAW)
    # Keep the performance line above Preview's bottom status line.
    cv2.putText(view,'disp %.1f low1 %.1f det %.1f age %.0fms' % (preview.fps,preview.low1,fps,age*1000),
                (8,view.shape[0]-30),cv2.FONT_HERSHEY_SIMPLEX,.40,(0,255,255),1)
    preview.render(view,now,status=status)


def _capture_loop(cfg, clock, app, image, state, build_id):
    cam,worker,controller,preview,touch = (state[k] for k in ('camera','worker','controller','preview','touch'))
    capture_rate=FrameRate()
    last_log=clock.perf_counter()
    read_times,draw_times,loop_times=[],[],[]
    while not app.need_exit():
        started=clock.perf_counter()
        img=cam.read()
        if worker.error is not None:
            raise RuntimeError('vision worker failed: '+worker.error)
        if img is None:
            if not cfg.VISION_ASYNC:
                controller.idle()
            continue
        captured=clock.perf_counter()
        read_times.append((captured-started)*1000)
        if getattr(cfg,'LENS_CORR',False):
            img.lens_corr(strength=cfg.LENS_STRENGTH)
        gray=to_gray(img,image)
        capture_rate.record(captured)
        button=touch.poll(preview) if touch is not None else None
        if button == 'selftest': controller.request_test.set()
        elif button == 'search': controller.request_search.set()
        worker.submit(gray,captured)
        now=clock.perf_counter()
        snapshot,fps,_=worker.snapshot(now)
        if worker.error is not None:
            raise RuntimeError('vision worker failed: '+worker.error)
        result,laser,age=display_result(snapshot,now,cfg.RESULT_MAX_AGE_S)
        if preview.due(now):
            t_draw=clock.perf_counter()
            try:
                _draw_preview(cfg,preview,aligned_frame(snapshot,gray,result),result,laser,age,fps,snapshot['status'] if snapshot else 'waiting',now)
            except Exception as exc:
                # Drawing/conversion errors belong to display, never to motor control.
                preview.enabled=False
                print('[preview] render disabled: %s' % exc)
            draw_times.append((clock.perf_counter()-t_draw)*1000)
        loop_times.append((clock.perf_counter()-started)*1000)
        now=clock.perf_counter()
        if now-last_log >= cfg.LOG_INTERVAL:
            capture_fps,_=capture_rate.values(now)
            display_fps,low1=preview.rate.values(now)
            avg,p95,worst,n=preview.take_show_stats()
            mean=lambda values: float(np.mean(values)) if values else 0.
            loop_avg=mean(loop_times)
            detector_result=snapshot.get('detector_result',snapshot['result']) if snapshot else {}
            print(format_perf_line(build_id,capture_fps,display_fps,low1,fps,result,
                mean(read_times),mean(draw_times),loop_avg,avg,p95,worst,n,age,
                preview.shown,preview.dropped,worker.dropped,detector_result=detector_result))
            metrics = worker.metrics(now)
            print('[throughput] valid_fps=%.1f valid_low1=%.1f worker_p95=%.1fms' %
                  (metrics['valid_fps'],metrics['valid_low1'],metrics['worker_p95_ms']))
            if preview.stream is not None:
                stream = preview.stream.metrics()
                print('[stream-perf] sent=%d fps=%.1f encode=%.1fms drop=%d' %
                      (stream['sent'],stream['fps'],stream['encode_ms'],stream['dropped']))
            if state['aim'] is not None: print(state['aim'].status_line())
            if p95 > 100: preview.warn_slow_once(p95)
            if loop_avg > MAIN_LOOP_SLOW_MS: print(format_slow_loop(loop_avg))
            last_log=now
            read_times.clear(); draw_times.clear(); loop_times.clear()


def _release(state):
    errors=[]
    def close(resource):
        if resource is not None and hasattr(resource,'close'):
            try: resource.close()
            except Exception as exc: errors.append(exc)
    worker=state.get('worker')
    close(worker)
    if worker is None or worker.thread is None or not worker.thread.is_alive():
        link=state.get('link')
        if link is not None:
            try:
                if link.stop() is False: errors.append(RuntimeError('stop on exit failed'))
            except Exception as exc: errors.append(exc)
        # Controller owns heading only after successful construction.
        if state.get('controller') is not None: close(state['controller'])
        else: close(state.get('heading'))
        close(link)
        close(state.get('uart'))
    close(state.get('preview'))
    touch=state.get('touch')
    close(touch.ts if touch is not None else None)
    close(state.get('camera'))
    if errors: raise RuntimeError('cleanup: '+'; '.join(map(str,errors)))


def run(entry, build_id=None):
    _assert_idle()
    cfg,clock=entry.Config,entry.time
    state=dict(camera=None,link=None,aim=None,uart=None,heading=None,controller=None,worker=None,preview=None,touch=None)
    try:
        from maix import app,camera,image
        cv2.setNumThreads(cfg.OPENCV_THREADS)
        cam=state['camera']=camera.Camera(cfg.CAMERA_W,cfg.CAMERA_H,image.Format.FMT_GRAYSCALE,fps=cfg.CAMERA_FPS,buff_num=cfg.CAMERA_BUFFERS)
        cam.skip_frames(cfg.SKIP_FRAMES)
        calibrator=None
        if cfg.ENABLE_GIMBAL:
            from .stepper_link import StepperLink
            link=state['link']=StepperLink(cfg,clock=clock)
            aim=state['aim']=entry.GimbalAim(cfg)
            if link.enable() is False: aim.fault('enable_failed')
            calibration_cfg=type('YawCalibration',(cfg,),{'CALIBRATE_AXIS':'yaw'}) if cfg.CALIBRATE_AXIS=='auto' else cfg
            calibrator=entry.GimbalCalibrator(calibration_cfg,link,clock.perf_counter())
            if calibrator.done: calibrator=None
            info=entry.probe_imu()
            print('[imu] %s' % ('available' if info['available'] else 'unavailable: '+str(info['error'])))
            print('[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d' % (cfg.YAW_DEV,cfg.ADDR_YAW,cfg.PITCH_DEV,cfg.ADDR_PITCH,cfg.ACC_TRACK))
        elif cfg.ENABLE_UART:
            from maix import uart
            state['uart']=uart.UART(cfg.UART_PORT,cfg.UART_BAUD)
        preview=state['preview']=entry.Preview(cfg,clock=clock)
        _active.append(preview)
        state['touch']=_configure_touch(cfg,preview,entry.TouchButtons,clock)
        if getattr(cfg,'ENABLE_IMU_FEEDFORWARD',False) or (getattr(cfg,'ENABLE_SEARCH_BUTTON',True) and getattr(cfg,'SEARCH_USE_IMU',True)):
            from .imu_heading import HeadingFeedforward
            state['heading']=HeadingFeedforward(cfg,clock=clock)
            _active.append(state['heading'])
        controller=state['controller']=entry.VisionControl(cfg,state['link'],state['aim'],calibrator,state['uart'],clock=clock,heading=state['heading'])
        if cfg.CALIBRATE_AXIS=='auto': controller._auto_axis='yaw'
        worker=state['worker']=entry.LatestFrameWorker(controller.process,controller.idle,cfg.VISION_ASYNC,clock=clock)
        _active.append(worker)
        print('[rect-center] %s block=%d c=%.0f display_width=%d display_fps=%d' % (build_id or entry.BUILD_ID,cfg.RECT_BLOCK_SIZE,cfg.RECT_C,preview.render_width,cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC,preview.render_width,preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC,cfg.RESULT_MAX_AGE_S*1000))
        print('[ui] 上电自检已改为屏幕按钮：SELF-TEST / SEARCH')
        _capture_loop(cfg,clock,app,image,state,build_id or entry.BUILD_ID)
    finally:
        _release(state)
'''

# ===== vision  (vision/__init__.py) =====
_MOD_SRC['vision'] = r'''
from .rectangle_center import RectangleCenterDetector
from .laser_spot import LaserSpotDetector, deviation, draw_laser_overlay
from .gimbal_aim import GimbalAim
from .gimbal_self_test import GimbalSelfTest, probe_imu
from .gimbal_calibrate import GimbalCalibrator
from .stepper_link import StepperLink

__all__ = ["RectangleCenterDetector", "LaserSpotDetector", "deviation",
           "draw_laser_overlay", "GimbalAim", "GimbalSelfTest", "probe_imu",
           "GimbalCalibrator", "StepperLink"]
'''

# ===== target_api  (target_api.py) =====
_MOD_SRC['target_api'] = r'''
"""25Efork 视觉能力对外 API —— 识别黑框 / 追踪靶心中心（B 轨）。

把工程内部的「检测器 + 追踪器 + 光斑偏差 + 云台闭环」收成一层稳定门面，
**不同小问直接 import 就能用**，不必关心内部模块路径与参数。

──────────────────────────────── 两个核心 API ────────────────────────────────

① 识别黑框（**无状态**，每帧独立、全图搜索，适合"取一次框"）::

    from target_api import detect_black_frame

    res = detect_black_frame(gray)      # gray: numpy uint8 或 MaixPy image
    if res:                             # res 为真 = 识别到（见"返回值"）
        print(res.center)               # (u, v)
        print(res.quad)                 # 外框四角 (4,2)

② 追踪中心（**有状态**，窄带跟踪 + 失锁自动重捕获，适合连续视频流）::

    from target_api import CenterTracker

    tracker = CenterTracker()
    while True:
        res = tracker.track(gray)
        if res:
            print(res.u, res.v, res.stage)     # stage: track / roi_search / full_search

函数式快捷用法（内部持有一个默认追踪器，小问里一行搞定）::

    from target_api import track_center, reset
    res = track_center(gray)             # 等价于 CenterTracker().track()，实例复用
    reset()                              # 换场景/重开一题时清状态

──────────────────────────────── 可选：激光偏差与云台 ────────────────────────────────

需要"光斑到靶心的偏差"或"自动瞄准"的小问::

    tracker = CenterTracker(with_laser=True)     # res 额外带 laser_u/v、dx、dy
    res = tracker.track(gray)
    print(res.dx, res.dy)                        # dx = 靶心 − 光斑（正=靶心在光斑右侧）

    from target_api import AimLoop               # 完整闭环（会驱动电机，谨慎）
    loop = AimLoop()                             # 需已完成开环标定，见 README
    loop.step(frame)                             # 内部：追踪 → 偏差 → 发脉冲
    loop.stop()

──────────────────────────────── 返回值 ────────────────────────────────

统一返回 ``Result``（dict 子类，支持 ``res["u"]`` 与 ``res.u`` 两种写法）：

    valid      bool    是否识别/追踪到黑框
    u, v       float   靶心像素坐标（已按卡尔曼/可选 EMA 平滑，供显示/常规使用）
    raw_u/v    float   未平滑中心（闭环用，见 config.AIM_USE_RAW）
    center     tuple   (u, v)；无效时为 None
    quad       (4,2)   外框四角，顺序已规整（左上起逆时针）
    inner_quad (4,2)   内框四角（胶带内沿）
    stage      str     track / roi_search / full_search / no_black_frame
    fail_stage str     仅 valid=False 时：本帧实际走过的层级，如 track+roi+full（调试用）
    fail_reason str    仅 valid=False 时：track_refine / no_proposal / rejected
    ms         float   本帧检测耗时（毫秒）
    laser_u/v  float   光斑位置（仅 with_laser=True 且光斑有效时有意义）
    dx, dy     float   靶心 − 光斑（仅 with_laser=True 时；无效为 None）
    aim_valid  bool    靶心与光斑都有效才为 True

``Result`` 的布尔值 = ``valid``，所以 ``if res:`` 直接就是"识别到了吗"。

──────────────────────────────── 输入 ────────────────────────────────

``frame`` 可以是 numpy uint8 灰度图（``(H,W)`` 或 ``(H,W,1)``）、三/四通道图（按 RGB 处理），
也可以是 MaixPy 的 ``image.Image``（自动 ``image2cv``，需要板上环境）。
**本模块顶层不 import maix**，PC 上用 numpy 即可离线跑通、便于单测。

──────────────────────────────── 参数覆盖 ────────────────────────────────

不想改 ``config.py`` 时用 ``load_config(...)`` 造一份覆盖::

    cfg = load_config(RECT_MIN_AREA=500, RECT_MAX_ASPECT=2.0)
    tracker = CenterTracker(cfg)
"""

import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from vision.rectangle_center import RectangleCenterDetector
from vision.laser_spot import LaserSpotDetector, deviation

__all__ = ["Result", "load_config", "to_gray", "detect_black_frame",
           "CenterTracker", "track_center", "reset", "AimLoop", "fixed_laser_error"]

_default_tracker = None


# --------------------------------------------------------------------- 工具
class Result(dict):
    """识别/追踪结果：``res["u"]`` 与 ``res.u`` 均可；``bool(res)`` 等价于 ``res.valid``。"""

    __slots__ = ()

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError("Result has no field %r" % name)

    def __setattr__(self, name, value):
        self[name] = value

    def __bool__(self):
        return bool(self.get("valid"))

    def __repr__(self):
        if not self.get("valid"):
            if self.get("fail_stage"):
                return "Result(invalid, stage=%s, fail=%s/%s)" % (
                    self.get("stage"), self.get("fail_stage"), self.get("fail_reason", "-"))
            return "Result(invalid, stage=%s)" % self.get("stage")
        return "Result(u=%.1f, v=%.1f, stage=%s)" % (
            self.get("u", float("nan")), self.get("v", float("nan")), self.get("stage"))


def load_config(**overrides):
    """返回 config.Config（可带覆盖项，不改磁盘上的 config.py）。

    ``load_config(RECT_MIN_AREA=500)`` → 派生一个只改了该字段的配置类。
    """
    from config import Config
    if not overrides:
        return Config
    cfg = type("ApiConfig", (Config,), dict(overrides))
    if "AIM_PROFILE" in overrides:
        from config import apply_aim_profile
        apply_aim_profile(cfg)
        for key, value in overrides.items():
            setattr(cfg, key, value)
    return cfg


def fixed_laser_error(result, cfg=None, laser_uv=None):
    """仅计算未平滑靶心减固定激光点，返回(dx,dy)像素；无效/非有限/越界返回None。

    默认固定点取config；laser_uv=(u,v)可覆盖。无图像处理、无硬件副作用。
    """
    cfg = cfg if cfg is not None else load_config()
    u, v = laser_uv if laser_uv is not None else (cfg.LASER_U_NOMINAL,cfg.LASER_V_NOMINAL)
    return deviation(dict(found=True,u=u,v=v),result,use_raw=True)


def to_gray(frame, image_module=None):
    """把输入统一成 ``(H, W)`` uint8 灰度图；无法转换返回 None。

    兼容：numpy 数组（含 ``(H,W,1)`` 哑轴）、有 ``__array__`` 的对象、MaixPy ``image.Image``。
    三/四通道按 **RGB/RGBA** 解释（MaixPy 相机默认 RGB）。
    """
    if frame is None:
        return None
    a = frame
    if not isinstance(a, np.ndarray):
        if hasattr(a, "__array__"):
            a = np.asarray(a)
        else:
            if image_module is None:
                from maix import image as image_module
            a = image_module.image2cv(frame, ensure_bgr=False, copy=False)
    a = np.asarray(a)
    if a.ndim == 3 and a.shape[2] == 1:
        a = a[:, :, 0]
    elif a.ndim == 3 and a.shape[2] == 3:
        a = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    elif a.ndim == 3 and a.shape[2] == 4:
        a = cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    if a.ndim != 2 or a.size == 0:
        return None
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return a


def _wrap(raw, gray):
    """把检测器的 dict 包成 Result，并补 center / image_size 等便捷字段。"""
    res = Result(raw)
    res["center"] = (res["u"], res["v"]) if res.get("valid") else None
    res["raw_center"] = (res["raw_u"], res["raw_v"]) if res.get("valid") else None
    if gray is not None:
        res["image_size"] = (int(gray.shape[1]), int(gray.shape[0]))
    res.setdefault("ms", res.get("detector_ms", 0.))
    res.setdefault("dx", None)
    res.setdefault("dy", None)
    res.setdefault("aim_valid", False)
    return res


# ------------------------------------------------------------- API ① 识别黑框
def detect_black_frame(frame, cfg=None, detector=None):
    """**无状态**识别黑框，返回 ``Result``（没识别到则 ``valid=False``，``bool(res)`` 为假）。

    每帧都是一次完整的"缩小图Canny/Otsu 候选 → 四边暗带验证 → 亚像素精修"，不依赖上一帧，
    适合"只在某几问里取一次黑框"的用法。传 ``detector`` 可复用分配，但每次都清空跟踪状态；
    连续视频使用 ``CenterTracker``。

    :param frame:    numpy uint8 灰度图或 MaixPy image
    :param cfg:      配置类（默认 config.Config；可用 load_config(...) 覆盖）
    :param detector: 复用已有的 RectangleCenterDetector（可选）
    """
    cfg = cfg if cfg is not None else load_config()
    det = detector if detector is not None else RectangleCenterDetector(cfg)
    gray = to_gray(frame)
    if gray is None:
        return Result(valid=False, stage="empty_frame", center=None,
                      dx=None, dy=None, aim_valid=False, ms=0.0)
    started = time.perf_counter()
    det.reset()  # This API is stateless even when callers reuse a detector allocation.
    raw = det.process(gray)
    out = _wrap(raw, gray)
    out["ms"] = out.get("detector_ms", (time.perf_counter() - started) * 1000.0)
    return out


# ------------------------------------------------------------- API ② 追踪中心
class CenterTracker:
    """**有状态**中心追踪器：窄带跟踪为主，失锁后 ROI 搜索 → 全图重捕获。

    典型用法：连续 ``track(gray)``；换场景/重新开题时 ``reset()``。
    ``with_laser=True`` 时返回值额外带光斑位置与 ``dx/dy``（靶心 − 光斑）。
    """

    def __init__(self, cfg=None, with_laser=False, use_raw=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.detector = RectangleCenterDetector(self.cfg)
        self.laser = LaserSpotDetector(self.cfg) if with_laser else None
        if use_raw is None:
            use_raw = bool(getattr(self.cfg, "AIM_USE_RAW", True))
        self.use_raw = bool(use_raw)

    def reset(self):
        """清空跟踪锁定状态（换靶、换场景时调用，避免拿旧框去搜）。"""
        self.detector = RectangleCenterDetector(self.cfg)
        if self.laser is not None:
            self.laser = LaserSpotDetector(self.cfg)
        return self

    def track(self, frame, now=None):
        """追踪一帧：返回 ``Result``；``valid`` 表示本帧是否拿到靶心。"""
        gray = to_gray(frame)
        if gray is None:
            self.detector.reset()
            return Result(valid=False, stage="empty_frame", center=None,
                          dx=None, dy=None, aim_valid=False, ms=0.0)
        raw = self.detector.process(gray, timestamp=now)
        res = _wrap(raw, gray)
        if not res.get("valid"):
            return res
        if self.laser is not None:
            ls = self.laser.process(gray, now)
            res["laser"] = ls
            dv = deviation(ls, res, use_raw=self.use_raw)
            if dv is not None:
                res["laser_u"], res["laser_v"] = ls["u"], ls["v"]
                res["dx"], res["dy"] = dv
            res["aim_valid"] = dv is not None
        return res


def track_center(frame, cfg=None, now=None):
    """函数式追踪：内部复用同一个默认 ``CenterTracker``（首次调用时创建）。"""
    global _default_tracker
    if _default_tracker is None:
        _default_tracker = CenterTracker(cfg)
    return _default_tracker.track(frame, now)


def reset():
    """重置 ``track_center()`` 使用的默认追踪器。"""
    if _default_tracker is not None:
        _default_tracker.reset()


# ------------------------------------------------------- 可选：追踪 + 云台闭环
class AimLoop:
    """追踪 + 光斑偏差 + 双轴云台闭环（会驱动 ZDT 电机）。

    ⚠ 使用前必须完成开环标定（``CALIBRATE_AXIS`` 跑一次填回 config），否则方向/增益
    是占位值，可能正反馈甩头。不需要电机的问请只用 ``detect_black_frame`` /
    ``CenterTracker``，不要实例化本类。

    ``step(frame)`` 返回的 ``Result`` 比追踪器多两个字段：
        aim_state  str   状态机状态（FINE/COARSE/SETTLE/HOLD/FAULT…）
        cmds       dict  {"yaw": (ccw, pulses, rpm) or None, "pitch": ...}
    """

    def __init__(self, cfg=None, enable_motors=True, with_laser=True, clock=None, heading=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.clock = clock if clock is not None else time
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = self.aim = None
        self.closed = False
        if enable_motors:
            from vision.stepper_link import StepperLink
            from vision.gimbal_aim import GimbalAim
            self.link = StepperLink(self.cfg, clock=self.clock)
            self.aim = GimbalAim(self.cfg)
            if self.link.enable() is False:
                self.aim.fault("enable_failed")
        from vision.aim_control import VisionControl
        calibrator = None
        if enable_motors:
            from vision.gimbal_calibrate import GimbalCalibrator
            cal_cfg = type("YawCalibration", (self.cfg,), {"CALIBRATE_AXIS": "yaw"}) if self.cfg.CALIBRATE_AXIS == "auto" else self.cfg
            calibrator = GimbalCalibrator(cal_cfg, self.link, self.clock.perf_counter())
            if calibrator.done:
                calibrator = None
            if heading is None and (getattr(self.cfg, "ENABLE_IMU_FEEDFORWARD", False) or (getattr(self.cfg, "ENABLE_SEARCH_BUTTON", True) and getattr(self.cfg, "SEARCH_USE_IMU", True))):
                from vision.imu_heading import HeadingFeedforward
                heading = HeadingFeedforward(self.cfg, clock=self.clock)
        self.control = VisionControl(self.cfg, self.link, self.aim, calibrator, clock=self.clock, heading=heading)
        if self.cfg.CALIBRATE_AXIS == "auto":
            self.control._auto_axis = "yaw"

    def step(self, frame, now=None):
        if self.closed:
            raise RuntimeError("AimLoop is closed")
        captured = self.clock.perf_counter() if now is None else float(now)
        gray = to_gray(frame)
        res = self.tracker.track(gray, captured)
        out = self.control.accept_result(res, captured)
        return _wrap(out, gray)

    def idle(self):
        if not self.closed:
            self.control.idle()

    def request_search(self):
        if not self.closed:
            self.control.request_search.set()

    def stop(self):
        if self.link is not None:
            self.control._stop_motion(self.clock.perf_counter())

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.stop()
        finally:
            try:
                self.control.close()
            finally:
                if self.link is not None:
                    self.link.close()
'''

_MAIN_SRC = r'''
"""MaixPy v4 entry. Configuration is in config.py; runtime lives in vision/."""
import os
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import Config
from vision.gimbal_aim import GimbalAim
from vision.gimbal_self_test import probe_imu
from vision.gimbal_calibrate import GimbalCalibrator
from vision.rate_meter import FrameRate
from vision.camera_source import to_gray
from vision.vision_worker import LatestFrameWorker
from vision.preview import Preview, TouchButtons, display_result, apply_maixvision_stream_policy
from vision.aim_control import VisionControl, send_uart
from vision.app_runtime import run

BUILD_ID = "2026-11-04-band-rise-v1.3.1"

# Compatibility aliases for existing PC tools and board integrations.
_FrameRate, _LatestVision, _Preview, _Touch = FrameRate, LatestFrameWorker, Preview, TouchButtons
_VisionControl, _display_result, _to_gray = VisionControl, display_result, to_gray
_send_uart, _apply_maixvision_stream_policy = send_uart, apply_maixvision_stream_policy


def main():
    return run(sys.modules[__name__], BUILD_ID)


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as exc:
        if exc.name != "maix":
            raise
        print("MaixPy SDK unavailable. PC: python -B tools/check_pc.py")
'''

_MOD_ORDER = ('config', 'vision.rate_meter', 'vision.camera_source', 'vision.vision_worker', 'vision.telemetry', 'vision.maixvision_stream', 'vision.preview', 'vision.rectangle_center', 'vision.laser_spot', 'vision.stepper_link', 'vision.imu_heading', 'vision.gimbal_self_test', 'vision.gimbal_aim', 'vision.gimbal_search', 'vision.aim_control', 'vision.gimbal_calibrate', 'vision.app_runtime', 'vision', 'target_api')

_REL_OF = {'config': 'config.py', 'vision.rate_meter': 'vision/rate_meter.py', 'vision.camera_source': 'vision/camera_source.py', 'vision.vision_worker': 'vision/vision_worker.py', 'vision.telemetry': 'vision/telemetry.py', 'vision.maixvision_stream': 'vision/maixvision_stream.py', 'vision.preview': 'vision/preview.py', 'vision.rectangle_center': 'vision/rectangle_center.py', 'vision.laser_spot': 'vision/laser_spot.py', 'vision.stepper_link': 'vision/stepper_link.py', 'vision.imu_heading': 'vision/imu_heading.py', 'vision.gimbal_self_test': 'vision/gimbal_self_test.py', 'vision.gimbal_aim': 'vision/gimbal_aim.py', 'vision.gimbal_search': 'vision/gimbal_search.py', 'vision.aim_control': 'vision/aim_control.py', 'vision.gimbal_calibrate': 'vision/gimbal_calibrate.py', 'vision.app_runtime': 'vision/app_runtime.py', 'vision': 'vision/__init__.py', 'target_api': 'target_api.py'}

_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision
_pkg.__path__ = []
_sys.modules["vision"] = _pkg
for _full in _MOD_ORDER:
    if _full == "vision":
        continue
    if "." in _full:                       # vision.rectangle_center 这类子模块
        _parent, _leaf = _full.split(".", 1)
        _m = _types.ModuleType(_full)
        setattr(_sys.modules[_parent], _leaf, _m)
        _sys.modules[_full] = _m
    else:                                  # config / target_api 这类顶层模块
        _sys.modules[_full] = _types.ModuleType(_full)
for _full in _MOD_ORDER:
    _m = _sys.modules[_full]
    _m.__dict__["__name__"] = _full
    _m.__dict__["__file__"] = _REL_OF[_full]   # 源码里若用 __file__（如 target_api）
    _m.__dict__["__source__"] = _MOD_SRC[_full]
    _m.__dict__["__package__"] = "vision" if _full == "vision" else _full.rpartition(".")[0]
    exec(compile(_MOD_SRC[_full], _full.replace(".", "/") + ".py", "exec"),
         _m.__dict__)
del _m, _full, _MOD_SRC

# --------------------------------------------------------------------------
# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'main.py', "exec"), globals())
