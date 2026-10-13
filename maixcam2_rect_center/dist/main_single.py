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

生成时间：2026-10-13 23:21:12
    config.py                    sha1=80995f136143
    main.py                      sha1=c45300d6f815
    target_api.py                sha1=46285b4d3f2a
    vision/aim_control.py        sha1=9825a5376da4
    vision/app_runtime.py        sha1=f0ba8fd64c51
    vision/camera_source.py      sha1=b398fedefe6e
    vision/contour_locator.py    sha1=afc07f26feea
    vision/gimbal_aim.py         sha1=bedb9d9175cc
    vision/gimbal_calibrate.py   sha1=9c4bb08a1b99
    vision/gimbal_search.py      sha1=73d7a93c2431
    vision/gimbal_self_test.py   sha1=6044e6cd6806
    vision/imu_heading.py        sha1=0823f6d211bd
    vision/kalman_filter.py      sha1=2af32c38c919
    vision/laser_spot.py         sha1=c131498c5ff9
    vision/maixvision_stream.py  sha1=d29c3a3d6482
    vision/preview.py            sha1=163cc055da9b
    vision/rate_meter.py         sha1=6f2a054c97a0
    vision/rect_geometry.py      sha1=87aeb55ad939
    vision/rectangle_center.py   sha1=cbff71fa0cf6
    vision/stepper_link.py       sha1=7567719e71f6
    vision/telemetry.py          sha1=5877d8470530
    vision/vision_worker.py      sha1=79e9784fa9f1
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': '80995f136143126e2f53f5804d7fc18830849ef4', 'vision/rate_meter.py': '6f2a054c97a00470409a59de9dea42b0bff9c6ba', 'vision/camera_source.py': 'b398fedefe6eaf931d372abe00ba267f7d9697b5', 'vision/vision_worker.py': '79e9784fa9f1ab052404abbc3ec86b72312faf7d', 'vision/telemetry.py': '5877d8470530e6da72b31939a8a93e2bc1973f6c', 'vision/maixvision_stream.py': 'd29c3a3d6482fe8296b85502d7354a71ca3877d8', 'vision/preview.py': '163cc055da9ba97bbdd06ed51991679855f364ef', 'vision/rect_geometry.py': '87aeb55ad939adfbe18b94eee409a094c2c6f86a', 'vision/contour_locator.py': 'afc07f26feeaf217250c8556b38b1b34e2ac9f48', 'vision/kalman_filter.py': '2af32c38c919a2d59c42a061c3f93931c0d9e923', 'vision/rectangle_center.py': 'cbff71fa0cf602d93f0b04d5c8d534fa212ddd66', 'vision/laser_spot.py': 'c131498c5ff980e594b1c8d5d2feda6195caee23', 'vision/stepper_link.py': '7567719e71f6b5a2b3b850c07180fe9b37d791fe', 'vision/gimbal_self_test.py': '6044e6cd6806a610b65175c14792b6cade202e2a', 'vision/gimbal_aim.py': 'bedb9d9175cc9867d410f475d6eca2a4d3fbeee6', 'vision/gimbal_search.py': '73d7a93c243137f03afb4dd320a0a71ed60e3b43', 'vision/imu_heading.py': '0823f6d211bdc60a6adc36d2f7aeed874d329a6a', 'vision/aim_control.py': '9825a5376da44c0501bc955a9a57e487e4609176', 'vision/gimbal_calibrate.py': '9c4bb08a1b99d6feaef77bc8cf3fc56655e87676', 'vision/app_runtime.py': 'f0ba8fd64c51d84b1d6bec72a55a8ca351ea6a3a', 'target_api.py': '46285b4d3f2a31f9cdbfbc75cdf7edd1182333b0', 'main.py': 'c45300d6f8155edbd680029da2ebaaf2376963ec'}

# --------------------------------------------------------------------------
# ① 各模块源码原样嵌入，运行时装进 sys.modules（保持原 import 语义）
# --------------------------------------------------------------------------
_MOD_SRC = {}

# ===== config  (config.py) =====
_MOD_SRC['config'] = r'''
class Config:
    # Camera profile. GRAYSCALE keeps the MaixCAM2 frame buffer small.
    CAMERA_W = 640
    CAMERA_H = 480
    CAMERA_FPS = 30
    CAMERA_BUFFERS = 2
    SKIP_FRAMES = 10
    # 镜头畸变校正：默认关闭 —— 未按本镜头标定过的软件畸变校正会**引入**畸变
    # （识别黑框时画面外凸/边缘拉伸，2026-10-10 实测定案）。确需把边缘直线拉直、
    # 且已按本镜头标定 LENS_STRENGTH 时，再临时打开。
    LENS_CORR = False
    LENS_STRENGTH = 1.5
    SHOW = True
    # 视频主循环与后台识别/串口解耦；后台忙时只保留最新帧，不积压旧图像。
    # ★ 画布宽度：0 = 用**屏幕原始宽度**（推荐）。MaixCAM2 屏幕 640×480 与本工程相机
    #   (CAMERA_W/H=640×480) 宽高比一致，画布=屏幕尺寸时 show() 拿到与屏幕同尺寸的图，
    #   **驱动层不必再每帧缩放一次**（旧值 480 会让 640×480 先被缩到 480×360、再被驱动
    #   放大回 640×480，纯属来回折腾）；画布坐标还与触摸坐标天然 1:1，省掉比例换算。
    #   >0 才按比例缩小 —— 只有"MaixVision 图传带宽吃紧"时才值得用（本机屏幕会被放大回全屏）。
    SHOW_WIDTH = 0
    SHOW_FPS = 30
    SHOW_JPEG_QUALITY = 70   # 只影响 MaixVision JPEG 图传，检测始终用原始灰度图。
    # ★ MaixVision 图传总开关。
    #   背景：MaixCDK 的 maixvision_mode() 只看**环境变量 MAIXVISION 是否为 "1"**；
    #   display.Display() 构造时据此创建图传通道 img_trans，而 show() 的第一步就是
    #   img_trans->send_image()。若处于 MaixVision 模式但画面预览**没真正连上**，
    #   该函数会 `while(!init){ if(ticks-t>500) return; }` **忙等 500ms 并空转 CPU**
    #   （源码 components/vision/src/maix_image_trans.cpp）——以 ~2 次/秒计 ≈ 烧掉近一个核，
    #   相机 read / 识别 / 闭环随之塌方（板端实测 show≈505ms、read≈107ms、detect_fps=0）。
    #   v0.6.0 默认 False：保留原 MAIXVISION 会话；Preview 只在打开本地 Display 时临时
    #   置 0，避免创建原生 img_trans，之后立即还原。图传改由独立进程编码/连接本机桥，
    #   因此未连预览不会让原生 send_image() 的忙等占住 Python GIL。
    #   True = 显式仅用本地屏，关闭图传。设备本地启动且无 MAIXVISION 会话时不启动图传进程。
    DISABLE_MAIXVISION_STREAM = False
    # MaixVision 仍使用原会话/本机桥接服务；编码和网络由独立进程承担。
    SHOW_STREAM_HOST = "127.0.0.1"
    SHOW_STREAM_PORT = 7899
    SHOW_STREAM_WIDTH = 480
    SHOW_STREAM_SLOW_MS = 300.0
    SHOW_STREAM_RETRY_S = 2.0
    SHOW_STREAM_RETRY_MAX_S = 15.0
    VISION_ASYNC = True
    # ★ 显示线程化：True = 绘制完成后把画布交给**独立显示线程**去 show()，主循环**永不等待 show**。
    #   屏幕驱动或 MaixVision 图传慢时，只表现为"预览掉帧"（丢帧而非排队），
    #   识别、串口与云台闭环仍跑满相机帧率。False = 旧行为（主循环内同步 show，慢就整体卡住）。
    SHOW_ASYNC = True
    RESULT_MAX_AGE_S = 0.20  # 以采图时刻计龄；过期结果不画框、不下发追踪指令。
    UART_RX_MAX_BYTES = 256
    UART_RX_BUFFER_MAX_BYTES = 1024
    # ---- 屏幕交互：把「上电自检」做成屏幕上的可点击按钮（不再上电自动跑）----
    # True=在显示画布右上角画 SELF-TEST 按钮，点一下才触发自检；无显示/无触摸屏时自动禁用。
    ENABLE_TOUCH_BUTTON = True
    ENABLE_SEARCH_BUTTON = True
    OPENCV_THREADS = 1
    LOG_INTERVAL = 2.0
    # ---- 双 ZDT 云台链路 -------------------------------------------------
    # ★ 实测轴映射定案（2026-10-09 实机测定，见 AGENTS.md §2；勿凭直觉改）：
    #     yaw   = UART2 / B0,B1 / /dev/ttyS2 / 驱动器 ID=2
    #     pitch = UART0 / A28,A29 / /dev/ttyS0 / 驱动器 ID=1
    #   起因：最初以为 "U0=yaw、U2=pitch"，实机发现正好相反。地址跟着"电机"走，不跟着端口。
    #   首次上板请先把云台架空，并确认驱动器已设为 UART_FUN/115200/0x6B。
    ENABLE_GIMBAL = True
    YAW_DEV, YAW_BAUD = "/dev/ttyS2", 115200
    YAW_PINS = {"B0": "UART2_TX", "B1": "UART2_RX"}
    PITCH_DEV, PITCH_BAUD = "/dev/ttyS0", 115200
    # ⚠ U0(A28/A29) 并非默认复用成 UART0，走空表会静默不通 —— 必须显式 pinmap；
    #    且 ttyS0 还是系统日志口（开机有内核日志/终端回显），故下面 FLUSH_RX_ON_START=True。
    PITCH_PINS = {"A28": "UART0_TX", "A29": "UART0_RX"}
    ADDR_YAW, ADDR_PITCH = 2, 1
    # pinmap 失败时：True=报错中止；False=只告警、仍尝试打开 UART（避免把真故障藏成"打不开"）。
    PINMAP_STRICT = False
    # 开机清空两条串口的 RX 缓冲（U0 是日志口，会混入内核启动日志/终端回显）。
    FLUSH_RX_ON_START = True
    # 闭环方向符号：两轴正方向均为 CCW（yaw CCW=左 / pitch CCW=上）。按几何推断取 -1
    # （相机与激光同装在云台上 ⇒ 云台转过去时靶心在画面里反向移动）。
    # ⚠ 未经标定：上电追踪若"越追越偏"（误差持续变大、云台一路跑向一侧），把对应那项改成 +1。
    DIR_SIGN_YAW = -1
    DIR_SIGN_PITCH = -1
    PULSE_PER_PX_YAW = 3.67       # 必须用实机开环标定替换
    PULSE_PER_PX_PITCH = 3.67     # 必须用实机开环标定替换
    GAIN_YAW = 0.30
    GAIN_PITCH = 0.30
    DEADBAND_PX = 2.0
    MAX_STEP_PULSES = 80
    RPM_FINE = 60
    # 曲线加减速档位（位置模式）：0 = 不用曲线、直接以设定速度瞬时启停（卡顿/冲击根源，勿用）；
    # ≠0 时每 (256-acc)*50us 变 1RPM，acc 越小越柔（常用 20~80）。自检与闭环共用本值。
    ACC_DEFAULT = 80
    ACC_TRACK = 80               # 追踪独立于自检，非零档位使动作按曲线启停
    COMMAND_INTERVAL_S = 0.08
    # ★ 闭环"跟手 ↔ 柔和"档位（历史教训：v9 很猛、v12+ 逐版放缓；这是跷跷板不是 bug）。
    #   "smooth"（默认）= 现有值：冲击与超调小，但接近目标偏慢（用户实报"慢慢挪过去"）。
    #   "responsive"    = v9 风格：大步长 + 高转速，明显跟手；代价是超调与机械抖动风险更高，
    #                     切换前先架空试、确认机械行程与限位。
    #   生效方式：本文件末尾会把该档位的 6 个参数写回 Config（只覆盖增益/转速/限幅，
    #   死区、低通、加减速档不动）。
    AIM_PROFILE = "smooth"
    # 接近靶心时两轴分别连续减小增益/转速，不因另一轴误差大而高速微调。
    AIM_PROGRESSIVE = True
    # 相对动作完成后还要等待一张采图时间晚于动作完成的画面，避免旧误差叠加指令。
    AIM_WAIT_MOTION = True
    AIM_MOTION_MARGIN_S = 0.03
    AIM_HISTORY_MAX_GAP_S = 0.30

    # 上电自检：度数对应的脉冲数必须按实际减速机构标定。
    # 0 表示未标定，程序跳过自动扫动，避免用错误比例撞限位。
    GIMBAL_SELF_TEST = True
    SELF_TEST_YAW_DEG = 15.0
    SELF_TEST_PITCH_DEG = 30.0
    # 固定脉冲自检幅度（AGENTS.md §3.2）：>0 时**不依赖**"脉冲/度"标定即可动作，
    # 因此屏幕按钮点了就能跑（推荐）。置 0 才回退到按 SELF_TEST_*_DEG × PULSES_PER_DEG 换算角度。
    SELF_TEST_PULSES = 400
    YAW_PULSES_PER_DEG = 0.0
    PITCH_PULSES_PER_DEG = 0.0
    # 两轴正方向均为 CCW：yaw CCW=左 / CW=右，pitch CCW=上(仰) / CW=下（见 AGENTS.md §2）。
    YAW_POSITIVE_CCW = True
    PITCH_POSITIVE_CCW = True
    SELF_TEST_RPM = 900
    SELF_TEST_SETTLE_S = 0.35
    MOTOR_PULSES_PER_REV = 3200
    # 减速比未知时留 0：自检角度换算优先用上面的实测 PULSES_PER_DEG，不猜齿轮比。
    # 若已知 i = 大齿轮齿数/小齿轮齿数，填进来即可让自检在未做脉冲/度标定时也生效。
    GIMBAL_GEAR_RATIO = 0.0

    # ---- 闭环口径（需求文档 §5.2）-----------------------------------------
    # True = 用未平滑的 raw_u/raw_v 算 dx/dy（推荐）：CENTER_ALPHA 只留给显示，
    # 闭环用平滑值会附加 ≈28ms 滞后，底盘运动时直接变成瞄准误差。
    # 切换坐标系后 PULSE_PER_PX_* 必须在同一套坐标下重新标定。
    AIM_USE_RAW = True
    # 离群拒收：|误差| 超过该值先当误检丢弃，连续 JUMP_CONFIRM_FRAMES 帧才采信。
    # 防止单帧误检（反光/干扰）给出 dx=2000px 而甩头。
    JUMP_REJECT_PX = 400.0
    JUMP_CONFIRM_FRAMES = 3
    # 粗/精两段增益：|e| > COARSE_PX 用 GAIN_COARSE（大步快速逼近），否则用 GAIN_YAW/PITCH。
    # 必须保持 gain <= 1，否则增量闭环不收敛（需求文档 §7.4）。
    COARSE_PX = 24.0
    GAIN_COARSE = 0.65
    RPM_COARSE = 150

    # ---- 链路健康（ACK / E2 看门狗，需求文档 §7.9）------------------------
    ENABLE_ACK_WATCHDOG = True
    E2_REENABLE = True          # 收到 E2（未使能/堵转）自动补发一次 0xF3
    E2_WINDOW_S = 2.0
    E2_FAULT_COUNT = 10         # 窗口内 E2 超过该次数 -> FAULT 停机
    # ACK 硬超时（ms）：自动识别 —— 只有在收到过确认帧（驱动器 Response=Receive/Reached/Both）
    # 之后才启用；Response=None 时收不到确认帧，该判据自动不生效，不会误判 FAULT。0 = 关闭。
    RX_TIMEOUT_MS = 300
    RX_TIMEOUT_LIMIT = 20

    # ---- 失锁重捕获（可选，默认关）---------------------------------------
    # 上电目标不在视野内时可开启自动慢扫；扫动为小步低转速、有累计脉冲上限，
    # 避免大幅甩动。开启前先确认机械行程与限位。
    ENABLE_SCAN = False
    LOST_FRAMES = 15
    SCAN_STEP_DEG = 1.5
    SCAN_LIMIT_DEG = 15.0
    SCAN_STEP_PULSES = 40       # 未标定脉冲/度时用的脉冲步长
    SCAN_LIMIT_PULSES = 400
    SCAN_RPM = 60
    SCAN_STEP_INTERVAL_S = 0.15

    # 手动 SEARCH：相对点击时的姿态，两轴各覆盖 ±45°。
    SEARCH_YAW_DEG = 45.0
    SEARCH_PITCH_DEG = 45.0
    SEARCH_RPM = 60
    SEARCH_ACC = 60
    SEARCH_SETTLE_S = 0.10
    SEARCH_TIMEOUT_S = 60.0
    # ★ 未标定"脉冲/度"时的兜底扫描幅度（脉冲）。与 SELF_TEST_PULSES 同口径：
    #   >0 ⇒ 即使 YAW/PITCH_PULSES_PER_DEG 与 GIMBAL_GEAR_RATIO 都是 0，SEARCH
    #   也**真的会扫**（+P → −2P → +P，每轴三段）。旧行为是直接 UNCALIBRATED 一动不动，
    #   实机表现就是"按 SEARCH 和没按一样"。置 0 可显式退回旧行为。
    SEARCH_PULSES = 400
    # ★ IMU 辅助收段：启用且陀螺仪相对朝向可用时，每段按**实测角度**提前收段，
    #   于是 ±45° 与真实角度一致，不依赖脉冲/度标定（这才是"和陀螺仪结合查找"）。
    #   未标定/无样本时自动退回脉冲口径，行为不变。
    SEARCH_USE_IMU = True
    SEARCH_IMU_MARGIN_DEG = 3.0   # 实测角度进入目标 ±该值即收段

    # 内置 IMU 在摄像头/云台上；相机姿态已包含电机运动，不再减去请求的步数。
    # 需先标定安装轴和陀螺仪偏置，再启用；仅陀螺仪提供相对朝向，不是绝对航向。
    ENABLE_IMU_FEEDFORWARD = False
    IMU_SENSOR_MOUNT = "camera"
    IMU_YAW_AXIS = (0.0, 0.0, 1.0)
    IMU_PITCH_AXIS = (0.0, -1.0, 0.0)
    IMU_AXES_CALIBRATED = False
    IMU_BIAS_DPS = None
    IMU_CALIBRATE_ON_START = False
    IMU_BIAS_CALIBRATE_S = 1.0
    IMU_BIAS_MAX_STD_DPS = 0.25
    IMU_BIAS_MAX_DPS = 2.0
    IMU_POLL_HZ = 200.0
    IMU_MAX_AGE_S = 0.10
    IMU_READ_MAX_S = 0.10
    IMU_MAX_PITCH_DEG = 80.0
    IMU_MAX_GAP_S = 0.20
    IMU_MAX_RATE_DPS = 450.0
    IMU_REFERENCE_MAX_AGE_S = 0.03
    IMU_HISTORY_S = 1.0
    IMU_HISTORY_MAX_SAMPLES = 256
    IMU_MAX_PREDICT_S = 0.05
    IMU_LEAD_S = 0.03
    IMU_VISUAL_GAP_S = 0.05
    IMU_HOLD_MAX_S = 4.0
    IMU_MAX_CORRECTION_DEG = 100.0
    # 图像暂时丢失时的姿态补偿按已标定角度限幅，避免沿用视觉微调的 80 脉冲上限。
    IMU_MAX_STEP_DEG = 30.0
    IMU_GAIN = 0.75
    IMU_RPM = 180
    IMU_ACC = 80

    # ---- 开环标定（一次性；标完把 CALIBRATE_AXIS 置回 ""）----------------
    # "yaw"/"pitch"：开机只做该轴开环标定，实测并打印 PULSE_PER_PX/DIR_SIGN/交叉项/回差。
    # 标定期不发闭环指令。跑一次拿到数值填进本文件后，务必置回 ""。
    CALIBRATE_AXIS = ""
    CALIBRATE_PULSES = 2000
    CALIBRATE_RPM = 300
    CALIBRATE_SAMPLES = 5
    CALIBRATE_SETTLE_S = 1.0
    CALIBRATE_TIMEOUT_S = 20.0

    # 兼容旧版纯视觉 ASCII 输出；开启云台时默认关闭，避免同一串口混发两种协议。
    # ⚠ 本组只在 ENABLE_GIMBAL=False 的降级路径使用；ENABLE_GIMBAL=True 时电机走
    #    YAW_DEV/PITCH_DEV 两条串口，本条的 /dev/ttyS0 是系统日志口，切勿改成实际接线。
    ENABLE_UART = False
    UART_PORT = "/dev/ttyS0"
    UART_BAUD = 115200

    # ---- 候选定位（Canny → findContours → 四边形筛选，全部在低分辨率图上）----
    # Canny 门限：低/高阈值，决定"边缘提取得多细"。调高 → 碎边更多、候选更多、更慢；
    # 调低 → 低对比度的胶带边可能漏掉（弱光/远距离场景）。换镜头/换光照先动这里。
    CANNY_LOW = 35
    CANNY_HIGH = 120
    # 边缘图闭运算核边长（像素）：Canny 的细线常断一两个像素，闭一下让胶带环的轮廓闭合、
    # findContours 少出碎段。1 = 关闭该步（最省，但断线多时候选会退化）。
    EDGE_CLOSE_PX = 3
    # ★ 搜索分辨率：候选定位（阈值化/形态学/轮廓树/多边形逼近）一律先把图缩到这么宽再做。
    #   这是"未看到框时低分辨率搜索"的落点；**锁定后的中心精修仍用原始分辨率**
    #   （`_refine` / `_verify` / 解中心都在原图坐标上算，见 rectangle_center.process）。
    SEARCH_WIDTH = 320
    # 单张掩膜里参与四边形拟合的最大轮廓数（已按面积预筛；一旦凑够 MAX_CANDIDATES 个
    # 四边形就立刻停扫 —— 多边形拟合是搜索路径的头号单点开销，必须封顶）。
    SEARCH_MAX_CONTOURS = 16
    # 候选四边形的最短边（原图像素）：更小的框不可能是靶纸，直接丢，省下全分辨率验证开销。
    MIN_EDGE_LENGTH_PX = 12.0
    # 单张掩膜最多产出多少个四边形（每个再试 inner/outer 两种假设）。
    # ⚠ 它直接决定最坏帧的验证次数（≤ MAX_CANDIDATES × 2 × 掩膜级数），是"控制运行时长"的旋钮。
    MAX_CANDIDATES = 8
    # ★ 搜索口径：True = **候选数不截断**（默认，对齐 v1–v3 `laser-readout-v1`/`target-api-v3`
    #   的"虽然卡顿但一点不丢"）。False = 有界口径，按 SEARCH_MAX_PROPOSALS 截断（快，可能丢帧）。
    SEARCH_EXHAUSTIVE = True
    # 单帧验证候选数硬上限（仅 SEARCH_EXHAUSTIVE=False 时生效）。
    # ⚠ 候选数有构造上限：每级 ≤ MAX_CANDIDATES × 2 = 16、共 2 级 ≈ 32 < 128，默认**够不到**，
    #   只有手工调小（≤32）才真正裁决；预算由 ROI 搜索与全图搜索**共享**一个计数器。
    SEARCH_MAX_PROPOSALS = 128
    EDGE_SAMPLES = 24
    REFINE_RADIUS = 6
    # 亚像素精修的迭代轮数（收敛判据见 `_accept`：单轮最大位移 < 0.2px 即停）。
    # 搜索帧的候选可能离真边几个像素，只精修一轮会停在半路 —— 实机表现为"首帧锁上时
    # 中心轻微跳一下"（实测首帧 vs 第二帧中心差 1.14px，迭代后降到 0.11px）。
    # 只影响搜索帧（稳态 track 路径本来就在不动点上），代价是最多多一次窄带精修。
    REFINE_ITERATIONS = 2
    # ★ 宽半径兜底精修（2026-10-13 实机定位）：候选四边形可能整体偏离真边——
    #   实机实测：掩膜给出的候选顶边比胶带外沿低约 14px，而 `_verify` 只从边向里采
    #   2.2~6.5px（0.2~0.6×BLACK_FRAME_MM）⇒ 直接采到纸面、把纸当成"暗带"而误拒。
    #   窄半径（REFINE_RADIUS=6 / TRACK_RADIUS=12）都纠不动这种十几 px 的偏差，
    #   于是画面里黑框清清楚楚却永远判 no_black_frame。
    #   改动：`_verify` 失败时先用本半径精修一次把四边拉回真边再验；
    #   0 或 <= REFINE_RADIUS 表示关闭该兜底（退回旧行为）。
    REFINE_RADIUS_WIDE = 20
    # ★ 宽半径"救回"路径的确证门槛：救回后的候选必须**确凿落在真胶带上**。
    #   实测（2026-10-13）：真靶纸中位对比度 **123**，而负样本 negative_e880 靠宽半径
    #   救回后只有 **40** —— 用通用的 MIN_EDGE_CONTRAST=20 会直接变成误检。
    #   只在宽半径路径上要求（不影响常规候选，常规路径仍用 MIN_EDGE_CONTRAST）。
    RECOVER_MIN_CONTRAST = 80.0
    # ★ 且**四边暗带灰度的最大值**必须够黑（≤ 本值）：真胶带四边都黑，而"纸边/褶皱"
    #   这类假边界总有一两条边只是中灰。实测：真靶纸 dark=62/49/75/78（max 78）、
    #   负样本 e880 dark=140/74/121/64（max 140）⇒ 本值取 100 可分开，且两侧都有余量。
    RECOVER_MAX_DARK = 100.0
    TRACK_RADIUS = 12
    # 运动预测（治"速度快捕获不到"）：窄带精修的搜索半径 TRACK_RADIUS 保持小值（定位精度高），
    # 靠"外推本帧位置"来跟上快速移动：用上两帧四角的**逐角位移**（含平移/缩放/旋转）预测本帧
    # 四角，再在预测位置附近做窄带精修；预测失准时自动回落到上一帧位置。
    # PREDICT_MAX_PX 是单帧预测位移上限，防噪点把预测甩飞（板端帧率低、目标位移可能很大时才需调大）。
    ENABLE_MOTION_PREDICT = True
    PREDICT_MAX_PX = 60.0
    # 捕获时间间隔超过此值或时钟回退时重建速度，避免断流后沿旧速度外推。
    PREDICT_MAX_GAP_S = 0.25
    ROI_PADDING = 0.20
    MIN_EDGE_COVERAGE = 0.55
    MAX_DARK_BRIGHT_RATIO = 0.72
    WEAK_EDGE_COVERAGE = 0.25
    # ★ 允许"一条边缺失"的阈值（实机必需）：黑胶带环在角落断开、或被强反光洗白是常态，
    #   旧规则要求四边覆盖率都 ≥WEAK_EDGE_COVERAGE，实测会把唯一正确候选拒掉
    #   （现象：画面里黑框清晰可见，却一直显示 NO BLACK RECTANGLE）。
    #   豁免条件：另三条边的覆盖率都 ≥STRONG_EDGE_COVERAGE 才允许第四条边缺失。
    STRONG_EDGE_COVERAGE = 0.85
    # ★ 凸包兜底候选的启用条件：轮廓面积 / 凸包面积 < 此值才算"环形带"。
    #   只有带（黑胶带环、U 形缺口环）需要凸包补缺口；实心背景/阴影块不启用，
    #   否则每条掩膜会凭空多出大片假候选，白耗 SEARCH_MAX_PROPOSALS 验证预算。
    HULL_FILL_RATIO = 0.60
    FILTER_RESET_PX = 25.0
    MIN_AREA_RATIO = 0.025
    MAX_AREA_RATIO = 0.92
    APPROX_EPSILON = 0.025
    MIN_ANGLE_DEG = 55.0
    MIN_INNER_RATIO = 0.30
    MAX_INNER_RATIO = 0.94
    DARK_CENTER_MIN = 90.0
    MIN_EDGE_CONTRAST = 20.0
    BLACK_FRAME_MM = 18.0

    # Physical target coordinates. The center is invariant to scale, but the
    # aspect ratio must match the printed target for a perspective correction.
    TARGET_SHORT_MM = 210.0
    TARGET_LONG_MM = 297.0
    CIRCLE_SAMPLES = 72
    CIRCLE_RADIUS_MM = 2.0

    # ---- 中心时序滤波（卡尔曼，v0.8 起取代一阶 EMA）--------------------------
    # 现场症状"车一晃中心就偏、画面抖"里混着两种成分，必须区别对待：
    #   · **必须跟踪的真实运动**（底盘/云台让靶心快速位移）；
    #   · **必须抑制的测量噪声**（边缘跳变、反光造成的逐帧抖动）。
    # 一阶 EMA 对两者一视同仁：想抑噪就得减小 alpha，代价是快速运动时输出滞后
    # （≈28ms，底盘一动就变成瞄准误差）。卡尔曼用"匀速运动模型"把二者分开：
    # 噪声进不了速度状态，而匀速运动能被模型零滞后跟上。机理见 vision/kalman_filter.py。
    KALMAN_ENABLE = True
    # 过程噪声（px/s²，等效加速度）：**越大越跟手、抑噪越弱**。取 40 的依据是实测标定
    # （tools/test_kalman.py，30fps、观测噪声 σ=0.9px）——
    #   q=40  → 静止抖动 σ 1.20→0.65（0.54 倍）；匀速/匀加速跟随误差 ≤0.8px（与 q=800 几乎无差）。
    #   q=260 → 静止抖动只降到 0.68 倍，抑噪明显变差，而机动误差一点没改善。
    # ⇒ 在"抑噪换机动手感"这条曲线上，40 附近的边际收益已经吃满，再调大纯亏抑噪。
    # ⚠ 它**只作用在 u/v（显示与常规口径）**：闭环用的 raw_u/raw_v 仍是逐帧原始测量
    #   （config.AIM_USE_RAW=True），所以滤波再重也不会变成瞄准滞后。
    KALMAN_PROCESS_PX = 40.0
    # 观测噪声（px）：**越大越平滑、滞后越大**。当前精修后的逐帧中心误差约 0.7px。
    KALMAN_MEASURE_PX = 0.9
    # 观测跳变上限（px）：超过即判"目标跳走了"，重建滤波器而不是硬拖（与 FILTER_RESET_PX 同口径）。
    KALMAN_RESET_PX = 25.0
    # 单次外推的时间上限（s）：超过视为断流，不外推、直接重建（与 PREDICT_MAX_GAP_S 同口径）。
    KALMAN_MAX_PREDICT_S = 0.25
    # 速度限幅（px/s）：防误检把速度状态甩飞（画面 640px 宽、30fps 下 6000px/s 已远超物理可能）。
    KALMAN_VELOCITY_LIMIT_PX_S = 6000.0
    # ---- 正视图（"透视变换成正视图再逆透视反投影"）------------------------------
    # ★ 逐帧输出的中心只有一条路径：`面中心 (w/2,h/2) 经 mm→px 单应逆投影` ——
    #   它就是"正视图取中心再逆投影"的闭式等价写法（正视图中心恒在 (w/2,h/2)），
    #   而且**没有重采样误差**（实测重采样版在 128px 短边下偏 3~4px、256px 下约 1px）。
    #   所以热路径不走重采样，本项默认 0 = 关闭。
    # 本项 >0 时启用 `RectangleCenterDetector.front_view()/front_view_center()`：
    #   把框透视变换成正视图（短边这么多像素），供**可视化**（tools/diagnose_detect.py --front）
    #   与**独立校验**（剖面法中心 vs 解析中心，见 tools/test_real_scene.py）。
    #   建议诊断时用 256 或更大；它不进逐帧路径，不影响帧率。
    FRONT_VIEW_SHORT_PX = 0

    # 旧口径保留：仅当 KALMAN_ENABLE=False 时作为一阶低通使用（现场对照用）。0 会禁用平滑。
    CENTER_ALPHA = 0.70

    # ---- 激光光斑读数（相机与激光固连 -> 光斑位置近似固定）------------------
    # 只在标称位置周围 LASER_ROI_RADIUS 内找最亮点，输出亚像素质心，并给出它到
    # 黑框中心的水平/垂直偏差，画在屏幕左上角。2026-10-07 现场标定：光斑约在
    # (294, 202)，背景灰度 145 / 峰值 177。换镜头或重装激光笔后重新标定这两个值，
    # 也能直接看板端日志 [perf] ... laser= 与屏幕读数反推真实位置。
    ENABLE_LASER = True
    LASER_U_NOMINAL = 294
    LASER_V_NOMINAL = 202
    LASER_ROI_RADIUS = 64
    # 光斑位置从哪来：
    #   "fixed"（默认） 不检测，直接用上面的标定常量 —— 光斑位置写死，每帧只算
    #                  "黑框中心 − 该常量"的像素差，零检测开销；
    #   "detect"        每 LASER_INTERVAL_S 实测一次光斑（自校准，吸收机构/温度漂移）。
    # 需要重新标定 U/V 时：临时改成 "detect"，把屏幕左上角读到的 `LASER (u,v)` 填回上面两行。
    LASER_MODE = "fixed"
    # 判据：峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)，且亮点面积在
    # [MIN_PIXELS, MAX_PIXELS] 内。面积上限用来挡白墙高光/灯罩反光（那些会很大）。
    LASER_MIN_PEAK = 140.0
    LASER_MIN_CONTRAST = 20.0
    LASER_MIN_PIXELS = 2
    LASER_MAX_PIXELS = 160
    # 限频：默认 0.30s ≈ 每秒算 3 次，其余帧沿用上次结果（显示仍是每帧画）。
    LASER_INTERVAL_S = 0.30
    # 单次更新跳变上限（像素）：超过判为误检，保留上次位置。0 = 关闭该门限。
    LASER_MAX_JUMP_PX = 40.0
    # 位置平滑（0 = 不平滑，直接用最新质心）。
    LASER_SMOOTH_ALPHA = 0.60


# =============================================================================
# 闭环档位落地：按 Config.AIM_PROFILE 覆盖增益/转速/单帧限幅。
#   · 现场只改上面的 `AIM_PROFILE = "smooth"`（或 "responsive"）这一行即可，
#     不需要逐个调 6 个参数（它们容易被改歪，见根 AGENTS.md §6 的跷跷板说明）。
#   · 只覆盖列出的 6 项；DEADBAND_PX / 低通 / ACC_TRACK / AIM_WAIT_MOTION 一律不动。
#   · 未识别的档位名 → 直接报错（宁可启动失败，也不要在板上悄悄用错参数）。
# =============================================================================
AIM_PROFILES = {
    "smooth": dict(GAIN_YAW=0.30, GAIN_PITCH=0.30, GAIN_COARSE=0.65,
                   RPM_FINE=60, RPM_COARSE=150, MAX_STEP_PULSES=80),
    "responsive": dict(GAIN_YAW=0.50, GAIN_PITCH=0.50, GAIN_COARSE=0.90,
                       RPM_FINE=120, RPM_COARSE=400, MAX_STEP_PULSES=360),
}
if Config.AIM_PROFILE not in AIM_PROFILES:
    raise ValueError("Config.AIM_PROFILE 必须是 %s 之一，当前为 %r"
                     % (sorted(AIM_PROFILES), Config.AIM_PROFILE))
for _name, _value in AIM_PROFILES[Config.AIM_PROFILE].items():
    setattr(Config, _name, _value)
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
        self.dropped = 0                          # 因来不及处理而被覆盖的帧数（丢帧计数）
        self.rate = FrameRate()                   # 识别帧率（detect_fps 口径）
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
        snapshot = self.callback(*item)  # callback = VisionControl.process（识别+偏差+闭环）
        with self._condition:
            self._latest = snapshot
            self.rate.record(self._clock.perf_counter())

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
        with self._condition:
            fps, low1 = self.rate.values(now)
            return self._latest, fps, low1

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
                     age_s, shown, drop_show, drop_vision, detector=None):
    """拼装 [perf] 行。

    读法：[perf] 里 capture / disp / detect_fps 是三路**不同**口径 ——
      "眼睛看到的刷新率"看 disp，"识别吞吐"看 detect_fps，别互相换算。
      read/draw/loop 属于**主循环**，show 属于**显示线程** —— 分开看就能一眼判定
      "到底谁在吃时间"（这正是 1.5fps 归因的关键）。

    ★ `det_*` 是**检测器自己的**结果，与前面的 valid/stage **不是一回事**：
      valid/stage 取自显示口径，结果过期时会被 `display_result` 整体替换成
      `{valid:false, stage:"stale"}` —— 于是 `stage=stale` 会掩盖检测器真实的
      track/roi_search/full_search/no_black_frame。2026-10-13 板端排查时正是被这一点
      挡住：分不清"检测成功但太慢"（改性能/新鲜度）与"检测失败"（改算法），而两者修法相反。
      det_stage / det_valid / det_ms 三个字段就是那次诊断盲区的对症修复。
    """
    detector = detector or {}
    return ("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
            "valid=%s stage=%s det_valid=%s det_stage=%s det_ms=%.1f | "
            "read=%.1f draw=%.1f loop=%.1f ms | "
            "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
            "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
            (build_id, capture_fps, display_fps, display_low1, vision_fps,
             result.get("valid", False), result.get("stage"),
             detector.get("valid", False), detector.get("stage"),
             float(detector.get("detector_ms", 0.0)),
             read_avg, draw_avg, loop_avg,
             show_avg, show_p95, show_max, show_n,
             age_s * 1000, shown, drop_show, drop_vision))


def format_slow_loop(loop_avg):
    """主循环偏慢（且此数**不含 show**）时的归因提示。"""
    return ("[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大；"
            "若 show 也恒定 ~505ms 则是图传忙等抢 CPU" % loop_avg)
'''

# ===== vision.maixvision_stream  (vision/maixvision_stream.py) =====
_MOD_SRC['vision.maixvision_stream'] = '"""MaixVision protocol-0 sender, isolated from camera/display/UART and their GIL.\n\nProtocol reference: Sipeed/MaixCDK 30f4b8b7, vision/src/maix_image_trans.cpp.\nThe board already runs the WebSocket bridge on localhost:7899; we are its CLIENT.\nOnly one bounded raw frame lives in an mmap. The producer takes a nonblocking\nfile lock and drops a frame if busy. JPEG, WebSocket and reconnect run in a fresh\nPython process which never imports maix (that import opens a default UART).\nThe source string also works when tools/build_single embeds this module.\n"""\ntry:\n    import fcntl\nexcept ImportError:  # PC/single-file smoke on Windows; stream needs the Linux board.\n    fcntl = None\nimport mmap\nimport os\nimport struct\nimport subprocess\nimport sys\nimport tempfile\nimport time\n\nimport numpy as np\n\n\n_CHILD_SOURCE = r\'\'\'\nimport base64\nimport fcntl\nimport hashlib\nimport mmap\nimport os\nimport select\nimport socket\nimport struct\nimport sys\nimport time\nimport cv2\nimport numpy as np\n\nHEADER = 128\nFRAME = struct.Struct("<QIII")\nSTATUS = struct.Struct("<iddQQ")\nSTATUS_OFFSET = 24\nMAGIC = bytes((0xAC, 0xBE, 0xCB, 0xCA))\nMAX_CONTROL = 65536\n\n\nclass ProtocolError(Exception):\n    pass\n\n\ndef protocol_frame(message, payload=b""):\n    body = bytes((0, message)) + payload\n    frame = MAGIC + struct.pack("<I", len(body) + 1) + body\n    return frame + bytes((sum(frame) & 255,))\n\n\ndef protocol_parse(frame):\n    if (len(frame) < 11 or len(frame) > MAX_CONTROL or frame[:4] != MAGIC\n            or struct.unpack_from("<I", frame, 4)[0] != len(frame) - 8\n            or frame[8] != 0 or sum(frame[:-1]) & 255 != frame[-1]):\n        raise ProtocolError("unsupported/corrupt MaixVision protocol")\n    return frame[9], frame[10:-1]\n\n\nclass WebSocketClient:\n    """Bounded RFC6455 client. No proxy: this is the local board bridge socket."""\n    def __init__(self, host, port, timeout=.5):\n        self.sock = socket.create_connection((host, port), timeout=timeout)\n        self.buffer = bytearray()\n        try:\n            key = base64.b64encode(os.urandom(16)).decode("ascii")\n            request = ("GET / HTTP/1.1\\r\\nHost: %s:%d\\r\\nUpgrade: websocket\\r\\n"\n                       "Connection: Upgrade\\r\\nSec-WebSocket-Key: %s\\r\\n"\n                       "Sec-WebSocket-Version: 13\\r\\n\\r\\n" % (host, port, key))\n            self.sock.sendall(request.encode("ascii"))\n            deadline = time.monotonic() + timeout\n            while b"\\r\\n\\r\\n" not in self.buffer:\n                if len(self.buffer) > 4096 or time.monotonic() > deadline:\n                    raise ProtocolError("WebSocket handshake exceeded bound")\n                chunk = self.sock.recv(4096)\n                if not chunk:\n                    raise ConnectionError("bridge closed during handshake")\n                self.buffer.extend(chunk)\n            raw, remaining = self.buffer.split(b"\\r\\n\\r\\n", 1)\n            if len(raw) > 4096:\n                raise ProtocolError("WebSocket handshake exceeded bound")\n            self.buffer = bytearray(remaining)\n            lines = raw.decode("ascii").split("\\r\\n")\n            headers = {}\n            for line in lines[1:]:\n                name, value = line.split(":", 1)\n                headers[name.lower()] = value.strip()\n            accept = base64.b64encode(hashlib.sha1(\n                (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()).decode("ascii")\n            if (lines[0].split()[1] != "101"\n                    or headers.get("upgrade", "").lower() != "websocket"\n                    or "upgrade" not in [x.strip() for x in headers.get("connection", "").lower().split(",")]\n                    or headers.get("sec-websocket-accept") != accept\n                    or "sec-websocket-extensions" in headers):\n                raise ProtocolError("invalid WebSocket handshake")\n        except Exception:\n            self.sock.close()\n            raise\n\n    def send(self, payload, opcode=2):\n        # All CLIENT frames must be masked, including ping replies.\n        n = len(payload)\n        if n > 4 * 1024 * 1024 or (opcode >= 8 and n > 125):\n            raise ProtocolError("outbound WebSocket payload exceeded bound")\n        head = bytes((0x80 | opcode, 0x80 | (n if n < 126 else 126 if n < 65536 else 127)))\n        if n >= 126:\n            head += struct.pack("!H" if n < 65536 else "!Q", n)\n        mask = os.urandom(4)\n        data = np.frombuffer(payload, np.uint8).copy()\n        for i in range(4):\n            data[i::4] ^= mask[i]\n        self.sock.sendall(head + mask + data.tobytes())\n\n    def _read(self, n):\n        deadline = time.monotonic() + .5\n        while len(self.buffer) < n:\n            if time.monotonic() > deadline:\n                raise TimeoutError("partial WebSocket frame timed out")\n            chunk = self.sock.recv(min(65536, max(1, n - len(self.buffer))))\n            if not chunk:\n                raise ConnectionError("bridge disconnected")\n            self.buffer.extend(chunk)\n        value = bytes(self.buffer[:n])\n        del self.buffer[:n]\n        return value\n\n    def receive(self):\n        if not self.buffer and not select.select([self.sock], [], [], 0)[0]:\n            return None\n        a, b = self._read(2)\n        if not a & 0x80 or a & 0x70 or b & 0x80:\n            raise ProtocolError("unsupported fragmented/extended/masked server frame")\n        opcode, n = a & 15, b & 127\n        if n in (126, 127):\n            n = struct.unpack("!H" if n == 126 else "!Q", self._read(2 if n == 126 else 8))[0]\n        if n > MAX_CONTROL or (opcode >= 8 and n > 125):\n            raise ProtocolError("bridge control payload exceeded bound")\n        payload = self._read(n)\n        if opcode == 8:\n            raise ConnectionError("bridge closed WebSocket")\n        if opcode == 9:\n            self.send(payload, opcode=10)\n            return None\n        if opcode == 10:\n            return None\n        if opcode != 2:\n            raise ProtocolError("expected binary MaixVision protocol")\n        return payload\n\n    def close(self):\n        self.sock.close()\n\n\ndef _status(mm, fd, state, sent=0, sequence=0, duration=0.0):\n    try:\n        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)\n    except BlockingIOError:\n        return\n    try:\n        STATUS.pack_into(mm, STATUS_OFFSET, state, time.monotonic(), duration, sent, sequence)\n    finally:\n        fcntl.flock(fd, fcntl.LOCK_UN)\n\n\ndef _latest(mm, fd, capacity, previous):\n    try:\n        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)\n    except BlockingIOError:\n        return None\n    try:\n        sequence, width, height, n = FRAME.unpack_from(mm)\n        if not sequence or sequence == previous:\n            return None\n        if (width <= 0 or height <= 0 or n != width * height * 3 or n > capacity):\n            raise ProtocolError("invalid shared frame dimensions")\n        return sequence, np.frombuffer(mm[HEADER:HEADER + n], np.uint8).reshape(height, width, 3)\n    finally:\n        fcntl.flock(fd, fcntl.LOCK_UN)\n\n\ndef _run_sender(fd, capacity, parent, host, port, fps, quality, retry, retry_max):\n    cv2.setNumThreads(1)\n    # flock locks belong to an open file description. pass_fds inherits the\n    # parent\'s description, whose locks would otherwise NOT exclude this child.\n    inherited = fd\n    fd = os.open("/proc/self/fd/%d" % inherited, os.O_RDWR)\n    os.close(inherited)\n    mm = mmap.mmap(fd, HEADER + capacity)\n    client = None\n    sent = previous = 0\n    retry_max = max(retry, retry_max)\n    delay, next_retry, next_image = retry, 0.0, 0.0\n    duration = 0.0\n    state, fmt = 0, 1\n    ready, handshake_deadline = False, 0.0\n    try:\n        while os.getppid() == parent and not mm[124]:\n            now = time.monotonic()\n            if client is None:\n                _status(mm, fd, state, sent, previous, duration)\n                if now < next_retry:\n                    time.sleep(.02)\n                    continue\n                try:\n                    client = WebSocketClient(host, port)\n                    client.send(protocol_frame(1, b"code"))\n                    ready, fmt = False, 1\n                    handshake_deadline = time.monotonic() + 2.0\n                    state = 0\n                except Exception as exc:\n                    client = None\n                    state, next_retry = (3 if isinstance(exc, ProtocolError) else 2), time.monotonic() + delay\n                    delay = min(retry_max, delay * 2)\n                    continue\n            try:\n                # Bounded control work: an abusive peer cannot starve newest images.\n                for _ in range(8):\n                    raw = client.receive()\n                    if raw is None:\n                        break\n                    message, payload = protocol_parse(raw)\n                    if message == 2:\n                        if payload != b"\\x01":\n                            raise ProtocolError("MaixVision rejected sender")\n                        ready, state, delay = True, 1, retry\n                    elif message == 14 and len(payload) == 1:\n                        requested = payload[0]\n                        if requested <= 2:\n                            fmt = requested\n                        client.send(protocol_frame(15, bytes((int(requested <= 2), requested))))\n                        state = 5 if ready and fmt == 0 else 1 if ready else 0\n                    elif message == 21 and len(payload) == 1:\n                        quality = min(100, payload[0])\n                        client.send(protocol_frame(22, bytes((1, quality))))\n                    else:\n                        raise ProtocolError("unsupported MaixVision control message")\n                if not ready and now > handshake_deadline:\n                    raise TimeoutError("MaixVision sender acknowledgment timed out")\n                if ready and fmt and now >= next_image:\n                    frame = _latest(mm, fd, capacity, previous)\n                    if frame is not None:\n                        sequence, array = frame\n                        started = time.monotonic()\n                        extension = ".jpg" if fmt == 1 else ".png"\n                        args = [cv2.IMWRITE_JPEG_QUALITY, quality] if fmt == 1 else [cv2.IMWRITE_PNG_COMPRESSION, 1]\n                        ok, encoded = cv2.imencode(extension, array, args)\n                        if not ok:\n                            raise RuntimeError("preview encoding failed")\n                        client.send(protocol_frame(6, bytes((fmt,)) + encoded.tobytes()))\n                        previous, sent = sequence, sent + 1\n                        next_image = time.monotonic() + 1.0 / max(1.0, fps)\n                        duration = time.monotonic() - started\n                        _status(mm, fd, state, sent, previous, duration)\n                _status(mm, fd, state, sent, previous, duration)\n                time.sleep(.005 if ready else .02)\n            except Exception as exc:\n                state = 3 if isinstance(exc, ProtocolError) else 2\n                client.close()\n                client, ready = None, False\n                next_retry = time.monotonic() + delay\n                delay = min(retry_max, delay * 2)\n                _status(mm, fd, state, sent, previous)\n    finally:\n        if client is not None:\n            client.close()\n        _status(mm, fd, 4, sent, previous)\n        mm.close()\n        os.close(fd)\n\'\'\'\n\n# Executing only defines the protocol helpers. No connection, worker or SDK import.\nexec(_CHILD_SOURCE.replace("import fcntl\\n", ""), globals())\n\n\nclass MaixVisionStream:\n    """One frame slot + one sender process. submit never waits for encoding/network."""\n    STATES = {0: "connecting", 1: "ready", 2: "retrying", 3: "unsupported",\n              4: "closed", 5: "paused"}\n\n    def __init__(self, cfg, width, height):\n        if fcntl is None:\n            raise RuntimeError("MaixVision isolated stream requires the Linux board")\n        self.cfg = cfg\n        self.capacity = max(1, int(width) * int(height) * 3)\n        self.file = tempfile.TemporaryFile(prefix="maixvision-frame-")\n        self.file.truncate(HEADER + self.capacity)\n        self.mm = mmap.mmap(self.file.fileno(), HEADER + self.capacity)\n        self.process = None\n        self.sequence = self.dropped = 0\n        self.closed = False\n        self.failed = False\n        self._last_status = {"state": "connecting", "sent": 0, "sequence": 0,\n                             "duration_ms": 0.0, "heartbeat": 0.0}\n\n    def _start(self):\n        fd = self.file.fileno()\n        args = [fd, self.capacity, os.getpid(), getattr(self.cfg, "SHOW_STREAM_HOST", "127.0.0.1"),\n                int(getattr(self.cfg, "SHOW_STREAM_PORT", 7899)),\n                max(1, int(getattr(self.cfg, "SHOW_FPS", 30))),\n                max(0, min(100, int(getattr(self.cfg, "SHOW_JPEG_QUALITY", 70)))),\n                max(.05, float(getattr(self.cfg, "SHOW_STREAM_RETRY_S", 2.0))),\n                max(.05, float(getattr(self.cfg, "SHOW_STREAM_RETRY_MAX_S", 15.0)))]\n        script = _CHILD_SOURCE + \'\\n_run_sender(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4], int(sys.argv[5]), float(sys.argv[6]), int(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]))\\n\'\n        self.process = subprocess.Popen([sys.executable, "-c", script] + [str(x) for x in args],\n                                        pass_fds=(fd,), stdin=subprocess.DEVNULL,\n                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)\n\n    def submit(self, frame):\n        if self.closed or self.failed:\n            return False\n        array = np.asarray(frame)\n        if (array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3\n                or not array.shape[0] or not array.shape[1]\n                or array.nbytes > self.capacity):\n            self.dropped += 1\n            return False\n        if self.process is None:\n            try:\n                self._start()\n            except Exception as exc:\n                self.failed = True\n                print("[preview] 图传进程不可用，保留本地显示: %s" % exc)\n                return False\n        if self.process.poll() is not None:\n            self.failed = True\n            print("[preview] 图传进程已退出，保留本地显示（请重新启动应用）")\n            return False\n        try:\n            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)\n        except BlockingIOError:\n            self.dropped += 1\n            return False\n        try:\n            self.sequence += 1\n            self.mm[HEADER:HEADER + array.nbytes] = array.tobytes()\n            FRAME.pack_into(self.mm, 0, self.sequence, array.shape[1], array.shape[0], array.nbytes)\n        finally:\n            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)\n        return True\n\n    def status(self):\n        if self.closed:\n            return {"state": "closed", "sent": 0, "dropped": self.dropped}\n        if self.failed:\n            return {"state": "unavailable", "sent": 0, "dropped": self.dropped}\n        try:\n            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)\n        except BlockingIOError:\n            return dict(self._last_status, dropped=self.dropped)\n        try:\n            state, heartbeat, duration, sent, sequence = STATUS.unpack_from(self.mm, STATUS_OFFSET)\n            self._last_status = {"state": self.STATES.get(state, "unsupported"), "sent": sent,\n                                 "sequence": sequence, "duration_ms": duration * 1000.0,\n                                 "heartbeat": heartbeat}\n        finally:\n            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)\n        return dict(self._last_status, dropped=self.dropped)\n\n    def close(self):\n        if self.closed:\n            return\n        self.closed = True\n        self.mm[124] = 1\n        try:\n            if self.process is not None:\n                try:\n                    self.process.wait(timeout=.8)\n                except subprocess.TimeoutExpired:\n                    self.process.kill()\n                    self.process.wait(timeout=.5)\n        finally:\n            self.mm.close()\n            self.file.close()\n'

# ===== vision.preview  (vision/preview.py) =====
_MOD_SRC['vision.preview'] = r'''
"""显示层：标注画面 →（letterbox）画布 → display.show()，附屏幕可点击按钮（从 main.py 搬迁，v0.4.6）。

时间轴 C 的实现（见 main.py 模块开头总纲）：
    · Preview      —— 显示线程 + 画布池；主循环 render() 投递即返回，show() 慢也只慢预览。
    · TouchButtons —— 触摸按下沿 → 按钮命中。触摸坐标是屏幕坐标，与画布 1:1。
    · display_result()            —— 把识别线程的最新快照翻译成"这一帧该画什么"（过期降级）。
    · apply_maixvision_stream_policy() —— 默认保留 MaixVision 会话；显式本地模式兼容。

★ clock 注入：构造可传 `clock`（默认标准库 time）。入口 main() 把自身 time 传进来，
  使 tools/test_main_self_test.py 的假时钟可穿透（同步显示路径在模拟时间轴下可复现）。
★ 隔离原则：显示层任何失败一律只打日志并置 Preview.enabled=False，绝不影响识别/串口/闭环。
"""
import os
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
    age = max(0.0, now - snapshot["captured_at"])
    if age > max_age:
        return {"valid": False, "stage": "stale"}, snapshot["laser"], age
    return snapshot["result"], snapshot["laser"], age


def apply_maixvision_stream_policy(cfg):
    """默认保留启动环境；明确选择本地模式才关闭会话（兼容旧入口别名）。

    Preview 自己把本地 Display 与图传适配器分开，创建 Display 时临时设置
    MAIXVISION=0 后立即恢复。图传进程使用本机 bridge 协议，不调用 native
    ImageTrans 的 500ms 忙等，也不 import maix 触发额外串口监听。
    """
    if not bool(getattr(cfg, "DISABLE_MAIXVISION_STREAM", False)):
        return False
    if os.environ.get("MAIXVISION") != "1":
        return False
    os.environ["MAIXVISION"] = "0"      # os.environ 赋值会同步 putenv，C 侧 getenv 随即生效
    print("[preview] 本地模式：已禁用 MaixVision 图传（MAIXVISION 1->0；终端日志照常）")
    return True


def _local_display_call(callback, *args):
    """Scope native Display creation/auto-reopen to local mode, then restore launch env."""
    session = os.environ.get("MAIXVISION")
    try:
        if session == "1":
            os.environ["MAIXVISION"] = "0"
        return callback(*args)
    finally:
        if session is None:
            os.environ.pop("MAIXVISION", None)
        else:
            os.environ["MAIXVISION"] = session


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
        self.display = None
        self.width = self.height = 0            # 物理屏幕尺寸（触摸坐标系）
        self.render_width = self.render_height = 0
        self.enabled = False                    # 显示是否可用；任何一步失败都置 False 并停用
        self.buttons = {}                       # name -> (x0,y0,x1,y1)，屏幕坐标
        self._button_draw = []                  # 预算好的按钮绘制参数（画布坐标），渲染时不再换算
        self._button_hot = {}                   # name -> 高亮截止时刻（秒；"刚被按下"的视觉反馈）
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
        self.stream = None
        self.stream_width = 0
        self._stream_state = None
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口/头部模式，画面不流转）")
            return
        try:
            from maix import display
            session = os.environ.get("MAIXVISION")
            streaming = session == "1" and not bool(getattr(cfg, "DISABLE_MAIXVISION_STREAM", False))
            # Native show() also sends to img_trans. Do not create that global transport
            # in this process: its C++ 500ms busy wait may hold Python's GIL.
            self.display = _local_display_call(display.Display)
            self.width = int(self.display.width())
            self.height = int(self.display.height())
            # 画布宽度：SHOW_WIDTH<=0 ⇒ 用屏幕原始宽度（画布＝屏幕 ⇒ show() 零缩放）。
            cap = int(getattr(cfg, "SHOW_WIDTH", 0) or 0)
            self.render_width = self.width if cap <= 0 else min(self.width, cap)
            self.render_height = max(1, int(round(self.height * self.render_width / self.width)))
            if streaming:
                try:
                    from .maixvision_stream import MaixVisionStream
                    self.stream_width = min(self.render_width, max(1, int(getattr(cfg, "SHOW_STREAM_WIDTH", 480))))
                    stream_height = max(1, int(round(self.render_height * self.stream_width / self.render_width)))
                    self.stream = MaixVisionStream(cfg, self.stream_width, stream_height)
                except Exception as exc:
                    print("[preview] 图传适配器不可用，保留本地显示: %s" % exc)
            if hasattr(display, "set_trans_image_quality"):
                display.set_trans_image_quality(int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)))
            self.enabled = True
            print("[preview] mode=%s screen=%dx%d canvas=%dx%d show_fps=%d jpeg=%d maixvision=%s"
                  % ("display-thread" if self.async_show else "sync-inprocess",
                     self.width, self.height, self.render_width, self.render_height,
                     int(getattr(cfg, "SHOW_FPS", 30)), int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)),
                     "on" if os.environ.get("MAIXVISION") == "1" else "off"))
            # 首帧在显示线程计量，不在构造/主循环执行同步 show 探针。
            if self.async_show:
                self.thread = threading.Thread(target=self._show_loop, name="display", daemon=True)
                self.thread.start()
        except Exception as exc:
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

    def hit(self, x, y):
        """触摸点 (x, y)（屏幕坐标）命中的按钮名；未命中返回 None。"""
        for name, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    def flash_button(self, name, now, hold=0.25):
        """把按钮标记为"刚被按下"，在 hold 秒内以高亮色重绘 —— 现场唯一能肉眼确认点击生效的反馈。

        为什么需要：触摸是"按下沿"语义，命中只在**一帧**发生，按钮外观若恒定不变，
        现场就无法区分"没按到 / 按到但没命中按钮 / 命中了但被拒绝 / 已成功触发"。
        """
        self._button_hot[name] = float(now) + float(hold)

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
        self._draw_buttons(canvas, status, now)              # 按钮与状态条画在画布之上
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
            _local_display_call(self.display.show, maix_image.cv2image(frame, bgr=True, copy=True))
            ms = (self._clock.perf_counter() - t0) * 1000.0
        except Exception as exc:
            print("[preview] show 探针失败（显示可能不可用）: %s" % exc)
            return -1.0
        if ms >= float(getattr(self.cfg, "SHOW_STREAM_SLOW_MS", 300)):
            print("[preview] ⚠ show 探针 %.0fms —— 命中 MaixCDK「MaixVision 图传未就绪」忙等分支："
                  "每次 show() 空转 500ms（maix_image_trans.cpp），约每秒烧掉半个 CPU 核，"
                  "是相机 read / 识别 / 闭环一起变慢的根因。" % ms)
            print("[preview] ⚠ 对策①：临时选择 Config.DISABLE_MAIXVISION_STREAM=True——"
                  "main() 会在建 Display 之前把 MAIXVISION 置 0，从源头不建图传，show() 回到 1~5ms。")
            print("[preview] ⚠ 对策②：或改从**设备本地**启动本应用（应用菜单 / 开机自启），"
                  "不要用 MaixVision 的“运行”按钮 —— 本地启动时 maixvision 模式本就关闭。")
            print("[preview] ⚠ 对策③：或让 MaixVision 的画面预览真正连上"
                  "（连上后图传走「压 JPEG 入队」的异步路径，不再忙等）。")
        else:
            print("[preview] show 探针 %.1fms —— 显示链路健康（本地屏典型 1~5ms）" % ms)
        return ms

    def warn_slow_once(self, p95):
        """显示侧变慢只提示一次（每 2s 窗口重复报会刷屏）；点明最可能的根因。"""
        if self._slow_warned:
            return
        self._slow_warned = True
        print("[preview] show slow: p95=%.0fms —— 瓶颈在显示侧（非识别）。"
              "若该值恒定 ≈505ms，即为 MaixCDK 的 MaixVision 图传忙等："
              "当前图传已隔离到独立进程；检查屏幕驱动或此前创建的 native Display。"
              "可临时选择 Config.DISABLE_MAIXVISION_STREAM=True 排查。" % p95)

    def _do_show(self, canvas, maix_image=None):
        """真正调用 display.show()，记耗时/帧率；失败即停用显示（不影响识别/闭环）。返回是否成功。"""
        if maix_image is None:
            from maix import image as maix_image
        t0 = self._clock.perf_counter()
        try:
            # bgr=True：cv2 是 BGR 序；copy=True：交给显示的必须是独立缓冲。
            # Only open()/auto-open creates native img_trans. Keep MAIXVISION
            # unchanged during normal shows and scope just the reopen operation.
            opened = getattr(self.display, "is_opened", None)
            if opened is not None:
                if not opened():
                    _local_display_call(self.display.open)
                    if not opened():
                        # Native open() returns an Err instead of raising. Do
                        # not let show() auto-open under the restored session.
                        raise RuntimeError("local display reopen failed")
                self.display.show(maix_image.cv2image(canvas, bgr=True, copy=True))
            else:
                # Older adapters without an open-state API retain local scope.
                _local_display_call(self.display.show,
                                    maix_image.cv2image(canvas, bgr=True, copy=True))
        except Exception as exc:
            self.enabled = False
            print("[preview] show 失败，停用显示: %s" % exc)
            return False
        ms = (self._clock.perf_counter() - t0) * 1000.0
        if self.probe_ms < 0:
            self.probe_ms = ms
            print("[preview] 本地 show 首帧 %.1fms；图传独立 latest-frame 进程" % ms)
        if self.stream is not None:
            try:
                height = max(1, int(round(self.render_height * self.stream_width / self.render_width)))
                frame = canvas if self.stream_width == self.render_width else cv2.resize(canvas, (self.stream_width, height))
                self.stream.submit(frame)
                state = self.stream.status()["state"]
                if state != self._stream_state:
                    self._stream_state = state
                    print("[preview] MaixVision stream=%s（本地显示/识别继续）" % state)
            except Exception as exc:
                print("[preview] 图传失败，保留本地显示: %s" % exc)
                try:
                    self.stream.close()
                except Exception as close_exc:
                    print("[preview] 图传清理失败: %s" % close_exc)
                finally:
                    self.stream = None
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

    def _draw_buttons(self, canvas, status="", now=None):
        # 按钮几何/文本宽度已在 add_button() 预算好，这里只画（不再每帧 getTextSize/换算）。
        # 刚被按下的按钮（flash_button 标记且未过期）改高亮色，作为点击生效的唯一视觉反馈。
        now = self._clock.perf_counter() if now is None else float(now)
        for (name, x0, y0, x1, y1, label, pos) in self._button_draw:
            hot = now < self._button_hot.get(name, -1e9)
            cv2.rectangle(canvas, (x0, y0), (x1, y1),
                          (0, 110, 235) if hot else (46, 150, 60), -1)   # 填充（BGR）
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
        if self.stream is not None:
            self.stream.close()
        if self.thread is not None and self.thread.is_alive():
            # Display is still owned by its show thread. Closing it here races
            # native show(); only transport cleanup is independent.
            raise RuntimeError("display thread did not stop; display remains owned by worker")
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

    ★ 为什么"只有按下的一瞬间有反应"是**预期行为**：按下沿语义下，只有 `pressed` 由
      False→True 的那一帧返回按钮名，其余帧（含按住不放、松开）一律返回 None。这是为了
      防止"按住不放反复启动自检/搜索"。它**不是**断链，所以本节必须给出可观测反馈，
      否则现场无法区分"没按到 / 按到但没命中 / 命中了但被拒绝 / 已成功触发"。
    ★ 为此本类补齐三件事：① 命中/未命中都打印一行 `[touch]`；② `read()` 异常时复位按下
      锁存；③ 锁存超过 `MAX_HOLD_S` 视为"松开帧被漏采样"，自愈复位 —— 否则一旦某次释放
      未被采到，`_down` 会永久停在 True，此后**所有新点击都被当成按住不放而永久失效**。
    """

    MAX_HOLD_S = 2.0        # 按下锁存上限：超过即认为松开帧被漏采样，解除锁存（本次按住不再触发）

    def __init__(self, cfg):
        self.ts = None
        self._down = False                 # 上一帧按下状态，用于识别"按下沿"
        self._down_at = -1e9               # 本次按下沿的时刻（判定锁存是否超时）
        self._suppress = False             # 锁存超时解锁后，等待真正松手才接受新的按下沿
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
            if self._down:
                # read 失败期间无法判定松手：复位锁存，避免按钮从此永久失灵。
                print("[touch] read 异常，复位按下锁存: %s" % exc)
            self._down = self._suppress = False
            return None
        now = time.perf_counter()
        if pressed:
            if self._suppress:             # 本次按住已判为超时锁存，等松手
                return None
            if self._down:
                if now - self._down_at > self.MAX_HOLD_S:
                    # 松开帧疑似被漏采样（主循环节拍 > 点按时长）：解除锁存，但要等真正松手
                    # 才接受新的按下沿，否则会"每 MAX_HOLD_S 重复触发一次"。
                    print("[touch] 按下锁存超时 %.1fs，已解除（等松手后重新生效）"
                          % (now - self._down_at))
                    self._down, self._suppress = False, True
                return None                # 按住不放：不重复返回，避免反复触发
            self._down, self._down_at = True, now
            name = preview.hit(int(x), int(y))
            if name is None:
                # 未命中：明确打日志，否则现场只看到"按了没反应"而无法判断是点偏了。
                print("[touch] 点击未命中按钮 @(%d,%d)" % (int(x), int(y)))
            else:
                print("[touch] hit=%s @(%d,%d)" % (name, int(x), int(y)))
            return name
        self._down = self._suppress = False   # 松开，允许下一次按下沿
        return None
'''

# ===== vision.rect_geometry  (vision/rect_geometry.py) =====
_MOD_SRC['vision.rect_geometry'] = r'''
"""纯几何工具：四边形规整、单应解算、批量采样、正视面反投影。

本模块**不持有任何状态、不读 config**，只做可独立验证的数学与 OpenCV 调用，
供 `contour_locator`（候选提议）与 `rectangle_center`（验证/精修/中心解算）共用。

为什么单独拆出来
--------------------------------------------------------------------------------
靶心 = 黑框矩形两条对角线的交点（射影不变量）。整条链路可以拆成三段互不依赖的数学：

    ① 图像四边形 → 物理矩形 (mm) 的 4 点单应 H        `homography`
    ② 物理矩形中心 (w/2, h/2) 经 H 投影回图像 = 靶心   `project_points`
    ③ 反向：图像点 → 物理坐标（诊断/交叉校验用）        `inverse_map_points`

把这三段与"批量采样""四边形体检"一起放在这里，检测器代码就只剩"状态机 + 门限"，
换候选生成方案（阈值掩膜 → Canny 轮廓）时不必碰数学。
"""
import cv2
import numpy as np


# --------------------------------------------------------------------- 四边形
def polygon_area(p):
    """有符号面积的 2 倍（>0 = CCW，<0 = CW）。用于把四角规整成统一方向。

    直接展开 shoelace 公式（而不是 `cv2.contourArea(oriented=True)` + 重塑视图）：
    本函数在每帧的每个候选上都要跑，省掉一次数组重塑与一次 C 调用的开销是可测的。
    """
    p = np.asarray(p, np.float64).reshape(4, 2)
    x, y = p[:, 0], p[:, 1]
    return float(x[0] * (y[1] - y[3]) + x[1] * (y[2] - y[0]) +
                 x[2] * (y[3] - y[1]) + x[3] * (y[0] - y[2]))


def order_quad(points):
    """把 4 个点规整为统一顺序：**[左上, 右上, 右下, 左下]**（CCW）。

    以质心为极点按极角排序 → 保证 CCW → 旋转到"x+y 最小"（左上角）打头。
    统一顺序是后续所有几何运算（边向量、对角线、单应四点对应）的前提。
    用切片拼接代替 `np.roll`：同结果、少一次完整数组分配（每个候选都要调）。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]   # 按极角排序
    if polygon_area(p) < 0:
        p = p[::-1]                                                  # 反了则翻成 CCW
    shift = int(np.argmin(p[:, 0] + p[:, 1]))
    if shift:
        p = np.concatenate((p[shift:], p[:shift]))                   # 左上角打头
    return p.copy()


def side_lengths(q):
    """四边长度 [边0, 边1, 边2, 边3]，边 i = 点 i → 点 i+1。"""
    q = np.asarray(q, np.float64)
    x, y = q[:, 0], q[:, 1]
    nx = np.concatenate((x[1:], x[:1]))
    ny = np.concatenate((y[1:], y[:1]))
    return np.hypot(nx - x, ny - y)


def quad_is_valid(q, shape, margin=1, min_edge=12.0, min_sine=0.20):
    """四边形体检：不越界、凸、边长下限、无明显退化尖角。

    允许任意透视（不要求接近直角/矩形），只排除两类坏形状：
      · 贴边/出界：角点落在图像边缘 margin 内 → 采样会越界、中心不可信；
      · 退化尖角：相邻边几乎平行（sin 很小）→ 四角拟合不稳定。

    `min_edge` 是**像素**下限，调用方在缩小图上用时按比例传小值。
    同样只用切片拼接，不用 np.roll（每个候选两次调用，是定位器的热点之一）。
    """
    h, w = shape
    q = np.asarray(q, np.float64)
    x, y = q[:, 0], q[:, 1]
    if (x.min() < margin or y.min() < margin or
            x.max() >= w - margin or y.max() >= h - margin):
        return False
    if not cv2.isContourConvex(q.astype(np.float32).reshape(-1, 1, 2)):
        return False
    dx = np.concatenate((x[1:] - x[:3], (x[0] - x[3],)))
    dy = np.concatenate((y[1:] - y[:3], (y[0] - y[3],)))
    lengths = np.hypot(dx, dy)
    if lengths.min() < min_edge:
        return False
    ex = np.concatenate((dx[1:], dx[:1]))
    ey = np.concatenate((dy[1:], dy[:1]))
    sine = np.abs(dx * ey - dy * ex) / (lengths * np.concatenate((lengths[1:], lengths[:1])))
    return bool(sine.min() > min_sine)


def quad_from_contour(cnt, epsilon_ratio, hull_fill_ratio, factors=(1.0, 1.6, 2.4, 3.5, 5.0)):
    """从一条轮廓提取凸四边形；**容忍缺口/粘连造成的非四边形状**。

    两级容忍（实机必需，见 tools/test_real_scene.py 的背景说明）：
      ① epsilon 由小到大**扫描**原轮廓 —— 修复"带小缺口的多边形降到 4 点"；
      ② 仍不成四边形、且该轮廓**本身就像环形带**时，对**凸包**同样扫描 —— 缺口被
         凸包补上，得到近似矩形。凸包只对"中空程度高"的轮廓启用
         （轮廓面积 < hull_fill_ratio × 凸包面积），实心背景/阴影块不参与，
         否则会凭空多出大片假候选、白耗候选预算。
    凸包在这里**只作候选提议**：真伪由调用方的四边暗带验证判定。
    """
    hull = cv2.convexHull(cnt)
    hull_area = abs(cv2.contourArea(hull))
    band_like = (hull_area > 0 and
                 abs(cv2.contourArea(cnt)) < hull_fill_ratio * hull_area)
    for base in ((cnt, hull) if band_like else (cnt,)):
        peri = cv2.arcLength(base, True)
        if peri <= 0:
            continue
        for factor in factors:
            approx = cv2.approxPolyDP(base, epsilon_ratio * factor * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                return order_quad(approx)
            if len(approx) < 4:
                break                                            # 已过度简化
    return None


def line_through(a, b):
    """过 a、b 两点的齐次直线系数 (a_, b_, c_)：a_·x + b_·y + c_ = 0。

    与 `cv2.fitLine` 的输出口径一致，用于某条边没有足够边缘证据时沿用输入边。
    """
    (x0, y0), (x1, y1) = np.asarray(a, np.float64), np.asarray(b, np.float64)
    dx, dy = x1 - x0, y1 - y0
    return np.array([-dy, dx, dy * x0 - dx * y0], np.float64)


# --------------------------------------------------------------------- 单应
def gaussian_solve(a, b):
    """带部分主元的高斯消元；消去行向量化，避免 64 次 Python 行循环。

    主元过小（矩阵退化，如四点共线）抛 ValueError —— 不做伪解。
    """
    m = np.column_stack((np.asarray(a, np.float64), np.asarray(b, np.float64)))
    for col in range(len(m)):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("degenerate quadrilateral")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]
        m[col] /= m[col, col]
        factors = m[:, col].copy()
        factors[col] = 0.0
        m -= factors[:, None] * m[col]
    return m[:, -1]


def homography(src, dst):
    """解 4 点对应的 3×3 单应 H（使 H·src ≡ dst）。

    先对 src/dst **归一化**（平移质心到原点、缩放到量级 1）再建 8×8 线性方程，
    求得 H_n 后用 T_dst·H_n·T_src⁻¹ 还原 —— 归一化显著提升大像素坐标下的数值稳定性。
    """
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))
    a[1::2, 3:6] = a[0::2, :3]
    a[0::2, 6:] = -uv[:, :1] * xy
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    """把物理矩形 (0,0)-(w,h) 映射到图像四边形 src_quad 的单应（mm → px）。"""
    return homography(src_quad, [[0, 0], [width_mm, 0],
                                 [width_mm, height_mm], [0, height_mm]])


def project_points(matrix, points):
    """齐次投影：把 (N,2) 点经 3×3 矩阵映射，做透视除法得 (N,2)。落无穷远点抛错。"""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    """单应逆映射（px → mm）；不做越界检查，由调用方负责。"""
    return project_points(np.linalg.inv(h), points)


# ------------------------------------------------------- 正视面（warp）辅助
def rectangle_corners(w, h, inset=0.0):
    """物理矩形四角（mm），inset>0 时向内缩进（取胶带内沿 = 内框）。顺序 LL→LR→UR→UL。"""
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    """正视面坐标下、以矩形中心为圆心、半径 radius_mm 的等分圆采样点（(samples,2)）。"""
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def front_view_size(width_mm, height_mm, short_side_px):
    """正视面目标分辨率：按物理短边固定像素数等比缩放，保证长宽比与靶纸一致。"""
    if width_mm <= 0 or height_mm <= 0 or short_side_px <= 0:
        raise ValueError("front view size needs positive dimensions")
    scale = float(short_side_px) / min(width_mm, height_mm)
    return max(8, int(round(width_mm * scale))), max(8, int(round(height_mm * scale)))


def warp_front_view(gray, rect_to_image, width_mm, height_mm, short_side_px):
    """把图像中的靶纸四边形**透视变换成正视图**（正视 = 矩形，无透视畸变）。

    矩阵口径（实测确认，别凭记忆改）：``cv2.warpPerspective`` 接受的矩阵是
    **源坐标 → 目标坐标**，即"图像像素 → 正视图像素"：

        M = mm_to_front · inv(rect_to_image)

    自检方法（`tools/test_real_scene.py` 有对应用例）：把 (0,0) 与 (w,h) 经该矩阵映射，
    应当正好落在图像里靶纸矩形的左上/右下角；若方向反了，得到的图会整片落在靶纸之外
    （表现为"正视图里只看得到背景"），而肉眼不一定立刻发现。

    正视图上一个像素对应固定的物理尺寸（短边 = `short_side_px` 像素），因此靶心恒在
    正视图中心，可直接用于可视化与"剖面法取中心"的独立校验。
    返回 (front_view, mm_to_front)；后者把物理 mm 映射到正视图像素。
    """
    fw, fh = front_view_size(width_mm, height_mm, short_side_px)
    mm_to_front = np.array([[fw / float(width_mm), 0, 0],
                            [0, fh / float(height_mm), 0],
                            [0, 0, 1]], np.float64)
    image_to_front = mm_to_front @ np.linalg.inv(np.asarray(rect_to_image, np.float64))
    view = cv2.warpPerspective(gray, image_to_front, (fw, fh), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REPLICATE)
    return view, mm_to_front


# --------------------------------------------------------------------- 采样
def sample_batch(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。

    相比逐点 Python 循环，remap 一次调用即可采样成百上千个点（四边验证/精修的关键提速）。
    borderMode=REPLICATE 保证边缘点也能返回数值（越界判据在调用方单独做）。
    """
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)
'''

# ===== vision.contour_locator  (vision/contour_locator.py) =====
_MOD_SRC['vision.contour_locator'] = r'''
"""候选定位器：**Canny → findContours → 四边形筛选**，只在低分辨率上跑。

职责边界（与 `rectangle_center` 的分工）
--------------------------------------------------------------------------------
本模块只回答一个问题："画面里哪些地方**可能是**靶纸黑框的四条边？"
它**不判真伪**：输出的每个候选都必须经 `rectangle_center._verify` 的四边暗带 + 内侧亮区
校验才算数。这样"提议"与"裁决"解耦，换提议策略不会动摇精度与误检防线。

两级提议（掩膜数量受控）
--------------------------------------------------------------------------------
    level 0 `edge`  : `Canny` 边缘图（**1 张掩膜**）→ 轮廓 → 四边形。
    level 1 `mask`  : 仅当 level 0 一个都没验过时，才补一张 Otsu 暗区掩膜
                      （用轮廓树层级直接定内/外框，省掉"两种假设都试"的开销）。

旧实现是"Otsu + 8 档 `DARK_THRESHOLDS` 逐档阈值化 × 每档 findContours"共 9~10 张掩膜，
板端实测单帧 ~312ms（detect_fps=3.2，见 板端识别问题整理_2026-10-13.md）。
改成"1 张（必要时 2 张）"后，最贵的那部分（阈值化 + 形态学 + 轮廓树，都在低分辨率上）
直接降了一个数量级。

为什么 Canny 边缘图的轮廓**能**近似出黑框矩形（而不是一堆碎线段）
--------------------------------------------------------------------------------
Canny 输出的是 1px 细线：胶带带的外沿与内沿各是一条**闭合细环**，环的轮廓正好包住整个
矩形内部，`approxPolyDP` 一把就降到 4 点。即使胶带环在角落断开（实机常态），带的外沿/
内沿/两端缺口会共同围成一个**闭合的细带**，其轮廓"中空程度"高（面积 ≪ 凸包面积），
于是 `quad_from_contour` 的凸包兜底被启用，缺口被补上 → 仍能得到完整矩形。
真伪始终由 `_verify` 裁决，所以"凸包把阴影拼成矩形"这类情况不会变成误检。

坐标口径
--------------------------------------------------------------------------------
搜索在缩小图上做（`SEARCH_WIDTH`），**返回的四边形已换算回原图坐标**（含 ROI 偏移），
调用方拿到的就是可直接 `_verify` / `_refine` 的原图坐标。
"""
import time

import cv2
import numpy as np

from .rect_geometry import quad_from_contour, quad_is_valid

# 提议级别：0 = Canny 边缘（默认，1 张掩膜）；1 = Otsu 暗区掩膜（兜底，仅第一级全失败时）
LEVEL_EDGE, LEVEL_MASK = 0, 1
LEVELS = (LEVEL_EDGE, LEVEL_MASK)


class ContourLocator:
    """低分辨率候选提议器（无跨帧状态；每帧独立、可重复调用）。"""

    def __init__(self, config):
        self.cfg = config
        close_px = int(getattr(config, "EDGE_CLOSE_PX", 3))
        self._kernel = None if close_px <= 1 else cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_px, close_px))
        self.last_stats = dict(masks=0, contours=0, quads=0, ms=0.0, level=None)

    # ------------------------------------------------------------------ 对外
    def locate(self, gray, roi=None, level=LEVEL_EDGE):
        """在 `roi`（默认全图）内提议候选四边形。

        返回 ``[(quad(float32 (4,2), 原图坐标), kind), ...]``，**已按优先级排序**：
        同一个四边形的 "inner" 假设排在 "outer" 之前（亮面孔比与背景粘连的黑环更稳定）。
        """
        started = time.perf_counter()
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        # 搜索恒在低分辨率：宽超过 SEARCH_WIDTH 就等比缩小（ROI 搜索时 w 小 → 不缩）。
        scale = min(1.0, float(self.cfg.SEARCH_WIDTH) / float(w))
        sw = max(32, int(round(w * scale)))
        sh = max(24, int(round(h * scale)))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        sxy = np.array([w / float(sw), h / float(sh)], np.float32)
        offset = np.array([x, y], np.float32)

        masks, contours_n = self._masks(small, level)
        quads, seen = [], []
        for mask in masks:
            for q_small in self._quads_from_mask(mask, small.shape):
                q = q_small * sxy + offset
                if not quad_is_valid(q, gray.shape,
                                     min_edge=float(getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.0))):
                    continue
                key = np.rint(q).astype(np.int32).tolist()
                if key in seen:                       # 同一个框被多条轮廓重复提及时只留一次
                    continue
                seen.append(key)
                quads.append(q)
                if len(quads) >= int(getattr(self.cfg, "MAX_CANDIDATES", 8)):
                    break
            if len(quads) >= int(getattr(self.cfg, "MAX_CANDIDATES", 8)):
                break
        proposals = []
        for q in quads:
            for kind in ("inner", "outer"):
                proposals.append((q, kind))
        self.last_stats = dict(masks=len(masks), contours=contours_n, quads=len(quads),
                               ms=(time.perf_counter() - started) * 1000.0, level=level)
        return proposals

    # ------------------------------------------------------------------ 掩膜
    def _masks(self, small, level):
        """生成该级别需要的掩膜（edge 级 1 张；mask 级 1 张）。返回 (掩膜列表, 轮廓数)。"""
        if level == LEVEL_EDGE:
            edges = cv2.Canny(small, self.cfg.CANNY_LOW, self.cfg.CANNY_HIGH)
            if self._kernel is not None:
                # 3px 闭运算把 Canny 常见的一两个像素断裂接上，避免轮廓碎成多段；
                # 这一步只在低分辨率图（≤320 宽）上做，代价可忽略。
                edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, self._kernel)
            return [edges], -1
        _, mask = cv2.threshold(small, 0, 255,
                                cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)   # 暗区为前景
        if self._kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        return [mask], -1

    # ------------------------------------------------------------------ 轮廓筛选
    def _quads_from_mask(self, mask, shape):
        """轮廓 → 四边形，按"像不像一个够大的框"廉价预筛后交给 `quad_from_contour`。

        ★ 命中足够多的四边形就**立刻停止扫轮廓**：`quad_from_contour` 内部要做
          epsilon 扫描（最多 5 次 `approxPolyDP`，环形带还要再来一遍凸包），单次不便宜；
          而调用方只会用前 `MAX_CANDIDATES` 个候选。不限次数的版本在"边缘密"的画面上
          会把预算全花在后面的碎轮廓上（实测是搜索路径的头号开销来源）。
        """
        found = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours = found[-2]
        if not contours:
            return []
        h, w = shape
        area_frame = float(w * h)
        limit_scan = int(getattr(self.cfg, "SEARCH_MAX_CONTOURS", 16))
        want = int(getattr(self.cfg, "MAX_CANDIDATES", 8))
        ids = sorted(range(len(contours)), key=lambda i: abs(cv2.contourArea(contours[i])),
                     reverse=True)[:limit_scan]
        out, min_ratio = [], float(self.cfg.MIN_AREA_RATIO)
        max_ratio = float(self.cfg.MAX_AREA_RATIO)
        min_side = max(8, int(0.04 * min(w, h)))
        for i in ids:
            if len(out) >= want:
                break
            cnt = contours[i]
            bx, by, bw, bh = cv2.boundingRect(cnt)
            if min(bw, bh) < min_side:
                continue                                        # 又小又扁：不是框
            area = abs(cv2.contourArea(cnt))
            # 边缘图的细环轮廓"面积"就是环内空腔面积，与实心掩膜的框面积口径一致。
            if not area_frame * min_ratio <= area <= area_frame * max_ratio:
                continue
            q = quad_from_contour(cnt, self.cfg.APPROX_EPSILON, self.cfg.HULL_FILL_RATIO)
            if q is not None:
                out.append(q)
        return out
'''

# ===== vision.kalman_filter  (vision/kalman_filter.py) =====
_MOD_SRC['vision.kalman_filter'] = r'''
"""靶心中心的卡尔曼滤波（匀加速噪声驱动的匀速模型，2D 位置 + 2D 速度）。

为什么要它（而不是继续用一阶 EMA）
--------------------------------------------------------------------------------
现场症状是"车一晃中心就偏、画面抖". 抖动有两个来源，必须区别对待：

  · **必须跟踪的真实运动** —— 底盘晃动/云台转动让靶心在图像里快速位移；
  · **必须抑制的测量噪声** —— 阈值化/边缘跳变给出的逐帧随机误差、反光造成的抖动。

一阶 EMA（`CENTER_ALPHA`）对两者一视同仁：想抑噪就得把 alpha 调小，代价是快速运动
时输出明显滞后（实测附加 ~28ms 滞后，底盘一动就变成瞄准误差）。卡尔曼用**运动模型**
把二者分开：噪声进不了速度状态，而匀速运动能被模型零滞后地跟上。

模型
--------------------------------------------------------------------------------
状态 x = [u, v, vu, vv]（位置 px、速度 px/s）；观测 z = [u, v]。
F(dt) 为匀速外推；Q 取"连续白噪声加速度"离散化形式（a ~ q），
即 Q = q² · [[dt⁴/4, dt³/2], [dt³/2, dt²]] ⊗ I₂；R = σ²I₂。
  · 过程噪声 `q`（px/s²）越大 → 越信任观测、越跟手、抑噪越弱；
  · 观测噪声 `σ`（px）越大 → 越信任模型、越平滑、滞后越大。
两者都可由 config 现场调；默认值按"板端 ~30fps、抖动幅度 1~3px"取。

鲁棒性（工程必需的几条硬规则，缺一条现场就会翻车）
--------------------------------------------------------------------------------
  1. **时钟不连续**（回退 / 间隔过长 / 首帧）→ 直接 reset 并把状态播种为本次观测，
     于是滤波输出 == 原始测量，绝不拿旧速度外推（`PREDICT_MAX_GAP_S` 口径）；
  2. **观测量跳变过大**（> reset_px）→ 判为新目标/重捕获，同样 reset 播种，
     避免滤波器用旧速度"拖"住一个已经跳走的目标；
  3. **有限性断言**：NaN/Inf 一律拒绝，绝不污染状态（板端整型/除零泄漏的教训）。
"""

import numpy as np

# 状态向量的下标语义（供外部读速度用）
U, V, VU, VV = 0, 1, 2, 3


class ConstantVelocityKalman2D:
    """2D 位置 + 速度的卡尔曼滤波；`update(z, t)` 返回后验位置，`predict_at(t)` 外推。"""

    def __init__(self, process_px=260.0, measure_px=0.9, reset_px=25.0,
                 max_predict_s=0.25, velocity_limit_px_s=6000.0):
        self.process_px = float(process_px)
        self.measure_px = float(measure_px)
        self.reset_px = float(reset_px)
        self.max_predict_s = float(max_predict_s)
        self.velocity_limit_px_s = float(velocity_limit_px_s)
        self.x = None                    # (4,) 状态；None = 尚未初始化
        self.p = None                    # (4,4) 协方差
        self.t = None                    # 上次更新时间（单调秒）
        self.resets = 0

    # ------------------------------------------------------------------ 生命周期
    def reset(self):
        """清空状态；下次 update 会重新播种。"""
        self.x = self.p = self.t = None
        return self

    @property
    def ready(self):
        return self.x is not None

    def seed(self, z, t):
        """用一次观测**播种**（不滤波）：后验 = 观测、速度未知（大协方差）。"""
        z = np.asarray(z, np.float64).reshape(2)
        if not np.all(np.isfinite(z)):
            return None
        self.x = np.array([z[0], z[1], 0.0, 0.0], np.float64)
        # 位置方差取观测噪声，速度方差取"一个观测噪声 / 一帧"的保守量级。
        self.p = np.diag([self.measure_px ** 2, self.measure_px ** 2,
                          (self.measure_px * 30.0) ** 2, (self.measure_px * 30.0) ** 2])
        self.t = float(t) if t is not None else None
        return self.x[:2].copy()

    # ------------------------------------------------------------------ 预测
    def _transition(self, dt):
        return np.array([[1.0, 0.0, dt, 0.0],
                         [0.0, 1.0, 0.0, dt],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]], np.float64)

    def _process_noise(self, dt):
        q2 = self.process_px ** 2
        g = np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0],
                      [dt ** 3 / 2.0, dt ** 2]], np.float64) * q2
        qm = np.zeros((4, 4), np.float64)
        qm[np.ix_([U, V], [U, V])] = g
        qm[np.ix_([VU, VV], [VU, VV])] = g
        return qm

    def predict(self, t):
        """按 dt = t − 上次更新外推状态；返回预测位置 (2,)。未初始化时返回 None。"""
        if self.x is None:
            return None
        if t is None:
            return self.x[:2].copy()
        if self.t is None:
            self.t = float(t)
            return self.x[:2].copy()
        dt = float(t) - self.t
        if dt <= 0.0 or dt > self.max_predict_s:
            return None                                   # 时钟回退/断流：不可外推
        f = self._transition(dt)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + self._process_noise(dt)
        self.t = float(t)
        self._clamp_velocity()
        return self.x[:2].copy()

    def predict_at(self, t):
        """**不改状态**地外推到时刻 t（用于"取上一帧模型预测本帧位置"这类只读场景）。"""
        if self.x is None or self.t is None or t is None:
            return None
        dt = float(t) - self.t
        if not 0.0 < dt <= self.max_predict_s:
            return None
        return (self.x[:2] + self.x[2:] * dt).copy()

    # ------------------------------------------------------------------ 更新
    def update(self, z, t, gate_px=None):
        """融合一次观测，返回后验位置 (2,)。

        · 未初始化 / 时钟不连续 / 观测跳变超限 → **reset 播种**（输出 == 观测）；
        · 观测非有限 → 返回 None（保留旧状态，调用方自行决定是否判定丢帧）。
        """
        z = np.asarray(z, np.float64).reshape(2)
        if not np.all(np.isfinite(z)):
            return None
        gate = self.reset_px if gate_px is None else float(gate_px)
        if self.x is None or self.t is None:
            self.resets += 1
            return self.seed(z, t)
        dt = float(t) - self.t if t is not None else None
        if dt is None or dt <= 0.0 or dt > self.max_predict_s:
            self.resets += 1
            return self.seed(z, t)
        if gate > 0 and float(np.linalg.norm(z - self.x[:2])) > gate:
            self.resets += 1                              # 目标跳走：重建而不是硬拖
            return self.seed(z, t)
        f = self._transition(dt)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + self._process_noise(dt)
        self._clamp_velocity()
        h = np.zeros((2, 4), np.float64)
        h[0, U] = h[1, V] = 1.0
        r = np.eye(2, dtype=np.float64) * self.measure_px ** 2
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ (z - h @ self.x)
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        self.p = 0.5 * (self.p + self.p.T)                # 数值对称化，防长期漂移
        self.t = float(t)
        self._clamp_velocity()
        return self.x[:2].copy()

    # ------------------------------------------------------------------ 读取
    def position(self):
        return None if self.x is None else self.x[:2].copy()

    def velocity(self):
        return None if self.x is None else self.x[2:].copy()

    def _clamp_velocity(self):
        limit = self.velocity_limit_px_s
        if limit <= 0:
            return
        speed = float(np.hypot(self.x[VU], self.x[VV]))
        if speed > limit:
            self.x[VU] *= limit / speed
            self.x[VV] *= limit / speed
'''

# ===== vision.rectangle_center  (vision/rectangle_center.py) =====
_MOD_SRC['vision.rectangle_center'] = r'''
"""靶心识别检测器：**Canny 轮廓定位 → 四边暗带验证 → 亚像素精修 → 单应解中心**。

================================================================================
几何原理（为什么靶心能"算"出来）
================================================================================
靶纸四周贴 1.8cm 黑胶带形成一个矩形环。相机斜看时该矩形在图像里是透视投影，
但**两条对角线的交点**在任何透视变换下都映射到对应四边形的对角线交点 —— 这是
**射影不变量**。于是：

    检测四边形四角 → 解 mm↔px 单应 → 把物理矩形中心 (w/2, h/2) 投影回图像 = 靶心 (u,v)

换一种说法就是题目要求的四步，二者**数学上完全等价**：

    四边形 → 透视变换成正视图 → 取正视图中心 → 逆透视反投影回原图

本模块把后一种说法也**真的做了一遍**（`warp_front_view` + `_front_view_center`），
得到一个与解析解**互相独立**的中心估计，用于交叉校验（两者不一致时置
`front_disagree=True`，提示几何/标定异常）。所以"正视图"不是装饰，是第二只眼睛。

================================================================================
单帧处理链
================================================================================
    ① 候选提议  `ContourLocator`  —— Canny → findContours → 四边形筛选（**低分辨率**）
    ② 四边验证  `_verify`         —— 从四边向里采"胶带暗带 + 内侧亮面"，覆盖率为真伪判据
    ③ 亚像素精修 `_refine`        —— 当前帧四条灰度窄带 → 有向跳变 → fitLine → 四角交点
    ④ 中心解算  `_center`         —— 单应把物理矩形中心投回原图（+ 正视图剖面交叉校验）
    ⑤ 时序滤波  `ConstantVelocityKalman2D` —— 抑制抖动、给下一帧提供速度预测

四级进入方式（越靠前越省算力；`process` 按上一帧是否锁定依次降级）：
    track       已锁定 → 只沿四边窄带精修（不跑 Canny/轮廓/掩膜，稳态 ~1ms）
    roi_search  刚失锁 → 在上一帧外框外扩 ROI_PADDING 的窗口内定位一次
    full_search 彻底丢失 → 全图低分辨率定位（1 张掩膜，必要时第 2 张）

掩膜纪律（v0.8 重构的核心）
--------------------------------------------------------------------------------
旧实现每帧要建 `Otsu + 8 档 DARK_THRESHOLDS` 共 9~10 张掩膜，每张都做
`threshold + morphologyEx + findContours`，再对每个候选做全分辨率四边采样 ——
板端实测单帧 ~312ms、detect_fps=3.2、结果 age>200ms 全被判 stale，闭环被锁死
（见 `板端识别问题整理_2026-10-13.md`）。
现在改为 **Canny 一级（1 张掩膜）+ Otsu 兜底一级（仅当第一级没找到四边齐备的候选）**，
且阈值化/形态学/轮廓树都在 `SEARCH_WIDTH` 的低分辨率图上完成，只有"验证/精修/解中心"
才回到原始分辨率 —— 这正是"减少掩膜数量、控制该项运行时长"的落点。
"""
import time

import cv2
import numpy as np

from .contour_locator import LEVELS, ContourLocator
from .kalman_filter import ConstantVelocityKalman2D
from .rect_geometry import (front_view_size, homography, homography_8x8,
                            inverse_map_points, line_through, order_quad, polygon_area,
                            project_points, quad_from_contour, quad_is_valid,
                            rectangle_corners, sample_batch, side_lengths, standard_circle,
                            warp_front_view)

# ---- 兼容别名（旧名字在 tools/diagnose_detect.py 等外部工具里被直接 import）--------
_polygon_area = polygon_area
_order_quad = order_quad
_side_lengths = side_lengths
_homography = homography
_rectangle = rectangle_corners
_sample = sample_batch
_line_through = line_through

__all__ = ["RectangleCenterDetector", "draw_result", "homography_8x8", "project_points",
           "inverse_map_points", "standard_circle", "quad_from_contour", "quad_is_valid",
           "_homography", "_rectangle", "_sample", "_polygon_area", "_order_quad",
           "_side_lengths", "_line_through"]


def _midline_mm(profile, extent_mm):
    """一维剖面里"两侧暗带（胶带）内沿之间的中线"，单位 mm；不可信时返回 None。

    先按分位数取暗/亮两档再取中值作阈值（避免个别反光或污点把 min/max 拉飞），
    然后**只在前/后 1/3 内**找暗带内沿（胶带恒在靶纸边缘，中段出现暗点属于印刷/阴影），
    最后对穿越点做线性插值得到亚像素位置。返回 None 表示"这个剖面没有可用的胶带暗带"。
    """
    prof = np.asarray(profile, np.float64).ravel()
    if prof.size < 12:
        return None
    dark, bright = float(np.percentile(prof, 8)), float(np.percentile(prof, 92))
    if bright - dark < 8.0:                      # 对比度太低：没有可见的暗带
        return None
    threshold = 0.5 * (dark + bright)
    scale = float(extent_mm) / prof.size          # mm/px；正视图一个像素对应多少 mm

    def inner_edge(forward):
        third = max(2, prof.size // 3)
        segment = prof[:third] if forward else prof[-third:]
        below = np.nonzero(segment <= threshold)[0]
        if below.size == 0:
            return None
        index = int(below[-1]) if forward else int(below[0]) + (prof.size - third)
        neighbour = index + 1 if forward else index - 1
        if not 0 <= neighbour < prof.size:
            return None
        step = prof[neighbour] - prof[index]
        if abs(step) < 1e-9:
            return float(index)
        # 从低于阈值到高于阈值的线性插值穿越点（亚像素）
        return float(index) + (threshold - prof[index]) / step

    left, right = inner_edge(True), inner_edge(False)
    if left is None or right is None or right <= left:
        return None
    return ((left + right) * 0.5 + 0.5) * scale    # +0.5：像素 i 的中心在 i+0.5 处


class RectangleCenterDetector:
    """有状态检测器：跨帧记住锁定框(_track)、中心滤波器与运动历史。

    状态字段：
        _track       上一帧验证通过的外框（dict 候选）；None = 未锁定
        _prev_quad   再上一帧的候选四角（运动预测用，仅同 kind 连续帧才建立）
        _lost        连续未检出帧数（>0 = 刚失锁，禁用预测，避免用旧速度外推）
        _kalman      中心卡尔曼滤波器（CV 模型，抑抖 + 提供速度预测）
        _smooth      上一帧输出中心（KALMAN_ENABLE=False 时退化为 EMA 口径）
        _locked_th   不再使用（保留字段以免外部代码 AttributeError）；见 README 迁移说明
        _roi_retry   失锁后是否还允许一次"ROI 搜索"（用一次即置 False，防反复局部搜）
    每帧重置的临时量：_search_count / _mask_count / _proposals。
    """

    def __init__(self, config):
        self.cfg = config
        self.last = None
        self._smooth = None
        self._track = None
        self._prev_quad = None                                 # 上上帧的候选四角（运动预测用）
        self._track_timestamp = self._prev_timestamp = None
        self._frame_shape = None
        self._lost = 0                                         # 连续未检出帧数（>0 时不预测）
        self._locked_th = None
        self._frame = 0
        self._locator = ContourLocator(config)
        # 四边上均匀分布的参数 t ∈ (0.12, 0.88)：避开四角（角点附近因透视最易模糊）。
        self._t = np.linspace(0.12, 0.88, config.EDGE_SAMPLES, dtype=np.float32)
        self._circle_cache = {}                                # 正视面圆周采样点缓存（按尺存）
        self._search_count = 0
        self._mask_count = 0
        self._proposals = 0
        self._roi_retry = True
        self._kalman = self._make_kalman(config)

    @staticmethod
    def _make_kalman(config):
        if not bool(getattr(config, "KALMAN_ENABLE", True)):
            return None
        return ConstantVelocityKalman2D(
            process_px=float(getattr(config, "KALMAN_PROCESS_PX", 260.0)),
            measure_px=float(getattr(config, "KALMAN_MEASURE_PX", 0.9)),
            reset_px=float(getattr(config, "KALMAN_RESET_PX",
                                   getattr(config, "FILTER_RESET_PX", 25.0))),
            max_predict_s=float(getattr(config, "KALMAN_MAX_PREDICT_S", 0.25)),
            velocity_limit_px_s=float(getattr(config, "KALMAN_VELOCITY_LIMIT_PX_S", 6000.0)))

    def reset(self):
        """清空图像坐标与运动历史；换分辨率、断流后不能复用旧框。"""
        self.last = self._smooth = self._track = self._prev_quad = None
        self._track_timestamp = self._prev_timestamp = None
        self._frame_shape = None
        self._lost = self._frame = 0
        self._locked_th = None
        self._roi_retry = True
        if self._kalman is not None:
            self._kalman.reset()
        return self

    # ------------------------------------------------------------------ 小工具
    def _continuous_time(self, timestamp):
        """两次采图时刻是否连续（可用于运动预测/滤波外推）。None 一律视为不连续。"""
        if timestamp is None or self._track_timestamp is None:
            return timestamp is None and self._track_timestamp is None
        gap = timestamp - self._track_timestamp
        return 0 < gap <= float(getattr(self.cfg, "PREDICT_MAX_GAP_S", 0.25))

    def _valid_quad(self, q, shape, margin=1):
        """四边形体检（几何判据集中在 rect_geometry.quad_is_valid，便于单测）。"""
        return quad_is_valid(q, shape, margin=margin,
                             min_edge=float(getattr(self.cfg, "MIN_EDGE_LENGTH_PX", 12.0)))

    def _layout(self, q, kind):
        """由四边形朝向推断物理长宽比，并给出该 kind 对应的物理矩形四角。

        horizontal：两条"横向边长之和" > 两条"纵向边长之和" → 长边水平 → w=长边(mm)。
        kind=="inner" 时物理矩形向内缩进一个胶带宽度 BLACK_FRAME_MM（对应内框角点）。
        """
        lengths = side_lengths(q)
        horizontal = lengths[0] + lengths[2] > lengths[1] + lengths[3]
        w, h = ((self.cfg.TARGET_LONG_MM, self.cfg.TARGET_SHORT_MM) if horizontal
                else (self.cfg.TARGET_SHORT_MM, self.cfg.TARGET_LONG_MM))
        inset = self.cfg.BLACK_FRAME_MM if kind == "inner" else 0.0
        return w, h, rectangle_corners(w, h, inset)

    # ------------------------------------------------------------------ ② 四边验证
    def _verify(self, gray, q, kind):
        """在正视面选取四边稀疏采样点并投影，不生成整幅 warp 图。

        每条边检查胶带厚度方向的暗带覆盖率与内侧亮区对比，允许局部反光；
        背景可以是黑色，不再强制外侧为白色。普通纸边及实心黑块会失败。
        """
        if not self._valid_quad(q, gray.shape):
            return None
        w, h, physical_quad = self._layout(q, kind)
        tape = float(self.cfg.BLACK_FRAME_MM)
        if tape <= 0 or 2 * tape >= min(w, h):
            raise ValueError("BLACK_FRAME_MM must be between 0 and half the short side")
        try:
            # 求 mm → px 单应：把物理矩形映射到图像候选四边形 q。
            rect_to_image = homography(physical_quad, q)
            t = self._t
            # 沿每条边的法向采 5 个偏移：0.20/0.40/0.60×胶带宽 = 黑带内部三点（应为暗），
            # 1.22/1.45×胶带宽 = 越过内沿的亮面两点（应为亮）。暗亮对比即边证据。
            offsets = tape * np.array([0.20, 0.40, 0.60, 1.22, 1.45])
            p = np.empty((4, len(t), 5, 2), np.float64)
            # 四条边分别参数化：(边0 上/边1 右/边2 下/边3 左)，每条边沿 t 铺采样点。
            p[0, :, :, 0] = (tape + t * (w - 2 * tape))[:, None]
            p[0, :, :, 1] = offsets
            p[1, :, :, 0] = w - offsets
            p[1, :, :, 1] = (tape + t * (h - 2 * tape))[:, None]
            p[2, :, :, 0] = (tape + (1 - t) * (w - 2 * tape))[:, None]
            p[2, :, :, 1] = h - offsets
            p[3, :, :, 0] = offsets
            p[3, :, :, 1] = (tape + (1 - t) * (h - 2 * tape))[:, None]
            mapped = project_points(rect_to_image, p.reshape(-1, 2)).reshape(p.shape)
        except (ValueError, np.linalg.LinAlgError):
            return None
        gh, gw = gray.shape
        # 任何采样点落到图像外 → 该候选不可信（中心也无法可靠外推）。
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        samples = sample_batch(gray, mapped).astype(np.float32)
        # 每条边、每个 t：暗带取三点最大值（黑带里最亮的那个仍应暗），亮面取两点中位数。
        dark = np.max(samples[:, :, :3], axis=2)
        bright = np.median(samples[:, :, 3:], axis=2)
        delta = bright - dark
        # "该证据成立" = 对比度达标 且 暗带确实比亮面暗（比值约束，排除两侧都亮的纸边）。
        coverage = ((delta >= self.cfg.MIN_EDGE_CONTRAST) &
                    (dark <= bright * self.cfg.MAX_DARK_BRIGHT_RATIO)).mean(axis=1)
        edge_contrast = np.median(delta, axis=1)
        # 至少三边的大部分采样成立，且对比度中位达标。
        if (np.count_nonzero(coverage >= self.cfg.MIN_EDGE_COVERAGE) < 3 or
                np.median(edge_contrast) < self.cfg.MIN_EDGE_CONTRAST):
            return None
        # ★ 允许**一条边**大面积缺失：实机上"缺一个角的黑框"是常态 —— 胶带在角落断掉、
        #    强反光把某一段胶带洗白、或被物体遮挡。旧规则一律要求四边覆盖率 ≥ WEAK_EDGE_COVERAGE，
        #    实测（tools/diagnose_detect.py）会把唯一正确的候选拒掉，表现即"基本识别不到"。
        #    放宽的边界很清楚：只有当**另外三条边都达到 STRONG_EDGE_COVERAGE** 时才豁免，
        #    所以"凭一条亮带猜框"依然被拒。
        if (coverage.min() < self.cfg.WEAK_EDGE_COVERAGE and
                np.sort(coverage)[::-1][:3].min() < self.cfg.STRONG_EDGE_COVERAGE):
            return None
        # ★ 逐边剖面的中位数**一次向量化**算完：这里原先对每边单独调 4 次 np.median，
        #   一轮验证要 15 次 np.median（单次调用开销 ~30µs）—— 剖析显示它占了 _verify
        #   六成时间，而它只是**诊断字段**。向量化后同样信息、调用次数降到 3 次。
        rows = [dict(dark=float(d), inside=float(b), contrast=float(c), coverage=float(v))
                for d, b, c, v in zip(np.median(dark, axis=1), np.median(bright, axis=1),
                                      edge_contrast, coverage)]
        outer = project_points(rect_to_image, rectangle_corners(w, h)).astype(np.float32)
        if not self._valid_quad(outer, gray.shape):
            return None
        coverage = np.asarray(coverage, np.float64)
        contrast = float(np.median(edge_contrast))
        # 返回候选：外框(图像)、内框(胶带内沿)、mm→px 单应、四边剖面、质量分。
        # score  = 中位对比度 + 60×平均覆盖率（历史口径，外部工具在用）。
        # rank   = score + 40×**最小**覆盖率 —— 多候选择优用的排序键：把"缺一条边"的候选
        #          压到"四边齐备"的候选之后。实机教训（板端真机帧）：一个顶边落在纸面上的
        #          假候选靠"一条边缺失"豁免通过验证，score 与真候选只差 3 分，而 rank 差 40 分
        #          ⇒ 用 rank 择优可以稳定选出真框，用 score 则可能选错。
        return dict(quad=q, outer_quad=outer,
                    inner_quad=project_points(rect_to_image, rectangle_corners(w, h, tape)).astype(np.float32),
                    kind=kind, rect_to_image=rect_to_image, width_mm=w, height_mm=h,
                    edge_profile=rows, contrast=contrast,
                    min_contrast=float(edge_contrast.min()), coverage=coverage,
                    score=float(contrast + 60 * coverage.mean()),
                    rank=float(contrast + 60 * coverage.mean() + 40 * coverage.min()))

    def _four_edges_complete(self, candidate):
        """四边齐备（没有靠"缺边豁免"通过）→ 可以直接锁定，不必再跑兜底级别。"""
        return (float(np.asarray(candidate["coverage"]).min()) >= self.cfg.MIN_EDGE_COVERAGE and
                float(candidate["contrast"]) >= self.cfg.MIN_EDGE_CONTRAST)

    # ------------------------------------------------------------------ ③ 亚像素精修
    def _refine(self, gray, q, kind, radius):
        """当前帧四条灰度窄带 → 有方向的边缘跳变 → fitLine → 四角交点。

        与 _verify 的区别：_verify 只判"像不像"，_refine 才真正**重新定位**每条边。
        步骤：沿四边已有点 (base) 的法向 ±radius 采样灰度 → 求有向梯度 → 每列取
        "最近预测位置"的跳变（offset 越小越优先，防止误锁到内侧印刷圆或另一侧胶带边）
        → 用 fitLine 拟合四条直线 → 相邻直线两两求交 = 亚像素四角。
        精修结果偏离原 q 超过 radius*2.8 或四角体检不过 → 判失败（返回 None）。

        ★ 容错（实机必需）：某条边胶带缺失/被反光洗白时，该边**沿用输入四边形的边**，
          只要另有至少三条边真拟合过就继续。旧实现任一条边证据不足就整框返回 None，
          实机"缺一个角"时会让窄带跟踪每帧都失败。
        """
        q = np.asarray(q, np.float32)
        nxt = np.roll(q, -1, axis=0)
        e = nxt - q
        lengths = np.linalg.norm(e, axis=1)
        if lengths.min() < 10:
            return None
        normal = np.column_stack((-e[:, 1], e[:, 0])) / lengths[:, None]   # 各边外/内法向
        base = q[:, None, :] * (1 - self._t[None, :, None]) + nxt[:, None, :] * self._t[None, :, None]
        offsets = np.arange(-radius, radius + 1, dtype=np.float32)
        # 每条边、每个 t、每个法向偏移 → 采样点网格 (4, len(t), 2r+1, 2)。
        points = base[:, :, None, :] + normal[:, None, None, :] * offsets[None, None, :, None]
        gh, gw = gray.shape
        inside = ((points[..., 0] >= 1) & (points[..., 0] < gw - 1) &
                  (points[..., 1] >= 1) & (points[..., 1] < gh - 1)).all(axis=2)
        values = sample_batch(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]                    # 中心差分近似梯度
        if kind == "outer":
            gradient *= -1                                                 # 外框要求暗→亮方向相反
        # 近预测边缘优先，避免捕获靶纸内部印刷圆或另一侧的胶带边缘。
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.45              # 偏离预测位置的惩罚
        indices = np.argmax(gradient - penalty, axis=2)                    # 每股取最强跳变
        strength = np.take_along_axis(gradient, indices[..., None], axis=2)[..., 0]
        selected = offsets[indices + 1]
        lines = []
        refitted = 0
        for i in range(4):
            good = inside[i] & (strength[i] >= self.cfg.MIN_EDGE_CONTRAST * 0.7)   # 有效采样
            shifts = selected[i, good]
            # 该边证据不足（胶带缺失/反光洗白）→ **沿用输入四边形的这条边**（=预测位置），
            # 而不是放弃整框精修：一条边没证据是实机常态，放弃会让帧率与锁定一起塌。
            if good.sum() < max(5, len(self._t) * self.cfg.WEAK_EDGE_COVERAGE):
                lines.append(line_through(q[i], q[(i + 1) % 4]))
                continue
            med = float(np.median(shifts))
            # 去掉偏离中位数过大的离群跳变（反光/杂物），再确认仍够点。
            good &= np.abs(selected[i] - med) <= max(2.5, float(np.median(np.abs(shifts - med))) * 3)
            if good.sum() < 5:
                lines.append(line_through(q[i], q[(i + 1) % 4]))
                continue
            samples = base[i, good] + selected[i, good, None] * normal[i]
            # 用鲁棒 Huber 回归拟合该边直线；fitLine 返回 (vx, vy, x0, y0)。
            vx, vy, x, y = cv2.fitLine(samples, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            # 直线 ax+by+c=0。
            lines.append(np.array([-vy, vx, vy * x - vx * y], np.float64))
            refitted += 1
        if refitted < 3:
            return None                                                    # 至少三条边真拟合过
        lines = np.asarray(lines)
        # 相邻两条直线（roll 1 使其错位一位）叉积求交点：交点齐次坐标 = 两直线系数叉积。
        crosses = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crosses[:, 2]) < 1e-8):
            return None                                                    # 有平行边 → 退化
        refined = (crosses[:, :2] / crosses[:, 2:]).astype(np.float32)
        if (np.linalg.norm(refined - q, axis=1).max() > radius * 2.8 or
                not self._valid_quad(refined, gray.shape)):
            return None                                                    # 跑太远或形状不合法
        return refined

    # ------------------------------------------------------------------ 候选受理
    def _accept(self, gray, q, kind):
        """**轻量受理**：只做四边验证（失败时试宽半径救援），**不做精修**。

        关键在于把"筛选"与"精修"分开：一帧里可能有十几个候选，但最终只会用到一两个。
        旧实现给每个候选都跑一遍"窄带精修 + 再验证"，单帧在板端帧捕获时做了 14 次精修
        （剖析：精修占 41% 时间）—— 绝大多数白做。现在验证只负责"像不像"，
        只有**最终选中的那个**才进 `_finalize` 做亚像素精修。
        """
        verified = self._verify(gray, q, kind)
        if verified is not None:
            return verified
        return self._rescue(gray, q, kind)

    def _finalize(self, gray, candidate, iterations):
        """对最终选中的候选做亚像素精修，**迭代到不动点**，并重新验证。

        为什么迭代：搜索帧的候选离真边可能有几个像素，只精修一轮会停在半路，于是
        "首帧"与"第二帧跟踪"的中心差 ~1px（实机表现：刚锁上时中心轻轻跳一下）。
        迭代到不动点后，两条路径收敛到同一位置（实测 1.14px → 0.11px）。
        """
        kind = candidate["kind"]
        base = candidate["quad"]
        best = candidate
        for _ in range(max(1, int(iterations))):
            refined = self._refine(gray, base, kind, self.cfg.REFINE_RADIUS)
            if refined is None:
                break
            precise = self._verify(gray, refined, kind)
            if precise is None:
                break
            moved = float(np.linalg.norm(np.asarray(refined, np.float64) -
                                        np.asarray(base, np.float64), axis=1).max())
            best, base = precise, refined
            if moved < 0.2:                              # 已到不动点
                break
        return best

    def _rescue(self, gray, q, kind):
        """★ 宽半径兜底精修：候选可能整体偏离真边，`_verify` 的固定小偏移会采到纸面。

        实机实测（2026-10-13）：掩膜给出的候选顶边比胶带外沿低约 14px，而 `_verify` 只从边
        向里采 0.2~0.6×BLACK_FRAME_MM（≈2~6px）⇒ 直接采到纸面、把纸当成"暗带"而误拒；
        窄半径（REFINE_RADIUS / TRACK_RADIUS）都纠不动这种十几 px 的偏差，
        于是画面里黑框清清楚楚却永远 no_black_frame。
        救援必须**确证落在真胶带上**才采纳：① 中位对比度 ≥ RECOVER_MIN_CONTRAST；
        ② **四边暗带灰度的最大值**必须够黑（≤ RECOVER_MAX_DARK）—— 真胶带四边都是黑的，
        而"纸边/褶皱"这类假边界总有一两条边只是中灰。
        """
        wide = float(getattr(self.cfg, "REFINE_RADIUS_WIDE", 0.0))
        if wide <= self.cfg.REFINE_RADIUS:
            return None
        refit = self._refine(gray, q, kind, wide)
        if refit is None:
            return None
        recovered = self._verify(gray, refit, kind)
        if recovered is None:
            return None
        darkest = max(row["dark"] for row in recovered["edge_profile"])
        if (recovered["contrast"] >= float(getattr(self.cfg, "RECOVER_MIN_CONTRAST", 80.0)) and
                darkest <= float(getattr(self.cfg, "RECOVER_MAX_DARK", 100.0))):
            return recovered
        return None

    # ------------------------------------------------------------------ ① 搜索
    def _search(self, gray, roi=None):
        """低分辨率提议 → 全分辨率验证；**单帧内跑完全部级别**（重捕获恒为 1 帧）。

        与旧实现的三点差别：
          · 提议来自 `ContourLocator`（Canny 边缘，必要时补一张 Otsu 掩膜），
            不再是"9~10 档阈值逐档 findContours"；
          · 同一级别内**验证完所有候选再择优**（`rank`，见 `_verify`），
            而不是"第一个通过就返回" —— 假候选靠"缺一条边豁免"抢先通过是实机出现过的
            真实误锁（板端真机帧中心偏 17px，而真候选 rank 高 40 分）；
          · **四边齐备**的候选立即锁定早退（普通场景只花 1 次验证 + 1 次精修），
            靠"缺边豁免/宽半径救援"通过的候选才继续扫完整个级别、比 rank 后再精修。
        `SEARCH_EXHAUSTIVE=True`（默认，用户明确要求的稳定性口径）时候选数不截断；
        False 时按 `SEARCH_MAX_PROPOSALS` 截断 —— 预算在 ROI 与全图搜索之间**共享**。
        """
        self._search_count += 1
        cap = None if bool(getattr(self.cfg, "SEARCH_EXHAUSTIVE", True)) \
            else int(self.cfg.SEARCH_MAX_PROPOSALS)
        iterations = max(1, int(getattr(self.cfg, "REFINE_ITERATIONS", 2)))
        best, best_rank = None, None
        for level in LEVELS:
            proposals = self._locator.locate(gray, roi, level)
            stats = self._locator.last_stats
            self._mask_count += int(stats["masks"])
            for q, kind in proposals:
                if cap is not None and self._proposals >= cap:
                    return self._finalize(gray, best, iterations) if best else None
                self._proposals += 1
                candidate = self._accept(gray, q, kind)
                if candidate is None:
                    continue
                if self._four_edges_complete(candidate):
                    return self._finalize(gray, candidate, 1)     # 四边齐备：立即锁定
                if best is None or candidate["rank"] > best_rank:
                    best, best_rank = candidate, candidate["rank"]
        return self._finalize(gray, best, iterations) if best else None

    # ------------------------------------------------------------------ ④ 中心解算
    def front_view(self, gray, result, short_px=None):
        """把已检出的框**透视变换成正视图**（正视 = 矩形，无透视畸变）。

        返回 (front_view, page_to_image, mm_to_front)；任何一步退化都返回 (None, None, None)。
        正视图一律按**整张物理靶纸**(w×h) 建立（用 `outer_quad` 求单应，与 kind 无关），
        因此无论锁定的是内框还是外框，胶带暗带恒落在正视图四周。
        用途：现场可视化（`tools/diagnose_detect.py --front out.png`）、以及
        `front_view_center` 的独立中心校验。
        """
        if not result or not result.get("valid"):
            return None, None, None
        short_px = int(getattr(self.cfg, "FRONT_VIEW_SHORT_PX", 0) if short_px is None else short_px)
        if short_px <= 0:
            return None, None, None
        w, h = result["width_mm"], result["height_mm"]
        try:
            page_to_image = homography(rectangle_corners(w, h), result["outer_quad"])
            view, mm_to_front = warp_front_view(gray, page_to_image, w, h, short_px)
        except (ValueError, np.linalg.LinAlgError):
            return None, None, None
        if view is None or view.size == 0:
            return None, None, None
        return view, page_to_image, mm_to_front

    def front_view_center(self, gray, result, short_px=None):
        """在**正视图里重新取中心**，再逆透视反投影回原图；失败返回 None。

        与 `process` 里的解析解（"单应把物理中心投回去"）是**两条独立的算法**：
            · 解析解建立在**四角几何**上 —— 四角一旦拟合偏了，中心就跟着偏；
            · 本方法建立在**灰度对称性**上 —— 对单个角点误差不敏感，但对光照不均敏感。
        两者接近说明几何自洽（`tools/test_real_scene.py` 用它锁住 `warp_front_view` 的
        矩阵方向：方向反了会得到"整片背景"，剖面法立刻失效）。
        ⚠ 本方法精度受正视图分辨率限制（128px 短边时典型偏差 3~4px，256px 时约 1px），
          因此**不参与**逐帧中心输出 —— 热路径用解析解，本方法供诊断/校验。
        """
        view, page_to_image, _ = self.front_view(gray, result, short_px)
        if view is None:
            return None
        rows, cols = view.shape
        # 只看中间一半行/列：避开垂直方向的胶带暗带，让剖面里只剩"两侧暗带 + 中间亮区"。
        band = slice(rows // 4, max(rows // 4 + 1, 3 * rows // 4))
        half = slice(cols // 4, max(cols // 4 + 1, 3 * cols // 4))
        cx = _midline_mm(view[band, :].mean(axis=0), result["width_mm"])
        cy = _midline_mm(view[:, half].mean(axis=1), result["height_mm"])
        if cx is None or cy is None:
            return None
        try:
            point = project_points(page_to_image, [[cx, cy]])[0]
        except (ValueError, np.linalg.LinAlgError):
            return None
        return np.asarray(point, np.float64)

    # ------------------------------------------------------------------ 运动预测
    def _motion_delta(self, timestamp=None):
        """上两帧四角的**逐角位移**（图像坐标）；无历史或刚失锁则 None。

        逐角位移天然同时包含平移、缩放与旋转，直接加在"上一帧四角"上即为匀速外推。
        只在**连续两帧都成功**（_lost == 0）时可用：失锁后速度已不可信，宁可退回原地精修。
        位移限幅 PREDICT_MAX_PX 防止噪点/误检把预测甩到画面外。
        """
        if not getattr(self.cfg, "ENABLE_MOTION_PREDICT", True):
            return None
        if self._prev_quad is None or self._track is None or self._lost:
            return None
        delta = np.asarray(self._track["quad"], np.float32) - np.asarray(self._prev_quad, np.float32)
        if timestamp is not None:
            if self._prev_timestamp is None or not self._continuous_time(timestamp):
                return None
            interval = self._track_timestamp - self._prev_timestamp
            if interval <= 0:
                return None
            # 最新帧线程会丢弃旧帧；按采图间隔外推，不能把每次回调当作等间隔视频帧。
            delta = delta * ((timestamp - self._track_timestamp) / interval)
        step = float(np.linalg.norm(delta, axis=1).max())
        cap = float(getattr(self.cfg, "PREDICT_MAX_PX", 60.0))
        if cap > 0 and step > cap:
            delta = delta * (cap / step)
        return delta

    # ------------------------------------------------------------------ 主入口
    def process(self, gray, timestamp=None):
        """单帧入口：按"track → roi_search → full_search"三级降级找框，输出中心与结果 dict。"""
        started = time.perf_counter()
        g = np.asarray(gray)
        if g.ndim == 3 and g.shape[2] == 1:
            g = g[:, :, 0]
        if g.ndim != 2 or g.dtype != np.uint8:
            raise ValueError("process expects a uint8 grayscale image")
        if min(g.shape) < 2:
            self.reset()
            self.last = dict(valid=False, stage="empty_frame", search_count=0,
                             masks=0, proposals=0, detector_ms=0.0)
            return self.last
        timestamp = None if timestamp is None else float(timestamp)
        if timestamp is not None and not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self._frame_shape != g.shape:
            self.reset()
            self._frame_shape = g.shape
        if self._track is not None and not self._continuous_time(timestamp):
            self._prev_quad = self._prev_timestamp = self._smooth = None
            if self._kalman is not None:
                self._kalman.reset()
        self._frame += 1
        self._search_count = self._mask_count = self._proposals = 0
        candidate, stage = None, "search"
        # ---- ① track：上一帧已锁定 → 先用**运动预测位置**、再用上一帧位置做窄带精修 ----
        # 预测位置让窄带跟上快速移动（不必靠加大 TRACK_RADIUS 牺牲慢速稳定性）；
        # 预测失准（突然停止/变向）时自动回落到上一帧位置，两级都试。
        if self._track is not None:
            kind = self._track["kind"]
            guesses = []
            delta = self._motion_delta(timestamp)
            if delta is not None:
                guesses.append(self._track["quad"] + delta)
            guesses.append(self._track["quad"])
            for guess in guesses:
                q = self._refine(g, guess, kind, self.cfg.TRACK_RADIUS)
                if q is None:
                    continue
                verified = self._verify(g, q, kind)
                if verified is not None:
                    candidate, stage = verified, "track"
                    break
        # ---- ② roi_search：刚失锁 → 在上一帧外框（含预测位移）外扩的窗口内搜一次 ----
        if candidate is None and self._track is not None and self._roi_retry:
            q = np.asarray(self._track["outer_quad"], np.float32)
            delta = self._motion_delta(timestamp)
            if delta is not None:
                q = q + delta                                  # 窗口中心跟上预测位移
            lo, hi = q.min(axis=0), q.max(axis=0)
            pad = np.maximum((hi - lo) * self.cfg.ROI_PADDING, 24)         # 外扩量（至少 24px）
            size = np.array([g.shape[1], g.shape[0]])
            lo = np.clip(np.floor(lo - pad), 0, size).astype(int)
            hi = np.clip(np.ceil(hi + pad), 0, size).astype(int)
            if np.all(hi - lo >= 2):
                candidate = self._search(g, (int(lo[0]), int(lo[1]),
                                             int(hi[0] - lo[0]), int(hi[1] - lo[1])))
            stage = "roi_search"
            self._roi_retry = False                                        # 只允许一次 ROI 搜索
        # ★ v0.6.4 口径保留：跟踪与 ROI 都失败后**必须**再跑一次完整全图搜索。
        #   旧守卫写作 `(not SEARCH_SPLIT_FRAMES and self._proposals < SEARCH_MAX_PROPOSALS)`：
        #   只要前面的 ROI 搜索把候选预算烧光，本帧就**跳过**全图搜索、直接判 no_black_frame
        #   —— 画面里黑框再清楚也照样"丢一帧"。这正是 v1–v3 没有、而后续版本引入的回归。
        exhaustive = bool(getattr(self.cfg, "SEARCH_EXHAUSTIVE", True))
        if (candidate is None and
                (exhaustive or self._search_count == 0 or
                 self._proposals < self.cfg.SEARCH_MAX_PROPOSALS)):
            # ---- ③ full_search：彻底丢失 → 全图低分辨率定位（1~2 张掩膜） ----
            candidate = self._search(g)
            stage = "full_search"
        if candidate is None:
            # 三路都没找到：清空平滑中心与预测历史，返回无效结果（不沿用旧框，避免"粘住"）。
            self._smooth = None
            self._lost += 1
            self._prev_quad = None
            self._prev_timestamp = None
            self.last = dict(valid=False, stage="no_black_frame", search_count=self._search_count,
                             masks=self._mask_count, proposals=self._proposals,
                             detector_ms=(time.perf_counter() - started) * 1000)
            return self.last
        # ---- 找到：锁定、清进度、算中心 ----
        # 预测历史只在"连续两帧都成功"时可用：上上帧 = 旧 _track，失锁后清空避免用旧速度外推。
        self._prev_quad = None
        self._prev_timestamp = None
        if (self._track is not None and not self._lost and
                self._track["kind"] == candidate["kind"] and self._continuous_time(timestamp)):
            # 重搜会重新选“左上角”，旋转跨过该分界时编号可能循环移位。
            # 只匹配同一种内/外框的物理角，避免把胶带厚度或编号切换当作速度。
            previous = np.asarray(self._track["quad"], np.float32)
            if stage == "track":
                self._prev_quad = previous                 # 窄带精修保留编号，稳态无需匹配
            else:
                matches = np.stack([np.roll(previous, shift, axis=0) for shift in range(4)])
                errors = np.sum((matches - candidate["quad"]) ** 2, axis=(1, 2))
                self._prev_quad = matches[int(np.argmin(errors))]
            self._prev_timestamp = self._track_timestamp
        self._track = candidate
        self._track_timestamp = timestamp
        self._lost = 0
        self._roi_retry = True
        w, h = candidate["width_mm"], candidate["height_mm"]
        matrix = candidate["rect_to_image"]
        # ★ 中心 = 正视面中心 (w/2, h/2) 经 mm→px 单应**逆投影**回原图。
        #   （正视图取中心再逆透视反投影，与这一步数学等价；见 `_front_view_center` 的交叉校验。）
        # ★ 中心 = 正视面中心 (w/2, h/2) 经 mm→px 单应**逆投影**回原图 ——
        #   这就是"透视变换成正视图 → 取正视图中心 → 逆透视反投影"的闭式等价写法：
        #   正视图中心恒在 (w/2, h/2)，把它经 rect_to_image 投回原图即得，且**没有重采样误差**。
        #   （重采样版本见 front_view()/front_view_center()，供可视化与独立校验。）
        raw = project_points(matrix, [[w / 2, h / 2]])[0]
        center = self._filter_center(raw, timestamp)
        # 圆周采样点（仅可视化/交叉校验用）按尺寸缓存，避免每帧重算。
        key = (w, h, self.cfg.CIRCLE_RADIUS_MM, self.cfg.CIRCLE_SAMPLES)
        if key not in self._circle_cache:
            self._circle_cache[key] = standard_circle(*key)
        result = dict(candidate)
        result.update(valid=True, u=float(center[0]), v=float(center[1]),
                      raw_u=float(raw[0]), raw_v=float(raw[1]),
                      quad=candidate["outer_quad"], detected_quad=candidate["quad"],
                      outer_inferred=candidate["kind"] == "inner",
                      circle_points=project_points(matrix, self._circle_cache[key]),
                      stage=stage, threshold=self._locked_th, search_count=self._search_count,
                      masks=self._mask_count, proposals=self._proposals,
                      detector_ms=(time.perf_counter() - started) * 1000)
        self.last = result
        return result

    # ------------------------------------------------------------------ ⑤ 时序滤波
    def _filter_center(self, raw, timestamp):
        """把逐帧中心送进卡尔曼滤波；返回给显示/常规使用的平滑中心。

        ★ 时钟口径：`timestamp` 与上一帧**不连续**（首帧 / 回退 / 断流 / 传 None）时，
          滤波器重新播种 —— 平滑输出 == 原始测量。这样"时钟不连续必须丢弃旧速度"
          这条既有约束（tools/test_tracking.test_clock_discontinuity）同时作用于滤波器，
          不会出现"检测已判不连续、滤波还在拿旧速度外推"的两套口径。
        """
        raw = np.asarray(raw, np.float64).reshape(2)
        kalman = self._kalman
        if kalman is not None:
            if timestamp is None or kalman.t is None:
                kalman.seed(raw, timestamp)
                estimate = raw
            else:
                estimate = kalman.update(raw, timestamp)
                if estimate is None:
                    estimate = raw
            self._smooth = np.asarray(estimate, np.float64).copy()
            return self._smooth
        # KALMAN_ENABLE=False：退回一阶 EMA（旧口径，保留以便现场对照）。
        alpha = float(self.cfg.CENTER_ALPHA)
        if (self._smooth is None or alpha <= 0 or
                np.linalg.norm(raw - self._smooth) > self.cfg.FILTER_RESET_PX):
            center = raw
        else:
            center = self._smooth + alpha * (raw - self._smooth)
        self._smooth = center
        return center

    # ------------------------------------------------------------------ 速度/预测查询
    def velocity(self):
        """当前中心速度 (vx, vy) px/s；滤波关闭或无状态时为 None。"""
        return None if self._kalman is None else self._kalman.velocity()

    def predict_center(self, timestamp):
        """按运动学模型外推中心位置（不改状态）；用于给 ROI/窄带中心提供先验。"""
        return None if self._kalman is None else self._kalman.predict_at(timestamp)


def draw_result(gray, result, show_width=480, top_offset=0):
    """缩小后再转 BGR 和描边，输出坐标仍为相机原图坐标。

    top_offset：屏幕上方的文字整体下移量（像素）。激光读数面板占用左上角时由
    main.py 传 vision.laser_spot.PANEL_H，避免两处文字叠在一起。

    绘制元素（均按 scale 换算到缩小图坐标，避免在原始分辨率上画）：
        · 绿色外框 quad、青色内框 inner_quad
        · 洋红四角圆点 + "P1..P4 (x,y)" 标签（原图坐标）
        · 蓝色靶心十字 + "(u,v) stage 耗时" 文字
        · 黄色小十字 = 正视图剖面中心 `front_u/v`（与蓝色十字不等时说明两法不一致）
    返回 BGR 画布，交由 Preview.render 再 letterbox 到屏幕画布。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / g.shape[1])        # 原图 → 缩小图 的比例
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not result.get("valid"):
        # 无效：只提示一行红字，不画任何框（过期结果已在 display_result 降级）。
        cv2.putText(canvas, "NO BLACK RECTANGLE", (8, 24 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        return canvas
    q = np.rint(result["quad"] * scale).astype(np.int32)          # 外框（缩小图坐标）
    iq = np.rint(result["inner_quad"] * scale).astype(np.int32)   # 内框（缩小图坐标）
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [iq], True, (0, 220, 220), 1)
    for i, (x, y) in enumerate(result["quad"]):
        p = tuple(q[i])
        cv2.circle(canvas, p, 4, (255, 0, 255), -1)               # 洋红实心点
        # 标签用**原图坐标**（x,y），便于与日志/回算对照；位置做边界夹取防出画。
        label = "P%d (%d,%d)" % (i + 1, round(float(x)), round(float(y)))
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .36, 1)[0][0]
        pos = (max(0, min(int(p[0]) + 5, canvas.shape[1] - tw - 1)),
               max(12, min(int(p[1]) - 7, canvas.shape[0] - 5)))
        cv2.putText(canvas, label, pos, cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 0), 1, cv2.LINE_AA)
    c = tuple(np.rint(np.array([result["u"], result["v"]]) * scale).astype(int))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)   # 蓝色靶心十字
    cv2.putText(canvas, "(%d,%d) %s %.1fms" % (round(result["u"]), round(result["v"]),
                result["stage"], result["detector_ms"]), (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
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
    两轴都是"误差为正发 CCW"，因此屏幕上这个符号可以直接进控制律。

    use_raw：闭环必须用这一档（True）—— 取 result["raw_u"]/["raw_v"]（未平滑）。
    result["u"]/["v"] 是经 CENTER_ALPHA 一阶低通的值，只适合显示；拿它闭环会附加
    约 28ms 滞后（需求文档 §5.2），底盘运动时直接变成瞄准误差。
    激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])   # 闭环档：未平滑中心
    else:
        tx, ty = float(result["u"]), float(result["v"])           # 显示档：CENTER_ALPHA 平滑中心
    return (tx - float(laser["u"]), ty - float(laser["v"]))       # dx=靶心−光斑, dy 同理


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

import math
import time

CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2


def estimate_move_seconds(pulses, rpm, acc=0, pulses_per_rev=3200):
    """估算从静止到静止的位置运动时长（秒），含驱动器的加减速曲线。

    非零 acc 每 ``(256-acc)*50us`` 改变 1 RPM；短行程使用三角曲线，
    长行程使用梯形曲线。这里只做排程估算，不代表收到实际到位反馈。
    """
    pulses, rpm, pulses_per_rev = float(pulses), float(rpm), float(pulses_per_rev)
    if not all(math.isfinite(v) for v in (pulses, rpm, pulses_per_rev)):
        raise ValueError("motion parameters must be finite")
    if pulses <= 0:
        return 0.0
    if rpm <= 0 or pulses_per_rev <= 0:
        raise ValueError("rpm and pulses_per_rev must be positive")
    rpm = min(5000.0, rpm)
    speed = rpm * pulses_per_rev / 60.0
    acc = max(0, min(255, int(acc)))
    if acc == 0:
        return pulses / speed
    ramp = (256 - acc) * 50e-6
    acceleration = pulses_per_rev / (60.0 * ramp)
    ramp_seconds = rpm * ramp
    ramp_distance = speed * ramp_seconds
    if pulses <= ramp_distance:
        return 2.0 * math.sqrt(pulses / acceleration)
    return 2.0 * ramp_seconds + (pulses - ramp_distance) / speed


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

    def __init__(self, cfg):
        self.cfg = cfg
        self._ports = {}
        self._rxbuf = {"yaw": bytearray(), "pitch": bytearray()}
        self._stats = {"tx": 0, "rx": 0, "ok": 0, "e2": 0, "err": 0,
                       "timeout": 0, "yaw": True, "pitch": True}
        self._last_status = {"yaw": None, "pitch": None}
        self._last_cmd = {"yaw": 0.0, "pitch": 0.0}
        self._last_ack = {"yaw": 0.0, "pitch": 0.0}
        # 保留最早一条尚未确认的命令时间；持续发新命令不能推迟断链检测。
        self._pending_ack_since = {"yaw": None, "pitch": None}
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
            self._last_cmd[axis] = time.perf_counter() if now is None else float(now)
            if self._pending_ack_since[axis] is None:
                self._pending_ack_since[axis] = self._last_cmd[axis]
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
            self._pending_ack_since[axis] = None
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
        # ACK 硬超时：只在收到过确认帧（Response=Receive/Reached/Both）之后才生效，
        # Response=None 时永远收不到确认帧，该判据自动不启用，不会误判 FAULT。
        timeout_ms = float(getattr(self.cfg, "RX_TIMEOUT_MS", 0) or 0)
        if timeout_ms > 0 and self._stats["ok"] > 0:
            for axis in self._ports:
                pending = self._pending_ack_since[axis]
                if pending is None:
                    # 兼容只注入时间戳的离线链路，也排除已经得到 ACK 的命令。
                    if self._last_ack[axis] >= self._last_cmd[axis]:
                        self._timeout_count[axis] = 0
                        continue
                    pending = self._last_cmd[axis]
                if now - pending <= timeout_ms / 1000.0:
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
        now = time.perf_counter() if now is None else float(now)
        return self._ingest(self._drain(), now)

    def stats(self):
        return dict(self._stats)

    def close(self):
        """控制线程停止后释放 UART；无 close 方法的 SDK 对象通过清引用析构。"""
        ports, self._ports = self._ports, {}
        self._stats["yaw"] = self._stats["pitch"] = False
        error = None
        for port in ports.values():
            close = getattr(port, "close", None)
            if close is not None:
                try:
                    close()
                except Exception as exc:
                    error = error or exc
        if error is not None:
            raise error
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
        acceleration = int(getattr(self.cfg, "ACC_DEFAULT", 0))
        if not self.link.move(axis, ccw, pulses, rpm, acceleration):
            self.state, self.reason = "FAULT", "uart_write_failed"
            self.link.stop(axis)
            print("[self-test] fault: %s command failed" % axis)
            return self.state
        pulses_per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
        move_s = estimate_move_seconds(pulses, rpm, acceleration, pulses_per_rev)
        settle_s = float(getattr(self.cfg, "SELF_TEST_SETTLE_S", 0.35))
        self.deadline = now + move_s + settle_s
        self.index += 1
        print("[self-test] %s %s %dp rpm=%d step=%d/%d" %
              (axis, "CCW" if ccw else "CW", pulses, rpm,
               self.index, len(self.sequence)))
        return self.state


def probe_imu():
    """尽力实例化 MaixPy IMU 并读取一次样本；不影响主程序运行。"""
    try:
        import maix
        imu = getattr(maix, "imu", None)
        if imu is None:
            import importlib
            imu = importlib.import_module("maix.imu")
        names = [name for name in dir(imu) if not name.startswith("_")]
        device = None
        sample = None
        errors = []
        for class_name in ("IMU", "Imu", "imu"):
            constructor = getattr(imu, class_name, None)
            if not callable(constructor):
                continue
            try:
                device = constructor()
                break
            except Exception as exc:
                errors.append("%s(): %s" % (class_name, exc))
        if device is None:
            return {"available": False, "module": imu, "names": names,
                    "error": "; ".join(errors) or "module has no recognized IMU constructor"}
        for method_name in ("read", "get_data", "get_accel_gyro", "get_gyro"):
            method = getattr(device, method_name, None)
            if callable(method):
                try:
                    sample = method()
                    return {"available": True, "module": imu, "names": names,
                            "device": device, "method": method_name,
                            "sample": sample, "error": None}
                except Exception as exc:
                    errors.append("%s(): %s" % (method_name, exc))
        return {"available": False, "module": imu, "names": names,
                "device": device, "error": "; ".join(errors) or
                "IMU object has no recognized read method"}
    except Exception as exc:
        return {"available": False, "module": None, "names": [], "error": str(exc)}
'''

# ===== vision.gimbal_aim  (vision/gimbal_aim.py) =====
_MOD_SRC['vision.gimbal_aim'] = r'''
"""基于像素误差的两轴增量控制器 + 状态机。

对照 `MaixCAM2串口直控云台需求文档.md` §7 实现：

- 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 ``laser_spot.deviation``）；
- **相对位置**命令（0xFD、mode=0），单次脉冲限幅；两轴独立渐变增益和速度；
- 可等待加减速运动完成与运动后的新图像，避免重复使用运动前误差；
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
        self._motion_dir = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = None
        self._stop_axes = []
        self._source = "vision"

    # ------------------------------------------------------------- 外部接口
    @property
    def capture_after(self):
        """最近一次任意轴停止后的采图门槛，供视觉/IMU参考共同使用。"""
        return max(self._capture_after.values())

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
        self.state = BOOT
        self.fault_reason = None
        self._lost = self._settle = self._scan_pos = 0
        self._scan_dir = 1
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        self.motion_stopped()
        return self.state

    def motion_stopped(self, axis=None, now=None):
        """上层已停机时清除排程和历史；提供停机时间可拒收停机前的图像。"""
        axes = ("yaw", "pitch") if axis is None else (axis,)
        for name in axes:
            if name not in self._busy_until:
                raise ValueError("axis must be yaw or pitch")
            self._busy_until[name] = -1e9
            self._capture_after[name] = float(now) if now is not None and math.isfinite(float(now)) else -1e9
            self._motion_dir[name] = 0
            self._jump[name] = 0
        self._last_measurement_time = None

    def _request_stop(self, axis, now):
        if axis not in self._stop_axes:
            self._stop_axes.append(axis)
        captured_at = self._last_measurement_time
        self.motion_stopped(axis, now=now)
        self._last_measurement_time = captured_at

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD / SCAN）。"""
        now = float(now)
        self._stop_axes = []
        if self.state == FAULT:
            return self._emit(None, None, {"yaw": None, "pitch": None})
        valid = bool(result and result.get("aim_valid") and
                     result.get("dx") is not None and result.get("dy") is not None)
        if not valid or not math.isfinite(now):
            return self._on_lost(now)
        try:
            dx, dy = float(result["dx"]), float(result["dy"])
            captured_at = float(result.get("captured_at", now))
        except (TypeError, ValueError, OverflowError):
            return self._on_lost(now)
        if not all(math.isfinite(v) for v in (dx, dy, captured_at)) or captured_at > now + 1e-6:
            return self._on_lost(now)
        if self._last_measurement_time is not None:
            if captured_at <= self._last_measurement_time:
                # 重复或倒序图像不是新观测，不能再追加相对位置动作。
                return self._emit(dx, dy, {"yaw": None, "pitch": None})
            gap = float(getattr(self.cfg, "AIM_HISTORY_MAX_GAP_S", 0.0))
            if gap > 0 and captured_at - self._last_measurement_time > gap:
                self._jump = {"yaw": 0, "pitch": 0}
        self._last_measurement_time = captured_at
        self._source = "imu" if (result.get("source") == "imu" and
                                    bool(getattr(self.cfg, "ENABLE_IMU_FEEDFORWARD", False))) else "vision"
        self._lost = 0
        self._scan_pos = 0
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            self._settle += 1
            self.state = SETTLE
            for axis in ("yaw", "pitch"):
                self._stop_if_needed(axis, 0.0, now, captured_at)
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
        self._settle = 0
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        self.state = COARSE if (coarse_px > 0 and max(abs(dx), abs(dy)) > coarse_px) else FINE
        cmds = {"yaw": self._axis_command("yaw", dx, now, captured_at),
                "pitch": self._axis_command("pitch", dy, now, captured_at)}
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
        out = {"state": self.state, "dx": dx, "dy": dy, "cmds": cmds,
               "stop_axes": list(self._stop_axes)}
        self.last = out
        return out

    def _deadband(self):
        return float(getattr(self.cfg, "DEADBAND_PX", 2.0))

    def _on_lost(self, now):
        for axis in ("yaw", "pitch"):
            if self._motion_dir[axis] and (not math.isfinite(now) or now < self._busy_until[axis]):
                self._request_stop(axis, now)
        self.motion_stopped(now=now)
        self._last_measurement_time = None
        self._jump = {"yaw": 0, "pitch": 0}
        self._lost += 1
        self._settle = 0
        scan_on = bool(getattr(self.cfg, "ENABLE_SCAN", False))
        if scan_on and math.isfinite(now) and self._lost >= int(getattr(self.cfg, "LOST_FRAMES", 15)):
            self.state = SCAN
            cmd = self._scan_command("yaw", now)
            return self._emit(None, None, {"yaw": cmd, "pitch": None})
        self.state = HOLD
        return self._emit(None, None, {"yaw": None, "pitch": None})

    def _gain(self, axis, error):
        cfg = self.cfg
        if self._source == "imu":
            return max(0.0, min(1.0, float(getattr(cfg, "IMU_GAIN", 0.75))))
        fine = float(getattr(cfg, "GAIN_YAW" if axis == "yaw" else "GAIN_PITCH", 0.5))
        coarse_px = float(getattr(cfg, "COARSE_PX", 0.0))
        if bool(getattr(cfg, "AIM_PROGRESSIVE", False)) and coarse_px > self._deadband():
            coarse = float(getattr(cfg, "GAIN_COARSE", min(1.0, fine * 1.8)))
            mix = max(0.0, min(1.0, (abs(error) - self._deadband()) / (coarse_px - self._deadband())))
            return max(0.0, min(1.0, fine + mix * (coarse - fine)))
        if coarse_px > 0 and abs(error) > coarse_px:
            return float(getattr(cfg, "GAIN_COARSE", min(1.0, fine * 1.8)))
        return fine

    def _rpm(self, error):
        if self._source == "imu" and abs(error) > float(getattr(self.cfg, "COARSE_PX", 24)):
            return max(1, min(5000, int(getattr(self.cfg, "IMU_RPM", 180))))
        coarse_px = float(getattr(self.cfg, "COARSE_PX", 0.0))
        fine = int(getattr(self.cfg, "RPM_FINE", 300))
        coarse = int(getattr(self.cfg, "RPM_COARSE", 1200))
        if bool(getattr(self.cfg, "AIM_PROGRESSIVE", False)) and coarse_px > self._deadband():
            mix = max(0.0, min(1.0, (abs(error) - self._deadband()) / (coarse_px - self._deadband())))
            return max(1, int(round(fine + mix * (coarse - fine))))
        if coarse_px > 0 and abs(error) > coarse_px:
            return int(getattr(self.cfg, "RPM_COARSE", 1200))
        return fine

    def _stop_if_needed(self, axis, error, now, captured_at):
        direction = self._motion_dir[axis]
        if not direction or captured_at <= self.last_command_time[axis]:
            return False
        if now >= self._busy_until[axis]:
            return False
        in_band = abs(error) <= self._deadband()
        reversed_error = error * direction < 0
        if in_band or reversed_error:
            self._request_stop(axis, now)
            return True
        return False

    def _axis_command(self, axis, error, now, captured_at):
        cfg = self.cfg
        # 离群拒收：单帧超大误差先当误检丢弃，连续若干帧才采信（防甩头）。
        jump = float(getattr(cfg, "JUMP_REJECT_PX", 0.0))
        if self._source != "imu" and jump > 0 and abs(error) > jump:
            self._jump[axis] += 1
            if self._jump[axis] < int(getattr(cfg, "JUMP_CONFIRM_FRAMES", 3)):
                return None
        else:
            self._jump[axis] = 0
        if self._stop_if_needed(axis, error, now, captured_at):
            return None
        if abs(error) <= self._deadband():
            return None
        interval = float(getattr(cfg, "COMMAND_INTERVAL_S", 0.08))
        if now - self.last_command_time[axis] < interval:
            return None
        if bool(getattr(cfg, "AIM_WAIT_MOTION", False)):
            if (now < self._busy_until[axis] or captured_at <= self._busy_until[axis]
                    or captured_at <= self._capture_after[axis]):
                return None
        sign = float(getattr(cfg, "DIR_SIGN_YAW" if axis == "yaw" else "DIR_SIGN_PITCH", 1))
        max_pulses = int(getattr(cfg, "MAX_STEP_PULSES", 360))
        if self._source == "imu":
            ppd = float(getattr(cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
            if ppd <= 0:
                ppd = float(getattr(cfg, "GIMBAL_GEAR_RATIO", 0.0)) * float(
                    getattr(cfg, "MOTOR_PULSES_PER_REV", 3200)) / 360.0
            step_deg = float(getattr(cfg, "IMU_MAX_STEP_DEG", 30.0))
            if not all(math.isfinite(v) for v in (ppd, step_deg)) or ppd <= 0 or not 0 < step_deg <= 45:
                return None
            max_pulses = min(0xFFFFFFFF, int(ppd * step_deg))
        if self.controller is not None:
            output = self.controller(axis, float(error), now)
            if output is None or not math.isfinite(float(output)) or float(output) == 0:
                return None
            pulses = min(max_pulses, max(1, int(round(abs(float(output))))))
            ccw = float(output) * sign > 0
        else:
            ppp = float(getattr(cfg, "PULSE_PER_PX_YAW" if axis == "yaw"
                                else "PULSE_PER_PX_PITCH", 3.67))
            pulses = min(max_pulses, max(1, int(round(abs(error) * ppp * self._gain(axis, error)))))
            ccw = error * sign > 0
        if max_pulses <= 0:
            return None
        rpm = self._rpm(error)
        self.last_command_time[axis] = now
        duration = estimate_move_seconds(pulses, rpm,
                                       getattr(cfg, "IMU_ACC", 80) if self._source == "imu" else
                                       getattr(cfg, "ACC_TRACK", getattr(cfg, "ACC_DEFAULT", 0)),
                                       getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))
        self._busy_until[axis] = now + duration + max(0.0, float(getattr(cfg, "AIM_MOTION_MARGIN_S", 0.0)))
        self._motion_dir[axis] = 1 if (1 if ccw else -1) * sign > 0 else -1
        return ccw, pulses, rpm

    def _scan_command(self, axis, now):
        cfg = self.cfg
        if now - self.last_command_time[axis] < float(getattr(cfg, "SCAN_STEP_INTERVAL_S", 0.15)):
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
        return ccw, step, int(getattr(cfg, "SCAN_RPM", 60))
'''

# ===== vision.gimbal_search  (vision/gimbal_search.py) =====
_MOD_SRC['vision.gimbal_search'] = r'''
"""按钮启动的非阻塞双轴寻靶：相对按钮时姿态，各轴覆盖正负 45°。

这里记录的是已请求的相对位置，不是编码器实测角度。只允许使用实测
脉冲/度或已知减速比；途中识别到黑框、取消或故障时立即停车，不猜测
剩余脉冲，也不自动回零。UART 与本状态机均由控制线程独占。

★ v0.6.3 两处修复（用户实报"按 SEARCH 和没按一样，且没有和陀螺仪结合查找"）：
  ① **未标定脉冲/度时不再一动不动**：旧行为是直接进 `UNCALIBRATED` 并 return，
     一条指令都不发（实机实测 `YAW/PITCH_PULSES_PER_DEG=0`、`GIMBAL_GEAR_RATIO=0`）。
     现按 `SEARCH_PULSES` 的**固定脉冲数**扫描 —— 与 `SELF_TEST_PULSES` 已有的口径一致。
     置 `SEARCH_PULSES=0` 可显式退回旧行为。
  ② **IMU 辅助收段**：给了 `heading` 且其相对朝向可用时，每段按**陀螺仪实测角度**提前
     收段（达到目标角度即停），因此 ±45° 不再依赖脉冲/度标定；未标定/无样本时静默退回
     脉冲口径。段与段之间用**相对脉冲增量**表达，所以某段提前收段也不会把后续端点带偏。
"""
import math

from .stepper_link import estimate_move_seconds


class SearchSweep:
    """每帧/idle 调用 update；构造不发运动，终态后不会再发运动指令。

    调度器启动前应停止其它运动。ACQUIRED/DONE/CANCELLED 的 stopped_at
    是停机帧成功写出的时刻：切回追踪必须等待该时刻之后采集的新画面。
    stop 写失败则为 FAULT，不能将 stop_attempted_at 当成成功停机。
    """

    TERMINAL = ("DONE", "ACQUIRED", "CANCELLED", "UNCALIBRATED", "FAULT")

    def __init__(self, cfg, link, now=0.0, heading=None):
        self.cfg, self.link, self.heading = cfg, link, heading
        self.state, self.reason = "RUNNING", None
        self.index, self.sequence = 0, []
        self.active_axis = None
        self.stopped_at = self.stop_attempted_at = None
        self.position_known = True
        self.commanded_pulses = {"yaw": 0, "pitch": 0}
        self.pulses_per_degree = {}
        self.limit_pulses = {}
        # 每段**相对起始位置**的脉冲增量与目标角度增量（IMU 收段用）。
        self.segment_deltas = []
        self.segment_angles = []
        self.imu_active = False
        self.imu_note = None
        self.calibration_report = {}
        self._measured_slots = set()
        self._reported_calibration = False
        self._seg_angles_at_start = {"yaw": 0.0, "pitch": 0.0}
        self.started_at = self.deadline = self._last_now = float(now)
        self._last_capture = -math.inf
        self._prepare()

    @property
    def done(self):
        return self.state in self.TERMINAL

    @property
    def positions_deg(self):
        """累计请求端点；中途停车后不代表实际姿态（见 position_known）。

        未标定脉冲/度（值为 None）的轴不参与换算 —— 它只有脉冲口径，没有角度口径。
        """
        return {axis: self.commanded_pulses[axis] / ppd
                for axis, ppd in self.pulses_per_degree.items() if ppd}

    def _prepare(self):
        try:
            if not math.isfinite(self.started_at):
                raise ValueError("invalid_timestamp")
            self.rpm = int(getattr(self.cfg, "SEARCH_RPM", 60))
            self.acc = int(getattr(self.cfg, "SEARCH_ACC", 60))
            self.settle_s = float(getattr(self.cfg, "SEARCH_SETTLE_S", 0.10))
            self.timeout_s = float(getattr(self.cfg, "SEARCH_TIMEOUT_S", 60.0))
            self.max_age_s = float(getattr(self.cfg, "RESULT_MAX_AGE_S", 0.20))
            self.per_rev = float(getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200))
            numbers = (self.settle_s, self.timeout_s, self.max_age_s, self.per_rev)
            if not all(math.isfinite(value) for value in numbers):
                raise ValueError("invalid_config")
            if (not 1 <= self.rpm <= 5000 or not 0 <= self.acc <= 255 or
                    self.settle_s < 0 or self.timeout_s <= 0 or
                    self.max_age_s <= 0 or self.per_rev <= 0):
                raise ValueError("invalid_config")
            ratio = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0))
            fallback = int(getattr(self.cfg, "SEARCH_PULSES", 0))
            for axis in ("yaw", "pitch"):
                angle = float(getattr(self.cfg, "SEARCH_" + axis.upper() + "_DEG", 45.0))
                if not math.isfinite(angle) or not 0 < angle <= 45.0:
                    raise ValueError("search_angle_out_of_range")
                ppd = float(getattr(self.cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
                if ppd <= 0 and math.isfinite(ratio) and ratio > 0:
                    ppd = self.per_rev * ratio / 360.0
                if not math.isfinite(ppd) or ppd <= 0:
                    # ★ 未标定脉冲/度：按固定脉冲数扫描（不再是"一动不动"）。
                    if fallback <= 0:
                        self.state, self.reason = "UNCALIBRATED", "pulse_per_degree_not_calibrated"
                        return
                    limit = fallback
                    known_ppd = None
                else:
                    # 端点先量化、再作差；不会因逐段取整把累计端点推到 ±45° 之外。
                    limit = int(math.floor(angle * ppd))
                    known_ppd = ppd
                if limit <= 0 or 2 * limit > 0xFFFFFFFF:
                    raise ValueError("invalid_pulse_count")
                self.pulses_per_degree[axis] = known_ppd
                self.limit_pulses[axis] = limit
                # 序列写成**相对增量**：+limit → −2·limit → +limit（等价于端点 +limit/−limit/0），
                # 这样某段被 IMU 提前收段时，后续段仍以"相对当前姿态"下发，不会累积错位。
                for delta, target_angle in ((limit, angle), (-2 * limit, -angle), (limit, 0.0)):
                    self.sequence.append((axis, delta))
                    self.segment_deltas.append((axis, delta))
                    self.segment_angles.append((axis, target_angle))
            self._activate_imu()
            self.timeout_at = self.started_at + self.timeout_s
        except (TypeError, ValueError, OverflowError) as exc:
            self.state, self.reason = "FAULT", str(exc) or "invalid_config"

    def _activate_imu(self):
        """尝试用 IMU 相对朝向做角度收段；任何不可用都静默退回脉冲口径。"""
        self.imu_active = False
        self.imu_note = "imu_off"
        if self.heading is None or not bool(getattr(self.cfg, "SEARCH_USE_IMU", True)):
            return
        try:
            if not self.heading.lock_reference(self.started_at):
                self.imu_note = "imu_reference_unavailable"
                return
        except Exception as exc:                                  # 传感器/线程异常都不致命
            self.imu_note = "imu_error:%s" % exc
            return
        self.imu_active = True
        self.imu_note = "imu_angle_terminated"

    def _measure_segment(self, now):
        """段结束时记录「指令脉冲 vs IMU 实测转角」，一次 SEARCH 即可反推脉冲/度。

        价值：现场不必专门跑开环标定 —— 点一次 SEARCH，日志里的
        `≈X 脉冲/度` 就是可以直接填回 `YAW/PITCH_PULSES_PER_DEG` 的标定值。
        只在 IMU 可用时统计；采样不可用则跳过（该项缺失不影响搜索本身）。
        """
        slot = self.index - 1
        if not (self.imu_active and 0 <= slot < len(self.segment_deltas)):
            return
        if slot in self._measured_slots:
            return                                                   # 同段只统计一次
        self._measured_slots.add(slot)
        axis, delta = self.segment_deltas[slot]
        angles = self._imu_angles(now)
        if angles is None:
            return
        moved = angles[axis] - self._seg_angles_at_start[axis]
        record = self.calibration_report.setdefault(axis, dict(pulses=0, deg=0.0))
        record["pulses"] += abs(delta)
        record["deg"] += moved

    def calibration_summary(self):
        """把累计的「脉冲↔实测角度」换算成脉冲/度；无有效样本返回 None。"""
        out = {}
        for axis, record in self.calibration_report.items():
            if abs(record["deg"]) < 1.0 or record["pulses"] <= 0:
                continue
            out[axis] = abs(record["pulses"] / record["deg"])       # 脉冲/度
        return out or None

    def _imu_angles(self, now):
        """当前相对扫描起点的 yaw/pitch 角度；不可用返回 None。"""
        try:
            state = self.heading.sample(now)
        except Exception:
            return None
        if not state or not state.get("valid"):
            return None
        try:
            return {"yaw": float(state["yaw_deg"]), "pitch": float(state["pitch_deg"])}
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def _mark_segment_start(self, now):
        angles = self._imu_angles(now) if self.imu_active else None
        self._seg_angles_at_start = angles or {"yaw": 0.0, "pitch": 0.0}

    def _imu_angle_reached(self, now):
        """本段是否已转到目标角度（IMU 实测），用于提前收段。"""
        slot = self.index - 1
        if not 0 <= slot < len(self.segment_angles):
            return False
        axis, target_angle = self.segment_angles[slot]
        if abs(target_angle) <= 0:
            return False                                          # 回零段不靠角度收，走脉冲时长
        margin = float(getattr(self.cfg, "SEARCH_IMU_MARGIN_DEG", 3.0))
        angles = self._imu_angles(now)
        if angles is None:
            return False
        moved = angles[axis] - self._seg_angles_at_start[axis]
        # 目标角度的符号 = 期望转向；只看同向推进量，避免反向抖动误判到点。
        return moved * target_angle > 0 and abs(moved) >= abs(target_angle) - margin

    def _target_is_fresh(self, result, now):
        if not result or not result.get("valid") or result.get("held") or result.get("predicted"):
            return False
        try:
            captured = float(result["captured_at"])
            coordinates = (float(result["u"]), float(result["v"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            return False
        if (not all(math.isfinite(value) for value in (captured,) + coordinates) or
                captured < self.started_at or captured <= self._last_capture or
                not 0 <= now - captured <= self.max_age_s):
            return False
        self._last_capture = captured
        return True

    def _finish(self, state, reason, now):
        self._measure_segment(now)
        summary = self.calibration_summary()
        if summary and not self._reported_calibration:
            self._reported_calibration = True
            print("[ui] SEARCH 实测脉冲/度：%s（可直接填回 config 的 %s_PULSES_PER_DEG）"
                  % ("、".join("%s≈%.2f" % (axis.upper(), value)
                              for axis, value in sorted(summary.items())),
                     "/".join(axis.upper() for axis in sorted(summary))))
        moving = self.active_axis is not None and now < self.deadline
        self.position_known = self.position_known and not moving
        self.stop_attempted_at = now
        try:
            accepted = self.link.stop(now=now)
        except Exception:
            accepted = False
        self.active_axis = None
        if accepted is False:
            self.state, self.reason = "FAULT", "stop_failed:%s" % reason
            self.position_known = False
        else:
            self.state, self.reason, self.stopped_at = state, reason, now
        return self.state

    def cancel(self, now):
        """按钮再次点击取消；禁止按已请求端点倒算回起点。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            now = self._last_now
        if not math.isfinite(now):
            now = self._last_now
        now = max(now, self._last_now)
        return self._finish("CANCELLED", "cancelled", now)

    def fail(self, reason, now):
        """控制器发现链路故障时调用；失败后的 update 不会再下发运动。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            now = self._last_now
        if not math.isfinite(now):
            now = self._last_now
        now = max(now, self._last_now)
        return self._finish("FAULT", str(reason), now)

    def update(self, now, result=None):
        """最多发一段位置指令；视觉识别在运动未完成时也能立即停车。"""
        if self.done:
            return self.state
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            return self.fail("invalid_timestamp", self._last_now)
        if not math.isfinite(now) or now < self._last_now:
            return self.fail("invalid_timestamp", self._last_now)
        self._last_now = now
        fault = getattr(self.link, "fault_reason", None)
        if fault:
            return self.fail("link:%s" % fault, now)
        if self._target_is_fresh(result, now):
            return self._finish("ACQUIRED", "black_frame_found", now)
        if now >= self.timeout_at:
            return self._finish("DONE", "timeout", now)
        if now < self.deadline:
            if not (self.imu_active and self._imu_angle_reached(now)):
                return self.state
            # ★ IMU 实测角度已到目标 → 立刻收段：比"估算脉冲时长"更准，且**不依赖脉冲/度标定**。
            if self.link.stop(axis=self.active_axis, now=now) is False:
                self.position_known = False
                return self.fail("stop_failed:imu_angle", now)
            self.deadline = now + self.settle_s
            self.active_axis = None
            return self.state
        self.active_axis = None
        if self.index >= len(self.sequence):
            return self._finish("DONE", "sweep_complete", now)
        axis, delta = self.sequence[self.index]
        pulses = abs(delta)
        positive_ccw = bool(getattr(self.cfg, axis.upper() + "_POSITIVE_CCW", True))
        ccw = positive_ccw if delta > 0 else not positive_ccw
        try:
            duration = estimate_move_seconds(pulses, self.rpm, self.acc, self.per_rev)
            accepted = self.link.move(axis, ccw, pulses, self.rpm, self.acc, now=now)
        except Exception:
            accepted = False
        if accepted is False:
            self.position_known = False
            return self.fail("uart_write_failed:%s" % axis, now)
        self.commanded_pulses[axis] += delta
        self.active_axis = axis
        self.deadline = now + duration + self.settle_s
        self._measure_segment(now)                  # 先结算刚结束的那一段（IMU 实测）
        self._mark_segment_start(now)               # IMU 角度基线（本段起点）
        self.index += 1
        return self.state
'''

# ===== vision.imu_heading  (vision/imu_heading.py) =====
_MOD_SRC['vision.imu_heading'] = r'''
"""Relative inertial heading for the B track, with no UART ownership.

MaixCAM2's onboard IMU is LSM6DSOWTR, exposed through ``maix.ext_dev.imu``.
The adapter below follows Sipeed's documented DUAL/raw API; gyro units are deg/s.
Sources checked at MaixPy a36c0f0 / MaixCDK 30f4b8b:
https://github.com/sipeed/MaixPy/blob/a36c0f0a85ba8b1a4f7eee7ab74af13afcd89bb0/examples/ext_dev/sensors/imu/imu_read_data.py
https://github.com/sipeed/MaixCDK/blob/30f4b8b7e3f66ded9cfa64fb081b46e30bc27248/components/ext_devs/ext_dev/src/lsm6dsowtr/maix_lsm6dsowtr.cpp

Gyros provide short-term *relative* orientation, not an absolute yaw reference.
The initial target lock is the reference; bias, mounting axes and motor degrees
must be calibrated separately. No magnetic heading or drift-free claim is made.
Camera mounting measures body + actual gimbal motion, so subtracting commanded
gimbal motion again would be incorrect. Base mounting instead needs a measured
gimbal position. Degrees returned here are never sent directly to a motor.
"""
import math
import threading
import time
from collections import deque


def _finite_vector(value):
    values = tuple(float(x) for x in value)
    if len(values) != 3 or not all(math.isfinite(x) for x in values):
        raise ValueError("expected three finite numbers")
    return values


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w * v - x * i - y * j - z * k,
            w * i + x * v + y * k - z * j,
            w * j - x * k + y * v + z * i,
            w * k + x * j - y * i + z * v)


def _advance(q, rates, dt):
    """Integrate body-frame angular velocity using a normalized quaternion."""
    omega = tuple(math.radians(x) for x in rates)
    speed = math.sqrt(_dot(omega, omega))
    if speed < 1e-12 or dt <= 0:
        return q
    half = speed * dt * 0.5
    scale = math.sin(half) / speed
    result = _multiply(q, (math.cos(half), *(x * scale for x in omega)))
    length = math.sqrt(_dot(result, result))
    return tuple(x / length for x in result)


def _angles(q):
    """Forward/left/up frame: yaw CCW left and pitch up are positive."""
    w, x, y, z = q
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    pitch = -math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    return yaw, pitch


def _wrapped_delta(value):
    return (value + 180.0) % 360.0 - 180.0


def _offset_quaternion(yaw, pitch):
    yaw, pitch = math.radians(yaw) / 2, -math.radians(pitch) / 2
    return _multiply((math.cos(yaw), 0.0, 0.0, math.sin(yaw)),
                     (math.cos(pitch), 0.0, math.sin(pitch), 0.0))


class MaixIMUSensor:
    """Official MaixPy adapter. Construct/read only in the sensor worker.

    LSM6DSOWTR returns seven zeros when data is not ready; those arrays must
    never replace a fresh moving sample or be integrated as a measured stop.
    The generic header's accel-unit description differs from this board's
    implementation, so this module deliberately does not use accel units.
    """

    def __init__(self):
        from maix.ext_dev import imu
        self.device = imu.IMU("default", mode=imu.Mode.DUAL,
                              acc_scale=imu.AccScale.ACC_SCALE_2G,
                              acc_odr=imu.AccOdr.ACC_ODR_208,
                              gyro_scale=imu.GyroScale.GYRO_SCALE_500DPS,
                              gyro_odr=imu.GyroOdr.GYRO_ODR_208, block=False)
        self.bias_dps = None
        if self.device.calib_gyro_exists():
            bias = self.device.load_calib_gyro()
            self.bias_dps = _finite_vector((bias.x, bias.y, bias.z))

    def read(self):
        raw = self.device.read()
        if not raw:
            return None
        if len(raw) != 7:
            raise ValueError("Maix IMU DUAL read() must return seven values")
        if all(float(x) == 0.0 for x in raw):
            return None
        return _finite_vector(raw[3:6])


class GyroHeading:
    """Pure deterministic estimator. Call only from one thread or under a lock.

    ``update`` takes raw gyro xyz in deg/s and a monotonic sample timestamp.
    Configured unit vectors express physical yaw-left/pitch-up in sensor axes.
    They must be perpendicular and verified on the mounted hardware; defaults
    are mathematical examples, not a claimed MaixCAM2 board mounting.
    Initialize in the calibrated level/home pose. Gyros alone cannot identify
    gravity or infer an unknown initial tilt. Quaternion integration accounts
    for subsequent camera tilt and roll without treating sensor z as world z.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.enabled = bool(getattr(cfg, "ENABLE_IMU_FEEDFORWARD", False))
        self.mount = str(getattr(cfg, "IMU_SENSOR_MOUNT", "camera")).lower()
        if self.mount not in ("camera", "base"):
            raise ValueError("IMU_SENSOR_MOUNT must be camera or base")
        self.yaw_axis = _finite_vector(getattr(cfg, "IMU_YAW_AXIS", (0, 0, 1)))
        up_axis = _finite_vector(getattr(cfg, "IMU_PITCH_AXIS", (0, -1, 0)))
        if (abs(_dot(self.yaw_axis, self.yaw_axis) - 1) > 1e-6 or
                abs(_dot(up_axis, up_axis) - 1) > 1e-6 or
                abs(_dot(self.yaw_axis, up_axis)) > 1e-6):
            raise ValueError("IMU yaw/pitch axes must be orthogonal unit vectors")
        self.down_axis = tuple(-x for x in up_axis)
        self.roll_axis = _cross(self.down_axis, self.yaw_axis)
        self.axes_calibrated = bool(getattr(cfg, "IMU_AXES_CALIBRATED", False))
        self.max_age = max(0.001, float(getattr(cfg, "IMU_MAX_AGE_S", 0.10)))
        self.max_gap = max(self.max_age, float(getattr(cfg, "IMU_MAX_GAP_S", 0.20)))
        self.max_predict = max(0.0, float(getattr(cfg, "IMU_MAX_PREDICT_S", 0.05)))
        self.max_rate = max(1.0, float(getattr(cfg, "IMU_MAX_RATE_DPS", 450.0)))
        self.max_pitch = max(1.0, min(89.0, float(getattr(cfg, "IMU_MAX_PITCH_DEG", 80.0))))
        self.reference_max_age = max(0.001, float(getattr(cfg, "IMU_REFERENCE_MAX_AGE_S", 0.03)))
        self.history_s = max(self.reference_max_age, min(2.0, float(getattr(cfg, "IMU_HISTORY_S", 1.0))))
        history_size = max(2, min(512, int(getattr(cfg, "IMU_HISTORY_MAX_SAMPLES", 256))))
        self.history = deque(maxlen=history_size)
        self.bias = None
        configured_bias = getattr(cfg, "IMU_BIAS_DPS", None)
        if configured_bias is not None:
            self.bias = _finite_vector(configured_bias)
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.last_time = None
        self.rates = (0.0, 0.0, 0.0)
        self.samples = 0
        self.error = None
        self.reference = None
        self.reference_time = None
        self.reference_gimbal = None
        self.yaw_delta = 0.0
        self.pitch_delta = 0.0
        self._last_wrapped_yaw = 0.0
        self._bias_window = None
        self.calibration_state = "IDLE"

    def reset_reference(self):
        self.reference = None
        self.reference_time = None
        self.reference_gimbal = None
        self.yaw_delta = self.pitch_delta = self._last_wrapped_yaw = 0.0

    def set_bias(self, bias_dps):
        self.bias = _finite_vector(bias_dps)
        self.last_time = None
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.history.clear()
        self.reset_reference()
        self.error = None

    def start_bias_calibration(self, now):
        """Explicit stationary calibration; never infer stillness during a turn.

        Hardware must be held still by the caller. Dispersion and a small rate
        threshold reject obvious movement, but cannot detect all slow rotation.
        The returned bias can be saved through application configuration.
        """
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("calibration requires a finite monotonic time")
        self.bias = None
        self.last_time = None
        self.error = None
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.history.clear()
        self.reset_reference()
        self._bias_window = dict(start=now, count=0, sums=[0.0] * 3,
                                 squares=[0.0] * 3)
        self.calibration_state = "COLLECTING"

    def _calibrate(self, raw, now):
        window = self._bias_window
        window["count"] += 1
        for index, value in enumerate(raw):
            window["sums"][index] += value
            window["squares"][index] += value * value
        duration = float(getattr(self.cfg, "IMU_BIAS_CALIBRATE_S", 1.0))
        if now - window["start"] < max(0.1, duration):
            return
        count = window["count"]
        mean = tuple(value / count for value in window["sums"])
        deviation = max(math.sqrt(max(0.0, window["squares"][i] / count - mean[i] ** 2))
                        for i in range(3))
        max_std = float(getattr(self.cfg, "IMU_BIAS_MAX_STD_DPS", 0.25))
        max_bias = float(getattr(self.cfg, "IMU_BIAS_MAX_DPS", 2.0))
        self._bias_window = None
        if count < 10 or deviation > max_std or max(abs(x) for x in mean) > max_bias:
            self.calibration_state = "REJECTED"
            self.error = "bias_calibration_requires_stationary_sensor"
            return
        self.set_bias(mean)
        self.calibration_state = "READY"

    def update(self, gyro_dps, now):
        now = float(now)
        try:
            raw = _finite_vector(gyro_dps)
            if not math.isfinite(now):
                raise ValueError("nonfinite_timestamp")
            if max(abs(x) for x in raw) > self.max_rate:
                raise ValueError("gyro_rate_out_of_range")
        except (ValueError, TypeError, OverflowError) as exc:
            self.fail(str(exc))
            return self.snapshot(now)
        if self._bias_window is not None:
            self._calibrate(raw, now)
            return self.snapshot(now)
        if self.bias is None:
            self.error = None
            return self.snapshot(now)
        corrected = tuple(x - b for x, b in zip(raw, self.bias))
        rates = (_dot(corrected, self.roll_axis), _dot(corrected, self.down_axis),
                 _dot(corrected, self.yaw_axis))
        if self.last_time is not None:
            dt = now - self.last_time
            if dt <= 0 or dt > self.max_gap:
                # Motion during a sensor outage is unknown. Discard the reference
                # instead of inventing rotation across the missing interval.
                self.q = (1.0, 0.0, 0.0, 0.0)
                self.reset_reference()
                self.history.clear()
                self.error = "sample_gap" if dt > 0 else "nonmonotonic_sample"
                self.last_time, self.rates = now, rates
                return self.snapshot(now)
            average = tuple(0.5 * (a + b) for a, b in zip(self.rates, rates))
            self.q = _advance(self.q, average, dt)
        self.last_time, self.rates = now, rates
        self.error = None
        self.samples += 1
        self.history.append((now, self.q, rates))
        while self.history and now - self.history[0][0] > self.history_s:
            self.history.popleft()
        if self.reference is not None:
            current_yaw, current_pitch = _angles(self.q)
            reference_yaw, reference_pitch = _angles(self.reference)
            yaw = _wrapped_delta(current_yaw - reference_yaw)
            self.yaw_delta += _wrapped_delta(yaw - self._last_wrapped_yaw)
            self._last_wrapped_yaw = yaw
            self.pitch_delta = current_pitch - reference_pitch
        return self.snapshot(now)

    def fail(self, error):
        self.error = str(error)
        self.last_time = None
        if self._bias_window is not None:
            self._bias_window = None
            self.calibration_state = "REJECTED"
        self.reset_reference()
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.history.clear()

    def snapshot(self, now):
        now = float(now)
        age = None if self.last_time is None else now - self.last_time
        fresh = age is not None and 0 <= age <= self.max_age
        current_yaw, current_pitch = _angles(self.q)
        if not self.enabled:
            state = "DISABLED"
        elif self.error:
            state = "ERROR"
        elif self._bias_window is not None:
            state = "CALIBRATING"
        elif not self.axes_calibrated:
            state = "AXES_UNCALIBRATED"
        elif self.bias is None:
            state = "BIAS_UNCALIBRATED"
        elif not fresh:
            state = "STALE"
        elif abs(current_pitch) > self.max_pitch:
            state = "ATTITUDE_OUT_OF_RANGE"
        else:
            state = "LOCKED" if self.reference is not None else "READY"
        # Sensor/body rates are not world yaw/pitch rates after a camera tilt.
        # A short quaternion derivative gives the common-frame Euler rates.
        next_yaw, next_pitch = _angles(_advance(self.q, self.rates, 0.0001))
        yaw_rate = _wrapped_delta(next_yaw - current_yaw) / 0.0001
        pitch_rate = (next_pitch - current_pitch) / 0.0001
        return dict(valid=state in ("READY", "LOCKED"), state=state,
                    mount=self.mount, age_s=age, sampled_at=self.last_time,
                    samples=self.samples, bias_dps=self.bias, error=self.error,
                    reference_locked=self.reference is not None,
                    reference_at=self.reference_time, yaw_deg=self.yaw_delta,
                    pitch_deg=self.pitch_delta, rate_yaw_dps=yaw_rate,
                    rate_pitch_dps=pitch_rate, calibration=self.calibration_state)

    def sample_at(self, captured_at):
        """Find a close sample at/before capture; never use a future sample.

        This is a bounded history, not an IMU IO operation. A short interpolation
        with the previous sample's angular rate synchronizes the returned pose.
        ``valid`` only describes capture-time alignment; consumers must also
        check the current sensor's freshness before issuing a motor command.
        """
        captured_at = float(captured_at)
        for sampled_at, q, rates in reversed(self.history):
            if sampled_at <= captured_at:
                age = captured_at - sampled_at
                valid = math.isfinite(captured_at) and age <= self.reference_max_age
                return dict(valid=valid, sampled_at=sampled_at, captured_at=captured_at,
                            age_s=age, quaternion=_advance(q, rates, age) if valid else q,
                            rates_dps=rates, state="ALIGNED" if valid else "HISTORY_STALE")
        return dict(valid=False, sampled_at=None, captured_at=captured_at,
                    age_s=None, quaternion=None, rates_dps=None, state="NO_HISTORY")

    def lock_reference(self, now, yaw_offset_deg=0.0, pitch_offset_deg=0.0,
                       gimbal_deg=None):
        """Lock a capture-aligned target direction with physical angle offsets.

        ``now`` is the visual frame's capture time in the same monotonic clock.
        Yaw offsets are positive CCW-left; pitch offsets positive up. Root must
        convert visual pixels using measured signed pixel/degree calibration.
        """
        if self.last_time is None or not self.snapshot(self.last_time)["valid"]:
            return False
        aligned = self.sample_at(now)
        if not aligned["valid"]:
            return False
        try:
            offsets = (float(yaw_offset_deg), float(pitch_offset_deg))
            if not all(math.isfinite(x) for x in offsets) or max(abs(x) for x in offsets) > 90:
                return False
        except (TypeError, ValueError, OverflowError):
            return False
        measured = self._gimbal(gimbal_deg)
        if self.mount == "base" and measured is None:
            return False
        self.reference = _multiply(aligned["quaternion"], _offset_quaternion(*offsets))
        self.reference_time = float(now)
        self.reference_gimbal = measured
        current_yaw, current_pitch = _angles(self.q)
        reference_yaw, reference_pitch = _angles(self.reference)
        yaw = _wrapped_delta(current_yaw - reference_yaw)
        self.yaw_delta = self._last_wrapped_yaw = yaw
        self.pitch_delta = current_pitch - reference_pitch
        return True

    @staticmethod
    def _gimbal(value):
        if value is None:
            return None
        try:
            result = (float(value["yaw"]), float(value["pitch"])) if isinstance(value, dict) else tuple(float(x) for x in value)
            if len(result) != 2 or not all(math.isfinite(x) for x in result):
                return None
            return result
        except (KeyError, ValueError, TypeError, OverflowError):
            return None

    def correction(self, now, gimbal_deg=None, lead_s=0.0):
        """Remaining heading error, positive motor CCW-left / pitch-up.

        Caller should use the absolute error once, or use a sample-time delta
        after a visual result; adding the full error to visual feedback doubles
        the already-observed rotation. Neither motor limits nor pulse conversion
        belong here. ``base`` needs actual measured relative gimbal angles.
        """
        result = self.snapshot(now)
        result.update(yaw_deg=0.0, pitch_deg=0.0)
        if not result["valid"] or self.reference is None:
            result["valid"] = False
            if result["state"] == "READY":
                result["state"] = "NO_REFERENCE"
            return result
        age = max(0.0, float(now) - self.last_time)
        prediction = min(self.max_predict, age + max(0.0, float(lead_s)))
        predicted = _advance(self.q, self.rates, prediction)
        current_yaw, _ = _angles(self.q)
        predicted_yaw, predicted_pitch = _angles(predicted)
        yaw = self.yaw_delta + _wrapped_delta(predicted_yaw - current_yaw)
        pitch = predicted_pitch - _angles(self.reference)[1]
        # Camera sensor already observes the completed portion of any motor
        # movement; never subtract that motion a second time.
        if self.mount == "base":
            measured = self._gimbal(gimbal_deg)
            if measured is None:
                result.update(valid=False, state="BASE_REQUIRES_GIMBAL_POSITION")
                return result
            yaw += measured[0] - self.reference_gimbal[0]
            pitch += measured[1] - self.reference_gimbal[1]
        result.update(yaw_deg=-yaw, pitch_deg=-pitch, prediction_s=prediction)
        return result


class HeadingFeedforward:
    """One daemon sensor worker, one latest state, nonblocking controller reads.

    The worker integrates each IMU sample even when vision drops frames. It does
    no UART operations. No sensor IO occurs while holding the state mutex, so a
    slow or stuck sensor cannot hold the controller's lock. Native driver's GIL
    behavior/IO timeout still requires board verification. Shutdown has a bounded
    join and never destroys an in-flight native device from another thread.

    Inject a sensor whose ``read()`` returns gyro xyz deg/s or None. Tests may set
    ``threaded=False`` and use ``ingest`` with a deterministic monotonic clock.
    """

    def __init__(self, cfg, sensor=None, clock=None, threaded=True):
        self.cfg = cfg
        self._clock = clock if clock is not None else time
        self._sensor = sensor
        self._estimator = GyroHeading(cfg)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._closed = False
        self._startup_calibration = bool(getattr(cfg, "IMU_CALIBRATE_ON_START", False))
        # Device construction may be slow. Reserve the stationary calibration
        # phase before starting the worker, so controllers cannot move motors
        # during initialization and then contaminate a later bias collection.
        self._startup_calibration_pending = (self._estimator.enabled and
                                             self._startup_calibration and
                                             self._estimator.bias is None and threaded)
        if (self._estimator.enabled and self._startup_calibration and
                self._estimator.bias is None and not threaded):
            # Manual ingestion has no device-constructor phase to resolve.
            self._estimator.start_bias_calibration(self._now())
        if self._estimator.enabled and threaded:
            self._thread = threading.Thread(target=self._run, name="imu-heading", daemon=True)
            self._thread.start()

    def _now(self):
        return float(self._clock.perf_counter())

    @property
    def worker_alive(self):
        """阻塞硬件读未退出时，应用不得另开同一传感器实例。"""
        return self._thread is not None and self._thread.is_alive()

    def _run(self):
        try:
            sensor = self._sensor if self._sensor is not None else MaixIMUSensor()
            self._sensor = sensor
            with self._lock:
                if self._estimator.bias is None:
                    supplied_bias = getattr(sensor, "bias_dps", None)
                    if supplied_bias is not None:
                        self._estimator.set_bias(supplied_bias)
                if self._estimator.bias is None and self._startup_calibration:
                    self._estimator.start_bias_calibration(self._now())
                self._startup_calibration_pending = False
        except Exception as exc:
            with self._lock:
                self._estimator.fail("imu_unavailable: %s" % exc)
                self._startup_calibration_pending = False
            return
        hz = max(1.0, min(400.0, float(getattr(self.cfg, "IMU_POLL_HZ", 200.0))))
        while not self._stop.is_set():
            started = self._now()
            try:
                gyro = sensor.read()               # IO intentionally outside lock
                received = self._now()
                if gyro is not None:
                    with self._lock:
                        max_read = max(0.001, float(getattr(self.cfg, "IMU_READ_MAX_S", self._estimator.max_age)))
                        if received - started > max_read or received < started:
                            self._estimator.fail("imu_read_timeout")
                        elif not self._stop.is_set():
                            self._estimator.update(gyro, received)
            except Exception as exc:
                with self._lock:
                    self._estimator.fail("imu_read_failed: %s" % exc)
            # One bounded poll per iteration, no queue or catch-up bursts.
            self._stop.wait(max(0.001, 1.0 / hz - (self._now() - started)))

    def ingest(self, gyro_dps, now=None):
        """Deterministic injected samples; incompatible with an active worker."""
        if self._thread is not None:
            raise RuntimeError("ingest requires threaded=False")
        with self._lock:
            return self._estimator.update(gyro_dps, self._now() if now is None else now)

    def sample(self, now=None):
        with self._lock:
            result = self._estimator.snapshot(self._now() if now is None else now)
            if self._startup_calibration_pending:
                result.update(valid=False, state="CALIBRATING", calibration="PENDING_SENSOR")
            if self._closed:
                result.update(valid=False, state="CLOSED")
            return result

    def sample_at(self, captured_at):
        with self._lock:
            result = self._estimator.sample_at(captured_at)
            if self._closed:
                result.update(valid=False, state="CLOSED")
            return result

    def lock_reference(self, now=None, yaw_offset_deg=0.0, pitch_offset_deg=0.0,
                       gimbal_deg=None):
        with self._lock:
            if self._closed or not self._estimator.snapshot(self._now())["valid"]:
                return False
            return self._estimator.lock_reference(
                self._now() if now is None else now, yaw_offset_deg, pitch_offset_deg, gimbal_deg)

    def reset_reference(self):
        with self._lock:
            self._estimator.reset_reference()

    def start_bias_calibration(self, now=None):
        with self._lock:
            self._estimator.start_bias_calibration(self._now() if now is None else now)
            self._startup_calibration_pending = False

    def correction(self, now=None, gimbal_deg=None, lead_s=0.0):
        with self._lock:
            result = self._estimator.correction(self._now() if now is None else now,
                                                gimbal_deg, lead_s)
            if self._startup_calibration_pending:
                result.update(valid=False, state="CALIBRATING", calibration="PENDING_SENSOR",
                              yaw_deg=0.0, pitch_deg=0.0)
            if self._closed:
                result.update(valid=False, state="CLOSED", yaw_deg=0.0, pitch_deg=0.0)
            return result

    def close(self):
        self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.10)
'''

# ===== vision.aim_control  (vision/aim_control.py) =====
_MOD_SRC['vision.aim_control'] = r'''
"""闭环调度中枢 + 旧版 ASCII 串口输出（从 main.py 搬迁，v0.4.6 模块化）。

VisionControl 把"识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层"整条链放进**识别线程**，
因为它独占 UART（同一轴收发必须串行）。主线程要触发自检只置 request_test 事件。
★ 调度优先级顺序不可调换（见 B 轨 AGENTS.md §3.2）：FAULT 必须最前。
★ clock 注入：构造可传 `clock`（默认标准库 time），使假时钟可穿透（见 vision_worker.py 说明）。
"""
import threading
import time
import math

from .gimbal_search import SearchSweep
from .gimbal_self_test import GimbalSelfTest
from .imu_heading import HeadingFeedforward
from .laser_spot import LaserSpotDetector, deviation
from .rectangle_center import RectangleCenterDetector


def send_uart(uart, result):
    """旧版纯视觉 ASCII 输出（ENABLE_GIMBAL=False 的降级路径才用；开云台时 uart 为 None）。"""
    if uart is None:
        return
    if result.get("valid"):
        payload = "@[%d,%d]%%\r\n" % (int(round(result["u"])), int(round(result["v"])))
    else:
        payload = "@[invalid]%%\r\n"
    uart.write(payload.encode("ascii"))


class VisionControl:
    """识别 + 电机状态由**识别线程独占**；主循环只提交图像、按钮事件、读结果。

    独占理由：UART 读写不可并发（两条轴各有独立串口，但同一轴的收发必须串行），
    把"识别 → 偏差 → 闭环下发 → 收 ACK"全放进识别线程，天然避免与主线程抢串口。
    主线程要触发自检只置 `request_test` 事件，由识别线程在自己的上下文里安全地启动。
    """

    def __init__(self, cfg, link, aim, calibrator, uart, clock=None, heading=None):
        self.cfg, self.link, self.aim = cfg, link, aim
        self.calibrator, self.uart = calibrator, uart
        self._clock = clock if clock is not None else time
        self.self_test = None
        self.request_test = threading.Event()   # 主线程置位、识别线程消费（按钮 → 自检）
        self.request_search = threading.Event()
        self.request_imu_bias = threading.Event()
        self.search = None
        self.notice = ""
        self.heading = heading
        if self.heading is None and bool(getattr(cfg, "ENABLE_IMU_FEEDFORWARD", False)):
            self.heading = HeadingFeedforward(cfg, clock=self._clock)
        self.last_visual_reference = -math.inf
        self.last_visual_capture = -math.inf
        self.control_source = "vision"
        self.last_commands = {"yaw": None, "pitch": None}
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == "fixed"
        # fixed 模式：光斑位置是常量，开机取一次即可，之后每帧只做减法（零检测开销）。
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True

    def process(self, gray, captured_at):
        """识别线程的单帧主流程：识别 → 光斑 → 偏差 → 串口 → 闭环 → 打包给显示层。"""
        started = self._clock.perf_counter()
        result = self.detector.process(gray, timestamp=captured_at)  # ① 按采图时间预测
        result["captured_at"] = float(captured_at)
        detected = self._clock.perf_counter()
        if self.laser is not None and not self.laser_static:
            self.laser_state = self.laser.process(gray, detected)   # ② detect 模式才实测光斑
        dv = deviation(self.laser_state, result, use_raw=self.cfg.AIM_USE_RAW) if self.laser_state else None
        if dv is not None:
            result["laser_u"], result["laser_v"] = self.laser_state["u"], self.laser_state["v"]
            result["dx"], result["dy"] = dv                 # dx=靶心−光斑（闭环用的那个偏差）
        now = self._clock.perf_counter()
        result["aim_valid"] = dv is not None
        return self.accept_result(result, captured_at, self.laser_state,
                                  started=started, detected=detected)

    def accept_result(self, result, captured_at, laser=None, started=None, detected=None):
        """统一 API/相机的控制入口；result 必须是本帧的真实视觉测量。"""
        now = self._clock.perf_counter()
        started = now if started is None else started
        detected = now if detected is None else detected
        result["captured_at"] = float(captured_at)
        result["age_s"] = max(0.0, now - captured_at)
        timestamp_valid = math.isfinite(float(captured_at)) and 0 <= now - captured_at <= self.cfg.RESULT_MAX_AGE_S
        ordered = captured_at > self.last_capture
        fresh = timestamp_valid and ordered
        result["aim_valid"] = bool(result.get("aim_valid") and fresh)
        if laser is not None:
            self.laser_state = laser
        if fresh:
            self.last_capture = captured_at
            if result.get("aim_valid"):
                self.last_visual_capture = captured_at
        # 过期结果不外发：串口与闭环都用降级后的 output，避免拿旧定位驱动电机。
        output = result if fresh else dict(valid=False, aim_valid=False, stage="stale")
        send_uart(self.uart, output)
        # 重复/倒序图像只触发保活，不能覆盖新参考或按旧误差打断正在进行的动作。
        self._control(output if ordered else None, now, new_measurement=ordered)
        # 返回给显示层的快照：识别结果 + 光斑 + 计时 + 状态串。
        return dict(result=result, laser=dict(self.laser_state) if self.laser_state else None,
                    captured_at=captured_at, completed_at=self._clock.perf_counter(),
                    detect_ms=(detected - started) * 1000, status=self.status())

    def idle(self):
        # 摄像头/图传停顿时继续收 ACK 与推进按钮自检，禁止重复发送旧视觉误差。
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        """给屏幕显示状态：故障优先，其后为自检、标定、搜索和闭环。"""
        if self.aim is not None and self.aim.state == "FAULT":
            return "FAULT"
        if self.self_test is not None and not self.self_test.done:
            return "self-test %d/%d" % (self.self_test.index, len(self.self_test.sequence))
        if self.calibrator is not None and not self.calibrator.done:
            return "calibrate %s" % self.calibrator.axis
        if self.search is not None:
            if not self.search.done:
                return "SEARCH %d/%d" % (self.search.index, len(self.search.sequence))
            if self.search.state in ("UNCALIBRATED", "FAULT"):
                return "SEARCH %s: %s" % (self.search.state, self.search.reason)
        if self.control_source == "imu":
            return "IMU HOLD %s" % self.aim.state
        if self.notice:
            return self.notice
        return self.aim.state if self.aim is not None else "vision"

    def imu_status(self):
        return self.heading.sample(self._clock.perf_counter()) if self.heading is not None else {
            "valid": False, "state": "DISABLED"}

    def stop(self, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        if self.search is not None and not self.search.done:
            self.search.cancel(now)
            self._finish_search()
            accepted = self.search.state != "FAULT"
        else:
            accepted = True if self.link is None else self._stop_motion(now)
        self.stale_stopped = True
        if self.heading is not None:
            self.heading.reset_reference()
        self.last_visual_reference = -math.inf
        return accepted

    def close(self):
        if self.heading is not None:
            self.heading.close()

    def _finish_search(self):
        if self.search.state == "FAULT":
            self.aim.fault("search:%s" % self.search.reason)
        elif self.search.stopped_at is not None:
            self.aim.motion_stopped(now=self.search.stopped_at)
            self.aim.update({"aim_valid": False}, now=self.search.stopped_at)
        self.stale_stopped = True
        self.last_visual_reference = -math.inf
        if self.heading is not None:
            self.heading.reset_reference()

    def _degree_scales(self):
        result = {}
        for axis in ("yaw", "pitch"):
            ppd = float(getattr(self.cfg, axis.upper() + "_PULSES_PER_DEG", 0.0))
            if ppd <= 0:
                ppd = float(getattr(self.cfg, "GIMBAL_GEAR_RATIO", 0.0)) * float(
                    getattr(self.cfg, "MOTOR_PULSES_PER_REV", 3200)) / 360.0
            ppp = float(getattr(self.cfg, "PULSE_PER_PX_" + axis.upper(), 0.0))
            sign = float(getattr(self.cfg, "DIR_SIGN_" + axis.upper(), 0.0))
            if not all(math.isfinite(v) for v in (ppd, ppp, sign)) or min(ppd, ppp) <= 0 or abs(sign) != 1:
                return None
            physical = 1 if bool(getattr(self.cfg, axis.upper() + "_POSITIVE_CCW", True)) else -1
            result[axis] = (ppd, ppp, sign * physical)
        return result

    def _lock_visual_heading(self, result, now):
        if self.heading is None or not result.get("aim_valid"):
            return
        scales = self._degree_scales()
        if scales is None:
            self.notice = "IMU: pulse/degree calibration required"
            return
        offsets = {axis: float(result[key]) * ppp / ppd * sign
                   for axis, key in (("yaw", "dx"), ("pitch", "dy"))
                   for ppd, ppp, sign in (scales[axis],)}
        captured = float(result["captured_at"])
        if (captured <= getattr(self.aim, "capture_after", -math.inf) or
                captured <= self.last_visual_reference):
            return
        if self.heading.lock_reference(captured, yaw_offset_deg=offsets["yaw"],
                                       pitch_offset_deg=offsets["pitch"]):
            self.last_visual_reference = captured

    def _inertial_result(self, now):
        if self.heading is None or now - self.last_visual_reference > float(
                getattr(self.cfg, "IMU_HOLD_MAX_S", 2.0)):
            return None
        scales = self._degree_scales()
        if scales is None:
            return None
        state = self.heading.correction(now, lead_s=float(getattr(self.cfg, "IMU_LEAD_S", 0.03)))
        if not state.get("valid"):
            return None
        angles = (float(state["yaw_deg"]), float(state["pitch_deg"]))
        if not all(math.isfinite(x) for x in angles) or max(abs(x) for x in angles) > float(
                getattr(self.cfg, "IMU_MAX_CORRECTION_DEG", 100.0)):
            return None
        errors = {axis: angle * ppd / ppp * sign
                  for axis, angle in zip(("yaw", "pitch"), angles)
                  for ppd, ppp, sign in (scales[axis],)}
        return dict(aim_valid=True, valid=False, source="imu", dx=errors["yaw"],
                    dy=errors["pitch"], captured_at=state["sampled_at"])

    def _dispatch(self, result, now):
        output = self.aim.update(result, now=now)
        for axis in output.get("stop_axes", ()):
            if not self._stop_motion(now, axis=axis, reset=False):
                return
        for axis, command in output["cmds"].items():
            if command is not None:
                ccw, pulses, rpm = command
                acc = getattr(self.cfg, "IMU_ACC", 80) if result.get("source") == "imu" else self.cfg.ACC_TRACK
                accepted = self.link.move(axis, ccw, pulses, rpm, acc, now=now)
                if accepted is False:
                    self._stop_motion(now)
                    self.aim.fault("tx:%s" % axis)
                    self.last_commands = {"yaw": None, "pitch": None}
                    return
                self.last_commands[axis] = command

    def _stop_motion(self, now, axis=None, reset=True):
        """停机帧未写成功时保持 FAULT，不能按已停机继续追加动作。"""
        accepted = self.link.stop(axis=axis, now=now)
        if accepted is False:
            if axis is not None:
                self.link.stop(now=now)                    # 尝试停止另一轴，随后阻断所有动作
            self.aim.fault("stop:%s" % (axis or "all"))
            return False
        if reset:
            self.aim.motion_stopped(axis, now=now)
        return True

    def _control(self, result, now, new_measurement):
        """闭环调度中枢：故障检查 → 按钮自检 → 标定 → 自检 → 失锁停机 → 追踪下发。

        优先级顺序**不可随意调换**（见 B 轨 AGENTS.md §3.2）；尤其 FAULT 必须在最前，
        因为链路故障要阻断的不只是追踪，还包括已启动的自检/标定。
        """
        link, aim = self.link, self.aim
        self.last_commands = {"yaw": None, "pitch": None}
        if link is None or aim is None:
            if self.request_search.is_set():
                self.request_search.clear()
                self.notice = "SEARCH unavailable: gimbal disabled"
                print("[ui] SEARCH 不可用：云台未启用（ENABLE_GIMBAL=False 或初始化失败）")
            if self.request_test.is_set():
                self.request_test.clear()
                print("[ui] 自检不可用：云台未启用（ENABLE_GIMBAL=False 或初始化失败）")
            return
        feedback = link.poll(now)                            # 非阻塞收两条串口的返回帧
        if feedback and bool(feedback.get("fault")) and aim.state != "FAULT":
            reason = feedback.get("reason")
            self._stop_motion(now)
            aim.fault("link:%s" % reason)                    # 链路故障 → 进入 FAULT
            if self.search is not None and not self.search.done:
                self.search.fail(reason, now)
        imu_calibrating = self.heading is not None and self.heading.sample(now).get("state") == "CALIBRATING"
        if self.request_test.is_set():                       # 屏幕按钮置位的自检请求
            self.request_test.clear()
            if self.self_test is not None and not self.self_test.done:
                print("[ui] 自检进行中，忽略重复点击")
            elif aim.state == "FAULT":
                print("[ui] 链路 FAULT，拒绝自检")
            elif self.calibrator is not None and not self.calibrator.done:
                print("[ui] 标定进行中，拒绝自检")
            elif self.search is not None and not self.search.done:
                print("[ui] 寻靶进行中，先取消 SEARCH 再自检")
            elif imu_calibrating:
                print("[ui] IMU 偏置标定中，拒绝自检")
            elif not self.cfg.GIMBAL_SELF_TEST:
                print("[ui] GIMBAL_SELF_TEST=False，忽略自检")
            else:
                if not self._stop_motion(now):              # 先停当前动作再起自检
                    return
                self.self_test = GimbalSelfTest(self.cfg, link, now)
                print("[ui] 收到点击 -> 启动上电自检（%d 步）" % len(self.self_test.sequence))
        if self.request_search.is_set():
            self.request_search.clear()
            # 每个分支都必须留下日志：按钮是"按下沿"触发，若启动/取消静默，现场只看到
            # "按了没反应"，无法区分"被拒绝"与"已生效"。
            if aim.state == "FAULT":
                self.notice = "SEARCH rejected: FAULT"
                print("[ui] SEARCH 被拒绝：链路 FAULT")
            elif self.calibrator is not None and not self.calibrator.done:
                self.notice = "SEARCH rejected: calibrating"
                print("[ui] SEARCH 被拒绝：正在标定")
            elif self.self_test is not None and not self.self_test.done:
                self.notice = "SEARCH rejected: self-test"
                print("[ui] SEARCH 被拒绝：自检进行中")
            elif imu_calibrating:
                self.notice = "SEARCH rejected: IMU calibrating"
                print("[ui] SEARCH 被拒绝：IMU 偏置标定中")
            elif self.search is not None and not self.search.done:
                self.search.cancel(now)
                self._finish_search()
                print("[ui] SEARCH 已取消（state=%s）" % self.search.state)
                return
            elif new_measurement and result and result.get("valid"):
                self.notice = "SEARCH: target already visible"
                print("[ui] SEARCH 未启动：当前画面已检出靶纸")
            elif self._stop_motion(now):
                self.notice = ""
                self.search = SearchSweep(self.cfg, link, now, heading=self.heading)
                # 把"这次到底靠什么在扫"打出来：未标定脉冲/度时靠固定脉冲兜底，
                # IMU 可用时按实测角度收段 —— 现场据此判断是否需要标定。
                print("[ui] SEARCH 已启动（%d 步，state=%s，收段=%s，脉冲/度=%s）"
                      % (len(self.search.sequence), self.search.state,
                         self.search.imu_note,
                         "已标定" if any(self.search.pulses_per_degree.values())
                         else "未标定→固定脉冲兜底"))
                if self.search.done:
                    self._finish_search()
        if self.request_imu_bias.is_set():
            self.request_imu_bias.clear()
            if (self.heading is not None and aim.state != "FAULT" and
                    not (self.search is not None and not self.search.done) and
                    not (self.self_test is not None and not self.self_test.done) and
                    not (self.calibrator is not None and not self.calibrator.done) and
                    self._stop_motion(now)):
                self.heading.start_bias_calibration(now)
                self.notice = "IMU bias: keep vehicle and gimbal stationary"
        # 链路故障必须阻断已启动的自检/标定，不只阻断普通追踪。
        if aim.state == "FAULT":
            return
        if self.heading is not None and self.heading.sample(now).get("state") == "CALIBRATING":
            return
        if self.calibrator is not None and not self.calibrator.done:
            self.calibrator.update(now, result or {"valid": False, "aim_valid": False})
            if self.calibrator.state == "FAULT":
                self._stop_motion(now)
                aim.fault("calibration:%s" % self.calibrator.reason)
            return
        if self.self_test is not None and not self.self_test.done:
            self.self_test.update(now)
            if self.self_test.state == "FAULT":
                self._stop_motion(now)
                aim.fault("self-test:%s" % self.self_test.reason)
            return
        if self.search is not None and not self.search.done:
            self.control_source = "search"
            self.search.update(now, result if new_measurement else None)
            if self.search.done:
                self._finish_search()
            return
        previous_source = self.control_source
        self.control_source = "vision"
        if new_measurement and result and result.get("aim_valid"):
            self._lock_visual_heading(result, now)
        elif not new_measurement and now - self.last_visual_capture < float(getattr(self.cfg, "IMU_VISUAL_GAP_S", 0.05)):
            return
        else:
            inertial = self._inertial_result(now)
            if inertial is not None:
                self.control_source = "imu"
                self.stale_stopped = False
                self._dispatch(inertial, now)
                return
            if previous_source == "imu":
                if not self.stale_stopped:
                    self._stop_motion(now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
                return
        if not new_measurement:
            # idle 路径：没有新视觉误差。若已超时且尚未停机，则停一次并置 aim 无效（防"漂"）。
            if now - self.last_capture > self.cfg.RESULT_MAX_AGE_S and not self.stale_stopped:
                self._stop_motion(now)
                aim.update({"aim_valid": False}, now=now)
                self.stale_stopped = True
            return
        if not result.get("aim_valid"):
            if not self.stale_stopped:                       # 视觉失效：停一次电机，不回零
                self._stop_motion(now)
            self.stale_stopped = True
        else:
            self.stale_stopped = False
        self._dispatch(result, now)
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
        self.started = float(now)
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
        if now < self.deadline:
            return self.state
        dv = None
        if result and result.get("aim_valid") and result.get("dx") is not None:
            dv = (float(result["dx"]), float(result["dy"]))
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
        travel = self.pulses / (ppr * rpm / 60.0)
        settle = float(getattr(self.cfg, "CALIBRATE_SETTLE_S", 1.0))
        return travel * 1.4 + settle

    def _send(self, ccw):
        ok = self.link.move(self.axis, ccw, self.pulses,
                            int(getattr(self.cfg, "CALIBRATE_RPM", 300)),
                            int(getattr(self.cfg, "ACC_DEFAULT", 0)))
        if not ok:
            self._fault("uart_write_failed")
        return bool(ok)

    def _fault(self, reason):
        self.state, self.reason = "FAULT", str(reason)
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
"""Reusable application assembly; camera, vision and display keep separate clocks.

MaixPy imports stay in ``run()`` so importing the application API on a PC does
not open hardware. The vision worker exclusively owns controller/UART calls;
touch input only sets request events, including while the camera has no frame.
"""
import time

import cv2
import numpy as np

from .aim_control import VisionControl
from .camera_source import to_gray
from .gimbal_aim import GimbalAim
from .gimbal_calibrate import GimbalCalibrator
from .laser_spot import PANEL_H, draw_laser_overlay
from .preview import (Preview, TouchButtons, apply_maixvision_stream_policy,
                      display_result)
from .rate_meter import FrameRate
from .rectangle_center import draw_result
from .stepper_link import StepperLink
from .telemetry import MAIN_LOOP_SLOW_MS, format_perf_line, format_slow_loop
from .vision_worker import LatestFrameWorker


class TrackingApplication:
    """Run B-track recognition, preview and gimbal control as an application.

    ``cfg`` is a Config-compatible object; ``clock`` supplies ``perf_counter``.
    Hardware is opened by ``run()`` and closed on exit, including startup errors.
    External callers may use ``request_search()`` / ``request_self_test()``
    while running, without touching the UART or needing a display.
    """

    def __init__(self, cfg, build_id, clock=None, controller_factory=None):
        self.cfg, self.build_id = cfg, str(build_id)
        self.clock = clock if clock is not None else time
        self.controller_factory = controller_factory or VisionControl
        self.cam = self.uart = self.link = self.aim = self.calibrator = None
        self.controller = self.worker = self.preview = self.touch = None
        self._running = False

    def request_self_test(self):
        """Queue a self-test request; the controller checks fault/feature guards."""
        return self._request("request_test")

    def request_search(self):
        """Queue a search toggle (start/cancel), safe from any calling thread."""
        return self._request("request_search")

    def _request(self, name):
        event = getattr(self.controller, name, None)
        if not self._running or event is None:
            return False
        event.set()
        return True

    def _poll_touch(self):
        if self.touch is None:
            return
        hit = self.touch.poll(self.preview)      # 命中/未命中都会在 TouchButtons 里打 [touch] 日志
        if hit is None:
            return
        # 按下沿只持续一帧；高亮 0.25s 是现场肉眼确认"这次点击确实被识别"的唯一反馈。
        self.preview.flash_button(hit, self.clock.perf_counter())
        accepted = (self.request_self_test() if hit == "selftest"
                    else self.request_search())
        if not accepted:
            print("[touch] %s 请求未被受理（应用未运行或无控制器）" % hit)

    def _configure_touch(self):
        cfg, preview = self.cfg, self.preview
        if preview.enabled and bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            # Screen coordinates, clamped to fit both buttons even on small LCDs.
            width, height = int(preview.width), int(preview.height)
            margin = min(8, max(0, min(width, height) // 8))
            search = bool(getattr(cfg, "ENABLE_SEARCH_BUTTON", True))
            gap = max(1, min(8, height // 12))
            if height - 2 * margin < 3:
                search = False
            count = 2 if search else 1
            bw = min(max(70, int(width * .30)), max(1, width - 2 * margin))
            available = max(1, height - 2 * margin - (gap if search else 0))
            bh = min(max(34, int(height * .20)), max(1, available // count))
            x0, x1 = max(0, width - bw - margin), max(1, width - margin)
            preview.add_button("selftest", x0, margin, x1, margin + bh)
            if search:
                y0 = margin + bh + gap
                preview.add_button("search", x0, y0, x1, y0 + bh)
            self.touch = TouchButtons(cfg)
        elif preview.enabled:
            print("[touch] ENABLE_TOUCH_BUTTON=False -> 不画按钮")
        elif bool(getattr(cfg, "ENABLE_TOUCH_BUTTON", True)):
            print("[touch] 无显示 -> 屏幕按钮不可用（可通过应用 API 请求自检/搜索）")

    def _open_gimbal(self):
        cfg = self.cfg
        if not cfg.ENABLE_GIMBAL:
            return
        try:
            self.link = StepperLink(cfg)
            self.link.enable()
            self.aim = GimbalAim(cfg)
            self.calibrator = GimbalCalibrator(cfg, self.link, self.clock.perf_counter())
            if self.calibrator.done:
                self.calibrator = None
            else:
                print("[gimbal] calibration mode axis=%s（本次只标定，不闭环）" %
                      self.calibrator.axis)
            print("[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d" %
                  (cfg.YAW_DEV, cfg.ADDR_YAW, cfg.PITCH_DEV, cfg.ADDR_PITCH,
                   cfg.ACC_DEFAULT))
        except Exception as exc:
            # Preserve any partly opened link for exit cleanup; never drive it
            # when setup did not complete.
            self.aim = self.calibrator = None
            print("[gimbal] disabled after init failure: %s" % exc)

    def run(self):
        """Run until MaixPy requests exit, then release owned resources."""
        if self._running:
            raise RuntimeError("tracking application is already running")
        previous_thread = self.worker.thread if self.worker is not None else None
        if previous_thread is not None and previous_thread.is_alive():
            raise RuntimeError("previous vision worker still owns hardware")
        previous_display = self.preview.thread if self.preview is not None else None
        if previous_display is not None and previous_display.is_alive():
            raise RuntimeError("previous display worker still owns hardware")
        previous_heading = getattr(self.controller, "heading", None)
        if bool(getattr(previous_heading, "worker_alive", False)):
            raise RuntimeError("previous IMU worker still owns hardware")
        # A second run opens fresh resources. A failed early startup must never
        # close or stop already released resources left over from a prior run.
        self.cam = self.uart = self.link = self.aim = self.calibrator = None
        self.controller = self.worker = self.preview = self.touch = None
        self._running = True
        cfg, clock = self.cfg, self.clock
        try:
            apply_maixvision_stream_policy(cfg)
            from maix import app, camera, image

            cv2.setNumThreads(cfg.OPENCV_THREADS)
            self.cam = camera.Camera(cfg.CAMERA_W, cfg.CAMERA_H,
                                     image.Format.FMT_GRAYSCALE,
                                     fps=cfg.CAMERA_FPS, buff_num=cfg.CAMERA_BUFFERS)
            self.cam.skip_frames(cfg.SKIP_FRAMES)
            if cfg.ENABLE_UART:
                try:
                    from maix import uart as maix_uart
                    self.uart = maix_uart.UART(cfg.UART_PORT, cfg.UART_BAUD)
                except Exception as exc:
                    print("[uart] open failed: %s" % exc)
            self._open_gimbal()
            self.preview = Preview(cfg, clock=clock)
            self._configure_touch()
            self.controller = self.controller_factory(
                cfg, self.link, self.aim, self.calibrator, self.uart, clock=clock)
            self.worker = LatestFrameWorker(self.controller.process, self.controller.idle,
                                           cfg.VISION_ASYNC, clock=clock)
            self._log_startup()
            self._capture_loop(app, image)
        finally:
            self._running = False
            self._shutdown()

    def _log_startup(self):
        cfg, preview = self.cfg, self.preview
        print("[rect-center] %s search_width=%d display_width=%d display_fps=%d" %
              (self.build_id, cfg.SEARCH_WIDTH, preview.render_width, cfg.SHOW_FPS))
        print("[preview] async=%s 画布=%dx%d（SHOW_WIDTH=%d；0=用屏幕原始宽度）" %
              (getattr(cfg, "SHOW_ASYNC", True), preview.render_width,
               preview.render_height, int(getattr(cfg, "SHOW_WIDTH", 0) or 0)))
        print("[vision] latest-frame worker=%s max_result_age=%.0fms" %
              (cfg.VISION_ASYNC, cfg.RESULT_MAX_AGE_S * 1000))
        print("[ui] 上电自检已改为屏幕按钮：点显示右上角 SELF-TEST 触发（不再上电自动跑）")
        if "search" in preview.buttons:
            print("[ui] SEARCH 按钮：点击启动搜索，再点击取消（不自动启动）")

    def _capture_loop(self, app, image):
        cfg, clock = self.cfg, self.clock
        worker, preview = self.worker, self.preview
        capture_rate = FrameRate()
        last_log = clock.perf_counter()
        read_times, draw_times, loop_times = [], [], []
        while not app.need_exit():
            started = clock.perf_counter()
            img = self.cam.read()
            captured = clock.perf_counter()
            # Touch input remains live during camera gaps. It only sets events;
            # async UART/control work remains exclusively on the vision worker.
            self._poll_touch()
            if img is None:
                if worker.thread is None:
                    self.controller.idle()
                self._check_worker()
                continue
            read_times.append((captured - started) * 1000)
            gray = to_gray(img, image)
            capture_rate.record(captured)
            worker.submit(gray, captured)
            now = clock.perf_counter()
            snapshot, vision_fps, _ = worker.snapshot(now)
            self._check_worker()
            result, laser_state, age = display_result(snapshot, now, cfg.RESULT_MAX_AGE_S)
            if preview.due(now):
                t_draw = clock.perf_counter()
                self._draw_preview(gray, result, laser_state, snapshot, vision_fps, age, now)
                draw_times.append((clock.perf_counter() - t_draw) * 1000)
            loop_times.append((clock.perf_counter() - started) * 1000)
            now = clock.perf_counter()
            if now - last_log >= cfg.LOG_INTERVAL:
                self._log_performance(now, capture_rate, vision_fps, result, age,
                                      read_times, draw_times, loop_times, snapshot)
                last_log = now
                read_times.clear()
                draw_times.clear()
                loop_times.clear()

    def _check_worker(self):
        if self.worker.error is not None:
            raise RuntimeError("vision worker failed: " + self.worker.error)

    def _draw_preview(self, gray, result, laser_state, snapshot, vision_fps, age, now):
        preview = self.preview
        view = draw_result(gray, result, preview.render_width,
                           top_offset=PANEL_H if laser_state is not None else 0)
        if laser_state is not None:
            draw_laser_overlay(view, laser_state, result,
                               min(1.0, preview.render_width / float(gray.shape[1])),
                               use_raw=self.cfg.AIM_USE_RAW)
        cv2.putText(view, "disp %.1f low1 %.1f det %.1f age %.0fms" %
                    (preview.fps, preview.low1, vision_fps, age * 1000),
                    (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    .40, (0, 255, 255), 1)
        preview.render(view, now, status=snapshot["status"] if snapshot else "waiting")

    def _log_performance(self, now, capture_rate, vision_fps, result, age,
                         read_times, draw_times, loop_times, snapshot=None):
        preview, worker = self.preview, self.worker
        capture_fps, _ = capture_rate.values(now)
        display_fps, display_low1 = preview.rate.values(now)
        show_avg, show_p95, show_max, show_n = preview.take_show_stats()
        read_avg = float(np.mean(read_times)) if read_times else 0.0
        draw_avg = float(np.mean(draw_times)) if draw_times else 0.0
        loop_avg = float(np.mean(loop_times)) if loop_times else 0.0
        # ★ det_* 取"识别线程自己的结果"（snapshot["result"]），不是显示口径的 result：
        #   后者过期时已被 display_result 换成 stage="stale"，会掩盖检测器真实 stage。
        detector = (snapshot or {}).get("result")
        print(format_perf_line(
            self.build_id, capture_fps, display_fps, display_low1, vision_fps,
            result, read_avg, draw_avg, loop_avg, show_avg, show_p95, show_max,
            show_n, age, preview.shown, preview.dropped, worker.dropped,
            detector=detector))
        if self.aim is not None:
            print(self.aim.status_line())
        if show_p95 > 100:
            preview.warn_slow_once(show_p95)
        if loop_avg > MAIN_LOOP_SLOW_MS:
            print(format_slow_loop(loop_avg))

    def _close_resource(self, resource, label):
        close = getattr(resource, "close", None)
        if close is not None:
            try:
                close()
            except Exception as exc:
                print("[%s] close on exit failed: %s" % (label, exc))

    def _shutdown(self):
        """Stop worker before UART/IMU; display/camera cleanup is independent."""
        worker_error = None
        if self.worker is not None:
            try:
                self.worker.close()
            except Exception as exc:
                worker_error = exc
        thread = self.worker.thread if self.worker is not None else None
        if thread is None or not thread.is_alive():
            if self.link is not None:
                try:
                    self.link.stop()
                except Exception as exc:
                    print("[gimbal] stop on exit failed: %s" % exc)
            self._close_resource(self.controller, "vision")
            self._close_resource(self.link, "gimbal")
            self._close_resource(self.uart, "uart")
        self._close_resource(self.preview, "preview")
        self._close_resource(getattr(self.touch, "ts", None), "touch")
        self._close_resource(self.cam, "camera")
        if worker_error is not None:
            raise worker_error
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
    u, v       float   靶心像素坐标（已按 CENTER_ALPHA 平滑，供显示/常规使用）
    raw_u/v    float   未平滑中心（闭环用，见 config.AIM_USE_RAW）
    center     tuple   (u, v)；无效时为 None
    quad       (4,2)   外框四角，顺序已规整（左上起逆时针）
    inner_quad (4,2)   内框四角（胶带内沿）
    stage      str     track / roi_search / full_search / no_black_frame
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

    cfg = load_config(MIN_AREA_RATIO=0.02, SEARCH_WIDTH=240)
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
from vision.aim_control import VisionControl
from vision.gimbal_search import SearchSweep
from vision.imu_heading import HeadingFeedforward
from vision.app_runtime import TrackingApplication

__all__ = ["Result", "load_config", "to_gray", "detect_black_frame",
           "CenterTracker", "track_center", "reset", "AimLoop",
           "TrackingApplication", "SearchSweep", "HeadingFeedforward"]

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
            return "Result(invalid, stage=%s)" % self.get("stage")
        return "Result(u=%.1f, v=%.1f, stage=%s)" % (
            self.get("u", float("nan")), self.get("v", float("nan")), self.get("stage"))


def load_config(**overrides):
    """返回 config.Config（可带覆盖项，不改磁盘上的 config.py）。

    ``load_config(MIN_AREA_RATIO=0.02)`` → 派生一个只改了该字段的配置类。
    """
    from config import Config
    if not overrides:
        return Config
    return type("ApiConfig", (Config,), dict(overrides))


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
    if a.size == 0:
        return None
    if a.ndim == 3 and a.shape[2] == 1:
        a = a[:, :, 0]
    elif a.ndim == 3 and a.shape[2] == 3:
        a = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    elif a.ndim == 3 and a.shape[2] == 4:
        a = cv2.cvtColor(a, cv2.COLOR_RGBA2GRAY)
    if a.ndim != 2:
        return None
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return a


def _wrap(raw, gray):
    """把检测器的 dict 包成 Result，并补 center / image_size 等便捷字段。"""
    res = Result(raw)
    res["center"] = (res["u"], res["v"]) if res.get("valid") else None
    if gray is not None:
        res["image_size"] = (int(gray.shape[1]), int(gray.shape[0]))
    res.setdefault("dx", None)
    res.setdefault("dy", None)
    res.setdefault("aim_valid", False)
    return res


# ------------------------------------------------------------- API ① 识别黑框
def detect_black_frame(frame, cfg=None, detector=None):
    """**无状态**识别黑框，返回 ``Result``（没识别到则 ``valid=False``，``bool(res)`` 为假）。

    每帧都是一次完整的"缩小图多阈值扫描 → 四边暗带验证 → 亚像素精修"，不依赖上一帧，
    适合"只在某几问里取一次黑框"的用法。想复用实例提速可传 ``detector``（但那样就带状态了，
    连续调用会走窄带跟踪路径）。

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
        """追踪一帧；now 为单调递增的采图时刻（秒），供丢帧后的运动预测使用。"""
        gray = to_gray(frame)
        if gray is None or gray.size == 0:
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
        self._clock = clock if clock is not None else time
        self._stopped = True
        self._controller = None
        self.heading = heading
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = None
        self.aim = None
        if enable_motors:
            from vision.stepper_link import StepperLink
            from vision.gimbal_aim import GimbalAim
            self.link = StepperLink(self.cfg)
            self.link.enable()
            self.aim = GimbalAim(self.cfg)

    def step(self, frame, now=None):
        """处理一帧；now 为采图时刻，与 clock.perf_counter 使用同一单调时钟。"""
        captured = self._clock.perf_counter() if now is None else float(now)
        res = self.tracker.track(frame, captured)
        now = self._clock.perf_counter()
        res["captured_at"] = captured
        res["age_s"] = max(0.0, now - captured)
        if not 0 <= now - captured <= float(getattr(self.cfg, "RESULT_MAX_AGE_S", 0.20)):
            res["aim_valid"] = False
        res["aim_state"] = None
        res["cmds"] = {"yaw": None, "pitch": None}
        if self.aim is None or self.link is None:
            return res
        controller = self._control()
        controller.accept_result(res, captured, res.get("laser"))
        res["aim_state"] = self.aim.state
        res["cmds"] = dict(controller.last_commands)
        res["control_source"] = controller.control_source
        res["search_state"] = controller.search.state if controller.search is not None else None
        self._stopped = controller.stale_stopped
        return res

    def _control(self):
        if (self._controller is None or self._controller.link is not self.link or
                self._controller.aim is not self.aim):
            self._controller = VisionControl(self.cfg, self.link, self.aim, None, None,
                                             clock=self._clock, heading=self.heading)
            self.heading = self._controller.heading
        return self._controller

    def request_search(self):
        """启动/取消 SEARCH；由 step()/idle() 消费请求，不在按钮线程写串口。"""
        if self.link is None or self.aim is None:
            return False
        self._control().request_search.set()
        return True

    def calibrate_imu_bias(self):
        """请求静止偏置标定；先停机，运行时仍需保持底盘和云台静止。"""
        controller = self._control()
        if controller.heading is None or self.link is None or self.aim is None:
            return False
        controller.request_imu_bias.set()
        return True

    def imu_status(self):
        return self._control().imu_status()

    def idle(self):
        """无新图像时维持 ACK、搜索、IMU 补偿和视觉过期停机。"""
        self._control().idle()

    def stop(self, now=None):
        now = self._clock.perf_counter() if now is None else float(now)
        if self._controller is not None:
            accepted = self._controller.stop(now)
            self._stopped = True if accepted else self._stopped
            return accepted
        if self.link is not None:
            if self.link.stop(now=now) is False:
                if self.aim is not None:
                    self.aim.fault("stop:all")
                return False
        if self.aim is not None:
            self.aim.motion_stopped(now=now)
        self._stopped = True
        return True

    def close(self):
        self.stop()
        if self._controller is not None:
            self._controller.close()
        if self.link is not None:
            close = getattr(self.link, "close", None)
            if close is not None:
                close()
'''

_MAIN_SRC = r'''
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
'''

_MOD_ORDER = ('config', 'vision.rate_meter', 'vision.camera_source', 'vision.vision_worker', 'vision.telemetry', 'vision.maixvision_stream', 'vision.preview', 'vision.rect_geometry', 'vision.contour_locator', 'vision.kalman_filter', 'vision.rectangle_center', 'vision.laser_spot', 'vision.stepper_link', 'vision.gimbal_self_test', 'vision.gimbal_aim', 'vision.gimbal_search', 'vision.imu_heading', 'vision.aim_control', 'vision.gimbal_calibrate', 'vision.app_runtime', 'target_api')

_REL_OF = {'config': 'config.py', 'vision.rate_meter': 'vision/rate_meter.py', 'vision.camera_source': 'vision/camera_source.py', 'vision.vision_worker': 'vision/vision_worker.py', 'vision.telemetry': 'vision/telemetry.py', 'vision.maixvision_stream': 'vision/maixvision_stream.py', 'vision.preview': 'vision/preview.py', 'vision.rect_geometry': 'vision/rect_geometry.py', 'vision.contour_locator': 'vision/contour_locator.py', 'vision.kalman_filter': 'vision/kalman_filter.py', 'vision.rectangle_center': 'vision/rectangle_center.py', 'vision.laser_spot': 'vision/laser_spot.py', 'vision.stepper_link': 'vision/stepper_link.py', 'vision.gimbal_self_test': 'vision/gimbal_self_test.py', 'vision.gimbal_aim': 'vision/gimbal_aim.py', 'vision.gimbal_search': 'vision/gimbal_search.py', 'vision.imu_heading': 'vision/imu_heading.py', 'vision.aim_control': 'vision/aim_control.py', 'vision.gimbal_calibrate': 'vision/gimbal_calibrate.py', 'vision.app_runtime': 'vision/app_runtime.py', 'target_api': 'target_api.py'}

_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision
_pkg.__path__ = []
_sys.modules["vision"] = _pkg
for _full in _MOD_ORDER:
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
    exec(compile(_MOD_SRC[_full], _full.replace(".", "/") + ".py", "exec"),
         _m.__dict__)
del _m, _full, _MOD_SRC

# --------------------------------------------------------------------------
# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'main.py', "exec"), globals())
