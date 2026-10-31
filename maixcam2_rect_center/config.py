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
    # True = 用 raw_u/raw_v 算 dx/dy：CENTER_ALPHA 只留给显示，闭环用平滑值会加 ≈28ms 滞后
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

    # ---- 搜索 ----
    CANNY_LOW = 35
    CANNY_HIGH = 120
    SEARCH_WIDTH = 320            # 搜索在低分辨率做；精修回到原图
    SEARCH_SPLIT_FRAMES = False   # 历史兼容项
    SEARCH_MAX_PROPOSALS = 128    # 仅 SEARCH_EXHAUSTIVE=False 时生效
    MAX_CANDIDATES = 8
    EDGE_SAMPLES = 24             # 每条边的采样列数
    REFINE_RADIUS = 6             # 窄带精修半径（原图 px）
    TRACK_RADIUS = 12             # 追踪时的窄带半径
    # 运动预测：用上两帧四角的逐角位移外推本帧位置，再在预测处窄带精修
    ENABLE_MOTION_PREDICT = True
    PREDICT_MAX_PX = 60.0         # 单帧预测位移上限，防噪点把预测甩飞
    ROI_PADDING = 0.20
    EDGE_CLOSE_PX = 3
    SEARCH_MAX_CONTOURS = 16
    SEARCH_EXHAUSTIVE = True
    # 提案阶段允许"部分角出画"的四边形：与 _verify 的 MAX_CLIPPED_EDGES 同口径。
    # 关掉它，贴边/大靶的候选在搜索层就被丢掉，验证层的出画容忍等于白写。
    SEARCH_ALLOW_CLIPPED = True
    DARK_THRESHOLDS = (25, 40, 55, 70, 85, 100, 120, 145)   # 仅供旧工具读取
    SEARCH_BUDGET_MS = 25.0
    MAX_CONTOURS = 128
    ENABLE_LINE_RECOVERY = False
    HULL_FILL_RATIO = 0.60
    MIN_EDGE_LENGTH_PX = 12.0

    # ---- 胶带证据判据 ----
    MIN_EDGE_COVERAGE = 0.55      # 一条边"成立"所需的采样覆盖率
    WEAK_EDGE_COVERAGE = 0.25     # 弱边下限（低于它必须有其它强边兜住）
    MIN_OBSERVED_EDGE_COVERAGE = 0.125   # 画面内整边无证据 → 拒 U 形开口
    STRONG_EDGE_COVERAGE = 0.85   # 强边：可放宽其它边
    MAX_DARK_BRIGHT_RATIO = 0.72  # 暗带最亮 ≤ 0.72×亮面，排除"两侧都亮"的纸边
    MIN_EDGE_CONTRAST = 20.0      # 暗/亮对比度下限（灰度）
    MIN_LINE_COVERAGE = 0.55      # 拟合一条边所需内点比例
    MAX_LINE_RESIDUAL_PX = 0.8    # 原图像素；拒绝用曲线拟合矩形边
    MIN_AREA_RATIO = 0.008
    MAX_AREA_RATIO = 0.92
    APPROX_EPSILON = 0.025
    MIN_ANGLE_DEG = 55.0
    MIN_INNER_RATIO = 0.30
    MAX_INNER_RATIO = 0.94
    DARK_CENTER_MIN = 90.0
    BLACK_FRAME_MM = 18.0
    REFINE_ITERATIONS = 2
    REFINE_RADIUS_WIDE = 20       # 窄带够不到真边时的宽半径精修
    # 宽半径救援（仅兜底）：主判据是相对亮度，绝对上限防过曝误收
    RECOVER_MIN_CONTRAST = 80
    RECOVER_MAX_DARK_ABS = 160.0
    RECOVER_MAX_DARK_RATIO = 0.60

    # ---- 部分可见 / 出画 ----
    # _verify 允许"3 条边有证据 + 第 4 条弱/被裁"，_refine 最多 1 条边用先验兜底；
    # 实测 ≥ PARTIAL_MIN_EDGES 即出版，结果标 partial（黄框），控制侧可降增益。
    CLIP_MIN_EDGE_VALID = 0.5     # 一条边至少 50% 采样点在图内才算"可见"
    MAX_CLIPPED_EDGES = 1         # 最多允许 1 条边被画面裁掉
    PARTIAL_MIN_EDGES = 3         # partial 至少实测 3 条边（第 4 条用先验边）
    # 复锁尺度门：3 边补出来的中心只可信"还是同一张靶"，按面积比判；
    # 完整 4 边 + 强证据不受限（真换靶/大扫动也能锁）。
    REACQUIRE_AREA_RATIO = 3.0
    PREDICT_MAX_GAP_S = 0.25

    # ---- 中心滤波（显示口径）----
    CENTER_ALPHA = 0.70           # 0 = 关闭
    FILTER_RESET_PX = 25.0
    KALMAN_ENABLE = True
    KALMAN_PROCESS_PX = 40.0
    KALMAN_MEASURE_PX = 0.9
    KALMAN_RESET_PX = 25.0
    KALMAN_MAX_PREDICT_S = 0.25
    KALMAN_VELOCITY_LIMIT_PX_S = 6000
    FRONT_VIEW_SHORT_PX = 0       # >0 才做正视面复核（当前关闭）
    FRONT_VIEW_MAX_ERROR_PX = 5.0

    # ---- 物理靶面 ----
    TARGET_SHORT_MM = 210.0
    TARGET_LONG_MM = 297.0
    CIRCLE_SAMPLES = 72
    CIRCLE_RADIUS_MM = 2.0

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
