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
    # True = 保留 MaixVision 图传（仅在「运行项目」模式下有效）；
    # False = 强制本地屏、不图传。★ 实测：图传通道没建立时 send_image() 每帧空转 500ms
    # 并卡住整个解释器，识别会被拖到 20 秒/帧，所以默认关闭。要看画面再临时打开。
    SHOW_STREAM = True
    # 0 = 用屏幕原始宽度（推荐）：640×480 画布与相机同尺寸，驱动层不再缩放
    SHOW_WIDTH = 0
    SHOW_FPS = 30
    SHOW_JPEG_QUALITY = 40
    # 子进程图传的投递宽度（0 = 原样投 640×480）。240 时编码/网络量约为原样 1/6，
    # 子进程单帧开销降到几 ms，30fps 节拍可稳定跟上；识别帧率本就不受图传影响。
    SHOW_STREAM_WIDTH = 240
    VISION_ASYNC = True      # 后台识别，忙时只保留最新帧，不积压旧图
    SHOW_ASYNC = True        # 独立显示线程：show() 慢只掉预览帧，不拖识别/串口/闭环
    RESULT_MAX_AGE_S = 0.40  # 以采图时刻计龄；过期结果不画框、不下发指令（30fps 下收紧窗口）
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
    MOTOR_PULSES_PER_REV = 3200   # 运动时间模型用；未标定前用出厂值
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
    COMMAND_INTERVAL_S = 0.08

    # ---- 闭环口径 ----
    # True = 用 raw_u/raw_v 算 dx/dy：本链路无滤波，两者恒等；保留该开关以兼容闭环代码
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
    UART_RX_MAX_BYTES = 256
    UART_RX_BUFFER_MAX_BYTES = 1024

    # ---- 识别（降采样 Otsu 提案 → 窄带黑带验证 → 四边梯度精修 → 内框对角线交点）----
    SEARCH_WIDTH = 320            # 提案阶段长边上限：1/2 尺度下胶带仍连续，像素量被钉住
    EDGE_SAMPLES = 24             # 每条边的采样位置数；参与覆盖率统计与拟线点数
    REFINE_RADIUS = 6             # 搜索态精修的法线搜索半径（px）
    TRACK_RADIUS = 12             # 跟踪态精修半径：允许比搜索态更大的帧间位移
    MIN_QUAD_SIDE = 14            # 提案四边形最短边（降采样尺度 px）
    MIN_CONTOUR_AREA = 350        # 提案连通域最小面积（降采样尺度 px2）
    MIN_AREA_RATIO = 0.025        # 同上，按画幅比例兜底（远处小靶纸不能让位给大暗块）
    MAX_AREA_RATIO = 0.92         # 提案面积上限占画幅比例
    MAX_CANDIDATES = 8            # 每种掩膜保留的提案上限；验证代价线性，别调太大
    MAX_CONTOURS = 128            # 亮面补集轮廓的截断数（暗背景下轮廓可能成百上千）
    APPROX_EPSILON = 0.025        # approxPolyDP 的周长比例系数
    ADAPTIVE_BLOCK_SIZE = 31      # Otsu 无目标时的局部阈值块大小
    ADAPTIVE_C = 8.0
    MAX_LINE_RESIDUAL_PX = 2.0    # 单边拟线残差上限：超了说明这条边本来就不是直线
    MIN_EDGE_CONTRAST = 20.0      # 暗带与纸面的灰度差下限
    MIN_EDGE_COVERAGE = 0.55      # 单边"达标采样点"占比下限
    WEAK_EDGE_COVERAGE = 0.25     # 允许一条边偏弱（反光/遮挡），其余边仍须达标
    MAX_DARK_BRIGHT_RATIO = 0.72  # 暗带灰度 / 纸面灰度 上限
    RECT_ASPECT_MIN = 1.05        # 实测四边形长边/短边窗：下限杀正方形（方形显示器/窗框）
    RECT_ASPECT_MAX = 3.40        # 上限给强透视留裕量（物理外框 1.35，纸边 1.41）
    BLACK_FRAME_MM = 18.0         # ★ 胶带实测宽度，直接进判据；换胶带宽度必须改这里
    TARGET_SHORT_MM = 210.0       # A4 纸面物理尺寸，单应估计用
    TARGET_LONG_MM = 297.0
    ROI_PADDING = 0.20            # 跟踪 ROI = 上帧外框 × (1+该比例) + ROI_MIN_PADDING
    ROI_MIN_PADDING = 24.0
    MIN_ROI_SIZE = 32             # ROI 任一边小于它就不要了，直接回全图
    CENTER_ALPHA = 0.70           # 显示用一阶低通；闭环走 raw_u/raw_v，不受此值影响
    FILTER_RESET_PX = 25.0        # 单帧跳变超过它直接对齐，不拖尾
    RECT_MODEL_MM = (TARGET_LONG_MM, TARGET_SHORT_MM)  # 纸面模型尺寸；只影响 width_mm/height_mm 输出
    RECT_KEEP_STAGES = False      # 真：把二值图留在 last_stages，供 --stage 导出

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
    ACC_TRACK = 80                # 曲线加减速档位：0 = 瞬时启停（勿用）；越小越柔
    AIM_WAIT_MOTION = True
    AIM_MOTION_MARGIN_S = 0.03
    AIM_HISTORY_MAX_GAP_S = 0.30
    DIVERGE_GUARD = True          # 发散兜底停机
    DIVERGE_CONFIRM_FRAMES = 6
    DIVERGE_RATIO = 1.15
    DIVERGE_MIN_PX = 4.0
    GIMBAL_CALIBRATED = True      # 改过脉冲/像素后若闭环发散，先置 False 只跑识别
    SEARCH_ANGLE_DEG = 60.0       # 从启动位置向左右各扫 60 度；确认机械行程后启用
    SELF_TEST_ANGLE_DEG = 5.0     # 自检按左、右、上、下往返，结束回到起点
    MANUAL_RPM = 120
    MANUAL_ACC = 80
    MANUAL_SETTLE_S = 0.25
    ENABLE_TOUCH_BUTTONS = True
    AUTO_TRACK_ON_BOOT = False   # 按 SEARCH 后才开始追踪；无触屏可用 AimLoop.request_search()


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
