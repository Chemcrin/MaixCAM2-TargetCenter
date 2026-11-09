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
    SHOW_STREAM = False
    # 0 = 用屏幕原始宽度（推荐）：640×480 画布与相机同尺寸，驱动层不再缩放
    SHOW_WIDTH = 0
    SHOW_FPS = 40
    SHOW_JPEG_QUALITY = 70
    VISION_ASYNC = True      # 后台识别，忙时只保留最新帧，不积压旧图
    SHOW_ASYNC = True        # 独立显示线程：show() 慢只掉预览帧，不拖识别/串口/闭环
    RESULT_MAX_AGE_S = 0.25  # 以采图时刻计龄；过期结果不画框、不下发指令（30fps 下 0.25s 很宽了）
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
    # False = 闭环用平滑后的 u/v（抑制抖动）；True = 用 raw_u/raw_v（未平滑）
    AIM_USE_RAW = False
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

    # ---- 识别（降采样 → 自适应阈值 → 连通域 → 四边验框 → 弦中点定心）----
    RECT_DOWNSCALE = 2            # 先降采样再找框；1 = 全分辨率（更慢，一般不必）
    # 阈值 = RECT_LEVEL_RATIO × 画面 90 分位亮度，夹在 [FLOOR+p2, CEIL] 之间。
    # ★ 这是唯一需要按现场调的识别参数：纸面偏暗就调低 RATIO。
    RECT_LEVEL_RATIO = 0.62
    RECT_LEVEL_FLOOR = 45.0
    RECT_LEVEL_CEIL = 160.0
    RECT_LEVEL_REFRESH = 15       # 跟踪时每 N 帧重算一次阈值（曝光慢变够用）
    RECT_MIN_AREA = 1200          # 候选最小面积（原图 px2；内部按降采样折算）
    RECT_MAX_AREA_RATIO = 0.55    # 外接框面积上限（占整幅比例），挡"锁到整面墙"
    RECT_MIN_ASPECT = 1.05        # 外接框长短边比。外接框不是最小外接矩形，
    RECT_MAX_ASPECT = 2.60        # 靶纸倾斜时它会变方，所以上限放宽到 2.6
    RECT_MIN_FILL = 0.03          # 连通域面积 / 外接框面积：环是空心的，很小
    RECT_MAX_FILL = 0.70          # 太满说明是实心块，不是框
    RECT_MIN_EDGE = 0.20          # ★ 四条边最弱一条的暗像素占比下限。
    #   胶带环实测 0.27~0.45；"框住靶心的稀疏暗结构"（窗棂/柜门缝）只有 0.10。
    RECT_PAPER_ROUNDS = 2         # 从框心向外扫几条线迭代几次（2 次足够收敛）
    RECT_TRACK_EXPAND = 1.60      # 跟踪 ROI = 上一帧框 × 该倍数
    RECT_TRACK_MARGIN = 16.0      # 跟踪 ROI 额外边距（原图 px）
    RECT_TRACK_SIZE_TOL = 0.55    # 跟踪时允许的尺寸变化（±55%）
    RECT_HOLD_FRAMES = 6          # 跟丢后按速度外推继续输出的帧数（30fps ≈ 0.2s）
    RECT_HOLD_MAX_PX = 90.0       # 外推距离上限，防止甩飞
    RECT_MAX_SPEED = 2500.0       # 速度估计上限（px/s），防止小 dt 把速度除爆
    RECT_TRACK_GATE = 0.45        # 跟踪门：实测中心离预测超过 0.45×框边长就判本帧作废
    RECT_SMOOTH = 0.55            # 中心一阶低通系数；1 = 不平滑
    RECT_KEEP_STAGES = False      # 真：把 mask 留在 last_stages，供 --stage 导出

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
