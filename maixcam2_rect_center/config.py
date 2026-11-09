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
    SHOW_FPS = 25
    SHOW_JPEG_QUALITY = 70
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

    # ---- 识别（阈值 → 连通域 → 环-洞结构 → 洞四边拟线 → 对角线交点）----
    RECT_DOWNSCALE = 2            # 搜索态降采样倍数（跟踪态按 ROI 自适应钉上限）
    RECT_LEVEL_RATIO = 0.62       # 阈值 = 亮面 90 分位 × 该比例（压在胶带与纸面之间）
    RECT_LEVEL_FLOOR = 45.0       # 阈值下限（再暗的胶带也要能进得来）
    RECT_LEVEL_CEIL = 160.0       # 阈值上限（再亮的背景也不许当纸面）
    RECT_CLOSE_SIZE = 3           # 二值闭运算核：桥接运动模糊的细断带；<3 = 关闭
    RECT_MIN_AREA = 1200          # 连通域最小面积（全分辨率 px2）
    RECT_MAX_AREA_RATIO = 1.01    # bbox 面积占画面上限：实效关闭。真机实测暗框与暗背景
                                  # 连成 mega-blob 时 bbox≈整幅（比值=1.0），任何 <1 的门
                                  # 都会把正确候选杀掉；误锁防护由洞尺寸窗/边比/亮度/
                                  # 暗带封闭性接管，要收紧再调小此值
    RECT_MIN_SIDE_PX = 24         # bbox 最短边下限（全分辨率 px）
    RECT_BBOX_ASPECT_MIN = 0.45   # bbox 边比宽松窗：纸面转 45° 时 bbox 近正方形，不能误杀
    RECT_BBOX_ASPECT_MAX = 3.4
    RECT_MIN_FILL = 0.04          # 连通域填充率窗：环是空心的，fill 天然偏低
    RECT_MAX_FILL = 0.75          # 上限挡实心块（真机立柱实测 0.88）
    RECT_MAX_CANDIDATES = 4       # 精测候选上限：真靶纸在杂乱场景里可能只排第 3（暗门/
                                  # 暗墙 bbox 更大），前 2 名全灭就轮不到它；搜索态瞬态开销可接受
    RECT_MIN_ASPECT = 1.05        # 洞（纸面）边比窗：下限杀正方形/圆洞；上限给透视留余量
    RECT_MAX_ASPECT = 3.2
    RECT_HOLE_MIN_PX = 70         # 洞长边尺寸窗（靶纸放低/后仰时洞会缩小，下限要容得住；
    RECT_HOLE_MAX_PX = 400        # 挡印刷环这类微型假框与远处的巨型假框）
    RECT_BAND_COVER = 0.60        # 洞向外一圈（0.06×洞短边厚）的暗占比下限：
                                  # 拦"细丝围亮区"/印刷细环；误锁细框结构就调高
    RECT_BAND_MIN_CONTRAST = 25.0 # 洞内均值 − Otsu 阈值 ≥ 该值才认（洞必须是亮纸面）
    RECT_MODEL_MM = (261.0, 174.0)  # 纸面模型尺寸；只影响 width_mm/height_mm 输出
    RECT_KEEP_STAGES = False      # 真：把二值图留在 last_stages，供 --stage 导出

    # ---- ROI 跟踪（锁定后的三件套：小窗找、外推跟、丢了续命）----
    RECT_TRACK_EXPAND = 1.35      # ROI = 上帧框 × 该倍数 + MARGIN
    RECT_TRACK_MARGIN = 10.0      # ROI 外扩像素
    RECT_TRACK_MAX_SIDE = 400.0   # ROI 降采样后最长边上限；≥典型 ROI 时跟踪态=全分辨率量测（与搜索态同口径，重捕获零跳变），大 ROI 才降采样兜底
    RECT_TRACK_SIZE_TOL = 0.25    # 跟踪尺寸门：块尺寸相对"锁定尺寸"的容差
    RECT_TRACK_GATE = 0.45        # 跟踪门：实测中心离预测超 0.45×框边 → 当帧作废
    RECT_HOLD_FRAMES = 6          # 抓不到时按速度外推续命的帧数
    RECT_HOLD_MAX_PX = 90.0       # 外推单帧限幅（防速度跑飞把 ROI 甩到画面另一头）
    RECT_MAX_SPEED = 2500.0       # 速度限幅 px/s；dt<4ms 不更新（连拍帧间隔会除出天文数字）
    RECT_SMOOTH = 0.55            # 中心一阶低通系数（重捕获/续命后直接对齐不平滑）

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
