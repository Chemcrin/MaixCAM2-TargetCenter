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
