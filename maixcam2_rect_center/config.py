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
