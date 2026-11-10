class Config:
    # Camera profile. GRAYSCALE keeps the MaixCAM2 frame buffer small.
    CAMERA_W = 640
    CAMERA_H = 480
    CAMERA_FPS = 30
    CAMERA_BUFFERS = 2
    SKIP_FRAMES = 10
    # The historical strength=1.5 is not calibrated to this lens. Plane
    # homography handles target perspective; radial correction stays off.
    LENS_CORR = False
    LENS_STRENGTH = 1.5
    SHOW = True
    # 检测不跳帧；显示独立限频，减少 RGB 转换和 MaixVision 图像传输开销。
    SHOW_WIDTH = 480
    SHOW_FPS = 15
    # ---- MaixVision 图传（独立进程；主线程只写共享内存，JPEG/WebSocket 在子进程）----
    SHOW_STREAM = True              # False = 关闭图传，回到纯本地屏
    SHOW_STREAM_HOST = "127.0.0.1"  # MaixVision 的 WebSocket 服务地址
    SHOW_STREAM_PORT = 7899
    SHOW_JPEG_QUALITY = 70
    SHOW_STREAM_RETRY_S = 2.0       # 子进程重连间隔
    SHOW_STREAM_RETRY_MAX_S = 15.0  # 重连退避上限
    OPENCV_THREADS = 1
    LOG_INTERVAL = 2.0
    ENABLE_UART = False
    UART_PORT = "/dev/ttyS0"
    UART_BAUD = 115200
    # @A,aim_valid,tx,ty,lx,ly,dx,dy,calibrated,laser_valid,target_valid%。
    UART_PROTOCOL = "aim"

    # 本对话最后版本：截图亮点为局部搜索初值，不是实际硬件精确标定。
    LASER_REFERENCE_SIZE = (453, 333)
    LASER_PIXEL = (209.0, 144.0)
    LASER_CALIBRATED = False
    LASER_MODE = "detect"
    LASER_SEARCH_RADIUS = 32
    LASER_MIN_CONTRAST = 8.0
    LASER_MIN_AREA = 2
    LASER_MAX_AREA = 100
    LASER_MAX_DIAMETER = 16
    # Low-resolution binary contour proposal; adaptive threshold is fallback.
    ADAPTIVE_BLOCK_SIZE = 31
    ADAPTIVE_C = 8.0
    MIN_CONTOUR_AREA = 350
    MIN_QUAD_SIDE = 14
    MIN_ROI_SIZE = 32
    ROI_MIN_PADDING = 24
    MAX_CONTOURS = 128
    MAX_CANDIDATES = 8
    MIN_AREA_RATIO = 0.025
    MAX_AREA_RATIO = 0.92
    APPROX_EPSILON = 0.025
    MAX_LINE_RESIDUAL_PX = 2.0
    SEARCH_WIDTH = 320
    EDGE_SAMPLES = 24
    REFINE_RADIUS = 6
    TRACK_RADIUS = 12
    ROI_PADDING = 0.20
    MIN_EDGE_COVERAGE = 0.55
    MAX_DARK_BRIGHT_RATIO = 0.72
    WEAK_EDGE_COVERAGE = 0.25
    FILTER_RESET_PX = 25.0
    MIN_EDGE_CONTRAST = 20.0
    BLACK_FRAME_MM = 18.0

    # Physical target coordinates. The center is invariant to scale, but the
    # aspect ratio must match the printed target for a perspective correction.
    TARGET_SHORT_MM = 210.0
    TARGET_LONG_MM = 297.0
    CIRCLE_SAMPLES = 72
    CIRCLE_RADIUS_MM = 2.0

    # Optional temporal smoothing. Zero disables it.
    CENTER_ALPHA = 0.70
