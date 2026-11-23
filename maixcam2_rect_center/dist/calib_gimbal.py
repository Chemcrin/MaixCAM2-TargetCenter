#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calib_gimbal.py · 25E 靶心识别（轨道 B）—— 单文件自包含版
================================================================================
★ 本文件由 tools/build_single.py 自动生成，请勿手改！
    改参数 -> config.py ；改算法 -> vision/*.py ；改流程 -> tools/calib_main.py
    改完在本目录重新执行：  python tools/build_single.py

用途：在 MaixVision 里点「运行当前文件」即可运行（不必用「运行项目」）。
      行为与模块化工程完全一致，输出也就完全一致。

生成时间：2026-11-23 21:57:56
    config.py                    sha1=8bcd19bda03a
    target_api.py                sha1=084f7844b3b7
    tools/calib_main.py          sha1=3c95f124966e
    vision/__init__.py           sha1=01904dd699d3
    vision/algo.py               sha1=a4578a955777
    vision/logic.py              sha1=52f3a7d6abfa
    vision/maixvision_stream.py  sha1=c30d784f26a0
    vision/output.py             sha1=7dc04c56e5d0
    vision/poll.py               sha1=992056fd530e
    vision/zdt.py                sha1=4fdd1d511e53
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': '8bcd19bda03ac887cb2f617565f0cacaeff46dd6', 'vision/poll.py': '992056fd530ecec7480c50b887eb69b6b1716aec', 'vision/algo.py': 'a4578a9557779521fea57262ea0a67d69c0aa05a', 'vision/maixvision_stream.py': 'c30d784f26a0da91756e9a63f71ba795fdda7a8d', 'vision/zdt.py': '4fdd1d511e538ef258cc06dca3f3fa16ecea60e2', 'vision/output.py': '7dc04c56e5d04fcc50fd21ec0357aad222fa2023', 'vision/logic.py': '52f3a7d6abfa6ce69eaeb60bb44d10f47c7e86f3', 'vision/__init__.py': '01904dd699d3c76bbd3d776a3b3a967cdbf01103', 'target_api.py': '084f7844b3b7371ac4b63786f0a8f5913afd5b48', 'tools/calib_main.py': '3c95f124966e66a91036b1659e2da3058f232dc1'}

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
    # True = 保留 MaixVision 图传（仅在「运行项目」模式下有效）；
    # False = 强制本地屏、不图传。★ 实测：图传通道没建立时 send_image() 每帧空转 500ms
    # 并卡住整个解释器，识别会被拖到 20 秒/帧，所以默认关闭。要看画面再临时打开。
    SHOW_STREAM = True
    # 0 = 用屏幕原始宽度（推荐）：640×480 画布与相机同尺寸，驱动层不再缩放
    SHOW_WIDTH = 0
    SHOW_FPS = 30
    SHOW_JPEG_QUALITY = 70
    # 子进程图传的投递宽度（0 = 原样投 640×480）。240 时编码/网络量约为原样 1/6，
    # 但画面细节（胶带边、同心圆）会被压掉，看判据时容易误判；默认投 480 兼顾清晰与带宽，
    # 现场追帧率再临时降回 240。
    SHOW_STREAM_WIDTH = 480
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
    ACC_TRACK = 80                # 兼容旧命令的兜底斜坡档（新命令自带 acc）
    # ★ 速度曲线：误差大走得快、误差小走得慢，一条指令一次完整的加速—匀速—减速。
    #   单位是**电机轴**度/秒；有减速比时输出端按比例更慢。
    AIM_DEG_S_PER_PX = 0.5        # 每 1px 误差想要多少度/秒（线性段）
    AIM_MIN_DEG_S = 8.0           # 最低速度：再小驱动器只有 1rpm 档，实际约 6°/s
    AIM_MAX_DEG_S = 60.0          # 最高速度：限制"短时间内转一大截"
    AIM_RAMP_S = 0.12             # 起停斜坡时间（驱动器按此反解 acc 档位）
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
    MANUAL_RPM = 15               # 手动/自检速度：15rpm ≈ 90°/s，可见又不甩
    MANUAL_ACC = 150              # 斜坡档：约 0.08s，起停不顿
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
'''

# ===== vision.poll  (vision/poll.py) =====
_MOD_SRC['vision.poll'] = r'''
"""① 轮询层：取帧 → 灰度 → 帧率统计 → 识别线程。

四层之一（见 `docs/识别链路四层分层方案.md`），职责只有"把画面安全地搬进识别线程"：

    FrameRate            滑动窗口帧率计（采集/识别/显示三处各自独立实例）
    to_gray              MaixPy 帧 → cv2 灰度 ndarray（零拷贝借用相机缓冲）
    to_gray_frame        校验入参、去掉 (H,W,1) 哑轴
    FramePoller          每帧一次：cam.read() → 可选镜头校正 → 灰度
    LatestFrameWorker    单识别线程 + 两个"最新槽位"，最新优先、丢弃旧帧

三条约束：

1. 不产生识别判据（"找到框没有"属于算法层与逻辑层）；
2. 不持有跨帧状态（`FramePoller` 每帧独立），唯一的跨帧状态是线程槽位与帧率窗口；
3. 不碰串口与屏幕，只碰相机。

依赖方向：**本层不 import 其余三层**，是依赖图的最底层。

`to_gray` 保持**零拷贝**语义：`image2cv(copy=False)` 借用的相机缓冲会在下一次
`cam.read()` 时被覆写，跨线程传递前必须 copy —— 这件事由 `LatestFrameWorker.submit` 负责。
"""
import threading
import time
from collections import deque

import cv2
import numpy as np

__all__ = ["FrameRate", "to_gray", "to_gray_frame", "FramePoller", "LatestFrameWorker"]

# ------------------------------------------------------------------ 帧率
class FrameRate:
    """滑动窗口帧率计。按"成功完成的事件时刻"计时；返回 (平均fps, low1)。

    三处复用（各自独立实例，口径不同、切勿混读）：
        capture_rate —— 主循环 cam.read() 速率   → [perf] capture=
        preview.rate —— 显示 show() 完成速率      → [perf] disp= / 屏幕文字 disp
        worker.rate  —— 识别帧处理完成速率        → [perf] detect_fps=

    low1 = "最慢 1% 帧间隔的倒数"：用于暴露**卡顿**——平均值好看但偶发长停顿
    （如显示阻塞）时，fps 可能仍高，而 low1 会被那几帧长间隔拉低，比平均值更诚实。
    """

    def __init__(self):
        self._times = deque(maxlen=120)   # 只留最近 120 个时刻，足够覆盖 2s 窗口

    def record(self, now):
        self._times.append(float(now))

    def values(self, now):
        ts = list(self._times)
        if len(ts) < 2:
            return 0.0, 0.0
        # 只看最近两秒内的时刻：统计窗口滚动，旧的读数随时间长自然淡出。
        # 关键：把"距上一个事件已过去的时间(stall)"也算作一次间隔 —— 否则一旦卡住，
        # 窗口里只剩早先的高速样本，会把上次的快读数永久挂在屏幕上。
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


# ------------------------------------------------------------------ 灰度
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


def to_gray_frame(gray):
    """校验输入并去掉 (H,W,1) 哑轴。返回 (H,W) uint8。

    只做格式规整，不做任何增强——真正的灰度转换在 `to_gray`。
    这里抛 `ValueError` 而不是静默返回，是为了让"喂错数据"立刻暴露在调用点。
    """
    g = np.asarray(gray)
    if g.ndim == 3 and g.shape[2] == 1:
        g = g[:, :, 0]
    if g.ndim != 2 or g.dtype != np.uint8:
        raise ValueError("process expects a uint8 grayscale image")
    return g


# ------------------------------------------------------------------ 取帧
class FramePoller:
    """每帧一次：`cam.read()` → 可选 `lens_corr` → 灰度。

    `poll()` 返回 `(gray, captured_at, read_ms)`；`gray is None` 表示本次没取到帧
    （相机尚未就绪/被独占），由调用方决定是 `continue` 还是走 `idle()` 保活。

    `captured_at` 在 `read()` 返回后**立刻**取，不含镜头校正与灰度转换的耗时。
    """

    def __init__(self, camera, image_module, lens_corr=False, lens_strength=1.5, clock=None):
        self._camera = camera
        self._image = image_module
        self._lens_corr = bool(lens_corr)
        self._lens_strength = float(lens_strength)
        self._clock = clock if clock is not None else time

    def poll(self):
        started = self._clock.perf_counter()
        img = self._camera.read()
        if img is None:
            return None, None, (self._clock.perf_counter() - started) * 1000.0
        captured = self._clock.perf_counter()
        if self._lens_corr:
            img.lens_corr(strength=self._lens_strength)
        return to_gray(img, self._image), captured, (captured - started) * 1000.0


# ------------------------------------------------------------------ 识别线程
class LatestFrameWorker:
    """单识别线程 + 两个"最新槽位"：一个待处理帧槽(_pending)、一个结果槽(_latest)。

    线程模型（时间轴 B）：
        · 主线程（生产者）：submit() 往 _pending 放**最新一帧**的副本；
        · 识别线程（消费者）：取走 _pending → callback() 处理 → 结果写 _latest；
        · 主线程（读者）：snapshot() 无锁等待地取回 _latest（可能滞后于当前摄像头帧）。

    "最新优先、丢弃旧帧"是刻意的：识别跟不上时保留最新画面才有意义，积压旧帧只会让
    追踪越追越偏。Camera / Display 这些**必须独占主线程**的资源一概不跨线程。
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
            if snapshot.get('detector_result', snapshot.get('result', {})).get('valid'):
                self.valid_rate.record(completed)
            self._durations.append((completed - started) * 1000)

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
                    # 有帧就处理、没帧就保活：空闲时继续收 ACK，但不重发旧误差。
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
        """按需取有效识别吞吐和识别+控制 P95；不在逐帧热路径计算分位数。"""
        with self._condition:
            fps, low1 = self.valid_rate.values(now)
            durations = list(self._durations)
        return dict(valid_fps=fps, valid_low1=low1,
                    worker_p95_ms=float(np.percentile(durations, 95)) if durations else 0.)
'''

# ===== vision.algo  (vision/algo.py) =====
_MOD_SRC['vision.algo'] = r'''
"""② 算法层：纯函数 —— 输出只由入参决定，无状态、无 I/O、不读 config、不打日志。

四层之一，只有三类内容：

    几何  line_cross / quad_diagonal_center / order_quad / project_points
          rect_quad / poly_area / edge_lengths / valid_quad
    采样  sample_points / profile_samples / refine_sample_points
    偏差  deviation（靶心 − 光斑，纯算术）

链路核心（为什么这样组织，每一条都对应板上的真实教训）：

1. **板上一次 Python 级 numpy/cv2 调用 ≈ 1ms，单帧耗时 ≈ 调用次数**（不是像素数）。
   热路径只允许整批一次的大图 cv2 调用（缩放/阈值/形态学/轮廓/remap），
   以及 `tolist()` 之后的小数组纯 Python 循环（求交/取中位/判门）。
2. **黑框是"一定物理宽度的暗带围出一块亮面"**。`profile_samples` 把物理矩形边
   按真实带宽（`BLACK_FRAME_MM`）向内外各取两点：暗带两点都须暗、纸面一点须明显更亮。
   带宽因此进入判据，细边框（显示器窄边、印刷细环）过不了；洞洞板、门框这类
   "没有闭合暗带"的结构也在覆盖率门上出局。
3. **中心取纸边（内框）对角线交点**。胶带上下不等宽时外框中心会偏出半个带宽，
   纸边不会；透视下每条边仍是直线，四线交角是几何精确解，不需要逐帧 warp。
4. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "line_cross", "quad_diagonal_center", "poly_area", "edge_lengths", "order_quad",
    "project_points", "rect_quad", "valid_quad",
    "sample_points", "profile_samples", "refine_sample_points",
    "deviation",
]


# ================================================================== 几何
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
    """对角线交点 = 透视下的矩形中心。退化时返回 None，由调用方判失败。

    纯 Python（板上每次 numpy 调用 ~1ms，四点算术不值得付这个钱）。
    """
    q = [(float(p[0]), float(p[1])) for p in quad]
    if len(q) != 4:
        return None
    p = line_cross(q[0], q[2], q[1], q[3])
    if p is None or not (math.isfinite(p[0]) and math.isfinite(p[1])):
        return None
    return p


def poly_area(quad):
    """有向面积（图像坐标 y 向下时，顺时针为负）。用于判四边形绕向。"""
    return float(cv2.contourArea(np.asarray(quad, np.float32).reshape(-1, 1, 2), oriented=True))


def edge_lengths(quad):
    """四条边的长度，按顶点循环顺序返回。"""
    q = np.asarray(quad, np.float32).reshape(4, 2)
    return np.linalg.norm(np.roll(q, -1, axis=0) - q, axis=1)


def order_quad(points):
    """把四点整理成循环顺序，并把 x+y 最小的点转到首位。

    不强制图像的 TL/TR/BR/BL —— 框转过 45° 后"左上"不再是物理角点；
    只保证四点在环上相邻，后续 edges[i] 与 edges[i+1] 一定是物理相邻边。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    center = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0]))]
    if poly_area(p) < 0:
        p = p[::-1]
    return np.roll(p, -int(np.argmin(p[:, 0] + p[:, 1])), axis=0).copy()


def project_points(matrix, points):
    """单应映射一批点；输入输出都是 (N,2)。"""
    p = np.asarray(points, np.float32).reshape(1, -1, 2)
    return cv2.perspectiveTransform(p, np.asarray(matrix, np.float64)).reshape(-1, 2)


def rect_quad(width, height, inset=0.0):
    """物理矩形的四角（mm），inset 为向内收缩量；顺序 TL/TR/BR/BL。"""
    return np.array([[inset, inset], [width - inset, inset],
                     [width - inset, height - inset], [inset, height - inset]], np.float32)


def valid_quad(quad, shape, min_side, min_area, allow_border=False, min_sine=0.16):
    """四边形合法性门：不出画、凸、最短边够长、相邻边不共线、面积够大。

    min_sine 是相邻边夹角的正弦下限（0.16 ≈ 9°），挡"几乎退化成一条线"的提案。
    阈值由调用方传入，本层不读 config。
    """
    q = np.asarray(quad, np.float32).reshape(4, 2)
    h, w = shape[0], shape[1]
    margin = 1 if allow_border else 2
    if (q[:, 0].min() < margin or q[:, 1].min() < margin or
            q[:, 0].max() >= w - margin or q[:, 1].max() >= h - margin):
        return False
    if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
        return False
    e = np.roll(q, -1, axis=0) - q
    lengths = np.linalg.norm(e, axis=1)
    if lengths.min() < float(min_side):
        return False
    cross = np.abs(e[:, 0] * np.roll(e[:, 1], -1) - e[:, 1] * np.roll(e[:, 0], -1))
    if np.min(cross / (lengths * np.roll(lengths, -1) + 1e-6)) < float(min_sine):
        return False
    return abs(poly_area(q)) >= float(min_area)


# ================================================================== 采样
def sample_points(gray, points):
    """按亚像素坐标批量采样灰度（一次 remap 取全部点，避免 Python 级逐点索引）。"""
    p = np.asarray(points, np.float32).reshape(-1, 2)
    return cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(points.shape[:-1])


def profile_samples(kind, width, height, tape, t, paper_inset=4.0):
    """构造"暗带两点 + 纸面一点"的物理采样坐标，返回 (4边, len(t), 3, 2)。

    kind="inner"：入参是纸边（内框）四角，暗带在纸面外侧，故暗带采样沿外法线；
    kind="outer"：入参是胶带外沿四角，暗带在内侧，纸面见证点再往内让一个带宽。
    两种情形下"纸面见证点"永远比暗带点更深地落在靶纸内部，判据方向因此一致。

    纯几何、可缓存：同一 (kind, 物理尺寸, 带宽) 的采样模板整段会话只算一次。
    """
    boundary = rect_quad(width, height, tape if kind == "inner" else 0.0)
    edges = np.roll(boundary, -1, axis=0) - boundary
    if np.linalg.norm(edges, axis=1).min() < 1:
        return None
    center = np.array([width * 0.5, height * 0.5], np.float32)
    mid = (boundary + np.roll(boundary, -1, axis=0)) * 0.5
    inward = center[None, :] - mid
    inward /= np.maximum(np.linalg.norm(inward, axis=1, keepdims=True), 1e-6)
    base = (boundary[:, None, :] * (1 - t[None, :, None]) +
            np.roll(boundary, -1, axis=0)[:, None, :] * t[None, :, None])
    if kind == "inner":
        dark_offsets = (-0.35 * tape, -0.72 * tape)   # 负号 = 沿外法线离开纸面
        paper_offsets = float(paper_inset)
    else:
        dark_offsets = (0.35 * tape, 0.72 * tape)
        paper_offsets = float(tape) + float(paper_inset)
    out = np.empty((4, len(t), 3, 2), np.float32)
    out[:, :, 0, :] = base + inward[:, None, :] * dark_offsets[0]
    out[:, :, 1, :] = base + inward[:, None, :] * dark_offsets[1]
    out[:, :, 2, :] = base + inward[:, None, :] * paper_offsets
    return out


def refine_sample_points(quad, t, radius):
    """构造沿边法线的窄带采样点，返回 (points, base, normals, offsets)。

    points 形如 (4边, len(t), 2*radius+1, 2)：每条边每个采样位置取一排灰度，
    梯度极值即亚像素边位置；base/normals 供调用方把位移还原成图像坐标。
    """
    q = np.asarray(quad, np.float32).reshape(4, 2)
    nxt = np.roll(q, -1, axis=0)
    center = q.mean(axis=0)
    normals = center[None, :] - (q + nxt) * 0.5
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-6)
    base = q[:, None, :] * (1 - t[None, :, None]) + nxt[:, None, :] * t[None, :, None]
    offsets = np.arange(-radius, radius + 1, dtype=np.float32)
    points = base[:, :, None, :] + normals[:, None, None, :] * offsets[None, None, :, None]
    return points, base, normals, offsets


# ================================================================== 偏差
def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定：dx = u靶心 − u光斑，与后续串口直控云台同一套符号：
    电机方向由各轴 DIR_SIGN 映射；像素误差为正不代表固定的电机方向。

    use_raw：闭环取这一档（True）—— 取 result["raw_u"]/["raw_v"]。
    result["u"]/["v"] 是显示用的平滑中心；闭环用 raw 是为了不引入相位滞后。
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
    if not all(map(math.isfinite, (tx, ty, lu, lv))):
        return None
    size = result.get('image_size')
    if size is not None and not (0 <= lu < size[0] and 0 <= lv < size[1]):
        return None
    return (tx - lu, ty - lv)       # dx=靶心−光斑, dy 同理
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

# ===== vision.zdt  (vision/zdt.py) =====
_MOD_SRC['vision.zdt'] = r'''
"""张大头（ZDT）Emm_V5.0 闭环步进驱动器 —— 设备驱动层。

本文件是**唯一**知道"电机怎么说话"的地方：协议帧怎么拼、两条串口怎么开、
返回帧怎么切、ACK/E2 怎么统计、运动时间怎么估。上层（云台决策、识别追踪）
只调用 ``StepperLink.enable()`` / ``move()`` / ``speed()`` / ``stop()`` / ``poll()``，
不关心字节序、校验字节和串口设备名。

**为什么单独成文件**：驱动是"设备方言"，和画图、显示、日志没有关系。原先两者
合在 ``output.py`` 里，改一个显示配色要翻电机协议，改协议又怕碰坏显示。分开之后
驱动可以单独回归（``tools/test_gimbal_actions.py`` 用假链路跑动作时序）。

层次位置：在 ``algo`` 之上、``output`` 之下。

    依赖方向：poll ← algo ← zdt ← output ← logic

本文件**不 import** 任何其它 vision 模块，也不在顶层 import maix
（``StepperLink`` 构造时才按需 import uart/pinmap），所以 PC 上能直接导入。

帧格式（Emm_V5.0，手册 p.36-38）::

    | addr | func | data ... | 0x6B |

    F3 使能    AB + 使能位 + 同步位
    FD 位置    方向 + 转速(2B) + 加减速档 + 脉冲数(4B) + 绝对/相对 + 同步位
    FE 立即停  98 + 同步位
    F6 速度    方向 + 转速(2B) + 加减速档 + 同步位

⚠ 驱动器菜单三项必须先改：``P_Serial → UART_FUN``（出厂 ``RxTx_OFF``，不改则串口
完全不响应）、``Checksum = 0x6B``、两轴 ``ID_Addr`` 分别为 1 / 2。写串口不校验
对端是否存在，这几项没配好时代码这边**看不出任何异常**，只是电机不动。
"""
import math
import time

__all__ = ["CR", "STATUS_OK", "STATUS_E2",
           "cmd_enable", "cmd_speed", "cmd_position", "cmd_stop_now",
           "frame_status", "estimate_move_seconds", "speed_profile", "StepperLink"]

# ZDT Emm_V5.0 帧头校验字节与状态码（手册 p.36-38）
CR = 0x6B
STATUS_OK = 0x02
STATUS_E2 = 0xE2        # 未使能 / 堵转类故障


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


def frame_status(frame):
    """返回帧第 3 字节 = 命令状态；帧太短返回 None。"""
    return int(frame[2]) if frame is not None and len(frame) >= 3 else None


def estimate_move_seconds(pulses, rpm, acc=0, pulses_per_rev=3200):
    """ZDT 的梯形/三角速度模型：估算走完指定脉冲需要多久（秒）。"""
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


def speed_profile(deg_s, ramp_s):
    """把"电机轴角速度 + 起停斜坡时间"换算成驱动器要的 (rpm, acc)。

    ZDT 的 acc 不是物理加速度，而是 0~255 的档位：0 = 瞬时启停，255 = 最缓。
    手册的斜坡模型是 ``t_ramp = rpm × (256 - acc) × 50µs``，这里反解 acc。

    ⚠ 低转速时斜坡时间有上限（rpm 越小，能拉出的斜坡越短）：算出来 acc < 1
    就取 1（最缓档），此时实际斜坡是 ``rpm × 255 × 50µs``，比要求短。
    这不是 bug，是驱动器本身的量化；想要更长的软启动只能提高目标转速。
    """
    deg_s = max(0.0, float(deg_s))
    rpm = int(round(deg_s * 60.0 / 360.0))
    rpm = max(1, min(5000, rpm))
    ramp_s = max(0.0, float(ramp_s))
    if ramp_s <= 0:
        return rpm, 0                     # 0 = 不加速（瞬时启停），仅调试用
    acc = int(round(256.0 - ramp_s / (rpm * 50e-6)))
    return rpm, max(1, min(255, acc))


def _readback_pin(pinmap, pin):
    """回读引脚当前实际功能（"某轴不通"的第一手证据）。失败返回 '?'。"""
    try:
        return pinmap.get_pin_function(pin)
    except Exception:
        return "?"


class StepperLink:
    """两条独立 UART 链路；任一轴打开失败不会阻塞另一轴。

    对外只暴露五个动作：``enable(axis=None)`` / ``move(axis, ccw, pulses, rpm, acc)`` /
    ``speed(...)`` / ``stop(axis=None)`` / ``poll(now)``，外加 ``stats()`` / ``close()``。
    上层不需要知道帧长什么样。
    """

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
        # ★ ACK 看门狗必须**按轴独立武装**（板端实锤）：
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
            print("[gimbal] maix uart/pinmap 不可用，双轴链路关闭: %s" % exc)
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

    def move_smooth(self, axis, ccw, pulses, deg_s, ramp_s=0.12, now=None):
        """按"角速度 + 起停斜坡"走一段相对位置，返回 ``(ok, rpm, acc)``。

        ``deg_s`` 是**电机轴**角速度（度/秒），不是输出轴；有减速比时传
        ``输出角速度 × 减速比``。驱动器自己按梯形曲线加速—匀速—减速，
        所以一条指令就是一次完整的平滑动作，不需要上层补小步。
        """
        rpm, acc = speed_profile(deg_s, ramp_s)
        ok = self.move(axis, ccw, pulses, rpm, acc, now=now)
        return ok, rpm, acc

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
            # 兼容用 __new__ 构造的轻量实例（离线解析），与上面的 _pending 同口径。
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
'''

# ===== vision.output  (vision/output.py) =====
_MOD_SRC['vision.output'] = r'''
"""④ 执行层：**唯一允许产生副作用的地方** —— 画布、屏幕、串口、脉冲、停机、日志。

四层之一，两类内容合成一文件：看得见的 + 会动的。

    画面  draw_result / draw_laser_overlay / display_result / aligned_frame
          Preview（画布池 + 独立显示线程，show() 慢也只掉预览帧）
          draw_preview（组帧 → 投递渲染）
    云台  stop_motion / apply_commands（把逻辑层的决策真正下发）
          协议帧与双串口链路在 `vision/zdt.py`，本层再导出以兼容旧调用点
    日志  perf_line（[perf] 行拼装）

三条约束：

1. **只接收值，不回查状态**：本层不 import 逻辑层，也不读 `aim.state`；
   故障判定由逻辑层做，本层只执行并如实返回成功/失败。
2. **显示失败绝不升级**：任何绘制/转换异常只打日志并置 `Preview.enabled=False`，
   不影响识别、串口与闭环。
3. **停机失败必须回报**：`stop_motion` 返回 False 表示停机没成功，调用方必须终止决策链。

依赖方向：本层 import 轮询层（FrameRate）与算法层（deviation）；顶层不 import maix，
`StepperLink.__init__` 里才 import —— 所以能在 PC 上用假链路跑回归。
"""
import math
import os
import threading
import time

import cv2
import numpy as np

from .algo import deviation
from .poll import FrameRate
from .zdt import (CR, STATUS_E2, STATUS_OK, StepperLink, cmd_enable,
                  cmd_position, cmd_speed, cmd_stop_now, estimate_move_seconds,
                  frame_status)

__all__ = ["draw_result", "draw_laser_overlay", "PANEL_H", "display_result",
           "aligned_frame", "draw_preview", "Preview", "perf_line",
           "cmd_enable", "cmd_position", "cmd_stop_now", "frame_status",
           "estimate_move_seconds", "StepperLink", "stop_motion", "apply_commands"]

# ------------------------------------------------------------------ 激光面板样式
_PANEL_BG = (32, 32, 32)
PANEL_LINE = 18
PANEL_H = 2 * PANEL_LINE + 10   # 面板高度（画布坐标）；draw_result 用它把文字整体下移
_TEXT_C1 = (255, 255, 0)     # BGR 青：光斑状态行
_TEXT_C2 = (120, 255, 120)   # BGR 浅绿：偏差行（有效）
_TEXT_C3 = (120, 120, 255)   # BGR 浅红：偏差行（无靶纸）
_MARK = (0, 0, 255)          # BGR 红：光斑标记（跟激光同色）
_LINK = (0, 160, 255)        # BGR 橙：光斑->靶心连线


# ------------------------------------------------------------------ 画框
def _draw_build(canvas, result, top_offset=0):
    """右上角打 BUILD_ID 短标识：不接串口也能确认板子上跑的是哪一版。

    BUILD_ID 形如 `1.9.0-2026-11-12`，屏上只显示首段版本号（`v1.9.0`），
    完整串看串口日志。若仍遇到旧的 `YYYY-MM-DD-<slug>-vN` 格式，退回显示末段。
    """
    tag = str(result.get("build") or "")
    if not tag:
        return
    head = tag.split("-", 1)[0]
    if "." in head:
        short = head
    else:
        short = tag.split("-", 3)[-1] if tag.count("-") >= 3 else tag
    text = "v" + short
    (tw, _th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .38, 1)
    cv2.putText(canvas, text, (max(4, canvas.shape[1] - tw - 8), 18 + top_offset),
                cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 220, 255), 1)


def draw_result(gray, result, show_width=480, top_offset=0):
    """把一帧结果画成 BGR 画布。坐标一律按 scale 换算，文字仍报原图坐标。

    配色语义（三条线各管一件事，别再混着画）：
        绿 2px   `detected_quad` —— 本帧**实际锁定**的轮廓，永远贴合画面里的边；
        青 1px   `inner_quad`    —— 纸边，靶心就是它的对角线交点；
        洋红 1px `quad`          —— 按物理带宽**外推**的胶带外沿，只作参考。
    把"实测边"和"外推边"画成同一条绿线，会在胶带宽度与 `BLACK_FRAME_MM` 不符时
    让画面看起来像"框飘了"，而中心其实是对的 —— 所以两者必须分开显示。
    """
    g = np.asarray(gray).squeeze()
    scale = min(1.0, float(show_width) / max(1, g.shape[1]))
    if scale < 1:
        g = cv2.resize(g, (int(round(g.shape[1] * scale)), int(round(g.shape[0] * scale))))
    canvas = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

    if not result.get("valid"):
        reason = result.get("fail_reason") or result.get("stage") or "-"
        cv2.putText(canvas, "NO TARGET  " + str(reason), (8, 24 + top_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 1)
        _draw_build(canvas, result, top_offset)
        return canvas

    detected = result.get("detected_quad")
    if detected is None:
        detected = result["quad"]
    q = np.rint(np.asarray(detected, np.float32) * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    for x, y in q:
        cv2.circle(canvas, (int(x), int(y)), 3, (255, 0, 255), -1)

    # 外推外框：与实测边不同才画，避免两条线叠在一起看着像粗边。
    outer = result.get("quad")
    if outer is not None and not np.allclose(np.asarray(outer, np.float32),
                                             np.asarray(detected, np.float32), atol=0.5):
        oq = np.rint(np.asarray(outer, np.float32) * scale).astype(np.int32)
        cv2.polylines(canvas, [oq], True, (255, 0, 255), 1, cv2.LINE_AA)

    # 内框（纸边）：定心用的就是它，画出来便于核对是否贴合。
    inner = result.get("inner_quad")
    if inner is not None:
        iq = np.rint(np.asarray(inner, np.float32) * scale).astype(np.int32)
        cv2.polylines(canvas, [iq], True, (255, 200, 0), 1, cv2.LINE_AA)

    c = (int(round(result["u"] * scale)), int(round(result["v"] * scale)))
    cv2.drawMarker(canvas, c, (255, 0, 0), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(canvas, "(%d,%d) area=%d aspect=%.2f %.1fms" %
                (round(result["u"]), round(result["v"]), int(result.get("area", 0)),
                 float(result.get("aspect", 0.)), result.get("detector_ms", 0.)),
                (8, 22 + top_offset), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 220, 0), 1)
    _draw_build(canvas, result, top_offset)
    return canvas


# ------------------------------------------------------------------ 激光面板
def _deviation_for_draw(laser, result, use_raw):
    """面板用的偏差 —— 与闭环同一口径（同一 use_raw），避免屏幕与控制看到的不是同一个数。"""
    return deviation(laser, result, use_raw=use_raw)


def draw_laser_overlay(canvas, laser, result=None, scale=1.0, use_raw=False):
    """在显示画布左上角画激光读数面板 + 光斑标记 + 到靶心的连线。

    use_raw 与闭环保持一致：True 时连线终点与 dx/dy 都用未平滑的 raw 中心，
    屏幕上看到的就是控制律真正在用的那个点。

    scale：**原图坐标 → 画布坐标** 的比例（main.py 传 render_width/原图宽）。
    面板本身用画布固定像素坐标，不随 scale 缩放。

    返回面板占用的高度（画布坐标），调用方据此把 draw_result 的文字整体下移。
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
    dv = _deviation_for_draw(laser, result, use_raw)
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


# ------------------------------------------------------------------ 快照翻译
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
        return snapshot.get('frame', live_gray)
    return live_gray


def _open_display(display_module, stream):
    """建 Display。`stream=True` 保留 MaixCDK 内置图传；False 在本进程内关掉它。

    内置图传在「连着 MaixVision 但连接未就绪」时会在 C++ 里忙等 500ms 并占住解释器锁，
    能把识别拖到 20 秒/帧。所以主程序在**导入 maix 之前**就把 `MAIXVISION` 置 0
    （见 `main.py`），图传改走独立子进程。这里的 set/restore 只是兜底：万一有别的入口
    没设，也保证构造 Display 这一刻是关的。
    """
    if stream:
        return display_module.Display()
    previous = os.environ.get("MAIXVISION")
    try:
        os.environ["MAIXVISION"] = "0"
        return display_module.Display()
    finally:
        if previous is None:
            os.environ.pop("MAIXVISION", None)
        else:
            os.environ["MAIXVISION"] = previous


# ------------------------------------------------------------------ 显示
class TouchButtons:
    """Three edge-triggered on-screen controls; touch is read on the camera thread."""

    LABELS = (('self_test', 'TEST'), ('search', 'SEARCH'), ('stop', 'STOP'))

    def __init__(self, width, height, touch=None):
        if touch is None:
            from maix import touchscreen
            touch = touchscreen.TouchScreen()
        self.touch = touch
        self.width, self.height = int(width), int(height)
        self._down = False
        self._down_at = 0.0
        self._flash = None
        self._flash_until = 0.0

    def _rects(self):
        gap, margin, height = 8, 8, 38
        width = min(104, max(65, (self.width-2*margin-2*gap)//3))
        return [(margin+i*(width+gap), self.height-height-margin, width, height)
                for i in range(3)]

    def poll(self, now):
        try:
            x, y, pressed = self.touch.read()
        except Exception as exc:
            self._down = False
            print('[touch] read failed: %s' % exc)
            return None
        if not pressed:
            self._down = False
            return None
        if self._down and now-self._down_at <= 2.0:
            return None
        self._down, self._down_at = True, now
        for (name, _), (rx, ry, rw, rh) in zip(self.LABELS, self._rects()):
            if rx <= x < rx+rw and ry <= y < ry+rh:
                self._flash, self._flash_until = name, now+0.25
                print('[touch] %s' % name)
                return name
        print('[touch] no button at %d,%d' % (x, y))
        return None

    def draw(self, canvas, now):
        scale_x = canvas.shape[1]/float(self.width)
        scale_y = canvas.shape[0]/float(self.height)
        for (name, label), (x, y, w, h) in zip(self.LABELS, self._rects()):
            x, y = round(x*scale_x), round(y*scale_y)
            w, h = round(w*scale_x), round(h*scale_y)
            active = name == self._flash and now < self._flash_until
            color = (45, 140, 45) if active else ((45, 45, 155) if name == 'stop' else (55, 55, 55))
            cv2.rectangle(canvas, (x, y), (x+w, y+h), color, -1)
            cv2.rectangle(canvas, (x, y), (x+w, y+h), (220, 220, 220), 1)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .48, 1)
            cv2.putText(canvas, label, (x+(w-tw)//2, y+(h+th)//2),
                        cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)


class Preview:
    """标注画面 →（letterbox）画布 → `display.show()`，show 跑在独立线程。

    ★ 线程模型（时间轴 C）
        主循环（生产者）  `render(view, ...)` —— letterbox 后把画布塞进"待上屏槽"就返回；
        显示线程（消费者） `_show_loop()`     —— 取最新画布 → `show()` → 画布还回池子。
        画布池固定 2 张：显示线程占 1 张、主线程画 1 张；两张都被占用时 `due()` 直接返回 False
        （丢帧，不排队、不等待）。因此 **show() 再慢也只表现为预览掉帧**，绝不拖慢主循环、
        识别线程与云台闭环。`SHOW_ASYNC=False` 时退回主循环内同步 `show()`。

    尺寸口径（`render()` 里同时出现，最易混淆）：
        · display.width/height  —— 物理屏幕尺寸
        · render_width/height   —— 实际送去 show() 的**画布**尺寸（SHOW_WIDTH 限制）
    默认 `SHOW_WIDTH=0` ⇒ 画布 == 屏幕：`show()` 不再缩放。相机 640×480 与
    MaixCAM2 屏幕 640×480 宽高比一致时，letterbox 退化成零开销。

    `SHOW=False` 时不建 Display；无显示 / 建失败 / show 失败只打日志并停用，
    绝不影响识别与闭环。
    """

    def __init__(self, cfg, clock=None):
        self.cfg = cfg
        self._clock = clock if clock is not None else time
        self.display = None
        self.stream_obj = None                  # MaixVision 图传（独立进程，可为 None）
        self.width = self.height = 0            # 物理屏幕尺寸
        self.render_width = self.render_height = 0
        self.enabled = False                    # 显示是否可用；任何一步失败都置 False 并停用
        self.shown = 0                          # 已成功 show() 的帧数（disp 口径）
        self.dropped = 0                        # 因显示线程忙而未画的帧数（丢帧，不排队）
        self.fps = 0.0
        self.low1 = 0.0
        self.rate = FrameRate()                 # 显示帧率（disp 口径，与识别帧率分开）
        self._period = 1.0 / max(1, int(getattr(cfg, "SHOW_FPS", 15)))   # 显示节拍周期
        self._t_show = -1e9                     # 下一次该画的时刻（固定时间轴）
        self._canvas = None                     # 同步模式的单张画布
        self._show_times = []                   # 本日志窗口内的 show() 耗时样本（ms）
        self._show_max = 0.0
        self._slow_warned = False               # "显示侧慢"告警只打一次，避免每 2s 刷屏
        # ---- 显示线程状态 ----
        self.async_show = bool(getattr(cfg, "SHOW_ASYNC", True))
        self._cond = threading.Condition()      # 保护 _free / _pending / _pool / _stop
        self._free = []                         # 空闲画布池
        self._pending = None                    # 待上屏画布（只留最新一张）
        self._pool = 0                          # 已分配画布数（上限 2，由 due() 保证）
        self._stop = False
        self.thread = None
        self.buttons = None
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口模式，画面不流转）")
            return
        try:
            from maix import display
            self.stream_on = bool(getattr(cfg, "SHOW_STREAM", True))
            # 本地屏一律用 MAIXVISION=0 建 Display：内置图传在"连接未就绪"时会在 C++ 里
            # 忙等 500ms 并占住解释器锁，把识别拖到 20 秒/帧。要 MaixVision 画面就走
            # 下面的独立进程（vision/maixvision_stream.py），它不碰主线程。
            self.display = _open_display(display, False)
            self.width = int(self.display.width())
            self.height = int(self.display.height())
            # 画布宽度：SHOW_WIDTH<=0 ⇒ 用屏幕原始宽度（画布＝屏幕 ⇒ show() 零缩放）。
            cap = int(getattr(cfg, "SHOW_WIDTH", 0) or 0)
            self.render_width = self.width if cap <= 0 else min(self.width, cap)
            self.render_height = max(1, int(round(self.height * self.render_width / self.width)))
            if hasattr(display, "set_trans_image_quality"):
                display.set_trans_image_quality(int(getattr(cfg, "SHOW_JPEG_QUALITY", 70)))
            self.enabled = True
            print("[preview] mode=%s screen=%dx%d canvas=%dx%d show_fps=%d stream=%s"
                  % ("display-thread" if self.async_show else "sync-inprocess",
                     self.width, self.height, self.render_width, self.render_height,
                     int(getattr(cfg, "SHOW_FPS", 30)), "on" if self.stream_on else "off"))
            # ★ 开机探针：立刻量一次 show() 真实耗时。本地屏典型 1~5ms；若命中 MaixCDK 的
            #   "MaixVision 图传未就绪"分支，send_image() 会在 C++ 里**忙等 500ms 并空转 CPU**
            #   （见 MaixCDK components/vision/src/maix_image_trans.cpp），表现为 show 恒定
            #   ~505ms、并连带把相机/识别/闭环一起拖慢。这条必须当场说清。
            self._probe_show()
            if self.stream_on:
                try:
                    from .maixvision_stream import MaixVisionStream
                    self.stream_obj = MaixVisionStream(cfg, self.render_width,
                                                       self.render_height, clock=self._clock)
                    print("[stream] MaixVision 图传已启动（独立进程，%dx%d）%s"
                          % (self.render_width, self.render_height,
                             "" if getattr(self.stream_obj, "enabled", False) else " —— 未启用"))
                except Exception as exc:
                    self.stream_obj = None
                    print("[stream] 图传启动失败（只影响 MaixVision 画面）: %s" % exc)
            if self.async_show:
                self.thread = threading.Thread(target=self._show_loop, name="display", daemon=True)
                self.thread.start()
        except Exception as exc:
            self.enabled = False
            # 建不了显示不算错误：板子仍照常识别 + 跑闭环，只是没有画面。
            print("[preview] 显示初始化失败（不影响识别/闭环，仅无预览）: %s" % exc)

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
        """把标注后的 `view`（任意尺寸）letterbox 到画布、叠状态条，然后交给显示线程。"""
        if not self.enabled:
            return
        now = float(now)
        # ★ 固定时间轴限频：节拍由"上一次预期时刻 + 周期"推进，而不是"show 完成后再等一周期"。
        #   好处：即使某次 show() 慢，也不会让后续节拍越拖越后；已经错过的节拍一次性跳过。
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
        if status:
            # 左下角状态串（aim 状态机名 / HOLD / FAULT）。
            status_y = self.render_height - (58 if self.buttons is not None else 8)
            cv2.putText(canvas, status, (8, status_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        if self.buttons is not None:
            self.buttons.draw(canvas, now)
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
        """显示线程主体：取最新画布 → show() → 画布还池。**慢也只慢自己**。"""
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
        `ImageTrans::send_image()` 在「处于 MaixVision 模式、但图传连接未就绪」时，
        会忙等 500ms 且空转 CPU —— 这正是本项目实测 show 恒定 ~505ms、
        disp≈1.9fps 且相机与识别一起变慢的根因。
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
        if ms >= 300.:
            print("[preview] show 探针 %.0fms：本地显示偏慢；检查 SDK 是否复用了旧图传对象。" % ms)
            print("[preview] 对策：确认 SHOW_STREAM 没被关掉；或从设备本地启动对照本地屏耗时。")
        else:
            print("[preview] show 探针 %.1fms —— 显示链路健康（本地屏典型 1~5ms）" % ms)
        return ms

    def warn_slow_once(self, p95):
        """显示侧变慢只提示一次（每 2s 窗口重复报会刷屏）；点明最可能的根因。"""
        if self._slow_warned:
            return
        self._slow_warned = True
        print("[preview] show slow: p95=%.0fms；检查本地屏驱动和 SDK 的图传对象缓存。" % p95)

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
        if self.stream_obj is not None:
            try:
                self.stream_obj.submit(canvas)
            except Exception as exc:
                print("[stream] 投递失败，停用图传: %s" % exc)
                self.stream_obj.close()
                self.stream_obj = None
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
        if self.stream_obj is not None:
            try:
                self.stream_obj.close()
            except Exception:
                pass
            self.stream_obj = None
        if self.display is not None:
            try:
                self.display.close()
            except Exception:
                pass


# ================================================================== 电机驱动
# 协议帧、双串口收发、ACK/E2 看门狗全部在 `vision/zdt.py`（设备驱动层）。
# 本层只保留"把决策真正下发"的 stop_motion / apply_commands，并把驱动符号
# 再导出，旧调用点（logic.py / target_api.py / tools）无需改动。

# ================================================================== 下发
def stop_motion(link, aim, now, axis=None):
    """停机。`axis=None` 停两轴。返回 False 表示停机失败（已置 FAULT）。"""
    if link is None:
        return True
    try:
        ok = link.stop(now=now) if axis is None else link.stop(axis, now=now)
    except Exception:
        ok = False
    if ok is False:
        if axis is not None:
            # 单轴停机失败 -> 兜底再停另一轴，避免云台卡在一侧持续出力。
            try:
                link.stop('pitch' if axis == 'yaw' else 'yaw', now=now)
            except Exception:
                pass
        if aim is not None:
            aim.fault('stop:'+(axis or 'all'))
        return False
    if aim is not None and hasattr(aim, 'mark_stopped'):
        aim.mark_stopped(now, axis)
    return True


def apply_commands(link, aim, commands, now, acc_track):
    """把 `{'yaw': (ccw, pulses, rpm, acc)|None, 'pitch': ...}` 真正下发。

    返回 `(ok, failed_axis, applied)`：

        ok          全部下发成功为 True；任一失败即 False（此时已停机并置 FAULT）
        failed_axis 失败的轴名，成功为 None
        applied     实际发出去的 `{axis: command}`，供逻辑层回填 `self.commands`

    每条指令自带 acc（起停斜坡），不再全局写死：远距离和近距离可以用不同的
    斜坡时间，速度曲线才不会一顿一顿。命令缺第 4 位时退回 `acc_track`。

    顺序：按 dict 迭代顺序逐个 `link.move`，首个失败立即停机返回。
    """
    applied = {}
    if link is None:
        return True, None, applied
    for axis, command in commands.items():
        if command is None:
            continue
        ccw, pulses, rpm = command[:3]
        acc = command[3] if len(command) > 3 else acc_track
        try:
            ok = link.move(axis, ccw, pulses, rpm, acc, now=now)
        except Exception:
            ok = False
        if ok is False:
            stop_motion(link, aim, now)
            if aim is not None:
                aim.fault('tx:'+axis)
            return False, axis, applied
        applied[axis] = command
    return True, None, applied

# ================================================================== 日志与预览
def perf_line(build_id, capture_fps, display_fps, display_low1, vision_fps,
               result, read_avg, draw_avg, loop_avg,
               show_avg, show_p95, show_max, show_n,
               age_s, shown, drop_show, drop_vision, detector_result=None):
    """拼装 [perf] 行。三路 fps 是**不同**口径：眼睛看到的看 disp，识别吞吐看 detect_fps。"""
    det = result if detector_result is None else detector_result
    fail_stage = det.get("fail_stage")
    fail_tag = "-" if not fail_stage else "%s/%s" % (fail_stage, det.get("fail_reason", "-"))
    split = det.get("t_ms") or {}
    split_tag = ""
    if split:
        split_tag = " det_split=%.1f/%.1f/%.1f" % (
            split.get("prep", 0.0), split.get("comp", 0.0), split.get("quad", 0.0))
    return ("[perf] %s capture=%.1f disp=%.1f low1=%.1f detect_fps=%.1f "
            "valid=%s stage=%s | read=%.1f draw=%.1f loop=%.1f ms | "
            "show avg=%.1f p95=%.1f max=%.1fms n=%d | "
            "age=%.0fms shown=%d drop(show)=%d drop(vision)=%d" %
            (build_id, capture_fps, display_fps, display_low1, vision_fps,
             result.get("valid", False), result.get("stage"),
             read_avg, draw_avg, loop_avg,
             show_avg, show_p95, show_max, show_n,
             age_s * 1000, shown, drop_show, drop_vision)) + (
            " det_valid=%s det_stage=%s det_ms=%.1f det_masks=%d det_fail=%s det_props=%d det_band=%d" %
            (det.get("valid", False), det.get("stage"), det.get("detector_ms", 0.),
             det.get("masks", 0), fail_tag, det.get("proposals", 0),
             int((det.get("band") or {}).get("contrast") or 0))) + split_tag

def draw_preview(cfg, preview, gray, result, laser, age, fps, status, now):
    view = draw_result(gray, result, preview.render_width, top_offset=PANEL_H if laser else 0)
    if laser is not None:
        draw_laser_overlay(view, laser, result,
                           min(1., preview.render_width/float(gray.shape[1])),
                           use_raw=cfg.AIM_USE_RAW)
    # Keep the performance line above Preview's bottom status line.
    cv2.putText(view, 'disp %.1f low1 %.1f det %.1f age %.0fms' % (preview.fps, preview.low1, fps, age*1000),
                (8, view.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX, .40, (0, 255, 255), 1)
    preview.render(view, now, status=status)
'''

# ===== vision.logic  (vision/logic.py) =====
_MOD_SRC['vision.logic'] = r'''
"""③ 逻辑层：流程编排、状态机、对外接口 —— `main.py` 的业务本体就在这里。

四层之一，装"决定该做什么"的全部代码，**一行副作用都不做**：

    结果契约   fail_result / finish_result
    检测器     RectangleCenterDetector（环-洞结构 + ROI 跟踪）
               LaserSpotDetector（光斑读数）
    闭环决策   GimbalAim（BOOT/COARSE/FINE/SETTLE/HOLD/FAULT）
               VisionControl（识别 → 偏差 → 决策；命令只填进 `self.commands`）
    主循环     `_capture_loop`（三条时间轴调度）/ `run`（装配）/ `_release`（按序释放）
    命令行入口 见根目录 `main.py`；对外脚本接口见根目录 `target_api.py`

依赖方向：**本层 import 轮询层、算法层、执行层**，反向一律禁止。
本层不直接产生副作用：

    绘制 → `output.draw_result` / `output.draw_preview` / `output.Preview`
    串口 → `output.stop_motion` / `output.apply_commands`

`link.move` / `link.stop` 在整份代码里只允许出现在执行层，逻辑层只产出
`{'yaw': (ccw, pulses, rpm) | None, 'pitch': ...}` 这样的决策值。

时序约定（不可改）：`stop_motion()` 返回 False 时**必须立刻终止**本轮决策链，
不能继续下发。"先停后走"与 ACK/E2 看门狗都依赖这一点。
"""
import math
import os
import threading
import time

import cv2
import numpy as np

from .algo import (deviation, edge_lengths, order_quad, profile_samples,
                   project_points, quad_diagonal_center, rect_quad,
                   refine_sample_points, sample_points, valid_quad)
from .output import (Preview, TouchButtons, aligned_frame, apply_commands, display_result,
                    draw_preview, perf_line, stop_motion)
from .poll import FramePoller, FrameRate, LatestFrameWorker, to_gray_frame
from .zdt import StepperLink, estimate_move_seconds, speed_profile

BUILD_ID = "1.11.1-2026-11-23"

# 主循环单轮超过该毫秒数（不含 show）即判为"慢"，打一次归因提示。
MAIN_LOOP_SLOW_MS = 66.0

# 结果阶段词汇（检测器写入 result["stage"]）
STAGE_PLANE = "plane_frame"
STAGE_NONE = "no_black_frame"


# ================================================================== 结果契约
def fail_result(stage, fail_stage, reason):
    """统一的失败结果。`fail_stage` 是走过的层，`reason` 是首次失败原因。"""
    return dict(valid=False, stage=stage, fail_stage=fail_stage, fail_reason=reason)


def finish_result(result, timestamp, image_size, started):
    """补齐时间戳、图像尺寸、耗时。"""
    result.update(captured_at=timestamp, image_size=image_size,
                  detector_ms=(time.perf_counter() - started) * 1000.0)
    return result


def _poly_area_py(quad):
    """四点多边形面积（纯 Python：板上每次 numpy 调用约 1ms，四点不值得付）。"""
    total = 0.0
    n = len(quad)
    for i in range(n):
        x1, y1 = float(quad[i][0]), float(quad[i][1])
        x2, y2 = float(quad[(i + 1) % n][0]), float(quad[(i + 1) % n][1])
        total += x1 * y2 - x2 * y1
    return abs(total) * 0.5


def _aspect_py(quad):
    """四边形长边/短边（>=1），纯 Python。仅作日志与显示的标量指标。"""
    n = len(quad)
    lengths = [math.hypot(float(quad[(i + 1) % n][0]) - float(quad[i][0]),
                          float(quad[(i + 1) % n][1]) - float(quad[i][1])) for i in range(n)]
    a = (lengths[0] + lengths[2]) * 0.5
    b = (lengths[1] + lengths[3]) * 0.5
    lo, hi = min(a, b), max(a, b)
    return float(hi / lo) if lo > 1e-6 else 0.0


# ================================================================== 编排
class RectangleCenterDetector:
    """黑框检测 + ROI 跟踪：低分辨率四边形提案 → 窄带黑带验证 → 当前帧四边精修。

    三段各自的理由（每一条都对应板上实测）：

    - **提案**：在 1/2 尺度上整幅一次 Otsu（无确认目标才退化到自适应阈值），
      连通域 + 多边形逼近出四边形。不用全图 Canny/Hough：板端一次 cv2 调用约 1ms，
      Hough 的参数扫描是几十次调用，帧率直接被吃掉。
    - **验证**：把物理矩形（纸面 + 胶带）按真实带宽向内、向外各取两点，
      要求"两点都暗、纸面一点明显更亮"，且四条边里至少三条覆盖达标。
      带宽因此进入判据 —— 细边框、印刷细环、洞洞板、门框在结构上就过不来。
    - **精修**：当前帧沿每条边取一排法线灰度，取梯度极值拟直线再求交，
      把提案阶段的降采样误差压到亚像素；精修不通过就退回提案值，不拿单帧失败换掉锁。
    - **中心取内框（纸边）对角线交点**：胶带上下不等宽时外框中心会偏出半个带宽，
      纸边不会。透视下每条边仍是直线，四线交角是几何精确解，不需要逐帧 warp。

    锁定后只在上帧框附近的 ROI 里重复同一套判据，失败才回全图重搜。
    """

    def __init__(self, config):
        self.cfg = config
        self.search_width = float(getattr(config, "SEARCH_WIDTH", 320))
        self.edge_samples = int(getattr(config, "EDGE_SAMPLES", 24))
        self.refine_radius = int(getattr(config, "REFINE_RADIUS", 6))
        self.track_radius = int(getattr(config, "TRACK_RADIUS", 12))
        self.min_contour_area = float(getattr(config, "MIN_CONTOUR_AREA", 350.0))
        self.min_quad_side = float(getattr(config, "MIN_QUAD_SIDE", 14.0))
        self.min_area_ratio = float(getattr(config, "MIN_AREA_RATIO", 0.025))
        self.max_area_ratio = float(getattr(config, "MAX_AREA_RATIO", 0.92))
        self.max_candidates = int(getattr(config, "MAX_CANDIDATES", 8))
        self.max_contours = int(getattr(config, "MAX_CONTOURS", 128))
        self.approx_epsilon = float(getattr(config, "APPROX_EPSILON", 0.025))
        self.max_line_residual = float(getattr(config, "MAX_LINE_RESIDUAL_PX", 2.0))
        self.adaptive_block = int(getattr(config, "ADAPTIVE_BLOCK_SIZE", 31))
        self.adaptive_c = float(getattr(config, "ADAPTIVE_C", 8.0))
        self.min_edge_coverage = float(getattr(config, "MIN_EDGE_COVERAGE", 0.55))
        self.weak_edge_coverage = float(getattr(config, "WEAK_EDGE_COVERAGE", 0.25))
        self.max_dark_ratio = float(getattr(config, "MAX_DARK_BRIGHT_RATIO", 0.72))
        self.min_edge_contrast = float(getattr(config, "MIN_EDGE_CONTRAST", 20.0))
        self.tape_mm = float(getattr(config, "BLACK_FRAME_MM", 18.0))
        self.aspect_min = float(getattr(config, "RECT_ASPECT_MIN", 1.05))
        self.aspect_max = float(getattr(config, "RECT_ASPECT_MAX", 3.40))
        self.short_mm = float(getattr(config, "TARGET_SHORT_MM", 210.0))
        self.long_mm = float(getattr(config, "TARGET_LONG_MM", 297.0))
        self.model_mm = tuple(getattr(config, "RECT_MODEL_MM", (297.0, 210.0)))
        self.roi_padding = float(getattr(config, "ROI_PADDING", 0.20))
        self.roi_min_padding = float(getattr(config, "ROI_MIN_PADDING", 24.0))
        self.min_roi_size = int(getattr(config, "MIN_ROI_SIZE", 32))
        self.center_alpha = float(getattr(config, "CENTER_ALPHA", 0.70))
        self.filter_reset = float(getattr(config, "FILTER_RESET_PX", 25.0))
        self.keep_stages = bool(getattr(config, "RECT_KEEP_STAGES", False))
        if self.tape_mm <= 0 or self.tape_mm * 2 >= self.short_mm:
            raise ValueError("BLACK_FRAME_MM must be less than half the short side")
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._sample_t = np.linspace(0.12, 0.88, max(6, self.edge_samples), dtype=np.float32)
        self._profile_cache = {}
        self._offset_cache = {}
        self.last_stages = {}
        self.last = None
        self.reset()

    def reset(self):
        """清空跟踪与平滑状态。换靶/重开一题/独立图片之间必须调用。"""
        self._track = None
        self._smooth = None
        self._frame = 0
        self._search_count = 0
        self._threshold_count = 0
        self._proposals = 0
        self._timing = dict(prep=0.0, comp=0.0, quad=0.0)

    # ------------------------------------------------------------- 内部工具
    def _valid_quad(self, q, shape, allow_border=False):
        return valid_quad(q, shape, self.min_quad_side, self.min_contour_area,
                          allow_border=allow_border)

    def _physical_layouts(self, q):
        """纸面横放还是竖放：两种对应关系都试，强透视会让图像边长次序反转。"""
        lengths = edge_lengths(q)
        pair_a = (lengths[0] + lengths[2]) * 0.5
        pair_b = (lengths[1] + lengths[3]) * 0.5
        short, long = self.short_mm, self.long_mm
        if pair_a >= pair_b:
            return ((long, short), (short, long))
        return ((short, long), (long, short))

    def _profile(self, gray, q, kind, width, height):
        """映射四边上的少量点：暗带两个采样点都须暗，框内的纸面须明显更亮。"""
        tape = self.tape_mm
        physical_boundary = rect_quad(width, height, tape if kind == "inner" else 0.0)
        try:
            phys_to_img = cv2.getPerspectiveTransform(physical_boundary, np.asarray(q, np.float32))
        except cv2.error:
            return None
        if not np.all(np.isfinite(phys_to_img)):
            return None
        key = (kind, width, height, tape)
        samples_phys = self._profile_cache.get(key)
        if samples_phys is None:
            samples_phys = profile_samples(kind, width, height, tape, self._sample_t)
            if samples_phys is None:
                return None
            self._profile_cache[key] = samples_phys
        mapped = project_points(phys_to_img, samples_phys.reshape(-1, 2)).reshape(samples_phys.shape)
        gh, gw = gray.shape
        if (mapped[..., 0].min() < 0 or mapped[..., 1].min() < 0 or
                mapped[..., 0].max() >= gw - 1 or mapped[..., 1].max() >= gh - 1):
            return None
        values = sample_points(gray, mapped).astype(np.float32)
        # 暗带两点取最大值：显示器窄边框的第二点可能已经跨进亮屏，取大值即判它不达标。
        dark = np.maximum(values[:, :, 0], values[:, :, 1])
        bright = values[:, :, 2]
        contrast = bright - dark
        coverage = ((contrast >= self.min_edge_contrast) &
                    (dark <= bright * self.max_dark_ratio)).mean(axis=1)
        if (np.count_nonzero(coverage >= self.min_edge_coverage) < 3 or
                coverage.min() < self.weak_edge_coverage):
            return None
        score = float(np.median(contrast) + 60.0 * coverage.mean())
        return dict(matrix=phys_to_img, coverage=coverage, score=score,
                    contrast=float(np.median(contrast)),
                    min_contrast=float(contrast.min()))

    def _verify(self, gray, q, kind):
        """把提案四边形当作物理解释一遍：带宽、亮度、覆盖率、内外框都自洽才算通过。"""
        q = order_quad(q)
        if not self._valid_quad(q, gray.shape):
            return None
        # 正方框（显示器、窗格）在这一步出局：靶纸的外框/纸边都不是正方形。
        # 这道门与带宽门互补 —— 带宽门管"暗带厚不厚"，它管"框像不像靶纸"。
        if not (self.aspect_min <= _aspect_py(q) <= self.aspect_max):
            return None
        best = None
        for width, height in self._physical_layouts(q):
            evidence = self._profile(gray, q, kind, width, height)
            if evidence is None:
                continue
            matrix = evidence["matrix"]
            outer = project_points(matrix, rect_quad(width, height)).astype(np.float32)
            inner = project_points(matrix, rect_quad(width, height, self.tape_mm)).astype(np.float32)
            if not self._valid_quad(outer, gray.shape):
                continue
            item = dict(quad=q, outer_quad=outer, inner_quad=inner, kind=kind,
                        rect_to_image=matrix, width_mm=width, height_mm=height,
                        score=evidence["score"], contrast=evidence["contrast"],
                        min_contrast=evidence["min_contrast"], coverage=evidence["coverage"])
            if best is None or item["score"] > best["score"]:
                best = item
        return best

    def _refine(self, gray, q, kind, radius):
        """当前帧窄带梯度采样 → 每边鲁棒拟线 → 相邻求交，得到亚像素四角。"""
        q = np.asarray(q, np.float32)
        if edge_lengths(q).min() < 10:
            return None
        points, base, normals, offsets = refine_sample_points(q, self._sample_t, radius)
        h, w = gray.shape
        valid = ((points[..., 0] >= 1) & (points[..., 0] < w - 1) &
                 (points[..., 1] >= 1) & (points[..., 1] < h - 1)).all(axis=2)
        values = sample_points(gray, points).astype(np.float32)
        gradient = values[:, :, 2:] - values[:, :, :-2]
        # 内框：向内是亮纸面（梯度为正）；外框：向内先跨过暗带（梯度为负）。
        if kind == "outer":
            gradient *= -1
        penalty = np.abs(offsets[1:-1])[None, None, :] * 0.35
        idx = np.argmax(gradient - penalty, axis=2)
        strength = np.take_along_axis(gradient, idx[..., None], axis=2)[..., 0]
        shift = offsets[idx + 1]
        min_points = max(5, int(len(self._sample_t) * self.weak_edge_coverage))
        lines = []
        for i in range(4):
            good = valid[i] & (strength[i] >= self.min_edge_contrast * 0.65)
            if good.sum() < min_points:
                return None
            median = float(np.median(shift[i, good]))
            spread = float(np.median(np.abs(shift[i, good] - median)))
            good &= np.abs(shift[i] - median) <= max(2.5, spread * 3.0)
            if good.sum() < 5:
                return None
            pts = base[i, good] + shift[i, good, None] * normals[i]
            vx, vy, x, y = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
            residual = np.abs((pts[:, 0] - x) * vy - (pts[:, 1] - y) * vx)
            if np.median(residual) > self.max_line_residual:
                return None
            lines.append([-vy, vx, vy * x - vx * y])
        lines = np.asarray(lines, np.float64)
        crossings = np.cross(np.roll(lines, 1, axis=0), lines)
        if np.any(np.abs(crossings[:, 2]) < 1e-8):
            return None
        refined = (crossings[:, :2] / crossings[:, 2:]).astype(np.float32)
        if np.linalg.norm(refined - q, axis=1).max() > radius * 2.8:
            return None
        return refined if self._valid_quad(refined, gray.shape) else None

    def _candidate_quads(self, mask):
        """暗块外轮廓（outer）+ 亮面补集轮廓（inner）都做提案，去重后按面积取前 N。"""
        h, w = mask.shape
        min_area = max(self.min_contour_area, mask.size * self.min_area_ratio)
        max_area = mask.size * self.max_area_ratio
        candidates = []
        for source, kind, limit in ((mask, "outer", None), (cv2.bitwise_not(mask), "inner", self.max_contours)):
            contours = cv2.findContours(source, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if limit is not None:
                contours = contours[:limit]
            eligible = []
            for contour in contours:
                _x, _y, bw, bh = cv2.boundingRect(contour)
                if min(bw, bh) < self.min_quad_side or bw * bh < min_area:
                    continue
                area = abs(float(cv2.contourArea(contour)))
                if min_area <= area <= max_area:
                    eligible.append((area, contour))
            # 取面积最大的若干个再做多边形逼近：检索顺序是空间顺序，不是面积顺序。
            for area, contour in sorted(eligible, key=lambda item: item[0], reverse=True)[:self.max_candidates]:
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, self.approx_epsilon * perimeter, True)
                if len(approx) != 4:
                    hull = cv2.convexHull(contour)
                    approx = cv2.approxPolyDP(hull, self.approx_epsilon * 1.4 * cv2.arcLength(hull, True), True)
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue
                q = order_quad(approx.reshape(4, 2))
                if not self._valid_quad(q, mask.shape, allow_border=True):
                    continue
                candidates.append((q, kind, area))
        # 闭合洞与亮面补集常常描述同一条边，留一个表示即可，避免重复验证。
        unique = []
        for candidate in sorted(candidates, key=lambda item: item[2], reverse=True):
            q, kind, _area = candidate
            duplicate = any(np.mean(np.linalg.norm(q - old[0], axis=1)) < 5.0 and kind == old[1]
                            for old in unique)
            if not duplicate:
                unique.append(candidate)
        return sorted(unique, key=lambda item: item[2], reverse=True)[:self.max_candidates]

    def _scan(self, gray, roi=None):
        """在 roi（或整幅）里做一次"降采样 → 阈值 → 提案 → 验证 → 精修"。"""
        x, y, w, h = roi if roi is not None else (0, 0, gray.shape[1], gray.shape[0])
        crop = gray[y:y + h, x:x + w]
        if crop.size == 0:
            return None
        started = time.perf_counter()
        scale = min(1.0, self.search_width / w)
        sw, sh = max(32, round(w * scale)), max(24, round(h * scale))
        small = cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else crop
        xy_scale = np.array([w / float(sw), h / float(sh)], np.float32)
        origin = np.array([x, y], np.float32)
        # 先全局 Otsu；本次扫描没有确认目标才退化到局部阈值。不做阈值扫描、不做全图 Hough。
        otsu, otsu_mask = cv2.threshold(small, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        self._timing["prep"] += (time.perf_counter() - started) * 1000.0
        for mode in ("otsu", "adaptive"):
            started = time.perf_counter()
            if mode == "otsu":
                mask = otsu_mask
                threshold = int(otsu)
            else:
                block = self.adaptive_block + (1 - self.adaptive_block % 2)
                mask = cv2.adaptiveThreshold(small, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, block, self.adaptive_c)
                threshold = "adaptive"
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            self._threshold_count += 1
            self._search_count += 1
            if self.keep_stages:
                self.last_stages = dict(binary=mask)
            proposals = self._candidate_quads(mask)
            self._proposals += len(proposals)
            self._timing["prep"] += (time.perf_counter() - started) * 1000.0
            for q, kind, _area in proposals:
                q = q * xy_scale + origin
                started = time.perf_counter()
                verified = self._verify(gray, q, kind)
                self._timing["comp"] += (time.perf_counter() - started) * 1000.0
                if verified is None:
                    continue
                started = time.perf_counter()
                refined = self._refine(gray, q, kind, self.refine_radius)
                if refined is not None:
                    precise = self._verify(gray, refined, kind)
                    if precise is not None:
                        verified = precise
                self._timing["quad"] += (time.perf_counter() - started) * 1000.0
                verified["threshold"] = threshold
                return verified
        return None

    def _search_roi(self, gray):
        """上帧外框外扩一圈的搜索窗；窗太小说明框已贴边，直接回全图。"""
        if self._track is None:
            return None
        q = np.asarray(self._track["outer_quad"], np.float32)
        lo, hi = q.min(axis=0), q.max(axis=0)
        pad = np.maximum((hi - lo) * self.roi_padding, self.roi_min_padding)
        lo = np.maximum(0, np.floor(lo - pad)).astype(int)
        hi = np.minimum([gray.shape[1], gray.shape[0]], np.ceil(hi + pad)).astype(int)
        if np.any(hi - lo < self.min_roi_size):
            return None
        return int(lo[0]), int(lo[1]), int(hi[0] - lo[0]), int(hi[1] - lo[1])

    def _fail_reason(self):
        """no_proposal = 画面里连候选块都没有；all_filtered = 有候选但带宽/亮度/覆盖率不过。"""
        return "all_filtered" if self._proposals else "no_proposal"

    # ------------------------------------------------------------- 对外入口
    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        ts = started if timestamp is None else float(timestamp)
        if not np.isfinite(ts):
            raise ValueError("non-finite timestamp")
        g = to_gray_frame(gray)
        self._frame += 1
        self._search_count = self._threshold_count = self._proposals = 0
        self._timing = dict(prep=0.0, comp=0.0, quad=0.0)
        candidate, search_mode = None, None
        if self._track is not None and g.size:
            old_q, kind = self._track["detected_quad"], self._track["kind"]
            q = self._refine(g, old_q, kind, self.track_radius)
            if q is not None:
                candidate = self._verify(g, q, kind)
            if candidate is not None:
                search_mode = "track"
            else:
                # 预测 ROI 内重搜一次；同一帧内完成，不拿上一帧的角点冒充本帧测量。
                roi = self._search_roi(g)
                if roi is not None:
                    candidate = self._scan(g, roi)
                    if candidate is not None:
                        search_mode = "roi_search"
        if candidate is None and g.size:
            candidate = self._scan(g)
            if candidate is not None:
                search_mode = "full_search"
        if candidate is None:
            self._track = self._smooth = None
            result = dict(valid=False, stage=STAGE_NONE, fail_stage="detect",
                          fail_reason=self._fail_reason(), center=None)
        else:
            candidate["detected_quad"] = candidate["quad"]
            self._track = candidate
            raw = quad_diagonal_center(candidate["inner_quad"])
            if raw is None:
                self._smooth = None
                result = dict(valid=False, stage=STAGE_NONE, fail_stage="detect",
                              fail_reason="degenerate", center=None)
            else:
                alpha = self.center_alpha
                if (self._smooth is None or alpha <= 0 or
                        math.hypot(raw[0] - self._smooth[0], raw[1] - self._smooth[1]) > self.filter_reset):
                    smooth = raw
                else:
                    smooth = (self._smooth[0] + alpha * (raw[0] - self._smooth[0]),
                              self._smooth[1] + alpha * (raw[1] - self._smooth[1]))
                self._smooth = smooth
                result = self._make_result(candidate, raw, smooth, search_mode)
        result.update(masks=1, proposals=self._proposals, search_count=self._search_count,
                      threshold=self._threshold_count, reject=dict(proposals=self._proposals),
                      t_ms=dict(self._timing), build=BUILD_ID)
        self.last = finish_result(result, ts,
                                  (int(g.shape[1]), int(g.shape[0])) if g.size else (0, 0),
                                  started)
        return self.last

    def _make_result(self, cand, raw, smooth, search_mode):
        """把验证通过的候选翻译成主轨结果契约（下游只认这里的字段）。

        逐帧不做单应反变换、不投影响应圆点：中心已经是内框对角线交点的几何解，
        每帧多付两次 numpy 调用（板端各约 1ms）换不到精度。
        """
        outer, inner = cand["outer_quad"], cand["inner_quad"]
        bx = outer[:, 0]
        by = outer[:, 1]
        return dict(valid=True, stage=STAGE_PLANE, search_mode=search_mode,
                    u=float(smooth[0]), v=float(smooth[1]),
                    raw_u=float(raw[0]), raw_v=float(raw[1]),
                    quad=outer, inner_quad=inner, detected_quad=cand["quad"],
                    outer_inferred=cand["kind"] == "inner", center_ref="inner",
                    bbox=(int(bx.min()), int(by.min()),
                          int(bx.max() - bx.min()), int(by.max() - by.min())),
                    area=_poly_area_py(outer), aspect=_aspect_py(inner),
                    band=dict(paper=cand["contrast"], contrast=cand["contrast"]),
                    confidence=float(np.mean(cand["coverage"])),
                    width_mm=float(self.model_mm[0]), height_mm=float(self.model_mm[1]),
                    rect_to_image=cand["rect_to_image"],
                    partial=False, hold=False, lost=0)


# ================================================================== 光斑
class LaserSpotDetector:
    """标称位置附近找最亮点，亚像素质心 + 限频 + 跳变门限。

    为什么只搜一个小 ROI：相机与激光笔刚性固连（同装在俯仰平台上），光斑在图像里的
    位置近似固定 —— 2026-10-07 现场一帧实测约 (294, 202)（640x480），背景灰度 145、
    峰值 177。因此不必全图找"最亮的东西"（那会把白墙高光、灯光、反光都算进去）。

    两种取位方式（`LASER_MODE`），云台侧拿到的 (u,v) 语义完全相同：
        "fixed"（默认） 光斑位置写死 = (LASER_U_NOMINAL, LASER_V_NOMINAL)，
            全程不做任何图像检测；每帧唯一的运算是"黑框中心 − 该常量"的两次减法。
            之所以可行：相机与激光刚性同转 → 光斑位置本来就是常量。
        "detect"        每 LASER_INTERVAL_S 在标称位附近 ROI 实测一次（自校准）。
            只在需要重新标定 U/V、或想验证"写死的值还准不准"时临时启用。

    输出契约（dict）：
        found    bool    本次是否有可信光斑（未更新时间沿用的仍是它）
        updated  bool    本次是否真的重算了（False = 限频沿用 / fixed 模式）
        u, v     float   光斑位置，相机原图坐标。**调用方必须先看 found**
        peak/bg  float   ROI 最大灰度 / 背景灰度
        area     int     超过阈值的像素数
        reason   str     原因码：ok / fixed / no_peak / too_big / too_small / jumped / roi
        ms       float   本次计算耗时（毫秒），未更新时为 0
    """

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

    def _detect(self, g):
        """在标称位附近的方形 ROI 内做一次"阈值 + 加权质心"，返回 (u,v)。

        判据链（任一不满足即判无光斑，并带 reason 便于排障）：
            峰值 >= max(LASER_MIN_PEAK, 背景 + LASER_MIN_CONTRAST)  → 否则 no_peak
            亮点像素数在 [LASER_MIN_PIXELS, LASER_MAX_PIXELS] 内    → 过大 too_big / 过小 too_small
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

    @staticmethod
    def _fixed_dict(nominal):
        return dict(found=True, updated=False, u=nominal[0], v=nominal[1],
                    peak=0.0, bg=0.0, area=0, reason="fixed", ms=0.0)

    def static_state(self):
        """fixed 模式的常量结果：光斑位置写死，不做任何图像运算。"""
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



# ================================================================== 闭环决策
# 状态语义（给日志和上层判断用）
BOOT = "BOOT"        # 上电，等第一帧合法误差
COARSE = "COARSE"    # 大误差，大步快速逼近
FINE = "FINE"        # 小误差，小步逼近
SETTLE = "SETTLE"    # 进入死区，停止发指令
HOLD = "HOLD"        # 视觉失效，保持不动
FAULT = "FAULT"      # 链路/机械故障，停机不再发指令


class GimbalAim:
    """基于像素误差的两轴增量比例控制器 + 状态机。

    - 闭环输入 = ``result["dx"]/["dy"]``（靶心 − 光斑，见 `deviation`）；
    - 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅；
      速度按误差排（远快近慢）、起停斜坡随指令下发，驱动器自己走完加/减速；
    - 状态机 ``BOOT / COARSE / FINE / SETTLE / HOLD / FAULT``；
    - 保护：离群拒收（``JUMP_REJECT_PX``）、死区（``DEADBAND_PX``）、指令限频
      （``COMMAND_INTERVAL_S``）、单帧限幅（``MAX_STEP_PULSES``）、发散看护（``DIVERGE_GUARD``）。

    本类**保持纯 Python（不 import maix）**，可在 PC 上离线仿真。
    控制律可整体替换：``controller(axis, error_px, now) -> 带符号脉冲数``，传 None 用内置比例律。
    """

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
        self._busy_until = {"yaw": -1e9, "pitch": -1e9}
        self._capture_after = {"yaw": -1e9, "pitch": -1e9}
        self._direction = {"yaw": 0., "pitch": 0.}
        self._last_measurement_time = None
        self._stops = []
        self._guard_error = {"yaw": None, "pitch": None}
        self._diverge = {"yaw": 0, "pitch": 0}

    # ------------------------------------------------------------- 外部接口
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
        self._lost = 0
        self._jump = {"yaw": 0, "pitch": 0}
        self.last_command_time = {"yaw": -1e9, "pitch": -1e9}
        return self.state

    def update(self, result, now=0.0):
        """输入视觉 result，返回 ``cmds``；无效结果按状态机处理（HOLD）。"""
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
        for axis, error in (("yaw", dx), ("pitch", dy)):
            if self._busy_until[axis] > now and (abs(error) <= self._deadband() or error*self._direction[axis] < 0):
                self._stops.append(axis)
                self._busy_until[axis] = now
                self._capture_after[axis] = now
        deadband = self._deadband()
        if abs(dx) <= deadband and abs(dy) <= deadband:
            self.state = SETTLE
            return self._emit(dx, dy, {"yaw": None, "pitch": None})
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

    def _speed(self, error):
        """远快近慢：目标角速度随误差线性上升，上下限夹住。

        单位是**电机轴**度/秒；有减速比时输出端按比例更慢。这里只管"想多快"，
        真正的 rpm/acc 换算在驱动层 `zdt.speed_profile`。
        """
        per_px = float(getattr(self.cfg, "AIM_DEG_S_PER_PX", 0.5))
        low = float(getattr(self.cfg, "AIM_MIN_DEG_S", 8.0))
        high = float(getattr(self.cfg, "AIM_MAX_DEG_S", 60.0))
        return max(low, min(high, abs(float(error))*per_px))

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
        # 速度按"误差大走得快、误差小走得慢"排，斜坡时间固定；驱动器自己完成
        # 加速—匀速—减速，一条指令就是一次完整动作，不会一顿一顿。
        rpm, acc = speed_profile(self._speed(error),
                                 float(getattr(cfg, "AIM_RAMP_S", 0.12)))
        self._direction[axis] = error
        self._busy_until[axis] = now+estimate_move_seconds(pulses, rpm, acc, getattr(cfg, "MOTOR_PULSES_PER_REV", 3200))+getattr(cfg, "AIM_MOTION_MARGIN_S", .03)
        return ccw, pulses, rpm, acc



# ================================================================== 串口唯一所有者
class VisionControl:
    """识别 → 偏差 → 决策 → 下发。串口的唯一所有者（识别线程内调用）。

    职责边界：本类只做**决策**，任何 `link.move` / `link.stop` 都经 `apply_commands`
    与 `stop_motion` 走执行层；本类不直接写串口。
    """

    def __init__(self, cfg, link=None, aim=None, clock=None):
        self.cfg, self.link, self.aim = cfg, link, aim
        self._clock = clock if clock is not None else time
        self.detector = RectangleCenterDetector(cfg)
        self.laser = LaserSpotDetector(cfg) if cfg.ENABLE_LASER else None
        self.laser_static = self.laser is not None and self.laser.mode == 'fixed'
        self.laser_state = self.laser.static_state() if self.laser_static else None
        self.last_capture = -1e9
        self.stale_stopped = True
        self.control_source = 'vision'
        self.commands = {'yaw': None, 'pitch': None}
        self._action_lock = threading.Lock()
        self._requested_action = None
        self._action = None
        self._action_steps = ()
        self._action_index = 0
        self._action_ready_at = 0.0
        self._manual_hold = not getattr(cfg, 'AUTO_TRACK_ON_BOOT', True)

    def request_action(self, name):
        """May be called from the camera/touch thread; UART work stays in this worker."""
        if name not in ('search', 'self_test', 'stop'):
            raise ValueError('unknown gimbal action: ' + str(name))
        with self._action_lock:
            self._requested_action = name

    def _take_action(self):
        with self._action_lock:
            name, self._requested_action = self._requested_action, None
        return name

    def _start_action(self, name, now):
        if not self._stop_motion(now):
            return
        self._action = None
        if name == 'stop':
            self._manual_hold = True
            self.aim.update({'aim_valid': False}, now=now)
            return
        self._manual_hold = False
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', False):
            print('[gimbal] %s ignored: GIMBAL_CALIBRATED=False' % name)
            return
        ppr = int(self.cfg.MOTOR_PULSES_PER_REV)
        angle = float(self.cfg.SEARCH_ANGLE_DEG if name == 'search' else self.cfg.SELF_TEST_ANGLE_DEG)
        pulses = round(ppr * angle / 360.0)
        if ppr <= 0 or not 0 < angle <= 60 or pulses <= 0:
            self.aim.fault('invalid_action_angle')
            return
        if name == 'search':
            self._action_steps = (('yaw', True, pulses), ('yaw', False, 2*pulses),
                                  ('yaw', True, pulses))
            self.detector.reset()
        else:
            self._action_steps = (('yaw', True, pulses), ('yaw', False, 2*pulses),
                                  ('yaw', True, pulses), ('pitch', True, pulses),
                                  ('pitch', False, 2*pulses), ('pitch', True, pulses))
        self._action = name
        self._action_index = 0
        self._action_ready_at = now
        self.aim.update({'aim_valid': False}, now=now)
        print('[gimbal] %s started, angle=%.1f deg' % (name, angle))

    def _advance_action(self, result, now):
        if self._action == 'search' and result and result.get('valid') and not result.get('hold'):
            if not self._stop_motion(now):
                return
            self._action = None
            self.aim.reset()
            print('[gimbal] target acquired; tracking')
            return
        if now < self._action_ready_at:
            return
        if self._action_index >= len(self._action_steps):
            print('[gimbal] %s complete' % self._action)
            self._action = None
            self._manual_hold = True
            return
        axis, ccw, pulses = self._action_steps[self._action_index]
        rpm, acc = int(self.cfg.MANUAL_RPM), int(self.cfg.MANUAL_ACC)
        if not 0 < rpm <= 5000 or not 0 < acc <= 255:
            self.aim.fault('invalid_manual_speed')
            self._stop_motion(now)
            return
        ok, _, applied = apply_commands(self.link, self.aim,
                                         {axis: (ccw, pulses, rpm, acc)}, now, acc)
        if not ok:
            self._action = None
            return
        self.commands.update(applied)
        self._action_index += 1
        self._action_ready_at = (now + estimate_move_seconds(pulses, rpm, acc,
                                 self.cfg.MOTOR_PULSES_PER_REV) + self.cfg.MANUAL_SETTLE_S)

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
        self._control(output, now, new_measurement=fresh)
        output.update(aim_state=self.aim.state if self.aim else None, cmds=dict(self.commands),
                      control_source=self.control_source)
        return output

    def idle(self):
        self._control(None, self._clock.perf_counter(), new_measurement=False)

    def status(self):
        if self.aim is not None and self.aim.state == 'FAULT':
            return 'FAULT'
        return self._action.upper() if self._action else (self.aim.state if self.aim else 'vision')

    def _stop_motion(self, now, axis=None):
        """停机（实现在执行层 `stop_motion`）。False = 停机失败，调用方必须终止决策链。"""
        return stop_motion(self.link, self.aim, now, axis)

    def _control(self, result, now, new_measurement):
        self.commands = {'yaw': None, 'pitch': None}
        link, aim = self.link, self.aim
        if link is None or aim is None:
            return
        # FAULT before poll: even E2 re-enable is forbidden once motion is faulted.
        if aim.state == 'FAULT':
            return
        feedback = link.poll(now)
        if feedback and feedback.get('fault'):
            self._stop_motion(now)
            aim.fault('link:%s' % feedback.get('reason'))
            return
        requested = self._take_action()
        if requested is not None:
            self._start_action(requested, now)
            if aim.state == 'FAULT':
                return
        if self._action is not None:
            self._advance_action(result if new_measurement else None, now)
            return
        if self._manual_hold:
            aim.update({'aim_valid': False}, now=now)
            return
        if not getattr(self.cfg, 'GIMBAL_CALIBRATED', True):
            if not self.stale_stopped:
                self._stop_motion(now)
            self.stale_stopped = True
            aim.update({'aim_valid': False}, now=now)
            return
        if not new_measurement:
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
        # 顺序（stops 先、cmds 后）与失败路径是不可变的时序约定。
        ok, _failed, applied = apply_commands(link, aim, output['cmds'], now, self.cfg.ACC_TRACK)
        if not ok:
            return
        self.commands.update(applied)


# ================================================================== 主循环
def _capture_loop(cfg, clock, app, image, state, build_id):
    cam, worker, controller, preview = (state[k] for k in ('camera', 'worker', 'controller', 'preview'))
    capture_rate = FrameRate()
    last_log = clock.perf_counter()
    read_times, draw_times, loop_times = [], [], []
    poller = FramePoller(cam, image, getattr(cfg, 'LENS_CORR', False),
                         getattr(cfg, 'LENS_STRENGTH', 1.5), clock)
    buttons = state.get('buttons')
    while not app.need_exit():
        started = clock.perf_counter()
        if buttons is not None:
            action = buttons.poll(started)
            if action is not None:
                controller.request_action(action)
        gray, captured, read_ms = poller.poll()
        if worker.error is not None:
            raise RuntimeError('vision worker failed: ' + worker.error)
        if gray is None:
            if not cfg.VISION_ASYNC:
                controller.idle()
            continue
        read_times.append(read_ms)
        capture_rate.record(captured)
        worker.submit(gray, captured)
        now = clock.perf_counter()
        snapshot, fps, _ = worker.snapshot(now)
        if worker.error is not None:
            raise RuntimeError('vision worker failed: ' + worker.error)
        result, laser, age = display_result(snapshot, now, cfg.RESULT_MAX_AGE_S)
        if preview.due(now):
            t_draw = clock.perf_counter()
            try:
                draw_preview(cfg, preview, aligned_frame(snapshot, gray, result), result,
                              laser, age, fps, snapshot['status'] if snapshot else 'waiting', now)
            except Exception as exc:
                # 绘制/转换错误属于显示，绝不能升级成电机错误。
                preview.enabled = False
                print('[preview] render disabled: %s' % exc)
            draw_times.append((clock.perf_counter()-t_draw)*1000)
        loop_times.append((clock.perf_counter()-started)*1000)
        now = clock.perf_counter()
        if now-last_log >= cfg.LOG_INTERVAL:
            capture_fps, _ = capture_rate.values(now)
            display_fps, low1 = preview.rate.values(now)
            avg, p95, worst, n = preview.take_show_stats()
            mean = lambda values: float(np.mean(values)) if values else 0.
            loop_avg = mean(loop_times)
            detector_result = snapshot.get('detector_result', snapshot['result']) if snapshot else {}
            print(perf_line(build_id, capture_fps, display_fps, low1, fps, result,
                             mean(read_times), mean(draw_times), loop_avg, avg, p95, worst, n, age,
                             preview.shown, preview.dropped, worker.dropped,
                             detector_result=detector_result))
            metrics = worker.metrics(now)
            print('[throughput] valid_fps=%.1f valid_low1=%.1f worker_p95=%.1fms' %
                  (metrics['valid_fps'], metrics['valid_low1'], metrics['worker_p95_ms']))
            if state['aim'] is not None:
                print(state['aim'].status_line())
            if p95 > 100:
                preview.warn_slow_once(p95)
            if loop_avg > MAIN_LOOP_SLOW_MS:
                print('[preview] main loop slow: loop=%.0fms（不含 show）；看 read= 是否偏大' % loop_avg)
            last_log = now
            read_times.clear(); draw_times.clear(); loop_times.clear()


def _release(state):
    errors = []
    def close(resource):
        if resource is not None and hasattr(resource, 'close'):
            try:
                resource.close()
            except Exception as exc:
                errors.append(exc)
    worker = state.get('worker')
    close(worker)
    if worker is None or worker.thread is None or not worker.thread.is_alive():
        link = state.get('link')
        if link is not None:
            # 退出停机也走执行层入口：`stop_motion` 内部已捕获异常并回报 False。
            if stop_motion(link, None, time.perf_counter()) is False:
                errors.append(RuntimeError('stop on exit failed'))
        close(link)
    close(state.get('preview'))
    close(state.get('camera'))
    if errors:
        raise RuntimeError('cleanup: ' + '; '.join(map(str, errors)))


def run():
    # 兜底：绕过 main.py 直接调 run()（单文件版/其它入口）时同样关掉内置图传。
    os.environ.setdefault("MAIXVISION", "0")
    from config import Config
    cfg, clock = Config, time
    state = dict(camera=None, link=None, aim=None, worker=None, controller=None, preview=None)
    try:
        from maix import app, camera, image
        cv2.setNumThreads(cfg.OPENCV_THREADS)
        cam = state['camera'] = camera.Camera(cfg.CAMERA_W, cfg.CAMERA_H,
                                             image.Format.FMT_GRAYSCALE,
                                             fps=cfg.CAMERA_FPS, buff_num=cfg.CAMERA_BUFFERS)
        cam.skip_frames(cfg.SKIP_FRAMES)
        if cfg.ENABLE_GIMBAL:
            link = state['link'] = StepperLink(cfg, clock=clock)
            aim = state['aim'] = GimbalAim(cfg)
            if link.enable() is False:
                aim.fault('enable_failed')
            print('[gimbal] enabled yaw=%s(ID%d) pitch=%s(ID%d) acc=%d' %
                  (cfg.YAW_DEV, cfg.ADDR_YAW, cfg.PITCH_DEV, cfg.ADDR_PITCH, cfg.ACC_TRACK))
        preview = state['preview'] = Preview(cfg, clock=clock)
        if getattr(cfg, 'ENABLE_TOUCH_BUTTONS', False) and preview.enabled:
            try:
                state['buttons'] = TouchButtons(preview.width, preview.height)
                preview.buttons = state['buttons']
            except Exception as exc:
                print('[touch] unavailable: %s' % exc)
        controller = state['controller'] = VisionControl(cfg, state['link'], state['aim'], clock=clock)
        worker = state['worker'] = LatestFrameWorker(controller.process, controller.idle,
                                                    cfg.VISION_ASYNC, clock=clock)
        print('[rect-center] %s search_width=%d tape=%.0fmm edge=%d refine=%d track_radius=%d '
              'display_width=%d display_fps=%d' %
              (BUILD_ID, cfg.SEARCH_WIDTH, cfg.BLACK_FRAME_MM, cfg.EDGE_SAMPLES,
               cfg.REFINE_RADIUS, cfg.TRACK_RADIUS, preview.render_width, cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC, preview.render_width, preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC, cfg.RESULT_MAX_AGE_S*1000))
        _capture_loop(cfg, clock, app, image, state, BUILD_ID)
    finally:
        _release(state)
'''

# ===== vision  (vision/__init__.py) =====
_MOD_SRC['vision'] = r'''
"""vision 包：四个分层 + 一个设备驱动，目录结构即分层结构。

    ① poll.py     轮询层  取帧 → 灰度 → 帧率 → 识别线程
    ② algo.py     算法层  纯函数：几何 / 采样模板 / 四边形判据 / 偏差
    ③ logic.py    逻辑层  结果契约 / 检测器（提案-验证-精修） / 闭环决策 / 主循环
    ④ output.py   执行层  绘制 / 屏幕 / 下发入口 —— 唯一允许产生副作用处
       zdt.py     驱动层  张大头 ZDT 步进：协议帧 + 双串口链路（只管收发，不做决策）

`zdt` 是 `output` 的下层：电机怎么说话只有它知道；`output` 只决定"该不该发、发什么"。
其它模块要转云台，一律通过 `zdt.StepperLink` 的 enable / move / speed / stop，
不允许自己拼协议帧、也不允许直接开串口。

依赖方向单向，不可反向：

    poll  ←──  algo  ←──  zdt
      ↑                    ↑
      └────────  output ←──┘
                   ↑
                 logic

    logic  → {poll, algo, output, zdt}    output → {poll, algo, zdt}
    zdt    → {}                           algo   → {}          poll → {}

`main.py`（应用根目录）也属逻辑层，只做路径引导后转交 `logic.run`。

本文件只作包标记（`app.yaml` 打包与 `check_app.py` 的 C3 都要求它存在），
不 re-export 任何符号 —— 需要什么直接 `from vision.xxx import yyy`，
依赖关系在 import 行上一眼可见，也不会出现"改了模块忘了改这里"的隐式入口。
"""
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

② 追踪中心（**有状态**，ROI 跟踪 + 速度外推 + 失锁自动重捕获，适合连续视频流）::

    from target_api import CenterTracker

    tracker = CenterTracker()
    while True:
        res = tracker.track(gray)
        if res:
            print(res.u, res.v, res.stage)     # stage: plane_frame / no_black_frame

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
    u, v       float   靶心像素坐标（一阶低通平滑后，供显示/常规使用）
    raw_u/v    float   未平滑中心（闭环用，见 config.AIM_USE_RAW）
    center     tuple   (u, v)；无效时为 None
    quad       (4,2)   外框四角（胶带外沿，仅画图用）
    inner_quad (4,2)   内框四角（纸边，靶心由它的对角线交点给出）
    stage      str     plane_frame / no_black_frame（另有 stale = 结果过期）
    hold       bool    True = 本帧是"速度外推续命"帧（跟踪短暂失败，仍在输出）
    fail_stage str     仅 valid=False 时：本帧实际走过的层级（调试用）
    fail_reason str    仅 valid=False 时：no_proposal / all_area / all_filtered
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

    cfg = load_config(MIN_EDGE_CONTRAST=30.0, MIN_EDGE_COVERAGE=0.65)
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

from vision.algo import deviation
from vision.logic import LaserSpotDetector, RectangleCenterDetector

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

    ``load_config(MIN_EDGE_COVERAGE=0.65)`` → 派生一个只改了该字段的配置类。
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

    每帧都是一次完整的"降采样 Otsu 提案 → 窄带黑带验证 → 四边拟线精修 → 内框对角线交点"，
    不依赖上一帧，适合"只在某几问里取一次黑框"的用法。传 ``detector`` 可复用分配，
    但每次都清空跟踪状态；连续视频使用 ``CenterTracker``。

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
    """**有状态**中心追踪器：锁定后只在 ROI 里找 + 速度外推续命，失锁回全图重捕获。

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

    ⚠ 使用前必须完成开环标定（把实测的 ``PULSE_PER_PX_*`` / ``DIR_SIGN_*`` 填回 config），
    否则方向/增益是占位值，可能正反馈甩头。不需要电机的问请只用 ``detect_black_frame`` /
    ``CenterTracker``，不要实例化本类。

    ``step(frame)`` 返回的 ``Result`` 比追踪器多两个字段：
        aim_state  str   状态机状态（FINE/COARSE/SETTLE/HOLD/FAULT…）
        cmds       dict  {"yaw": (ccw, pulses, rpm, acc) or None, "pitch": ...}
    """

    def __init__(self, cfg=None, enable_motors=True, with_laser=True, clock=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.clock = clock if clock is not None else time
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = self.aim = None
        self.closed = False
        if enable_motors:
            from vision.logic import GimbalAim
            from vision.zdt import StepperLink
            self.link = StepperLink(self.cfg, clock=self.clock)
            self.aim = GimbalAim(self.cfg)
            if self.link.enable() is False:
                self.aim.fault("enable_failed")
        from vision.logic import VisionControl
        self.control = VisionControl(self.cfg, self.link, self.aim, clock=self.clock)

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
        """请求左右各 60 度搜索；识别到黑框后自动切回闭环。"""
        if not self.closed:
            self.control.request_action('search')

    def request_self_test(self):
        """请求小角度左、右、上、下电机自检。"""
        if not self.closed:
            self.control.request_action('self_test')

    def stop(self):
        if self.link is not None:
            self.control.request_action('stop')
            self.control._stop_motion(self.clock.perf_counter())

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.stop()
        finally:
            if self.link is not None:
                self.link.close()
'''

_MAIN_SRC = r'''
"""云台开环标定（板端运行）——量出 PULSE_PER_PX 与 DIR_SIGN。

为什么需要它：闭环把"像素误差"换算成"脉冲数"，靠的就是 `PULSE_PER_PX`
（1 像素差要发几个脉冲）。这个值取决于镜头视场、细分、减速比，算不出来，
只能实测。方向符号 `DIR_SIGN_*` 同理，接反了会越追越偏。

做法（每轴四步，全程自动）：

    1. 记住靶心当前位置；
    2. 朝正方向走 N 个脉冲，等停稳，再量靶心 → 得到"正走位移"；
    3. 反向走 N（回到起点附近）→ 顺便量回程误差，即齿隙；
    4. 再反向走 N → 得到"反走位移"；最后正向走 N 回起点。

    PULSE_PER_PX = 2N / (|正走位移| + |反走位移|)
    DIR_SIGN     = +1（正走让坐标变小）/ -1（正走让坐标变大）

跑法：MaixVision 打开 `dist/calib_gimbal.py` →「运行当前文件」。
      也可在模块化工程里把本文件当入口，但要用「运行项目」。

⚠ 运行前把靶纸完整放进画面。画面左上角会实时显示识别状态。
⚠ 探针幅度小（约 5~10°）、速度慢（30°/s），不会甩头；中途丢目标会原路退回。
⚠ 结果只打印在控制台，不会自动改 config.py —— 确认后手工填回。
"""
import os

# 关掉 MaixCDK 内置图传：标定脚本自己建本地屏，避免"连接未就绪"时 show() 忙等 500ms
os.environ.setdefault("MAIXVISION", "0")

import time

import cv2
import numpy as np

from config import Config as cfg
from vision.logic import RectangleCenterDetector
from vision.zdt import StepperLink, estimate_move_seconds

PROBE_PULSES = 80          # 起始探针脉冲数（3200 细分下约 9°）
PROBE_DEG_S = 30.0         # 探针速度：慢，肉眼能跟
PROBE_RAMP_S = 0.15        # 起停斜坡
SETTLE_S = 0.45            # 停稳后再量（齿隙回弹要时间）
MEASURE_FRAMES = 4         # 每次量几帧取平均
VALID_FRAMES = 6           # 起始锁定需要连续几帧
MAX_SPREAD_PX = 3.0        # 这几帧的中心最大离散度
WorstPxPerPulse = 4.0      # 安全余量：假设 1 脉冲最多走 4px，防止探针把靶纸推出画面


def _mean(points):
    return (sum(p[0] for p in points)/len(points), sum(p[1] for p in points)/len(points))


def _spread(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return max(max(xs)-min(xs), max(ys)-min(ys))


class Viewer:
    """相机 + 本地屏 + 标注。只做标定用，不复用主程序的显示线程。"""

    def __init__(self):
        from maix import camera, display, image
        self.image = image
        self.cam = camera.Camera(cfg.CAMERA_W, cfg.CAMERA_H,
                                 image.Format.FMT_GRAYSCALE, fps=30)
        self.cam.skip_frames(10)
        self.disp = display.Display()
        self.n = 0

    def read(self):
        img = self.cam.read()
        gray = self.image.image2cv(img, ensure_bgr=False, copy=False)
        gray = np.asarray(gray)
        if gray.ndim == 3:
            gray = gray[:, :, 0]
        return np.ascontiguousarray(gray)

    def show(self, gray, res, line1="", line2=""):
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        ok = bool(res and res.get("valid"))
        if ok:
            u, v = float(res["u"]), float(res["v"])
            cv2.drawMarker(canvas, (int(round(u)), int(round(v))), (255, 0, 0),
                           cv2.MARKER_CROSS, 18, 2)
            cv2.putText(canvas, "target (%d,%d)" % (round(u), round(v)),
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
        else:
            reason = (res or {}).get("fail_reason") or (res or {}).get("stage") or "-"
            cv2.putText(canvas, "NO TARGET  " + str(reason), (8, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
        if line1:
            cv2.putText(canvas, line1, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, .45,
                        (0, 255, 255), 1)
        if line2:
            cv2.putText(canvas, line2, (8, 74), cv2.FONT_HERSHEY_SIMPLEX, .45,
                        (255, 220, 0), 1)
        self.disp.show(self.image.cv2image(canvas, bgr=True, copy=True))
        self.n += 1


def _grab(viewer, det):
    gray = viewer.read()
    return gray, det.process(gray, timestamp=time.time())


def _wait_target(viewer, det, app):
    """等画面里出现稳定黑框，返回 (u, v)；超时/用户退出返回 None。"""
    print("[calib] 等待目标：把靶纸完整放进画面")
    points, last_log = [], 0.0
    while not app.need_exit():
        gray, res = _grab(viewer, det)
        viewer.show(gray, res, "waiting for target", "keep the whole frame in view")
        if res.get("valid") and not res.get("hold"):
            points.append((float(res["u"]), float(res["v"])))
            if len(points) > VALID_FRAMES:
                points.pop(0)
            if len(points) == VALID_FRAMES and _spread(points) <= MAX_SPREAD_PX:
                center = _mean(points)
                print("[calib] 锁定目标 u=%.1f v=%.1f" % center)
                return center
        else:
            points = []
        if time.time()-last_log > 1.0:
            last_log = time.time()
            print("[calib] 还没看到黑框（%s）" %
                  (res.get("fail_reason") or res.get("stage") or "-"))
    return None


def _measure_center(viewer, det, app, timeout=1.5):
    """连量几帧取平均；超时或丢目标返回 None。"""
    points = []
    end = time.time()+timeout
    while time.time() < end and not app.need_exit():
        gray, res = _grab(viewer, det)
        viewer.show(gray, res, "measuring", "%d/%d" % (len(points), MEASURE_FRAMES))
        if res.get("valid") and not res.get("hold"):
            points.append((float(res["u"]), float(res["v"])))
            if len(points) >= MEASURE_FRAMES:
                return _mean(points)
    return None


def _move_wait(link, axis, ccw, pulses):
    ok, rpm, acc = link.move_smooth(axis, ccw, pulses, PROBE_DEG_S, PROBE_RAMP_S)
    if not ok:
        print("[calib] %s 下发失败" % axis)
        return False
    wait = estimate_move_seconds(pulses, rpm, acc,
                                 getattr(cfg, "MOTOR_PULSES_PER_REV", 3200)) + SETTLE_S
    time.sleep(max(0.05, wait))
    return True


def _choose_probe(center, axis):
    """按靶心到画面边缘的距离定探针幅度，避免把靶纸推出画面。"""
    u, v = center
    margin = min(u, cfg.CAMERA_W-u) if axis == "yaw" else min(v, cfg.CAMERA_H-v)
    limit = max(12, int(margin*0.6/WorstPxPerPulse))
    return max(8, min(PROBE_PULSES, limit))


def _probe_axis(viewer, det, link, app, axis, center):
    """一轴四步探针。返回 (pulse_per_px, dir_sign, backlash_px, d_plus, d_minus) 或 None。"""
    comp = 0 if axis == "yaw" else 1
    n = _choose_probe(center, axis)
    print("[calib] %s 探针 %d 脉冲（约 %.1f°）" % (axis, n, n*360.0/cfg.MOTOR_PULSES_PER_REV))
    start = center

    if not _move_wait(link, axis, True, n):
        return None
    plus = _measure_center(viewer, det, app)
    if plus is None:
        print("[calib] %s 正走后丢目标，原路退回" % axis)
        _move_wait(link, axis, False, n)
        return None

    if not _move_wait(link, axis, False, n):
        return None
    back = _measure_center(viewer, det, app)

    if not _move_wait(link, axis, False, n):
        return None
    minus = _measure_center(viewer, det, app)
    if minus is None:
        print("[calib] %s 反走后丢目标，回到起点" % axis)
        _move_wait(link, axis, True, n)
        return None

    _move_wait(link, axis, True, n)          # 回起点

    d_plus = plus[comp]-start[comp]
    d_minus = minus[comp]-start[comp]
    backlash = abs(back[comp]-start[comp]) if back is not None else float("nan")
    travel = abs(d_plus)+abs(d_minus)
    if travel < 6.0:
        print("[calib] %s 位移太小（%.1fpx），结果不可信；换更大的靶或更近的距离重试" %
              (axis, travel))
        return None
    k = 2.0*n/travel
    sign = 1 if d_plus < 0 else -1
    print("[calib] %s  n=%d  Δ+=%+.1fpx  Δ-=%+.1fpx  回程差=%.1fpx" %
          (axis, n, d_plus, d_minus, backlash))
    print("[calib] %s  PULSE_PER_PX=%.4f  DIR_SIGN=%+d" % (axis, k, sign))
    return k, sign, backlash, d_plus, d_minus


def _fov_hint(k):
    """用标定值反推水平视场，做一次常识校验（40° 左右才像这颗镜头）。"""
    px_per_pulse = 1.0/max(1e-6, k)
    px_per_deg = px_per_pulse/(360.0/cfg.MOTOR_PULSES_PER_REV)
    f_px = px_per_deg*180.0/np.pi
    return 2.0*np.degrees(np.arctan((cfg.CAMERA_W/2.0)/max(1.0, f_px)))


def main():
    from maix import app
    viewer = Viewer()
    det = RectangleCenterDetector(cfg)
    link = StepperLink(cfg)
    print("[calib] 串口: yaw=%s(ID%d) pitch=%s(ID%d)" %
          (cfg.YAW_DEV, cfg.ADDR_YAW, cfg.PITCH_DEV, cfg.ADDR_PITCH))
    if not link.enable():
        print("[calib] 使能失败：检查串口/驱动器供电。中止。")
        link.close()
        return
    result = {}
    try:
        center = _wait_target(viewer, det, app)
        if center is None:
            print("[calib] 没有拿到稳定目标，中止")
            return
        for axis in ("yaw", "pitch"):
            got = _probe_axis(viewer, det, link, app, axis, center)
            if got is not None:
                result[axis] = got
            # 一轴测完，重新锁一次中心，避免另一轴从偏移位置起测
            again = _measure_center(viewer, det, app, timeout=2.0)
            if again is None:
                print("[calib] 重新锁定失败，中止")
                break
            center = again
        print("")
        print("=" * 62)
        print(" 标定结果（确认后手工填回 config.py）")
        print("=" * 62)
        if "yaw" in result:
            k, sign, back, dp, dm = result["yaw"]
            print("  PULSE_PER_PX_YAW = %.4f" % k)
            print("  DIR_SIGN_YAW     = %+d" % sign)
            print("  # 回程差 %.1fpx；反推水平视场约 %.0f°" % (back, _fov_hint(k)))
        else:
            print("  YAW 未测出，保持原值")
        if "pitch" in result:
            k, sign, back, dp, dm = result["pitch"]
            print("  PULSE_PER_PX_PITCH = %.4f" % k)
            print("  DIR_SIGN_PITCH     = %+d" % sign)
            print("  # 回程差 %.1fpx；反推水平视场约 %.0f°" % (back, _fov_hint(k)))
        else:
            print("  PITCH 未测出，保持原值")
        print("=" * 62)
        print("[calib] 提示：视场应在 30~90°；超出说明细分/减速比与假设不符。")
    finally:
        link.stop()
        link.close()
        print("[calib] 电机已停，串口已关")


if __name__ == "__main__":
    main()
'''

_MOD_ORDER = ('config', 'vision.poll', 'vision.algo', 'vision.maixvision_stream', 'vision.zdt', 'vision.output', 'vision.logic', 'vision', 'target_api')

_REL_OF = {'config': 'config.py', 'vision.poll': 'vision/poll.py', 'vision.algo': 'vision/algo.py', 'vision.maixvision_stream': 'vision/maixvision_stream.py', 'vision.zdt': 'vision/zdt.py', 'vision.output': 'vision/output.py', 'vision.logic': 'vision/logic.py', 'vision': 'vision/__init__.py', 'target_api': 'target_api.py'}

_pkg = _types.ModuleType("vision")          # 造出虚拟包 vision
_pkg.__path__ = []
_sys.modules["vision"] = _pkg
for _full in _MOD_ORDER:
    if _full == "vision":
        continue
    if "." in _full:                       # vision.logic 这类子模块
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
# ② 主程序：入口原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'tools/calib_main.py', "exec"), globals())
