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

生成时间：2026-11-09 17:41:46
    config.py                    sha1=19d275305a5f
    main.py                      sha1=c2f5fb47aa9e
    target_api.py                sha1=228e453c656f
    vision/__init__.py           sha1=6e0b16db8298
    vision/algo.py               sha1=14cb88e0d143
    vision/logic.py              sha1=885a16c6ce9f
    vision/maixvision_stream.py  sha1=c30d784f26a0
    vision/output.py             sha1=4369254f9fc7
    vision/poll.py               sha1=992056fd530e
"""

import sys as _sys
import types as _types

_SRC_SHA1 = {'config.py': '19d275305a5fda2d01cf67c746c07dc64bc2c7fe', 'vision/poll.py': '992056fd530ecec7480c50b887eb69b6b1716aec', 'vision/algo.py': '14cb88e0d143b93a39d42d2af678fceb55bfb4bd', 'vision/maixvision_stream.py': 'c30d784f26a0da91756e9a63f71ba795fdda7a8d', 'vision/output.py': '4369254f9fc76aa1fdec9ad193e4fa3952b8cb63', 'vision/logic.py': '885a16c6ce9faf941b69e5d6ac2b550863cb331e', 'vision/__init__.py': '6e0b16db82984685e6015211e17ef270fd60c952', 'target_api.py': '228e453c656f41797d4dc6099e2d87b96ffb53ed', 'main.py': 'c2f5fb47aa9ee402ff1d756199b5db926a96bcf0'}

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
    # 自适应阈值（搜框/跟踪都用它）：block 要大于框带宽（半分辨率下框带 ~10px），
    # C 是"比局部均值暗多少才算暗"。框找不到了先调这两个，别去动 LEVEL_RATIO。
    RECT_ADAPT_BLOCK = 31
    RECT_ADAPT_C = 12.0
    RECT_LEVEL_FLOOR = 45.0
    RECT_LEVEL_CEIL = 160.0
    RECT_LEVEL_REFRESH = 15       # 跟踪时每 N 帧重算一次阈值（曝光慢变够用）
    RECT_MIN_AREA = 1200          # 候选最小面积（原图 px2；内部按降采样折算）
    RECT_MAX_AREA_RATIO = 0.55    # 外接框面积上限（占整幅比例），挡"锁到整面墙"
    RECT_MIN_ASPECT = 1.05        # 外接框长短边比。外接框不是最小外接矩形，
    RECT_MAX_ASPECT = 2.60        # 靶纸倾斜时它会变方，所以上限放宽到 2.6
    RECT_MIN_FILL = 0.03          # 连通域面积 / 外接框面积：环是空心的，很小
    RECT_MAX_FILL = 0.70          # 太满说明是实心块，不是框
    RECT_MIN_BAND_CONTRAST = 30.0 # ★ 框内(中间40%)比框边亮多少灰度才认。
    #   白纸压在深色板上实测 100+；洞洞板/显示器边框只有 10~30。
    #   这一条决定"锁得到靶纸"还是"锁到洞洞板上"，是搜框最关键的一道门。
    RECT_PAPER_ROUNDS = 2         # 从框心向外扫几条线迭代几次（2 次足够收敛）
    RECT_TRACK_EXPAND = 1.35      # 跟踪 ROI = 上一帧框 × 该倍数（越小越快，太小会框不住运动）
    RECT_TRACK_MARGIN = 10.0      # 跟踪 ROI 额外边距（原图 px）
    RECT_TRACK_MAX_SIDE = 240.0   # 跟踪降采样后最长边上限；超过就再降一档，钉住每帧像素量
    RECT_TRACK_SIZE_TOL = 0.25    # 跟踪时相对"锁定尺寸"的容差（±25%）
    RECT_HOLD_FRAMES = 6          # 跟丢后按速度外推继续输出的帧数（30fps ≈ 0.2s）
    RECT_HOLD_MAX_PX = 90.0       # 外推距离上限，防止甩飞
    RECT_MAX_SPEED = 2500.0       # 速度估计上限（px/s），防止小 dt 把速度除爆
    RECT_TRACK_GATE = 0.45        # 跟踪门：实测中心离预测超过 0.45×框边长就判本帧作废
    RECT_SMOOTH = 0.55            # 中心一阶低通系数；1 = 不平滑
    RECT_KEEP_STAGES = False      # 真：把 mask 留在 last_stages，供 --stage 导出
    RECT_DEBUG_MAP = True         # 真：首帧把灰度图打成 64x32 字符画（现场调阈值用）

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

    几何  signed_area / order_quad / line_cross / quad_diagonal_center
    图像  downscale / auto_level / dark_mask / components / frame_candidates
          paper_rect / rect_quad / nearest_component
    偏差  deviation（靶心 − 光斑，纯算术）

四条刻意的设计约束（属于本层，不要在别处放宽）：

1. **只认"四周被暗带围住的矩形"**：面积 → 长宽比 → 填充率 → 四边压暗带。
   不做图案识别（纸面同心圆不参与），因此"亮面被暗边包住"的东西（屏幕、窗框）
   在形状上可能被当成靶纸 —— 这是本方案的已知边界。
2. **每次调用只做几次 cv2/numpy 运算**：板上一次 Python 级调用约 1ms，
   本层的调用次数直接等于单帧耗时。禁止在这里写"逐轮廓 / 逐采样点"的循环。
3. **中心取"框心向外扫四条线"的中点**，不取外接框中心 —— 阴影会让暗带在一侧变宽，
   外接框中心会系统性偏移（实测 6~8px），过框心的弦中点则没有这个问题。
4. 参数全部由调用方传入，所以本层可以在 PC 上脱离 `Config` 单独断言。

依赖方向：**本层不 import 其余三层**，只依赖 cv2 / numpy / math。
"""
import math

import cv2
import numpy as np

__all__ = [
    "signed_area", "order_quad", "line_cross", "quad_diagonal_center",
    "downscale", "auto_level", "dark_mask", "adaptive_mask", "components",
    "frame_candidates",
    "nearest_component", "paper_rect", "rect_quad", "box_center", "deviation",
]

# ================================================================== 几何
def signed_area(p):
    """带符号面积的 2 倍：>0 逆时针。用于把四角规整成统一方向。"""
    p = np.asarray(p, np.float64)
    return float(p[0, 0] * (p[1, 1] - p[3, 1]) + p[1, 0] * (p[2, 1] - p[0, 1]) +
                 p[2, 0] * (p[3, 1] - p[1, 1]) + p[3, 0] * (p[0, 1] - p[2, 1]))


def order_quad(points):
    """四角规整为 [左上, 右上, 右下, 左下]（逆时针）。"""
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


def rect_quad(left, top, right, bottom):
    """轴对齐矩形 → 四角（顺序与 order_quad 一致）。"""
    return np.array([[left, top], [right, top], [right, bottom], [left, bottom]], np.float32)


def box_center(box):
    """(x, y, w, h) → 中心点。"""
    return (float(box[0]) + float(box[2]) * 0.5, float(box[1]) + float(box[3]) * 0.5)


# ================================================================== 图像
def downscale(gray, factor):
    """整数倍降采样；factor<=1 原样返回。

    用 INTER_AREA（2×2 均值）而不是隔点取样：同样的调用次数，均值把反光噪点
    压掉一半，胶带环更连续。降采样是"减少像素"的唯一手段，也是本链路提速的第一步。
    """
    f = int(factor)
    if f <= 1:
        return gray
    h, w = gray.shape[:2]
    return cv2.resize(gray, (max(8, w // f), max(8, h // f)), interpolation=cv2.INTER_AREA)


def auto_level(small, lo_pct=2.0, hi_pct=90.0, ratio=0.62, floor=45.0, ceil=160.0):
    """自适应二值阈值：压在"胶带（暗）"与"纸面（亮）"之间。

    取 90 分位当亮面基准，阈值 = ratio × 它：实测纸面约 190、胶带约 50，
    0.62×190≈118 正好卡在中间；纸面随曝光变暗时阈值同步下移，不会失效。
    分位数只在 1/16 的像素上算（步长 4），两次 numpy 调用就够。
    """
    sub = small[::4, ::4]
    lo = float(np.percentile(sub, lo_pct))
    hi = float(np.percentile(sub, hi_pct))
    return float(np.clip(ratio * hi, max(float(floor), lo + 15.0), float(ceil)))


def dark_mask(small, level):
    """暗于 level 的像素置 255。胶带、门缝、阴影都进来，靠后面的形状门筛掉。"""
    return cv2.threshold(small, float(level), 255, cv2.THRESH_BINARY_INV)[1]


def adaptive_mask(small, block=31, c=12.0):
    """自适应二值：比"局部均值 − C"暗的像素置 255。

    为什么不用全局阈值：框体的绝对灰度随曝光漂（白墙场景 ~110，暗室 ~50），
    全局阈值永远在"框没变暗"和"背景影子被算进来"之间摇摆。
    自适应只回答"这块是不是比它周围暗"—— 框夹在亮纸和亮墙之间，天然成立；
    大片均匀的暗背景局部均值和它一样，不会被算成暗。
    block 必须大于框带宽（半分辨率下框带 ~10px，取 31），否则框会被均值吃掉。
    """
    k = int(block) | 1
    return cv2.adaptiveThreshold(small, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY_INV, k, float(c))


def components(mask):
    """连通域统计：一次调用拿到所有块的 (x, y, w, h, 面积) 与质心。

    比 findContours + 逐轮廓 boundingRect 快一个数量级 —— 板上每次 Python 级调用
    约 1ms，几百个轮廓就是几百毫秒，而这里只有一次。
    """
    n, _lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    if n <= 1:
        return np.zeros((0, 5), np.int32), np.zeros((0, 2), np.float64)
    return stats[1:], cent[1:]


def frame_candidates(stats, cent, mask, gray, level, min_area, max_area_ratio,
                     min_aspect, max_aspect, min_fill, max_fill, min_contrast,
                     rounds, probe=8):
    """挑"能从框心扫出一个内框、且框内明显比框体亮"的连通域。

    三道门：形状（面积/长宽比/填充率） → **能扫出内框** → **框内比框体亮**。
      · "能扫出内框"：从框心向四个方向找第一个"亮→暗"的跨越（见 `paper_rect`）。
        这一步对倾斜免疫——只要结构是"暗带围住一块亮区"，扫线必然四向命中；
        而窗棂、柜门缝、洞洞板这种稀疏暗结构扫不出一个占外接框大半的内框。
        ★ 早期用"外接框四条边压暗带"当判据，靶纸一倾斜（约 10°）就整帧漏检，
        因为斜框的外接框边缘大部分是亮背景。换成扫内框后不再依赖正置假设。
      · "框内比框体亮"：区分"白纸压在深色板/胶带上"与"同灰度的暗背景"。
        对比度取 内框灰度 − 框体灰度（框体=该连通域的暗像素均值），旋转无关。
    返回**所有被扫描过的候选**（带 ok 标记与各项实测值），由调用方决定如何排序 ——
    排序规则属于业务，不属于算法层。
    """
    h, w = mask.shape[:2]
    total = float(h * w)
    if stats.size == 0:
        return []
    sx = stats[:, 0].astype(np.float64)
    sy = stats[:, 1].astype(np.float64)
    bw = stats[:, 2].astype(np.float64)
    bh = stats[:, 3].astype(np.float64)
    area = stats[:, 4].astype(np.float64)
    box = bw * bh
    short = np.minimum(bw, bh)
    aspect = np.maximum(bw, bh) / np.maximum(1.0, short)
    fill = area / np.maximum(1.0, box)
    keep = ((short >= 14.0) & (area >= float(min_area)) &
            (box <= total * float(max_area_ratio)) &
            (aspect >= float(min_aspect)) & (aspect <= float(max_aspect)) &
            (fill >= float(min_fill)) & (fill <= float(max_fill)))
    idx = np.flatnonzero(keep)
    if idx.size == 0:
        return []
    idx = idx[np.argsort(-box[idx])][:int(probe)]
    out = []
    for i in idx:
        x0, y0 = int(sx[i]), int(sy[i])
        w0, h0 = int(bw[i]), int(bh[i])
        inner = paper_rect(gray, (x0, y0, w0, h0), level, rounds)
        if inner is None:
            continue                     # 扫不出内框 → 不是"暗带围住亮区"的结构
        ix0, iy0, ix1, iy1 = inner
        rw = (ix1 - ix0) / max(1.0, float(w0))
        rh = (iy1 - iy0) / max(1.0, float(h0))
        if not (0.40 <= rw <= 0.98 and 0.40 <= rh <= 0.98):
            continue                     # 内框太小/太偏：扫到的是内部某个暗点
        crop = gray[y0:y0 + h0, x0:x0 + w0]
        body = crop[mask[y0:y0 + h0, x0:x0 + w0] > 0]
        comp_mean = float(body.mean()) if body.size else float(crop.mean())
        paper = float(gray[iy0:iy1, ix0:ix1].mean())
        contrast = max(0.0, paper - comp_mean)
        ok = contrast >= float(min_contrast)
        out.append(dict(x=x0, y=y0, w=w0, h=h0, area=float(area[i]),
                        aspect=float(aspect[i]), fill=float(fill[i]),
                        edge=min(rw, rh), paper=paper, body=comp_mean,
                        contrast=contrast, ok=ok))
    return out



def nearest_component(stats, cent, target, ref_w, ref_h, size_tol=0.55, dist_ratio=1.2):
    """在连通域里挑离预测中心最近、尺寸与上一帧相当的那块；没有则 None。

    跟踪时用它：ROI 里通常只有胶带环这一块尺寸对得上，其它碎块要么太小、
    要么离预测中心太远。尺寸容差 ±55% 覆盖"车在靠近/远离"的情况。
    """
    if stats.size == 0:
        return None
    dx = cent[:, 0] - float(target[0])
    dy = cent[:, 1] - float(target[1])
    dist = np.hypot(dx, dy)
    wx = stats[:, 2] / max(1.0, float(ref_w))
    hy = stats[:, 3] / max(1.0, float(ref_h))
    ok = (wx > 1.0 - float(size_tol)) & (wx < 1.0 + float(size_tol)) & \
         (hy > 1.0 - float(size_tol)) & (hy < 1.0 + float(size_tol))
    ok &= dist < float(dist_ratio) * max(float(ref_w), float(ref_h))
    if not np.any(ok):
        return None
    idx = np.flatnonzero(ok)
    k = int(idx[np.argmin(dist[idx])])
    return dict(x=int(stats[k, 0]), y=int(stats[k, 1]),
                w=int(stats[k, 2]), h=int(stats[k, 3]),
                area=float(stats[k, 4]), cx=float(cent[k, 0]), cy=float(cent[k, 1]))


def _outer_dark(seg, level):
    """一维剖面里最外圈暗游程的起点（相对 seg 起点的偏移）；没有暗则 None。

    seg 从框心出发向外。暗游程按"相邻索引差 > 2"分组（1~2px 的反光断口不算新游程），
    最后一组就是最靠外的暗带 —— 对"印刷环嵌在纸面里"的结构，它必然是框体本身，
    其起点即纸边。
    """
    idx = np.flatnonzero(seg < level)
    if idx.size == 0:
        return None
    if idx.size == 1:
        return int(idx[0])
    gap = np.flatnonzero(np.diff(idx) > 2)
    return int(idx[gap[-1] + 1]) if gap.size else int(idx[0])


def paper_rect(gray, box, level, rounds=1):
    """框心向外扫四条线，返回纸边 (left, top, right, bottom)；扫不到返回 None。

    为什么从里往外扫：胶带环的外接框总是偏的（下缘常被阴影撑宽，实测偏 6~8px），
    而纸面一定在框内。从框心向四个方向找第一个"亮→暗"的跨越，取左右/上下的中点 ——
    对中心对称图形，过任意点的水平弦中点仍在中心线上，所以一轮就足以把偏差消掉。
    """
    x, y, w, h = int(box[0]), int(box[1]), int(box[2]), int(box[3])
    if w < 12 or h < 12:
        return None
    cx, cy = x + w // 2, y + h // 2
    best = None
    for _ in range(max(1, int(rounds))):
        i, j = cx - x, cy - y
        # 每个方向取相邻三条线的最小值再扫：胶带反光会让单独一行整段变亮，
        # 三行取最小等于"只要有任意一行是暗的就算暗"，能跨过 1~2px 的断口。
        row = gray[max(y, cy - 1):min(y + h, cy + 2), x:x + w].min(axis=0)
        col = gray[y:y + h, max(x, cx - 1):min(x + w, cx + 2)].min(axis=1)
        # 沿每个方向取"最外圈暗带的内侧边界"= 纸边。
        # 不能取第一个暗点：纸面上印的同心环也是暗的，第一下就撞在环上，
        # 内框会缩成一个点；也不能取最后一个暗像素：那会跑到框的外沿。
        # 正确做法是按暗游程分组，取最后一组（最靠外的那圈）的起点。
        # 某个方向完全扫不到暗（纸边贴着外接框）就退回用外接框那条边：
        # 缺一角不该让整帧失败，缺的那条边本来就在外接框上。
        # 比较电平用这条线自己的亮暗中点（p10/p90 的均值），不依赖全局阈值 ——
        # 框的绝对灰度随曝光漂，但"框比纸暗"这件事不变。
        lv_r = 0.5 * (float(np.percentile(row, 10)) + float(np.percentile(row, 90)))
        lv_c = 0.5 * (float(np.percentile(col, 10)) + float(np.percentile(col, 90)))
        step = _outer_dark(row[i:], lv_r)
        right = x + w - 1 if step is None else x + i + step
        step = _outer_dark(row[:i + 1][::-1], lv_r)
        left = x if step is None else x + i - step
        step = _outer_dark(col[j:], lv_c)
        bottom = y + h - 1 if step is None else y + j + step
        step = _outer_dark(col[:j + 1][::-1], lv_c)
        top = y if step is None else y + j - step
        if right - left < 12 or bottom - top < 12:
            break
        best = (left, top, right, bottom)
        ncx, ncy = (left + right) // 2, (top + bottom) // 2
        if abs(ncx - cx) <= 1 and abs(ncy - cy) <= 1:
            break
        cx, cy = ncx, ncy
    return best


# ================================================================== 偏差
def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定：dx = u靶心 − u光斑；电机方向由各轴 DIR_SIGN 映射，像素误差为正不代表
    固定的电机方向。激光丢失或黑框无效时返回 None。
    """
    if not laser or not laser.get("found"):
        return None                        # ★ 必须先看 found：found=False 时 u/v 只是标称占位
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])
    else:
        tx, ty = float(result["u"]), float(result["v"])
    lu, lv = float(laser['u']), float(laser['v'])
    if not all(map(math.isfinite, (tx, ty, lu, lv))):
        return None
    size = result.get('image_size')
    if size is not None and not (0 <= lu < size[0] and 0 <= lv < size[1]):
        return None
    return (tx - lu, ty - lv)
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

# ===== vision.output  (vision/output.py) =====
_MOD_SRC['vision.output'] = r'''
"""④ 执行层：**唯一允许产生副作用的地方** —— 画布、屏幕、串口、脉冲、停机、日志。

四层之一，两类内容合成一文件：看得见的 + 会动的。

    画面  draw_result / draw_laser_overlay / display_result / aligned_frame
          Preview（画布池 + 独立显示线程，show() 慢也只掉预览帧）
          draw_preview（组帧 → 投递渲染）
    云台  cmd_* / frame_status（ZDT 协议帧，纯字节构造）
          estimate_move_seconds（梯形/三角速度模型 → 运动耗时）
          StepperLink（双串口链路：非阻塞收发 + ACK/E2 看门狗）
          stop_motion / apply_commands（把逻辑层的决策真正下发）
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
    """右上角打 BUILD_ID 短标识：不接串口也能确认板子上跑的是哪一版。"""
    tag = result.get("build")
    if not tag:
        return
    short = str(tag).split("-", 3)[-1] if str(tag).count("-") >= 3 else str(tag)
    text = "v" + short
    (tw, _th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .38, 1)
    cv2.putText(canvas, text, (max(4, canvas.shape[1] - tw - 8), 18 + top_offset),
                cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 220, 255), 1)


def draw_result(gray, result, show_width=480, top_offset=0):
    """把一帧结果画成 BGR 画布。坐标一律按 scale 换算，文字仍报原图坐标。

    绿色四边形 = 胶带外沿，青色线 = 纸边（定心用），蓝色十字 = 靶心。
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

    q = np.rint(np.asarray(result["quad"], np.float32) * scale).astype(np.int32)
    cv2.polylines(canvas, [q], True, (0, 255, 0), 2, cv2.LINE_AA)
    for x, y in q:
        cv2.circle(canvas, (int(x), int(y)), 3, (255, 0, 255), -1)

    # 内框（纸边）：定心用的就是它，画出来便于核对是否贴合。
    inner = result.get("inner_quad")
    if inner is not None:
        iq = np.rint(np.asarray(inner, np.float32) * scale).astype(np.int32)
        cv2.polylines(canvas, [iq], True, (255, 200, 0), 1, cv2.LINE_AA)

    bbox = result.get("bbox")
    if bbox:
        x, y, bw, bh = bbox
        cv2.rectangle(canvas, (int(round(x * scale)), int(round(y * scale))),
                      (int(round((x + bw) * scale)), int(round((y + bh) * scale))),
                      (0, 220, 255), 1)

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
    """建 Display。stream=True（默认）保留 MaixVision 图传；False 才强制本地屏。

    历史坑：早期为了绕开"图传未就绪时 send_image() 忙等 500ms"，在建 Display 前把
    MAIXVISION 置 0。副作用是**图传被彻底关掉**——从 MaixVision 运行也看不到画面。
    现在默认不碰这个变量；慢的问题交给显示线程（show 再慢只掉预览帧，不拖识别）。
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
        if not bool(getattr(cfg, "SHOW", False)):
            print("[preview] SHOW=False -> 不建显示（纯串口模式，画面不流转）")
            return
        try:
            from maix import display
            self.stream_on = bool(getattr(cfg, "SHOW_STREAM", True))
            # 本地屏一律用 MAIXVISION=0 建 Display：内置图传在"连接未就绪"时会在 C++ 里
            # 忙等 500ms 并占住解释器锁，把识别拖到 20 秒/帧。要 MaixVision 画面就走
            # 下面的独立进程（`vision/maixvision_stream.py`），它不碰主线程。
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
            cv2.putText(canvas, status, (8, self.render_height - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
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


# ================================================================== 协议帧
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


def _readback_pin(pinmap, pin):
    """回读引脚当前实际功能（"某轴不通"的第一手证据）。失败返回 '?'。"""
    try:
        return pinmap.get_pin_function(pin)
    except Exception:
        return "?"



# ================================================================== 双轴链路
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
    """把 `{'yaw': (ccw, pulses, rpm)|None, 'pitch': ...}` 真正下发。

    返回 `(ok, failed_axis, applied)`：

        ok          全部下发成功为 True；任一失败即 False（此时已停机并置 FAULT）
        failed_axis 失败的轴名，成功为 None
        applied     实际发出去的 `{axis: command}`，供逻辑层回填 `self.commands`

    顺序：按 dict 迭代顺序逐个 `link.move`，首个失败立即停机返回。
    """
    applied = {}
    if link is None:
        return True, None, applied
    for axis, command in commands.items():
        if command is None:
            continue
        ccw, pulses, rpm = command
        try:
            ok = link.move(axis, ccw, pulses, rpm, acc_track, now=now)
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
        split_tag = " det_path=%s det_split=%.1f/%.1f" % (
            "track" if "track" in split else "search",
            split.get("track", 0.0), split.get("search", 0.0))
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

    结果契约   fail_result / success_result / locate_result / finish_result
    检测器     RectangleCenterDetector（降采样搜框 + ROI 跟踪 + 速度预测）
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
import time

import cv2
import numpy as np

from .algo import (adaptive_mask, auto_level, box_center, components,
                   dark_mask, deviation, downscale, frame_candidates,
                   nearest_component, paper_rect, rect_quad)
from .output import (Preview, aligned_frame, apply_commands, display_result,
                    draw_preview, estimate_move_seconds, perf_line, stop_motion,
                    StepperLink)
from .poll import FramePoller, FrameRate, LatestFrameWorker, to_gray_frame

BUILD_ID = "2026-11-09-fast-track-v15"

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


def _inner_quad(box, offset):
    """由外接框与"纸边相对框心的偏移比例"还原纸边矩形（只用于绘制与核对）。"""
    x, y, w, h = (float(v) for v in box)
    ox, oy = float(offset[0]), float(offset[1])
    iw = max(4.0, w * (1.0 - 2.0 * abs(ox)))
    ih = max(4.0, h * (1.0 - 2.0 * abs(oy)))
    cx = x + w * 0.5 + ox * w
    cy = y + h * 0.5 + oy * h
    return rect_quad(cx - iw * 0.5, cy - ih * 0.5, cx + iw * 0.5, cy + ih * 0.5)


# ================================================================== 编排
class RectangleCenterDetector:
    """黑框检测 + 跨帧跟踪。一帧只有十来次 cv2/numpy 调用。

    为什么这样组织（每一条都是板上实测出来的）：

    - **降采样 2 倍**再找框：像素少四倍，胶带环在 1/2 图上依旧连续。
    - **自适应阈值**（90 分位 × 0.62）：压在胶带与纸面之间，曝光变了阈值跟着变。
    - **连通域**而不是 findContours：一次调用拿到所有块，省掉几百次 Python 级调用。
    - **锁定后只在 ROI 里找**：ROI 约为框的 1.6 倍，像素只有全图的 1/6。
    - **速度预测 + 短暂保持**：画面晃一下、或者某一帧阈值没抓住，用上一帧的速度
      外推继续输出，不再立刻掉进 HOLD。
    - **中心取"框心向外扫四条线"的中点**：外接框中心会被阴影带偏 6~8px，
      弦中点不会（见 `algo.paper_rect`）。
    """

    def __init__(self, config):
        self.cfg = config
        self.scale = int(getattr(config, "RECT_DOWNSCALE", 2))
        self.level_ratio = float(getattr(config, "RECT_LEVEL_RATIO", 0.62))
        self.level_floor = float(getattr(config, "RECT_LEVEL_FLOOR", 45.0))
        self.level_ceil = float(getattr(config, "RECT_LEVEL_CEIL", 160.0))
        self.level_refresh = int(getattr(config, "RECT_LEVEL_REFRESH", 15))
        self.min_area = float(getattr(config, "RECT_MIN_AREA", 300))
        self.max_area_ratio = float(getattr(config, "RECT_MAX_AREA_RATIO", 0.55))
        self.min_aspect = float(getattr(config, "RECT_MIN_ASPECT", 1.05))
        self.max_aspect = float(getattr(config, "RECT_MAX_ASPECT", 2.60))
        self.min_fill = float(getattr(config, "RECT_MIN_FILL", 0.03))
        self.max_fill = float(getattr(config, "RECT_MAX_FILL", 0.70))
        self.paper_rounds = int(getattr(config, "RECT_PAPER_ROUNDS", 2))
        self.track_expand = float(getattr(config, "RECT_TRACK_EXPAND", 1.60))
        self.track_margin = float(getattr(config, "RECT_TRACK_MARGIN", 16.0))
        self.size_tol = float(getattr(config, "RECT_TRACK_SIZE_TOL", 0.55))
        self.hold = int(getattr(config, "RECT_HOLD_FRAMES", 6))
        self.smooth = float(getattr(config, "RECT_SMOOTH", 0.55))
        self.hold_max = float(getattr(config, "RECT_HOLD_MAX_PX", 90.0))
        self.max_speed = float(getattr(config, "RECT_MAX_SPEED", 2500.0))
        self.gate = float(getattr(config, "RECT_TRACK_GATE", 0.45))
        self.track_max_side = float(getattr(config, "RECT_TRACK_MAX_SIDE", 240.0))
        self.min_contrast = float(getattr(config, "RECT_MIN_BAND_CONTRAST", 30.0))
        self.adapt_block = int(getattr(config, "RECT_ADAPT_BLOCK", 31))
        self.adapt_c = float(getattr(config, "RECT_ADAPT_C", 12.0))
        self.prior = (
            float(getattr(config, "LASER_U_NOMINAL", 0.0)) if getattr(config, "ENABLE_LASER", False) else -1.0,
            float(getattr(config, "LASER_V_NOMINAL", 0.0)))
        if not getattr(config, "ENABLE_LASER", False):
            self.prior = (-1.0, -1.0)
        self._last_log = -1e9
        self.debug_map = bool(getattr(config, "RECT_DEBUG_MAP", False))
        self._printed_map = False
        self.keep_stages = bool(getattr(config, "RECT_KEEP_STAGES", False))
        self.last = None
        self.last_stages = {}
        self.reset()

    def reset(self):
        """清空跟踪状态。无历史帧时等价于"从头开始搜框"。"""
        self.level = None
        self.center = None          # 平滑后的中心（全分辨率）
        self.measured = None        # 最近一次实测中心
        self.velocity = (0.0, 0.0)  # px/s
        self.box = None             # 最近一次 bbox（全分辨率）
        self.ref = (0.0, 0.0)       # 锁定时的框尺寸，跟踪时作为尺寸基准
        self.offset = (0.0, 0.0)    # 纸边中心相对框心的偏移（按框尺寸归一）
        self.lost = 0
        self.tracking = False
        self._frames = 0
        self._last_components = 0
        self._printed_map = False
        self._last_ts = None
        self._hit_ts = None

    # ------------------------------------------------------------- 内部工具
    def _debug_map(self, small, level):
        """把降采样后的灰度图打成 64×32 的十六进制字符画（一次搜索只打一次）。

        现场调阈值时，这是唯一能"看见板子看到什么"的手段：每格是一个十六进制位
        （0=最暗 f=最亮），配合 lv 就能判断阈值该往上还是往下。
        """
        if not self.debug_map or self._printed_map:
            return
        self._printed_map = True
        m = cv2.resize(small, (64, 32), interpolation=cv2.INTER_AREA)
        print("[map] ---- level=%.0f (0=dark f=bright)" % level)
        for row in m:
            print("[map] " + "".join("%X" % (v >> 4) for v in row))

    def _log_candidates(self, cands, s, level, level_b=None):
        """搜框时把候选打一行日志（限频 1s），现场排障靠它，别删。"""
        now = time.perf_counter()
        if now - self._last_log < 1.0:
            return
        self._last_log = now
        parts = []
        for c in cands[:5]:
            parts.append("%s(%d,%d %dx%d e=%.2f con=%.0f%s)" % (
                "D",
                c["x"] * s, c["y"] * s, c["w"] * s, c["h"] * s, c["edge"],
                c["contrast"], "" if c["ok"] else "/x"))
        print("[cand] lv=%.0f lvb=%.0f n=%d %s"
              % (level, -1.0 if level_b is None else level_b, len(cands),
                 " ".join(parts) or "-"))

    def _level_for(self, img):
        return auto_level(img, 2.0, 90.0, self.level_ratio,
                          self.level_floor, self.level_ceil)

    def _predict(self, ts):
        """按速度外推当前位置；限幅防止甩飞。"""
        if self.center is None:
            return None
        dt = 0.0 if self._last_ts is None else float(ts) - self._last_ts
        dt = 0.0 if dt < 0.0 else (0.25 if dt > 0.25 else dt)
        px = self.center[0] + self.velocity[0] * dt
        py = self.center[1] + self.velocity[1] * dt
        lo_x, hi_x = self.center[0] - self.hold_max, self.center[0] + self.hold_max
        lo_y, hi_y = self.center[1] - self.hold_max, self.center[1] + self.hold_max
        return (lo_x if px < lo_x else (hi_x if px > hi_x else px),
                lo_y if py < lo_y else (hi_y if py > hi_y else py))

    @staticmethod
    def _fail_reason(stats):
        if not stats.get("components"):
            return "no_proposal"
        return "all_filtered"

    # ------------------------------------------------------------- 全图搜框
    def _search(self, g, stats):
        s = self.scale
        small = downscale(g, s)
        level = self._level_for(small)
        mask = adaptive_mask(small, self.adapt_block, self.adapt_c)
        self._debug_map(small, level)
        comps, _cent = components(mask)
        stats["components"] = int(comps.shape[0])
        area_min = self.min_area / float(s * s)
        # 暗环法：背景比纸亮（亮墙 + 黑胶带）时才成立。
        dark = frame_candidates(comps, _cent, mask, small, level, area_min,
                                self.max_area_ratio, self.min_aspect, self.max_aspect,
                                self.min_fill, self.max_fill, self.min_contrast,
                                self.paper_rounds)
        cands = dark
        cands = dark
        self._log_candidates(cands, s, level)
        good = [c for c in cands if c["ok"]]
        if not good:
            return None
        # 通过全部判据的候选按"离预期瞄准点最近"排序：操作手把相机对准靶纸，
        # 所以离光斑标称位置最近的那一个才是靶纸。面积最大不代表最像靶纸 ——
        # 实测洞洞板面积更大、四边也有暗孔，只有"框内比框边亮"能把它筛掉。
        px, py = self.prior
        if px < 0.0 or py < 0.0:
            px, py = g.shape[1] * 0.5, g.shape[0] * 0.5
        good.sort(key=lambda c: (abs((c["x"] + c["w"] * 0.5) * s - px) +
                                 abs((c["y"] + c["h"] * 0.5) * s - py),
                                 -c["contrast"]))
        c = good[0]
        paper = paper_rect(small, (c["x"], c["y"], c["w"], c["h"]), level,
                           self.paper_rounds)
        box = (c["x"] * s, c["y"] * s, c["w"] * s, c["h"] * s)
        bcx, bcy = box_center(box)
        if paper is not None:
            pcx = (paper[0] + paper[2]) * 0.5 * s
            pcy = (paper[1] + paper[3]) * 0.5 * s
        else:
            pcx, pcy = bcx, bcy
        return dict(center=(pcx, pcy), box=box, level=level, reacquire=True,
                    offset=((pcx - bcx) / max(1.0, box[2]),
                            (pcy - bcy) / max(1.0, box[3])),
                    area=c["area"] * s * s, aspect=c["aspect"], edge=c["edge"],
                    contrast=c["contrast"])

    # ------------------------------------------------------------- ROI 跟踪
    def _track(self, g, ts):
        box = self.box
        pred = self._predict(ts)
        if box is None or pred is None:
            return None
        s = self.scale
        h, w = g.shape[:2]
        hw = box[2] * self.track_expand * 0.5 + self.track_margin
        hh = box[3] * self.track_expand * 0.5 + self.track_margin
        x0, y0 = int(pred[0] - hw), int(pred[1] - hh)
        x1, y1 = int(pred[0] + hw), int(pred[1] + hh)
        x0, y0 = (0 if x0 < 0 else x0), (0 if y0 < 0 else y0)
        x1, y1 = (w if x1 > w else x1), (h if y1 > h else y1)
        if x1 - x0 < 24 or y1 - y0 < 24:
            return None
        # 尺寸参考用"锁定时的尺寸"而不是上一帧的尺寸：逐帧比较会一步步缩到更小的
        # 连通域（实测 250x180 -> 114x112 -> 50x52，最后锁到里面的圆环上）。
        ref_w = max(8.0, self.ref[0])
        ref_h = max(8.0, self.ref[1])
        # 跟踪时的降采样按 ROI 尺寸自适应：把每帧送进连通域的像素量钉在固定上限，
        # 否则目标一变大，连通域耗时就跟着涨（实测 3ms -> 34ms）。
        span = max(x1 - x0, y1 - y0)
        while span / s > self.track_max_side:
            s += 1
        small = downscale(g[y0:y1, x0:x1], s)
        self._frames += 1
        if self.level is None or self._frames % self.level_refresh == 0:
            self.level = self._level_for(small)
        mask = adaptive_mask(small, self.adapt_block, self.adapt_c)
        comps, cent = components(mask)
        self._last_components = int(comps.shape[0])
        c = nearest_component(comps, cent, ((pred[0] - x0) / s, (pred[1] - y0) / s),
                              ref_w / s, ref_h / s, self.size_tol)
        if c is None:
            return None
        # ROI 原点是原图坐标，只有块内坐标才乘缩放系数（乘错一次就差半个画面）。
        fbox = (x0 + c["x"] * s, y0 + c["y"] * s, c["w"] * s, c["h"] * s)
        bcx, bcy = box_center(fbox)
        cx = bcx + self.offset[0] * fbox[2]
        cy = bcy + self.offset[1] * fbox[3]
        # 跟踪门：测量值离预测太远说明锁到了别的东西（或上帧预测跑飞），当帧作废。
        gate = self.gate * max(box[2], box[3])
        if abs(cx - pred[0]) > gate or abs(cy - pred[1]) > gate:
            return None
        return dict(center=(cx, cy),
                    box=fbox, level=self.level, offset=self.offset,
                    area=c["area"] * s * s,
                    aspect=max(fbox[2], fbox[3]) / max(1.0, min(fbox[2], fbox[3])),
                    edge=None)

    # ------------------------------------------------------------- 状态推进
    def _on_hit(self, det, ts):
        meas = det["center"]
        # 速度只在帧间隔足够长时更新：连拍两帧 dt≈0.2ms，除出来是天文数字，
        # 外推会把 ROI 甩到画面另一头（实测会锁到墙上的别的东西）。
        if self.measured is not None and self._hit_ts is not None:
            dt = ts - self._hit_ts
            if 0.004 <= dt <= 0.25:
                a = 0.55
                vx = self.velocity[0] * (1.0 - a) + (meas[0] - self.measured[0]) / dt * a
                vy = self.velocity[1] * (1.0 - a) + (meas[1] - self.measured[1]) / dt * a
                speed = (vx * vx + vy * vy) ** 0.5
                if speed > self.max_speed:
                    vx *= self.max_speed / speed
                    vy *= self.max_speed / speed
                self.velocity = (vx, vy)
        # 重新捕获：直接对齐，不做低通。判定"重新捕获"的标准是全图搜框回来的
        # 结果离当前中心超过半个框 —— 这种情况多半是换场景/换靶，旧中心已经没意义。
        jumped = (det.get("reacquire") and self.center is not None and
                  abs(meas[0] - self.center[0]) + abs(meas[1] - self.center[1]) >
                  0.6 * max(det["box"][2], det["box"][3]))
        if self.center is None or self.lost > 0 or jumped:
            self.center = meas
        else:
            k = self.smooth
            self.center = (self.center[0] + k * (meas[0] - self.center[0]),
                           self.center[1] + k * (meas[1] - self.center[1]))
        self.measured = meas
        self.box = det["box"]
        if det.get("reacquire"):
            self.ref = (float(det["box"][2]), float(det["box"][3]))
        else:
            # 参考尺寸只允许慢速跟随（每帧 <=2%），这样"车在靠近"能跟上，
            # "跳到别的小块上"跟不上。
            self.ref = (self.ref[0] + max(-0.02 * self.ref[0], min(0.02 * self.ref[0],
                                                                 det["box"][2] - self.ref[0])),
                        self.ref[1] + max(-0.02 * self.ref[1], min(0.02 * self.ref[1],
                                                                 det["box"][3] - self.ref[1])))
        self.offset = det["offset"]
        self.level = det["level"]
        self.lost = 0
        self.tracking = True
        self._hit_ts = ts
        self._last_ts = ts
        return self._make_result(det, hold=False)

    def _on_missing(self, ts, stats):
        if self.center is None or self.box is None or self.lost >= self.hold:
            self.tracking = False
            return fail_result(STAGE_NONE, "plane", self._fail_reason(stats))
        self.lost += 1
        self.velocity = (self.velocity[0] * 0.7, self.velocity[1] * 0.7)
        pred = self._predict(ts)
        self.center = pred
        self._last_ts = ts
        det = dict(center=pred, box=self.box, offset=self.offset, level=self.level,
                   area=0.0, aspect=0.0, edge=None)
        return self._make_result(det, hold=True)

    def _make_result(self, det, hold):
        box = det["box"]
        cx, cy = self.center
        outer = rect_quad(box[0], box[1], box[0] + box[2], box[1] + box[3])
        edge = det.get("edge")
        return dict(valid=True, stage=STAGE_PLANE,
                    u=float(cx), v=float(cy),
                    raw_u=float(det["center"][0]), raw_v=float(det["center"][1]),
                    quad=outer, inner_quad=_inner_quad(box, det["offset"]),
                    detected_quad=outer, outer_inferred=False,
                    center_ref="paper",
                    area=float(det.get("area") or 0.0),
                    aspect=float(det.get("aspect") or 0.0),
                    bbox=(int(box[0]), int(box[1]), int(box[2]), int(box[3])),
                    partial=bool(hold), hold=bool(hold), lost=int(self.lost),
                    refined_edges=4, weak_edges=0, clipped_edges=0,
                    rect_to_image=None, circle_points=None,
                    band=dict(level=det.get("level"), edge=edge,
                              paper=det.get("paper"), body=det.get("body"),
                              contrast=float(det.get("contrast") or 0.0)),
                    confidence=float(1.0 if not hold else
                                     max(0.2, 1.0 - self.lost / float(max(1, self.hold)))))

    def _detect(self, g, ts):
        timings = {}
        stats = dict(components=0)
        self._last_components = 0
        det = None
        if self.tracking:
            t0 = time.perf_counter()
            det = self._track(g, ts)
            timings["track"] = (time.perf_counter() - t0) * 1000.0
        if det is None:
            t0 = time.perf_counter()
            det = self._search(g, stats)
            timings["search"] = (time.perf_counter() - t0) * 1000.0
            timings.pop("track", None)
        if self.tracking and det is not None and "search" not in timings:
            stats["components"] = self._last_components
        result = self._on_hit(det, ts) if det is not None else self._on_missing(ts, stats)
        return result, stats, timings

    # ------------------------------------------------------------- 对外入口
    def process(self, gray, timestamp=None):
        started = time.perf_counter()
        ts = started if timestamp is None else float(timestamp)
        if not np.isfinite(ts):
            raise ValueError("non-finite timestamp")
        g = to_gray_frame(gray)
        if g.size == 0:
            result, stats, timings = fail_result("empty_frame", "none", "empty"), \
                dict(components=0), {}
        else:
            result, stats, timings = self._detect(g, ts)
        result.update(masks=1, proposals=stats.get("components", 0), search_count=1,
                      reject=dict(stats), threshold=self.level, build=BUILD_ID)
        result["t_ms"] = timings
        self.last = finish_result(result, ts,
                                  (int(g.shape[1]), int(g.shape[0])) if g.size else (0, 0),
                                  started)
        return self.last


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
    - 每帧只发一帧**相对位置**命令（0xFD、mode=0），单帧脉冲限幅，``ACC_TRACK`` 曲线加减速；
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
        return self.aim.state if self.aim else 'vision'

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
    while not app.need_exit():
        started = clock.perf_counter()
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
        controller = state['controller'] = VisionControl(cfg, state['link'], state['aim'], clock=clock)
        worker = state['worker'] = LatestFrameWorker(controller.process, controller.idle,
                                                    cfg.VISION_ASYNC, clock=clock)
        print('[rect-center] %s scale=1/%d adapt=%d/%.0f display_width=%d display_fps=%d' %
              (BUILD_ID, cfg.RECT_DOWNSCALE, cfg.RECT_ADAPT_BLOCK, cfg.RECT_ADAPT_C,
               preview.render_width, cfg.SHOW_FPS))
        print('[preview] async=%s canvas=%dx%d' % (cfg.SHOW_ASYNC, preview.render_width, preview.render_height))
        print('[vision] latest-frame worker=%s max_result_age=%.0fms' % (cfg.VISION_ASYNC, cfg.RESULT_MAX_AGE_S*1000))
        _capture_loop(cfg, clock, app, image, state, BUILD_ID)
    finally:
        _release(state)
'''

# ===== vision  (vision/__init__.py) =====
_MOD_SRC['vision'] = r'''
"""vision 包：四个模块，一一对应四层。目录结构即分层结构。

    ① poll.py     轮询层  取帧 → 灰度 → 帧率 → 识别线程
    ② algo.py     算法层  纯函数：模糊 / 二值化 / 轮廓 / 几何 / 偏差
    ③ logic.py    逻辑层  结果契约 / 检测器 / 闭环决策 / 主循环 / 对外接口
    ④ output.py   执行层  绘制 / 屏幕 / 串口 / 电机 —— 唯一允许产生副作用处

依赖方向单向，不可反向：

    poll  ←──  algo
      ↑         ↑
      └── output ←── logic

    logic → {poll, algo, output}      output → {poll, algo}
    algo  → {}                        poll   → {}

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

    每帧都是一次完整的"轮廓候选 → 内外框亚像素拟合 → 8×8 单应 → 同心环校验"，
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

    ⚠ 使用前必须完成开环标定（把实测的 ``PULSE_PER_PX_*`` / ``DIR_SIGN_*`` 填回 config），
    否则方向/增益是占位值，可能正反馈甩头。不需要电机的问请只用 ``detect_black_frame`` /
    ``CenterTracker``，不要实例化本类。

    ``step(frame)`` 返回的 ``Result`` 比追踪器多两个字段：
        aim_state  str   状态机状态（FINE/COARSE/SETTLE/HOLD/FAULT…）
        cmds       dict  {"yaw": (ccw, pulses, rpm) or None, "pitch": ...}
    """

    def __init__(self, cfg=None, enable_motors=True, with_laser=True, clock=None):
        self.cfg = cfg if cfg is not None else load_config()
        self.clock = clock if clock is not None else time
        self.tracker = CenterTracker(self.cfg, with_laser=with_laser)
        self.link = self.aim = None
        self.closed = False
        if enable_motors:
            from vision.logic import GimbalAim
            from vision.output import StepperLink
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
        """兼容占位：本版本已无失锁扫描能力，调用只打一行提示，不驱动电机。"""
        print("[api] request_search: 本版本未编入搜索扫描（见 AGENTS.md），忽略")

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
            if self.link is not None:
                self.link.close()
'''

_MAIN_SRC = r'''
"""MaixPy v4 入口：只做路径引导，然后转交逻辑层。

四层结构（见 `docs/识别链路四层分层方案.md`）：

    vision/poll.py     ① 轮询层  取帧 → 灰度 → 帧率 → 识别线程
    vision/algo.py     ② 算法层  纯函数：模糊 / 二值化 / 轮廓 / 几何 / 偏差
    vision/logic.py    ③ 逻辑层  结果契约 + 检测器 + 闭环决策 + 主循环（本文件的业务本体）
    vision/output.py   ④ 执行层  绘制 / 屏幕 / 串口 / 电机 —— 唯一允许产生副作用处

**本文件属逻辑层**：MaixCAM 要求入口必须名为 `main.py` 且位于应用根目录，
所以这里只保留 `sys.path` 引导与一次转交。装配、三条时间轴调度、退出释放
全部在 `vision/logic.py`（`run` / `_capture_loop` / `_release`），两边不重复实现。

三条独立时间轴：
    A 采图（主线程）→ 投递最新帧给 B
    B 识别+闭环（LatestFrameWorker 线程，串口归它独占）
    C 显示（Preview 的 show 线程）
跨轴只传"带时间戳的最新结果"，过期结果整体判废、不进闭环（RESULT_MAX_AGE_S）。
"""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from vision.logic import run


def main():
    return run()


if __name__ == "__main__":
    try:
        main()
    except ModuleNotFoundError as exc:
        if exc.name != "maix":
            raise
        print("MaixPy SDK unavailable. PC: python -B tools/check_app.py")
'''

_MOD_ORDER = ('config', 'vision.poll', 'vision.algo', 'vision.maixvision_stream', 'vision.output', 'vision.logic', 'vision', 'target_api')

_REL_OF = {'config': 'config.py', 'vision.poll': 'vision/poll.py', 'vision.algo': 'vision/algo.py', 'vision.maixvision_stream': 'vision/maixvision_stream.py', 'vision.output': 'vision/output.py', 'vision.logic': 'vision/logic.py', 'vision': 'vision/__init__.py', 'target_api': 'target_api.py'}

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
# ② 主程序：main.py 原文照搬，exec 进本文件全局命名空间
#    这样 __name__ == "__main__" 判断、traceback 文件名都与原来一致
# --------------------------------------------------------------------------
exec(compile(_MAIN_SRC, 'main.py', "exec"), globals())
