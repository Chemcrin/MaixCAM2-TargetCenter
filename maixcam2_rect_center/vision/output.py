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
        split_tag = " det_split=%.1f/%.1f/%.1f" % (
            split.get("blur", 0.0), split.get("bin", 0.0), split.get("cand", 0.0))
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
