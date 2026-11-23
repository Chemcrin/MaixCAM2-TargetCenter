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
