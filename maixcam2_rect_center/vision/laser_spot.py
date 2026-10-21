"""激光光斑读数（fixed 口径）：光斑位置写死，只算"靶心 − 光斑"偏差。

相机与激光笔刚性固连（同装在俯仰平台上）⇒ 光斑在图像里的位置是常量
（`LASER_U_NOMINAL` / `LASER_V_NOMINAL`，实机标定值）。因此不做任何图像
检测：启动时取一次 `FixedLaser.static_state()`，之后每帧唯一的运算是
`deviation()` 里的两次减法。

重标定：换镜头或重装激光笔后，看屏幕读数 `LASER (u,v)`（draw_laser_overlay
画的面板）或板端日志 `[perf] ... laser=` 反推真实位置，填回 config 两个常量。

输出契约（dict，全模块唯一形状）：
    found   bool    恒为 True（位置是常量，不存在"丢光斑"）
    updated bool    恒为 False（语义保留：不做重算）
    u, v    float   光斑位置，相机原图坐标
    reason  str     恒为 "fixed"
    ms      float   恒为 0（不做图像运算）
"""
import math

_PANEL_BG = (32, 32, 32)
PANEL_LINE = 18
PANEL_H = 2 * PANEL_LINE + 10   # 面板高度（显示画布坐标）；main 用它把框文字整体下移
_TEXT_C1 = (255, 255, 0)     # BGR 青：光斑状态行
_TEXT_C2 = (120, 255, 120)   # BGR 浅绿：偏差行（有效）
_TEXT_C3 = (120, 120, 255)   # BGR 浅红：偏差行（无靶纸）
_MARK = (0, 0, 255)          # BGR 红：光斑标记（跟激光同色）
_LINK = (0, 160, 255)        # BGR 橙：光斑->靶心连线


class FixedLaser:
    """写死的光斑位置：启动时构造一次，之后所有调用返回同一份常量。"""

    def __init__(self, config):
        self.nominal = (float(config.LASER_U_NOMINAL), float(config.LASER_V_NOMINAL))
        self._state = dict(found=True, updated=False, u=self.nominal[0], v=self.nominal[1],
                           reason="fixed", ms=0.0)

    def static_state(self):
        """常量结果 dict（内容不随帧变化，返回共享引用，调用方勿改）。"""
        return self._state


def deviation(laser, result, use_raw=False):
    """靶心相对光斑的偏差：dx>0 靶心在光斑右侧，dy>0 靶心在光斑下方。

    约定取自需求文档 §7.2（dx = u靶心 − u光斑），与后续串口直控云台同一套符号：
    电机方向由各轴 DIR_SIGN 映射；像素误差为正不代表固定的电机方向。

    use_raw：闭环必须用这一档（True）—— 取 result["raw_u"]/["raw_v"]（未平滑）。
    result["u"]/["v"] 是经 CENTER_ALPHA 一阶低通的值，只适合显示；拿它闭环会附加
    约 28ms 滞后（需求文档 §5.2），底盘运动时直接变成瞄准误差。
    黑框无效时返回 None（光斑是常量、恒 found，不会再因"丢光斑"返回 None）。
    """
    if not laser or not laser.get("found"):
        return None
    if not result or not result.get("valid"):
        return None
    if use_raw and result.get("raw_u") is not None and result.get("raw_v") is not None:
        tx, ty = float(result["raw_u"]), float(result["raw_v"])   # 闭环档：未平滑中心
    else:
        tx, ty = float(result["u"]), float(result["v"])           # 显示档：CENTER_ALPHA 平滑中心
    lu, lv = float(laser['u']), float(laser['v'])
    if not all(map(math.isfinite, (tx, ty, lu, lv))):
        return None
    size = result.get('image_size')
    if size is not None and not (0 <= lu < size[0] and 0 <= lv < size[1]):
        return None
    return (tx - lu, ty - lv)       # dx=靶心−光斑, dy 同理


def draw_laser_overlay(canvas, laser, result=None, scale=1.0, use_raw=False):
    """在显示画布左上角画激光读数面板 + 光斑标记 + 到靶心的连线。

    use_raw 与 deviation() 保持一致：True 时连线终点与 dx/dy 都用未平滑的 raw 中心，
    屏幕上看到的就是控制律真正在用的那个点。

    scale：**原图坐标 → 画布坐标** 的比例（main.py 传 render_width/原图宽）。
    本函数内部所有坐标（光斑 p、连线终点 c）都要乘 scale，才能与 draw_result 画的框对齐。
    **注意**：面板本身（左上角矩形与文字）用画布固定像素坐标，不随 scale 缩放。

    返回面板占用的高度（像素，显示画布坐标），调用方据此把 draw_result 的文字
    整体下移，避免和本面板重叠。
    """
    if not laser:
        return 0
    import cv2
    # 第 1 行：写死的光斑位置。
    line1 = "LASER (%d,%d) [fixed]" % (round(laser["u"]), round(laser["v"]))
    # 第 2 行：光斑→靶心偏差（用同一 use_raw 口径，与闭环一致）。
    dv = deviation(laser, result, use_raw=use_raw)
    if dv is not None:
        line2 = "D-CENTER dx=%+.1f dy=%+.1f px" % (dv[0], dv[1])
        line2_color = _TEXT_C2                       # 浅绿：有效
    elif result is not None and result.get("valid"):
        line2 = "D-CENTER -- (laser lost)"
        line2_color = _TEXT_C3                       # 浅红：有靶心无光斑（fixed 下不应出现）
    else:
        line2 = "D-CENTER -- (no black rect)"
        line2_color = _TEXT_C3                       # 浅红：无靶心
    # 面板宽度按两行文字较宽者自适应（固定画布像素，不随 scale 缩放）。
    w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0] for t in (line1, line2))
    cv2.rectangle(canvas, (2, 2), (min(canvas.shape[1] - 1, w + 14), PANEL_H - 2), _PANEL_BG, -1)
    cv2.putText(canvas, line1, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, _TEXT_C1, 1, cv2.LINE_AA)
    cv2.putText(canvas, line2, (8, 18 + PANEL_LINE), cv2.FONT_HERSHEY_SIMPLEX, .45, line2_color, 1, cv2.LINE_AA)
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
