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
