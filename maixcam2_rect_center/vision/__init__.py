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
