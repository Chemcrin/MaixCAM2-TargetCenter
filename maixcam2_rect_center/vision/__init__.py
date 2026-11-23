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
