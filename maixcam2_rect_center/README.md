# MaixCAM2 黑框中心识别与双轴云台瞄准

应用目录内唯一文档：说明这套系统**是什么结构、各部分管什么、关键机制为什么这样设计**。
具体数值与当前状态看 `config.py` 和代码注释；协作流程看仓库根 `AGENTS.md`；全部文档的入口看 `docs/README.md`。

## 1. 任务与思路

2025 电赛 E 题：识别 A4 靶纸上的靶心，引导云台上的激光指向。
靶心是红色记号笔点，在 165cm 处不足 1px，物理上不可见——因此不走"找红点"，
而是识别靶纸四周 1.8cm 黑胶带构成的矩形，用矩形中心（经透视校正）作为靶心像素坐标。

## 2. 代码结构（4 层 + 入口）

```
main.py              入口桩（属逻辑层）：sys.path 引导 → vision.logic.run
config.py            全部可调参数，现场只改这一个文件（层外共享，四层只读）
target_api.py        对外门面（属逻辑层）：detect_black_frame / CenterTracker / AimLoop
vision/poll.py       ① 轮询层：取帧 → 灰度、帧率计、识别线程（时间轴 B）
vision/algo.py       ② 算法层：纯函数（模糊 / 二值化 / 轮廓过滤 / 几何 / 偏差）
vision/logic.py      ③ 逻辑层：结果契约 + 检测器 + 闭环决策 + 主循环
vision/output.py     ④ 执行层：绘制 + 激光面板 + 显示线程（时间轴 C）+ ZDT 协议 + 下发
tools/               门禁与 PC 复现脚本：check_app / check_pc / build_single / run_pc（不进应用包）
```

`vision/` 刻意只留 4 个文件，**一个文件就是一层**，文件名即层名：

| 文件 | 层 | 一句话职责 | 硬约束 |
|---|---|---|---|
| `poll.py` | ① 轮询 | 把画面安全搬进识别线程 | 不产生识别判据；跨线程必须 copy；不 import 其余三层 |
| `algo.py` | ② 算法 | 算出靶心与光斑 | 纯函数：无状态、无 I/O、不读 `config` |
| `logic.py` | ③ 逻辑 | 决定该做什么 | 不调 `link.*`，只产出 `commands` 决策值 |
| `output.py` | ④ 执行 | 画到屏幕、把误差变成电机动作 | `link.move` / `link.stop` **只允许出现在这里** |

依赖方向单向：`poll ← algo ← output ← logic`（箭头指"被依赖"），反向一律禁止。
反向依赖属评审否决项：`check_app.py` 只核对包内清单与语法，不做依赖图检查，
改 `vision/` 时按上表逐条自查。

顶层不 import maix（`StepperLink`/`Preview` 构造时才 import），因此 4 层全部能在 PC 上导入与离线验证。

## 3. 三条时间轴

```
相机帧 ──► FramePoller ──► LatestFrameWorker（识别线程）
                              │  VisionControl.process()
                              │    ├─ RectangleCenterDetector → (u,v)
                              │    ├─ deviation → dx,dy
                              │    └─ GimbalAim → apply_commands → 串口
                              ▼
                         最新结果槽 ──► Preview.show 线程（只画，不控制）
```

- **A 采图**（主线程）：`cam.read()` → 可选镜头校正 → 灰度；`captured_at` 在 `read()` 后**立刻**取。
- **B 识别+闭环**（`rect-vision` 线程）：最新帧优先，忙时丢旧帧；**串口归它独占**。
- **C 显示**（`display` 线程）：`show()` 慢只掉预览帧，绝不拖慢 A/B。

跨轴只传"带时间戳的最新结果"；采图时刻算起超过 `RESULT_MAX_AGE_S`（0.20s）的结果
**不画框、不下发指令**，`[perf]` 里 stage 会被替换成 `stale`。

## 4. 识别链路

单帧固定五步，每步可单独出图验证（`run_pc.py --stage binary`）：

- **二值化是地基**：`adaptiveThreshold` 反色后黑胶带变白环。窗口与常数决定环是否闭合，
  这两个没调好，后面的过滤全部白费。
- **三层过滤是判据**：面积（含防止锁整幅的上限）→ 四边形（`approxPolyDP` 四点）
  → 长宽比（正立外接框 `w/h`，前提是靶纸横放）。逐层 `continue`，淘汰计数进结果里的 `reject` 字段。
- **中心是对角线交点**：四点已由近似得到，交点透视下无偏，不用外接框中心。
- **无跨帧状态**：每帧独立，同一张图必得同一结果，便于离线复现与现场调参。
- **已知边界**：遮挡与贴边不可用（外轮廓不闭合）、竖放或大角度倾斜会被长宽比拒、
  画面里更大的深色矩形会抢锁。取舍与补法见 `docs/视觉识别实现说明.md`。
- **合成图通过不代表真机可用**：厚胶带真机帧的候选生成不稳定，实测会出现"靶纸被漏检、
  背景小矩形反被锁上"的组合；误锁比漏检更危险（漏检只 HOLD，误锁会把云台带偏）。
  上板前必须用真机帧逐帧复核，不能只看合成图或单帧命中。

## 5. 云台链路

| 环节 | 位置 | 关键设计 |
|---|---|---|
| 协议帧 | `output.cmd_*` | 0x6B 校验、大端参数、0xFD 相对位置；纯函数可逐字节回归 |
| 双串口链路 | `output.StepperLink` | 非阻塞收发 + ACK/E2 看门狗（**按轴独立武装**） |
| 闭环 | `logic.GimbalAim` | 脉冲 = 误差px × `PULSE_PER_PX` × `GAIN`；死区/限幅/离群拒收/发散看护 |
| 仲裁与下发 | `logic.VisionControl` → `output.apply_commands` | 串口唯一所有者；停机失败立即终止决策链 |

轴映射与方向的**实机定案**（哪个串口是 yaw、正方向朝哪、pinmap 注意事项）
见仓库根 `AGENTS.md` §2——接错轴闭环必发散，动硬件前先读那张表。

> 本版本**不含**失锁扫描、自动标定、IMU 前馈、MaixVision 图传与屏幕按钮：
> 这些能力连同对应 config 开关已于 2026-11-02 一并移除，需要时按 `AGENTS.md` §0′ 的边界重新加回。

## 6. 入口、配置与对外 API

- `main.py` 只做装配与主循环；一切可调项在 `config.py`。
- `config.py` 末尾的 `apply_aim_profile(AIM_PROFILE)` 会在加载时**覆盖**六个增益/转速/限幅值——
  改手感先看它，别直接改被覆盖的字面值（机制详见 `AGENTS.md` §3.1）。
- `target_api.py` 是对外 API：`detect_black_frame` / `CenterTracker` / `AimLoop`，
  供其他小问直接复用，不依赖屏幕与云台硬件（云台部分可选）。

## 7. 上板与排障入口

- 板端日志 `[rect-center] <BUILD_ID>` 必须与本地逐字一致，否则跑的是旧代码；
- `[perf]` 日志看各阶段耗时；帧率异常先查新鲜度门链路；
  未锁定时看行尾 `det_fail=full/<原因>`：`all_area` 是轮廓全被面积门淘汰（阈值或二值化），
  `all_filtered` 是有轮廓但没凑出合格四边形/长宽比，`no_proposal` 是二值图里没有轮廓——
  三者改法完全不同；分阶段调参见 `docs/视觉识别实现说明.md`；
- 云台不动先看 `[gimbal]` 的 pinmap readback 与 `[aim] state=`；方向反了看 `AGENTS.md` §2。

## 8. 开发流程

门禁命令、BUILD_ID 规则、git 规范、完成定义——全部见仓库根 `AGENTS.md`，此处不重复。
