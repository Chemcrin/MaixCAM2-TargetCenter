# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-11-12，fast-perspective-v17）
- `maixcam2_rect_center/` 四层：`vision/{poll,algo,logic,output}.py` 一文件一层；
  BUILD_ID 在 `vision/logic.py`（当前 `2026-11-12-fast-perspective-v17`），`main.py` 只是入口桩。
- 视觉引擎已换成 GPT-Replay 的 fast-perspective 实现（2026-11-12），旧 ring_hole 链路整体退役；
  云台闭环（GimbalAim / VisionControl / output）一行未动，靠 result 契约联动。
- 图传定案（2026-11-11）：SHOW_STREAM=True 走子进程图传（MAIXVISION=0 建屏+独立进程 JPEG/WS，
  不拖识别）；调优后 2fps 根因待板端复测确认（也可能是板端旧代码）。
- `GPT-Replay/` 保留为参考/备份，不再合入；其顶点顺序/中心口径有缺陷，见 §1。

## 1 视觉链路（Otsu 提案 + 带宽验证 + 四边精修 + 内框交点，ROI 跟踪）
- 提案：长边钉到 320 降采样 → 整幅 Otsu（本次无目标再试自适应阈值）→ 闭运算 → 连通域 +
  approxPolyDP；暗块外轮廓与亮面补集各一轮，去重后按面积取前 8。
- 验证：物理矩形（A4 210×297 纸面 + 胶带 BLACK_FRAME_MM=18mm）四边各取"暗-暗-亮"三点，
  门 = 暗/亮灰度比 ≤0.72 / 对比 ≥20 / 单边覆盖率 ≥0.55（允许一条边降到 0.25）/
  实测长短边比 ∈[1.05,3.4] / 拟线残差 ≤2px。
- 精修：当前帧沿边法线取 ±REFINE_RADIUS 灰度 → 梯度极值 → 每边 fitLine → 相邻求交；
  跟踪态半径用 TRACK_RADIUS=12。残差超限就退回提案值，不拿单帧失败换掉锁。
- **中心 = inner_quad（纸边）对角线交点**。⚠ GPT 原版取的是 detected quad（外框），
  胶带上下不等宽会偏半个带宽；这是移植时唯一的算法改动，已用 smoke 锁住语义。
- 带宽是**双向硬门**：换胶带规格必须改 BLACK_FRAME_MM，过窄过宽都稳定拒检。
- 已知边界：正方形框（长短边比 <1.05）拒；靶纸大半出画拒；角部遮挡 50×40px 仍能锁定；
  单边缺口到 45% 仍能检出。
- 固件核对（PC）：real_board_frame / real_scene_corner_gap 新旧链路都拒（前者是带灰遮挡块的
  截图，不是相机原始帧）；negative_e421/e880 旧链路误检、新链路拒。
- 依赖单向：poll ← algo ← output ← logic。


## 2 板端（MaixCAM2）
- 关键结论：**板上一次 Python 级 numpy 调用 ≈ 1ms**，所以耗时 ≈ 调用次数，
  不是像素数。任何"逐点/逐边/逐轮廓"的写法都会把单帧拖到几百毫秒。
- 图传（SHOW_STREAM）默认关：通道没建立时 `send_image()` 每帧空转 500ms 且卡解释器锁，
  能把识别拖到 20 秒/帧。要画面再临时开，代价是掉到 2fps。
- ⚠ 禁 pkill/kill（只走 MaixVision 停止按钮）。

## 3 纪律
- 文档口径：写架构不写实现，数值/行号/版本/日期不进文档；变更记录不进文档（git 即历史）；
  导航在 `docs/README.md`；禁止元叙述。
- 改代码：改 `vision/logic.py` 的 BUILD_ID → build_single.py → check_pc；新增模块同步 MODULES。
- ⚠ 本机 Edit/Write 写 CRLF ⇒ check_app C6 红；改完用 python 归一 LF。
- ⚠ 用 `bash -c "...python -c '...'"` 时，正文里的**反引号会被 shell 吃掉**：
  写文档/记忆要落成 .py 脚本再跑，别用内联。
- ⚠ main.py 曾被外部整份覆盖：改完立即 build_single --smoke + check_app --manifest。
- Python 一律 `D:/Python311/python.exe`；中文路径禁用 cv2.imread/imwrite。
- 板端 BUILD_ID 与本地逐字一致才算新代码。
- git：按路径精确 add，禁 `git add -A`；前缀 code:/docs:/tools:/repo:。
  工作区里 `GPT-Replay/` 常被别人暂存着，提交前先 `git restore --staged GPT-Replay`。

## 4 别踩的坑
- 应用目录曾冲到 35M（tools/precision_report 单目录 34M，v1.1.0 无 C11 门）。v1.3.0 的
  check_app 有 C11 30MB 硬门；产物一律落 `../reports/`。
- 耗时不可跨机器比较；「识别不给力」先取板端 [perf] 或原始帧。
- MaixVision：编辑器缓冲**不随磁盘重载**；菜单栏在副屏 y=-145 点不到，用 Ctrl+O 开对话框；
  编辑器 set_value 对 Monaco 无效。运行 (39,1415)、停止 (39,1482，仅运行中是方块)。
- 手感：PULSE_PER_PX_*=3.67（未标定占位）、ACC_TRACK=80、DEADBAND=1.0；
  GAIN/GAIN_COARSE/RPM/MAX_STEP 被 `apply_aim_profile("smooth")` 覆盖，读字面值会读错。
  实测闭环会过冲（dx 86 → -55），属云台标定问题，不是识别问题。
- 轴：ttyS0(U0,ID=1)=pitch、ttyS2(U2,ID=2)=yaw，正方向均 CCW；DIR_SIGN_*=-1 未标定，
  越追越偏就翻号。AIM_PROFILE="smooth" 用户明确保持。
