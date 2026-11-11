# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-11-10，quad-track-v15）
- `maixcam2_rect_center/` 四层：`vision/{poll,algo,logic,output}.py` 一文件一层；
  BUILD_ID 在 `vision/logic.py`（当前 `2026-11-09-quad-track-v15`），`main.py` 只是入口桩。
- 最新提交 `f26cf3c`（归档 dist 探针）；v15 引擎链在 `41ce5d3`，GPT-Replay 优化快照在 `4e63aa6`。
- v10→v15 演进见 2026-11-10 之前流水：环-洞引擎（ring_hole）替代弦中点，中心=洞内框
  对角线交点；候选上限 4、暗带检查裁剪到洞邻域、打分制选洞。
- ⚠ config.py 当前 `SHOW_STREAM=True`（2026-11-10 观察），与"默认关"结论不一致，上板前确认。
- `GPT-Replay/` 是另一条并行实现（fast-perspective-v1，单体 rectangle_center.py，
  Otsu 提案+窄带验证+单应），未合入现行应用，仅作参考/备份。

## 1 视觉链路（环-洞结构 + 洞四边拟线 + 对角线交点，ROI 跟踪）
- 搜索态：1/2 降采样 → 阈值 = 0.62×90 分位（夹 [45,160]）→ 连通域预筛
  （面积/边比 0.45~3.4/填充率 0.04~0.75，前 4 候选）→ 全分辨率精测。
- 精测 ring_hole：裁剪图内 Otsu + RETR_CCOMP 两层（顶层=暗块外边界、child=洞）；
  门 = 洞长边 [70,400] / 边比 [1.05,3.2] / 洞周暗带封闭性 ≥0.60（0.06×短边厚）/ 洞亮度 ≥25；
  打分 = 对比度 − 40×|边比−1.5|，全场最优。
- hole_quad：洞轮廓投影到 minAreaRect 两轴分四边 → 中段 75% 拟线（剔残差 3px×2 轮）
  → 四线交角 + 凸性/残差门（残差/短边 >0.03 拒）→ 回退 minAreaRect（填充率 ≥0.55）。
- **中心 = 洞（纸面）内框对角线交点**：透视精确解，不量圆心、不解单应；
  外框由内框沿角平分线外推 band_w，只画图不进判据。
- 跟踪：ROI=预测心外扩 1.35×+10px，降采样按 ROI 自适应（MAX_SIDE=400）；
  尺寸门 ±25%（基准=锁定尺寸，每帧最多跟随 2%）；跟踪门 0.45×框边；丢帧续命 6 帧
  （速度×0.7、外推限幅 90px）；速度限幅 2500px/s 且 dt∈[4ms,250ms] 才更新；平滑 0.55。
- 已知边界：**环必须围出洞**（门框立柱无洞被结构性排除，这是设计前提）；
  "亮面被暗边包住"（屏幕/窗框）仍可能误锁——用户明确要求不加图案判据，靠场景约束。
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
