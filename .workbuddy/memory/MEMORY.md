# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-11-09，fast-track-v9 / app 1.5.0）
- `maixcam2_rect_center/` 四层：`vision/{poll,algo,logic,output}.py` 一文件一层；
  BUILD_ID 在 `vision/logic.py`（当前 `2026-11-10-fast-track-v9`），`main.py` 只是入口桩。
- 最新提交 `bc3210f`；上一版（Canny+内框直线拟合）在 `5e71566`，可整份回退。
- 门禁 4 项全过：check_app --manifest + build_single --check/--smoke + test_smoke（8 组 25 断言）。
- **板端已跑通**：detect_fps 38.8~42.6、det_ms 8.8~34.3、valid 持续 True。

## 1 视觉链路（降采样+阈值+连通域+弦中点，ROI 跟踪）
- 五段：1/2 降采样（INTER_AREA）→ 阈值 = 0.62×画面 90 分位（夹 [max(45,p2+15),160]）
  → `connectedComponentsWithStats` → 四道门挑环 → 框心向外扫四条线取弦中点。
- 四道门：面积 / 长短边比 1.05~2.6 / 填充率 0.03~0.70 / **四条边最弱一条暗率 ≥0.20**。
  最后一道专治"框住靶心的稀疏暗结构"（窗棂、柜门缝）：环 0.27~0.45，它们只有 0.10。
- **圆心不量、单应不解**：中心 = 弦中点；实测对真值 3.8px / 2.3px（bbox 心是 7.7 / 5.9）。
- 跟踪：ROI=上帧框×1.35，速度外推，降采样档位按 ROI 自适应（MAX_SIDE=240）；
  抓不到就外推续命 6 帧；跟踪门 0.45×框边长；速度限幅 2500px/s 且 dt≥4ms 才更新。
- 尺寸基准用**锁定时的尺寸**（每帧最多跟随 2%），否则会一路缩到纸面圆环。
- 已知边界：≥3 条可见边才能定环；"亮面被暗边包住"（屏幕/窗框）仍可能误锁——
  用户明确要求不在识别里加图案判据，靠场景约束解决。
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
