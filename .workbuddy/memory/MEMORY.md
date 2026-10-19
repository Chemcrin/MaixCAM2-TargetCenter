# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、且不在 AGENTS.md 里的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-10-19）
- 唯一应用 `maixcam2_rect_center/`：version 1.0.0，BUILD_ID `2026-10-18-relock-ackfix-v1.0.0`，
  25 文件包；门禁 check_pc 24/24、check_app OK=124 FAIL=0。
- git 只有 `main`；tag `v1.0.0`=f511e23（发布点，应用代码与 main 逐字一致）、`archive/v0.3.2`、
  `baseline-dual-track-2026-10-05`。改代码直接提交 main；发布 commit→记忆→tag→push。
- 已删：codex v0.9.1 快照（落后于 v1.0.0，不回退）、8 条历史分支；归档在 `存档/历史快照/`。
- 待办（未定案）：目录改名 `maixcam2_rect_center/`→`maixcam2_target_aim/`，须同步 app.yaml id 与工具路径。

## 1 v1.0.0 链路
- 分层：rect_geometry / contour_locator(Canny+Otsu) / kalman_filter / rectangle_center(状态机)
  / app_runtime 装配（main.py 仅 40 行）。
- 精度：121 相位全检出，最大误差 0.0852px；track 0.809ms/帧；18 负样本 0 误检。
- ★P0 已知回归：v0.9.x 丢了 v0.8.0「缺边豁免」。`_verify` 的 MIN_OBSERVED_EDGE_COVERAGE=0.125
  要求每边有证据；`_refine` 在 refitted<4 时返回 None。后果：real_board_frame.png /
  real_scene_corner_gap.png 被判 no_black_frame，test_real_scene 两条已标 expectedFailure
  （修好会报 unexpected success = 复查信号）。改这两处牵动 U 形负样本，须专门一轮。

## 2 板端（MaixCAM2，2026-10-18 实测）
- 检测器空载 17.3ms/帧；全应用 det_ms=39~71ms；30fps 未达成（track 56~115ms）⇒ 下一步削主线程 draw。
- 已修：ACK 看门狗改按轴武装；`_refine` 越界判定改逐点。
- ⚠ 禁止 pkill/kill 应用进程（整机重启、清 /tmp/maixpy_run），停止只用 MaixVision 按钮。
- SSH root@192.168.2.16(sipeed)；看 `[perf]` 的 det_ms。

## 3 纪律
- 改代码：改 BUILD_ID → build_single.py → check_app/check_pc；新增模块同步 MODULES。
- ⚠ 本机 Write 会写 CRLF ⇒ check_app C6 红；改完全仓 LF 归一。
- ⚠ main.py 曾被外部整份覆盖：改完立即 build_single --smoke + check_app --manifest。
- Python 一律 `D:/Python311/python.exe`；中文路径禁用 cv2.imread/imwrite（用 run_pc 的 unicode 版）。
- 板端 BUILD_ID 与本地逐字一致才算新代码。
- git：按路径精确 add，禁 `git add -A`；前缀 code:/docs:/tools:/repo:。

## 4 别踩的坑
- 绝对耗时不可跨机器比较；「识别不给力」先取板端 [perf] 或原始帧。
- 手感主参数 DEG_PER_PX_*=0.30、GAIN=0.3、ACC_TRACK=80、DEADBAND=3.0；一次只改一个。
  SEARCH_EXHAUSTIVE=True 是用户明确要的稳定性口径，别为速度动。
- 轴：ttyS0(U0,ID=1)=pitch、ttyS2(U2,ID=2)=yaw，正方向均 CCW；DIR_SIGN_*=-1 未标定，
  越追越偏就翻号。AIM_PROFILE="smooth" 用户明确保持。
