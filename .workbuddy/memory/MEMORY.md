# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-10-19）
- 唯一应用 `maixcam2_rect_center/`：version 1.0.1，BUILD_ID `2026-10-19-recall-fix-v1.0.1`，
  25 文件包；门禁 check_pc 24/24、check_app OK=124 FAIL=0。
- git 只有 `main`；tag `v1.0.0`=f511e23（发布点，应用代码与 main 逐字一致）、`archive/v0.3.2`、
  `baseline-dual-track-2026-10-05`。改代码直接提交 main；发布 commit→记忆→tag→push。
- 已删：codex v0.9.1 快照（落后 v1.0.0）、8 条历史分支；归档在 `存档/历史快照/`。
- 待办：目录改名 → `maixcam2_target_aim/`（须同步 app.yaml id 与工具路径）。

## 1 v1.0.x 链路
- 分层：rect_geometry / contour_locator / kalman_filter / rectangle_center / app_runtime（main 仅 40 行）。
- 精度：121 相位全检出，最大误差 0.0852px；track 0.809ms/帧。负样本 18 例 0 误检
  （仅实心块/照片；空心框会误锁，见 10-21）。
- ★v1.0.1 已修：`_refine` 共线硬门限（24×0.55=14点）才是丢帧主因 → 删硬拒、≥5 内点重拟合；
  宽半径救援改相对暗度（0.60×内侧，≤160）。test_real_scene 6/6。

## 2 板端（2026-10-18 实测）
- 空载 17.3ms/帧；det_ms=39~71ms；30fps 未达成（track 56~115ms）⇒ 下一步削 draw。
- 上板待验：`det_stage` 应从 full_search/roi_search 变首帧后 track。
- 已修：ACK 看门狗改按轴武装；`_refine` 越界判定改逐点。
- ⚠ 禁 pkill/kill（整机重启、清 /tmp/maixpy_run），停止只用 MaixVision 按钮。
- SSH root@192.168.2.16(sipeed)；看 `[perf]` 的 det_ms。

## 3 纪律
- 改代码：改 BUILD_ID → build_single.py → check_app/check_pc；新增模块同步 MODULES。
- ⚠ 本机 Write 会写 CRLF ⇒ check_app C6 红；改完全仓 LF 归一。
- ⚠ main.py 曾被外部整份覆盖：改完立即 build_single --smoke + check_app --manifest。
- Python 一律 `D:/Python311/python.exe`；中文路径禁用 cv2.imread/imwrite（用 run_pc unicode 版）。
- 板端 BUILD_ID 与本地逐字一致才算新代码。
- git：按路径精确 add，禁 `git add -A`；前缀 code:/docs:/tools:/repo:。

## 4 别踩的坑
- 耗时不可跨机器比较；「识别不给力」先取板端 [perf] 或原始帧。
- 手感**实际生效**的是 PULSE_PER_PX_*=3.67（未标定占位）、GAIN=0.30、ACC_DEFAULT=80、
  DEADBAND=1.0；`DEG_PER_PX_*` 已从代码删除（AGENTS §3.1 已于 10-21 重写为 PULSE_PER_PX 口径）。
  GAIN/GAIN_COARSE/RPM/MAX_STEP 被 `apply_aim_profile("smooth")` 覆盖成
  0.30/0.65/60/150/80，读 config 字面值会读错（机制已写入 AGENTS §3.1）。
  SEARCH_EXHAUSTIVE=True 是用户明确要的稳定性口径，别为速度动。
- 轴：ttyS0(U0,ID=1)=pitch、ttyS2(U2,ID=2)=yaw，正方向均 CCW；DIR_SIGN_*=-1 未标定，
  越追越偏就翻号。AIM_PROFILE="smooth" 用户明确保持。
