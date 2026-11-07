# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-11-07，v1.3.0）
- `maixcam2_rect_center/` = **v1.3.0**（commit `517ba9d`，tag `v1.3.0`）：小组四层重写，
  `vision/{poll,algo,logic,output}.py` 一文件一层；BUILD_ID 在 `vision/logic.py`
  （`2026-11-07-merge-team-v1.3.0-v1`），`main.py` 只是入口桩，不再持有版本号。
- 回退点 tag `pre-merge-team-v1.3.0-2026-11-07`（= v1.1.0 全内容，commit `8de34e1`）；
  更早的 v1.3.1 线在 `pre-rollback-v1.3.1-2026-11-04`。
- 门禁 4 项：check_app --manifest + build_single --check/--smoke + test_smoke。
  旧 21 个回归用例随旧模块删除，只剩合成图冒烟，**真机覆盖为零**。

## 1 视觉链路（v1.3.0）
- 自适应反色二值化（block 31 / C 8，**不用绝对灰度阈值**）+ 三层过滤：面积（含 0.92 上限）
  → approxPolyDP 四点 → 正立外接框长宽比 1.10–1.80（前提靶纸横放）。中心取对角线交点，无跨帧状态。
- PC 实测约 1–9ms（随输入分辨率）；依赖单向：poll ← algo ← output ← logic。
- ⚠ **相对 v1.1.0 已知回退**：真机帧 `real_scene_corner_gap.png`（510×377）v1.1.0 能 partial
  锁到靶纸 (246,161)，v1.3.0 漏掉靶纸、反锁画面顶部小矩形 (297.5,65.5)；扫 eps 0.02–0.06 ×
  C 5/8/12 全档不恢复（靶纸始终凑不出候选）。合成图正常（中心误差 <2px）。
- ⚠ 误锁比漏检危险：漏检只 HOLD，误锁把云台带偏。上板前必须用真机帧跑 `run_pc.py` 复核。

## 2 板端
- v1.2 实测 `valid_fps=0.0`、`det_ms=105.6`、`reject=full/all_filtered`——完全检不出。
- v1.3.0 未上板；上板先看 `[perf] det_valid=/det_stage=/det_fail=`（本版已无 track 阶段）。
- ⚠ 禁 pkill/kill（只走 MaixVision 停止按钮）。

## 3 纪律
- 文档口径：写架构不写实现，数值/行号/版本/日期不进文档；变更记录不进文档（git 即历史）；
  导航在 `docs/README.md`；禁止元叙述。
- 改代码：改 `vision/logic.py` 的 BUILD_ID → build_single.py → check_pc；新增模块同步 MODULES。
- ⚠ 本机 Edit/Write 写 CRLF ⇒ check_app C6 红；改完用 python 归一 LF。
- ⚠ main.py 曾被外部整份覆盖：改完立即 build_single --smoke + check_app --manifest。
- Python 一律 `D:/Python311/python.exe`；中文路径禁用 cv2.imread/imwrite。
- 板端 BUILD_ID 与本地逐字一致才算新代码。
- git：按路径精确 add，禁 `git add -A`；前缀 code:/docs:/tools:/repo:。

## 4 别踩的坑
- 应用目录曾冲到 35M（tools/precision_report 单目录 34M，v1.1.0 无 C11 门）。v1.3.0 的
  check_app 有 C11 30MB 硬门；产物一律落 `../reports/`。
- 耗时不可跨机器比较；「识别不给力」先取板端 [perf] 或原始帧。
- 手感：PULSE_PER_PX_*=3.67（未标定占位）、ACC_TRACK=80、DEADBAND=1.0；
  GAIN/GAIN_COARSE/RPM/MAX_STEP 被 `apply_aim_profile("smooth")` 覆盖，读字面值会读错。
- 轴：ttyS0(U0,ID=1)=pitch、ttyS2(U2,ID=2)=yaw，正方向均 CCW；DIR_SIGN_*=-1 未标定，
  越追越偏就翻号。AIM_PROFILE="smooth" 用户明确保持。
