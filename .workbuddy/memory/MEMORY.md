# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-10-26）
- 唯一应用 `maixcam2_rect_center/`：version 1.1.0，BUILD_ID `2026-10-26-simple-darkframe-v1.1.0`；
  25 文件包，check_pc 24/24、check_app OK=124 FAIL=0。
- git 只有 `main`；tag `v1.0.0`（旧发布点）/`archive/v0.3.2`/`baseline-dual-track-2026-10-05`。
- 待办：目录改名 `maixcam2_target_aim/`（同步 app.yaml id 与工具路径）。

## 1 v1.1 视觉链路（识别简化）
- 搜索：Canny+Otsu 两层提案 → A4 长宽比/面积预筛 → outer/inner × 6/20px 直线精修 → 四边胶带软证据打分；
  高置信即停，无 verify/rescue 级联、无 front view、无运动预测。中心 = 四边形对角线交点。
- 硬门槛：≥3 边暗带、弱边 0<contrast<10、暗/亮≤0.72、暗带厚度/18mm≥0.62、梯度支持≥0.45；
  暗背景 `_tape_ratio=None` 时用 SEARCH_MIN_SCORE=220 兜底。
- PC：全图搜索 median 4.2ms（真实 board 7ms）、track 1.26ms；performance 30/30 max .14px、
  precision 76/76 max .76px、subpixel 121/121 max .064px、e2e 16/16；实心块/窗格/椭圆/照片拒检。

## 2 板端（2026-10-18 实测；v1.1 未上板）
- 旧版 det_ms=39~71ms、track 56~115ms，30fps 未达成；v1.1 需看 `[perf] det_ms/det_stage`。
- `det_stage` 应在首帧后保持 track；⚠ 禁 pkill/kill（只走 MaixVision 停止按钮）。

## 3 纪律
- 文档口径：写架构不写实现，数值/行号/版本/日期不进文档；变更记录不进文档（git 即历史）；
  导航在 `docs/README.md`；禁止元叙述。
- 改代码：改 BUILD_ID → build_single.py → check_app/check_pc；新增模块同步 MODULES。
- ⚠ 本机 Write 会写 CRLF ⇒ check_app C6 红；改完全仓 LF 归一。
- ⚠ main.py 曾被外部整份覆盖：改完立即 build_single --smoke + check_app --manifest。
- Python 一律 `D:/Python311/python.exe`；中文路径禁用 cv2.imread/imwrite。
- 板端 BUILD_ID 与本地逐字一致才算新代码。
- git：按路径精确 add，禁 `git add -A`；前缀 code:/docs:/tools:/repo:。

## 4 别踩的坑
- 耗时不可跨机器比较；「识别不给力」先取板端 [perf] 或原始帧。
- 手感实际生效：PULSE_PER_PX_*=3.67（未标定占位）、GAIN=0.30、ACC_DEFAULT=80、DEADBAND=1.0；
  GAIN/GAIN_COARSE/RPM/MAX_STEP 被 `apply_aim_profile("smooth")` 覆盖，读 config 字面值会读错。
  `DEG_PER_PX_*` 已删除，调它无效。
- 轴：ttyS0(U0,ID=1)=pitch、ttyS2(U2,ID=2)=yaw，正方向均 CCW；DIR_SIGN_*=-1 未标定，
  越追越偏就翻号。AIM_PROFILE="smooth" 用户明确保持。
