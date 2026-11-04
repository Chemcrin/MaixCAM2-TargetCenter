# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-11-04）
- 唯一应用 `maixcam2_rect_center/`：BUILD_ID `2026-11-04-track-band-v1.3.0`；
  check_pc 15/15、check_app OK=113 FAIL=0。
- git 只有 `main`；tag `v1.0.0`（旧发布点）/`archive/v0.3.2`/`baseline-dual-track-2026-10-05`。
- 待办：目录改名 `maixcam2_target_aim/`（同步 app.yaml id 与工具路径）。

## 1 视觉链路（v1.3.0，当前）
- 提案：自适应反色二值化 → `RETR_CCOMP`（外轮廓 + 内孔，**纸面是内孔**）→ 面积/四点(或 minAreaRect)/对边长度比
  → 四层胶带证据（方向得分 + 至少3边成立 + 暗带厚度比 + 四边同宽 + 框内亮度）→ 择优（有时序先验取"近且大"）。
- 跨帧：锁定后 `refine_edges_1d` 沿四边法向一维搜索重建四边形（stage=track），满 `RECT_TRACK_MAX_FRAMES` 回全图；
  丢锁后位置先验只在 `RECT_PRIOR_FRAMES` 内有效。中心仍取对角线交点。
- `RECT_BLOCK_SIZE` 必须**大于胶带像素宽度**，否则胶带被"挖空"成两条细线导致四边凑不出。
  现场 165cm 处纸约 71px、胶带约 6px（比例恒定 8.4%，不随距离变）。
- PC：中位 2.2ms / p95 3.3ms（640×480）；识别回归 `tools/test_rectangle_detect.py` 全过
  （合成中心 err≤1.5px、旋转±35°/竖放、角点重复 std 0.019px、负样本 e421 + 细边框屏拒检、真实帧 err 0.5px）。
- 已知边界：纸面缺口过大整帧判负（靠跟踪跨过）；"白亮面+同宽黑边"的同类物体会抢锁。
- 实测：p014 检测中心 (251.7,220.8) vs HoughCircles 同心圆圆心 (252.5,218.5)，偏 2.4px。

## 2 板端（2026-11-04 实测 v1.2；v1.3 未上板）
- v1.2 板端 `valid_fps=0.0`、`det_ms=105.6`、`reject=full/all_filtered det_props=115`——即完全检不出。
- 上板要看 `[perf] det_stage`：应出现 `track`；长期停在 `full_search` 说明跟踪没接上（查位移门/暗带门）。
- ⚠ 禁 pkill/kill（只走 MaixVision 停止按钮）。

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
