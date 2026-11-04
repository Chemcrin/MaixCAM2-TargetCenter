# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、AGENTS.md 之外的结论。契约/门禁/目录树见根 AGENTS.md；流水见 YYYY-MM-DD.md。

## 0 现状（2026-11-04 回滚后）
- 唯一应用 `maixcam2_rect_center/`：已回滚到 **v1.1.0**（内容 = `ce64ab9`），
  BUILD_ID `2026-11-04-rollback-v1.1.0`（commit `8de34e1`）；check_pc 24/24、应用目录 1.5M。
- 回退点 tag：`pre-rollback-v1.3.1-2026-11-04`（v1.3.1 全部内容）；旧发布 tag `v1.0.0`。
- git 只有 `main`；另有两个 v1.1.0 提交：`9abcd5f`(10-26，已被回滚) 与 `ce64ab9`(10-31，现行)。
- 待办：在 v1.1.0 基线上重新调参；目录改名 `maixcam2_target_aim/`（未动）。

## 1 视觉链路（v1.1.0，当前）
- 提案：`contour_locator` 320 宽低分辨率，Canny(35/120) 优先，凑不出完整四边才补 Otsu。
- 判据：四边暗带 `_verify`（覆盖率/相对亮度/对比度）→ 窄带(6px)/宽半径(20px) 亚像素直线精修
  → 对角线交点；`MAX_LINE_RESIDUAL_PX=0.8` 拒绝曲线拟合。
- 跨帧：上帧四角 + 逐角位移预测(`PREDICT_MAX_PX=60`) → 12px 窄带；**partial 锁**允许最多
  1 边出画 + 1 边先验兜底（实测 ≥3 边）；复锁按面积比 `REACQUIRE_AREA_RATIO=3.0`。
- 模块：contour_locator / rectangle_center / rect_geometry / kalman_filter（只平滑显示）。
- ⚠ 全为**绝对灰度阈值**（CANNY 35/120、DARK_CENTER_MIN=90、RECOVER_MIN_CONTRAST=80），
  换光照即失效。v1.3 改自适应二值化正是为此，此线没那层保护。
- 已知短板：板端真机拼合帧顶边无胶带证据仍拒检；录像 1022 帧 valid 266 / partial 120。
- 历史 v1.3.x（内孔 CCOMP 提案 + 四层胶带证据 + refine_edges_1d，PC 中位 2.2ms，
  但 v1.2 板端 valid_fps=0）完整留在 `pre-rollback-v1.3.1-2026-11-04`。

## 2 板端
- v1.2 实测 `valid_fps=0.0`、`det_ms=105.6`、`reject=full/all_filtered`——完全检不出。
- v1.1.0 板端数据未记录；重新上板先看 `[perf] det_stage` 是否出现 `track`。
- ⚠ 禁 pkill/kill（只走 MaixVision 停止按钮）。

## 3 纪律
- 文档口径：写架构不写实现，数值/行号/版本/日期不进文档；变更记录不进文档（git 即历史）；
  导航在 `docs/README.md`；禁止元叙述。
- 改代码：改 BUILD_ID → build_single.py → check_app/check_pc；新增模块同步 MODULES。
- ⚠ 本机 Edit/Write 写 CRLF ⇒ check_app C6 红；改完用 python 归一 LF。
- ⚠ main.py 曾被外部整份覆盖：改完立即 build_single --smoke + check_app --manifest。
- Python 一律 `D:/Python311/python.exe`；中文路径禁用 cv2.imread/imwrite。
- 板端 BUILD_ID 与本地逐字一致才算新代码。
- git：按路径精确 add，禁 `git add -A`；前缀 code:/docs:/tools:/repo:。

## 4 别踩的坑
- ⚠ v1.1.0 的 `clean_artifacts.py` 不覆盖 `precision_report/`：跑一次 verify_precision 生成 34M，
  应用目录冲到 36M，超 MaixVision 30MB 上限。上板前
  `rm -rf tools/precision_report tools/e2e_report`。v1.1.0 无 C11 体积门，check_app 不报警。
- 耗时不可跨机器比较；「识别不给力」先取板端 [perf] 或原始帧。
- 手感：PULSE_PER_PX_*=3.67（未标定占位）、GAIN=0.30、ACC_DEFAULT=80、DEADBAND=1.0；
  GAIN/GAIN_COARSE/RPM/MAX_STEP 被 `apply_aim_profile("smooth")` 覆盖，读字面值会读错。
- 轴：ttyS0(U0,ID=1)=pitch、ttyS2(U2,ID=2)=yaw，正方向均 CCW；DIR_SIGN_*=-1 未标定，
  越追越偏就翻号。AIM_PROFILE="smooth" 用户明确保持。
