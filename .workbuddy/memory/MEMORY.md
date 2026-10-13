# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、且不在 AGENTS.md 里的结论。契约/门禁/git 见**根 AGENTS.md**；流水见 `YYYY-MM-DD.md`；算法与上板见 `maixcam2_rect_center/README.md`。
> 精炼优先（超 3KB 会被注入截断）。

## 0 · 当前状态（2026-10-13 晚）
**单应用仓库**：可烧录代码只有 `maixcam2_rect_center/`（`id=maixcam2_rect_center`，v0.7.0，
BUILD_ID `2026-10-13-c-track-release-v0.7.0`，22 文件门禁全绿）。历史上并行维护过两套实现，
**已收敛为一份**，A 轨代码已删（`存档/A轨（已停用）/` 10-13 亦清除，可 `git show eff3561` 取回）。

- **契约与日志都在仓库根**：`AGENTS.md`（10-13 从应用目录上提并合并，应用目录内不再放）、
  `.workbuddy/memory/`。不要再在应用目录里建 AGENTS.md / 日志。
- **待执行迁移**（已定案未执行）：① `maixcam2_rect_center/` → `maixcam2_target_aim/`（需同步
  app.yaml 的 id、build_single/check_app 路径假设、全部文档引用）；② 删重复副本
  `maixcam2_opus_rework_Acp/`（未跟踪，源码与应用目录逐字节相同）。**执行前必须确认无并发写入。**
- **git 落后**：HEAD=`v0.4.0`(=v0.7.0 锚点 3e4f9c4)，`main` 停在 7b36b32；`origin` 已配但
  `git push origin v0.7.0 v0.4.0` 因 ISP 阻断 github 一直失败，待网络恢复。

## 1 · ★ 板端现状实锤（10-13 晚，用户截图 + [perf] 日志）
- 靶纸画面里清晰可见，但 `detect_fps=3.2`（单帧 ~312ms；同板 track 健康态 28.4fps）⇒ 超
  `RESULT_MAX_AGE_S=0.20s` ⇒ 结果**全判 stale** ⇒ aim 恒 HOLD、云台不动；`drop(vision)=32k`；
  read 4.2→22.7ms、disp 29.8→11.6fps。`test_real_scene.py:43` 预警的"板端 >200ms 全 stale"**已实发**。
- 异常：`[aim] dx=3620848588800993985`（整数巨值）与 SETTLE |dx|≤2px 矛盾；本地源码与 dist 均有
  `float()` 强转，`%s` 打印 float 必带小数点 ⇒ 本地产生不出该形态 ⇒ **疑板端构建≠本地源码**。
  **改算法前必须先 rebuild + 重烧验证。**
- 诊断盲区：`[perf]` 的 stage 被 `display_result` 整体替换为 "stale"，掩盖检测器真实 stage；
  建议补 `det_stage/det_valid/det_ms`（同步改 test_main_self_test 字段冻结用例）。
- 文档：`maixcam2_rect_center/板端识别问题整理_2026-10-13.md`（P0×3+P1×3+P2 + 修法顺序）。

## 2 · 改动纪律
- 改 `vision/*`/`config.py`/`main.py`/`target_api.py`：**改 BUILD_ID → `build_single.py` 重生成 →
  跑门禁**，顺序不能反；新增模块同步 `build_single.py` 的 `MODULES` 清单。
- 板端日志 BUILD_ID 与本地逐字一致才算新代码。
- 一律 `"D:/Python311/python.exe"`（托管 venv 装不上 numpy/cv2）。
- ⚠ `main.py` 被外部进程整份覆盖：改完立即 `build_single --smoke` + `check_app --manifest`（C8 是覆盖探针）。
- ⚠ **有并发写入时不要重命名/删除应用目录**：对方相对路径写入会在旧路径重新长出半个目录（10-13 处理过）。
- ⚠ 本机中文路径：**禁用 `cv2.imread/imwrite`**（返回 None），用 `run_pc.imread_unicode/imwrite_unicode`。
- ⚠ 本机截屏不可用（GDI/PrintWindow 对全屏返回纯黑）⇒ 观察本机屏幕只能靠用户提供截图。

## 3 · git
- **严禁 `git add -A` / `git add .`**（并发写盘）；按路径精确 add。
- **提交前缀（10-13 起）**：`code:` 应用源码 / `docs:` 文档 / `tools:` 工具 / `repo:` 仓库整理；
  `deepseek:` `gpt:` 为历史遗留前缀，不再使用。
- `git_round --check` 第 3 步长期红（`apply_gimbal_track_patch.py` v20 结构性误报）**勿修**，改手工精确 add+commit。
- 远程 `origin https://github.com/Chemcrin/MaixCAM2-TargetCenter.git`；发布约定：commit → 记忆落档 → `vX.Y.Z` 锚点分支 → push。

## 4 · 关键结论（防重复翻车）
- **12 个死参数**（改了无效）：`SEARCH_BUDGET_MS` `SEARCH_MAX_PROPOSALS` `SEARCH_GEOMETRY_STEPS`
  `MAX_CONTOURS` `LINE_MAX_PROPOSALS` `LINE_SEARCH_INTERVAL` `ACQUIRE_CONFIRM_FRAMES` `CONFIRM_CORNER_PX`
  `RECHECK_INTERVAL` `MAX_LINE_RESIDUAL_PX` `MIN_TRANSITION_COVERAGE` `MAX_BOUNDARY_WIDTH_ERROR`。
  `SEARCH_EXHAUSTIVE=True` 是用户明确要的稳定性口径，别为速度动它。
- **绝对耗时不可跨机器比较**：PC C~10ms；板端 track~35ms（28.4fps）、full_search~312ms（3.2fps）。
- PC 两张实拍图 v0.6.2+ 能检出 ≠ 板端有效；定性"识别不给力"必须板端 `[perf]`/原始帧。
- v0.6.5 宽半径兜底（`REFINE_RADIUS_WIDE=20`+`RECOVER_MIN_CONTRAST=80`+`RECOVER_MAX_DARK=100`）
  修"候选比胶带外沿低 14px 被误拒"，PC 固件 `real_board_frame.png` 通过；板端仍 3.2fps ⇒ 先查构建再改算法。
- 轴映射（实测）：`ttyS0`(U0,ID=1)=pitch、`ttyS2`(U2,ID=2)=yaw，两轴正方向 CCW；
  `AIM_PROFILE="smooth"` 用户明确保持；ACC 语义/ZDT 协议/死参数全表/算子选型定论见根 AGENTS.md 与 10-1x 日志。
