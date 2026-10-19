# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、且不在 AGENTS.md 里的结论。契约/门禁/git 见**根 AGENTS.md**；流水见 `YYYY-MM-DD.md`；算法与上板见 `maixcam2_rect_center/README.md`。
> 精炼优先（超 3KB 会被注入截断）。

## 0 · 当前状态（2026-10-18 晚，**v1.0.0 已发布**）
**单应用仓库**：可烧录代码只有 `maixcam2_rect_center/`（`id=maixcam2_rect_center`，app 版本 **1.0.0**，
BUILD_ID `2026-10-18-relock-ackfix-v1.0.0`，**25 文件**包，门禁全绿：check_pc 24/24、check_app OK=124 FAIL=0）。

- **v1.0.0 的代码来源**：桌面 codex 快照 `MaixCAM2-TargetCenter-codex-maixcam2-spec-v0.8.0/...`
  的 `maixcam2_rect_center/`（v0.9.1，板端实测通过）**整体导入本仓库**——两套实现差异巨大
  （`rectangle_center.py` 差 ~950 行），不是补丁关系。
  ★ **2026-10-19 该快照已删除**：逐文件比对确认其 app 目录 v0.9.1 全面落后于主线 v1.0.0
  （BUILD_ID/README/.gitignore/app.yaml 均已更新），**故不回退不合并**；独有文档已抢救进
  `存档/分析文档/`、`存档/历史脚本/`、`.workbuddy/memory/2026-10-17.md`，127 个源文件/文档
  归档为 `存档/历史快照/codex-snapshot_2026-10-18_源码与文档.zip`（1.41MB，`*.zip` 需 `-f` 入库）。
  旧根 app（`target_center_25e`，BUILD_ID `2026-10-05-r8`）可从 `eff3561^` 取回，无需另存。
- **git 结构（2026-10-18 起）**：`main` = 主干；tag `v1.0.0` 仍指 `f511e23`（发布点），
  其后 `404b0e1` 为工作区整理。**版本锚点用 annotated tag `vX.Y.Z`**，不再建同名分支
  （同名会触发 `refname is ambiguous` 警告）。改代码直接提交到 `main`，发布时 commit → 记忆落档 → tag → push。
- **契约与日志都在仓库根**：`AGENTS.md`、`.workbuddy/memory/`。**不要在应用目录里建 AGENTS.md / 日志**
  （快照里那份应用内 `AGENTS.md` 属历史残留，已随删除清理）。
- **待执行迁移**（已定案未执行，勿与发版混做）：`maixcam2_rect_center/` → `maixcam2_target_aim/`
  （同步 app.yaml id、build_single/check_app 路径、全部文档引用）。**执行前确认无并发写入。**
  历史重复副本 `maixcam2_opus_rework_Acp/` 已不存在，无需再处理。

## 1 · v1.0.0 识别链路与闭环（= v0.9.0 复核 + v0.9.1 修复）
- **分层**：`rect_geometry`（纯几何）/ `contour_locator`（Canny 主级 + Otsu 兜底，2 张掩膜）/ `kalman_filter`
  / `rectangle_center`（状态机 track→roi_search→full_search）/ `app_runtime.run` 装配（`main.py` 仅 40 行）。
- **精度（PC 解析真值）**：121 相位全检出，**最大中心误差 0.0852px**、静态抖动 **0.0139px**；
  锁定态 `track` **0.809ms/帧**；18 个结构化负样本 0 危险误检（U 形/网格/棋盘/纯噪声全拒）。
- **★ 已知回归（v1.0.0 未修，下一轮 P0）**：v0.9.x 重写**丢掉了 v0.8.0 的「缺边豁免」**——
  ① `_verify` 的 `MIN_OBSERVED_EDGE_COVERAGE=0.125` 要求每边都有证据（整条边 coverage=0 即拒）；
  ② `_refine` 在 `refitted<4` 时返回 None（v0.8.0 会「某边无证据时沿用输入四边形的边」）。
  后果：`tools/real_board_frame.png`（v0.8.0 能检，0.04px）与 `tools/real_scene_corner_gap.png`（0.33px）
  被判 `no_black_frame`；`test_real_scene` 两条已标 `unittest.expectedFailure`（**修好后会报
  unexpected success**，以它为复查信号）。两条闸门都是放宽召回的守卫，改它牵动 U 形误检，须专门一轮。
- **v0.8.0 时代仍有参考价值的数**：掩膜 9~10 → ≤2；多候选择优 `rank = score + 40×最小覆盖率`；
  `KALMAN_PROCESS_PX=40`；`SEARCH_EXHAUSTIVE=True` 是用户明确要的稳定性口径，别为速度动它。

## 2 · 板端状态（2026-10-18 实测，v0.9.1）
- **已解决**：① 云台完全不动 = ACK 看门狗**两轴共用 `ok` 计数**武装，pitch 不回帧即 `FAULT: ack_timeout:pitch`
  ⇒ `_control` 首行 return（连屏幕按钮都被清掉）；改为 `_ack_seen` **按轴**武装 + 从未回帧的轴只 WARN 一次。
  ② 复锁慢 = `_refine` 用 `.all(axis=2)` 判越界，`TRACK_RADIUS=12` 时最外侧一点越界就废整条边
  ⇒ `refitted<4` ⇒ 每帧退回 roi/full_search；改为**逐探测点判越界**。实测复锁 = 1 个 full_search 帧后全 track。
- **板端数（MaixCAM2）**：检测器空载 **17.3ms/帧**（refine 9.1 / verify 5.6 / 解中心 0.4）；全应用同跑
  `det_ms=39~71ms`（主线程 draw 抢 CPU）；搜索态 0.5~1.4s，`locate`(Canny+轮廓) 占 100% ⇒ **`_verify` 不是瓶颈**。
- **未完成**：30fps 未达成（板端 track 56~115ms/帧）⇒ 下一步**削主线程 draw**（去四角 `P1(x,y)` 标签、
  降 `SHOW_FPS`/`SHOW_WIDTH`），不动检测参数；pitch 方向未实证（dy 只在 -1~-2.6px 抖动，需 >20px 试验）；
  pitch 从不回 ACK ⇒ 开驱动器 `Response=Receive` 才能确认到位。
- ⚠ **不要 kill/pkill 应用进程**：`pkill -f main.py`、`kill -INT` 数秒内导致**整机重启**（相机/ISP 未拆流），
  且清空 `/tmp/maixpy_run`。停止只能用 MaixVision 停止按钮。板子曾出现 22/80/7899 全 timeout（需断电重启）。
- 部署走 SSH（root/sipeed，192.168.2.16）比点「运行项目」更可查；`[perf]` 看 `det_valid/det_stage/det_ms`。

## 3 · 改动纪律
- 改 `vision/*`/`config.py`/`main.py`/`target_api.py`：**改 BUILD_ID → `build_single.py` 重生成 → 跑门禁**，
  顺序不能反；新增模块同步 `build_single.py` 的 `MODULES` 清单。
- ⚠ **换行**：`Write`/`pathlib.write_text` 在本机会写成 **CRLF** → `check_app C6` 直接红。
  **每次改完跑一遍全仓 CRLF 归一**（.py/.yaml/.md/.json）。
- ⚠ `main.py` 曾被外部进程整份覆盖：改完立即 `build_single --smoke` + `check_app --manifest`（C8 是覆盖探针）。
- ⚠ 有并发写入时不要重命名/删除应用目录（对方会在旧路径重新长出半个目录）。
- 一律 `"D:/Python311/python.exe"`（托管 venv 装不上 numpy/cv2）。
- ⚠ 本机中文路径：**禁用 `cv2.imread/imwrite`**（返回 None），用 `run_pc.imread_unicode/imwrite_unicode`。
- 板端日志 BUILD_ID 与本地逐字一致才算新代码。

## 4 · git
- **严禁 `git add -A` / `git add .`**（并发写盘）；按路径精确 add。
- **提交前缀**：`code:` 应用源码 / `docs:` 文档 / `tools:` 工具 / `repo:` 仓库整理（`deepseek:`/`gpt:` 已弃用）。
- 远程 `origin https://github.com/Chemcrin/MaixCAM2-TargetCenter.git`（2026-10-18 起 push 正常）。
  发布约定：commit → 记忆落档 → **annotated tag `vX.Y.Z`** → push `main` + tag。
- 工具生成的报告目录（`tools/e2e_report`、`precision_report`、`subpixel_report`）已进 `.gitignore`，不入库。

## 5 · 关键结论（防重复翻车）
- **旧 `DARK_THRESHOLDS` 等 14 个参数已删除**；新旧对拍由 `tools/test_performance.py::LegacyConfig` 补回参数集。
- **绝对耗时不可跨机器比较**：PC track ~0.8ms；板端 track 56~115ms。PC 能检出 ≠ 板端有效。
  「识别不给力」必须先取板端 `[perf]` 或原始帧。
- v0.6.5 宽半径兜底（`REFINE_RADIUS_WIDE=20`+`RECOVER_MIN_CONTRAST=80`+`RECOVER_MAX_DARK=100`）
  仍在 `_rescue` 里，但**只在候选初次验证失败时触发**，救不了"验证过了、精修却失败"的帧（见 §1 回归）。
- 轴映射（实测）：`ttyS0`(U0,ID=1)=pitch、`ttyS2`(U2,ID=2)=yaw，两轴正方向 CCW；
  `AIM_PROFILE="smooth"` 用户明确保持；ACC 语义/ZDT 协议/算子选型见根 AGENTS.md 与 10-1x 日志。
- 闭环只需要 `DIR_SIGN` 正确；`PULSE_PER_PX` 有 ±6× 容错（比例误差 0.25×~6× 都能 SETTLE）。
