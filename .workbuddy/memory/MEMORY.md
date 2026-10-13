# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记跨会话必须知道、且不在 AGENTS.md 里的结论。契约/门禁/git 见**根 AGENTS.md**；流水见 `YYYY-MM-DD.md`；算法与上板见 `maixcam2_rect_center/README.md`。
> 精炼优先（超 3KB 会被注入截断）。

## 0 · 当前状态（2026-10-13 晚，v0.8.0）
**单应用仓库**：可烧录代码只有 `maixcam2_rect_center/`（`id=maixcam2_rect_center`，app 版本 0.8.0，
BUILD_ID `2026-10-13-canny-kalman-v0.8.0`，**25 文件**门禁全绿）。历史上并行维护过两套实现，**已收敛为一份**，
A 轨代码已删（可 `git show eff3561` 取回）。

- **契约与日志都在仓库根**：`AGENTS.md`、`.workbuddy/memory/`。**不要再在应用目录里建 AGENTS.md / 日志**。
  （10-13 外部进程又把应用目录里的 AGENTS.md 删了一次 —— 它本来就不该在那里。）
- **待执行迁移**（已定案未执行）：① `maixcam2_rect_center/` → `maixcam2_target_aim/`（同步 app.yaml 的 id、
  build_single/check_app 路径假设、全部文档引用）；② 删重复副本 `maixcam2_opus_rework_Acp/`。
  **执行前必须确认无并发写入。**
- **git 落后**：HEAD=`v0.4.0`(=v0.7.0 锚点)，`main` 停更；`origin` 已配但 push 长期被 ISP 阻断（github 不可达），
  待网络恢复 `git push origin <新锚点> v0.4.0`。

## 1 · ★ v0.8.0 识别链路重构（本轮主交付）
- **分层**：`rect_geometry`（纯几何）/ `contour_locator`（Canny 轮廓定位）/ `kalman_filter`（中心滤波）
  / `rectangle_center`（状态机 track→roi_search→full_search）。`main.py` 未增行；对外仍是 `target_api`。
- **掩膜 9~10 张 → ≤2 张**（Canny 主级 + Otsu 兜底），阈值/形态学/轮廓**全在 320 宽低分辨率**上做，
  验证/精修/解中心回到原分辨率。契约已写成断言 `test_runtime_performance: out["masks"] <= 2`。
- **多候选择优**：`rank = score + 40×最小覆盖率`（`score` 口径不动）。实机误锁证据：板端真机帧的假候选
  （顶边落在纸面）靠"缺边豁免"通过、中心偏 **17.28px**，score 只差 3 分、rank 差 40 分。
  四边齐备即早退；靠豁免/救援通过的才扫完级别比 rank。**筛选与精修分离**（精修只给最终选中的候选，
  板端帧精修次数 14→3）。
- **卡尔曼**：`KALMAN_PROCESS_PX=40`（抑噪 0.63 倍、匀速/匀加速跟随 ≤0.8px，q 再大纯亏抑噪）；
  时钟不连续/跳变/NaN 一律重建或拒绝；**只作用 u/v**，闭环仍用 raw。
- **实测（同进程交错 A/B vs v0.7.0）**：检出 30/30、**最大中心误差 0.803→0.691px**、误检 0；
  缺角胶带首帧 0.83→**0.33px**、第二帧漂移 0.79→**0.00px**；板端真机帧 0.04px；平移 track 22→23/24；
  **稳态 track 单帧不变（~1.0ms）**；首次捕获中位 1.72→2.27ms（**+32%，诚实记录**，换来候选/掩膜硬上界）。
- **`[perf]` 新增 `det_valid/det_stage/det_ms`** —— 修 v0.7.0 排查最大盲区（stage=stale 掩盖检测器真实 stage）。
- **正视图**：`cv2.warpPerspective` 的矩阵口径实测为 **src→dst**（`mm_to_front·inv(rect_to_image)`）；
  写反会得到"整片背景"且肉眼未必发现，`test_real_scene` 第 5 项已锁死。逐帧中心仍走闭式。

## 2 · 板端待验证（v0.8.0 尚未上板）
- v0.7.0 板端实锤：靶纸清晰可见但 `detect_fps=3.2`（单帧 ~312ms）⇒ 超 `RESULT_MAX_AGE_S=0.20`
  ⇒ 结果**全判 stale** ⇒ aim 恒 HOLD；`drop(vision)=32k`；read 4.2→22.7ms、disp 29.8→11.6fps。
  根因是**每帧都在跑全图搜索**（锁不上）；v0.8.0 要验证的正是"能否锁上 + 单帧搜索耗时"。
- 异常：`[aim] dx=3620848588800993985`（整数巨值，与 SETTLE |dx|≤2px 矛盾；数值随会话变化 = 实时垃圾测量）。
  本地源码与 dist 都有 `float()` 强转 ⇒ 本地产生不出该形态 ⇒ **疑板端构建≠本地源码，改算法前先 rebuild+重烧**。
- 文档：`maixcam2_rect_center/板端识别问题整理_2026-10-13.md`（P0×3+P1×3+P2 + 修法顺序）。

## 3 · 改动纪律
- 改 `vision/*`/`config.py`/`main.py`/`target_api.py`：**改 BUILD_ID → `build_single.py` 重生成 → 跑门禁**，
  顺序不能反；新增模块同步 `build_single.py` 的 `MODULES` 清单。
- ⚠ **换行**：`Write`/`pathlib.write_text` 在本机会写成 **CRLF** → `check_app C6` 直接红。
  **每次改完跑一遍全仓 CRLF 归一**（.py/.yaml/.md/.json）。
- ⚠ `main.py` 被外部进程整份覆盖：改完立即 `build_single --smoke` + `check_app --manifest`（C8 是覆盖探针）。
- ⚠ 有并发写入时不要重命名/删除应用目录（对方会在旧路径重新长出半个目录）。
- 一律 `"D:/Python311/python.exe"`（托管 venv 装不上 numpy/cv2）。
- ⚠ 本机中文路径：**禁用 `cv2.imread/imwrite`**（返回 None），用 `run_pc.imread_unicode/imwrite_unicode`。
- 板端日志 BUILD_ID 与本地逐字一致才算新代码。

## 4 · git
- **严禁 `git add -A` / `git add .`**（并发写盘）；按路径精确 add。
- **提交前缀**：`code:` 应用源码 / `docs:` 文档 / `tools:` 工具 / `repo:` 仓库整理（`deepseek:`/`gpt:` 已弃用）。
- `git_round --check` 第 3 步长期红（v20 结构性误报）**勿修**，改手工精确 add+commit。
- 远程 `origin https://github.com/Chemcrin/MaixCAM2-TargetCenter.git`；发布约定：commit → 记忆落档 →
  `vX.Y.Z` 锚点分支 → push。

## 5 · 关键结论（防重复翻车）
- **旧 `DARK_THRESHOLDS` 等 14 个参数已在 v0.8.0 连同实现删除**；做新旧对拍由
  `tools/test_performance.py::LegacyConfig` 补回旧参数集。`SEARCH_EXHAUSTIVE=True` 是用户明确要的
  稳定性口径，别为速度动它。
- **绝对耗时不可跨机器比较**：PC track ~1ms；板端 track ~35ms（28.4fps）、旧 full_search ~312ms（3.2fps）。
- PC 固件能检出 ≠ 板端有效；定性"识别不给力"必须板端 `[perf]`/原始帧。
- v0.6.5 宽半径兜底（`REFINE_RADIUS_WIDE=20`+`RECOVER_MIN_CONTRAST=80`+`RECOVER_MAX_DARK=100`）
  保留在 `_rescue` 里，用于"候选比胶带外沿低 14px"的板端真机帧。
- 轴映射（实测）：`ttyS0`(U0,ID=1)=pitch、`ttyS2`(U2,ID=2)=yaw，两轴正方向 CCW；
  `AIM_PROFILE="smooth"` 用户明确保持；ACC 语义/ZDT 协议/算子选型见根 AGENTS.md 与 10-1x 日志。
