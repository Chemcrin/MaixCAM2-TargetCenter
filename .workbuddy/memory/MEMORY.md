# 项目长期记忆 —— 25Efork（MaixCAM2 靶心识别）

> 只记"跨会话必须知道、且不在 AGENTS.md 里"的结论；流水见 `YYYY-MM-DD.md`。
> 协作契约 / 目录归属 / 门禁 / git 见仓库根 **`AGENTS.md`（每轮自动注入）**。
> 本文件须精炼（超长会被注入截断）；细节放各轨 README 与专题文档。

## 0 · 当前状态

| 轨 | 目录 | BUILD_ID | 状态 |
|---|---|---|---|
| A（DeepSeek） | 根 | `2026-10-05-r7` | 性能治理 + 抗抖跟踪已完成 |
| B（名义 GPT） | `maixcam2_rect_center/` | `2026-10-13-c-track-release-v0.7.0` | B 轨历史 → C 轨 v0.6.0→v0.6.5 **已全部并入本目录**（d3fe8f9）；**v0.7.0 = C 轨代码发布版**（工作分支 `v0.4.0` + 锚点分支 `v0.7.0`） |
| **C（Opus 重写）** | `maixcam2_opus_rework_Acp/` | `2026-10-13-wide-refine-recovery-v0.6.5` | 源码已并入 B 轨目录并以 **v0.7.0** 发布；该目录**仍未入 git**（外部进程产出），仅存历史/对照 |

### C 轨（当前，10-13）
- **v0.7.0 发布（10-13，C 轨代码即为现行版）**：C 轨 v0.6.0→v0.6.5 合入 `maixcam2_rect_center/` 后仅升版本号
  （`2026-10-13-c-track-release-v0.7.0`，app.yaml 0.7.0）。门禁全绿：`check_app` OK=112/WARN=2/FAIL=0（包内 **22 文件**）、
  视觉 30/30 检出、0 误检、max 0.803px、target_api 全过、gimbal PASS(11)、main e2e PASS(4)、`build_single --check/--smoke` 过。
- **v0.6.5 宽半径兜底精修**：`_verify` 失败时先用 `REFINE_RADIUS_WIDE=20` 精修把四边拉回真边再验
  （实机掩膜候选顶边比胶带外沿低 ~14px，旧窄半径 6/12px 纠不动 ⇒ 明明可见却 `no_black_frame`）。
  救回路径加确证门槛 `RECOVER_MIN_CONTRAST=80`（真靶纸对比度 123，负样本 e880 仅 40）+ `RECOVER_MAX_DARK=100`（真胶带 dark max 78，e880 达 140）。
- **v0.6.2 根因 = 实机黑胶带环"不闭合"**（实拍右上角胶带缺失）：旧候选生成要求"恰好 4 点凸四边形 + outer 必须有亮孔" ⇒ 一个候选产不出
  （合成图闭环永远干净 4 点，故离线 29/30 全绿掩盖）；旧门槛要求四边覆盖率都 ≥`WEAK_EDGE_COVERAGE`（缺失边仅 0.21）⇒ 正确候选被拒。
  改：`_quad_from_contour()`（epsilon 扫描 + 凸包兜底，仅环形带启用，`HULL_FILL_RATIO=0.6`）；去掉"outer 亮孔"守卫；
  `_verify` 加 `STRONG_EDGE_COVERAGE=0.85`（另三边都 ≥0.85 才允许第四条缺失）；`_refine` 某边无证据时沿用输入边。
  结果：合成集 29/30→30/30、0.803px 不变、负样本 0/5；真实缺角可检出（中心偏差 0.83px）。
- **v0.6.3**：`SearchSweep._prepare` 旧行为在 `PULSES_PER_DEG=0 且 GEAR_RATIO=0` 时**直接 UNCALIBRATED 一条不发**（实机"按 SEARCH 没反应"）
  ⇒ 新增 `SEARCH_PULSES=400` 固定脉冲兜底（=8.89 脉冲/度×45°，同 `SELF_TEST_PULSES` 口径），序列改**相对脉冲增量**；
  加 IMU 按实测角度收段 + `calibration_summary()`（点一次 SEARCH 即得脉冲/度）。
- **v0.6.4**：恢复 v1–v3 口径 `SEARCH_EXHAUSTIVE=True`（默认）。**但实测该开关在默认配置下无可观测差异**——
  `SEARCH_MAX_PROPOSALS=128` 是**第 13 个死参数**：单帧候选构造上限 `≈96 < 128` ⇒ 永不生效
  （实测真实帧 1~6、负样本 3~6、纯噪声 0、满屏 15 假框也只 16）。**别再拿它当"识别变好"依据。**
- **PC（Windows）可用化**：`tools/check_pc.py`（一键全量）、`run_pc.py`（图/视频/摄像头跑同一检测器）、`diagnose_detect.py`（逐边诊断）、
  `test_real_scene.py`+`real_scene_corner_gap.png`（真实场景回归固件，**必须进库**）。结果 **PASS=18 SKIP=1 KNOWN=1 FAIL=0**。
  ⚠ 本机路径含中文，**禁用 `cv2.imread/imwrite`**（Windows 返回 None），用 `run_pc.imread_unicode/imwrite_unicode`。
  `check_app.py` C1（app.yaml `id` 与目录名不一致，整份从 B 轨拷来）留用户定。
- `AIM_PROFILE = "smooth"|"responsive"`（默认 smooth=原值；用户已明确**保持 smooth**）。
- ⚠ **`main.py` 被外部进程整份覆盖多次**：**改它必须用带 assert 的脚本**，改完立即 `--smoke`。
- ⚠ 两张实拍截图在 v0.6.2+ 已能检出、候选仅 1~6 ⇒ **不能**证明哪个版本有效；定性"识别不给力"需板端原始帧或 `[perf]` 日志。
- ⚠ **绝对耗时不可跨机器比较**（本机稳态 B~11ms、C~10ms；文档 0.86/0.929ms 只是标定时机器状态）。`test_runtime_performance` 已改相对口径。

### C 轨 SEARCH / IMU 事实（10-13 复查，重要）
- 按钮链：`Preview` 注册 `search` 按钮（需 `ENABLE_TOUCH_BUTTON`+`ENABLE_SEARCH_BUTTON`）→ 主循环 `_poll_touch`
  → `TouchButtons.poll` 按下沿命中 → `request_search()`（置 `controller.request_search`）→ **识别线程** `_control` 消费。
- `_control` 的 SEARCH 分支拒绝顺序：FAULT / 标定中 / 自检中 / IMU 偏置标定中 → 拒绝；已在搜索 → 取消；
  当前帧已检出靶纸 → 拒绝；否则 `_stop_motion` 后 `SearchSweep`。
- `SearchSweep` 序列**逐轴三段**：yaw(+P,−2P,+P) 再 pitch(+P,−2P,+P) → 端点 +P/−P/0，覆盖是 **"+" 十字不是矩形**（对角可能漏）。
  每段 `link.move` 后 `deadline=now+estimate_move_seconds+settle`；途中拿到新鲜有效帧即 `ACQUIRED` 停车；受 `SEARCH_TIMEOUT_S=60` 约束。
- ★★ **默认配置下 SEARCH 完全不调用陀螺仪**：`VisionControl` 仅在 `ENABLE_IMU_FEEDFORWARD=True` 时建 `HeadingFeedforward`，
  而该项**默认 False** ⇒ `search.heading is None` ⇒ `_activate_imu()` 立刻 `imu_note="imu_off"`，只用脉冲时长收段。
  即使打开，还需 `IMU_AXES_CALIBRATED=True` 且偏置已设（`IMU_BIAS_DPS` 或板载 `calib_gyro`），否则 `lock_reference` 失败退回脉冲口径。
  `IMU_YAW_AXIS/IMU_PITCH_AXIS` 默认值是**数学示例、非实测安装轴**。
- 转动中识别：搜索与检测同处识别线程，检测先于决策；`_target_is_fresh` 要求 `valid` 且 `0 ≤ now−captured ≤ RESULT_MAX_AGE_S(0.20s)`、
  `captured ≥ started_at` 且严格递增；检测器走 track→roi_search(一次)→full_search(全图多阈值+Canny)，三路都无 → `no_black_frame`。
  风险点：板端单帧检测 >200ms 时帧被判 stale，搜索"看不见"靶纸。

### B 轨 v0.5.2（10-12）运动预测 + 搜索单帧完成
- 根因：① `track` 只按 `TRACK_RADIUS=12`px 窄带精修，单帧位移 >12px 即掉出（实测 18px/帧 `0/10`、36px `0/5`）；
  ② v0.5.1 的 `SEARCH_BUDGET_MS=25ms` 软预算把一次搜索拆到十几帧 ⇒ "重捕获很慢"。
- 改：① `_motion_delta()` 用上两帧四角逐角位移外推本帧四角，在预测位置窄带精修（`ENABLE_MOTION_PREDICT=True`、`PREDICT_MAX_PX=60`；
  失锁 `_lost>0` 禁用）；② `SEARCH_SPLIT_FRAMES=False` ⇒ 单帧跑完整搜索，重捕获恒 1 帧。验证：36px `0/5→4/5`、18px `0/10→9/10`。
- ⚠ **别靠加大 `TRACK_RADIUS` 治快移**：半径 80 时连 6px/帧都开始掉（大步长误锁印刷线/另一侧黑边）。
- v0.5.1：v0.4.3(`26eea20`) 把 `_search` 改"合作式"（每帧轮换 1 阈值/验 1 候选）⇒ 重捕获 1→11 帧；改回单帧穷尽 → 11→1。
- v0.4.6 模块化：`main.py` 923→296 行，实现下沉 `vision/{rate_meter,camera_source,vision_worker,preview,aim_control,telemetry}.py`；
  ★ **时钟注入**（构造可传 `clock`）；旧私有名以别名保留 ⇒ 两个入口测试零改动通过。默认 `SHOW_ASYNC=True`；⚠ `Display()` 仍**不可**放子进程（v16 坑）。
- 板上恒 ~2fps 根因 = **MaixCDK 图传未就绪时 `send_image()` 忙等 500ms 空转 CPU**（show 挤 505~509ms）。
  治本 `DISABLE_MAIXVISION_STREAM=True`（B 轨默认）⇒ 建 Display 前置 `MAIXVISION=0` ⇒ show 1~5ms（代价：MaixVision 预览无图）。
  ⚠ 证伪：降 `SHOW_FPS` 无效；「关预览丢帧」也无效；`touchscreen.read()` 非阻塞。

## 1 · 项目一句话
2025 电赛 E 题视觉模块：靶心 = A4 靶纸四周 1.8cm 黑胶带矩形**对角线交点**（射影不变量），输出 `(u,v)`（A 轨另给 `px_per_cm`）。
光斑口径（10-07 定死）：`LASER_MODE="fixed"` 全程不检测，每帧仅"黑框中心−常量"两次减法；实测光斑 ≈ (294,202)。
旧瞄准代码备份在 `电赛/25Efork_rollback_backup_2026-10-07_2100/`。

## 2 · 改动纪律
- 改 `vision/*`/`config.py`/`main.py`/`target_api.py` 后：**改 BUILD_ID → `build_single.py` 重生成 → 跑门禁**，顺序不能反。
  新增/改名模块必须同步 `build_single.py` 的 `MODULES` 清单（单文件版按序 exec，被依赖者须在前）。
- `tools/*.jpg`、`screenshot_*.png` 是调试产物：别当源文件改，别进包。

## 3 · 双轨并发 / git
严禁 `git add -A`（并行目录随时被外部进程写盘），**按路径精确 add**，前缀 `deepseek:`/`gpt:`/`repo:`/`docs:`。
- ⚠ **B/C 轨 main.py 被外部进程多次整份覆盖**：改完入口立刻 `build_single.py`+`check_app.py --manifest`；**C8 是覆盖探针**。
- **B 轨每轮交 git**：一条命令 `"D:/Python311/python.exe" -B tools/git_round.py -m "gpt: 摘要"`。
  ⚠ `git_round --check` 第 3 步（`apply_gimbal_track_patch --check`）**长期红**（v20 血统、锚点结构性误报）→ **勿修、勿改其 `LATEST_BUILD`**；
  此红存在时改**手工精确 add+commit**（绝不 `-A`）。
- **B 轨另有 `maixcam2_rect_center/AGENTS.md`**（边界/轴映射/调参/每轮交 git）——**改 B 轨先读它**；根 `AGENTS.md` 是跨轨契约。
- **远程已配置**（旧"尚无远程仓库"结论已过期）：`origin https://github.com/Chemcrin/MaixCAM2-TargetCenter.git`。
  版本发布约定（沿 v0.5.0/v0.5.1/v0.5.2 先例）：代码 commit（`gpt:`）→ 记忆落档（`repo:`）→ 建 `vX.Y.Z` 分支锚点 → push。

## 4 · 门禁
`"D:/Python311/python.exe" -B tools/check_app.py --manifest`（C1..C10 + 包内清单）。B 轨包内 **22 文件**（C 轨合入后，dist 已排除）。
改源码必更新 BUILD_ID，板端日志与本地逐字一致才算新代码。**托管 venv 装不上 numpy/cv2 —— 一律用 `"D:/Python311/python.exe"`。**

## 5 · A 轨性能（README §三′）
症状入口 `[fps]` 的 `掩膜=N`（稳态个位、锁定=1；长期 12/24=每帧全扫）。提速只动性能组；动 `MIN_AREA_RATIO`/`SCORE_*`/`MORPH_*` 必跑
`test_hardscene --ab`；性能结论必须同进程交错 A/B。`findContours` 代价∝轮廓数 → `boundary_estimate()` 预筛；`_masks` 必须生成器。

## 6 · A 轨抗抖（README §三″）
症状入口 `保持=Y/N`（长期 Y 查**检测**不是滤波器）。抖动位移是**必须跟踪的真实运动**；重低通"滤掉"= 照旧位置打（已证伪）。
改 `TRACK_*`/`KIND_BONUS` 必跑 `test_jitter.py --unit` 看**四项指标**；`TRACK_JUMP_*` 别往小调。

## 7 · B 轨要点（详见 B 轨 README）
- **12 个死参数**（改了无效）：`SEARCH_BUDGET_MS` `SEARCH_MAX_PROPOSALS` `SEARCH_GEOMETRY_STEPS` `MAX_CONTOURS` `LINE_MAX_PROPOSALS`
  `LINE_SEARCH_INTERVAL` `ACQUIRE_CONFIRM_FRAMES` `CONFIRM_CORNER_PX` `RECHECK_INTERVAL` `MAX_LINE_RESIDUAL_PX` `MIN_TRANSITION_COVERAGE` `MAX_BOUNDARY_WIDTH_ERROR`。
- 基线：30 场景检出 29（fixture#2 强反光属安全失败）、max 0.803px、0 误检、稳态 ~1ms。
- ★ **视觉算子选型定论（已固化，别再调研）—— 不换算子**：换纯 Canny = 等价搬迁（检测算子对中心贡献同量级 0.83 vs 0.75px，
  真正到 0.061px(13.6×) 的是 `fitLine` 亚像素精修）；照搬国一前端 = 更差（11/30；面积界硬编码尺度 + sum/diff 极值法 45° 退化）；
  「逆透视求中心」成立且 B 轨已在用（warp 中心 ≡ `project_points(mm→px,(w/2,h/2))` = 对角线交点，倾角 0–78° 误差恒 0.000px）。
  · 两条防弯路：① **追踪不走 Canny/二值化**（走 `_refine` 窄带灰度梯度+fitLine）；② A 轨"全分辨率多 3~4MB"顾虑在 B 轨不成立（Canny 跑 320 宽）。
  · ★ 铁律：走边缘主导候选时**四角排序必须用极角法**，禁用 sum/diff。
  · 工具 `tools/compare_guoyi_scheme.py`；详见根目录 `A轨黑框方案对比与Canny可行性.html`、`国一视觉方案可行性验证.html`。
- 参数有效性：`SEARCH_BUDGET_MS` 仅 `SEARCH_SPLIT_FRAMES=True` 生效；`SEARCH_MAX_PROPOSALS` 现为单帧候选硬上限 128；新增
  `ENABLE_MOTION_PREDICT`/`PREDICT_MAX_PX`/`SEARCH_SPLIT_FRAMES`。
- 对外 API `target_api.py`：`detect_black_frame`/`CenterTracker`/`AimLoop`，返回 `Result`（dict 子类，`bool(res)==res.valid`）。

## 8 · B 轨版本演进速查
- **v3 基线（`main`，`b32cd08`）**：⚠ 轴映射/方向是**旧值**、`ACC_DEFAULT=0`、无自检优先/LENS_CORR —— **上板前必须重核标定**。
  git 里**不存在 v9 快照**（`track-curve-v9` 是外部覆写的未入库中间态）；安全分支 `backup/b-track-v20-pre-v9-rollback`（`2a08703`）。
- **v0.4.0**（`a06f81d`）修轴映射 `U1_*/U2_*`→`YAW_*/PITCH_*`、`DIR_SIGN=-1/-1`、`ACC_DEFAULT 0→80`；
  **v0.4.1**（`6093827`）取消上电自动自检 → 屏幕 `SELF-TEST` 按钮（固定 `SELF_TEST_PULSES=400`）。
- **v0.7.0**（`2026-10-13-c-track-release-v0.7.0`）：C 轨（Opus 重写）v0.6.0→v0.6.5 合入 B 轨目录后的发布版，即**现行最新版**；
  旧 B 轨 ≤v0.5.2 在 `v0.5.2` 等分支锚点可回退；工作分支 `v0.4.0`（自 d3fe8f9 起含 C 轨合并）。
- **历史教训（别重踩）**：v16 `Display()` 进 spawn 子进程 → 主循环一帧不画（卡 `SAMPLE_VO_StartChn` ⇒ 设备 VO/fb0 卡死，**重启设备**）；
  v20 屏幕 `FPS` 曾画主循环速率（虚高）；**v9 更跟手**（GAIN 0.5/COARSE 0.9、RPM 120·400、MAX_STEP 360），v12+ 逐版放缓 —— **"跟手↔柔和"是跷跷板不是 bug**，改回需用户确认。
- **v11 定案**：上电顺序 = 人工标定 → 自检 → auto_cal → 自动标定 → 追踪解锁；自检只用链路+时间（无黑框照走），但 `aim.state=="FAULT"` 时**不发任何指令**。
  改主循环控制流**必跑** `tools/test_main_self_test.py`。
- **v9 定案**：闭环发帧带 ZDT 曲线加减速 `ACC_TRACK=200`；`main.py` 发送点必须用 `Config.ACC_TRACK`（静态测试锁死，亦是覆盖探针）。

## 9 · 云台/ZDT 速查
- **实测轴映射（v8 定案，已落码）**：`ttyS0`(U0,ID=1)=**pitch**、`ttyS2`(U2,ID=2)=**yaw**；两轴正方向都是 CCW（yaw=左、pitch=上）。
- 引脚：U0(A28/A29) **必须显式 pinmap**（空表静默不通）、是系统日志口（`FLUSH_RX_ON_START` 清 RX）；U2(B0/B1) 需 pinmap。
- ZDT：Checksum=0x6B、多字节大端；F3 使能/F6 速度/FD 位置(mode0 相对 mode1 绝对)/FE 停止；dir 0x00=CW 0x01=CCW；16 细分 3200 脉冲=1 圈；返回 02 成功/E2 堵转。
- **acc 语义**：`acc=0` = 不曲线、直接以设定速度启停（卡顿根源）；≠0 每 `(256-acc)*50us` 变 1RPM，**越小越柔**（常用 20-80）。
- 驱动器菜单必设：`P_Serial=UART_FUN`（出厂 RxTx_OFF！）、`UartBaud=115200`、`ID_Addr`、`Checksum=0x6B`、`Response=Receive`、`MotType=1.8°`、`MStep=16`、`P_Pul=PUL_OFF`。
- `_ingest()` 无故障返回 `fault=False`：判断必须走 `stepper_link.fault_reason()`（None=无故障）。`dx=u靶心−u光斑`，为正发 CCW；反了改 `DIR_SIGN_*`。
- 五条回头路：① 闭环=定点跟踪；② `PULSE_PER_PX` 必须实测；③ 相对位置+增量比例律+单帧限幅；④ 死区 ≥1.5×回差（齿轮回差 1°≈9.7px）；⑤ 闭环用 raw_u/raw_v。
- 未决：FAULT 需手动重启；`ENABLE_SCAN` 默认关；光斑极性（fixed 模式已绕过）。

## 10 · 下一步（板端）
① 看开机 `[preview] show 探针 ?ms`（≈505ms 则改**从设备本地启动**）；② 验证屏幕按钮（SELF-TEST/SEARCH）命中；③ 若"识别不给力"仍需板端原始帧或 `[perf]` 日志（**不要**用现有两张截图当证据）。
