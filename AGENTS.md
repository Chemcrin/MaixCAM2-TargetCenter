# AGENTS.md · 25Efork 协作与工程契约

> **本仓库唯一的协作契约**，面向 AI 编码代理与开发者协作者。**动手改代码前先读完本文件。**
>
> 分工：**本文件管"协作与流程"**（边界 / 门禁 / git / DoD）；**[`maixcam2_rect_center/README.md`](maixcam2_rect_center/README.md)
> 管"算法与上板"**（应用目录内唯一文档）。板端排查类现行文档在 `存档/项目自检/`：
> [硬件标定与上板检查](存档/项目自检/硬件标定与上板检查.md) ·
> [板端识别问题整理](存档/项目自检/板端识别问题整理_2026-10-13.md) ·
> [板端故障排查](存档/项目自检/板端故障排查_2026-10-18.md)。
> 冲突时按上述分工各管各的；改一处请顺手同步另一处。
>
> ⚠ **章节编号已冻结**：源码注释引用了本文件的 §2 / §3.2 / §5 / §6 等小节号，
> **只能改内容，不能改编号**。

---

## 0. 项目一句话

2025 全国大学生电子设计竞赛本科组 **E 题《简易自行瞄准装置》** 的视觉模块：
由 A4 靶纸四周 1.8cm 黑胶带矩形的两条对角线交点，反算**靶心像素坐标 `(u, v)`**，串口交给云台主控。
运行平台 **MaixCAM2**（MaixPy v4，屏 640×480）。

> 靶心是红色记号笔点，165cm 处只有约 0.67px，物理不可见 → **只能由黑框矩形几何反算**。
> 这是全部识别设计的出发点。

工程形态：MaixPy 应用（`app.yaml` + `main.py` + `config.py` + `vision/`），
另生成单文件版 `dist/main_single.py` 供 MaixVision「运行当前文件」。

---

## 0′. 仓库现状（单应用）

```
25Efork/
├── AGENTS.md                   ★ 本文件（唯一契约，仓库根）
├── .workbuddy/memory/          日志与长期记忆（★ 放仓库根，不放应用目录）
│   ├── YYYY-MM-DD.md           逐日流水（append-only）
│   └── MEMORY.md               跨会话结论（≤3 KB，超了注入会被截断）
├── maixcam2_rect_center/       ★ 唯一可烧录应用（id 与目录名一致，version 1.0.1）
│   ├── main.py / config.py / target_api.py / vision/*.py（20 模块）
│   ├── dist/main_single.py     单文件版（自动生成）
│   ├── tools/                  门禁 6 / 回归 21 / 验证 4 / 一次性诊断 5（不进应用包）
│   └── README.md               算法与上板说明（★ 应用目录内唯一文档）
└── 存档/                       非烧录资料（只读）：项目自检 / 分析文档 / 提示词 /
                                历史脚本 / 历史快照 / reference(upstream 2025E)
```

- **只有一个应用**。历史实现一律进 `存档/` 或 git 历史（`git log` / `git show` 可取回）。
- **文档归位**：契约 → 本文件；日志 → `.workbuddy/memory/`；算法/上板 → 应用目录 `README.md`；
  板端/标定现行 → `存档/项目自检/`；其余归 `存档/` 对应子类。**应用目录除 README 外不放任何 `*.md`**。
- **提交前缀**按改动性质：`code:`（应用源码）/ `docs:` / `tools:` / `repo:`。
  `deepseek:` / `gpt:` 是历史遗留，不再使用。
- **不要**在 `maixcam2_rect_center/` 内新建 `AGENTS.md` / 日志文件。

---

## 1. ★ 工作边界（最容易出事，先看这条）

一句话：仓库里只有一块"能烧录的代码"，其余都是资料——动代码只动应用目录，动文档先想清楚它属于哪一层。

1. 只改 `maixcam2_rect_center/` 内的代码；不要往仓库根放 `main.py` / `config.py` / `vision/`。
2. 文档归位，口径见 §0′。
3. `dist/main_single.py` 是生成物，**不要手改**——用 `tools/build_single.py` 重生成。
4. `main.py` 是唯一反复被并发覆写的文件（见 §5.1）：改完**立刻** `build_single.py` + 门禁 + 提交。
5. `存档/` 只读：不 import、不打包、不顺手修。

---

## 2. ★ 电机顺序与轴映射（2026-10-09 实机定案，别凭直觉改）

背景：最初按丝印以为 "U0=yaw、U2=pitch"，实机发现**正好相反**。轴对不上，闭环参数再对也必发散——代码里叫 "yaw" 的必须真的驱动 yaw 轴。本目录走"代码侧互换"这条路（U0 腾给 pitch，代价是 U0 为系统日志口、pitch 会混内核日志噪声）。

| 代码里的轴 | 串口 | 丝印 | 引脚 | 设备路径 | 驱动器 ID | pinmap |
|---|---|---|---|---|---|---|
| **pitch** | UART0 | `U0T`/`U0R` | A28 / A29 | `/dev/ttyS0` | **1** | ★ **必须显式 pinmap**（空表静默不通；⚠ ttyS0 也是系统日志口） |
| **yaw** | UART2 | `U2T`/`U2R` | B0 / B1 | `/dev/ttyS2` | **2** | 需 pinmap 切到 `UART2` 功能 |

`config.py` 对应项：`PITCH_DEV`/`PITCH_PINS`、`YAW_DEV`/`YAW_PINS`、`ADDR_PITCH=1`/`ADDR_YAW=2`
（**地址跟着电机走，不跟端口**）、`PINMAP_STRICT=False`（pinmap 失败只告警，避免把真故障藏成"打不开"）、
`FLUSH_RX_ON_START=True`（U0 混有内核日志，开机清 RX）。

**方向**：两轴正方向均 CCW（`yaw CCW=左`，`pitch CCW=上`），即 `YAW/PITCH_POSITIVE_CCW = True`；
自检序列 = pitch 上→下、yaw 左→右。
**闭环符号**：`DIR_SIGN_YAW = DIR_SIGN_PITCH = -1`，按几何推断、**未经标定**——
上电追踪若越追越偏（误差持续变大、云台一路跑向一侧），把对应项改 `+1`；
`DIVERGE_GUARD` 会兜底停机，不会撞到底。

### 驱动器菜单必设（不设则串口完全无效）

`P_Serial = UART_FUN`（**出厂是 `RxTx_OFF`**）、`UartBaud = 115200`、`ID_Addr`（见上表）、
`Checksum = 0x6B`、`Response = Receive`、`MotType = 1.8°`、`MStep = 16`、`P_Pul = PUL_OFF`。
未做过 `Cal` 的板子只有 6 项菜单——先空载 `Cal` 一次解锁完整菜单。

---

## 3. 调参（现场只改 `config.py` 一个文件）

人话：追踪手感 = "画面偏差 1px，云台修正多少脉冲"。所有参数在 `config.py`，但注意——
**文件加载时会执行 `apply_aim_profile(AIM_PROFILE)`，强制覆盖其中 6 个值**，直接改字面值可能不生效。

### 3.1 追踪手感

换算链：**脉冲 = |误差px| × `PULSE_PER_PX_轴` × `GAIN_轴`**（再受 `MAX_STEP_PULSES` 限幅）。

> ⚠ **profile 覆盖机制**（`config.py` 末尾 `apply_aim_profile`，模块加载即执行）：
> 当前 `AIM_PROFILE="smooth"` 会把 `GAIN_YAW/PITCH=0.30`、`GAIN_COARSE=0.65`、
> `RPM_FINE=60`、`RPM_COARSE=150`、`MAX_STEP_PULSES=80` **写死覆盖**，config 字面值不作数。
> 想改这 6 个：换 profile（`"responsive"`）或改 profiles 表。其余参数以字面值为准。

| 参数 | 生效值 | 说明 |
|---|---|---|
| `PULSE_PER_PX_YAW/_PITCH` | 3.67（**未标定占位**） | ★ 主参数：1px 偏差发几个脉冲。越大越快越易超调；未经实机标定，偏大可能振荡 |
| `GAIN_*` / `RPM_*` / `MAX_STEP_PULSES` | profile 覆盖 | 单帧修正比例（必须 ≤1）/ 转速 / 单帧限幅 |
| `DEADBAND_PX` | 1.0 | 死区，应 ≥1.5× 实测回差，否则在齿隙里来回抖 |
| `COARSE_PX` | 24.0 | \|e\| 超过它切换粗调增益 `GAIN_COARSE` |
| `ACC_DEFAULT` | 80 | 曲线加减速档位；**越小越柔；0 = 瞬时启停（勿用）** |
| `COMMAND_INTERVAL_S` | 0.08 | 发帧间隔；太小会指令堆叠"冲过头" |

> 改手感**一次只动一个数**，跑一轮看效果再动下一个——一次改多个分不清谁起作用。
> 标定现状：`CALIBRATE_AXIS=""`（不做自动标定），直接用 `PULSE_PER_PX` 字面值；
> 要重标就临时改 `"auto"`，标完置回。

### 3.2 上电时序与自检

现行行为（旧版"上电自动跑自检"**已废弃**，别按旧描述理解）：

- 上电 = 使能 → 建显示/触摸 → 进主循环（识别 + 闭环）；**上电不发脉冲**。
- 自检 = 屏幕右上角 `SELF-TEST` **按钮**点击触发，按固定脉冲走 4 步
  （`SELF_TEST_PULSES=400`，不依赖标定；置 0 才回退按角度换算）。
  自检与视觉彻底解耦，调度在 `vision/aim_control.py`。
- 视觉标定 3s 收不到有效 `dx/dy` 立即失败并交还主流程（`CALIBRATE_VISUAL_TIMEOUT_S`），不会干等。

### 3.3 光斑读数

`LASER_MODE="fixed"`：全程不检测，每帧只算"黑框中心 − 常量"（`LASER_U/V_NOMINAL`）。
重装激光笔/换镜头后重标：临时改 `"detect"`，把屏幕左上角读数填回这两个常量，再改回 `"fixed"`。

---

## 4. 改完必须做的事

人话：让"板子跑的"和"仓库里的"能对上号，并让门禁证明没改坏。命令一律在 `maixcam2_rect_center/` 下、用 `"D:/Python311/python.exe"` 执行（为什么见 §4.1）。

1. **提升 `main.py` 顶部 `BUILD_ID`**（格式 `YYYY-MM-DD-<slug>-vN`，当前 `2026-10-19-recall-fix-v1.0.1`）。
   板端打印 `[rect-center] <BUILD_ID>`，**与本地逐字一致才算跑的新代码**。
   改了 `main.py` / `config.py` / `vision/*` / `target_api.py` 任一个都要做。
2. **重生成单文件版**：`tools/build_single.py`。
   ⚠ 新增/改名 `vision/*.py` 或根级模块后，必须同步脚本内 `MODULES` 清单。
3. **跑门禁（一条命令全包，共 24 项）**：

   ```bash
   "D:/Python311/python.exe" -B tools/check_pc.py   # build --check/--smoke + check_app --manifest(应 25 文件/0 FAIL) + 21 项测试
   ```

   - `--quick` 跳过 2 个慢用例；改**视觉阈值**（`MIN_*`/`MAX_*`/`SCORE_*`）、**主循环控制流**、
     **显示/搜索预算**时建议跑全量。
   - Linux 专属用例标 SKIP、打包身份问题标 KNOWN；**退出码 1 才是真失败**。
   - 单独复跑某项：`tools/test_*.py` 均可直接跑。针对性建议：阈值 → `test_performance` + `test_real_scene`；
     主循环 → `test_main_self_test`。

### 4.1 PC（Windows）上怎么跑

板端专属的东西（MaixPy SDK / 显示 / 串口）PC 上跑不了，其余全部可在 PC 复现——调参与复现问题先走 PC：

```bash
"D:/Python311/python.exe" -B tools/run_pc.py --images 图/*.png   # 图片/视频/摄像头跑真实检测器
"D:/Python311/python.exe" -B tools/diagnose_detect.py <图片>     # 逐边打印"为什么没检出"
```

- ⚠ 本机路径含中文：**不要**用 `cv2.imread/imwrite`（返回 None），用 `run_pc.py` 的
  `imread_unicode/imwrite_unicode`。
- ⚠ 一律 `"D:/Python311/python.exe"`（系统 Python 已带 numpy/cv2）；WorkBuddy 托管 venv
  装不上 numpy/cv2，别在那里浪费时间。
- 现场只改 `config.py`；动阈值前先想是否该调曝光/位置，**别一上来放宽判据**。

---

## 5. ★ 每轮交 git（本项目最痛的坑：入口文件会被并发覆盖）

### 5.1 为什么要每轮交

`main.py` 在本项目被其他会话 / IDE 未保存标签页 / MaixVision 烧录**整份覆写过多次**，
写入者用的都是旧快照。`vision/*`、`config.py` 从未出事，**只有入口文件会丢**，
症状是"代码明明写了，上电却不生效"。git 是唯一可靠的回滚点 → **每完成一轮就提交**。

同理：有并行会话工作时**不要重命名或删除应用目录**——对方的相对路径写入会在旧路径
重新长出半个目录。确认无并发写入后再动目录结构。

### 5.2 一条命令提交（推荐）

```bash
"D:/Python311/python.exe" -B tools/git_round.py --check              # 检查 BUILD_ID / dist / 待提交清单
"D:/Python311/python.exe" -B tools/git_round.py -m "code: <摘要>"     # 精确 add 应用目录并 commit
```

脚本默认只 add `maixcam2_rect_center/`；要连记忆一起提交加 `--with-memory`。

### 5.3 手工提交要点

- 按路径精确 add；**绝不 `git add -A` / `git add .`**（并发写盘，会提交别人写一半的文件）。
- **门禁全绿后立即提交**——拖太久可能被并发覆盖，把坏版本入库。
- 仓库配了 `core.autocrlf=false` + `.gitattributes eol=lf`，**别改**。

### 5.4 回滚

```bash
git log --oneline -15 -- maixcam2_rect_center/main.py   # 看该文件历史
git checkout <sha> -- <文件>                             # 恢复单文件；或 git revert <sha>（更安全）
```

恢复后**必须重跑** `build_single.py --check` + `check_app.py --manifest`，确认没有半新半旧。

### 5.5 进库 / 不进库

- ✅ 应用本体、`tools/*.py` 与测试固件 png（`negative_e*.png` / `real_board_frame.png` /
  `real_scene_corner_gap.png`）、`dist/main_single.py`、应用目录 `*.md`、`.workbuddy/memory/*.md`。
- ❌ `__pycache__/`、可再生调试图（`e2e_report/`、`screenshot*/frame*/mask*/debug_*`）——`.gitignore` 兜底。
- 提交身份 `CloudyRain <cloudyrain@example.com>`（占位邮箱，推远程前先改）；
  远程 `github.com/Chemcrin/MaixCAM2-TargetCenter`。

---

## 6. 常见坑（都真实踩过）

1. MaixVision「运行当前文件」只下发一个文件 → `ModuleNotFoundError`。
   用「运行项目」，或先 `build_single.py` 再跑 `dist/main_single.py`。
2. MaixVision 烧录/并发会话会把 `main.py` 改回旧内容 → 上板前 `build_single.py --check` 当场验过期。
3. `image2cv()` 对灰度图返回 `(H,W,1)` 三维哑轴 → 别按 `ndim==2` 判灰度，按实际 shape 判断。
4. U0(A28/A29) **必须显式 pinmap**，空表静默不通（"yaw 不动"的历史根因之一）；
   ttyS0 还是系统日志口，故开机 `FLUSH_RX_ON_START`。
5. 位置模式 `acc=0` = 瞬时启停 → 卡顿/冲击。自检与闭环都给非 0 acc，越小越柔。
6. 标定依赖视觉会阻塞（黑框出视野干等）→ 已加 `CALIBRATE_VISUAL_TIMEOUT_S` 快速失败。
7. `app.yaml` 的 `exclude` 漏目录 → 文档混进应用包；改完核 `check_app.py --manifest`。
8. 调试抓屏迅速堆爆工作区 → 定期 `tools/clean_artifacts.py`（扫描）/ `--apply`（删除），
   保持应用目录 ≤5MB；脚本只删可再生产物，**保留测试固件**。
9. 显示在主进程 daemon 线程（`SHOW_ASYNC=True`，`vision/preview.py`）；**不要把 `Display()`
   放进子进程**——子进程起不来就一帧不画且无报错。排查先看 `[preview] mode=`。
10. 板端单帧 > `RESULT_MAX_AGE_S`(0.20s) 判 stale，且 `[perf]` 的 stage 被替换成 `"stale"`、
    **掩盖检测器真实阶段**。帧率塌到个位数先查这条链，
    详见 `存档/项目自检/板端识别问题整理_2026-10-13.md`。

---

## 7. 完成定义（DoD）

- **改完源码**：`BUILD_ID` 已更新 → `dist/main_single.py` 已重生成 → `check_pc.py` 全绿
  → **按精确路径提交**（前缀见 §0′）。
- **上板**：日志 `[rect-center] <BUILD_ID>` 与本地一致；`SELF-TEST` 出 4 步动作
  （pitch 上→下、yaw 左→右）；追踪起停柔和、停下激光不晃。
- **留痕**：往 `.workbuddy/memory/YYYY-MM-DD.md` **追加**一行本轮做了什么、怎么验证的
  （只记有长期价值的结论与坑）；跨会话必须知道的约定 → `MEMORY.md`。
  ⚠ 都在仓库根，不要写进应用目录。

---

## 8. 待办：目录改名（定案未执行）

`maixcam2_rect_center/` → `maixcam2_target_aim/`（用户 2026-10-13 定案）。
需同步：`app.yaml` 的 `id`、`build_single.py` / `check_app.py` 的路径假设、本文件与 README 全部路径引用。
执行前置：确认无并发写入（`find maixcam2_rect_center -mmin -5` 无输出）→ `git mv`（保留历史）
→ 改引用 → 门禁 → 提交（前缀 `repo:`）。
