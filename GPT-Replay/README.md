# GPT-Replay 历史代码备份

按用户明确指定的位置另存，恢复时间 2026-11-09；2026-11-10 按用户后续要求优化高帧率、旋转和透视检测。当前备份构建号 `2026-11-10-fast-perspective-v1`、应用版本 `1.4.0`。外层是备份容器，可运行应用位于 `maixcam2_rect_center/`。

## 来源及恢复边界

现行 `../maixcam2_rect_center/` 已被后续代码替换，且 Git 中没有完整的 `laser-spot-v1` 原始快照。因此本备份是“历史 Git 文件 + 本对话明确给出的代码/补丁”的恢复版，不能声称所有文件与当时磁盘逐字一致。未对现行应用执行回退或覆盖。

| 内容 | 来源 |
|---|---|
| 黑框算法、配置基础、图标、`vision/__init__.py` | Git `fe530c8` |
| 黑带采样 `[0.20,0.40,0.60]`、取最大值、Hough 默认关闭 | 本对话 10 月 6 日最终修订，与后续 Git `d2f36e7` 的提交说明相互印证 |
| `main.py` 性能日志/限频/实际光斑调用 | 从本对话最终代码重建 |
| `vision/aiming.py` 和 7 项光斑测试 | 从本对话最后的完整代码重建 |
| 门禁和黑框性能测试 | Git `d2f36e7`，保留历史测试判据 |
| Maix 桩 | Git `bfff85a`，备份独立运行，无需导入现行工程 |
| `test_replay.py`、说明和校验工具 | 本次恢复新增，仅用于备份验证 |
| 黑框检测器、性能回归和应用参数 | 本轮优化为低分辨率二值提案、ROI 跟踪、旋转透视带宽验证和原图四边精修 |

`restore_history.py` 是本次提取 Git 文件的一次性脚本，会拒绝覆盖已有文件；它只恢复基础文件，不能独自重建对话补写的全部模块。完整恢复应直接使用本目录或压缩包。

## 使用和完整性

1. 在 MaixVision 中选择 `GPT-Replay/maixcam2_rect_center`，使用“运行项目”。
2. 确认启动构建号为 `2026-10-06-laser-spot-v1`。
3. 如需核验备份，运行 `D:/Python311/python.exe -B seal_backup.py --check`。
4. 优化后交付包为 `GPT-Replay_2026-11-10-fast-perspective-v1.zip`；初始恢复包保留在 `GPT-Replay_2026-10-06-laser-spot-v1.zip`。优化版清单为 `SHA256SUMS_fast-perspective-v1.txt`，ZIP 校验值为同名 `.zip.sha256`。

PC 旋转、透视、低分辨率、失锁和光斑测试已执行；未连接 MaixCAM2 或云台，PC 耗时不作为板端 FPS 保证。优化版测试输出见 `validation.txt`。
