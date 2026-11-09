# GPT 对话恢复版：黑框中心与实际激光光斑

恢复日期：2026-11-09。功能基线为本对话最后完成的 `2026-10-06-laser-spot-v1`，应用版本 `1.2.0`。

这是历史恢复备份，来源与校验说明见上级 `README.md`。没有复制当前仓库后来改写的四层实现、电机闭环或同心圆检测。旧的镜头经验校正、光斑搜索初值和识别局限均保留，不能把本次 PC 验证视为板端重新验收。

## 运行

在 MaixVision 中打开本目录 `GPT-Replay/maixcam2_rect_center`，点击“运行项目”。不要只运行当前 `main.py`，也不要把 `GPT-Replay` 外层目录当成应用根。

设备依赖：MaixPy v4 的 `maix`、OpenCV `cv2`、NumPy。启动日志应显示 `2026-10-06-laser-spot-v1`。本版本只识别、绘制及可选发送坐标，不驱动电机。`ENABLE_UART=False` 为默认值。

| 模块 | 功能 |
|---|---|
| `main.py` | 灰度采图、兼容 `(H,W,1)`、15 FPS 显示限频、性能日志 |
| `config.py` | 640×480 相机、阈值、物理尺寸、激光坐标和串口配置 |
| `vision/rectangle_center.py` | 缩小图黑环/亮面双候选，原图四边直线精修，窄带跟踪，8×8 方程、高斯消元、72 点逆映射 |
| `vision/aiming.py` | 参考位置附近实际亮斑检测、偏差计算、箭头、ASCII 串口包 |
| `tools/` | 历史门禁、合成场景回归、光斑回归、独立 Maix 桩 |

## 激光和偏差

默认 `LASER_MODE="detect"`：仅在参考位置附近 ±32 px 查找小而孤立的亮斑，去背景后用连通域和亮度加权质心求实际位置。无可靠光斑即无效，不使用旧帧光点。固定参考模式需要显式设 `LASER_MODE="fixed"`。

`LASER_REFERENCE_SIZE=(453,333)`、`LASER_PIXEL=(209,144)` 来自本对话截图，仅作为搜索初值；实际落点会因距离、曝光、裁剪、镜头校正而变化。当前 `LENS_CORR=True`、强度 1.5 也是历史经验值，不是该相机的标定结果。

`dx=靶心u−光斑u`，`dy=靶心v−光斑v`，原点左上，右/下为正。两者都有效才画橙色箭头，箭头从激光点指向黑框中心。绿框为黑框外沿，黄色线为内沿，顶点有坐标标注。灰度亮点检测无法保证区分所有点状反光。

串口 `UART_PROTOCOL="aim"`：

```text
@A,aim_valid,tx,ty,lx,ly,dx,dy,calibrated,laser_valid,target_valid%\r\n
```

`aim_valid=0` 时偏差字段填 0，下游必须先检查有效位。旧 `center` 协议可显式选择，只输出 `@[u,v]%` / `@[invalid]%`。

## PC 验证

在本目录执行：

```powershell
D:/Python311/python.exe -B tools/check_app.py --manifest
D:/Python311/python.exe -B tools/test_aiming.py
D:/Python311/python.exe -B tools/test_replay.py
D:/Python311/python.exe -B tools/test_performance.py
```

保留历史测试判据与已知漏检，不为“通过”修改算法/测试阈值。默认 `ENABLE_LINE_RECOVERY=False`，以避免失锁时 Hough 组合引起的秒级阻塞；极端反光断环可能漏检。本次重跑的实际数字记录在上级 `validation.txt`。
