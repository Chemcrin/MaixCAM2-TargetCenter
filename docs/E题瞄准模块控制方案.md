# E 题瞄准模块控制方案（MaixCAM2 视觉 + ZDT 云台 + MSPM0G3507 天猛星）

> 本文档承接《E题视觉方案_MaixCAM2.md》，把系统架构落到实处：云台由两个张大头 ZDT 闭环步进电机驱动，主控小车为立创天猛星 MSPM0G3507，相机装在云台上、激光笔刚性固定在相机顶部。  
> 所有 MaixPy / ZDT 相关接口均已核对官方资料，ZDT 多字节字段的字节序请以你手上驱动器版本的手册为准（文中已标注待确认项）。

---

## 0. 结论速览

| 问题             | 结论                                                                                            |
| -------------- | --------------------------------------------------------------------------------------------- |
| 相机在云台上，闭环怎么设计？ | **误差 `e = T_px − L_px`**（靶面目标点像素 − 激光点像素）。这是唯一正确解，且**自动吸收视差**。                                |
| 激光不在画面中心有关系吗？  | **没关系，反而更简单**。用 `e = T_px − L_px` 时，视差被完全吸收，**机械装配不需要精确同轴对齐**，只要激光点在相机视野内即可。                  |
| 云台用速度模式还是位置模式？ | **跟踪阶段用速度模式（0xFD→0xF6），扫描阶段用位置模式**。视觉伺服本质是"按图像误差调云台速度"，速度模式天然匹配。                              |
| 谁控制云台？         | **MaixCAM2 直接控制**。它有两个以上可用 UART，用 `pinmap` 分配即可。这样"瞄准模块"= MaixCAM2 + 云台 + 激光，一个开关独立供电，完全符合题意。 |
| 最大的隐藏风险是什么？    | **MaixCAM2 是 Ubuntu，开机十几秒**，而基本(2) 只有 2s。必须做开机自启 + 启动优化 + 赛前实测，见第 9 节。                        |

---

## 1. 系统架构（落定版）

```
                 ┌──────────── 开关1 + LED ────────────┐
                 ▼                                     │
      ┌──────────────────────┐                         │
      │  MSPM0G3507 天猛星    │  巡迹 PID · 电机 PWM     │
      │  （题目强制）          │  编码器测速/里程          │
      └──────────┬───────────┘                         │
                 │ UART  (里程 / 圈进度 / 状态)          │
                 │                                     │
                 │              ┌──────── 开关2 + LED ──┘
                 ▼              ▼
      ┌──────────────────────────────────────────────┐
      │              MaixCAM2（瞄准模块大脑）           │
      │  视觉：靶纸检测 · 单应矩阵 · 激光点识别          │
      │  控制：图像空间 PID + IMU 前馈                  │
      │  板载 QMI8658 六轴 IMU（装在云台上）            │
      └───┬────────────┬────────────┬─────────────────┘
          │ UART4      │ UART2      │ GPIO
          ▼            ▼            ▼
   ┌───────────┐ ┌───────────┐  ┌──────────┐
   │ ZDT 驱动1 │ │ ZDT 驱动2 │  │ 激光继电器│
   │ Pan  地址1│ │ Tilt 地址2│  └──────────┘
   └─────┬─────┘ └─────┬─────┘
         ▼             ▼
     ┌────────┐   ┌────────┐
     │ 42步进 │   │ 42步进 │
     │  水平轴 │   │ 俯仰轴 │
     └────────┘   └───┬────┘
                      ▼
              ┌──────────────────┐
              │  相机 (MaixCAM2) │
              │   ↑ 顶部装激光笔  │
              └──────────────────┘
```

**关键点：**

1. **MaixCAM2 同时承担"视觉"和"云台控制"**，省掉一块 MCU。ZDT 驱动器自带 FOC 闭环（力矩/速度/位置三环 20kHz），MaixCAM2 只需要按 30~50Hz 下发速度指令，对 Linux 的实时性要求很低，完全可行。
2. **MSPM0 只管小车**，符合题目"巡迹、电机控制必须用 MSPM0"的强制要求，也不会被云台任务拖累。
3. **两路独立开关**天然对应：开关1 = 小车系统，开关2 = 瞄准模块。两个 LED 分别指示。
4. **UART 分工见第 4 节**。MaixCAM2 的 UART0 是系统终
5. .端，**绝对不要占用**。

---

## 2. 核心几何：视差分析与闭环设计

这是整个方案里最需要想清楚的一段。

### 2.1 物理布置

```
        ┌── 激光笔 (装在相机顶部，高 h)
        │
   ┌────┴─────┐
   │  MaixCAM2 │  ← 相机光心
   │  相机      │
   └────┬─────┘
        │ 云台 Tilt 轴
   ═════╧═════  云台 Pan 轴
```

- 激光笔发光中心在相机**光心正上方 h** 处（典型 3~6cm）
- 激光束方向与相机光轴**大致平行**（装配误差 ±几度）
- **激光笔与相机刚性连接**，一起随云台转动

### 2.2 视差：激光点不在画面中心

激光束平行于光轴但向上偏移 h 时，打在距离 d 的靶面上，其图像位置为：

```
Δy_px = − h · f_px / d          （负号 = 在画面中心上方）
f_px  = W / (2 · tan(HFOV / 2))   （水平像素焦距）
```

取 W = 1280、HFOV = 60° → **f_px ≈ 1108 px**：

| 相机到靶距离 d          | h = 3cm 时 Δy | h = 5cm 时 Δy |
| ----------------- | ------------ | ------------ |
| 50 cm（小车在 B 点）    | 66 px        | 111 px       |
| 100 cm（场地中部）      | 33 px        | 55 px        |
| 165 cm（小车在 C/D 点） | 20 px        | 34 px        |

**结论：激光点在画面中既不居中，垂直位置还随距离显著变化（20~111px）。**

### 2.3 因此：绝对不能用"图像中心闭环"

很多资料会告诉你"相机和激光同轴，让靶心移到画面中心即可"。**在这个装法下这是错的**——因为激光点根本不在画面中心，而且偏移量随距离变。

### 2.4 正确解：`e = T_px − L_px`

```
T_px = project(H, T_cm)     # 靶面目标点当前在图像中的位置
L_px = 激光点在图像中的位置（每帧实测）
e    = T_px − L_px
```

云台驱动让 `e → 0`，即"把靶面目标点拖到激光点所在的位置"。

**为什么它严格正确、且完全无视差误差？**

激光点是靶面上的一个**真实物理点**。它在图像中的位置是 `L_px`。而单应矩阵 `H` 是"靶面 cm 坐标 → 图像像素"的**双射**，所以：

```
H⁻¹(L_px)  = 激光在靶面上的真实位置 L_cm
```

要求 `L_cm = T_cm`，等价于 `H(L_cm) = H(T_cm)`，等价于 `L_px = T_px`。

**这是一个恒等关系**，与 h、f、d 全都无关。所以：

> ✅ **视差被数学上严格吸收，不引入任何瞄准误差。**

### 2.5 三个直接红利

**红利一：机械装配难度大幅降低。**  
不需要把激光光轴和相机光轴精确对齐（那是最难调的机械活）。只要满足：

- 激光点在相机视野内
- 激光打在靶面上时，光斑清晰可检

装配误差全部由闭环吸收。**这一条能省掉你在赛场上好几个小时。**

**红利二：可以实时预判得分。**  
`px_per_cm` 由 H 局部求导得到，于是：

```python
err_cm = (L_px - T_px) / px_per_cm     # 单位 cm
```

这个数**直接就是题目里的 D₁ / D₂**。挂在屏幕上实时显示，测试时一眼就知道当前扣几分，不用等感光纸出痕迹。

**红利三：打靶心和画圆共用同一条代码路径。**  
只需换 `T_cm`：

```python
T_cm = (10.5, 14.85)                                  # 打靶心
T_cm = (10.5 + 6*cos(θ), 14.85 + 6*sin(θ))            # 画 6cm 圆，θ = 2π·圈进度
```

### 2.6 相机在云台上的额外影响

- **H 每帧都在变**。云台一转，靶纸在画面里就移动，单应矩阵必须持续更新（每 1~2 帧重检一次靶纸）。
- **靶纸必须始终在视野内**。画圆时云台只需转动约 `6cm / d` 弧度（100cm 时约 3.4°），靶纸不会跑出画面，但要保证 H 连续更新。
- **云台转动 → IMU 测到角速度**。因为 MaixCAM2 板载 QMI8658 就装在云台上，这正好给了第 7 节的前馈信号。

---

## 3. ZDT（张大头）Emm42_V5.0 协议详解

### 3.1 驱动器关键特性

| 项目   | 参数                                                |
| ---- | ------------------------------------------------- |
| 型号   | Emm42_V5.0（基于 ZDT_X42_V1.2 软硬件框架）                 |
| 主控   | 32 位 ARM（Cortex-M4F @120MHz）                      |
| 闭环   | FOC 矢量闭环，力矩/速度/位置环均 20kHz+，板载 16384 线磁编码器         |
| 控制精度 | **< 0.08°**（远超本题需求）                               |
| 供电   | 7~32V（推荐 24V；车上用 12V 也在范围内）                       |
| 电流   | 0~3000mA 可设                                       |
| 最高转速 | 0.1 ~ 3000+ RPM                                   |
| 细分   | 1~256 任意细分，支持细分插补                                 |
| 通讯   | 串口 TTL / RS232 / RS485 / CAN，支持 Modbus-RTU 与自定义协议 |
| 多机   | 支持最多 255 个地址，支持多机同步控制                             |

### 3.2 帧格式

```
┌────────┬────────┬──────────────┬────────┐
│  地址   │ 功能码  │   指令数据    │  校验   │
│  1 B   │  1 B   │    N B       │  1 B   │
└────────┴────────┴──────────────┴────────┘
```

- **地址**：1~255，**0 为广播地址**
- **校验**：默认固定 `0x6B`（可选 XOR / CRC-8 / Modbus，菜单 `Checksum` 可改）
- **多字节字段**：张大头协议惯例为**大端（高字节在前）**。⚠️ 请以你手上驱动器版本的手册二次确认。

### 3.3 常用命令（重点）

#### 电机使能 `0xF3`

```
addr  F3  AB  state  save  6B
```

| 字段      | 值           | 说明                     |
| ------- | ----------- | ---------------------- |
| `AB`    | 固定          | 该命令的固定标识               |
| `state` | `00` / `01` | `00`=失能，`01`=使能        |
| `save`  | `00` / `01` | `00`=不存储（掉电失效），`01`=存储 |

示例（地址 1，使能，不存储）：

```
01 F3 AB 01 00 6B
```

#### 速度模式 `0xF6`（跟踪阶段主力）

```
addr  F6  dir  speed_H  speed_L  acc  sync  6B     ← 共 8 字节
```

| 字段                | 说明                                    |
| ----------------- | ------------------------------------- |
| `dir`             | `00` = CW，`01` = CCW                  |
| `speed_H:speed_L` | 目标转速，2 字节大端，单位 RPM，范围 0 ~ 5000        |
| `acc`             | 加速度档位 `00`~`FF`，`00` = 不启用曲线加减速（立即到速） |
| `sync`            | 多机同步标志，`00` = 独立运行                    |

示例（地址 1，CCW，1500RPM，加速度 10）：

```
01 F6 01 05 DC 0A 00 6B
```

（`0x05DC` = 1500）

> 建议：**跟踪时 `acc` 给一个中等值（如 `0x10`）**，既保证响应快，又不会因为速度突变导致云台抖动丢步（虽然闭环不会丢步，但机械冲击会让画面抖）。

#### 位置模式 `0xFD`（扫描阶段用）

```
addr  FD  dir  speed_H  speed_L  acc  pos3 pos2 pos1 pos0  mode  sync  6B
```

共 **13 字节**。

| 字段       | 说明                                 |
| -------- | ---------------------------------- |
| `pos[4]` | 脉冲数，4 字节大端                         |
| `mode`   | `00` = **相对位置**，`01` = **绝对位置**    |
| `sync`   | `00` = 独立运行，`01` = 同步（等 `0xFF` 触发） |

**脉冲数换算**（16 细分，1.8° 步进电机，200 步/圈）：

```
每圈脉冲 = 200 × 细分 = 200 × 16 = 3200
转 1°   = 3200 / 360 ≈ 8.89 脉冲
```

#### 立即停止 `0xFE`

```
addr  FE  98  00  6B
```

#### 同步运动 `0xFF`

```
addr  FF  66  6B
```

#### 读取实时转速 `0x35`

发送：

```
addr  35  6B
```

返回（8 字节）：

```
addr  35  dir  speed_H  speed_L  6B     ← 需开启 Response
```

#### 读取当前位置 `0x36`

发送 `addr 36 6B`，返回当前位置（脉冲数）。

#### 清零位置 `0x0A`

```
addr  0A  6B
```

#### 到位回传（被动上报）

驱动器到位后主动上报 **4 字节**：

```
addr  FD  9F  6B
```

> 需要在驱动器菜单里把 `Response` 设为 `Reached` 或 `Both`。跟踪阶段用不上，但**扫描阶段判断"是否到位"很有用**（比盲等时间可靠）。

### 3.4 赛前必修：驱动器菜单配置

**这一步漏了，通讯完全不工作，且很难排查。**

| 菜单         | 应设为                         | 说明                                         |
| ---------- | --------------------------- | ------------------------------------------ |
| `P_Serial` | **`UART_FUN`**              | 把 R/A/H 与 T/B/L 端口复用为串口。**默认不是这个，必须改**     |
| `UartBaud` | **`115200`**                | 与 MaixCAM2 一致。MaixCAM2 官方验证可靠的波特率就是 115200 |
| `ID_Addr`  | **Pan = 1，Tilt = 2**        | 两台地址必须不同                                   |
| `Checksum` | `0x6B`                      | 默认值，与本文档代码一致                               |
| `MStep`    | `16`                        | 位置模式的脉冲换算要用                                |
| `En`       | `Hold`                      | 一直使能（不接线时的默认值）                             |
| `Response` | `Reached`（扫描用）/ `None`（跟踪用） | 见第 4.2 节关于共用总线的说明                          |
| `MotType`  | 按你的电机选 `1.8°` 或 `0.9°`      | 影响脉冲换算                                     |
| `Cal`      | 首次必须做                       | 编码器校准，空载执行                                 |

> ⚠️ 换驱动器或拆装过驱动板后，磁铁与编码器相对位置改变 → **必须恢复出厂设置再重新校准编码器**。

### 3.5 接线（TTL 串口）

```
MaixCAM2 TX  ──→  ZDT 驱动 R/A/H (RX)
MaixCAM2 RX  ←──  ZDT 驱动 T/B/L (TX)
MaixCAM2 GND ────  ZDT 驱动 GND          ← 必须共地！
```

驱动器 V+ / GND 接 7~32V（车上用 12V 即可）。

---

## 4. MaixCAM2 串口与引脚分配

### 4.1 可用的 UART

MaixCAM2 有 5 组串口：

| 串口        | TX / RX 引脚    | 设备路径             | 用途建议                       |
| --------- | ------------- | ---------------- | -------------------------- |
| UART0     | U0T / U0R     | `/dev/ttyS0`     | **系统终端与日志串口，不要占用**         |
| UART1     | 需 pinmap 分配   | `/dev/ttyS1`     | 备选                         |
| UART2     | B0 / B1       | `/dev/ttyS2`     | **→ ZDT Tilt（地址 2）**       |
| UART3     | 需 pinmap 分配   | `/dev/ttyS3`     | 备选                         |
| **UART4** | **A21 / A22** | **`/dev/ttyS4`** | **板上默认引脚 → ZDT Pan（地址 1）** |

**推荐分配：**

| 用途        | 串口              | 说明                |
| --------- | --------------- | ----------------- |
| ZDT Pan   | UART4 (A21/A22) | 默认引脚，接线最省事        |
| ZDT Tilt  | UART2 (B0/B1)   | 默认引脚              |
| MSPM0 天猛星 | UART1 或 UART3   | 需查 pinmap 文档选可用引脚 |

> 如果引脚不够，**也可以两个 ZDT 共用一条 UART 总线**：地址分别为 1 和 2，把两台驱动器的 `Response` 都设为 `None`（永不发送），就不会有总线争用。代价是失去到位回传。见 4.2。

### 4.2 一条总线带两台电机（备选）

TTL 串口是点对点的，但 ZDT 协议首字节是地址，所以：

- MaixCAM2 的 TX 并联到两台驱动器的 RX → 两边都能收到
- 两台驱动器的 TX 并联回 MaixCAM2 的 RX → **只要都不发送就没有冲突**

所以把 `Response` 设为 `None`，就可以一条 UART 控制两台电机，多机同步也能用。

**代价**：读转速（0x35）、读位置（0x36）、到位回传（FD 9F）都收不到了。  
**结论**：跟踪阶段用速度模式、不需要读回 → 可以用一总线方案；但如果扫描要到位确认，建议还是**两个独立 UART**。

### 4.3 pinmap 配置代码

```python
from maix import uart, pinmap, err, sys

if sys.device_id() == "maixcam2":
    # UART4 默认引脚 A21/A22，仍建议显式设置一次
    err.check_raise(pinmap.set_pin_function("A21", "UART4_TX"), "set UART4 TX failed")
    err.check_raise(pinmap.set_pin_function("A22", "UART4_RX"), "set UART4 RX failed")
    # UART2 默认引脚 B0/B1
    err.check_raise(pinmap.set_pin_function("B0", "UART2_TX"), "set UART2 TX failed")
    err.check_raise(pinmap.set_pin_function("B1", "UART2_RX"), "set UART2 RX failed")
else:
    raise RuntimeError("本方案针对 MaixCAM2")

print("串口设备列表:", uart.list_devices())

ser_pan  = uart.UART("/dev/ttyS4", 115200)   # Pan  (addr 1)
ser_tilt = uart.UART("/dev/ttyS2", 115200)   # Tilt (addr 2)
```

> ⚠️ 引脚默认可能被其他功能占用（SPI / WiFi 等）。使用前务必对照 MaixCAM2 的 PINMAP 文档确认没冲突。

---

## 5. 云台闭环控制

### 5.1 为什么用速度模式

|      | 速度模式                | 位置模式         |
| ---- | ------------------- | ------------ |
| 语义   | "转多快"               | "转到哪个角度"     |
| 天然匹配 | ✅ 视觉伺服（图像误差 → 云台速度） | ❌ 每帧都要算绝对角度  |
| 到位处理 | 速度给 0 即停，闭环步进保持力矩   | 需要判断到位（有延迟）  |
| 抖动   | 好，可连续调节             | 每帧跳目标位置，容易顿挫 |
| 适合   | **锁定 / 跟踪 / 画圆**    | **扫描（点到点）**  |

**结论：跟踪用速度模式，扫描用位置模式，两者随时切换（同一驱动器自由切换，不需要重新使能）。**

### 5.2 控制律

```python
# 图像空间 PID（每轴独立）
e_x = T_px[0] - L_px[0]        # Pan 轴误差（像素）
e_y = T_px[1] - L_px[1]        # Tilt 轴误差（像素）

u_pan  = Kp_x*e_x + Ki_x*∫e_x + Kd_x*de_x + Kff_x*(-gyro_x)
u_tilt = Kp_y*e_y + Ki_y*∫e_y + Kd_y*de_y + Kff_y*(-gyro_y)

# 图像空间输出 → ZDT 转速（RPM）
rpm_pan  = clamp(u_pan,  -RPM_MAX, RPM_MAX)
rpm_tilt = clamp(u_tilt, -RPM_MAX, RPM_MAX)
```

**几个关键细节：**

1. **误差符号要先实测确定**。云台转动方向与图像位移方向的关系取决于电机安装方向和 `dir` 设置。第一次调试时先手动给一个小的正速度，看激光点在图像里往哪边跑，必要时把 PID 输出取反（加一个 `SIGN_PAN = ±1`）。
2. **图像空间误差与距离无关**，所以 PID 增益**不需要随距离调整**。这是图像空间闭环最大的好处。（小角度下，图像位移 ≈ 角度变化 × f_px，与距离无关。）
3. **RPM 换算**：
   ```
   ω(°/s) = RPM × 6 / 减速比      (直驱时减速比 = 1)
   ```
   直驱时 1 RPM = 6°/s。图像误差 40px 对应 `40/1108 rad = 2.07°`，若要 0.1s 内消除 → 20.7°/s ≈ 3.5 RPM。可见**需要的转速极低，RPM_MAX 给 20~60 就绰绰有余**，不要设太大，否则容易过冲振荡。


4. **加死区**。云台机械有回差（背隙），误差小于 2~3px 时应当把速度置 0，否则会在死区附近来回抖。
   ```python
   if abs(e_x) < DEADZONE_PX: rpm_pan = 0
   ```

5. **积分限幅（anti-windup）**。目标丢失时积分会累积到爆炸，重新锁定时会猛冲。必须做：
   ```python
   integral += e * dt
   integral = clamp(integral, -I_LIMIT, I_LIMIT)
   if not tracked: integral = 0
   ```

6. **丢帧处理**。激光点或靶纸丢失时：
   - 短暂丢帧（1~3 帧）：保持上一次速度输出（惯性前冲）或速度归零
   - 持续丢帧：速度归零，进入搜索

### 5.3 画圆的云台表现

画 6cm 圆 → 靶面上半径 6cm，在图像上约 `6 × px_per_cm` 像素。165cm/1280px 下 `px_per_cm ≈ 6.7`，即图像上半径仅 **40 像素**；100cm 时约 11 px/cm → **66 像素**。

对应云台转角：
```
θ_gimbal ≈ 6cm / d 弧度
```
100cm → 0.06 rad = **3.4°**；165cm → 2.1°。

**非常小的角度**，单圈 20s 内完成毫无压力，且精度容易保证。这是发挥(3) 反而比发挥(1)(2) 更容易做好的原因——难的是**同步**，不是精度。

---

## 6. 扫描搜索策略（基本 2 / 3 的 2s 内锁定）

### 6.1 先算清楚搜索空间

好消息：**俯仰方向几乎不用扫。**

- 靶高 ≤50cm，靶心约在离地 25cm 处；相机装在车上，高度约 15~20cm
- 靶距 50~165cm
- 需要的俯仰角 = `atan((25-17)/d)`：d=50cm → 9.1°，d=165cm → 2.8°
- 相机垂直 FOV（720 高、f_px=1108）= `2·atan(360/1108)` = **36°**

**结论：只要相机近似水平安装，垂直 FOV 36° 已经完全覆盖 2.8°~9.1° 的需求，俯仰轴在扫描阶段可以不动（或只设 1~2 个固定档）。**

只有水平方向需要扫。全景 360°。

### 6.2 扫描参数

| 参数 | 建议值 | 理由 |
|---|---|---|
| 水平步长 | **48°** | 相机 HFOV 60°，重叠 12°，边缘畸变区不依赖 |
| 步数 | **8 步**（覆盖 360°） | 48 × 8 = 384° > 360° |
| 每步停留 | **180~200 ms** | 30fps 下 5~6 帧，足够 find_rects 出结果 |
| 位置模式转速 | **60~120 RPM** | 48° 直驱下需转 0.133 圈；120RPM 时 0.067s，加加减速约 0.1s |
| 加速度档位 | `0x80` | 快速但不失步 |

时间预算：8 × (0.1s 转动 + 0.19s 检测) ≈ **2.3s**，略超 2s。

**优化到 2s 以内的三个手段：**

1. **利用题目的"位置和姿态自定"** —— 基本(2) 的原文是"将小车放置在场地中，**位置和姿态自定**"。如果赛区允许队伍自己摆放，就把车放在靶正前方 60~90cm 处、车头朝靶，**几乎不需要扫描**，2s 绰绰有余。
2. **扫描时用低分辨率**：`camera` 多通道（`cam.add_channel(640, 360, ...)` + `cam.get_channel(1)`）做快速粗检，命中后再切高分辨率精定位。`find_rects` 的耗时会显著下降。
3. **优先扫描"合理方向"**：先看当前视野（第 0 步），没有再从中间向两侧交替扫（0°, +48°, −48°, +96°, −96°…），而不是从头到尾单向扫。**平均命中步数从 4.5 降到 2.5**。

### 6.3 扫描流程

```python
def scan_for_target(pan, tilt, cam, timeout_s=1.8):
    t_start = time.time()
    # 第 0 步：先看当前视野（廉价的可能命中）
    order = [0, +48, -48, +96, -96, +144, -144, 180]
    for deg in order:
        if time.time() - t_start > timeout_s:
            return False
        if deg != 0:
            pan.move_abs_deg(deg, rpm=120, acc=0x80)     # 位置模式，绝对角度
            wait_reach(pan, timeout_ms=200)               # 收到位回传或超时
        # 连续若干帧检测
        for _ in range(6):
            img = cam.read()
            q = detect_target(img)
            if q is not None:
                return True                                # 命中，立刻切入闭环
    return False
```

> **绝对角度**需要一个"零位"参考。做法：上电后先让云台慢速转到机械限位（或用限位开关回零），把该位置定义为 0°；或者干脆用**相对位置模式**，只记录"相对上电位置的偏移"，扫描时按偏移量走。后者简单，推荐。

---

## 7. IMU 前馈与"反自旋"

### 7.1 为什么需要

动态瞄准（发挥 1/2）时小车在跑，还会转向、颠簸。相机在云台上，会跟着晃。**纯图像闭环有 1~2 帧的固有延迟**，单靠 PID 会在车快速转向时明显滞后。

但 MaixCAM2 **板载 QMI8658 六轴 IMU，而且整块板就装在云台上** —— 这意味着 IMU 直接测到的就是相机（云台）的角速度，是天然的**前馈信号源**。

### 7.2 两种用法

**用法一：前馈补偿（降低滞后）**

```python
rpm_pan += Kff_yaw * gyro_z      # 符号按安装方向实测确定
```

角速度前馈是"预测性"的——在图像误差还没变大之前就先动起来，显著改善相位滞后。

**用法二：反自旋（小陀螺）**

小车急转弯时，云台会被动跟着转。如果不补偿，靶纸会瞬间甩出视野。做法：

```python
# 目标：让云台在世界坐标系中保持指向不变
rpm_pan_hold = -gyro_z * RAD2DEG / 6.0 / K  # 抵消车体旋转
```

MaixPy 提供姿态解算模块：

```python
from maix import ahrs, tensor
# ahrs.AHRS(kp, ki)
#   .init(ax, ay, az, mx=0, my=0, mz=0)
#   .update(ax, ay, az, gx, gy, gz, mx, my, mz, dt)
#   .get_angle(acc, gyro, mag, dt, radian=False) -> 姿态角
#   .reset()
```

> ⚠️ 原始 IMU 数据的读取接口（`maix.imu` / `maix.qmi8658` 模块）在你的固件版本上请用 `dir()` 或官方 API 文档确认签名后再用。**AHRS 的签名已核对**（见上方注释）。

### 7.3 实用建议

- **先别急上 IMU**。用纯图像 PID 把基本要求跑通，再在发挥部分加前馈。前馈是"锦上添花"，PID 没调好时加前馈只会更乱。
- 前馈系数 `Kff` 从 `0` 开始慢慢加，加到"车转向时靶纸不再甩出视野"即可，不要贪大。

---

## 8. 与 MSPM0G3507 天猛星的通信

### 8.1 天猛星板的可用资源（重要）

立创天猛星是 **LQFP-64** 的 MSPM0G3507 板，板载：

| 板载资源 | 引脚 | 备注 |
|---|---|---|
| OLED | — | 调试显示 |
| **LSM6DS3 六轴 IMU** | PA27 / PA28（I2C） | 小车姿态，非必需 |
| WS2812 RGB | PB26 | 状态指示 |
| 无线模块 | PB17 / PB18（UART7） | 不用可释放 |
| 蜂鸣器 | PB27 | |
| **QEI 编码器 PHA/PHB** | PA29 / PA30 | **测速/里程直接可用** |
| CH340E 串口 | PA10 / PA11（UART0） | 烧录 + 调试日志，**建议保留** |
| ENTER 按键 | PB21 | |
| 板载 LED | PB22 | |

**禁止占用（blocked）**：PA3/PA4（32.768kHz 晶振）、PA5/PA6（40MHz 晶振）、PA19/PA20（SWD 调试）。

**分配建议：**
- 与 MaixCAM2 通信：用 **UART1/UART2/UART3** 中的一路，自行分配空闲引脚（避开上表 blocked 项）
- 巡迹传感器：ADC / GPIO
- 电机驱动：PWM
- 编码器：优先用板载 QEI（PA29/PA30），另一路用 Timer 计数

> ⚠️ 引脚复用前必须确认板载功能是否还需要（`.syscfg` 里能看到）。改动后用 SysConfig 重新生成配置。

### 8.2 为什么 MaixCAM2 → MSPM0 的数据量很小

| 方向 | 内容 | 频率 | 字节 |
|---|---|---|---|
| MSPM0 → MaixCAM2 | 里程、单圈进度、行驶状态 | 50 Hz | ~8 |
| MaixCAM2 → MSPM0 | 可选（启动指令 / 状态回显） | 按需 | ~4 |

**画圆需要的"圈进度"必须来自 MSPM0 的编码器**，不能用时间。这是发挥(3) 同步误差 < 1/4 圈的保证。

### 8.3 自定义协议（推荐）

```
┌──────┬──────┬──────┬──────┬────────────┬────────┐
│ 0xAA │ 0x55 │ type │ len  │  payload   │ CRC8   │
└──────┴──────┴──────┴──────┴────────────┴────────┘
```

**type = 0x10：MSPM0 → MaixCAM2（里程/进度），payload 8 字节**

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `odom_mm` | uint32 | 累计里程，mm（**小端**） |
| 4 | `lap_progress` | uint16 | 当前圈完成度 ×10000（0~10000 对应 0~1.0） |
| 6 | `lap_count` | uint8 | 已完成的圈数 |
| 7 | `flags` | uint8 | bit0: 正在行驶 bit1: 已归零 |

**type = 0x11：MaixCAM2 → MSPM0（可选）**

| 字节 | 说明 |
|---|---|
| 0 | `cmd`：0=空闲 1=开始循迹 2=停止 |
| 1 | `param` |

### 8.4 MaixPy 侧代码

```python
import struct

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def parse_mspm0_frame(buf: bytes):
    """返回 (odom_mm, lap_progress, lap_count, flags) 或 None"""
    if len(buf) < 6:
        return None
    if buf[0] != 0xAA or buf[1] != 0x55:
        return None
    length = buf[3]
    if len(buf) < 4 + length + 1:
        return None
    body = buf[:4 + length]
    if crc8(body) != buf[4 + length]:
        return None
    if buf[2] != 0x10:
        return None
    odom, prog, laps, flags = struct.unpack('<IHBB', body[4:4+8])
    return odom, prog / 10000.0, laps, flags
```

> 接收建议用 `serial.read()` 在独立线程里做缓冲，主循环只读最新值（避免阻塞视觉）。MaixPy 有 `maix.thread` 模块可用。注意：**`set_received_callback()` 与 `read()` 不能同时用**，二选一。

### 8.5 也可以用 MaixPy 自带的 `maix.comm`

MaixPy 提供高层通信协议模块 `maix.comm`（已核对签名）：

```python
from maix import comm
# comm.CommProtocol(buff_size=1024, header=3148663466, method_none_raise=False)
p = comm.CommProtocol()
p.report(cmd=0x02, body=struct.pack('<hh', err_x, err_y))   # 主动上报
msg = p.get_msg(timeout=0)                                   # 收请求
if msg and msg.is_req:
    p.resp_ok(msg.cmd, b'')                                  # 回 OK
```

**优点**：自带帧头、长度、校验，不用自己写。
**缺点**：MSPM0 侧要按同一格式实现解析（`header` 是魔数，需要对齐）。
**建议**：如果你不想在 MSPM0 侧写复杂解析，用 8.3 的自定义 6+N 字节协议更省事。

---

## 9. ⚠️ 最大的隐藏风险：MaixCAM2 开机时间 vs 2 秒

### 9.1 问题

MaixCAM2 跑的是 **Ubuntu Linux**，从通电到系统就绪通常需要 **十几到二十几秒**。

而题目基本(2) 要求：**"启动瞄准模块在 2s 内发射激光击中靶心"**。

参考国一作品公开的测试数据：基本(2) **1.94s 完成**，其测试方案写明 **"启动瞄准模块时开始计时，击中目标点后停止计时"**。

**如果"启动"= 合上电源开关，那么以 Linux 为视觉平台的方案在 2s 内物理上不可能完成。**

### 9.2 三种可能的解读与对应准备

| 解读 | 含义 | 你的准备 |
|---|---|---|
| A. "启动" = 合上电源开关 | 计时含开机时间 | ❌ MaixCAM2 不可能达标。**必须在赛前向赛区/裁判确认** |
| B. "启动" = 给一次启动信号（按键/上位机指令），系统此前已上电就绪 | 计时不含开机 | ✅ 主流做法，按这个准备 |
| C. 测试流程中瞄准模块开关一直合着，只在基本(1) 时断开一次 | 视同 B | ✅ |

**行动项（必须做，越早越好）：**

1. **向赛区/指导老师确认计时起点**，并保留书面/邮件依据。
2. **实测你手上 MaixCAM2 的启动到程序就绪时间**（从合开关开始秒表计时，到屏幕上出现你就绪指示为止），把这个数字写进设计报告。
3. **实现开机自启**（见 9.3），并**设计一个明确的"就绪信号"**：屏幕显示 `READY` + 板载 LED 亮 + 串口发一条就绪帧。让裁判能清楚看到"系统已就绪，现在才是有效的启动时刻"。
4. **设计报告里专门写一节"启动时序"**，说明系统上电→自检→就绪的时间，以及"就绪后接到启动信号 2s 内完成"的设计。这是最专业的处理方式。

### 9.3 开机自启的三种做法

MaixCAM2 基于 **systemd**。

**方法一：`/maixapp/auto_start.txt`（最简单）**
在设备上创建 `/maixapp/auto_start.txt`，写入要自启的应用名。删除该文件即取消。

**方法二：自建 systemd service（推荐用于生产）**

```ini
# /etc/systemd/system/aiming.service
[Unit]
Description=E-Problem Aiming Vision
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /root/aiming/main.py
Restart=always
WorkingDirectory=/root/aiming
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

```bash
systemctl daemon-reload
systemctl enable aiming.service
systemctl start aiming.service
```

> 参考 MaixCAM2 自带的 `launcher.service` 写法。
> ⚠️ 用 systemd 自启会让 MaixVision 无法停止该程序（屏幕和摄像头被占用）。**开发阶段先用方法一或手动运行，比赛前一晚再切到 systemd。**

**方法三：`/etc/rc.local`（MaixCAM 一代适用；MaixCAM2 用 systemd 更规范）**

### 9.4 启动时间优化清单

| 优化项 | 做法 | 预期收益 |
|---|---|---|
| 禁用无关服务 | `systemctl disable` 蓝牙、WiFi（比赛不需要联网）、打印服务等 | 2~5s |
| 程序轻量化 | **不要在启动时加载神经网络模型**。本方案纯传统 CV，无需模型 | 大 |
| 延迟加载相机 | 先起进程、后开相机，避免等待硬件就绪阻塞 | 1~2s |
| 根文件系统只读 | 避免 fsck | 1~2s |
| 关闭 OLED/UI | 只跑后台逻辑，不做复杂界面 | 少量 |
| 硬件 | 用 eMMC 版本（32GB/64GB）而不是 TF 卡启动 | 明显 |

**目标：把"合开关 → 程序就绪"压到 10s 以内，并且在就绪后立刻可用。**

### 9.5 兜底：如果赛区坚持 A 解读

只能换视觉平台或做非常规设计（例如让 MaixCAM2 走独立常电，只用开关控制云台与激光）。**但注意这可能被判违规。** 因此**第一条行动项（向赛区确认）是最重要的**，不要拖到最后一天。

---


## 10. 完整代码骨架

```python
# aiming_main.py  —  MaixCAM2 · 视觉 + ZDT 云台闭环
import math, struct, time as pytime
import numpy as np
import cv2
from maix import camera, display, image, app, time, uart, pinmap, err, sys

# ============================================================
# 0. 配置（TODO: 全部现场标定后写死）
# ============================================================
W, H = 1280, 720
A4_W_CM, A4_H_CM = 21.0, 29.7
TARGET_CM = (A4_W_CM / 2, A4_H_CM / 2)      # 靶心 (10.5, 14.85)
R_CIRCLE_CM = 6.0

EXPOSURE, GAIN = 6000, 40                    # TODO 标定
WB_GAIN = [1.5, 1.0, 1.8]                    # TODO 标定
TH_LASER = [[70, 100, -45, 25, -128, -30],   # TODO 紫蓝主体
            [88, 100, -20, 20,  -60,  -5]]   # TODO 过曝白心
TH_BLACK = [[0, 45, -25, 25, -25, 25]]       # TODO

# 云台 PID（图像空间，像素误差 → RPM）
KP_X, KI_X, KD_X = 0.06, 0.004, 0.010
KP_Y, KI_Y, KD_Y = 0.06, 0.004, 0.010
I_LIMIT      = 300.0
DEADZONE_PX  = 2.5
RPM_MAX      = 45.0
SIGN_PAN, SIGN_TILT = +1, +1                 # TODO 实测方向后确定
KFF_YAW      = 0.0                           # TODO 前馈系数，从 0 开始加

DETECT_TARGET_EVERY = 2
MAX_LOST_FRAMES     = 5
MIN_QUAD_AREA_RATIO = 0.01

MODE_IDLE, MODE_AIM, MODE_CIRCLE, MODE_SCAN = 0, 1, 2, 3

# ============================================================
# 1. 几何工具（与视觉方案文档一致）
# ============================================================
def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def dist(a, b):
    return float(np.hypot(a[0]-b[0], a[1]-b[1]))

def order_corners(pts):
    p = np.array(pts, dtype=np.float32).reshape(4, 2)
    s = p.sum(axis=1); d = p[:, 0] - p[:, 1]
    return np.array([p[np.argmin(s)], p[np.argmax(d)],
                     p[np.argmax(s)], p[np.argmin(d)]], dtype=np.float32)

def is_convex(q):
    sg = [cross(q[i], q[(i+1) % 4], q[(i+2) % 4]) for i in range(4)]
    return all(v > 0 for v in sg) or all(v < 0 for v in sg)

def quad_area(q):
    return abs(sum(cross(q[i], q[(i+1) % 4], q[(i+2) % 4]) for i in range(4))) / 2.0

def is_valid_quad(q, img):
    if quad_area(q) < MIN_QUAD_AREA_RATIO * img.width() * img.height():
        return False
    if not is_convex(q):
        return False
    w1, w2 = dist(q[0], q[1]), dist(q[3], q[2])
    h1, h2 = dist(q[1], q[2]), dist(q[0], q[3])
    for a, b in ((w1, w2), (h1, h2)):
        if min(a, b) < 1e-3 or max(a, b) / min(a, b) > 2.5:
            return False
    ar = ((w1 + w2) / 2) / max(1e-3, (h1 + h2) / 2)
    return 0.45 <= ar <= 1.6

def build_homography(quad_px):
    src = np.float32([[0, 0], [A4_W_CM, 0], [A4_W_CM, A4_H_CM], [0, A4_H_CM]])
    return cv2.getPerspectiveTransform(src, np.float32(quad_px))

def project(Hm, x_cm, y_cm):
    v = Hm @ np.array([x_cm, y_cm, 1.0])
    return (None if abs(v[2]) < 1e-9 else (float(v[0]/v[2]), float(v[1]/v[2])))

def px_per_cm(Hm, x_cm, y_cm):
    p0 = project(Hm, x_cm, y_cm)
    px = project(Hm, x_cm + 1.0, y_cm)
    py = project(Hm, x_cm, y_cm + 1.0)
    return 0.0 if not (p0 and px and py) else float((dist(p0, px) + dist(p0, py)) / 2.0)

def quad_roi(q, img, pad=40):
    xs = [p[0] for p in q]; ys = [p[1] for p in q]
    x0 = max(0, int(min(xs)) - pad); y0 = max(0, int(min(ys)) - pad)
    x1 = min(img.width(), int(max(xs)) + pad); y1 = min(img.height(), int(max(ys)) + pad)
    return [x0, y0, max(1, x1-x0), max(1, y1-y0)]

# ============================================================
# 2. ZDT 驱动（张大头 Emm42_V5.0）
# ============================================================
class ZDTMotor:
    """地址 1~255；校验固定 0x6B；多字节大端（以官方手册为准）"""
    F_ENABLE, F_SPEED, F_POS, F_STOP, F_SYNC = 0xF3, 0xF6, 0xFD, 0xFE, 0xFF
    CS = 0x6B

    def __init__(self, ser, addr: int, name: str = ""):
        self.ser, self.addr, self.name = ser, addr & 0xFF, name
        self.last_target_pulses = 0

    def _send(self, data: bytes):
        self.ser.write(data)

    def enable(self, on: bool = True, store: bool = False):
        self._send(bytes([self.addr, self.F_ENABLE, 0xAB,
                          0x01 if on else 0x00, 0x01 if store else 0x00, self.CS]))

    def speed(self, rpm: float, acc: int = 0x10):
        """rpm 可为负（负号自动切换 dir）"""
        d = 0x00 if rpm >= 0 else 0x01
        v = int(min(5000, abs(rpm)))
        self._send(bytes([self.addr, self.F_SPEED, d, (v >> 8) & 0xFF, v & 0xFF,
                          acc & 0xFF, 0x00, self.CS]))

    def stop_speed(self):
        self.speed(0.0, acc=0)

    def move_abs(self, pulses: int, rpm: float = 120.0, acc: int = 0x80, absolute: bool = True):
        """位置模式：4 字节脉冲（大端）"""
        d = 0x00 if rpm >= 0 else 0x01
        v = int(min(5000, abs(rpm)))
        p = int(pulses) & 0xFFFFFFFF
        self._send(bytes([self.addr, self.F_POS, d, (v >> 8) & 0xFF, v & 0xFF,
                          acc & 0xFF,
                          (p >> 24) & 0xFF, (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF,
                          0x01 if absolute else 0x00,   # mode
                          0x00,                          # sync: 独立运行
                          self.CS]))
        self.last_target_pulses = p

    def move_abs_deg(self, deg: float, mstep: int = 16, motor_deg: float = 1.8,
                     gear: float = 1.0, rpm: float = 120.0, acc: int = 0x80):
        """按角度定位。gear 为减速比（直驱填 1.0）"""
        pulses_per_rev = (360.0 / motor_deg) * mstep * gear
        self.move_abs(int(round(deg / 360.0 * pulses_per_rev)), rpm, acc, absolute=True)

    def emergency_stop(self):
        self._send(bytes([self.addr, self.F_STOP, 0x98, 0x00, self.CS]))

# ============================================================
# 3. 图像空间 PID（含 anti-windup / 死区 / 丢帧保持）
# ============================================================
class ImgPID:
    def __init__(self, kp, ki, kd, i_limit, deadzone, out_max, sign=1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.deadzone, self.out_max, self.sign = i_limit, deadzone, out_max, sign
        self.integral, self.prev_e, self.prev_out = 0.0, 0.0, 0.0

    def reset(self):
        self.integral, self.prev_e, self.prev_out = 0.0, 0.0, 0.0

    def step(self, e, dt, valid=True):
        if not valid:                       # 丢帧：积分清零，保持上次输出
            self.integral = 0.0
            self.prev_e = 0.0
            return self.prev_out
        if abs(e) < self.deadzone:
            e = 0.0
        self.integral += e * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        de = (e - self.prev_e) / dt if dt > 1e-6 else 0.0
        u = self.kp * e + self.ki * self.integral + self.kd * de
        u = max(-self.out_max, min(self.out_max, u))
        self.prev_e, self.prev_out = e, u
        return self.sign * u

# ============================================================
# 4. 主程序
# ============================================================
def main():
    if sys.device_id() != "maixcam2":
        raise RuntimeError("本方案针对 MaixCAM2")

    # --- 串口 ---
    err.check_raise(pinmap.set_pin_function("A21", "UART4_TX"), "pinmap A21")
    err.check_raise(pinmap.set_pin_function("A22", "UART4_RX"), "pinmap A22")
    err.check_raise(pinmap.set_pin_function("B0",  "UART2_TX"), "pinmap B0")
    err.check_raise(pinmap.set_pin_function("B1",  "UART2_RX"), "pinmap B1")
    ser_pan  = uart.UART("/dev/ttyS4", 115200)
    ser_tilt = uart.UART("/dev/ttyS2", 115200)
    ser_mcu  = uart.UART("/dev/ttyS1", 115200)     # 需按 pinmap 分配引脚

    pan  = ZDTMotor(ser_pan,  1, "pan")
    tilt = ZDTMotor(ser_tilt, 2, "tilt")
    pan.enable(True); tilt.enable(True)
    tilt.move_abs_deg(0.0)          # TODO 俯仰预置角，按实际安装确定

    # --- 相机 ---
    cam = camera.Camera(W, H, image.Format.FMT_RGB888, fps=30)
    disp = display.Display()
    cam.skip_frames(30)
    cam.exp_mode(camera.AeMode.Manual)
    cam.exposure(EXPOSURE)
    cam.gain(GAIN)
    cam.awb_mode(camera.AwbMode.Manual)
    cam.set_wb_gain(WB_GAIN)

    pid_x = ImgPID(KP_X, KI_X, KD_X, I_LIMIT, DEADZONE_PX, RPM_MAX, SIGN_PAN)
    pid_y = ImgPID(KP_Y, KI_Y, KD_Y, I_LIMIT, DEADZONE_PX, RPM_MAX, SIGN_TILT)

    Hmat, last_quad, frame_id, lost = None, None, 0, 0
    laser_prev = None
    lap_progress = 0.0
    mode = MODE_AIM
    t_prev = time.time()
    mcu_buf = b""

    while not app.need_exit():
        t_now = time.time()
        dt = max(1e-3, t_now - t_prev)
        t_prev = t_now
        img = cam.read()

        # ---------- 4.1 读取 MSPM0 上报（非阻塞） ----------
        data = ser_mcu.read()
        if data:
            mcu_buf = (mcu_buf + data)[-64:]
            parsed = parse_mspm0_frame(mcu_buf)
            if parsed:
                _, lap_progress, _, _ = parsed
                mcu_buf = b""

        # ---------- 4.2 靶纸检测（降频） ----------
        if frame_id % DETECT_TARGET_EVERY == 0 or Hmat is None:
            q = detect_target(img, last_quad)      # 见视觉方案文档的三级检测
            if q is not None:
                last_quad = q
                Hmat = build_homography(q)
                lost = 0
            else:
                lost += 1

        target_ok = (Hmat is not None) and (lost < MAX_LOST_FRAMES)

        # ---------- 4.3 目标点生成（打靶心 / 画圆共用） ----------
        T_px = None
        if target_ok:
            if mode == MODE_CIRCLE:
                th = 2.0 * math.pi * lap_progress
                T_cm = (TARGET_CM[0] + R_CIRCLE_CM * math.cos(th),
                        TARGET_CM[1] + R_CIRCLE_CM * math.sin(th))
            else:
                T_cm = TARGET_CM
            T_px = project(Hmat, T_cm[0], T_cm[1])

        # ---------- 4.4 激光点检测 ----------
        L_px = None
        if target_ok and last_quad is not None:
            roi = quad_roi(last_quad, img, pad=60)
            r = detect_laser(img, roi, laser_prev)     # 见视觉方案文档
            if r:
                L_px, laser_prev = (r[0], r[1]), (r[0], r[1])
            else:
                laser_prev = None

        # ---------- 4.5 云台闭环：e = T_px - L_px ----------
        both_ok = (T_px is not None) and (L_px is not None)
        if both_ok:
            rpm_pan  = pid_x.step(T_px[0] - L_px[0], dt, True)
            rpm_tilt = pid_y.step(T_px[1] - L_px[1], dt, True)
        else:
            rpm_pan  = pid_x.step(0.0, dt, False)      # 丢帧：保持上次输出
            rpm_tilt = pid_y.step(0.0, dt, False)
        pan.speed(rpm_pan)
        tilt.speed(rpm_tilt)

        # ---------- 4.6 误差（cm），可直接预判 D1 / D2 ----------
        s = px_per_cm(Hmat, TARGET_CM[0], TARGET_CM[1]) if target_ok else 0.0
        err_cm = ((L_px[0]-T_px[0])/s, (L_px[1]-T_px[1])/s) if (both_ok and s > 0) else (0.0, 0.0)

        # ---------- 4.7 调试可视化 ----------
        if last_quad is not None:
            for i in range(4):
                a_, b_ = last_quad[i], last_quad[(i+1) % 4]
                img.draw_line(int(a_[0]), int(a_[1]), int(b_[0]), int(b_[1]),
                              image.COLOR_GREEN, 2)
        if T_px:
            img.draw_cross(int(T_px[0]), int(T_px[1]), image.COLOR_RED, 8, 2)
        if L_px:
            img.draw_cross(int(L_px[0]), int(L_px[1]), image.COLOR_YELLOW, 8, 2)
        img.draw_string(8, 8,
            f"mode={mode} s={s:.2f}px/cm err=({err_cm[0]:+.2f},{err_cm[1]:+.2f})cm "
            f"lost={lost} prog={lap_progress:.2f}",
            image.COLOR_WHITE, 1.4)
        disp.show(img)

        frame_id += 1

    pan.emergency_stop(); tilt.emergency_stop()
    pan.enable(False); tilt.enable(False)
    cam.close()

if __name__ == "__main__":
    main()
```

> `detect_target()` / `detect_laser()` / `parse_mspm0_frame()` 见《E题视觉方案_MaixCAM2.md》与本文档第 8.4 节。

---

## 11. 整定与调试清单

### 11.1 分步联调顺序（不要跳步）

| 步骤 | 目标 | 通过标准 |
|---|---|---|
| 1 | ZDT 使能 | 发 `01 F3 AB 01 00 6B`，电机轻微锁死，屏幕显示使能状态 |
| 2 | 速度模式单轴测试 | 发 `01 F6 00 00 64 0A 00 6B`（100RPM 正转），电机会转；反方向同理 |
| 3 | 确认方向 | 手动给小转速，看激光点在图像里往哪跑，定 `SIGN_PAN/SIGN_TILT` |
| 4 | 位置模式测试 | 发 90° 定位，用量角器/目视确认角度正确 |
| 5 | 视觉单独跑 | 不接云台，屏幕上看靶纸框和激光十字是否稳定 |
| 6 | 静态锁定 | 靶纸固定，看云台能否把激光点压到靶心；先给很小的 KP |
| 7 | 调 PID | KP 由小到大到临界振荡，退一半；再加 KD 压振；KI 最后加一点点 |
| 8 | 死区 | 观察稳定后的抖动幅度，设 DEADZONE_PX 略大于它 |
| 9 | 扫描 | 测试 8 步扫描能否在 2s 内命中 |
| 10 | 动态跟踪 | 小车跑圈，观察是否连续锁定 |
| 11 | 画圆 | 接上圈进度，检查同步误差 |

### 11.2 PID 整定要点

- **先单轴**。Pan 调好了再调 Tilt，不要同时调。
- **KP 起点要小**。建议从 `0.02` 开始，逐步加到出现小幅振荡，然后退到 60% 作为工作值。
- **云台是二阶系统**（惯量 + 电机），**KD 很有用**，能压住超调和振荡。
- **KI 通常不需要**（图像伺服几乎没有稳态误差）。如果重载下压不下去再加一点点，并务必做限幅。
- **积分项在丢帧时必须清零**，否则目标重新出现时会猛冲。
- **前馈系数从 0 开始**。

### 11.3 RPM_MAX 的经验值

直驱情况下，图像上 100px 的误差只需云台转 `100/1108 rad ≈ 5.2°`。若要 0.15s 消除，需要 34.5°/s ≈ **5.8 RPM**。

所以：

```
RPM_MAX = 30 ~ 60 就足够
```

**不要**设成几百上千——那必然过冲振荡，还会让画面严重拖影导致检测失败。

---

## 12. 坑清单

| # | 坑 | 现象 | 修法 |
|---|---|---|---|
| 1 | **ZDT 的 `P_Serial` 没设成 `UART_FUN`** | 串口发了没反应 | 菜单里改，这是最容易漏的一步 |
| 2 | **两台电机地址相同** | 只动一台 / 乱动 | `ID_Addr` 分别设 1 和 2 |
| 3 | **MaixCAM2 与 ZDT 没有共地** | 通信时好时坏或完全不通 | 必须接 GND |
| 4 | **占用 UART0** | 系统启动异常/日志乱入 | 用 UART1~UART4，UART0 留给系统 |
| 5 | **用图像中心做闭环** | 激光永远偏靶心上/下方 | 必须用 `e = T_px − L_px`（第 2.3 节） |
| 6 | **PID 输出符号搞反** | 云台朝着远离目标的方向跑，误差越来越大 | 手动测试确定 `SIGN` |
| 7 | **RPM_MAX 设太大** | 云台猛冲、过冲、画面拖影 | 30~60 足够 |
| 8 | **积分不清零** | 目标丢失后重新出现，云台猛冲一下 | 丢帧时 `integral = 0` |
| 9 | **没设死区** | 稳定后云台一直小幅抖动 | `DEADZONE_PX` 设 2~3 |
| 10 | **`move_abs_deg` 的细分/减速比填错** | 扫到的角度不对，靶纸总在视野边缘 | 核对 `MStep` 和实际减速比 |
| 11 | **MaixCAM2 开机太慢，2s 来不及** | 基本(2) 直接 0 分 | 见第 9 节，自启 + 优化 + 赛前确认 |
| 12 | **用时间推进画圆进度** | 同步误差超 1/4 圈 | 必须用 MSPM0 编码器里程 |
| 13 | **系统自启和 MaixVision 抢摄像头** | MaixVision 连不上/程序跑不起来 | 开发期别用 systemd 自启 |
| 14 | **激光继电器与大电流电机共电源** | 云台一动画面就抖/复位 | 电机独立供电 + 大电容 + 磁珠 |
| 15 | **整车高度超 25cm** | 不予测试 | 相机+云台+激光笔总高要卡进 25cm |

---

## 13. 供电与接地

| 项 | 建议 |
|---|---|
| 电池 | 2S/3S 锂电（7.4V/11.1V）或 12V 锂电池 |
| 开关1（小车） | MSPM0 天猛星 + 电机驱动 + 巡迹传感器 |
| 开关2（瞄准） | MaixCAM2 + 2×ZDT 驱动器 + 激光笔 |
| ZDT 供电 | 直接吃 12V（驱动器范围 7~32V），**每台驱动器 V+ 并 470~1000µF 电解电容** |
| MaixCAM2 供电 | 5V（Type-C 或扩展口），**单独一路 LDO/DCDC，不要和电机共线** |
| 激光笔 | 5V 经继电器/MOS 控制，与相机同轴固定 |
| **共地** | **MaixCAM2、ZDT、MSPM0 三者的 GND 必须连在一起**（UART 通信前提） |
| 抗扰 | 电机电源线与信号线分开走；信号线用屏蔽或双绞；ZDT 电源入口加磁珠 |
| 电流余量 | 两台 ZDT 峰值可到 6A，电池与线径要留够 |

> ⚠️ 常见故障："云台一动，MaixCAM2 就重启"。原因几乎都是**电机大电流把 5V 电源拉垮**。对策：电机与 MaixCAM2 完全独立供电 + 大电容 + 单点接地。

---

## 附录：本文档新引用的接口（已核对）

```python
# 引脚复用
from maix import pinmap, err
pinmap.set_pin_function("A21", "UART4_TX")     # 返回 Err，用 err.check_raise 检查
uart.list_devices()                             # 列出串口设备

# 姿态（MaixCAM2 板载 QMI8658）
from maix import ahrs, tensor
# ahrs.AHRS(kp, ki)   .init(ax,ay,az,mx=0,my=0,mz=0)
#   .update(ax,ay,az,gx,gy,gz,mx,my,mz,dt)
#   .get_angle(acc, gyro, mag, dt, radian=False) -> 姿态角
#   .reset()

# 高层通信协议
from maix import comm
# comm.CommProtocol(buff_size=1024, header=3148663466, method_none_raise=False)
#   .report(cmd, body)  .get_msg(timeout=0)  .resp_ok(cmd, body)  .resp_err(cmd, code, msg)
#   .valid()  .set_method(method)  .get_method()  .get_uart_port()  .get_uart_ports()

# 多目标跟踪（可选，用于提高动态瞄准的稳定性）
from maix import tracker
# tracker.Tracker(max_lost_buff_num=60, track_thresh=0.5, high_thresh=0.6,
#                 match_thresh=0.8, max_history=20)
#   .update(objs)   # objs: list[TrackerObject(x,y,w,h,class_id,score)]

# ZDT Emm42_V5.0 常用帧（地址记为 A，校验 0x6B）
#   使能      : A    F3 AB 01 00 6B
#   失能      : A    F3 AB 00 00 6B
#   速度模式  : A    F6 dir spdH spdL acc sync 6B          (8 B)
#   位置模式  : A    FD dir spdH spdL acc p3 p2 p1 p0 mode sync 6B   (13 B)
#   立即停止  : A    FE 98 00 6B
#   同步运动  : A    FF 66 6B
#   读转速    : A    35 6B        → 返回 A 35 dir spdH spdL 6B
#   读位置    : A    36 6B        → 返回 A 36 p3 p2 p1 p0 6B
#   清零位置  : A    0A 6B
#   到位回传  : A    FD 9F 6B     （需菜单 Response = Reached/Both）
```
