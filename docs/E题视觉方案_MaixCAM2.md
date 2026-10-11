# 2025 电赛 E 题 · 视觉方案详细设计（MaixCAM2 / MaixPy v4）

> 目标：用一块 MaixCAM2 完成「靶纸矩形追踪 + 靶心定位 + 激光光斑识别 + 靶面坐标解算 + 画圆目标点生成」，输出给云台控制侧闭环。
> 本文档所有 API 签名均按 MaixPy v4 官方 API 文档核对过，可直接照着写代码。

---

## 0. 方案结论（先看这段）

| 问题 | 结论 |
|---|---|
| 要不要上 YOLO / 深度学习？ | **不要**。靶纸是已知形状、已知尺寸、已知颜色的强结构目标，传统 CV 更快、更可控、更好标定。MaixCAM2 的 3.2TOPS@INT8 留着做兜底（万一现场光照极端）而不是主路径。 |
| 靶心怎么找？ | **不能靠找红点**。靶心红点直径 ≤0.1cm，在图像上不到 1 个像素。**唯一可行方案是用靶纸黑色外框的 4 个角点做单应变换，把靶面物理坐标 (10.5, 14.85) cm 投影成像素坐标。** |
| 激光点怎么找？ | LAB 阈值二值化 + 形态学 + `find_blobs`，再用距离变换求亚像素质心。关键是**锁定相机曝光与白平衡**。 |
| 输出的核心量是什么？ | **一个"目标像素点" `T_px`**，而不是"误差"。这样相机装在云台上 / 固定在车体上两种装法都能用同一套视觉输出。 |
| 精度瓶颈在哪？ | **不在视觉**。视觉定位重复性可以做到 <0.3px；真正的误差来自「相机光轴–激光光轴」的机械同轴标定。标定花的每一分钟都比调算法值钱。 |

---

## 1. 硬件与运行环境确认

### 1.1 MaixCAM2 规格（与本方案相关的部分）

| 项目 | 规格 | 对本方案的意义 |
|---|---|---|
| SoC | AX630C：2×Cortex-A53 @1.2GHz（跑 Ubuntu Linux）+ 1×RISC-V E907（跑 RTOS） | A53 上跑 numpy/OpenCV 完全够用；Linux 环境可直接 `pip install opencv-python` |
| NPU | 12.8 TOPS @INT4 / 3.2 TOPS @INT8 | 本方案用不到，留作兜底 |
| 内存 | 1GB / 4GB LPDDR4 | 1GB 版本足够，但 4K 图 + numpy 副本时注意内存 |
| 摄像头 | 最高 8M(4K)@30fps，4-lane MIPI CSI，22Pin，支持双路 CSI | **高分辨率是这个题的关键红利**，见第 2 节精度预算 |
| ISP | **带 AI ISP** | 暗光/高动态下噪点少，激光点信噪比更好 |
| 屏幕 | 2.4" 640×480 IPS 电容触摸 | 现场调试用，可做阈值滑块 |
| IMU | **板载六轴（三轴加速度 + 三轴角速度）** | 动态瞄准前馈补偿可用，时间戳与图像对齐 |
| 接口 | 2.54mm PMOD 引出 20 IO + 1.25mm 6pin；UART/SPI/I2C/PWM/ADC | 串口给主控；若云台由 MaixCAM2 直接驱动则用 PWM |
| 尺寸 | 66.5 × 50 × 21.2 mm，1/4" 标准螺纹孔 | 装车体上直接上三脚架螺丝孔 |

### 1.2 软件栈

- **MaixPy v4（Python）**：主开发方式，本文档全部按此写。
- **MaixVision IDE**：PC 端连板子预览、看帧率、调阈值、保存图片。**赛前必备**。
- 可选：`import cv2`（Linux 环境装 opencv-python），用于单应矩阵求解、距离变换、形态学。

> ⚠️ 一个必须知道的坑：MaixPy 的 `img.binary()` / `img.erode()` / `img.dilate()` / `img.open()` / `img.close()` **默认是原地（in-place）操作**，会直接改掉原图。要保留原图必须先用 `img.copy()`。

---

## 2. 需求 → 视觉指标拆解

### 2.1 题目对视觉的硬性要求

| 题目条目 | 视觉需要提供的能力 | 时限 | 精度 |
|---|---|---|---|
| 基本 (2) | 任意位姿下 **2s 内**找到靶心并锁定 | 2s | D₁ ≤ 2cm |
| 基本 (3) | 指定点、任意朝向下 **4s 内**找到靶心 | 4s | D₁ ≤ 2cm |
| 发挥 (1)(2) | 小车运动全程 **连续**输出靶心坐标 | 20s/圈，2 圈 | D₁ ≤ 2cm |
| 发挥 (3) | 输出靶面 6cm 圆上的目标点，与小车圈进度同步 | 20s | D₂ ≤ 2cm，同步误差 < 1/4 圈 |

### 2.2 几何条件

场地：轨道外沿 100cm × 100cm 正方形，A/B/C/D 为四个外顶点（逆时针）。
靶：距 AB 线段外 50cm 竖立，靶面与 AB 平行，靶高 ≤50cm。

按 A=(0,0) B=(100,0) C=(100,100) D=(0,100) 建立场地坐标，靶心大致在 (50, −50)：

| 小车位置 | 到靶心水平距离 |
|---|---|
| B 点 | 50 cm（最近） |
| A 点 / 中点 | 70.7 cm |
| C / D 点 | **158 cm（最远）** |

**结论：相机到靶面的工作距离范围约 50 ~ 165 cm（含高度差）。**

### 2.3 分辨率 / 精度预算（关键！）

靶纸为 A4 竖放：**21.0cm（宽）× 29.7cm（高）**。黑色胶带描出的是这 21×29.7cm 的外轮廓（见 5.1 节的说明）。

设相机水平 FOV = 60°（M12 标准镜头典型值，**务必用你自己的镜头实测标定**）：

| 距离 d | 水平视场宽 = 2d·tan30° | 640px 出图 | 1280px 出图 | 0.5cm 激光斑 | A4 宽 21cm 占屏 |
|---|---|---|---|---|---|
| 50 cm | 57.7 cm | 11.1 px/cm | 22.2 px/cm | 5.5 / 11.1 px | 233 / 466 px |
| 100 cm | 115.5 cm | 5.5 px/cm | 11.1 px/cm | 2.8 / 5.5 px | 116 / 233 px |
| **165 cm** | 190.5 cm | **3.36 px/cm** | **6.72 px/cm** | **1.7 / 3.4 px** | **70 / 141 px** |

**读表结论：**

1. **远距离（C/D 点）必须用 ≥1280 宽**。640×480 下光斑只有 1.7px，`pixels_threshold` 默认 10 根本检不出来。
2. 即使用 1280，远距离光斑也只有 3.4px —— **光斑检测必须配 `dilate` 撑大 + 调低 `pixels_threshold`**，这是刚需不是优化。
3. A4 靶纸在 165cm/1280px 下只占 141px 宽 → **靶纸 4 个角点的定位精度直接决定了整个系统的精度上限**。角点必须做到 ±1px 以内。

### 2.4 一个提高分辨率的杀招：`set_windowing`

MaixCAM2 支持**传感器窗口裁剪**：

```python
cam.set_windowing([x, y, w, h])   # 在 sensor 读出阶段裁剪
cam.get_sensor_size()             # 查传感器原始分辨率
```

裁剪后传感器只读出中心一块，输出同样大小的图，**等效焦距变长，px/cm 成比例提升，且没有插值损失**（不是数码变焦放大）。

用法：远距离（靶纸在画面中太小）时，动态把窗口收窄到靶纸附近；近距离时放开。**代价是视野变窄，需要视觉侧同步更新 ROI 映射关系，否则坐标会错位——用之前必须先搞清 windowing 与输出图坐标的对应关系（建议先写个测试脚本用已知位置的光斑验证）。**

---

## 3. 三条决定成败的工程结论

### 结论一：靶心只能靠几何反算，不能靠颜色识别

题目靶面构成：
- 1.8cm 宽**黑色胶带**沿四周贴出长方形 → 勾勒靶纸外轮廓
- **红色油性记号笔**在中心标一个点作为靶心，**直径 ≤0.1cm**
- 以靶心为圆心画 2 / 4 / 6 / 8 / 10cm 半径的红色圆，**圆弧线宽 ≤0.1cm**

在 165cm / 1280px（6.72 px/cm）下：
- 靶心红点直径 0.1cm → **0.67 像素**（不可见）
- 红圆线宽 0.1cm → 0.67 像素（r=6cm 的圆周长约 37.7cm ≈ 253px 弧长，勉强能看出淡淡的弧，但不能作为主定位依据）

**所以正确做法是：**

```
检测黑胶带矩形的 4 个角点
        ↓
已知该矩形 = A4 外轮廓 = 21.0cm × 29.7cm
        ↓
用 4 组对应点求单应矩阵 H（靶面 cm 坐标 → 图像像素）
        ↓
靶心像素 = H · (10.5, 14.85, 1)
```

这套方法对靶纸在图像中的**倾斜、旋转、缩放全部免疫**，而且天然给出了 px/cm 尺度，画圆部分直接复用。

### 结论二：输出的核心是"目标像素点"，不是"误差"

视觉模块每帧只输出 4 个量：

| 输出 | 含义 |
|---|---|
| `T_px = (tx, ty)` | **靶面目标点在图像中的像素坐标**。打靶心时 = 靶心投影；画圆时 = 6cm 圆周点投影 |
| `L_px = (lx, ly)` | 激光光斑在图像中的像素坐标（未检出时置无效标志） |
| `s` | 当前靶面尺度 px/cm（由 H 局部求导得到，用于换算 cm 误差、判断 D₁/D₂） |
| `q` | 质量标志位（靶纸有效 / 激光有效 / 跟踪锁定状态 / 置信度） |

**已确定的装法：相机装在云台上（Tilt 轴），激光笔刚性固定在相机顶部。**

这个装法有一个必须先想清楚的几何后果：**激光笔在相机光心上方 h，所以激光点在图像中不在中心，且偏移量随距离变化**：

```
Δy_px = − h · f_px / d        f_px = W / (2·tan(HFOV/2))
```

取 W = 1280、HFOV = 60° → `f_px ≈ 1108 px`：

| 距离 d | h = 3cm | h = 5cm |
|---|---|---|
| 50 cm | 66 px | 111 px |
| 100 cm | 33 px | 55 px |
| 165 cm | 20 px | 34 px |

所以**绝不能用"图像中心闭环"**。正确解是：

- 误差 `e = T_px − L_px`（靶面目标点像素 − 激光点像素）

这个式子是**严格正确**的：激光点是靶面上的真实物理点，而 H 是"靶面 cm ↔ 图像 px"的双射，所以 `L_cm = H⁻¹(L_px)` 就是激光的真实落点；要求 `L_cm = T_cm` 等价于 `L_px = T_px`。**视差与 h、f、d 全部被吸收。**

直接推论：**机械装配不需要精确对齐激光与相机光轴**，只要激光点在相机视野内即可——这能省掉赛场上好几个小时的机械调试。

（完整的视差推导、云台闭环与 ZDT 控制见《E题瞄准模块控制方案.md》第 2 节）

### 结论三：锁定曝光与白平衡，比调阈值重要一百倍

激光笔是极亮的小点光源。相机的自动曝光（AE）和自动白平衡（AWB）会因为激光出现而**反复调整整幅画面的亮度和色温** → 你辛苦标定的 LAB 阈值会周期性失效，表现为"时好时坏"。

**必须在程序开头就锁死：**

```python
cam.exp_mode(camera.AeMode.Manual)      # 曝光模式：手动
cam.exposure(6000)                      # 曝光值：现场标定
cam.gain(40)                            # 模拟增益
cam.awb_mode(camera.AwbMode.Manual)     # 白平衡：手动
cam.set_wb_gain([1.5, 1.0, 1.8])        # R/G/B 增益，现场标定
# 可选：cam.luma(...)、cam.constrast(...)、cam.saturation(...)
```

标定方法：在**只开普通照明、激光笔关闭**的情况下，调整 exposure/gain 使靶纸在预览里亮度适中（不欠曝不过曝）；然后开激光，确认光斑仍能饱和（B 通道接近 255）而画面其余部分亮度不变。手动模式下 AE/AWB 不再自动漂移。

---

## 4. 总体流水线

```
┌─────────────────────────────────────────────────────────────┐
│  相机读取 │  cam.read()  →  img (1280×720 RGB888)            │
└──────────────────────────┬──────────────────────────────────┘
                           │
        ┌──────────────────┴──────────────────┐
        │                                     │
        ▼  (10~15 Hz，可跳帧)                  ▼  (每帧 30+ Hz)
┌───────────────────────────┐      ┌────────────────────────────┐
│  A. 靶纸检测               │      │  B. 激光光斑检测            │
│  ① find_rects (主)         │      │  ① ROI 限定在靶纸附近       │
│  ② 黑阈值 blob (兜底)      │      │  ② LAB 阈值二值化           │
│  ③ 上帧位姿外推 (兜底)     │      │  ③ 形态学 open + dilate     │
│  → 4 角点 → 角点排序        │      │  ④ find_blobs 筛选打分      │
└───────────┬───────────────┘      │  ⑤ 距离变换求亚像素质心      │
            │                      └────────────┬───────────────┘
            ▼                                   │
┌───────────────────────────┐                   │
│  C. 单应矩阵 H             │                   │
│  靶面 cm ↔ 图像 px         │                   │
│  靶心 / 6cm 圆点 → T_px    │                   │
└───────────┬───────────────┘                   │
            │                                   │
            └──────────────┬────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  D. 时序滤波与失效保护                                       │
│  门控 → One Euro Filter → 丢帧外推 → 置信度                  │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  E. 输出协议  UART  921600                                  │
│  T_px, L_px, px_per_cm, quality, flags                      │
└─────────────────────────────────────────────────────────────┘
```

**频率分配的核心思想**：靶纸检测（`find_rects`）比较贵，做 10~15Hz 就够，中间帧用 `H` 的平滑外推；激光点检测便宜，每帧都做。

---

## 5. 模块一：靶纸矩形检测与单应矩阵

### 5.1 先搞清楚"检测的是哪条边"

黑胶带是 1.8cm 宽的带子贴着 A4 边缘粘的。所以视觉上有**两圈**矩形边：
- **外沿** ≈ A4 外轮廓 ≈ 21.0cm × 29.7cm
- **内沿** ≈ 21.0−2×1.8 = 17.4cm × 29.7−2×1.8 = 26.1cm

`find_rects` 返回哪一圈是不确定的，会出现尺度跳变（一次框外沿、一次框内沿 → 靶心位置抖动）。

**两个对策（都要做）：**

1. **用几何合理性筛掉混淆项**：先算矩形面积，取**面积更大**的那个（外沿）。同时记录尺度 `s` 做跳变检测——`s` 突然变化超过 1.3 倍就判定为"换边了"，丢帧重检。
2. **靶心不用猜，靠对称性**：无论框到外沿还是内沿，矩形中心就是靶纸中心。**靶心 = 矩形中心**（因为红点是标在靶纸几何中心的）。所以更稳的做法是用**矩形的中心 + 局部尺度**来算靶心，而不是严格用 21×29.7 的绝对尺寸去投影。

> 实操建议：把「靶心 = H·(10.5, 14.85)」和「靶心 = 4 角点对角线的交点」都算出来，跟踪阶段用后者（更稳），只在需要精确 px/cm 换算时用前者。两者差值可以作为一个自检指标（正常应 < 1cm）。

### 5.2 三级检测策略

#### 级别 1（主）：`find_rects`

```python
rects = img.find_rects(roi=[], threshold=10000)
```

- **签名**：`find_rects(self, roi: list[int] = [], threshold: int = 10000) -> list[Rect]`
- **原理**：基于图像梯度的四边形检测（与 AprilTag 的 quad detection 同源），**不依赖颜色阈值**，所以对靶纸上紫外感光痕迹（大片深色斑块）天然免疫。
- `threshold` 越大检测越严格（默认 10000）。靶纸对比度高，建议 8000~15000。 **数值越低越容易在杂乱背景里误检。**
- 返回的 `Rect` 对象提供：
  - `r.corners()` → `[[x,y], [x,y], [x,y], [x,y]]` 四个角点
  - `r.rect()` → `[x, y, w, h]` 外接框
  - `r.magnitude()` → 该四边形的边缘强度（**可作置信度打分**）
  - `r.x() / r.y() / r.w() / r.h()`

```python
def detect_by_find_rects(img, roi=[]):
    cands = []
    for r in img.find_rects(roi=roi, threshold=10000):
        cs = r.corners()
        if len(cs) != 4:
            continue
        quad = order_corners(cs)                       # 见 5.3
        if not is_valid_quad(quad, img):               # 见 5.4
            continue
        score = r.magnitude() * 0.6 + area_ratio(quad, img) * 10000 * 0.4
        cands.append((score, quad))
    if not cands:
        return None
    cands.sort(key=lambda t: -t[0])
    return cands[0][1]
```

#### 级别 2（兜底）：黑阈值二值化 + `find_blobs`

当 `find_rects` 因为**运动模糊**（动态瞄准时小车在动）或**反光**漏检时启用。

```python
TH_BLACK = [[0, 45, -25, 25, -25, 25]]   # L 低、A/B 接近中性 = 黑

def detect_by_black_blob(img, roi=[]):
    tmp = img.copy()                      # ⚠️ binary 是原地操作，必须 copy
    tmp.binary(TH_BLACK)                  # 原地二值化：命中阈值→白，其余→黑
    tmp.open(1)                           # 开运算（先腐蚀后膨胀）去掉孤立噪点
    tmp.close(2)                          # 闭运算（先膨胀后腐蚀）把断开的框补上
    # 二值化后是纯黑白，用白色 LAB 阈值找块
    blobs = tmp.find_blobs(
        [[80, 100, -128, 127, -128, 127]],
        roi=roi,
        area_threshold=200,
        pixels_threshold=200,
        merge=True,
        margin=8)
    if not blobs:
        return None
    b = max(blobs, key=lambda x: x.pixels())
    pts = b.mini_corners()                # ⚠️ 注意是 mini_corners()，不是 min_corners()
    if len(pts) != 4:
        return None
    return order_corners(pts)
```

> **注意拼写**：MaixPy v4 里是 `blob.mini_corners()`（mini，不是 min）。文档里写错的代码跑不起来。
>
> **注意**：`mini_corners()` 返回的是该 blob 的**最小面积矩形**角点。它假设目标是旋转矩形——对**倾斜（有透视）的靶纸不够准**，只在"靶纸基本正对相机、只是旋转了几度"时可用于兜底。真正有透视时优先用级别 1。

#### 级别 3（兜底）：上一帧位姿外推

已经锁定后，如果本帧两种方法都失败：
- 用上一帧的 4 角点 + 上一帧到本帧的时间差做线性外推
- 或者用上一帧 H 把靶面 4 个角投影出来，在当前帧对应位置取一个 **ROI**，只在这个 ROI 里重试级别 1（缩小搜索范围，提高速度与成功率）
- **重要**：外推超过 3~5 帧仍失败 → 判定丢靶，`quality=0`，通知控制侧停止闭环进入搜索模式

### 5.3 角点排序 —— 最容易出错的一步

`find_rects().corners()` 返回的 4 个点**顺序不保证**（不同帧可能从不同角开始）。必须重新排成 `[左上, 右上, 右下, 左下]`，否则单应矩阵会求出镜像/扭曲，靶心会跑到画面外。

```python
import numpy as np

def order_corners(pts):
    """排成 [左上, 右上, 右下, 左下]"""
    p = np.array(pts, dtype=np.float32).reshape(4, 2)
    s = p.sum(axis=1)          # x + y：左上最小，右下最大
    d = p[:, 0] - p[:, 1]      # x - y：右上最大，左下最小
    tl = p[np.argmin(s)]
    br = p[np.argmax(s)]
    tr = p[np.argmax(d)]
    bl = p[np.argmin(d)]
    return np.array([tl, tr, br, bl], dtype=np.float32)
```

**还要做凸性 / 朝向校验**，防止退化四边形：

```python
def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def is_convex_ccw(q):
    """图像坐标 y 轴向下，顺序正确时 4 个叉积应同号"""
    signs = []
    for i in range(4):
        signs.append(cross(q[i], q[(i+1) % 4], q[(i+2) % 4]))
    return all(v > 0 for v in signs) or all(v < 0 for v in signs)
```

如果 `is_convex_ccw` 为 False → 说明有自交或顺序错乱，直接丢弃这一帧的候选。

### 5.4 候选几何筛选

```python
A4_W_CM, A4_H_CM = 21.0, 29.7

def is_valid_quad(q, img):
    W, H = img.width(), img.height()
    # 1) 面积占比（靶纸占画面不能太小，否则角点精度不够）
    area = abs(sum(cross(q[i], q[(i+1) % 4], q[(i+2) % 4]) for i in range(4))) / 2.0
    if area < 0.01 * W * H:          # 至少占 1% 画面
        return False
    # 2) 凸四边形
    if not is_convex_ccw(q):
        return False
    # 3) 对边长度比（透视下会变，给宽范围）
    def dist(a, b):
        return float(np.hypot(a[0]-b[0], a[1]-b[1]))
    w1, w2 = dist(q[0], q[1]), dist(q[3], q[2])
    h1, h2 = dist(q[1], q[2]), dist(q[0], q[3])
    for a, b in ((w1, w2), (h1, h2)):
        if min(a, b) < 1e-3 or max(a, b) / min(a, b) > 2.5:
            return False
    # 4) 长宽比（A4 是 1:1.414；透视会偏移，给 0.45~1.6 的宽容区间）
    ar = ((w1 + w2) / 2) / max(1e-3, (h1 + h2) / 2)
    if not (0.45 <= ar <= 1.6):
        return False
    return True
```

### 5.5 单应矩阵构建与投影

```python
import cv2
import numpy as np

def build_homography(quad_px):
    """输入排好序的 [TL, TR, BR, BL] 像素坐标 → 返回 3×3 H（靶面 cm → 图像 px）"""
    src = np.float32([
        [0.0,        0.0],         # TL
        [A4_W_CM,    0.0],         # TR
        [A4_W_CM,    A4_H_CM],     # BR
        [0.0,        A4_H_CM],     # BL
    ])
    dst = np.float32(quad_px)
    # 恰好 4 点用 getPerspectiveTransform 比 findHomography 更快更确定
    return cv2.getPerspectiveTransform(src, dst)

def project(H, x_cm, y_cm):
    """靶面 cm 坐标 → 图像像素坐标"""
    v = H @ np.array([x_cm, y_cm, 1.0])
    if abs(v[2]) < 1e-9:
        return None
    return float(v[0] / v[2]), float(v[1] / v[2])

def px_per_cm(H, x_cm=10.5, y_cm=14.85):
    """在靶面某点处求局部尺度（px/cm）。单应不是等距变换，必须局部求导"""
    p0 = project(H, x_cm, y_cm)
    px = project(H, x_cm + 1.0, y_cm)
    py = project(H, x_cm, y_cm + 1.0)
    if not (p0 and px and py):
        return 0.0
    sx = np.hypot(px[0]-p0[0], px[1]-p0[1])
    sy = np.hypot(py[0]-p0[0], py[1]-p0[1])
    return float((sx + sy) / 2.0)
```

**靶面关键点（cm 坐标）速查：**

| 点 | cm 坐标 |
|---|---|
| 靶心 | (10.5, 14.85) |
| 6cm 圆周点（角度 θ） | (10.5 + 6·cosθ, 14.85 + 6·sinθ) |
| 2cm 圆周点 | (10.5 + 2·cosθ, 14.85 + 2·sinθ) |
| A4 四角 | (0,0) (21,0) (21,29.7) (0,29.7) |

### 5.6 靶纸检测的抗干扰要点

| 干扰源 | 表现 | 对策 |
|---|---|---|
| **紫外感光痕迹** | 靶面上大片深色斑块（20~60s 才消失） | 用 `find_rects`（梯度法，找结构而非颜色）；黑阈值兜底时用**周长/面积比**区分"细环"和"实心斑"（矩形环的 `perimeter²/area` 远大于实心块） |
| **镜头畸变** | 靶纸靠近画面边缘时直线变弯，H 不准 | 先 `img.lens_corr(strength)` 校正（`strength` 约为负的畸变系数，默认 1.8，需实测）；或把靶纸尽量放在画面中央 |
| **运动模糊** | 动态瞄准时矩形边缘糊掉，`find_rects` 漏检 | 提高曝光时间/增益（手动模式下缩短曝光时间以冻结运动），或降低分辨率提高帧率 |
| **内外沿跳变** | 尺度 s 突然变化 | 尺度跳变检测（见 5.1） |
| **背景干扰** | 误检到墙角、桌沿等 | 面积下限 + 长宽比 + `magnitude` 阈值；已锁定后用 ROI 限制搜索范围 |

---

## 6. 模块二：激光光斑识别

### 6.1 405nm 激光在 RGB 传感器上的成像特性

题目建议激光：**波长 405nm，光功率 ≤10mW**，靶上光斑直径 ≤0.5cm。

405nm 处于可见光最紫端。硅基 RGB Bayer 传感器对它的响应：
- **B 通道响应最强** → 光斑在图像上呈**蓝紫色**
- 光功率密度极高 → **光斑中心过曝饱和**（B=255，G 也偏高，R 偏低）
- 光斑边缘因衍射/散射有**紫色晕圈**
- 在紫外感光纸上照射后会留下**荧光痕迹**，痕迹本身也是蓝紫色，**滞留 20~60s** ← 这是最大的干扰源

**LAB 空间下的表现**：
- `L`（亮度）→ 很高（接近 100）
- `a`（红绿）→ 接近中性或略偏负
- `b`（黄蓝）→ **显著为负**（蓝色分量）

### 6.2 LAB 阈值标定方法（赛前必做）

不要瞎猜阈值。用 MaixVision 的 **find_blobs 示例应用**的滑块工具，或者自己写一段带触摸屏滑块的小程序：

1. 把激光笔固定在已知位置，对着靶纸打开
2. 用滑块调 `L_min, L_max, a_min, a_max, b_min, b_max`，目标是**只有光斑被高亮**
3. 记录下边界值，然后再补一个**过曝白心**的阈值组
4. 在不同距离（50cm / 100cm / 165cm）、不同靶纸磨损状态下各测一遍，取**交集**

建议的初始值（**必须现场标定后覆盖**）：

```python
TH_LASER = [
    [70, 100, -45, 25, -128, -30],   # 第 1 组：紫蓝主体
    [88, 100, -20, 20,  -60,  -5],   # 第 2 组：过曝白心（L 极高，b 不够负）
]
```

**为什么需要两组？** 光斑中心 RGB 都饱和时，LAB 里 `a≈0, b≈0`（白），用第 1 组会漏掉中心；用第 2 组的高 L 匹配白心。两组配 `merge=True` 合并成一个完整的斑。

### 6.3 快速路径：直接 `find_blobs`（推荐主用）

`find_blobs` **内部自带阈值分割**，不需要手动二值化：

```python
def detect_laser_fast(img, roi):
    blobs = img.find_blobs(
        TH_LASER,
        roi=roi,                  # [x, y, w, h]，限定在靶纸附近
        x_stride=1, y_stride=1,   # 光斑小，不要跳像素
        area_threshold=2,         # 远距离时很小，阈值要低
        pixels_threshold=2,       # 同上
        merge=True,               # 合并碎裂的光斑
        margin=3)                 # 合并距离
    if not blobs:
        return None
    return pick_best_laser(blobs, img)
```

**完整签名**（核对过）：

```python
find_blobs(self, thresholds: list[list[int]] = [], invert: bool = False,
           roi: list[int] = [], x_stride: int = 2, y_stride: int = 1,
           area_threshold: int = 10, pixels_threshold: int = 10,
           merge: bool = False, margin: int = 0,
           x_hist_bins_max: int = 0, y_hist_bins_max: int = 0) -> list[Blob]
```

**关键参数说明：**

| 参数 | 作用 | 本方案的取值理由 |
|---|---|---|
| `thresholds` | LAB 阈值，可传多组 | 见 6.2，两组 |
| `roi` | `[x,y,w,h]` 只算这块区域 | **性能与抗干扰的关键**。限定在靶纸外框内扩一圈 |
| `x_stride`/`y_stride` | 横向/纵向跳像素采样 | 光斑只有 2~10px，**必须设 1**，跳采样会漏掉 |
| `area_threshold` | 外接框面积下限 | 远距离光斑小，设 2~4 |
| `pixels_threshold` | 有效像素数下限 | 设 2~4，别用默认 10 |
| `merge` | 合并未被过滤的色块 | **必开**。二值化后光斑常碎成几块 |
| `margin` | 合并时的边缘扩展 | 2~4 |

### 6.4 稳健路径：手动二值化 + 形态学 + `find_blobs`

需要可视化调试、或需要更强的去噪时用这条：

```python
def detect_laser_robust(img, roi):
    x, y, w, h = roi
    tmp = img.crop(x, y, w, h).copy()   # 裁 ROI，减少计算量
    tmp.binary(TH_LASER)                # 原地二值化：命中→白(255)，其余→黑(0)
    tmp.open(1)                         # 开运算：去掉孤立亮点（传感器热噪、纸面反光颗粒）
    tmp.dilate(1)                       # 膨胀：把 1~2px 的光斑撑到能被 blob 检出
    blobs = tmp.find_blobs(
        [[80, 100, -128, 127, -128, 127]],   # 二值图里找白色
        area_threshold=2, pixels_threshold=2,
        merge=True, margin=3)
    if not blobs:
        return None
    b = pick_best_laser(blobs, tmp)
    if b is None:
        return None
    # ⚠️ 坐标要从 ROI 局部还原到全图
    return b.cxf() + x, b.cyf() + y, b
```

**形态学函数签名（核对过）：**

```python
erode(self,  size: int, threshold: int = -1, mask: Image = None) -> Image   # 腐蚀
dilate(self, size: int, threshold: int = 0,  mask: Image = None) -> Image   # 膨胀
open(self,   size: int, threshold: int = 0,  mask: Image = None) -> Image   # 先腐蚀后膨胀
close(self,  size: int, threshold: int = 0,  mask: Image = None) -> Image   # 先膨胀后腐蚀
binary(self, thresholds=[], invert=False, zero=False, mask=None,
       to_bitmap=False, copy=False) -> Image
```

- `size` 是核半径，实际核大小 = `(2·size+1) × (2·size+1)`。`size=1` → 3×3，`size=2` → 5×5。
- **全部是原地操作**（`copy` 默认 `False`），必须先 `copy()`。
- `binary()` 的 `invert=True` 会把阈值取反（找"不满足阈值"的像素）。

### 6.5 备选路径：numpy 蓝度法（最鲁棒）

LAB 阈值是三维盒子，对光照变化敏感。一个更物理、更可控的判据是**"蓝度"**：

```python
import numpy as np
import cv2
from maix import image

def detect_laser_numpy(img, roi):
    x, y, w, h = roi
    # image2cv: Image → numpy 数组；ensure_bgr=True 保证通道顺序是 BGR
    mat = image.image2cv(img, ensure_bgr=True, copy=False)
    patch = mat[y:y+h, x:x+w]                      # 形状 (h, w, 3)，ch0=B ch1=G ch2=R
    B = patch[:, :, 0].astype(np.int16)
    G = patch[:, :, 1].astype(np.int16)
    R = patch[:, :, 2].astype(np.int16)

    # 判据 1：蓝度 = B - max(G, R)，激光点这个值很大
    blueness = B - np.maximum(G, R)
    # 判据 2：绝对亮度，排除紫外痕迹（痕迹蓝但不亮）
    mask = ((blueness > 25) & (B > 150)).astype(np.uint8)

    # 形态学去噪（OpenCV，kernel 用 numpy 构造）
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.dilate(mask, k, iterations=1)

    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 3 or area > 2000:                # 面积门限
            continue
        cx, cy = cents[i]                          # 已经是浮点质心
        # 圆度筛选：外接框填充率
        bw, bh = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        fill = area / max(1, bw * bh)
        if fill < 0.45:                            # 太细长 → 不是光斑
            continue
        score = area * fill
        if best is None or score > best[0]:
            best = (score, cx, cy, i, labels, mask)
    if best is None:
        return None
    return refine_center_distance_transform(best, x, y)
```

**优缺点：**
- ✅ 鲁棒性最好，对光照/白平衡漂移不敏感（相对量而非绝对量）
- ✅ 可以顺便输出 `blueness` 的峰值作为置信度
- ⚠️ 需要 `import numpy, cv2`，且 `image.image2cv` 有数据拷贝开销
- ⚠️ 在 A53 上跑 1280×720 的 numpy 运算建议**只对 ROI 做**（先在 MaixPy 原生 `find_blobs` 粗定位，再用 numpy 在 64×64 的小窗口里精定位）

### 6.6 候选筛选与打分 `pick_best_laser`

无论走哪条路径，都要对候选 blob 打分并取最优。可用的 `Blob` 属性（核对过）：

| 属性 | 含义 | 本方案用途 |
|---|---|---|
| `blob.x() / y() / w() / h()` | 外接框 | 尺寸筛选 |
| `blob.cx() / cy()` | 整数质心 | 粗位置 |
| **`blob.cxf() / cyf()`** | **浮点质心** | **直接用，精度约 0.1~0.3px** |
| `blob.pixels()` | 有效像素数 | 面积筛选 |
| `blob.roundness()` | 圆度 0~1 | **激光点是圆的，阈值 >0.5** |
| `blob.density()` / `extent()` | 密度比 | >0.6 |
| `blob.solidity()` | 实心度 | >0.7 |
| `blob.convexity()` | 凸度 | >0.9 |
| `blob.perimeter()` | 周长 | 辅助 |
| `blob.enclosing_circle()` | 外接圆 `(x,y,r)` | 半径合理性 |
| `blob.code()` | 命中的阈值组下标掩码 | **判断是第 1 组还是第 2 组命中的** |

```python
def pick_best_laser(blobs, img, prev_px=None, max_jump_px=120):
    W, H = img.width(), img.height()
    best, best_score = None, -1.0
    for b in blobs:
        w, h = b.w(), b.h()
        area = b.pixels()
        if area < 2:
            continue
        # 1) 形状：光斑应接近圆
        if b.roundness() < 0.45:
            continue
        if w > 0 and h > 0 and max(w, h) / min(w, h) > 3.0:
            continue
        # 2) 亮度复核：取质心处原图像素，B 通道要足够强
        cx, cy = int(b.cxf()), int(b.cyf())
        if not (0 <= cx < W and 0 <= cy < H):
            continue
        pix = img.get_pixel(cx, cy, rgbtuple=True)   # [R, G, B]
        if pix[2] < 120 or pix[2] <= pix[0] + 20:    # B 不够强 或 B 不比 R 高 → 不是激光
            continue
        # 3) 位置门控：与上一帧的跳变不能太大
        if prev_px is not None:
            jump = ((b.cxf()-prev_px[0])**2 + (b.cyf()-prev_px[1])**2) ** 0.5
            if jump > max_jump_px:
                continue
        # 4) 打分
        score = area * 1.0 + b.roundness() * 50.0 + (pix[2] - pix[0]) * 0.5
        if b.code() == 1:      # 命中的是第 1 组（紫蓝主体）而非第 2 组（白心），更可信
            score += 30
        if score > best_score:
            best_score, best = score, b
    return best
```

### 6.7 亚像素质心：从"够用"到"极准"

**层次 1（够用）**：直接用 `blob.cxf() / cyf()` —— 浮点质心，重复性约 0.2~0.4px。**大部分情况到此为止。**

**层次 2（推荐）**：**距离变换峰值法**。对二值 mask 做距离变换，取距离峰值点作为圆心。这对**过曝的大光斑**特别有效——因为过曝区的形状会被镜头衍射拉成不规则的团，质心会偏，而距离变换的峰值稳定落在几何中心。

```python
def refine_center_distance_transform(best, off_x, off_y):
    score, cx, cy, idx, labels, mask = best
    comp = (labels == idx).astype(np.uint8)
    # 确保是 8 位单通道
    dist = cv2.distanceTransform(comp, cv2.DIST_L2, 5)
    _, max_val, _, max_loc = cv2.minMaxLoc(dist)
    return float(max_loc[0] + off_x), float(max_loc[1] + off_y), float(max_val)
```

**层次 3（极限）**：**亮度加权质心**（intensity-weighted centroid）。用光斑的亮度分布（用 `B - R` 作权重）算加权中心。适合光斑刚好不饱和、亮度分布对称的情况。

```python
def refine_center_weighted(img, roi):
    x, y, w, h = roi
    mat = image.image2cv(img, ensure_bgr=True, copy=False)[y:y+h, x:x+w]
    B = mat[:, :, 0].astype(np.float32)
    R = mat[:, :, 2].astype(np.float32)
    W = np.clip(B - R, 0, None)
    W[W < 20] = 0.0
    tot = W.sum()
    if tot <= 0:
        return None
    ys, xs = np.mgrid[0:h, 0:w]
    cx = float((xs * W).sum() / tot) + x
    cy = float((ys * W).sum() / tot) + y
    return cx, cy
```

> 实用建议：**层次 1 打底 + 层次 2 兜底（当光斑面积 > 30px 时启用）**。层次 3 只在有富余算力时做。

### 6.8 干扰源与对策汇总

| 干扰源 | 现象 | 对策 |
|---|---|---|
| **紫外感光纸旧痕迹**（20~60s 滞留） | 蓝紫色斑块被误认为光斑 | ① `L` / `B` 绝对亮度下限；② 面积上限；③ **位置连续性门控**（痕迹是过去的轨迹，不会追着当前点跑） |
| **光斑过曝成一坨白** | LAB 的 `b` 不显著负，第 1 组阈值漏检；质心偏移 | ① 加第 2 组"白心"阈值；② 距离变换求中心（6.7 层次 2） |
| **光斑太小（1~2px）** | `pixels_threshold=10` 默认值直接过滤掉 | ① 降低 `pixels_threshold` 到 2；② `dilate(1)` 撑大；③ 提高分辨率 / `set_windowing` |
| **纸面反光颗粒、传感器热噪** | 孤立亮点 | `open(1)` 开运算；`area_threshold >= 2` |
| **激光被遮挡/熄灭** | 完全无候选 | 返回 `None` + `flags.laser_valid = 0`；连续 K 帧无 → 报警（题目要求"激光必须连续发光"） |
| **相机自动曝光压暗画面** | 阈值周期性失效 | **锁定 AE/AWB**（见结论三） |
| **白墙/浅色背景误检** | 面积过大的白斑 | 面积上限（如 2000px）；ROI 限定在靶纸内 |

---

## 7. 模块三：时序滤波与跟踪

### 7.1 为什么必须滤波

- `find_rects` 的角点在不同帧之间存在 ±1~2px 抖动 → 经过 H 放大，靶心可能跳 ±3~5px
- 激光点受曝光噪声影响，质心抖动 ±0.5px
- 小车运动 → 靶纸在画面里快速移动，逐帧独立检测会有"呼吸"效应

**控制侧对抖动非常敏感**（云台会在抖动的设定值上来回抖），所以必须滤波。

### 7.2 One Euro Filter（首选）

低延迟 + 强抖动抑制，专为交互式跟踪设计，代码量只有 30 行：

```python
import math

class OneEuroFilter:
    def __init__(self, freq=30.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, freq):
        tau = 1.0 / (2 * math.pi * cutoff)
        te = 1.0 / freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x, t=None):
        if self.t_prev is not None and t is not None and t > self.t_prev:
            self.freq = 1.0 / (t - self.t_prev)
        if self.x_prev is None:
            self.x_prev = x
            self.t_prev = t
            return x
        # 微分
        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.d_cutoff, self.freq)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        # 自适应截止频率：动得越快，滤波越弱（延迟越低）
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, self.freq)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat
```

**调参建议：**
- 打靶心（静态、要稳）：`min_cutoff=0.6, beta=0.004`
- 动态瞄准（要快）：`min_cutoff=2.5, beta=0.02`
- 画圆（要平滑圆滑）：`min_cutoff=1.5, beta=0.01`

靶心用 `(tx, ty)` 两个独立的 One Euro Filter；激光点同理。

### 7.3 门控（Outlier Gating）

滤波之前先剔除野点，否则野点会污染滤波状态：

```python
def gate(prev, cur, dt, v_max_px_s=2000.0):
    if prev is None or cur is None:
        return cur
    d = math.hypot(cur[0]-prev[0], cur[1]-prev[1])
    if d > v_max_px_s * dt:
        return None      # 判定为野点
    return cur
```

`v_max_px_s` 按现场估算：小车最大线速度 × px/cm 再留 2 倍余量。

### 7.4 丢帧外推

```python
class TrackState:
    def __init__(self):
        self.pos = None       # (x, y)
        self.vel = (0.0, 0.0)
        self.lost = 0

    def update(self, meas, dt):
        if meas is not None:
            if self.pos is not None:
                vx = (meas[0] - self.pos[0]) / dt
                vy = (meas[1] - self.pos[1]) / dt
                # 一阶低通，避免速度估计抖动
                self.vel = (0.7*self.vel[0] + 0.3*vx, 0.7*self.vel[1] + 0.3*vy)
            self.pos = meas
            self.lost = 0
        else:
            if self.pos is not None:
                self.pos = (self.pos[0] + self.vel[0]*dt,
                            self.pos[1] + self.vel[1]*dt)
            self.lost += 1
        return self.pos
```

**失效阈值**：
- 连续丢帧 1~3 帧 → 外推，`quality` 降级但继续输出
- 连续丢帧 > 5 帧 → `flags.laser_valid = 0`，通知控制侧"目标丢失"
- 连续丢帧 > 15 帧 → 判定彻底丢靶，清空 TrackState，进入全图搜索

---

## 8. 靶面坐标系、画圆解算与控制接口

### 8.1 靶面坐标系定义

以靶纸**左上角**为原点，x 向右（0→21cm），y 向下（0→29.7cm）：

```
(0,0) ──────────────────── (21,0)
  │                          │
  │        · 靶心             │
  │     (10.5, 14.85)         │
  │                          │
(0,29.7) ──────────────── (21,29.7)
```

**为什么用靶面坐标系？** 因为题目的一切精度要求（D₁ ≤ 2cm、D₂ ≤ 2cm）都是在**靶面上**测量的。在靶面坐标系里做误差计算，物理意义直接对应评分。

### 8.2 画圆解算（发挥 3）

**目标**：小车沿轨迹走 1 圈，激光在靶面上同步画一个半径 6cm 的圆，同步误差 < 1/4 圈。

```python
import math

R_CM = 6.0
TARGET_CENTER_CM = (10.5, 14.85)

def circle_target_cm(progress, r_cm=R_CM):
    """
    progress: 小车当前圈的完成度，0.0 ~ 1.0（由 MSPM0 从编码器里程上报）
    返回靶面上应被激光击中的点（cm）
    """
    theta = 2.0 * math.pi * progress
    return (TARGET_CENTER_CM[0] + r_cm * math.cos(theta),
            TARGET_CENTER_CM[1] + r_cm * math.sin(theta))
```

然后统一投影成像素目标点：

```python
def target_px(H, T_cm):
    return project(H, T_cm[0], T_cm[1])
```

- **打靶心**：`T_cm = (10.5, 14.85)`
- **画圆**：`T_cm = circle_target_cm(progress)`

**两者共用同一条代码路径** —— 这是第 0 节"输出目标像素点"设计的直接红利。

### 8.3 三个必须注意的细节

**① 圈进度必须用里程（编码器），不能用时间。**
题目要求同步误差 < 1/4 圈 = 90°。如果按时间线性推进进度，小车速度波动就会破坏同步。**用 MSPM0 上报的"已行驶里程 / 单圈周长"作为 `progress`**，天然同步。

**② `progress` 的 0 点要对齐。**
画圆的起始角 θ=0 对应靶面上"靶心右侧 6cm"的点。小车起点的位置决定了这个点是否合适。建议：**允许在代码里加一个相位偏移常量 `PHASE_OFFSET`**，赛前实测后写死，避免起终点接缝处偏差超限。

**③ 激光笔连续发光。**
发挥部分要求"运动期间激光笔必须连续发光射向靶面，不满足要求不计成绩"。视觉侧要持续监控 `laser_valid`，一旦连续丢失就上报告警（控制侧决定重试或降级）。

### 8.4 闭环误差（装法已确定，不再是"两种装法"）

相机装在云台上、激光笔固定在相机顶部，因此：

| 项 | 说明 |
|---|---|
| 激光点在图像中的位置 | **相对相机恒定**（刚性连接，不随云台角度变），但**不在画面中心**，且随距离有视差 |
| 靶心在图像中的位置 | 随云台角度 + 车体位姿变化 |
| **控制误差** | **`e = T_px − L_px`** |
| 视觉侧额外工作 | 无——直接输出 `T_px` 与 `L_px` 两个点即可 |

> ⚠️ **不要用 `e = T_px − 图像中心`**。因为激光笔在相机顶部，激光点根本不在画面中心，且偏移随距离从 20px 变到 111px。详见结论二，以及《E题瞄准模块控制方案.md》第 2 节。

---

## 9. 输出协议（UART → 云台控制侧）

`uart.UART` 签名（核对过）：

```python
uart.UART(port='', baudrate=115200, databits=..., parity=..., stopbits=..., flow_ctrl=...)
u.set_baudrate(921600)
u.write_str("...")                    # 发字符串
u.write(data)                         # 发字节
u.available(timeout=0)                # 可读字节数
u.read()                              # 读
u.set_received_callback(callback)     # 接收回调
```

**建议波特率 921600**（115200 传 30Hz 的包够用但余量小，且动态瞄准希望延迟低）。

### 9.1 帧格式

```
┌──────┬──────┬──────┬──────┬────────────────────┬────────┐
│ 0xAA │ 0x55 │ type │ len  │      payload       │ CRC8   │
│  1B  │  1B  │  1B  │  1B  │       len B        │  1B    │
└──────┴──────┴──────┴──────┴────────────────────┴────────┘
```

### 9.2 type = 0x01：视觉结果帧（MaixCAM2 → 控制侧），payload 22 字节

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `t_px_x` | int16 | 目标点像素 x（×10 定点，即 0.1px 精度） |
| 2 | `t_px_y` | int16 | 目标点像素 y（×10） |
| 4 | `l_px_x` | int16 | 激光点像素 x（×10），无效时置 0 |
| 6 | `l_px_y` | int16 | 激光点像素 y（×10），无效时置 0 |
| 8 | `err_x` | int16 | `l_px − t_px` 的 x（×10），控制侧可直接用 |
| 10 | `err_y` | int16 | 同上 y |
| 12 | `px_per_cm` | uint16 | 靶面尺度 ×10（范围 0~6553.5） |
| 14 | `err_cm_x` | int16 | 靶面 cm 误差 ×100（用于直接读 D₁/D₂） |
| 16 | `err_cm_y` | int16 | 同上 |
| 18 | `quality` | uint8 | 0~100 置信度 |
| 19 | `flags` | uint8 | bit0:靶纸有效 bit1:激光有效 bit2:跟踪锁定 bit3:丢帧外推 |
| 20 | `frame_id` | uint8 | 帧序号（控制侧判断丢包/延迟） |
| 21 | `reserved` | uint8 | 预留 |

### 9.3 type = 0x02：控制侧 → MaixCAM2（可选）

| 字段 | 说明 |
|---|---|
| `mode` | 0=待机 1=打靶心 2=画圆 3=全图搜索 |
| `progress` | uint16，画圆进度 ×10000（0~10000 对应 0~1.0） |
| `phase_offset` | int16，相位偏移 ×1000 (度) |

**注意**：`progress` 从 MSPM0 来。如果你用**一块 MaixCAM2 同时做视觉和控制**，就不需要这个反向包，进程内变量直接传，延迟还更低。

### 9.4 打包代码

```python
import struct

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

def pack_vision_frame(t_px, l_px, s, err_cm, quality, flags, fid):
    tx, ty = int(round(t_px[0]*10)), int(round(t_px[1]*10))
    if l_px:
        lx, ly = int(round(l_px[0]*10)), int(round(l_px[1]*10))
        ex, ey = lx - tx, ly - ty
    else:
        lx, ly, ex, ey = 0, 0, 0, 0
    payload = struct.pack('<hhhhhhHhhBBBB',
                          tx, ty, lx, ly, ex, ey,
                          int(round(s*10)),
                          int(round(err_cm[0]*100)), int(round(err_cm[1]*100)),
                          quality, flags, fid, 0)
    body = bytes([0xAA, 0x55, 0x01, len(payload)]) + payload
    return body + bytes([crc8(body)])
```

---

## 10. 相机参数锁定（完整清单）

程序启动阶段一次性做完：

```python
from maix import camera, display, image

W, H = 1280, 720
cam = camera.Camera(W, H, image.Format.FMT_RGB888, fps=30)
disp = display.Display()

# ---- 1. 让自动算法先收敛，再锁死 ----
cam.skip_frames(30)

# ---- 2. 锁定曝光 ----
cam.exp_mode(camera.AeMode.Manual)     # AeMode: Invalid=-1, Auto=0, Manual=1
cam.exposure(6000)                     # 实测标定
cam.gain(40)                           # 实测标定（增益别太高，噪声会变大）
# cam.iso(...)                         # 部分 sensor 支持

# ---- 3. 锁定白平衡 ----
cam.awb_mode(camera.AwbMode.Manual)    # AwbMode: Invalid=-1, Auto=0, Manual=1
cam.set_wb_gain([1.5, 1.0, 1.8])       # [R, G, B] 增益，实测标定

# ---- 4. 图像风格（按需） ----
# cam.luma(128)
# cam.constrast(0)       # 注意官方拼写就是 constrast
# cam.saturation(0)      # 建议调低饱和度，减少色彩噪声对阈值的影响

# ---- 5. 可选：传感器窗口裁剪换取等效长焦（见 2.4） ----
# cam.set_windowing([x, y, w, h])
print("sensor size:", cam.get_sensor_size())
print("aiisp mode:", cam.get_aiisp_workmode())
```

**标定步骤（赛前必做，10 分钟）：**
1. 只开室内常亮照明，架好靶纸，激光关
2. 用 MaixVision 看预览，调 `exposure` / `gain` 让靶纸不过曝不欠曝（靶纸米白区域 L 约 75~88）
3. 调 `set_wb_gain` 让靶纸的黑胶带框看起来是中性灰黑（不偏蓝不偏黄）
4. 开激光，确认光斑 B 通道饱和（≈255），而画面其余部分亮度**没有变化**（说明 AE 确实锁住了）
5. 记录参数，写死进代码

---

## 11. 性能预算与调参优先级

### 11.1 推荐频率分配

| 环节 | 频率 | 说明 |
|---|---|---|
| 相机采集 | 30 ~ 60 fps | 动态瞄准建议 60fps（可降到 640×480 换帧率） |
| 激光点检测 | **每帧** | 便宜，且是控制反馈的直接来源 |
| 靶纸检测（`find_rects`） | **10 ~ 15 Hz** | 贵；中间帧用 H 平滑外推 |
| H 更新 | 10 ~ 15 Hz | 同上 |
| 控制输出 | 30 ~ 60 Hz | 每帧都发，保证控制侧不饿死 |

### 11.2 端到端延迟预算（目标 < 40ms）

| 环节 | 典型耗时 |
|---|---|
| sensor 曝光 + 读出 | 10 ~ 16 ms（曝光时间越长越大，动态时要把曝光压短） |
| ISP + 传给 A53 | 3 ~ 8 ms |
| 靶纸检测（15Hz 时摊薄） | 平均 < 2 ms |
| 激光点检测（ROI 内） | 2 ~ 6 ms |
| 滤波 + 打包 + 串口 | < 1 ms |
| **合计** | **约 20 ~ 35 ms** |

> 动态瞄准时**把曝光时间压短**（提高 gain 补偿），可以显著降低运动模糊和延迟，牺牲一点噪点——这是划算的。

### 11.3 调参优先级（时间有限时按这个顺序）

1. **锁曝光/白平衡**（不做这个，后面全是白干）
2. **靶纸 4 角点定位精度**（决定一切的上限）
3. **激光点检出率**（调 `TH_LASER` + `pixels_threshold`）
4. **One Euro Filter 参数**（消除抖动，控制侧会立刻变好调）
5. 质心精细化（距离变换 / 亮度加权）
6. numpy 蓝度法替换 LAB 阈值（鲁棒性升级）
7. `set_windowing` 长焦（远距离精度救急）

---

## 12. 赛前标定 SOP

| # | 项目 | 方法 | 产出 |
|---|---|---|---|
| 1 | 相机内参（可选） | 棋盘格标定，或用标准长度物实测 | `fx, fy, cx, cy`（`find_apriltags` 支持传入） |
| 2 | 镜头畸变系数 | 拿直尺/直线物在画面不同位置拍，看弯曲量 | `lens_corr(strength)` 的 `strength` 值 |
| 3 | **靶面 px/cm 表** | 靶纸放 50 / 100 / 165cm，各测一次 `px_per_cm(H, 10.5, 14.85)` | 距离–尺度查找表（用于粗搜时的先验） |
| 4 | **激光 LAB 阈值** | MaixVision 滑块工具，3 个距离各标一遍取交集 | `TH_LASER` |
| 5 | **黑框 LAB 阈值** | 同上（仅级别 2 兜底用） | `TH_BLACK` |
| 6 | 曝光/白平衡 | 见第 10 节 | `exposure / gain / wb_gain` |
| 7 | **相机光轴–激光光轴偏置** | 固定靶纸，让激光打靶心，读激光点在图像中的位置 | 偏置常量（同轴装法必需） |
| 8 | 角点重复性验证 | 静止状态下连续测 100 帧，统计靶心像素坐标的标准差 | 目标 **σ < 0.5px**；超过 1px 说明角点提取有问题 |
| 9 | 激光点重复性验证 | 同上 | 目标 **σ < 0.3px** |

---

## 13. 完整代码骨架

> 这是一份结构完整的可运行骨架，`TODO: 标定` 处需要现场实测填入。建议先在 MaixVision 里跑通、看到画面上画出靶纸框和光斑框，再接入控制。

```python
# vision_main.py  —  MaixCAM2 / MaixPy v4
import math
import numpy as np
import cv2
from maix import camera, display, image, app, time, uart

# ============================================================
# 0. 配置与阈值（TODO: 全部现场标定后写死）
# ============================================================
W, H            = 1280, 720
A4_W_CM, A4_H_CM = 21.0, 29.7
TARGET_CM       = (A4_W_CM / 2, A4_H_CM / 2)      # 靶心 (10.5, 14.85)
R_CIRCLE_CM     = 6.0

EXPOSURE, GAIN  = 6000, 40                        # TODO
WB_GAIN         = [1.5, 1.0, 1.8]                 # TODO

TH_LASER = [
    [70, 100, -45, 25, -128, -30],                # TODO 紫蓝主体
    [88, 100, -20, 20,  -60,  -5],                # TODO 过曝白心
]
TH_BLACK = [[0, 45, -25, 25, -25, 25]]            # TODO

DETECT_TARGET_EVERY = 2        # 每 N 帧做一次靶纸检测
MAX_LOST_FRAMES     = 5
MIN_QUAD_AREA_RATIO = 0.01

BYTE_ORDER = '<'   # 小端

# ============================================================
# 1. 几何工具
# ============================================================
def cross(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def dist(a, b):
    return float(np.hypot(a[0]-b[0], a[1]-b[1]))

def order_corners(pts):
    """排成 [左上, 右上, 右下, 左下]"""
    p = np.array(pts, dtype=np.float32).reshape(4, 2)
    s = p.sum(axis=1)
    d = p[:, 0] - p[:, 1]
    return np.array([p[np.argmin(s)], p[np.argmax(d)],
                     p[np.argmax(s)], p[np.argmin(d)]], dtype=np.float32)

def is_convex(q):
    signs = [cross(q[i], q[(i+1) % 4], q[(i+2) % 4]) for i in range(4)]
    return all(v > 0 for v in signs) or all(v < 0 for v in signs)

def quad_area(q):
    return abs(sum(cross(q[i], q[(i+1) % 4], q[(i+2) % 4]) for i in range(4))) / 2.0

def is_valid_quad(q, img):
    area = quad_area(q)
    if area < MIN_QUAD_AREA_RATIO * img.width() * img.height():
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
    src = np.float32([[0, 0], [A4_W_CM, 0],
                      [A4_W_CM, A4_H_CM], [0, A4_H_CM]])
    return cv2.getPerspectiveTransform(src, np.float32(quad_px))

def project(Hm, x_cm, y_cm):
    v = Hm @ np.array([x_cm, y_cm, 1.0])
    if abs(v[2]) < 1e-9:
        return None
    return float(v[0] / v[2]), float(v[1] / v[2])

def px_per_cm(Hm, x_cm, y_cm):
    p0 = project(Hm, x_cm, y_cm)
    px = project(Hm, x_cm + 1.0, y_cm)
    py = project(Hm, x_cm, y_cm + 1.0)
    if not (p0 and px and py):
        return 0.0
    return float((dist(p0, px) + dist(p0, py)) / 2.0)

def quad_roi(q, img, pad=40):
    xs = [p[0] for p in q]
    ys = [p[1] for p in q]
    x0 = max(0, int(min(xs)) - pad)
    y0 = max(0, int(min(ys)) - pad)
    x1 = min(img.width(),  int(max(xs)) + pad)
    y1 = min(img.height(), int(max(ys)) + pad)
    return [x0, y0, max(1, x1 - x0), max(1, y1 - y0)]

# ============================================================
# 2. 靶纸检测（三级）
# ============================================================
def detect_target_l1(img, roi=None):
    cands = []
    rects = img.find_rects(roi=roi or [], threshold=10000)
    for r in rects:
        cs = r.corners()
        if len(cs) != 4:
            continue
        q = order_corners(cs)
        if not is_valid_quad(q, img):
            continue
        score = r.magnitude() * 0.6 + quad_area(q) / (img.width() * img.height()) * 10000 * 0.4
        cands.append((score, q))
    if not cands:
        return None
    cands.sort(key=lambda t: -t[0])
    return cands[0][1]

def detect_target_l2(img, roi=None):
    tmp = img.copy()
    tmp.binary(TH_BLACK)
    tmp.open(1)
    tmp.close(2)
    blobs = tmp.find_blobs([[80, 100, -128, 127, -128, 127]],
                           roi=roi or [], area_threshold=200,
                           pixels_threshold=200, merge=True, margin=8)
    if not blobs:
        return None
    b = max(blobs, key=lambda x: x.pixels())
    pts = b.mini_corners()
    if len(pts) != 4:
        return None
    q = order_corners(pts)
    return q if is_valid_quad(q, img) else None

def detect_target(img, last_quad=None):
    """三级检测。返回排好序的 4 角点，或 None"""
    roi = quad_roi(last_quad, img) if last_quad is not None else None
    q = detect_target_l1(img, roi)
    if q is None:
        q = detect_target_l2(img, roi)
    if q is None:
        return None
    # 尺度跳变检测：防止内外沿互换
    return q

# ============================================================
# 3. 激光点检测
# ============================================================
def pick_best_laser(blobs, img, prev_px=None, max_jump_px=150):
    W_, H_ = img.width(), img.height()
    best, best_score = None, -1.0
    for b in blobs:
        if b.pixels() < 2:
            continue
        if b.roundness() < 0.45:
            continue
        w, h = b.w(), b.h()
        if w > 0 and h > 0 and max(w, h) / min(w, h) > 3.0:
            continue
        cx, cy = int(b.cxf()), int(b.cyf())
        if not (0 <= cx < W_ and 0 <= cy < H_):
            continue
        pix = img.get_pixel(cx, cy, rgbtuple=True)     # [R, G, B]
        if pix[2] < 120 or pix[2] <= pix[0] + 20:
            continue
        if prev_px is not None:
            if dist((b.cxf(), b.cyf()), prev_px) > max_jump_px:
                continue
        score = b.pixels() * 1.0 + b.roundness() * 50.0 + (pix[2] - pix[0]) * 0.5
        if b.code() == 1:
            score += 30
        if score > best_score:
            best_score, best = score, b
    return best

def detect_laser(img, roi, prev_px=None):
    blobs = img.find_blobs(TH_LASER, roi=roi,
                           x_stride=1, y_stride=1,
                           area_threshold=2, pixels_threshold=2,
                           merge=True, margin=3)
    if not blobs:
        return None
    b = pick_best_laser(blobs, img, prev_px)
    if b is None:
        return None
    return float(b.cxf()), float(b.cyf()), float(b.pixels())

# ============================================================
# 4. One Euro 滤波
# ============================================================
class OneEuroFilter:
    def __init__(self, freq=30.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.freq, self.min_cutoff, self.beta, self.d_cutoff = freq, min_cutoff, beta, d_cutoff
        self.x_prev, self.dx_prev, self.t_prev = None, 0.0, None
    @staticmethod
    def _alpha(cutoff, freq):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)
    def __call__(self, x, t):
        if self.t_prev is not None and t > self.t_prev:
            self.freq = 1.0 / max(1e-3, t - self.t_prev)
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dx = (x - self.x_prev) * self.freq
        a_d = self._alpha(self.d_cutoff, self.freq)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, self.freq)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat

# ============================================================
# 5. 主循环
# ============================================================
def main():
    cam = camera.Camera(W, H, image.Format.FMT_RGB888, fps=30)
    disp = display.Display()
    cam.skip_frames(30)
    cam.exp_mode(camera.AeMode.Manual)
    cam.exposure(EXPOSURE)
    cam.gain(GAIN)
    cam.awb_mode(camera.AwbMode.Manual)
    cam.set_wb_gain(WB_GAIN)

    ser = uart.UART("/dev/ttyS0", 921600)

    fx = OneEuroFilter(30, 0.6, 0.004)     # 靶心 x
    fy = OneEuroFilter(30, 0.6, 0.004)     # 靶心 y
    lx = OneEuroFilter(30, 1.0, 0.008)     # 激光 x
    ly = OneEuroFilter(30, 1.0, 0.008)     # 激光 y

    Hmat = None
    last_quad = None
    frame_id = 0
    lost_cnt = 0
    tl_hint = (0.0, 0.0)
    laser_prev = None
    t0 = time.time()

    while not app.need_exit():
        img = cam.read()
        t = time.time() - t0

        # ---------- A. 靶纸检测 ----------
        if frame_id % DETECT_TARGET_EVERY == 0 or Hmat is None:
            q = detect_target(img, last_quad)
            if q is not None:
                last_quad = q
                Hmat = build_homography(q)
                lost_cnt = 0
            else:
                lost_cnt += 1

        target_ok = Hmat is not None and lost_cnt < MAX_LOST_FRAMES

        # ---------- B. 目标点（靶面 → 像素） ----------
        t_px = None
        if target_ok:
            # TODO: 打靶心用 TARGET_CM；画圆改为 circle_target_cm(progress)
            raw = project(Hmat, TARGET_CM[0], TARGET_CM[1])
            if raw:
                t_px = (fx(raw[0], t), fy(raw[1], t))

        # ---------- C. 激光点检测 ----------
        l_px = None
        if target_ok and last_quad is not None:
            roi = quad_roi(last_quad, img, pad=60)
            r = detect_laser(img, roi, laser_prev)
            if r:
                l_px = (lx(r[0], t), ly(r[1], t))
                laser_prev = (r[0], r[1])
            else:
                laser_prev = None

        # ---------- D. 误差与打包 ----------
        s = px_per_cm(Hmat, TARGET_CM[0], TARGET_CM[1]) if target_ok else 0.0
        err_cm = (0.0, 0.0)
        if t_px and l_px and s > 0:
            err_cm = ((l_px[0] - t_px[0]) / s, (l_px[1] - t_px[1]) / s)

        flags = (1 if target_ok else 0) | (2 if l_px else 0)
        if target_ok and l_px:
            flags |= 4
        quality = 100 if (target_ok and l_px) else (60 if target_ok else 0)

        if t_px:
            pkt = pack_vision_frame(t_px, l_px, s, err_cm, quality, flags, frame_id & 0xFF)
            ser.write(pkt)

        # ---------- E. 调试可视化 ----------
        if last_quad is not None:
            for i in range(4):
                a_, b_ = last_quad[i], last_quad[(i + 1) % 4]
                img.draw_line(int(a_[0]), int(a_[1]), int(b_[0]), int(b_[1]),
                              image.COLOR_GREEN, 2)
        if t_px:
            img.draw_cross(int(t_px[0]), int(t_px[1]), image.COLOR_RED, 8, 2)
        if l_px:
            img.draw_cross(int(l_px[0]), int(l_px[1]), image.COLOR_YELLOW, 8, 2)
        img.draw_string(8, 8, f"fps={1.0/max(1e-3,t-last_t):.1f} s={s:.2f}px/cm",
                        image.COLOR_WHITE, 1.5)
        disp.show(img)

        last_t = t
        frame_id += 1

    cam.close()
    ser.close()

if __name__ == "__main__":
    main()
```

> 上面的 `last_t` 需要在循环外初始化（`last_t = 0.0`），`pack_vision_frame` / `crc8` 用第 9.4 节的实现。

---

## 14. 调试与可视化建议

| 工具 | 用途 |
|---|---|
| **MaixVision** | PC 端实时预览、看帧率、单步调试。开发期必备 |
| **`img.save("/root/debug/frame_%d.jpg" % n)`** | 保存关键帧，赛后复盘；也可用于报告插图 |
| **触摸屏 + 阈值滑块** | 现场快速标 LAB 阈值（MaixCAM2 自带 640×480 触摸屏） |
| **画面上叠加 debug 文字** | 显示 `fps` / `px_per_cm` / 光斑面积 / `quality` / `lost_cnt` —— 一眼看出哪一环坏了 |
| **二值图单独显示** | 把 `tmp`（二值化后的图）`disp.show()` 出来，看形态学效果对不对 |
| **串口波形（VOFA+）** | 把 `err_x / err_y / px_per_cm` 发到 PC 上看曲线，调滤波参数必备 |

---

## 15. 验收指标（自测）

| 指标 | 目标值 | 怎么测 |
|---|---|---|
| 靶纸检测成功率 | ≥ 98%（静止） | 连续 300 帧统计 |
| 靶纸检测成功率（动态） | ≥ 90% | 小车跑圈时统计，允许外推 |
| 靶心像素重复性 σ | < 0.5 px | 静止 100 帧的靶心坐标标准差 |
| 激光点检出率 | ≥ 95% | 连续 300 帧统计 |
| 激光点重复性 σ | < 0.3 px | 静止 100 帧 |
| 端到端延迟 | < 40 ms | 从 `cam.read()` 返回到 `ser.write()` 完成 |
| 帧率 | ≥ 30 fps | 画面叠加的 fps 计数 |
| 靶面尺度误差 | < 3% | 用已知长度物（如 21cm 靶纸宽）交叉验证 |

---

## 16. 常见坑清单（照单排查）

| # | 坑 | 现象 | 修法 |
|---|---|---|---|
| 1 | **`binary` / `erode` 是原地操作** | 原图被破坏，后续检测全乱 | 先 `img.copy()` |
| 2 | **`mini_corners()` 拼错成 `min_corners()`** | AttributeError | 是 `mini_corners()` |
| 3 | **角点顺序没归一化** | 靶心跑到画面外，云台乱转 | 用 `order_corners()` |
| 4 | **单应矩阵左右镜像** | 画出的 6cm 圆左右颠倒 | `order_corners` 后做 `is_convex` 校验；必要时交换 TR/BL |
| 5 | **`find_rects` 框到内沿** | 尺度 s 突然跳变、靶心偏移 | 尺度跳变检测；或用"矩形中心 = 靶心" |
| 6 | **`pixels_threshold` 用了默认值 10** | 远距离光斑检不出来 | 降到 2~4，并 `dilate(1)` |
| 7 | **没锁 AE/AWB** | 阈值"时好时坏"，白天晚上不一样 | 手动曝光 + 手动白平衡 |
| 8 | **紫外感光纸旧痕迹被当光斑** | 画面里到处是候选点 | L/B 亮度下限 + 面积上限 + 位置门控 |
| 9 | **光斑过曝成一坨白，质心偏** | 激光点定位偏移 2~3px | 距离变换求中心；或加"白心"阈值组 |
| 10 | **ROI 坐标没换算回全图** | 用 `crop` 后的 blob 坐标直接当全图坐标 | 加回 ROI 偏移 `+x, +y` |
| 11 | **`set_windowing` 后坐标系没改** | 所有坐标整体偏移 | 用之前先写测试脚本验证映射关系 |
| 12 | **忘了串口波特率两端不一致** | 收到乱码 | 两端都设 921600 |
| 13 | **控制侧用"像素误差"直接当角度** | 云台增益随距离变化，近处抖远处慢 | 像素误差 ≈ 角度误差 × 焦距，与距离无关；若必须转角度，用 `err_cm / 距离` |
| 14 | **画圆进度用时间而不是里程** | 同步误差超 1/4 圈 | 用 MSPM0 上报的编码器里程 |

---

## 附：本方案用到的 MaixPy v4 API 速查（已核对签名）

```python
# 相机
camera.Camera(width=-1, height=-1, format=..., device=None, fps=-1, buff_num=3, open=True, raw=False)
cam.read(block=True, block_ms=-1)     cam.skip_frames(num)       cam.close()
cam.exp_mode(AeMode)                  cam.exposure(v)            cam.gain(v)
cam.awb_mode(AwbMode)                 cam.set_wb_gain([r,g,b])   cam.set_windowing([x,y,w,h])
cam.add_channel(w,h,fmt,fps,buff_num,open)   cam.get_channel()
cam.hmirror(v)  cam.vflip(v)  cam.get_sensor_size()  cam.get_aiisp_workmode()

# 图像（Image 类方法）
img.find_blobs(thresholds=[], invert=False, roi=[], x_stride=2, y_stride=1,
               area_threshold=10, pixels_threshold=10, merge=False, margin=0,
               x_hist_bins_max=0, y_hist_bins_max=0) -> list[Blob]
img.find_rects(roi=[], threshold=10000) -> list[Rect]
img.find_circles(roi=[], x_stride=2, y_stride=1, threshold=2000,
                 x_margin=10, y_margin=10, r_margin=10, r_min=2, r_max=-1, r_step=2)
img.find_line_segments(roi=[], merge_distance=0, max_theta_difference=15) -> list[Line]
img.find_apriltags(roi=[], families=..., fx=-1, fy=-1, cx=-1, cy=-1)
img.get_regression(thresholds=[], invert=False, roi=[], x_stride=2, y_stride=1,
                   area_threshold=10, pixels_threshold=10, robust=False) -> list[Line]

img.binary(thresholds=[], invert=False, zero=False, mask=None, to_bitmap=False, copy=False)
img.erode(size, threshold=-1, mask=None)   img.dilate(size, threshold=0, mask=None)
img.open(size, threshold=0, mask=None)     img.close(size, threshold=0, mask=None)
img.crop(x, y, w, h) -> Image              img.resize(w, h, fit, method) -> Image
img.copy() -> Image                        img.mean_pool(x_div, y_div, copy=False)
img.rotation_corr(x_rot=0, y_rot=0, z_rot=0, x_trans=0, y_trans=0, zoom=1.0, fov=60.0, corners=[])
img.lens_corr(strength=1.8, zoom=1.0, x_corr=0.0, y_corr=0.0)
img.to_format(Format)  -> None   # ⚠️ 原地，仅支持 RGB888/BGR888/RGBA8888/BGRA8888/GRAYSCALE
img.get_pixel(x, y, rgbtuple=False) -> list[int]
img.save(path, quality=95)

img.draw_rect(x, y, w, h, color, thickness=1)        img.draw_cross(x, y, color, size=5, thickness=1)
img.draw_line(x1, y1, x2, y2, color, thickness=1)    img.draw_circle(x, y, radius, color, thickness=1)
img.draw_keypoints(keypoints, color, size=4, thickness=-1, line_thickness=0)
img.draw_string(x, y, text, color, scale=1, thickness=-1, wrap=True, wrap_space=4, font='')
img.draw_image(x, y, img)

# 模块级
image.image2cv(img, ensure_bgr=True, copy=True) -> numpy.ndarray
image.cv2image(array, bgr=True, copy=True) -> Image
image.Format.FMT_RGB888 / FMT_BGR888 / FMT_GRAYSCALE / ...
image.COLOR_WHITE / BLACK / RED / GREEN / BLUE / YELLOW / ORANGE / PURPLE / GRAY / INVALID

# Blob
b.x() b.y() b.w() b.h() b.cx() b.cy() b.cxf() b.cyf() b.pixels()
b.rect() b.corners() b.mini_corners() b.rotation_deg() b.roundness()
b.density() b.extent() b.solidity() b.convexity() b.elongation() b.compactness()
b.perimeter() b.area() b.enclosing_circle() b.enclosed_ellipse()
b.major_axis_line() b.minor_axis_line() b.code() b.count()

# Rect
r.rect() r.corners() r.x() r.y() r.w() r.h() r.magnitude()

# 串口 / PWM
uart.UART(port='', baudrate=115200, databits=..., parity=..., stopbits=..., flow_ctrl=...)
u.write_str(s)  u.write(data)  u.available(timeout=0)  u.read()  u.set_received_callback(cb)
pwm.PWM(id, freq=1000, duty=0, enable=True, duty_val=-1)
p.duty(v)  p.freq(v)  p.enable()  p.disable()  p.is_enabled()

# 应用
app.need_exit()     time.time()     display.Display().show(img)
```
