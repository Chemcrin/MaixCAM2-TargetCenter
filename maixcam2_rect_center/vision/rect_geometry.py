"""纯几何工具：四边形规整、单应解算、批量采样、正视面反投影。

本模块**不持有任何状态、不读 config**，只做可独立验证的数学与 OpenCV 调用，
供 `contour_locator`（候选提议）与 `rectangle_center`（验证/精修/中心解算）共用。

为什么单独拆出来
--------------------------------------------------------------------------------
靶心 = 黑框矩形两条对角线的交点（射影不变量）。整条链路可以拆成三段互不依赖的数学：

    ① 图像四边形 → 物理矩形 (mm) 的 4 点单应 H        `homography`
    ② 物理矩形中心 (w/2, h/2) 经 H 投影回图像 = 靶心   `project_points`
    ③ 反向：图像点 → 物理坐标（诊断/交叉校验用）        `inverse_map_points`

把这三段与"批量采样""四边形体检"一起放在这里，检测器代码就只剩"状态机 + 门限"，
换候选生成方案（阈值掩膜 → Canny 轮廓）时不必碰数学。
"""
import cv2
import numpy as np


# --------------------------------------------------------------------- 四边形
def polygon_area(p):
    """有符号面积的 2 倍（>0 = CCW，<0 = CW）。用于把四角规整成统一方向。

    直接展开 shoelace 公式（而不是 `cv2.contourArea(oriented=True)` + 重塑视图）：
    本函数在每帧的每个候选上都要跑，省掉一次数组重塑与一次 C 调用的开销是可测的。
    """
    p = np.asarray(p, np.float64).reshape(4, 2)
    x, y = p[:, 0], p[:, 1]
    return float(x[0] * (y[1] - y[3]) + x[1] * (y[2] - y[0]) +
                 x[2] * (y[3] - y[1]) + x[3] * (y[0] - y[2]))


def order_quad(points):
    """把 4 个点规整为统一顺序：**[左上, 右上, 右下, 左下]**（CCW）。

    以质心为极点按极角排序 → 保证 CCW → 旋转到"x+y 最小"（左上角）打头。
    统一顺序是后续所有几何运算（边向量、对角线、单应四点对应）的前提。
    用切片拼接代替 `np.roll`：同结果、少一次完整数组分配（每个候选都要调）。
    """
    p = np.asarray(points, np.float32).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]   # 按极角排序
    if polygon_area(p) < 0:
        p = p[::-1]                                                  # 反了则翻成 CCW
    shift = int(np.argmin(p[:, 0] + p[:, 1]))
    if shift:
        p = np.concatenate((p[shift:], p[:shift]))                   # 左上角打头
    return p.copy()


def side_lengths(q):
    """四边长度 [边0, 边1, 边2, 边3]，边 i = 点 i → 点 i+1。"""
    q = np.asarray(q, np.float64)
    x, y = q[:, 0], q[:, 1]
    nx = np.concatenate((x[1:], x[:1]))
    ny = np.concatenate((y[1:], y[:1]))
    return np.hypot(nx - x, ny - y)


def quad_is_valid(q, shape, margin=1, min_edge=12.0, min_sine=0.20):
    """四边形体检：不越界、凸、边长下限、无明显退化尖角。

    允许任意透视（不要求接近直角/矩形），只排除两类坏形状：
      · 贴边/出界：角点落在图像边缘 margin 内 → 采样会越界、中心不可信；
      · 退化尖角：相邻边几乎平行（sin 很小）→ 四角拟合不稳定。

    `min_edge` 是**像素**下限，调用方在缩小图上用时按比例传小值。
    同样只用切片拼接，不用 np.roll（每个候选两次调用，是定位器的热点之一）。
    """
    h, w = shape
    q = np.asarray(q, np.float64)
    x, y = q[:, 0], q[:, 1]
    if (x.min() < margin or y.min() < margin or
            x.max() >= w - margin or y.max() >= h - margin):
        return False
    if not cv2.isContourConvex(q.astype(np.float32).reshape(-1, 1, 2)):
        return False
    dx = np.concatenate((x[1:] - x[:3], (x[0] - x[3],)))
    dy = np.concatenate((y[1:] - y[:3], (y[0] - y[3],)))
    lengths = np.hypot(dx, dy)
    if lengths.min() < min_edge:
        return False
    ex = np.concatenate((dx[1:], dx[:1]))
    ey = np.concatenate((dy[1:], dy[:1]))
    sine = np.abs(dx * ey - dy * ex) / (lengths * np.concatenate((lengths[1:], lengths[:1])))
    return bool(sine.min() > min_sine)


def quad_from_contour(cnt, epsilon_ratio, hull_fill_ratio, factors=(1.0, 1.6, 2.4, 3.5, 5.0)):
    """从一条轮廓提取凸四边形；**容忍缺口/粘连造成的非四边形状**。

    两级容忍（实机必需，见 tools/test_real_scene.py 的背景说明）：
      ① epsilon 由小到大**扫描**原轮廓 —— 修复"带小缺口的多边形降到 4 点"；
      ② 仍不成四边形、且该轮廓**本身就像环形带**时，对**凸包**同样扫描 —— 缺口被
         凸包补上，得到近似矩形。凸包只对"中空程度高"的轮廓启用
         （轮廓面积 < hull_fill_ratio × 凸包面积），实心背景/阴影块不参与，
         否则会凭空多出大片假候选、白耗候选预算。
    凸包在这里**只作候选提议**：真伪由调用方的四边暗带验证判定。
    """
    hull = cv2.convexHull(cnt)
    hull_area = abs(cv2.contourArea(hull))
    band_like = (hull_area > 0 and
                 abs(cv2.contourArea(cnt)) < hull_fill_ratio * hull_area)
    for base in ((cnt, hull) if band_like else (cnt,)):
        peri = cv2.arcLength(base, True)
        if peri <= 0:
            continue
        for factor in factors:
            approx = cv2.approxPolyDP(base, epsilon_ratio * factor * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                return order_quad(approx)
            if len(approx) < 4:
                break                                            # 已过度简化
    return None


def line_through(a, b):
    """过 a、b 两点的齐次直线系数 (a_, b_, c_)：a_·x + b_·y + c_ = 0。

    与 `cv2.fitLine` 的输出口径一致，用于某条边没有足够边缘证据时沿用输入边。
    """
    (x0, y0), (x1, y1) = np.asarray(a, np.float64), np.asarray(b, np.float64)
    dx, dy = x1 - x0, y1 - y0
    return np.array([-dy, dx, dy * x0 - dx * y0], np.float64)


# --------------------------------------------------------------------- 单应
def gaussian_solve(a, b):
    """带部分主元的高斯消元；消去行向量化，避免 64 次 Python 行循环。

    主元过小（矩阵退化，如四点共线）抛 ValueError —— 不做伪解。
    """
    m = np.column_stack((np.asarray(a, np.float64), np.asarray(b, np.float64)))
    for col in range(len(m)):
        pivot = col + int(np.argmax(np.abs(m[col:, col])))
        if abs(m[pivot, col]) < 1e-10:
            raise ValueError("degenerate quadrilateral")
        if pivot != col:
            m[[col, pivot]] = m[[pivot, col]]
        m[col] /= m[col, col]
        factors = m[:, col].copy()
        factors[col] = 0.0
        m -= factors[:, None] * m[col]
    return m[:, -1]


def homography(src, dst):
    """解 4 点对应的 3×3 单应 H（使 H·src ≡ dst）。

    先对 src/dst **归一化**（平移质心到原点、缩放到量级 1）再建 8×8 线性方程，
    求得 H_n 后用 T_dst·H_n·T_src⁻¹ 还原 —— 归一化显著提升大像素坐标下的数值稳定性。
    """
    src, dst = np.asarray(src, np.float64), np.asarray(dst, np.float64)
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    ss, ds = float(np.max(np.abs(src - sc))), float(np.max(np.abs(dst - dc)))
    if min(ss, ds) < 1e-8:
        raise ValueError("zero-size rectangle")
    xy, uv = (src - sc) / ss, (dst - dc) / ds
    a = np.zeros((8, 8), np.float64)
    a[0::2, :3] = np.column_stack((xy, np.ones(4)))
    a[1::2, 3:6] = a[0::2, :3]
    a[0::2, 6:] = -uv[:, :1] * xy
    a[1::2, 6:] = -uv[:, 1:] * xy
    v = gaussian_solve(a, uv.reshape(-1))
    hn = np.append(v, 1.0).reshape(3, 3)
    ts = np.array([[1 / ss, 0, -sc[0] / ss], [0, 1 / ss, -sc[1] / ss], [0, 0, 1]])
    td_inv = np.array([[ds, 0, dc[0]], [0, ds, dc[1]], [0, 0, 1]])
    h = td_inv @ hn @ ts
    return h / h[2, 2] if abs(h[2, 2]) > 1e-12 else h


def homography_8x8(src_quad, width_mm, height_mm):
    """把物理矩形 (0,0)-(w,h) 映射到图像四边形 src_quad 的单应（mm → px）。"""
    return homography(src_quad, [[0, 0], [width_mm, 0],
                                 [width_mm, height_mm], [0, height_mm]])


def project_points(matrix, points):
    """齐次投影：把 (N,2) 点经 3×3 矩阵映射，做透视除法得 (N,2)。落无穷远点抛错。"""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    z = np.column_stack((p, np.ones(len(p)))) @ np.asarray(matrix).T
    if np.any(np.abs(z[:, 2]) < 1e-9):
        raise ValueError("point at projective infinity")
    return z[:, :2] / z[:, 2:]


def inverse_map_points(h, points):
    """单应逆映射（px → mm）；不做越界检查，由调用方负责。"""
    return project_points(np.linalg.inv(h), points)


# ------------------------------------------------------- 正视面（warp）辅助
def rectangle_corners(w, h, inset=0.0):
    """物理矩形四角（mm），inset>0 时向内缩进（取胶带内沿 = 内框）。顺序 LL→LR→UR→UL。"""
    return np.array([[inset, inset], [w - inset, inset],
                     [w - inset, h - inset], [inset, h - inset]], np.float64)


def standard_circle(width_mm, height_mm, radius_mm, samples=72):
    """正视面坐标下、以矩形中心为圆心、半径 radius_mm 的等分圆采样点（(samples,2)）。"""
    a = np.arange(samples) * (2 * np.pi / samples)
    return np.array([width_mm / 2, height_mm / 2]) + radius_mm * np.column_stack((np.cos(a), np.sin(a)))


def front_view_size(width_mm, height_mm, short_side_px):
    """正视面目标分辨率：按物理短边固定像素数等比缩放，保证长宽比与靶纸一致。"""
    if width_mm <= 0 or height_mm <= 0 or short_side_px <= 0:
        raise ValueError("front view size needs positive dimensions")
    scale = float(short_side_px) / min(width_mm, height_mm)
    return max(8, int(round(width_mm * scale))), max(8, int(round(height_mm * scale)))


def warp_front_view(gray, rect_to_image, width_mm, height_mm, short_side_px):
    """把图像中的靶纸四边形**透视变换成正视图**（正视 = 矩形，无透视畸变）。

    矩阵口径（实测确认，别凭记忆改）：``cv2.warpPerspective`` 接受的矩阵是
    **源坐标 → 目标坐标**，即"图像像素 → 正视图像素"：

        M = mm_to_front · inv(rect_to_image)

    自检方法（`tools/test_real_scene.py` 有对应用例）：把 (0,0) 与 (w,h) 经该矩阵映射，
    应当正好落在图像里靶纸矩形的左上/右下角；若方向反了，得到的图会整片落在靶纸之外
    （表现为"正视图里只看得到背景"），而肉眼不一定立刻发现。

    正视图上一个像素对应固定的物理尺寸（短边 = `short_side_px` 像素），因此靶心恒在
    正视图中心，可直接用于可视化与"剖面法取中心"的独立校验。
    返回 (front_view, mm_to_front)；后者把物理 mm 映射到正视图像素。
    """
    fw, fh = front_view_size(width_mm, height_mm, short_side_px)
    mm_to_front = np.array([[fw / float(width_mm), 0, 0],
                            [0, fh / float(height_mm), 0],
                            [0, 0, 1]], np.float64)
    image_to_front = mm_to_front @ np.linalg.inv(np.asarray(rect_to_image, np.float64))
    view = cv2.warpPerspective(gray, image_to_front, (fw, fh), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REPLICATE)
    return view, mm_to_front


# --------------------------------------------------------------------- 采样
def sample_batch(gray, points):
    """一次 OpenCV remap 双线性采样整批点，越界由调用方拒绝。

    相比逐点 Python 循环，remap 一次调用即可采样成百上千个点（四边验证/精修的关键提速）。
    borderMode=REPLICATE 保证边缘点也能返回数值（越界判据在调用方单独做）。
    """
    shape = points.shape[:-1]
    p = points.reshape(-1, 2).astype(np.float32)
    values = cv2.remap(gray, p[:, 0].reshape(1, -1), p[:, 1].reshape(1, -1),
                       cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return values.reshape(shape)
