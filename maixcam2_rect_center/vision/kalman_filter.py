"""靶心中心的卡尔曼滤波（匀加速噪声驱动的匀速模型，2D 位置 + 2D 速度）。

为什么要它（而不是继续用一阶 EMA）
--------------------------------------------------------------------------------
现场症状是"车一晃中心就偏、画面抖". 抖动有两个来源，必须区别对待：

  · **必须跟踪的真实运动** —— 底盘晃动/云台转动让靶心在图像里快速位移；
  · **必须抑制的测量噪声** —— 阈值化/边缘跳变给出的逐帧随机误差、反光造成的抖动。

一阶 EMA（`CENTER_ALPHA`）对两者一视同仁：想抑噪就得把 alpha 调小，代价是快速运动
时输出明显滞后（实测附加 ~28ms 滞后，底盘一动就变成瞄准误差）。卡尔曼用**运动模型**
把二者分开：噪声进不了速度状态，而匀速运动能被模型零滞后地跟上。

模型
--------------------------------------------------------------------------------
状态 x = [u, v, vu, vv]（位置 px、速度 px/s）；观测 z = [u, v]。
F(dt) 为匀速外推；Q 取"连续白噪声加速度"离散化形式（a ~ q），
即 Q = q² · [[dt⁴/4, dt³/2], [dt³/2, dt²]] ⊗ I₂；R = σ²I₂。
  · 过程噪声 `q`（px/s²）越大 → 越信任观测、越跟手、抑噪越弱；
  · 观测噪声 `σ`（px）越大 → 越信任模型、越平滑、滞后越大。
两者都可由 config 现场调；默认值按"板端 ~30fps、抖动幅度 1~3px"取。

鲁棒性（工程必需的几条硬规则，缺一条现场就会翻车）
--------------------------------------------------------------------------------
  1. **时钟不连续**（回退 / 间隔过长 / 首帧）→ 直接 reset 并把状态播种为本次观测，
     于是滤波输出 == 原始测量，绝不拿旧速度外推（`PREDICT_MAX_GAP_S` 口径）；
  2. **观测量跳变过大**（> reset_px）→ 判为新目标/重捕获，同样 reset 播种，
     避免滤波器用旧速度"拖"住一个已经跳走的目标；
  3. **有限性断言**：NaN/Inf 一律拒绝，绝不污染状态（板端整型/除零泄漏的教训）。
"""

import numpy as np

# 状态向量的下标语义（供外部读速度用）
U, V, VU, VV = 0, 1, 2, 3


class ConstantVelocityKalman2D:
    """2D 位置 + 速度的卡尔曼滤波；`update(z, t)` 返回后验位置，`predict_at(t)` 外推。"""

    def __init__(self, process_px=260.0, measure_px=0.9, reset_px=25.0,
                 max_predict_s=0.25, velocity_limit_px_s=6000.0):
        self.process_px = float(process_px)
        self.measure_px = float(measure_px)
        self.reset_px = float(reset_px)
        self.max_predict_s = float(max_predict_s)
        self.velocity_limit_px_s = float(velocity_limit_px_s)
        self.x = None                    # (4,) 状态；None = 尚未初始化
        self.p = None                    # (4,4) 协方差
        self.t = None                    # 上次更新时间（单调秒）
        self.resets = 0

    # ------------------------------------------------------------------ 生命周期
    def reset(self):
        """清空状态；下次 update 会重新播种。"""
        self.x = self.p = self.t = None
        return self

    @property
    def ready(self):
        return self.x is not None

    def seed(self, z, t):
        """用一次观测**播种**（不滤波）：后验 = 观测、速度未知（大协方差）。"""
        z = np.asarray(z, np.float64).reshape(2)
        if not np.all(np.isfinite(z)):
            return None
        self.x = np.array([z[0], z[1], 0.0, 0.0], np.float64)
        # 位置方差取观测噪声，速度方差取"一个观测噪声 / 一帧"的保守量级。
        self.p = np.diag([self.measure_px ** 2, self.measure_px ** 2,
                          (self.measure_px * 30.0) ** 2, (self.measure_px * 30.0) ** 2])
        self.t = float(t) if t is not None else None
        return self.x[:2].copy()

    # ------------------------------------------------------------------ 预测
    def _transition(self, dt):
        return np.array([[1.0, 0.0, dt, 0.0],
                         [0.0, 1.0, 0.0, dt],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0]], np.float64)

    def _process_noise(self, dt):
        q2 = self.process_px ** 2
        g = np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0],
                      [dt ** 3 / 2.0, dt ** 2]], np.float64) * q2
        qm = np.zeros((4, 4), np.float64)
        qm[np.ix_([U, V], [U, V])] = g
        qm[np.ix_([VU, VV], [VU, VV])] = g
        return qm

    def predict(self, t):
        """按 dt = t − 上次更新外推状态；返回预测位置 (2,)。未初始化时返回 None。"""
        if self.x is None:
            return None
        if t is None:
            return self.x[:2].copy()
        if self.t is None:
            self.t = float(t)
            return self.x[:2].copy()
        dt = float(t) - self.t
        if dt <= 0.0 or dt > self.max_predict_s:
            return None                                   # 时钟回退/断流：不可外推
        f = self._transition(dt)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + self._process_noise(dt)
        self.t = float(t)
        self._clamp_velocity()
        return self.x[:2].copy()

    def predict_at(self, t):
        """**不改状态**地外推到时刻 t（用于"取上一帧模型预测本帧位置"这类只读场景）。"""
        if self.x is None or self.t is None or t is None:
            return None
        dt = float(t) - self.t
        if not 0.0 < dt <= self.max_predict_s:
            return None
        return (self.x[:2] + self.x[2:] * dt).copy()

    # ------------------------------------------------------------------ 更新
    def update(self, z, t, gate_px=None):
        """融合一次观测，返回后验位置 (2,)。

        · 未初始化 / 时钟不连续 / 观测跳变超限 → **reset 播种**（输出 == 观测）；
        · 观测非有限 → 返回 None（保留旧状态，调用方自行决定是否判定丢帧）。
        """
        z = np.asarray(z, np.float64).reshape(2)
        if not np.all(np.isfinite(z)):
            return None
        gate = self.reset_px if gate_px is None else float(gate_px)
        if self.x is None or self.t is None:
            self.resets += 1
            return self.seed(z, t)
        dt = float(t) - self.t if t is not None else None
        if dt is None or dt <= 0.0 or dt > self.max_predict_s:
            self.resets += 1
            return self.seed(z, t)
        if gate > 0 and float(np.linalg.norm(z - self.x[:2])) > gate:
            self.resets += 1                              # 目标跳走：重建而不是硬拖
            return self.seed(z, t)
        f = self._transition(dt)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + self._process_noise(dt)
        self._clamp_velocity()
        h = np.zeros((2, 4), np.float64)
        h[0, U] = h[1, V] = 1.0
        r = np.eye(2, dtype=np.float64) * self.measure_px ** 2
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ (z - h @ self.x)
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        self.p = 0.5 * (self.p + self.p.T)                # 数值对称化，防长期漂移
        self.t = float(t)
        self._clamp_velocity()
        return self.x[:2].copy()

    # ------------------------------------------------------------------ 读取
    def position(self):
        return None if self.x is None else self.x[:2].copy()

    def velocity(self):
        return None if self.x is None else self.x[2:].copy()

    def _clamp_velocity(self):
        limit = self.velocity_limit_px_s
        if limit <= 0:
            return
        speed = float(np.hypot(self.x[VU], self.x[VV]))
        if speed > limit:
            self.x[VU] *= limit / speed
            self.x[VV] *= limit / speed
