"""中心卡尔曼滤波回归：python -B tools/test_kalman.py。

锁住的是"抑抖"与"跟手"这对相互拉扯的指标 —— 只看一项很容易把另一项调坏
（项目既有教训，见根 AGENTS.md 的"跷跷板"说明）：

    1. **跟手**：匀速运动下输出误差必须接近 0（一阶 EMA 会明显滞后，本用例给出对照）；
       匀加速（1500px/s²）下误差仍须 ~1px —— 过程噪声挂在加速度上，机动由模型吸收；
    2. **抑噪**：随机测量噪声下输出的标准差必须显著小于输入；
    3. **鲁棒**：时钟不连续 / 观测跳变 / NaN 一律"重置或拒绝"，绝不拿旧速度外推；
    4. **限幅**：速度状态有上界，防误检把速度甩飞。

PC 耗时只作诊断，不代表 MaixCAM2 实测。
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from config import Config                                     # noqa: E402
from vision.kalman_filter import ConstantVelocityKalman2D     # noqa: E402
from vision.rectangle_center import RectangleCenterDetector   # noqa: E402
from test_performance import scene                            # noqa: E402
import test_tracking as tk                                    # noqa: E402

FAILURES = []


def check(condition, label):
    if not condition:
        FAILURES.append(label)
    print("  %s %s" % ("OK  " if condition else "FAIL", label))


def make(**overrides):
    params = dict(process_px=Config.KALMAN_PROCESS_PX, measure_px=Config.KALMAN_MEASURE_PX,
                  reset_px=Config.KALMAN_RESET_PX, max_predict_s=Config.KALMAN_MAX_PREDICT_S,
                  velocity_limit_px_s=Config.KALMAN_VELOCITY_LIMIT_PX_S)
    params.update(overrides)
    return ConstantVelocityKalman2D(**params)


def test_constant_velocity_has_no_lag():
    """匀速运动：模型能零滞后跟上；一阶 EMA（CENTER_ALPHA）作对照，滞后明显。"""
    fps, speed, frames = 30.0, 240.0, 90          # 240px/s ≈ 8px/帧，与实机抖动/运动同量级
    truth, measure = [], []
    for i in range(frames):
        truth.append(np.array([120.0 + speed * i / fps, 200.0]))
        measure.append(truth[-1] + np.array([0.6, -0.4]))     # 恒定小偏差，模拟固定测量偏置
    kalman, ema, smooth = make(), None, None
    kalman_err, ema_err = [], []
    for i, z in enumerate(measure):
        t = i / fps
        est = kalman.update(z, t)
        smooth = z if smooth is None else smooth + Config.CENTER_ALPHA * (z - smooth)
        if i > 10:                                            # 跳过收敛段
            kalman_err.append(float(np.linalg.norm(est - truth[i])))
            ema_err.append(float(np.linalg.norm(smooth - truth[i])))
    kalman_err, ema_err = np.asarray(kalman_err), np.asarray(ema_err)
    check(kalman_err.max() < 1.2,
          "匀速运动跟手：卡尔曼误差 max=%.2fpx（< 1.2）" % kalman_err.max())
    check(kalman_err.mean() < ema_err.mean() * 0.5,
          "必须比一阶 EMA 更跟手：卡尔曼 mean=%.2f vs EMA mean=%.2f"
          % (kalman_err.mean(), ema_err.mean()))


def test_acceleration_is_followed():
    """机动跟随：匀加速（1500px/s²，约等于云台一个步进的量级）下误差仍须 ~1px。

    这条是"卡尔曼会不会变成新的滞后源"的反证：CV 模型的过程噪声挂在**加速度**上，
    所以速度变化由模型自己吸收，不靠"降低增益去跟"，实测 q 从 40 到 2000 误差都在 0.8px。
    """
    rng = np.random.default_rng(11)
    fps, accel = 30.0, 1500.0
    kalman = make()
    position = velocity = 0.0
    errors = []
    for i in range(300):
        velocity += accel / fps
        position += velocity / fps
        estimate = kalman.update(np.array([position, 200.0]) + rng.normal(0, 1.0, 2), i / fps)
        if i > 30:
            errors.append(abs(float(estimate[0]) - position))
    errors = np.asarray(errors)
    check(errors.mean() < 1.0 and errors.max() < 4.0,
          "匀加速跟随：mean=%.2fpx max=%.2fpx（需 mean<1.0 / max<4.0）"
          % (errors.mean(), errors.max()))


def test_jitter_is_suppressed():
    """抖动抑制：随机测量噪声下输出的标准差必须明显小于输入（否则滤波没起作用）。"""
    rng = np.random.default_rng(20261013)
    fps, frames = 30.0, 240
    truth = np.array([320.0, 240.0])
    noises = rng.normal(0.0, 1.2, (frames, 2))
    kalman, outputs = make(), []
    for i in range(frames):
        kalman.update(truth + noises[i], i / fps)
        if i > 30:
            outputs.append(kalman.position())
    outputs = np.asarray(outputs)
    input_std = float(np.std(noises[31:]))
    output_std = float(np.std(outputs - truth))
    check(output_std < input_std * 0.58,
          "静止抖动被抑制：输入 σ=%.3f → 输出 σ=%.3f（需 < 0.6 倍）" % (input_std, output_std))


def test_discontinuity_and_outliers_reset():
    """时钟不连续 / 观测跳变 → **重置播种**（输出 == 本次观测，绝不外推）。"""
    kalman = make()
    kalman.update(np.array([100.0, 100.0]), 1.000)
    kalman.update(np.array([105.0, 101.0]), 1.033)
    check(kalman.ready, "连续两帧后滤波器应处于就绪态")
    out = kalman.update(np.array([140.0, 130.0]), 9.500)      # 断流 8.5s
    check(np.allclose(out, [140.0, 130.0]), "断流后必须重置播种（输出==观测），实际 %s" % out)
    out = kalman.update(np.array([141.0, 131.0]), 9.533)
    check(kalman.velocity() is not None and np.linalg.norm(kalman.velocity()) < 900.0,
          "重置后速度必须从零重建，实际 %s" % (kalman.velocity(),))
    out = kalman.update(np.array([900.0, 900.0]), 9.566)      # 位置跳 1000+ px
    check(np.allclose(out, [900.0, 900.0]), "观测跳变超限必须重建（输出==观测），实际 %s" % out)
    check(kalman.resets >= 2, "两个重置场景都应被计数，实际 resets=%d" % kalman.resets)
    before = kalman.position().copy()
    check(kalman.update(np.array([np.nan, 10.0]), 9.600) is None, "NaN 观测必须被拒绝")
    check(np.allclose(kalman.position(), before), "拒绝 NaN 后状态不得被污染")
    check(kalman.update(np.array([np.inf, 10.0]), 9.633) is None, "Inf 观测必须被拒绝")


def test_velocity_is_limited():
    """速度限幅：连续的巨幅跳变不得把速度状态甩飞（防误检驱动错误预测）。"""
    kalman = make(velocity_limit_px_s=1000.0, reset_px=100000.0)   # 关掉跳变重建，单看限幅
    for i in range(40):
        kalman.update(np.array([320.0 + 400.0 * i, 240.0]), i / 30.0)
    speed = float(np.linalg.norm(kalman.velocity()))
    check(speed <= 1000.0 + 1e-6, "速度必须被限幅在 1000px/s 内，实际 %.1f" % speed)


def test_detector_wiring():
    """检测器集成：连续时钟下 u 走滤波、raw_u 走测量；时钟不连续时两者相等（口径统一）。"""
    image, truth = scene(tk.SMALL_Q)
    detector = RectangleCenterDetector(Config)
    detector.process(image, timestamp=2.00)
    second = detector.process(image, timestamp=2.02)
    check(second["valid"] and second["stage"] == "track", "第二帧应走跟踪")
    # 时钟回退/断流：滤波器必须重新播种 → 平滑输出 == 原始测量
    detector.process(image, timestamp=2.04)
    back = detector.process(image, timestamp=2.00)
    check(np.allclose([back["u"], back["v"]], [back["raw_u"], back["raw_v"]]),
          "时钟不连续必须让滤波输出回到原始测量（u=%.3f/%.3f raw=%.3f/%.3f）"
          % (back["u"], back["v"], back["raw_u"], back["raw_v"]))
    # 关闭滤波时应退回 EMA 口径（保留的现场对照能力）
    class NoKalman(Config):
        KALMAN_ENABLE = False
    plain = RectangleCenterDetector(NoKalman)
    plain.process(image)
    check(plain._kalman is None, "KALMAN_ENABLE=False 时不得创建滤波器")
    again = plain.process(image)
    check(again["valid"], "关闭滤波后仍必须正常工作（退回 EMA）")
    # 预测接口：有速度后应能外推，且不得改变内部状态
    detector.process(image, timestamp=2.06)
    before = detector._kalman.position().copy()
    predicted = detector.predict_center(2.09)
    check(predicted is not None and np.all(np.isfinite(predicted)),
          "应能给出有限的外推位置，实际 %s" % predicted)
    check(np.allclose(detector._kalman.position(), before),
          "predict_center 不得改变滤波器状态")


def main():
    for test in (test_constant_velocity_has_no_lag, test_acceleration_is_followed,
                 test_jitter_is_suppressed,
                 test_discontinuity_and_outliers_reset, test_velocity_is_limited,
                 test_detector_wiring):
        print("[%s]" % test.__name__)
        test()
    if FAILURES:
        print("\nkalman regression: %d failure(s): %s" % (len(FAILURES), FAILURES))
        return 1
    print("\nkalman regression: PASS (6)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
