"""动态视觉回归：python -B tools/test_tracking.py。

靶心真值由合成场景 helper 中的 OpenCV 投影提供；覆盖移动、转向、
采样间隔变化、四角循环换位、内外沿切换以及摄像头分辨率变化。
PC 耗时只作诊断，不代表 MaixCAM2 实测帧率。
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config import Config
from tools.test_performance import scene
from vision.rectangle_center import RectangleCenterDetector


SMALL_Q = np.float32([[100, 90], [240, 90], [240, 300], [100, 300]])
BASE_Q = np.float32([[180, 70], [445, 80], [475, 400], [150, 420]])


class Regression:
    def __init__(self):
        self.failures = []
        self.report = {}

    def check(self, condition, label):
        if not condition:
            self.failures.append(label)

    def measure(self, detector, image, truth, timestamp=None):
        result = detector.process(image, timestamp=timestamp)
        error = None
        if result["valid"]:
            error = float(np.linalg.norm(np.array([result["raw_u"], result["raw_v"]]) - truth))
            self.check(error < 3.0, "center error exceeds 3 px")
        proposals = int(result.get("proposals", 0))
        # ★ v0.6.4：候选预算**只在有界口径**（SEARCH_EXHAUSTIVE=False）下是硬约束。
        #   默认的穷尽口径（对齐 v1–v3）刻意不截断候选 —— 它正是"一点不丢"的来源，
        #   所以不能拿预算上限去断言它（否则等于把要修的行为又钉回去）。
        if not bool(getattr(detector.cfg, "SEARCH_EXHAUSTIVE", True)):
            self.check(proposals <= detector.cfg.SEARCH_MAX_PROPOSALS,
                       "per-frame candidate budget exceeded")
        observation = dict(valid=bool(result["valid"]), stage=result["stage"],
                           error_px=None if error is None else round(error, 3),
                           searches=int(result.get("search_count", 0)), proposals=proposals)
        return result, observation

    def sequence(self, name, quad, positions, timestamps=None):
        detector = RectangleCenterDetector(Config)
        frames = []
        for index, position in enumerate(positions):
            image, truth = scene(quad + [position, 0])
            timestamp = timestamps[index] if timestamps is not None else None
            _, observation = self.measure(detector, image, truth, timestamp)
            frames.append(observation)
        self.report[name] = dict(valid_frames=sum(frame["valid"] for frame in frames),
                                 frames=len(frames), observations=frames)
        self.check(all(frame["valid"] for frame in frames), name + " loses a visible target")
        return detector, frames


def test_motion(regression):
    _, jumps = regression.sequence("large_jump", SMALL_Q, [0, 0, 220, 220])
    regression.check(jumps[2]["valid"] and jumps[2]["stage"] == "full_search",
                     "failed local search must reacquire a distant target in the same frame")
    regression.sequence("fast_reversal", SMALL_Q, [0, 20, 40, 60, 30, 0, -30, -60])
    _, variable = regression.sequence("variable_interval", SMALL_Q,
                                      [0, 20, 80, 100, 70, 40],
                                      [1.0, 1.02, 1.08, 1.10, 1.12, 1.14])
    regression.check(variable[2]["valid"] and variable[2]["stage"] == "track",
                     "constant physical velocity with a longer sample interval should remain on track")


def test_perspective_jitter(regression):
    detector = RectangleCenterDetector(Config)
    rng = np.random.default_rng(20261012)
    center = BASE_Q.mean(axis=0)
    frames = []
    movements = zip((0, 12, -10, 16, -16, 8, 0), (0, 2, -2, 3, -3, 1, 0))
    for index, (position, angle) in enumerate(movements):
        radians = np.deg2rad(angle)
        rotation = np.array([[np.cos(radians), -np.sin(radians)],
                             [np.sin(radians), np.cos(radians)]])
        quad = ((BASE_Q - center) @ rotation.T + [320 + position, 240] +
                rng.uniform(-4, 4, (4, 2)))
        image, truth = scene(quad, noise=True)
        result, observation = regression.measure(detector, image, truth, 6.0 + index * .03)
        regression.check(result["valid"], "perspective deformation and jitter must retain the target")
        frames.append(observation)
    regression.report["perspective_jitter"] = dict(valid_frames=sum(frame["valid"] for frame in frames),
                                                   frames=len(frames), observations=frames)


def test_loss_and_negatives(regression):
    detector = RectangleCenterDetector(Config)
    image, truth = scene(SMALL_Q)
    regression.measure(detector, image, truth, 2.0)
    frames = []
    for index, brightness in enumerate((20, 120, 220)):
        blank = np.full((480, 640), brightness, np.uint8)
        result, observation = regression.measure(detector, blank, truth, 2.02 + index * .02)
        regression.check(not result["valid"], "blank frame must invalidate a previous lock")
        frames.append(observation)
    solid = np.full((480, 640), 175, np.uint8)
    cv2.rectangle(solid, (150, 70), (460, 400), 20, -1)
    result, observation = regression.measure(detector, solid, truth, 2.08)
    regression.check(not result["valid"], "solid black rectangle must remain rejected after loss")
    frames.append(observation)
    moved, moved_truth = scene(SMALL_Q + [220, 0])
    result, observation = regression.measure(detector, moved, moved_truth, 2.50)
    regression.check(result["valid"], "first visible frame after loss must reacquire")
    frames.append(observation)
    regression.report["loss_and_reacquisition"] = dict(observations=frames)


def test_resolution_change(regression):
    detector = RectangleCenterDetector(Config)
    image, truth = scene(BASE_Q)
    _, initial = regression.measure(detector, image, truth, 3.0)
    frames = [initial]
    for index, size in enumerate(((80, 60), (640, 480))):
        resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        # resize maps pixel centers, so include its half-pixel translation in the truth.
        resized_truth = (truth + .5) * np.array(size) / [640, 480] - .5
        try:
            result, observation = regression.measure(detector, resized, resized_truth, 3.02 + index * .02)
            regression.check(result["valid"], "resolution change must reacquire immediately")
            frames.append(observation)
        except (cv2.error, ValueError) as error:
            regression.check(False, "resolution change raises " + type(error).__name__)
            frames.append(dict(valid=False, stage="exception", exception=type(error).__name__))
    regression.report["resolution_change"] = dict(observations=frames)


def test_outside_prediction(regression):
    class LargePredict(Config):
        PREDICT_MAX_PX = 1000.0

    reports = []
    for positions in ((180, 380, 100), (180, -80, 100)):
        detector = RectangleCenterDetector(LargePredict)
        frames = []
        for position in positions:
            image, truth = scene(SMALL_Q + [position, 0])
            try:
                result, observation = regression.measure(detector, image, truth)
                regression.check(result["valid"], "off-screen motion prediction must fall back to full search")
                frames.append(observation)
            except (cv2.error, ValueError, ZeroDivisionError) as error:
                regression.check(False, "off-screen predicted ROI raises " + type(error).__name__)
                frames.append(dict(valid=False, stage="exception", exception=type(error).__name__))
        reports.append(dict(positions=positions, observations=frames))
    regression.report["outside_prediction"] = reports


def test_clock_discontinuity(regression):
    detector = RectangleCenterDetector(Config)
    for position, timestamp in ((0, 7.0), (5, 7.02), (10, 7.04)):
        image, truth = scene(SMALL_Q + [position, 0])
        regression.measure(detector, image, truth, timestamp)
    regression.check(detector._motion_delta() is not None, "clock fixture must establish motion first")
    frames = []
    clocks = (("backward", 7.01), ("duplicate", 7.01), ("implicit_clock", None),
              ("explicit_clock", 8.0), ("long_gap", 8.50))
    for index, (reason, timestamp) in enumerate(clocks):
        image, truth = scene(SMALL_Q + [15 + index * 5, 0])
        result, observation = regression.measure(detector, image, truth, timestamp)
        regression.check(result["valid"], "clock discontinuity must preserve current-frame detection")
        regression.check(detector._motion_delta() is None, "clock discontinuity must discard old velocity")
        if result["valid"]:
            regression.check(np.allclose([result["u"], result["v"]], [result["raw_u"], result["raw_v"]]),
                             "clock discontinuity must reset stale center smoothing")
        observation["reason"] = reason
        frames.append(observation)
    regression.report["clock_discontinuity"] = dict(observations=frames)


def test_shared_proposal_budget(regression):
    # ★ v0.8：候选提议改成 Canny 轮廓（1 张掩膜）+ Otsu 兜底（第 2 张）后，
    #   本帧的候选数从"旧阈值档位算出来的 4/5"变成"定位器实际提议出来的 10 / 11"，
    #   所以预算标定值随之更新（下表的 10 / 11 是实测值，不是拍的）。
    #   为什么 MIN_EDGE_CONTRAST=100：本固件在原位还贴了一张 paper=100 的**弱靶纸**，
    #   抬高对比度门槛让它不可验证，才能逼出"跟踪失败 → ROI 搜索 → 全图搜索"这条真路径。
    #   候选预算**只在有界口径**（SEARCH_EXHAUSTIVE=False）下生效；默认的穷尽口径不截断。
    class BudgetConfig(Config):
        MIN_EDGE_CONTRAST = 100.0
        SEARCH_EXHAUSTIVE = False

    far, truth = scene(SMALL_Q + [220, 0])
    weak, _ = scene(SMALL_Q, paper=100)
    weak_mask = weak != 175
    far[weak_mask] = weak[weak_mask]
    frames = []
    # (cap, 期望 valid, 期望 search_count)：ROI 搜索恰好要 10 个候选才试完，
    # 全图搜索只要 1 个就命中（低分辨率定位 + "四边齐备即早退"）。
    for cap, valid, searches in ((10, False, 1), (11, True, 2)):
        cfg = type("Budget", (BudgetConfig,), dict(SEARCH_MAX_PROPOSALS=cap))
        detector = RectangleCenterDetector(cfg)
        initial, initial_truth = scene(SMALL_Q)
        acquired, _ = regression.measure(detector, initial, initial_truth)
        regression.check(acquired["valid"], "budget fixture must acquire an initial lock")
        result, observation = regression.measure(detector, far, truth)
        observation["cap"] = cap
        regression.check(result["proposals"] == cap,
                         "ROI 与全图搜索必须共享同一个候选计数器：cap=%d 实际 proposals=%d"
                         % (cap, result["proposals"]))
        regression.check(bool(result["valid"]) is valid and result["search_count"] == searches,
                         "cap=%d 期望 valid=%s/search_count=%d，实际 valid=%s/search_count=%d"
                         % (cap, valid, searches, result["valid"], result["search_count"]))
        if valid:
            regression.check(result["stage"] == "full_search",
                             "剩余预算必须允许同帧全图重捕获，stage=%s" % result["stage"])
            regression.check(float(np.hypot(result["raw_u"] - truth[0],
                                            result["raw_v"] - truth[1])) < 2.0,
                             "重捕获中心误差必须 < 2px")
        frames.append(observation)
    regression.report["shared_proposal_budget"] = dict(observations=frames)


def test_exhaustive_search_never_truncates_candidates(regression):
    """穷尽口径（默认，对齐 v1–v3）：候选预算不得截断搜索、不得跳过全图搜索。

    复现的是用户实报的丢帧机制：ROI 搜索把候选预算烧光后，旧守卫会**跳过**全图搜索，
    于是"画面里黑框清清楚楚"却判 no_black_frame。这里把预算压到 1（远小于真实候选数），
    穷尽口径下仍必须找回目标，且实际验证的候选数明显超过该预算。
    """
    class ExhaustiveConfig(Config):
        MIN_EDGE_CONTRAST = 100.0      # 同上：让原位那张弱靶纸不可验证
        SEARCH_EXHAUSTIVE = True
        SEARCH_MAX_PROPOSALS = 1

    far, truth = scene(SMALL_Q + [220, 0])
    weak, _ = scene(SMALL_Q, paper=100)
    weak_mask = weak != 175
    far[weak_mask] = weak[weak_mask]
    detector = RectangleCenterDetector(ExhaustiveConfig)
    initial, initial_truth = scene(SMALL_Q)
    acquired, _ = regression.measure(detector, initial, initial_truth)
    regression.check(acquired["valid"], "exhaustive fixture must acquire an initial lock")
    result, observation = regression.measure(detector, far, truth)
    regression.check(result["valid"], "穷尽口径下不得因候选预算而丢帧")
    regression.check(result["stage"] == "full_search",
                     "跟踪/ROI 失败后必须真的跑完全图搜索，stage=%s" % result["stage"])
    regression.check(result["proposals"] > ExhaustiveConfig.SEARCH_MAX_PROPOSALS,
                     "穷尽口径必须验证超过预算的候选，proposals=%d" % result["proposals"])
    regression.report["exhaustive_search"] = dict(observations=[observation])


def rotated_quad(angle):
    radians = np.deg2rad(angle)
    rotation = np.array([[np.cos(radians), -np.sin(radians)],
                         [np.sin(radians), np.cos(radians)]])
    return np.float32([[-70, -99], [70, -99], [70, 99], [-70, 99]]) @ rotation.T + [320, 240]


def inner_corners(quad):
    transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]),
                                           np.float32(quad))
    return cv2.perspectiveTransform(np.float32([[[18, 18], [192, 18], [192, 279], [18, 279]]]),
                                   transform).reshape(4, 2)


def test_corner_correspondence(regression):
    detector = RectangleCenterDetector(Config)
    frames = []
    for index, angle in enumerate((35, 40, 70)):
        image, truth = scene(rotated_quad(angle))
        result, observation = regression.measure(detector, image, truth, 4.0 + index * .02)
        frames.append(observation)
        regression.check(result["valid"], "rotating target must remain visible")
    delta = detector._motion_delta()
    if result["valid"] and delta is not None:
        # The x+y starting corner changes past 45 degrees. Associate the first measured
        # corner with physical paper coordinates, rather than copying detector ordering.
        current_truth = inner_corners(rotated_quad(70))
        prior_truth = inner_corners(rotated_quad(40))
        first = np.asarray(result["detected_quad"])[0]
        physical_index = int(np.argmin(np.linalg.norm(current_truth - first, axis=1)))
        physical_motion = current_truth[physical_index] - prior_truth[physical_index]
        regression.check(float(np.dot(delta[0], physical_motion)) > 0,
                         "canonical corner switch must preserve physical motion direction")
        regression.check(np.linalg.norm(delta[0] - physical_motion) < 4.0,
                         "corner velocity must compare the same physical corner")
    else:
        regression.check(False, "same-kind reacquisition should retain usable motion history")
    regression.report["corner_correspondence"] = dict(observations=frames,
                                                       first_corner_delta=None if delta is None else
                                                       np.round(delta[0], 3).tolist())


def test_edge_kind_change(regression):
    """内外框（kind）切换不得伪造角点速度。

    v0.8 起候选由 Canny 轮廓提议：同一个框的 "inner"/"outer" 两种假设都会被尝试，
    最终是哪种由"哪条轮廓先验过"决定。本固件三帧稳定在外框（见下面的 kinds 断言）。
    契约不变：**kind 一旦在相邻帧之间变化，运动历史必须作废** —— 否则会把"胶带厚度"
    或"四角编号循环移位"当成真实位移，把错误速度外推进预测位置。
    """
    quad = np.float32([[82.53907855, 83.73233729], [444.45757880, 35.14604327],
                       [445.36120184, 398.24090161], [148.89773265, 365.83540159]])
    detector = RectangleCenterDetector(Config)
    frames, kinds = [], []
    for index, (position, glare) in enumerate(((0, False), (0, False), (30, True))):
        image, truth = scene(quad + [position, 0], glare=glare, noise=True)
        result, observation = regression.measure(detector, image, truth, 5.0 + index * .02)
        regression.check(result["valid"], "glare and translation must not lose the target")
        kinds.append(result.get("kind"))
        frames.append(observation)
    regression.check(len(set(kinds)) == 1, "本固件的 kind 必须稳定（fixture 语义），实际 %s" % kinds)
    regression.check(detector._track["kind"] == kinds[-1], "锁定框的 kind 必须与结果一致")
    # 反面对照：kind 不变且时钟连续 → 运动历史必须可用（否则"翻转即丢弃"可能是假阳性）
    regression.check(detector._motion_delta() is not None,
                     "同 kind 的连续帧必须保留运动历史（速度预测的前提）")
    # 正面对照：把上一帧记录的 kind 翻转过来 → 本帧视为"内外框切换"，运动历史必须作废。
    flipped = dict(detector._track)
    flipped["kind"] = "inner" if flipped["kind"] == "outer" else "outer"
    detector._track = flipped
    image, truth = scene(quad + [60, 0], glare=False, noise=True)
    result, observation = regression.measure(detector, image, truth, 5.10)
    regression.check(result["valid"], "kind 翻转后必须重新锁定")
    regression.check(detector._motion_delta() is None,
                     "kind 切换必须丢弃角点速度")
    regression.report["edge_kind_change"] = dict(kinds=kinds, observations=frames)


def main():
    cv2.setNumThreads(Config.OPENCV_THREADS)
    regression = Regression()
    for test in (test_motion, test_perspective_jitter, test_loss_and_negatives, test_resolution_change,
                 test_outside_prediction, test_clock_discontinuity, test_shared_proposal_budget,
                 test_exhaustive_search_never_truncates_candidates,
                 test_corner_correspondence, test_edge_kind_change):
        test(regression)
    regression.report["failures"] = regression.failures
    print(json.dumps(regression.report, ensure_ascii=False, indent=2))
    if regression.failures:
        raise SystemExit("tracking regression: %d failure(s)" % len(regression.failures))
    print("tracking regression: PASS")


if __name__ == "__main__":
    main()
