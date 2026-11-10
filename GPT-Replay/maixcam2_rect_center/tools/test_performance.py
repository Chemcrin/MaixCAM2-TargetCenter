"""PC 回归/性能比较：python -B tools/test_performance.py [--legacy PATH]。

合成图中心真值来自独立 OpenCV 投影，不用待测实现生成预期结果。
PC 耗时只比较改动前后；不能换算为 MaixCAM2 实测帧率。
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from vision.rectangle_center import RectangleCenterDetector, draw_result


def scene(q, paper=220, tape=20, background=175, glare=False, noise=False):
    h = cv2.getPerspectiveTransform(np.float32([[0, 0], [210, 0], [210, 297], [0, 297]]), np.float32(q))
    target = np.full((298, 211), tape, np.uint8)
    target[18:280, 18:193] = paper
    cv2.circle(target, (105, 148), 38, int(paper * .72), 1)
    if glare:
        cv2.ellipse(target, (85, 13), (23, 18), 0, 0, 360, min(255, paper + 20), -1)
        cv2.ellipse(target, (120, 140), (28, 9), 25, 0, 360, 250, -1)
    result = cv2.warpPerspective(target, h, (640, 480), borderValue=background)
    if noise:
        noise_arr = np.random.default_rng(11).normal(0, 2, result.shape)
        result = np.clip(result.astype(np.float32) + noise_arr, 0, 255).astype(np.uint8)
    truth = cv2.perspectiveTransform(np.float32([[[105, 148.5]]]), h).reshape(2)
    return result, truth


def fixtures():
    rng = np.random.default_rng(20261005)
    out = []
    for i in range(24):
        q = [[rng.uniform(70,190), rng.uniform(35,100)],
             [rng.uniform(395,550), rng.uniform(30,120)],
             [rng.uniform(425,560), rng.uniform(355,445)],
             [rng.uniform(60,185), rng.uniform(335,445)]]
        paper, tape, bg = (120, 20, 95) if i % 4 == 0 else (220, 20, 175)
        if i % 4 == 1:
            bg = 20  # 黑环外沿与暗背景粘连，亮面孔必须可用。
        out.append(scene(q, paper, tape, bg, glare=i % 4 == 2, noise=True))
    # 24 roll angles expose any corner-ordering assumption, including near-square projections.
    for angle in range(0, 360, 15):
        a = np.deg2rad(angle)
        rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        q = np.array([[-105,-148.5], [105,-148.5], [105,148.5], [-105,148.5]]) @ rot.T + [320,240]
        out.append(scene(q))
    return out


def benchmark(factory, image, count=60):
    det = factory(Config)
    acquire(det, image)  # 连续确认完成后才计入稳态耗时。
    times = []
    tracked = 0
    for _ in range(count):
        t = time.perf_counter()
        r = det.process(image)
        times.append((time.perf_counter() - t) * 1000)
        tracked += r.get('stage') == 'track'
    return dict(median_ms=round(float(np.median(times)), 3),
                p95_ms=round(float(np.percentile(times, 95)), 3), tracked=tracked)


def acquire(det, image, limit=120):
    for i in range(limit):
        result = det.process(image)
        if result['valid']:
            return result, i + 1
    return result, limit


def run():
    cv2.setNumThreads(Config.OPENCV_THREADS)
    data = fixtures()
    errors, failed = [], []
    acquisition_frames = []
    cold_times = []
    for i, (image, truth) in enumerate(data):
        detector = RectangleCenterDetector(Config)
        t0 = time.perf_counter()
        result, frames = acquire(detector, image)
        cold_times.append((time.perf_counter() - t0) * 1000)
        acquisition_frames.append(frames)
        if not result.get('valid'):
            failed.append(i)
        else:
            errors.append(float(np.linalg.norm([result['raw_u'] - truth[0], result['raw_v'] - truth[1]])))
    negatives = [np.full((480,640), v, np.uint8) for v in (20,120,220)]
    bright_box = np.full((480,640), 175, np.uint8)
    cv2.rectangle(bright_box, (150,70), (460,400), 225, -1)
    negatives.append(bright_box)
    dark_box = np.full((480,640), 175, np.uint8)
    cv2.rectangle(dark_box, (150,70), (460,400), 20, -1)
    negatives.append(dark_box)
    # Textured scene with edges and reflections but no target frame.
    yy, xx = np.mgrid[:480, :640]
    texture = np.clip(145 + 14 * np.sin(xx / 19.0) + 11 * np.cos(yy / 13.0), 0, 255).astype(np.uint8)
    cv2.rectangle(texture, (40, 30), (250, 220), 20, 3)
    cv2.rectangle(texture, (330, 50), (600, 430), 232, 2)
    cv2.line(texture, (0, 360), (640, 290), 36, 5)
    negatives.append(texture)
    false_positives = sum(acquire(RectangleCenterDetector(Config), im)[0]['valid'] for im in negatives)
    negative_results = [RectangleCenterDetector(Config).process(im) for im in negatives]

    base_q = np.array([[180,70],[445,80],[475,400],[150,420]], np.float32)
    det = RectangleCenterDetector(Config)
    acquire(det, scene(base_q)[0])
    movement_errors, stages = [], []
    for dx in (0,3,6,9,12,15,18,21,24,27,90,94,98):
        im, truth = scene(base_q + [dx,0])
        r = det.process(im)
        if not r['valid']:
            r, _ = acquire(det, im)
        stages.append(r['stage'])
        movement_errors.append(float(np.linalg.norm(np.array([r.get('raw_u',-999),r.get('raw_v',-999)]) - truth)))
    lost = det.process(negatives[-1])
    reacquired, _ = acquire(det, data[0][0])

    # Same object rotates between adjacent frames: corners move, but the
    # detector must keep using cyclic contour order and current-frame pixels.
    rotation_detector = RectangleCenterDetector(Config)
    rotation_errors, rotation_invalid = [], []
    for angle in range(0, 360, 15):
        rad = np.deg2rad(angle)
        rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
        quad = (np.array([[-100,-140], [100,-140], [100,140], [-100,140]]) @ rot.T + [320,240])
        im, truth = scene(quad)
        r = rotation_detector.process(im)
        rotation_errors.append(float(np.hypot(r.get('raw_u', -1)-truth[0],
                                              r.get('raw_v', -1)-truth[1])) if r['valid'] else 999.0)
        rotation_invalid.append(not r['valid'])

    # Resolution scaling checks that image-pixel floor gates do not silently
    # reject an otherwise identical target on a smaller Maix camera profile.
    small_errors, small_failed = [], []
    for index, (im, truth) in enumerate(data[:12]):
        small = cv2.resize(im, (320, 240), interpolation=cv2.INTER_AREA)
        r, _ = acquire(RectangleCenterDetector(Config), small)
        if not r['valid']:
            small_failed.append(index)
        else:
            small_errors.append(float(np.hypot(r['raw_u']-truth[0]*0.5,
                                                r['raw_v']-truth[1]*0.5)))

    blank = np.full((480,640), 120, np.uint8)
    blank_detector = RectangleCenterDetector(Config)
    blank_times = []
    for _ in range(8):
        start = time.perf_counter()
        blank_result = blank_detector.process(blank)
        blank_times.append((time.perf_counter()-start)*1000)
    blank_no_target = not blank_result['valid']
    report = dict(fixtures=len(data), detected=len(errors), failed=failed,
                  max_center_error_px=round(max(errors, default=999),3),
                  acquisition_frames_max=max(acquisition_frames, default=0),
                  cold_search_ms_p95=round(float(np.percentile(cold_times, 95)), 3),
                  rotation_sweep_frames=len(rotation_errors), rotation_invalid=sum(rotation_invalid),
                  rotation_max_error_px=round(max(rotation_errors),3),
                  small_resolution_cases=len(data[:12]), small_resolution_failed=small_failed,
                  small_resolution_max_error_px=round(max(small_errors, default=999),3),
                  negative_cases=len(negatives), false_positives=false_positives,
                  negative_search_max=max(r.get('search_count', 0) for r in negative_results),
                  negative_threshold_max=max(r.get('thresholds', 0) for r in negative_results),
                  blank_no_target=blank_no_target,
                  blank_search_max=max(blank_result.get('search_count', 0), 0),
                  blank_ms_p95=round(float(np.percentile(blank_times, 95)),3),
                  motion_max_error_px=round(max(movement_errors),3), motion_stages=stages,
                  lost_valid=lost['valid'], reacquired=reacquired['valid'],
                  optimized=benchmark(RectangleCenterDetector, data[0][0]))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    assert not failed, 'missed rotation/perspective fixture'
    assert max(errors) < 2, 'corner/center regression'
    assert max(acquisition_frames) <= 3, 'cold acquisition took too many frames'
    assert not any(rotation_invalid) and max(rotation_errors) < 2, 'rapid rotation tracking failed'
    assert not small_failed and max(small_errors) < 2, 'low-resolution geometry failed'
    assert blank_no_target and blank_result.get('search_count', 0) <= 2, 'blank scene search unbounded'
    assert not false_positives, 'solid rectangles falsely accepted'
    assert not any(r['valid'] for r in negative_results), 'negative scene false positive'
    assert report['negative_search_max'] <= 2, 'unbounded full-frame search'
    assert report['negative_threshold_max'] <= 1, 'threshold sweep regression'
    assert max(movement_errors) < 3, 'motion tracking drift'
    assert not lost['valid'] and reacquired['valid'], 'loss/reacquisition failed'
    assert report['optimized']['tracked'] == 60, 'steady state must not scan thresholds'


if __name__ == '__main__':
    run()
