"""逐阶段耗时剖析：精修 / 验证 / 搜索定位 / 解中心 各占多少毫秒。

PC 与板端的绝对值不可比，但**占比**可比。上板跑一次就能看出该优化哪一段：

    python -B tools/profile_frame.py                 # 合成靶 + 无靶两种场景，各 60 帧
    python -B tools/profile_frame.py --image real_board_frame.png --repeat 100
    python -B tools/profile_frame.py --budget 33.3   # 30fps 预算，用来换算能到多少帧

输出每帧的各阶段累计毫秒与占比，并给出"按当前单帧耗时能跑多少 fps"。
"""
import argparse
import json
import statistics
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from vision.contour_locator import ContourLocator
from vision.rectangle_center import RectangleCenterDetector

ACC = {}
CALLS = {}
_STAGES = ('refine', 'verify', 'locate', 'center')


def _instrument(cls, name, key):
    original = getattr(cls, name)

    def wrapped(*args, **kwargs):
        started = time.perf_counter()
        try:
            return original(*args, **kwargs)
        finally:
            ACC[key] = ACC.get(key, 0.) + time.perf_counter() - started
            CALLS[key] = CALLS.get(key, 0) + 1
    wrapped.__name__ = name
    setattr(cls, name, wrapped)


def _reset():
    ACC.clear()
    CALLS.clear()


def _synthetic(width=240., target=True):
    """与 verify_subpixel 同口径的解析靶图；target=False 时给纯噪声（搜索态最坏情况）。"""
    if not target:
        return np.random.default_rng(7).integers(0, 256, (480, 640), dtype=np.uint8)
    from verify_subpixel import render_exact
    return render_exact(320., 240., width)[0]


def run(label, frames, config):
    det = RectangleCenterDetector(config)
    _reset()
    det.process(frames[0], 0.)                       # 首帧锁定/冷启动不计入
    _reset()
    wall = []
    valid = 0
    for i, img in enumerate(frames):
        t0 = time.perf_counter()
        out = det.process(img, 1. + i / 40.)
        wall.append((time.perf_counter() - t0) * 1000.)
        valid += bool(out.get('valid'))
    total = sum(wall)
    per_frame = total / len(frames)
    row = dict(scenario=label, frames=len(frames), valid=valid,
               frame_ms=round(statistics.median(wall), 3),
               frame_p95_ms=round(float(np.percentile(wall, 95)), 3),
               implied_fps=round(1000.0 / statistics.median(wall), 1) if wall else None,
               stages={k: round(ACC.get(k, 0.) * 1000. / len(frames), 3) for k in _STAGES},
               calls={k: round(CALLS.get(k, 0) / len(frames), 2) for k in _STAGES})
    accounted = sum(ACC.values())
    share = {k: (ACC.get(k, 0.) / accounted * 100.) for k in _STAGES if accounted > 0}
    row['share_pct'] = {k: round(v, 1) for k, v in share.items()}
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--image', type=Path, help='用真实帧（可重复）代替合成图')
    ap.add_argument('--repeat', type=int, default=60)
    ap.add_argument('--budget', type=float, default=1000. / 30.,
                    help='单帧预算毫秒，默认 30fps 的 33.3ms')
    args = ap.parse_args()
    cv2.setNumThreads(Config.OPENCV_THREADS)
    for cls, name, key in ((RectangleCenterDetector, '_refine', 'refine'),
                           (RectangleCenterDetector, '_verify', 'verify'),
                           (RectangleCenterDetector, '_center', 'center'),
                           (ContourLocator, 'locate', 'locate')):
        _instrument(cls, name, key)

    rows = []
    if args.image:
        gray = cv2.imdecode(np.fromfile(str(args.image), np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise SystemExit('无法读取 %s' % args.image)
        rows.append(run(args.image.name, [gray] * args.repeat, Config))
    else:
        for w in (120., 240., 320.):
            frames = [_synthetic(w) for _ in range(args.repeat)]
            frames = [f if i == 0 else np.roll(f, (i % 3) - 1, axis=1) for i, f in enumerate(frames)]
            rows.append(run('track w=%d' % w, frames, Config))
        rows.append(run('search (no target)', [_synthetic(0, False)] * args.repeat, Config))

    for r in rows:
        print('%-20s median=%7.2fms p95=%7.2fms  -> %5.1f fps  valid=%d/%d' % (
            r['scenario'], r['frame_ms'], r['frame_p95_ms'], r['implied_fps'], r['valid'], r['frames']))
        print('   stages ms/frame ' + '  '.join('%s=%.3f' % (k, v) for k, v in r['stages'].items()))
        print('   share %            ' + '  '.join('%s=%.0f%%' % (k, v) for k, v in r['share_pct'].items()))
        print('   calls/frame       ' + '  '.join('%s=%.2f' % (k, v) for k, v in r['calls'].items()))
        if r['frame_ms'] > args.budget:
            print('   ⚠ 超过 %.1fms 预算，缺口 %.1fms' % (args.budget, r['frame_ms'] - args.budget))
    print(json.dumps(dict(budget_ms=args.budget, rows=rows), ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
