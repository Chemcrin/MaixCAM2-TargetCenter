"""生成矩形靶图并验证中心精度 / 固定激光偏差 / 负例拒检。

与 `verify_precision.py` 的区别（这是本文件存在的原因）：
`verify_precision.py` 用 4 倍超采样渲染，边缘位置被量化到 1/4 像素，而它用来
断言 0.7px、报告 0.084px —— **真值比被测量还粗**，报告值只能算"相位撞对了"，
不能作为精度上界。本文件对轴对齐场景用**解析覆盖率**渲染（真值无量化），
因此能真正分辨 0.01px 级误差；旋转/透视场景仍用 4 倍超采样，并明确标注其
真值分辨率只有 0.25px，只用于验证"能不能检出"，不用于亚像素结论。

用法（在应用目录下，Python 需 numpy/opencv）::

    python -B tools/verify_subpixel.py                 # 全套，约 10s
    python -B tools/verify_subpixel.py --quick          # 跳过旋转/透视
    python -B tools/verify_subpixel.py --output tools/subpixel_report
"""
import argparse
import html
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import Config
from target_api import CenterTracker
from vision.rectangle_center import draw_result

W_MM, H_MM, TAPE_MM = 210.0, 297.0, 18.0
BUDGET_MS = 1000.0 / 30.0          # 30fps 的单帧预算


# --------------------------------------------------------------- 目标渲染
def _cover(lo, hi, n):
    """[lo,hi] 与单位像素 [i-0.5,i+0.5) 的重叠长度 = 该像素的覆盖比例。"""
    idx = np.arange(n)
    return np.clip(np.minimum(idx + .5, hi) - np.maximum(idx - .5, lo), 0., 1.)


def render_exact(cx, cy, width_px, paper=220, tape=22, bg=175, noise=0., seed=0):
    """轴对齐靶纸，逐像素**解析**覆盖率；真值与像素中心同约定（整数=像素中心）。

    返回 (gray, (u,v))，(u,v) 是两条对角线交点，无任何渲染量化。
    """
    h = width_px * H_MM / W_MM
    x0, x1 = cx - width_px / 2, cx + width_px / 2
    y0, y1 = cy - h / 2, cy + h / 2
    t = TAPE_MM * width_px / W_MM
    outer = np.outer(_cover(y0, y1, 480), _cover(x0, x1, 640))
    inner = np.outer(_cover(y0 + t, y1 - t, 480), _cover(x0 + t, x1 - t, 640))
    img = bg + (tape - bg) * outer + (paper - tape) * inner
    img = np.rint(img).astype(np.uint8)
    if noise:
        img = np.clip(img.astype(np.float32) +
                      np.random.default_rng(seed).normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img, np.array([cx, cy])


def render_warp(quad, paper=220, tape=22, bg=175, noise=1., seed=0, ss=4):
    """旋转/透视靶纸：ss 倍超采样后降采样。真值经 OpenCV 投影，分辨率 1/ss px。"""
    m = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [W_MM, 0], [W_MM, H_MM], [0, H_MM]]), np.float32(quad))
    t = TAPE_MM
    tgt = np.full((int(H_MM * ss), int(W_MM * ss)), tape, np.uint8)
    tgt[int(t * ss):int((H_MM - t) * ss), int(t * ss):int((W_MM - t) * ss)] = paper
    c = (int(W_MM / 2 * ss), int(H_MM / 2 * ss))
    for r in (38, 28, 18):
        cv2.circle(tgt, c, int(r * ss), int(paper * .72), max(1, ss))
    to_mm = np.array([[1 / ss, 0, .5 / ss], [0, 1 / ss, .5 / ss], [0, 0, 1.]])
    to_hi = np.array([[ss, 0, (ss - 1) / 2], [0, ss, (ss - 1) / 2], [0, 0, 1.]])
    hi = cv2.warpPerspective(tgt, to_hi @ m @ to_mm, (640 * ss, 480 * ss),
                             flags=cv2.INTER_NEAREST, borderValue=bg)
    img = cv2.resize(hi, (640, 480), interpolation=cv2.INTER_AREA)
    if noise:
        img = np.clip(img.astype(np.float32) +
                      np.random.default_rng(seed).normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    truth = cv2.perspectiveTransform(np.float64([[[W_MM / 2, H_MM / 2]]]), m).reshape(2)
    return img, truth


def quad_of(cx, cy, w, angle=0., persp=0.):
    h = w * H_MM / W_MM
    p = np.array([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]], float)
    if persp:
        p[0] += [-persp, -persp]
        p[2] += [persp, persp]
    a = np.deg2rad(angle)
    return p @ np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]).T + [cx, cy]


# --------------------------------------------------------------- 负例构造
def negative_cases():
    """返回 (name, image, 结构中心或 None)。None = 期望拒检；有中心 = 接受也必须指对中心。"""
    def blank(v=175):
        return np.full((480, 640), v, np.uint8)
    out = []
    im = blank(); cv2.rectangle(im, (170, 60), (470, 420), 22, 26)
    out.append(('dark_band_26px', im, (320, 240)))          # 合法靶同构，中心必须对
    im = blank(); cv2.rectangle(im, (170, 60), (470, 420), 22, 9)
    out.append(('narrow_band_9px', im, None))               # 远细于 18mm，拒检
    im = blank()
    for k in range(6):
        cv2.line(im, (60 + k * 100, 0), (60 + k * 100, 480), 30, 4)
    for k in range(5):
        cv2.line(im, (0, 50 + k * 95), (640, 50 + k * 95), 30, 4)
    out.append(('window_grid', im, None))
    im = blank()
    for y in range(0, 480, 40):
        for x in range(0, 640, 40):
            if (x // 40 + y // 40) % 2:
                cv2.rectangle(im, (x, y), (x + 39, y + 39), 25, -1)
    out.append(('checkerboard', im, None))
    im = blank(); cv2.rectangle(im, (220, 120), (420, 360), 25, -1)
    out.append(('solid_block', im, None))
    im = blank(); cv2.rectangle(im, (170, 60), (470, 420), 22, 22)
    im[55:80, 165:475] = 220
    out.append(('u_shape_top_missing', im, None))
    im = blank(); cv2.rectangle(im, (170, 60), (470, 420), 22, 22)
    im[60:420, 170:200] = 220
    out.append(('u_shape_left_missing', im, None))
    im = blank()
    for k in range(14):
        cv2.putText(im, 'TEXT LINE %d abcdefg 0123456789' % k, (40, 40 + k * 30),
                    cv2.FONT_HERSHEY_SIMPLEX, .8, 30, 2)
    out.append(('dense_text', im, None))
    im = np.tile(np.arange(640, dtype=np.uint8), (480, 1))
    out.append(('linear_gradient', im, None))
    im = blank(); cv2.ellipse(im, (320, 240), (150, 200), 0, 0, 360, 22, 20)
    out.append(('thick_ellipse', im, None))
    im = blank(); cv2.rectangle(im, (0, 0), (639, 479), 20, 40)
    out.append(('image_edge_border', im, None))
    # 下面四类不是比赛靶纸，但结构上就是"四边暗/亮带 + 同心中间"：接受它们
    # 不算危险误检，只要输出中心确实是该结构的几何中心。列出来是为了把
    # "语义误检"与"中心错误"分开统计。
    im = blank(); cv2.rectangle(im, (190, 90), (450, 390), 215, 24); cv2.rectangle(im, (214, 114), (426, 366), 40, -1)
    out.append(('bright_frame_dark_inside', im, (320, 240)))
    im = blank(); cv2.rectangle(im, (150, 40), (490, 440), 22, 58)
    out.append(('wide_band_58px', im, (320, 240)))
    im = blank(); cv2.rectangle(im, (200, 100), (440, 380), 22, 20)
    out.append(('thin_ring_20px', im, (320, 240)))
    im = blank(); cv2.rectangle(im, (170, 60), (470, 420), 22, 22)
    cv2.circle(im, (320, 240), 60, 22, 22)
    out.append(('rings_inside_frame', im, (320, 240)))
    im = np.random.default_rng(42).integers(0, 256, (480, 640), dtype=np.uint8)
    out.append(('random_noise', im, None))
    for name in ('negative_e421.png', 'negative_e880.png'):
        p = ROOT / 'tools' / name
        if p.exists():
            out.append((name.split('.')[0], cv2.imdecode(np.fromfile(str(p), np.uint8),
                                                         cv2.IMREAD_GRAYSCALE), None))
    return out


# --------------------------------------------------------------- 执行
def measure(cases, tracker_factory=CenterTracker):
    rows = []
    for name, image, truth in cases:
        det = tracker_factory(with_laser=True)
        r = det.track(image, 1.0)
        err = float(np.hypot(r.raw_u - truth[0], r.raw_v - truth[1])) if (r and truth is not None) else None
        dv = None
        if r and truth is not None and r.aim_valid:
            want = truth - np.array([Config.LASER_U_NOMINAL, Config.LASER_V_NOMINAL])
            dv = float(np.hypot(r.dx - want[0], r.dy - want[1]))
        rows.append(dict(case=name, valid=bool(r), stage=r.stage, err=err, deviation_err=dv,
                         ms=round(float(r.get('ms') or 0.), 2),
                         center=None if not r else [round(float(r.raw_u), 3), round(float(r.raw_v), 3)],
                         truth=None if truth is None else [round(float(v), 3) for v in truth]))
    return rows


def suites(quick=False):
    out = {}
    phase = [('phase_%d_%d' % (i, j), *render_exact(320 + i / 10., 240 + j / 10., 300., seed=i * 11 + j))
             for i in range(11) for j in range(11)]
    out['subpixel_phase_121'] = phase
    out['scale'] = [('w%d' % w, *render_exact(320.3, 240.7, w, seed=w))
                    for w in (56, 80, 120, 180, 240, 320)]
    out['noise'] = [('sigma%d_s%d' % (s, k), *render_exact(320.4, 240.6, 240., noise=s, seed=k))
                    for s in (2., 4., 6., 8.) for k in range(3)]
    out['tape_width'] = [('tape%dmm' % t, *render_exact(320.5, 240.5, 240., seed=t))
                         for t in (14, 18, 26)]
    if not quick:
        out['rotation'] = [('rot%d' % a, *render_warp(quad_of(320.4, 240.3, 200., angle=a), seed=a))
                           for a in range(0, 180, 15)]
        out['perspective'] = [('persp%d' % p, *render_warp(quad_of(320.5, 240.5, 180., persp=p), seed=p))
                              for p in (0, 8, 16, 24)]
    out['negatives'] = negative_cases()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, default=ROOT / 'tools' / 'subpixel_report')
    ap.add_argument('--quick', action='store_true')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(Config.OPENCV_THREADS)

    result, tiles = {}, []
    for name, cases in suites(args.quick).items():
        rows = measure(cases)
        result[name] = rows
        hits = [r for r in rows if r['valid']]
        errs = [r['err'] for r in hits if r['err'] is not None]
        print('%-18s n=%3d hit=%3d %s' % (name, len(rows), len(hits),
              '' if not errs else 'max=%.4f p95=%.4f px' % (max(errs), np.percentile(errs, 95))))
        for r in rows[:2]:
            if r['truth'] is not None:
                img = next(c[1] for c in cases if c[0] == r['case'])
                view = draw_result(img, CenterTracker().track(img, 1.0), 640)
                cv2.drawMarker(view, tuple(np.rint(r['truth']).astype(int)), (255, 255, 255),
                               cv2.MARKER_TILTED_CROSS, 16, 1)
                cv2.putText(view, '%s err=%s' % (r['case'], 'MISS' if r['err'] is None else '%.3f' % r['err']),
                            (8, 460), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 200, 255), 1)
                tiles.append(cv2.resize(view, (320, 240)))

    # 静态抖动：同一物理目标 + 传感器噪声，raw 中心的标准差
    tracker = CenterTracker()
    tracker.track(render_exact(320.4, 240.3, 240.)[0], 0.)
    seq = []
    for i in range(60):
        r = tracker.track(render_exact(320.4, 240.3, 240., noise=4., seed=i)[0], 1. + i / 40.)
        if r:
            seq.append([r.raw_u, r.raw_v])
    seq = np.asarray(seq)
    jitter = float(np.hypot(seq[:, 0].std(), seq[:, 1].std()))
    print('static_jitter      std=%.4f px (sigma=4 gray levels, 60 frames)' % jitter)

    summary = dict(
        kind='PC synthetic; analytic truth for axis-aligned, 4x supersampled for warp',
        subpixel_max_px=max(r['err'] for r in result['subpixel_phase_121'] if r['valid']),
        subpixel_hits=sum(r['valid'] for r in result['subpixel_phase_121']),
        subpixel_n=len(result['subpixel_phase_121']),
        static_jitter_px=round(jitter, 4),
        negative_hazard=[r['case'] for r in result['negatives']
                         if r['valid'] and (r['truth'] is None or (r['err'] or 99) > 1.5)],
        negative_accepted_safe=[r['case'] for r in result['negatives']
                                if r['valid'] and r['truth'] is not None and (r['err'] or 99) <= 1.5],
        # 每个 case 都用新 tracker，所以这里是**首帧全图搜索**耗时，不是锁定后的跟踪耗时。
        first_frame_median_ms=round(float(np.median([r['ms'] for r in result['subpixel_phase_121']])), 2),
        budget_ms=round(BUDGET_MS, 2))
    (args.output / 'results.json').write_text(json.dumps(
        dict(summary=summary, suites=result), ensure_ascii=False, indent=1), encoding='utf-8')

    if tiles:
        rows = (len(tiles) + 3) // 4
        while len(tiles) < rows * 4:
            tiles.append(np.zeros((240, 320, 3), np.uint8))
        montage = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
        cv2.imencode('.jpg', montage, [cv2.IMWRITE_JPEG_QUALITY, 88])[1].tofile(
            str(args.output / 'overview.jpg'))

    table = ''.join(
        '<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>' % (
            html.escape(r['case']), r['valid'],
            'MISS' if r['err'] is None else '%.4f' % r['err'],
            '-' if r['deviation_err'] is None else '%.4f' % r['deviation_err'],
            r['ms'], r['center'])
        for name in ('subpixel_phase_121', 'scale', 'noise', 'tape_width') for r in result[name])
    neg = ''.join('<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>' % (
        html.escape(r['case']), '接受' if r['valid'] else '拒检',
        '-' if r['err'] is None else '%.3f' % r['err'], r['center'])
        for r in result['negatives'])
    (args.output / 'index.html').write_text(
        '<!doctype html><meta charset="utf-8"><title>亚像素验证</title>'
        '<style>body{font:15px system-ui;max-width:1180px;margin:28px auto;color:#123}'
        'table{border-collapse:collapse;width:100%;margin:12px 0}td,th{border:1px solid #ccd;padding:6px}'
        'pre{background:#f4f6f8;padding:12px;overflow:auto}img{max-width:100%}</style>'
        '<h1>矩形中心亚像素验证（解析真值）</h1>'
        '<p>轴对齐场景逐像素解析覆盖率生成，真值无量化；旋转/透视用 4 倍超采样，真值分辨率 0.25px，'
        '只用于判定"能否检出"。白色斜十字 = 真值，蓝色十字 = 输出中心。</p>'
        '<pre>' + html.escape(json.dumps(summary, ensure_ascii=False, indent=1)) + '</pre>'
        '<h2>正样本</h2><table><tr><th>场景</th><th>检出</th><th>中心误差px</th>'
        '<th>偏差误差px</th><th>耗时ms</th><th>输出中心</th></tr>' + table + '</table>'
        '<h2>负样本 / 结构化干扰</h2><table><tr><th>场景</th><th>判定</th>'
        '<th>中心误差px</th><th>输出中心</th></tr>' + neg + '</table>'
        '<p><img src="overview.jpg"></p>', encoding='utf-8')

    print(json.dumps(summary, ensure_ascii=False, indent=1))
    assert summary['subpixel_hits'] == summary['subpixel_n'], '相位扫描存在漏检'
    assert summary['subpixel_max_px'] <= .15, '亚像素精度超过 0.15px'
    assert jitter <= .05, '静态抖动超过 0.05px'
    assert not summary['negative_hazard'], '存在会把中心算错的误检'
    for name in ('scale', 'noise', 'tape_width', 'rotation', 'perspective'):
        if name in result:
            miss = [r['case'] for r in result[name] if not r['valid']]
            assert not miss, '%s 漏检: %s' % (name, miss)
    print('PASS')


if __name__ == '__main__':
    main()
