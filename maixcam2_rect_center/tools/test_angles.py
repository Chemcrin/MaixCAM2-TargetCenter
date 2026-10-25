"""多角度/多尺度/透视/镜头畸变回归：每个姿态都必须检出，中心误差有上限。

生成方式与 `verify_precision` 独立：由针孔相机模型把 A4 靶纸按 yaw/pitch/roll
投影到 640×480，真值由同一投影给出；径向畸变项额外做前向映射。

运行：python -B tools/test_angles.py
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))

from config import Config
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector

W, H = 210., 297.
CX, CY, F = 320., 240., 700.


def _rot(yaw, pitch, roll):
    y, p, r = map(math.radians, (yaw, pitch, roll))
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    Rz = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
    return Rz @ Rx @ Ry


def _pinhole(yaw, pitch, roll, width_px, uc=320., vc=240.):
    R = _rot(yaw, pitch, roll)
    dist = F * W / width_px
    tx, ty = (uc - CX) * dist / F, (vc - CY) * dist / F
    corners = np.array([[-W/2, -H/2, 0], [W/2, -H/2, 0], [W/2, H/2, 0], [-W/2, H/2, 0]])
    P = corners @ R.T + np.array([tx, ty, dist])
    q = np.column_stack((F*P[:, 0]/P[:, 2] + CX, F*P[:, 1]/P[:, 2] + CY)).astype(np.float32)
    return q, np.array([uc, vc])


def _fit(yaw, pitch, roll, width_px, uc=320., vc=240., margin=6., k1=0.):
    """收缩目标直到（畸变后的）四角都在画面内，保证测的是识别而不是越界。"""
    for _ in range(30):
        q, truth = _pinhole(yaw, pitch, roll, width_px, uc, vc)
        qd = _distort_uv(q, k1) if k1 else q
        if (qd[:, 0].min() >= margin and qd[:, 1].min() >= margin and
                qd[:, 0].max() <= 640 - margin and qd[:, 1].max() <= 480 - margin):
            td = _distort_uv(truth.reshape(1, 2), k1)[0] if k1 else truth
            return q, td
        width_px *= .92
    raise RuntimeError('target cannot be placed inside the frame')


def _distort_uv(uv, k1, k2=0.):
    uv = np.asarray(uv, np.float32).reshape(-1, 2)
    x, y = (uv[:, 0] - CX) / F, (uv[:, 1] - CY) / F
    r2 = x*x + y*y
    s = 1 + k1*r2 + k2*r2*r2
    return np.column_stack((F*x*s + CX, F*y*s + CY)).reshape(-1, 2)


def _distort_image(img, k1, k2=0., border=175.):
    h, w = img.shape
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    xd, yd = (xs - CX) / F, (ys - CY) / F
    xu, yu = xd.copy(), yd.copy()
    for _ in range(6):                       # 逆畸变：输出像素 -> 针孔图坐标
        r2 = xu*xu + yu*yu
        xu, yu = xd / (1 + k1*r2), yd / (1 + k1*r2)
    return cv2.remap(img, (F*xu + CX).astype(np.float32), (F*yu + CY).astype(np.float32),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=border)


def _check(name, cases, limit):
    errors, misses, times = [], [], []
    for label, image, truth in cases:
        detector = RectangleCenterDetector(Config)
        result = detector.process(image)
        times.append(result['detector_ms'])
        if not result['valid']:
            misses.append(label)
            continue
        errors.append((float(np.hypot(result['raw_u'] - truth[0], result['raw_v'] - truth[1])), label))
    worst = max(errors, default=(999., 'miss'))
    print('%-18s n=%3d hit=%3d miss=%2d max=%.2fpx med=%.2fpx ms_med=%.1f' % (
        name, len(cases), len(errors), len(misses), worst[0],
        float(np.median([e for e, _ in errors])) if errors else 999.,
        float(np.median(times))))
    if misses:
        print('  miss:', misses[:12])
    if errors and worst[0] > 1.0:
        print('  worst:', [(round(e, 2), label) for e, label in sorted(errors, reverse=True)[:6]])
    assert not misses, '%s: %d missed' % (name, len(misses))
    assert worst[0] <= limit, '%s: max center error %.2fpx > %.2fpx (%s)' % (
        name, worst[0], limit, worst[1])
    return errors


def _poses():
    clean = []
    for yaw in (-60, -45, -30, -15, 0, 15, 30, 45, 60):
        for pitch in (-45, -30, -15, 0, 15, 30, 45):
            q, truth = _fit(yaw, pitch, 0, 190.)
            clean.append(('y%d_p%d' % (yaw, pitch), scene(q)[0], truth))
    for roll in range(0, 360, 15):
        q, truth = _fit(25, 20, roll, 180.)
        clean.append(('roll%d' % roll, scene(q)[0], truth))
    for yaw, pitch in ((0, 0), (25, 20), (-35, 25), (45, -20)):
        for uc, vc in ((320, 240), (120, 120), (540, 120), (120, 380), (540, 380), (320, 70), (320, 430)):
            q, truth = _fit(yaw, pitch, 0, 130., uc, vc)
            clean.append(('off_%d_%d_y%d_p%d' % (uc, vc, yaw, pitch), scene(q)[0], truth))
    for width in (80, 110, 140, 170, 200, 230, 260):
        for yaw, pitch in ((0, 0), (30, 25), (50, -35), (60, 40)):
            q, truth = _fit(yaw, pitch, 0, width)
            clean.append(('w%d_y%d_p%d' % (width, yaw, pitch), scene(q)[0], truth))
    for yaw, pitch, roll in ((30, 20, 45), (45, 30, 120), (55, -25, 200), (-40, 35, 300)):
        for uc, vc in ((320, 240), (180, 160), (460, 320)):
            q, truth = _fit(yaw, pitch, roll, 150., uc, vc)
            clean.append(('comb_%d_%d_r%d' % (uc, vc, roll), scene(q)[0], truth))
    return clean


def _distorted():
    out = []
    for k1 in (-0.30, -0.15, 0.15, 0.30):
        for yaw, pitch, roll in ((0, 0, 0), (20, 15, 0), (-30, 20, 45),
                                 (40, -25, 120), (55, 30, 200), (-45, 35, 300)):
            for uc, vc in ((320, 240), (160, 150), (480, 330)):
                q, truth = _fit(yaw, pitch, roll, 170., uc, vc, k1=k1)
                out.append(('k%+.2f_y%d_p%d_r%d_%d_%d' % (k1, yaw, pitch, roll, uc, vc),
                            _distort_image(scene(q)[0], k1), truth))
    return out


def _real_corner():
    path = ROOT / 'tools' / 'real_scene_corner_gap.png'
    if not path.exists():
        return []
    base = cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_GRAYSCALE)
    center = np.array([246.03, 161.29], np.float64)
    out = []
    for angle in (-45, -30, -15, 0, 15, 30, 45):
        for scale in (.8, 1.0, 1.2):
            matrix = cv2.getRotationMatrix2D((base.shape[1] / 2, base.shape[0] / 2), angle, scale)
            image = cv2.warpAffine(base, matrix, (640, 480), flags=cv2.INTER_LINEAR, borderValue=146)
            truth = matrix @ np.r_[center, 1.]
            out.append(('rot%d_s%.1f' % (angle, scale), image, truth))
    return out


def main():
    cv2.setNumThreads(Config.OPENCV_THREADS)
    print('=' * 78)
    clean = _poses()
    distorted = _distorted()
    real = _real_corner()
    _check('clean multi-angle', clean, 2.0)
    _check('radial distortion', distorted, 4.0)
    if real:
        _check('real frame warped', real, 4.0)
    else:
        print('real frame warped   SKIP (fixture missing)')
    print('=' * 78)
    print('multi-angle PASS: clean %d, lens %d, real %d' % (len(clean), len(distorted), len(real)))


if __name__ == '__main__':
    main()
