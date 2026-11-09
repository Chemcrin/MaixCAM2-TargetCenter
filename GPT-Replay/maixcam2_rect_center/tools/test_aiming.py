"""重建本对话的 7 项光斑回归。python -B tools/test_aiming.py"""
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import Config
from vision.aiming import LaserSpotDetector, add_aiming, draw_aiming, pack_aiming


class TestConfig(Config):
    LASER_REFERENCE_SIZE = (640, 480)
    LASER_PIXEL = (295, 208)


def frame(spots=()):
    yy, xx = np.mgrid[:480, :640]
    g = np.full((480, 640), 120, np.float32)
    for x, y, amplitude in spots:
        g += amplitude * np.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * 1.5 ** 2))
    return np.clip(g, 0, 255).astype(np.uint8)


class AimingTests(unittest.TestCase):
    def test_actual_spot_and_negative_offset(self):
        spot = LaserSpotDetector(TestConfig).process(frame([(298.5, 205.5, 80)]))
        self.assertTrue(spot['valid'])
        self.assertLess(abs(spot['u'] - 298.5), .5)
        self.assertLess(abs(spot['v'] - 205.5), .5)
        result = add_aiming(dict(valid=True, u=268.5, v=175.5), (480, 640), TestConfig, spot)
        self.assertAlmostEqual(result['dx'], -30, delta=.5)
        self.assertAlmostEqual(result['dy'], -30, delta=.5)
        self.assertTrue(result['aim_valid'])

    def test_loss_never_reuses_coordinates(self):
        detector = LaserSpotDetector(TestConfig)
        self.assertTrue(detector.process(frame([(295, 208, 80)]))['valid'])
        lost = detector.process(frame())
        self.assertFalse(lost['valid'])
        result = add_aiming(dict(valid=True, u=270, v=130), (480, 640), TestConfig, lost)
        self.assertFalse(result['aim_valid'])
        self.assertIsNone(result['laser_u'])
        self.assertIsNone(result['dx'])
        self.assertEqual(pack_aiming(result), b'@A,0,270,130,0,0,0,0,0,0,1%\r\n')

    def test_competing_spots_rejected(self):
        spot = LaserSpotDetector(TestConfig).process(frame([(285, 208, 90), (305, 208, 90)]))
        self.assertFalse(spot['valid'])
        self.assertEqual(spot['stage'], 'ambiguous')

    def test_reflection_and_bright_line_rejected(self):
        for shape in ('box', 'line'):
            g = frame()
            if shape == 'box':
                cv2.rectangle(g, (280, 190), (315, 227), 245, -1)
            else:
                cv2.line(g, (270, 200), (318, 200), 245, 2)
            self.assertFalse(LaserSpotDetector(TestConfig).process(g)['valid'])

    def test_target_loss_and_overlay_scaling(self):
        spot = dict(valid=True, u=300, v=200, contrast=80, stage='measured')
        no_target = add_aiming(dict(valid=False), (480, 640), TestConfig, spot)
        self.assertTrue(no_target['laser_valid'])
        self.assertFalse(no_target['aim_valid'])
        self.assertIsNone(no_target['dy'])
        canvas = np.zeros((240, 320, 3), np.uint8)
        draw_aiming(canvas, no_target, (480, 640))
        self.assertGreater(int(canvas[95:106, 145:156, 2].max()), 100)
        result = add_aiming(dict(valid=True, u=270, v=180), (480, 640), TestConfig, spot)
        self.assertEqual(pack_aiming(result), b'@A,1,270,180,300,200,-30,-20,0,1,1%\r\n')
        draw_aiming(canvas, result, (480, 640))
        self.assertGreater(int(canvas[90:102, 135:150, 1].max()), 100)

    def test_explicit_fixed_mode_and_old_protocol(self):
        class Fixed(TestConfig):
            LASER_MODE = 'fixed'
        result = add_aiming(dict(valid=True, u=100, v=150), (240, 320), Fixed)
        self.assertEqual(result['laser_u'], 147.5)
        self.assertEqual(result['laser_v'], 104)
        self.assertEqual(pack_aiming(result, 'center'), b'@[100,150]%\r\n')

    def test_weak_spot(self):
        spot = LaserSpotDetector(TestConfig).process(frame([(294, 209, 15)]))
        self.assertTrue(spot['valid'])


if __name__ == '__main__':
    cv2.setNumThreads(1)
    unittest.main()
