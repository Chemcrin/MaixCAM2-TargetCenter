"""验证备份的独立入口、真实检测/光斑/偏差组合和灰度桥接。"""
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools/maix_stub'))
import main
from config import Config
from maix import app, display, image
from test_aiming import frame, TestConfig
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector, draw_result
from vision.aiming import LaserSpotDetector, add_aiming, draw_aiming, pack_aiming


class ReplayTests(unittest.TestCase):
    def test_end_to_end_aim_vector(self):
        gray, truth = scene([[180, 70], [445, 80], [475, 400], [150, 420]])
        glow = frame([(295, 208, 130)]).astype(np.int16) - 120
        gray = np.clip(gray.astype(np.int16) + glow, 0, 255).astype(np.uint8)
        result = RectangleCenterDetector(Config).process(gray)
        self.assertTrue(result['valid'])
        spot = LaserSpotDetector(TestConfig).process(gray)
        self.assertTrue(spot['valid'])
        add_aiming(result, gray.shape, TestConfig, spot)
        self.assertTrue(result['aim_valid'])
        self.assertAlmostEqual(result['dx'], truth[0] - 295, delta=1.5)
        self.assertAlmostEqual(result['dy'], truth[1] - 208, delta=1.5)
        canvas = draw_result(gray, result, Config.SHOW_WIDTH)
        draw_aiming(canvas, result, gray.shape)
        self.assertEqual(canvas.shape, (360, 480, 3))
        self.assertTrue(pack_aiming(result).startswith(b'@A,1,'))

    def test_maix_gray_bridge(self):
        gray = np.full((40, 60), 120, np.uint8)
        self.assertEqual(main._to_gray(image.Image(gray), image).shape, gray.shape)
        self.assertEqual(main._to_gray(image.Image(gray[:, :, None]), image).shape, gray.shape)

    def test_main_loop_with_stub(self):
        app.reset()
        app.LIMIT = 5
        before = display.Display.shows
        main.main()
        self.assertEqual(app._state['n'], 6)
        self.assertGreater(display.Display.shows, before)
        self.assertTrue(str(Path(main.__file__).resolve()).startswith(str(ROOT)))


if __name__ == '__main__':
    cv2.setNumThreads(1)
    unittest.main()
