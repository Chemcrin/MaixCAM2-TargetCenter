"""Real-scene positives (incl. one missing corner) and structural negatives.

v1.0.1 regression locks:
  * the two real board frames must be found (v1.0.0's `_refine` collinearity gate
    rejected them);
  * the first frame may full-search, but the second must be `track` (a miss used to
    force a 0.5~1.4s full search on every board frame);
  * +40 gray exposure shift must still be rescued (absolute RECOVER_MAX_DARK removed);
  * a window grid must still be rejected (relative dark-band gate).
"""
import unittest
from pathlib import Path
from test_support import ROOT, load_config
import cv2
import numpy as np
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector


def read(path):
    return cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_GRAYSCALE)


class RealSceneTests(unittest.TestCase):
    def test_real_negatives(self):
        for name in ('negative_e421.png','negative_e880.png'):
            image=read(ROOT/'tools'/name)
            self.assertIsNotNone(image)
            self.assertFalse(RectangleCenterDetector(load_config()).process(image)['valid'],name)

    def test_front_view_direction_and_independent_center(self):
        image, truth=scene([[170,90],[430,65],[485,395],[110,420]])
        r=RectangleCenterDetector(load_config(FRONT_VIEW_SHORT_PX=512)).process(image)
        self.assertTrue(r['valid'])
        self.assertTrue(r['geometry_consistent'])
        self.assertLess(np.linalg.norm(r['front_view_center']-truth),1.5)
        view=r['front_view']
        self.assertLess(np.median(view[3,:]),50)
        self.assertGreater(np.median(view[view.shape[0]//2,:]),180)

    def test_board_positive_and_then_tracks(self):
        path=ROOT/'tools'/'real_board_frame.png'
        if not path.exists(): self.skipTest('real_board_frame.png not supplied; cannot claim board accuracy')
        detector=RectangleCenterDetector(load_config())
        first=detector.process(read(path))
        self.assertTrue(first['valid'])
        second=detector.process(read(path))
        self.assertEqual(second['stage'],'track', 'a found board must lock instead of full-searching every frame')

    def test_board_positive_under_brighter_exposure(self):
        path=ROOT/'tools'/'real_board_frame.png'
        if not path.exists(): self.skipTest('real_board_frame.png not supplied; cannot claim board accuracy')
        image=read(path)
        bright=np.clip(image.astype(np.int16)+40,0,255).astype(np.uint8)
        self.assertTrue(RectangleCenterDetector(load_config()).process(bright)['valid'])

    def test_corner_gap_positive(self):
        path=ROOT/'tools'/'real_scene_corner_gap.png'
        if not path.exists(): self.skipTest('real_scene_corner_gap.png not supplied; real corner-gap precision unverified')
        self.assertTrue(RectangleCenterDetector(load_config()).process(read(path))['valid'])

    def test_window_grid_still_rejected(self):
        image=np.full((480,640),175,np.uint8)
        for k in range(6): cv2.line(image,(60+k*100,0),(60+k*100,480),30,4)
        for k in range(5): cv2.line(image,(0,50+k*95),(640,50+k*95),30,4)
        self.assertFalse(RectangleCenterDetector(load_config()).process(image)['valid'])


if __name__ == '__main__': unittest.main()
