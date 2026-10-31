import unittest
from unittest.mock import patch
from test_support import load_config
import cv2
import numpy as np
from test_performance import scene
from vision.rectangle_center import RectangleCenterDetector
from vision.rect_geometry import homography_8x8, project_points, rectangle_corners


class TrackingTests(unittest.TestCase):
    def test_full_search_even_after_roi_exhaustion(self):
        cfg = load_config(SEARCH_EXHAUSTIVE=False, SEARCH_MAX_PROPOSALS=128)
        d = RectangleCenterDetector(cfg)
        image,_ = scene([[180,70],[445,80],[475,400],[150,420]])
        self.assertTrue(d.process(image,0)['valid'])
        original = d._search
        calls=[]
        def search(g, roi=None):
            calls.append(roi)
            if roi is not None:
                d._proposals = cfg.SEARCH_MAX_PROPOSALS
                return None
            return original(g,roi)
        refine = d._refine
        # Force the tracking strip to fail, but allow genuine full-search refinement.
        # Unrefined proposals are now intentionally invalid.
        with patch.object(d,'_refine',side_effect=lambda g,q,k,r,**kw:None if r in (cfg.TRACK_RADIUS,cfg.REFINE_RADIUS_WIDE) else refine(g,q,k,r,**kw)), patch.object(d,'_search',side_effect=search):
            result=d.process(image,.03)
        self.assertEqual(len(calls),2)
        self.assertIsNone(calls[-1])
        self.assertTrue(result['valid'])

    def test_fast_translation_and_turn(self):
        d=RectangleCenterDetector(load_config())
        q=np.array([[220,130],[340,130],[340,300],[220,300]],float)
        stages=[]
        for i in range(24):
            dx = i*10 if i < 12 else (23-i)*10
            image,truth=scene(q+[dx,0])
            r=d.process(image,i/30)
            self.assertTrue(r['valid'])
            self.assertLess(np.linalg.norm([r['raw_u']-truth[0],r['raw_v']-truth[1]]),1.)
            stages.append(r['stage'])
        self.assertGreaterEqual(stages.count('track'),22)
        self.assertFalse(d.process(np.full((480,640),200,np.uint8),1)['valid'])
        self.assertTrue(d.process(scene(q)[0],1.03)['valid'])

    def test_geometry_direction_and_frame_resize(self):
        q=np.array([[30,20],[400,30],[450,350],[20,400]],float)
        h=homography_8x8(q,210,297)
        np.testing.assert_allclose(project_points(h,rectangle_corners(210,297)),q,atol=1e-8)
        d=RectangleCenterDetector(load_config())
        d.process(scene(q)[0],0)
        r=d.process(np.ones((40,60),np.uint8),.03)
        self.assertFalse(r['valid'])
        self.assertIsNone(d._prev_quad)
        self.assertIsNone(d._smooth)

    def test_clipped_target_cold_full_lock(self):
        """靶纸上边出画 60px、胶带带面仍可见时，冷启动也应能四边全测。"""
        q=np.array([[180,70],[445,80],[475,400],[150,420]],float)
        image,truth=scene(q+[0,-60])
        r=RectangleCenterDetector(load_config()).process(image)
        self.assertTrue(r['valid'], r['stage'])
        self.assertEqual(r['clipped_edges'],0)
        self.assertLess(np.linalg.norm([r['raw_u']-truth[0],r['raw_v']-truth[1]]),1.0)

    def test_clipped_target_keeps_lock(self):
        """连续上移直到整条上边出画：track 不应掉回搜索，末段允许 partial。"""
        q=np.array([[180,70],[445,80],[475,400],[150,420]],float)
        d=RectangleCenterDetector(load_config())
        partial=0
        for i,dy in enumerate(range(0,-101,-10)):
            image,truth=scene(q+[0,dy])
            r=d.process(image,i*.025)
            self.assertTrue(r['valid'],'frame %d dy=%d stage=%s'%(i,dy,r['stage']))
            self.assertLess(np.linalg.norm([r['raw_u']-truth[0],r['raw_v']-truth[1]]),1.0)
            partial+=bool(r['partial'])
        self.assertGreaterEqual(partial,2,'deep clip should publish partial locks')

    def test_three_edge_occlusion_with_prior(self):
        """锁定后顶边被亮块遮掉一半：有先验边时应 partial 保住，而不是整帧丢。"""
        q=np.array([[180,70],[445,80],[475,400],[150,420]],float)
        d=RectangleCenterDetector(load_config())
        image,truth=scene(q)
        self.assertTrue(d.process(image,0)['valid'])
        top=(q[0]+q[1])/2.0
        half=.5*(q[1][0]-q[0][0])/2.0
        img=image.copy()
        cv2.rectangle(img,(int(top[0]-half),int(top[1]-22)),(int(top[0]+half),int(top[1]+22)),230,-1)
        r=d.process(img,.025)
        self.assertTrue(r['valid'],r['stage'])
        self.assertTrue(r['partial'])
        self.assertLess(np.linalg.norm([r['raw_u']-truth[0],r['raw_v']-truth[1]]),1.0)

    def test_u_shape_still_rejected(self):
        """画面内整条边没有胶带证据 = U 形，冷启动仍须拒绝（不能靠 partial 放行）。"""
        q=np.array([[180,70],[445,80],[475,400],[150,420]],float)
        image,_=scene(q)
        p0,p1=q[0].astype(int),q[1].astype(int)
        cv2.line(image,(p0[0],p0[1]+4),(p1[0],p1[1]+4),220,26)
        self.assertFalse(RectangleCenterDetector(load_config()).process(image)['valid'])


if __name__ == '__main__': unittest.main()
