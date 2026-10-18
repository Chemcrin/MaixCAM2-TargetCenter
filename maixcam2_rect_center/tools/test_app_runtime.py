import os
import threading
import unittest
from unittest.mock import patch
from test_support import Clock, load_config
from vision import app_runtime
from vision.preview import TouchButtons, _local_display_call
from vision.vision_worker import LatestFrameWorker


class RuntimeTests(unittest.TestCase):
    def test_initialization_failure_closes_camera(self):
        import main
        import types,sys
        closed=[]
        class Camera:
            def __init__(self,*a,**kw): pass
            def skip_frames(self,n): raise RuntimeError('camera init failed')
            def close(self): closed.append('camera')
        maix=types.ModuleType('maix')
        maix.app=object()
        maix.camera=types.SimpleNamespace(Camera=Camera)
        maix.image=types.SimpleNamespace(Format=types.SimpleNamespace(FMT_GRAYSCALE=0))
        with patch.dict(sys.modules,maix=maix), self.assertRaisesRegex(RuntimeError,'camera init failed'):
            main.main()
        self.assertEqual(closed,['camera'])

    def test_previous_owner_blocks_restart(self):
        release=threading.Event()
        thread=threading.Thread(target=lambda:release.wait(3))
        thread.start()
        class Owner: pass
        owner=Owner();owner.thread=thread
        app_runtime._active.append(owner)
        try:
            with self.assertRaisesRegex(RuntimeError,'previous'): app_runtime._assert_idle()
        finally:
            release.set();thread.join();app_runtime._assert_idle()

    def test_worker_failure_visible(self):
        import numpy as np
        def fail(gray,captured): raise ValueError('bad detector')
        worker=LatestFrameWorker(fail)
        try:
            worker.submit(np.ones((3,3),np.uint8),0)
            worker.thread.join(1)
            self.assertIn('bad detector',worker.error)
        finally: worker.close()

    def test_overlay_uses_the_detected_frame(self):
        import numpy as np
        from vision.preview import aligned_frame
        source = np.full((4,5),17,np.uint8)
        live = np.full_like(source,99)
        worker = LatestFrameWorker(lambda g,t:dict(result=dict(valid=True),laser=None,captured_at=t),asynchronous=False)
        worker.submit(source,1.)
        source[:] = 88  # Camera buffer reused after submission.
        snapshot,_,_ = worker.snapshot(1.)
        paired = aligned_frame(snapshot,live,snapshot['result'])
        self.assertTrue((paired==17).all())
        self.assertIs(aligned_frame(snapshot,live,dict(valid=False)),live)
        self.assertEqual(worker.valid_rate.total,1)
        worker.close()

    def test_touch_edges_error_recovery_and_flash(self):
        clock=Clock()
        touch=TouchButtons.__new__(TouchButtons)
        touch._clock=clock;touch._down=False;touch._pressed_at=-1e9
        class Touch:
            def read(self): return (5,5,True)
        touch.ts=Touch()
        class Preview:
            enabled=True
            def hit(self,x,y): return 'search'
            def flash_button(self,name,now): self.flashed=name
        preview=Preview()
        self.assertEqual(touch.poll(preview),'search')
        self.assertIsNone(touch.poll(preview))
        clock.now=2.1
        self.assertEqual(touch.poll(preview),'search')
        touch.ts.read=lambda: (_ for _ in ()).throw(OSError('touch'))
        self.assertIsNone(touch.poll(preview))
        self.assertFalse(touch._down)
        self.assertEqual(preview.flashed,'search')

    def test_display_context_restored_on_exception(self):
        with patch.dict(os.environ,MAIXVISION='1'):
            with self.assertRaises(ZeroDivisionError): _local_display_call(lambda:1/0)
            self.assertEqual(os.environ['MAIXVISION'],'1')


if __name__ == '__main__': unittest.main()
