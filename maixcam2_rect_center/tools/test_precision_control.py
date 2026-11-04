"""真实合成图→公开AimLoop→电机命令→模拟画面运动，验证最终固定点误差。"""
import json
from unittest.mock import patch
import cv2
import numpy as np
from test_support import ROOT, load_config
from test_performance import scene
from target_api import AimLoop
from vision.stepper_link import estimate_move_seconds


class Clock:
    now = 1.
    def perf_counter(self): return self.now


def main():
    cv2.setNumThreads(1)
    cfg = load_config(ENABLE_SEARCH_BUTTON=False,ENABLE_IMU_FEEDFORWARD=False)
    clock = Clock()
    offset = np.zeros(2)
    class Link:
        def __init__(self,cfg,clock=None):
            self.pending = {}
            self.moves = 0
        def enable(self): return True
        def advance(self,now):
            for axis,(start,end,origin,delta) in list(self.pending.items()):
                index = 0 if axis=='yaw' else 1
                offset[index] = origin+delta*min(1.,max(0.,(now-start)/(end-start)))
                if now>=end: del self.pending[axis]
        def move(self,axis,ccw,pulses,rpm,acc,now):
            index = 0 if axis=='yaw' else 1
            duration = estimate_move_seconds(pulses,rpm,acc,cfg.MOTOR_PULSES_PER_REV)
            ppp = cfg.PULSE_PER_PX_YAW if axis=='yaw' else cfg.PULSE_PER_PX_PITCH
            self.pending[axis] = now,now+duration,float(offset[index]),(1 if ccw else -1)*pulses/ppp
            self.moves += 1
            return True
        def stop(self,axis=None,now=None):
            self.advance(clock.now if now is None else now)
            if axis is None: self.pending.clear()
            else: self.pending.pop(axis,None)
            return True
        def poll(self,now): return dict(fault=False)
        def close(self): pass

    base = np.array([[220.,115.],[410.,115.],[410.,385.],[220.,385.]])
    with patch('vision.stepper_link.StepperLink',Link):
        loop = AimLoop(cfg=cfg,clock=clock)
        try:
            settled = 0
            initial = None
            for frame in range(1000):
                clock.now = 1.+frame/40
                loop.link.advance(clock.now)
                image,truth = scene(base+offset)
                r = loop.step(image,now=clock.now)
                assert r.valid and r.aim_valid,'image/offset must be valid every frame'
                assert r.aim_state!='FAULT',r
                expected = truth-[cfg.LASER_U_NOMINAL,cfg.LASER_V_NOMINAL]
                assert np.linalg.norm([r.dx-expected[0],r.dy-expected[1]])<.3
                if initial is None: initial = expected.tolist()
                settled = settled+1 if r.aim_state=='SETTLE' and not loop.link.pending else 0
                if settled>=20: break
            assert settled>=20,'closed loop did not settle'
            assert np.max(np.abs(expected))<=cfg.DEADBAND_PX+.2,expected
            assert loop.link.moves>0
            report = dict(simulation=True,frames=frame+1,initial_error_px=initial,
                          final_truth_error_px=expected.tolist(),commands=loop.link.moves)
            output = ROOT/'tools'/'precision_report'/'control_simulation.json'
            output.parent.mkdir(parents=True,exist_ok=True)
            output.write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps(report,indent=2))
        finally: loop.close()


if __name__=='__main__': main()
