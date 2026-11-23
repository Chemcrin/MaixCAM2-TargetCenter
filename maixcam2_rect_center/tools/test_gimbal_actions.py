"""Offline checks for touch-triggered ZDT motion sequencing."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from config import Config
from vision.logic import GimbalAim, VisionControl
from vision.output import TouchButtons


class Clock:
    def __init__(self):
        self.now = 1.0

    def perf_counter(self):
        return self.now


class Link:
    def __init__(self):
        self.calls = []

    def poll(self, now):
        return {}

    def stop(self, axis=None, now=None):
        self.calls.append(('stop', axis))
        return True

    def move(self, axis, ccw, pulses, rpm, acc, now=None):
        self.calls.append(('move', axis, ccw, pulses))
        return True


class PlantLink(Link):
    """把脉冲换成画面位移的假被控对象：CCW 让坐标变小（与真机同口径）。"""

    def __init__(self, px_per_pulse=1.76):
        Link.__init__(self)
        self.u, self.v = 320.0, 240.0
        self.k = float(px_per_pulse)

    def move(self, axis, ccw, pulses, rpm, acc, now=None):
        Link.move(self, axis, ccw, pulses, rpm, acc, now)
        delta = pulses*self.k*(1.0 if ccw else -1.0)
        if axis == 'yaw':
            self.u -= delta
        else:
            self.v -= delta
        return True


class ActionsTest(unittest.TestCase):
    def setUp(self):
        # 动作时序测试不跑自标定，否则探针会抢在按键动作之前占住帧
        self.cfg = type('TestConfig', (Config,), dict(AUTO_CALIBRATE=False))
        self.clock = Clock()
        self.link = Link()
        self.aim = GimbalAim(self.cfg)
        self.control = VisionControl(self.cfg, self.link, self.aim, self.clock)

    def tick(self, result=None):
        self.control._control(result, self.clock.now, result is not None)

    def test_search_sweep_and_hold(self):
        self.control.request_action('search')
        self.tick()
        for _ in range(3):
            self.clock.now = self.control._action_ready_at + .01
            self.tick()
        moves = [c for c in self.link.calls if c[0] == 'move']
        p = round(Config.MOTOR_PULSES_PER_REV * Config.SEARCH_ANGLE_DEG / 360)
        self.assertEqual(moves, [('move', 'yaw', True, p),
                                 ('move', 'yaw', False, 2*p),
                                 ('move', 'yaw', True, p)])
        self.clock.now = self.control._action_ready_at + .01
        self.tick()
        self.assertTrue(self.control._manual_hold)

    def test_acquisition_interrupts_scan(self):
        self.control.request_action('search')
        self.tick()
        self.clock.now += .1
        before = len(self.link.calls)
        self.tick({'valid': True, 'hold': False, 'aim_valid': False})
        self.assertIsNone(self.control._action)
        self.assertFalse(self.control._manual_hold)
        self.assertEqual(self.link.calls[before], ('stop', None))

    def test_self_test_and_stop(self):
        self.control.request_action('self_test')
        self.tick()
        for _ in range(5):
            self.clock.now = self.control._action_ready_at + .01
            self.tick()
        moves = [c[1:3] for c in self.link.calls if c[0] == 'move']
        self.assertEqual(moves, [('yaw', True), ('yaw', False), ('yaw', True),
                                 ('pitch', True), ('pitch', False), ('pitch', True)])
        self.control.request_action('stop')
        self.tick()
        count = len([c for c in self.link.calls if c[0] == 'move'])
        self.clock.now += 10
        self.tick()
        self.assertEqual(count, len([c for c in self.link.calls if c[0] == 'move']))

    def test_touch_press_edge(self):
        class Touch:
            def __init__(self):
                self.value = (15, 450, True)

            def read(self):
                return self.value

        touch = Touch()
        buttons = TouchButtons(640, 480, touch)
        self.assertEqual(buttons.poll(1), 'self_test')
        self.assertIsNone(buttons.poll(1.1))
        touch.value = (0, 0, False)
        buttons.poll(1.2)
        touch.value = (125, 450, True)
        self.assertEqual(buttons.poll(1.3), 'search')

    def test_move_failure_faults_and_stops(self):
        self.link.move = lambda *args, **kwargs: False
        self.control.request_action('search')
        self.tick()
        self.assertEqual(self.aim.state, 'FAULT')
        self.assertIsNone(self.control._action)
        self.assertIn(('stop', None), self.link.calls)


class AutoCalibrationTest(unittest.TestCase):
    def test_four_step_probe_recovers_scale_and_sign(self):
        """四步探针应能从假被控对象反推出 1/1.76 脉冲/px、方向 -1。"""
        cfg = type('CalibConfig', (Config,), dict(
            AUTO_CALIBRATE=True, AUTO_TRACK_ON_BOOT=True,
            CAL_PROBE_PULSES=60, CAL_PROBE_DEG_S=30.0, CAL_SETTLE_S=0.10,
            CAL_TIMEOUT_S=1.0, CAL_START_TIMEOUT_S=5.0,
            CAL_VALID_FRAMES=4, CAL_STABLE_PX=3.0))
        clock = Clock()
        link = PlantLink(1.76)
        aim = GimbalAim(cfg)
        control = VisionControl(cfg, link, aim, clock)

        def result():
            return dict(valid=True, hold=False, aim_valid=True,
                        raw_u=link.u, raw_v=link.v, u=link.u, v=link.v,
                        captured_at=clock.now)

        for _ in range(400):
            clock.now += 0.15
            control._control(result(), clock.now, True)
            if control._calib_done:
                break

        self.assertTrue(control._calib_done)
        self.assertAlmostEqual(aim.tuning['yaw']['ppp'], 1/1.76, places=2)
        # 假对象里 CCW 让 u 变小；误差为正（靶心偏右）时该往 CCW 走 → sign=+1
        self.assertEqual(aim.tuning['yaw']['sign'], 1)
        self.assertAlmostEqual(aim.tuning['pitch']['ppp'], 1/1.76, places=2)


if __name__ == '__main__':
    unittest.main()
