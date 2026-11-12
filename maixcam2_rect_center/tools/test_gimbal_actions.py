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


class ActionsTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.link = Link()
        self.aim = GimbalAim(Config)
        self.control = VisionControl(Config, self.link, self.aim, self.clock)

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


if __name__ == '__main__':
    unittest.main()
