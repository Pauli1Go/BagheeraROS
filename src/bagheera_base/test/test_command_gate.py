"""Tests for the command publication helpers."""

import unittest

import math

from bagheera_base.command_gate import (
    ProgressWatchdog,
    StopCommandGate,
    turn_description,
    undock_turn_speed,
)


class StopCommandGateTest(unittest.TestCase):
    def test_only_first_consecutive_stop_is_published(self):
        gate = StopCommandGate()

        self.assertTrue(gate.should_publish(0.0, 0.0))
        self.assertFalse(gate.should_publish(0.0, 0.0))
        self.assertFalse(gate.should_publish(0.0, 0.0))

    def test_motion_rearms_the_next_stop(self):
        gate = StopCommandGate()

        self.assertTrue(gate.should_publish(0.0, 0.0))
        self.assertTrue(gate.should_publish(0.08, 0.0))
        self.assertTrue(gate.should_publish(0.08, 0.0))
        self.assertTrue(gate.should_publish(0.0, 0.0))
        self.assertFalse(gate.should_publish(0.0, 0.0))


class ProgressWatchdogTest(unittest.TestCase):
    def test_slow_steady_progress_is_not_a_stall(self):
        # 0.045 m/s, the undock that hit the old fixed 18 s timeout.
        watchdog = ProgressWatchdog(5.0, 0.05)
        watchdog.reset(0.0)
        for step in range(1, 301):
            now = step * 0.1
            self.assertFalse(watchdog.stalled(now, 0.045 * now))

    def test_stall_is_detected_after_one_window(self):
        watchdog = ProgressWatchdog(5.0, 0.05)
        watchdog.reset(0.0)
        self.assertFalse(watchdog.stalled(5.0, 0.30))
        self.assertFalse(watchdog.stalled(9.9, 0.32))
        self.assertTrue(watchdog.stalled(10.0, 0.32))


class UndockTurnTest(unittest.TestCase):
    def turn(self, target, progress):
        return undock_turn_speed(target, progress, math.radians(2.0), 0.8, 0.3, 0.3)

    def test_left_turn_is_positive_until_done(self):
        self.assertEqual(self.turn(math.pi / 2, 0.0), 0.3)
        self.assertIsNone(self.turn(math.pi / 2, math.radians(89.0)))

    def test_right_turn_is_negative_until_done(self):
        self.assertEqual(self.turn(-math.pi / 2, 0.0), -0.3)
        self.assertEqual(self.turn(-math.pi / 2, math.radians(-45.0)), -0.3)
        self.assertIsNone(self.turn(-math.pi / 2, math.radians(-89.0)))

    def test_zero_means_no_turn(self):
        self.assertIsNone(self.turn(0.0, 0.0))

    def test_description(self):
        self.assertEqual(turn_description(math.pi / 2), "90.0 deg left")
        self.assertEqual(turn_description(-math.radians(45.0)), "45.0 deg right")
        self.assertEqual(turn_description(0.0), "no turn")


if __name__ == "__main__":
    unittest.main()
