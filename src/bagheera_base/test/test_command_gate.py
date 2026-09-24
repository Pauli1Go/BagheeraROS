"""Tests for the command publication helpers."""

import unittest

from bagheera_base.command_gate import ProgressWatchdog, StopCommandGate


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


if __name__ == "__main__":
    unittest.main()
