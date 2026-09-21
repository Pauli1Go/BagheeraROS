"""Tests for suppressing redundant idle stop commands."""

import unittest

from bagheera_base.command_gate import StopCommandGate


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


if __name__ == "__main__":
    unittest.main()
