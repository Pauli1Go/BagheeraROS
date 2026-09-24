import math
import unittest

from bagheera_base.dock_trigger import FinalApproach, final_approach_command, front_offset


class FinalApproachTest(unittest.TestCase):
    def setUp(self):
        self.params = FinalApproach()

    def test_aligned_robot_drives_straight(self):
        linear, angular, reason = final_approach_command(-0.12, 0.0, 0.0, self.params)
        self.assertIsNone(reason)
        self.assertAlmostEqual(linear, 0.05)
        self.assertAlmostEqual(angular, 0.0)

    def test_heading_error_is_turned_back(self):
        # Robot turned 5 deg to the left of the axis: steer right.
        _, angular, _ = final_approach_command(-0.1, 0.0, math.radians(5.0), self.params)
        self.assertLess(angular, 0.0)

    def test_lateral_offset_uses_bounded_trim(self):
        # 2 cm right of the axis: aim slightly left, never more than 4 deg.
        _, angular, _ = final_approach_command(-0.1, -0.02, 0.0, self.params)
        self.assertGreater(angular, 0.0)
        _, angular_far, _ = final_approach_command(-0.1, -0.5, 0.0, self.params)
        self.assertAlmostEqual(angular_far, self.params.heading_gain * self.params.trim_max)

    def test_overtravel_stops(self):
        linear, angular, reason = final_approach_command(0.031, 0.0, 0.0, self.params)
        self.assertEqual((linear, angular), (0.0, 0.0))
        self.assertIsNotNone(reason)

    def test_front_offset_includes_yaw(self):
        # 3.8 deg at the pins moved the contacts ~3 cm on 2026-09-23.
        self.assertAlmostEqual(
            front_offset(-0.006, math.radians(3.8), self.params), 0.025, places=3
        )


if __name__ == "__main__":
    unittest.main()
