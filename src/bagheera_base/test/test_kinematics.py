import unittest

from bagheera_base.controller_mapping import apply_deadzone, axes_to_twist
from bagheera_base.kinematics import axle_to_base_link_twist, shift_twist_covariance_x


class KinematicsTest(unittest.TestCase):
    def test_axle_twist_is_shifted_to_sensor_base(self):
        vx, vy = axle_to_base_link_twist(0.0, 1.0, 0.08)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, 0.08)

    def test_axle_twist_covariance_is_shifted_to_sensor_base(self):
        covariance = [0.0] * 36
        covariance[7] = 0.01
        covariance[35] = 0.04
        shifted = shift_twist_covariance_x(covariance, 0.08)
        self.assertAlmostEqual(shifted[7], 0.01 + 0.08**2 * 0.04)
        self.assertAlmostEqual(shifted[11], 0.08 * 0.04)
        self.assertAlmostEqual(shifted[31], 0.08 * 0.04)

    def test_controller_mapping_separates_driving_and_turning(self):
        self.assertEqual(apply_deadzone(0.04, 0.05), 0.0)
        linear, angular = axes_to_twist(-1.0, 1.0, 0.05, 0.25, 1.0)
        self.assertEqual(linear, 0.25)
        self.assertEqual(angular, 0.0)
        linear, angular = axes_to_twist(0.0, 1.0, 0.05, 0.25, 1.0)
        self.assertEqual(linear, 0.0)
        self.assertEqual(angular, -1.0)


if __name__ == "__main__":
    unittest.main()
