import math
import unittest

from bagheera_base.controller_mapping import apply_deadzone, axes_to_twist
from bagheera_base.kinematics import (
    DifferentialOdometry,
    axle_to_base_link_twist,
    shift_twist_covariance_x,
    signed_delta32,
    twist_to_wheels,
)


class KinematicsTest(unittest.TestCase):
    def test_twist_to_wheels(self):
        self.assertEqual(twist_to_wheels(0.2, 0.0, 0.285, 0.6), (200, 200))
        self.assertEqual(twist_to_wheels(0.0, 1.0, 0.2, 0.6), (-100, 100))

    def test_wheel_limit_preserves_curvature(self):
        left, right = twist_to_wheels(0.6, 2.0, 0.3, 0.6)
        self.assertEqual(right, 600)
        self.assertEqual(left, 200)

    def test_non_finite_twist_rejected(self):
        with self.assertRaises(ValueError):
            twist_to_wheels(math.nan, 0.0, 0.285, 0.6)

    def test_encoder_rollover(self):
        self.assertEqual(signed_delta32(-2147483648, 2147483647), 1)

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

    def test_odometry_forward_and_turn(self):
        odometry = DifferentialOdometry(1000, 0.5)
        odometry.update(0, 0, 0.0, 0.0)
        forward = odometry.update(1000, 1000, 1.0, 1.0)
        self.assertAlmostEqual(forward.x, 1.0)
        self.assertAlmostEqual(forward.y, 0.0)
        turn = odometry.update(1000, 1500, 0.0, 0.5)
        self.assertAlmostEqual(turn.yaw, 1.0)
        self.assertAlmostEqual(turn.angular_velocity, 1.0)

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
