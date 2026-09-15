"""Tests for compass calibration, mounting transform and heading math."""

import math
import unittest

from bagheera_base.compass_math import (
    CompassCalibration,
    apply_calibration,
    fit_planar_calibration,
    rotate_z,
    tilt_compensated_heading,
)


class CompassMathTest(unittest.TestCase):
    def test_mounting_rotation_maps_sensor_y_to_base_x(self):
        transformed = rotate_z((0.0, 1.0, 0.0), -math.pi / 2.0)
        self.assertAlmostEqual(transformed[0], 1.0)
        self.assertAlmostEqual(transformed[1], 0.0)

    def test_level_heading_uses_ros_positive_yaw(self):
        heading, strength = tilt_compensated_heading(
            (0.0, -50.0e-6, 0.0), (0.0, 0.0, 9.80665)
        )
        self.assertAlmostEqual(heading, math.pi / 2.0)
        self.assertAlmostEqual(strength, 50.0e-6)

    def test_bias_and_matrix_are_applied(self):
        calibration = CompassCalibration(
            (1.0, 2.0, 3.0),
            ((2.0, 0.0, 0.0), (0.0, 3.0, 0.0), (0.0, 0.0, 4.0)),
            1.0,
            0.0,
            1,
        )
        self.assertEqual(apply_calibration((2.0, 3.0, 4.0), calibration), (2.0, 3.0, 4.0))

    def test_planar_fit_corrects_ellipse(self):
        samples = []
        for degree in range(360):
            angle = math.radians(degree)
            samples.append((
                80.0e-6 + 30.0e-6 * math.cos(angle),
                -40.0e-6 + 12.0e-6 * math.sin(angle),
                20.0e-6,
            ))
        result = fit_planar_calibration(samples)
        self.assertGreater(result["coverage"], 0.95)
        self.assertAlmostEqual(result["bias_t"][0], 80.0e-6, places=7)
        self.assertAlmostEqual(result["bias_t"][1], -40.0e-6, places=7)
        calibration = CompassCalibration(
            tuple(result["bias_t"]),
            tuple(tuple(row) for row in result["matrix"]),
            result["horizontal_field_t"],
            0.0,
            len(samples),
        )
        radii = [math.hypot(*apply_calibration(sample, calibration)[:2]) for sample in samples]
        self.assertLess(max(radii) - min(radii), 2.0e-6)


if __name__ == "__main__":
    unittest.main()
