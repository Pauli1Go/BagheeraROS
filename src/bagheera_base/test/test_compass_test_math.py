"""Unit tests for the compass characterization calculations."""

import math
import unittest

from bagheera_base.compass_test_math import (
    circular_mean,
    circular_std,
    integrate_trapezoid,
    scan_yaw_from_signatures,
    TrapezoidIntegrator,
    unwrap_near,
)


class CompassTestMathTest(unittest.TestCase):
    def test_circular_statistics_cross_wrap(self):
        values = [math.radians(179.0), math.radians(-179.0)]
        self.assertAlmostEqual(abs(circular_mean(values)), math.pi)
        self.assertLess(circular_std(values), math.radians(2.0))

    def test_unwrap_near_follows_second_revolution(self):
        value = unwrap_near(math.radians(5.0), math.radians(355.0))
        self.assertAlmostEqual(value, math.radians(365.0))

    def test_trapezoid_integration(self):
        samples = [(0.0, 0.0), (1.0, 2.0), (2.0, 0.0)]
        self.assertAlmostEqual(integrate_trapezoid(samples, 0.0, 2.0), 2.0)

    def test_incremental_rate_integration_and_gap_rejection(self):
        integrator = TrapezoidIntegrator(maximum_gap=0.5)
        self.assertEqual(integrator.update(0.0, 1.0), 0.0)
        self.assertAlmostEqual(integrator.update(0.2, 1.0), 0.2)
        self.assertAlmostEqual(integrator.update(1.0, 9.0), 0.2)
        self.assertAlmostEqual(integrator.update(1.2, 1.0), 1.2)

    def test_scan_shift_is_recovered_near_expected_angle(self):
        reference = [None] * 360
        for index, value in ((10, 1.0), (60, 2.0), (150, 3.0), (270, 4.0)):
            for width in range(-12, 13):
                reference[(index + width) % 360] = value + abs(width) * 0.01
        shift = 45
        current = [reference[(index + shift) % 360] for index in range(360)]
        result = scan_yaw_from_signatures(
            reference,
            current,
            math.radians(shift),
            math.radians(1.0),
            minimum_pairs=50,
        )
        self.assertIsNotNone(result)
        yaw, score, pairs = result
        self.assertAlmostEqual(yaw, math.radians(shift))
        self.assertAlmostEqual(score, 0.0)
        self.assertGreaterEqual(pairs, 50)


if __name__ == "__main__":
    unittest.main()
