"""Unit tests for WT901 register decoding and startup gyro calibration."""

import math
import struct
import unittest

from bagheera_base.wt901_protocol import GyroBiasEstimator, decode_motion_block


class Wt901ProtocolTest(unittest.TestCase):
    def test_motion_block_is_converted_to_si_units(self):
        block = struct.pack(
            "<9h",
            0,
            -2048,
            2048,
            0,
            -16384,
            16384,
            1,
            2,
            3,
        )
        sample = decode_motion_block(block)
        self.assertAlmostEqual(sample.acceleration[0], 0.0)
        self.assertAlmostEqual(sample.acceleration[1], -9.80665)
        self.assertAlmostEqual(sample.acceleration[2], 9.80665)
        self.assertAlmostEqual(sample.angular_velocity[0], 0.0)
        self.assertAlmostEqual(sample.angular_velocity[1], math.radians(-1000.0))
        self.assertAlmostEqual(sample.angular_velocity[2], math.radians(1000.0))

    def test_invalid_block_length_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_motion_block(bytes(17))

    def test_stationary_bias_is_removed(self):
        estimator = GyroBiasEstimator(3)
        self.assertFalse(estimator.update((0.1, -0.2, 0.3)))
        self.assertFalse(estimator.update((0.2, -0.1, 0.4)))
        self.assertTrue(estimator.update((0.0, 0.0, 0.2)))
        corrected = estimator.correct((0.2, -0.2, 0.4))
        self.assertAlmostEqual(corrected[0], 0.1)
        self.assertAlmostEqual(corrected[1], -0.1)
        self.assertAlmostEqual(corrected[2], 0.1)


if __name__ == "__main__":
    unittest.main()
