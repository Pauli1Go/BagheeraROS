"""Unit tests for PMW3901-to-robot axis mapping."""

import unittest

from bagheera_base.pmw3901 import Pmw3901, transform_counts


class _FakeSpi:
    def __init__(self, burst):
        self.burst = burst

    def xfer2(self, _request):
        return self.burst


class OpticalFlowTransformTest(unittest.TestCase):
    def test_identity(self):
        self.assertEqual(
            transform_counts(12, -4, swap_xy=False, invert_x=False, invert_y=False),
            (12, -4),
        )

    def test_swap_and_invert(self):
        self.assertEqual(
            transform_counts(12, -4, swap_xy=True, invert_x=True, invert_y=False),
            (4, 12),
        )

    def test_no_motion_burst_is_a_zero_measurement(self):
        sensor = Pmw3901.__new__(Pmw3901)
        sensor.spi = _FakeSpi(
            [0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 42, 0, 0, 0, 0, 0]
        )
        self.assertEqual(sensor.read_motion(), (0, 0, 42))

    def test_motion_burst_preserves_signed_counts(self):
        sensor = Pmw3901.__new__(Pmw3901)
        sensor.spi = _FakeSpi(
            [0x00, 0x80, 0x00, 0xFE, 0xFF, 0x03, 0x00, 99, 0, 0, 0, 0, 0]
        )
        self.assertEqual(sensor.read_motion(), (-2, 3, 99))


if __name__ == "__main__":
    unittest.main()
