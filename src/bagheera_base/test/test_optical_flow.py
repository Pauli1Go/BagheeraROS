"""Unit tests for PMW3901-to-robot axis mapping."""

import unittest

from bagheera_base.pmw3901 import transform_counts


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


if __name__ == "__main__":
    unittest.main()
