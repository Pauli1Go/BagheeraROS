"""Tests for the dock setup geometry."""

import math
import unittest

from bagheera_base.dock_setup_math import (
    check_undock,
    docked_map_pose_from_now,
    mean_pose,
    offsets_from_sample,
    staging_map_pose,
)
from bagheera_base.pose_math import compose


class DockSetupMathTest(unittest.TestCase):
    def test_mean_pose_wraps_the_yaw(self):
        pose = mean_pose([(0.0, 0.0, math.radians(179.0)), (2.0, 2.0, math.radians(-179.0))])
        self.assertAlmostEqual(pose[0], 1.0)
        self.assertAlmostEqual(abs(pose[2]), math.pi, places=6)

    def test_docked_pose_is_carried_back_by_odometry(self):
        # Robot docked facing +y in the map; the odom frame is rotated.
        docked_map = (1.0, 2.0, math.radians(90.0))
        odom_docked = (5.0, 5.0, math.radians(10.0))
        # Reversed 0.6 m straight: in odom, 0.6 m against its heading.
        odom_now = (
            5.0 - 0.6 * math.cos(math.radians(10.0)),
            5.0 - 0.6 * math.sin(math.radians(10.0)),
            math.radians(10.0),
        )
        map_now = compose(docked_map, (-0.6, 0.0, 0.0))
        carried = docked_map_pose_from_now(map_now, odom_now, odom_docked)
        for value, expected in zip(carried, docked_map):
            self.assertAlmostEqual(value, expected, places=9)

    def test_tag_straight_ahead_gives_negative_translation_x(self):
        tx, ty, offset = offsets_from_sample((0.0, 0.0, 0.0), (0.489, 0.0), 0.0)
        self.assertAlmostEqual(tx, -0.489)
        self.assertAlmostEqual(ty, 0.0)
        self.assertAlmostEqual(offset, 0.0)

    def test_staging_pose_moves_with_the_dock(self):
        offsets = (-0.6325, 0.0076, -0.0170)
        here = staging_map_pose((0.0, 0.0, 0.0), offsets)
        there = staging_map_pose((3.0, 1.0, math.radians(90.0)), offsets)
        self.assertAlmostEqual(here[0], -0.6325)
        self.assertAlmostEqual(there[0], 3.0 - 0.0076)
        self.assertAlmostEqual(there[1], 1.0 - 0.6325)

    def test_undock_values_are_checked(self):
        check_undock(0.8, -90.0)
        with self.assertRaises(ValueError):
            check_undock(0.0, 90.0)
        with self.assertRaises(ValueError):
            check_undock(0.8, 200.0)


if __name__ == "__main__":
    unittest.main()
