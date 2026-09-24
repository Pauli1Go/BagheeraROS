import math
import unittest

from bagheera_base.dock_calibrate import offsets_from_sample


def plugin_dock_pose(tag, axis_yaw, translation_x, translation_y, axis_yaw_offset):
    """Same composition as TagChargingDock::composeDockPose."""
    yaw = axis_yaw + axis_yaw_offset
    return (
        tag[0] + math.cos(yaw) * translation_x - math.sin(yaw) * translation_y,
        tag[1] + math.sin(yaw) * translation_x + math.cos(yaw) * translation_y,
        yaw,
    )


class DockCalibrateTest(unittest.TestCase):
    def test_offsets_reproduce_the_docked_pose(self):
        dock = (1.25, -0.40, math.radians(93.0))
        tag = (1.23, 0.09)
        axis_raw = math.radians(88.5)
        tx, ty, offset = offsets_from_sample(dock, tag, axis_raw)
        x, y, yaw = plugin_dock_pose(tag, axis_raw, tx, ty, offset)
        self.assertAlmostEqual(x, dock[0], places=9)
        self.assertAlmostEqual(y, dock[1], places=9)
        self.assertAlmostEqual(math.remainder(yaw - dock[2], math.tau), 0.0, places=9)

    def test_tag_straight_ahead_gives_negative_translation_x(self):
        # Dock facing +x, ID 1 0.489 m ahead of the docked base_link.
        tx, ty, offset = offsets_from_sample((0.0, 0.0, 0.0), (0.489, 0.0), 0.0)
        self.assertAlmostEqual(tx, -0.489)
        self.assertAlmostEqual(ty, 0.0)
        self.assertAlmostEqual(offset, 0.0)


if __name__ == "__main__":
    unittest.main()
