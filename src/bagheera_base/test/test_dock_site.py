"""Tests for the dock site file (maps/dock.yaml) and its fallbacks."""

import math
import os
import tempfile
import unittest

from bagheera_base.dock_site import DockSite, dump_dock_site, load_dock_site

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "nav2_navigation.yaml")


class DockSiteTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.site = os.path.join(self.directory.name, "dock.yaml")

    def tearDown(self):
        self.directory.cleanup()

    def write(self, text):
        with open(self.site, "w", encoding="utf-8") as handle:
            handle.write(text)

    def test_missing_file_uses_nav2_defaults(self):
        site = load_dock_site(CONFIG, self.site)
        self.assertEqual(len(site.pose), 3)
        self.assertGreater(site.reverse_distance_m, 0.0)
        self.assertAlmostEqual(site.turn_angle_rad, math.pi / 2)
        self.assertEqual(site.source, CONFIG)

    def test_site_file_overrides_pose_and_undock(self):
        self.write(
            "dock_pose: [2.0, -3.5, 0.5]\n"
            "undock:\n  reverse_distance_m: 0.6\n  turn_angle_deg: -45\n"
        )
        site = load_dock_site(CONFIG, self.site)
        self.assertEqual(site.pose, (2.0, -3.5, 0.5))
        self.assertAlmostEqual(site.reverse_distance_m, 0.6)
        self.assertAlmostEqual(site.turn_angle_rad, math.radians(-45.0))
        self.assertEqual(site.source, self.site)

    def test_partial_file_keeps_other_defaults(self):
        self.write("undock:\n  turn_angle_deg: 0\n")
        default = load_dock_site(CONFIG, os.path.join(self.directory.name, "none.yaml"))
        site = load_dock_site(CONFIG, self.site)
        self.assertEqual(site.pose, default.pose)
        self.assertEqual(site.turn_angle_rad, 0.0)

    def test_invalid_values_are_rejected(self):
        for text in (
            "dock_pose: [1.0, 2.0]\n",
            "dock_pose: [1.0, .nan, 0.0]\n",
            "undock:\n  reverse_distance_m: 0\n",
            "undock:\n  turn_angle_deg: 200\n",
        ):
            self.write(text)
            with self.assertRaises(ValueError, msg=text):
                load_dock_site(CONFIG, self.site)

    def test_dump_round_trip(self):
        original = DockSite((1.192, 1.884, 1.624), 0.8, math.radians(-90.0), 0.012, "x")
        self.write(dump_dock_site(original))
        site = load_dock_site(CONFIG, self.site)
        self.assertEqual(site.pose, original.pose)
        self.assertAlmostEqual(site.turn_angle_rad, original.turn_angle_rad)
        self.assertAlmostEqual(site.axis_yaw_offset, original.axis_yaw_offset)


if __name__ == "__main__":
    unittest.main()
