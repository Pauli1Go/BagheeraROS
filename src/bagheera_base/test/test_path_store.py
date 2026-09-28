"""Tests for the saved patrol paths (maps/paths/<name>.yaml)."""

import tempfile
import unittest

from bagheera_base.path_store import (
    PatrolPath,
    delete_path,
    directory_stamp,
    list_paths,
    load_path,
    rename_path,
    save_path,
    validate_name,
)


class PathStoreTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.folder = self.directory.name

    def tearDown(self):
        self.directory.cleanup()

    def test_round_trip(self):
        path = PatrolPath("lager", "once", False, [(1.0, 2.0, 0.5), (-3.25, 4.0, -1.2)])
        save_path(self.folder, path)
        loaded = load_path(self.folder, "lager")
        self.assertEqual(loaded.mode, "once")
        self.assertFalse(loaded.closed)
        self.assertEqual(loaded.waypoints, path.waypoints)

    def test_closed_route_returns_to_the_first_waypoint(self):
        points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
        self.assertEqual(PatrolPath("a", closed=True, waypoints=points).route(), points + [points[0]])
        self.assertEqual(PatrolPath("a", closed=False, waypoints=points).route(), points)
        self.assertEqual(PatrolPath("a", closed=True, waypoints=points[:1]).route(), points[:1])

    def test_names(self):
        self.assertEqual(validate_name("start_dock_cycle"), "start_dock_cycle")
        for name in ("cancel", "status", "Lager", "1st", "with-dash", "a b", ""):
            with self.assertRaises(ValueError, msg=name):
                validate_name(name)

    def test_list_rename_delete(self):
        save_path(self.folder, PatrolPath("b", waypoints=[(0.0, 0.0, 0.0)]))
        save_path(self.folder, PatrolPath("a"))
        with open(f"{self.folder}/broken.yaml", "w", encoding="utf-8") as handle:
            handle.write("mode: sightseeing\n")
        paths, errors = list_paths(self.folder)
        self.assertEqual([p.name for p in paths], ["a", "b"])
        self.assertEqual(len(errors), 1)
        before = directory_stamp(self.folder)
        rename_path(self.folder, "b", "c")
        self.assertNotEqual(directory_stamp(self.folder), before)
        self.assertEqual(load_path(self.folder, "c").waypoints, [(0.0, 0.0, 0.0)])
        with self.assertRaises(FileNotFoundError):
            load_path(self.folder, "b")
        with self.assertRaises(FileExistsError):
            rename_path(self.folder, "a", "c")
        delete_path(self.folder, "a")
        self.assertEqual([p.name for p in list_paths(self.folder)[0]], ["c"])

    def test_invalid_waypoint_is_rejected(self):
        with open(f"{self.folder}/x.yaml", "w", encoding="utf-8") as handle:
            handle.write("waypoints:\n  - [1.0, 2.0]\n")
        with self.assertRaises(ValueError):
            load_path(self.folder, "x")


if __name__ == "__main__":
    unittest.main()
