from collections import deque
import unittest

try:
    from bagheera_base.dock_sleep import scan_is_steady
except ModuleNotFoundError as error:  # needs a ROS 2 environment
    scan_is_steady = None
    IMPORT_ERROR = error


@unittest.skipIf(scan_is_steady is None, "ROS 2 message packages not available")
class ScanIsSteadyTest(unittest.TestCase):
    def test_ten_hertz_is_steady(self):
        stamps = deque(i * 0.1 for i in range(21))
        self.assertTrue(scan_is_steady(stamps, 2.0, 2.0, 8.0))

    def test_startup_gap_is_not_steady(self):
        stamps = deque([0.0, 0.1, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0])
        self.assertFalse(scan_is_steady(stamps, 2.0, 2.0, 8.0))

    def test_old_scans_expire(self):
        stamps = deque(i * 0.1 for i in range(21))
        self.assertFalse(scan_is_steady(stamps, 5.0, 2.0, 8.0))
        self.assertEqual(len(stamps), 0)


if __name__ == "__main__":
    unittest.main()
