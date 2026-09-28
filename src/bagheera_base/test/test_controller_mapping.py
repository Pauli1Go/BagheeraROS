import unittest

from bagheera_base.controller_mapping import apply_deadzone, axes_to_twist


class ControllerMappingTest(unittest.TestCase):
    def test_controller_mapping_separates_driving_and_turning(self):
        self.assertEqual(apply_deadzone(0.04, 0.05), 0.0)
        linear, angular = axes_to_twist(-1.0, 1.0, 0.05, 0.25, 1.0)
        self.assertEqual(linear, 0.25)
        self.assertEqual(angular, 0.0)
        linear, angular = axes_to_twist(0.0, 1.0, 0.05, 0.25, 1.0)
        self.assertEqual(linear, 0.0)
        self.assertEqual(angular, -1.0)


if __name__ == "__main__":
    unittest.main()
