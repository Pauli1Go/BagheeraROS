import re
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / "bagheera_base"


class LoggerUsageTest(unittest.TestCase):
    def test_no_deprecated_warn(self):
        """rclpy's deprecated ``warn()`` wrapper hides the real call site.

        Every ``.warn()`` call in a node then shares one logging context, so a
        throttled warn followed by a plain one raises ValueError and kills the
        node (pose_persistence died that way on a dock wake-up).
        """
        offenders = [
            f"{path.name}:{number}"
            for path in sorted(PACKAGE.glob("*.py"))
            for number, line in enumerate(path.read_text().splitlines(), 1)
            if re.search(r"\.warn\(", line)
        ]
        self.assertEqual(offenders, [], "use get_logger().warning() instead")


if __name__ == "__main__":
    unittest.main()
