import math
import unittest

from nav_msgs.msg import OccupancyGrid

from bagheera_base.goal_pose_bridge import (
    DEFAULT_FOOTPRINT,
    blocked_footprint_cell,
    parse_footprint,
)


def office_corner_grid():
    """5 cm grid around the 2026-09-23 goal: wall from y=-0.87 downwards."""
    grid = OccupancyGrid()
    grid.info.resolution = 0.05
    grid.info.width = 60
    grid.info.height = 60
    grid.info.origin.position.x = -2.0
    grid.info.origin.position.y = -1.5
    grid.info.origin.orientation.w = 1.0
    data = []
    for row in range(grid.info.height):
        y = grid.info.origin.position.y + (row + 0.5) * grid.info.resolution
        data.extend([100 if y <= -0.87 else 0] * grid.info.width)
    grid.data = data
    return grid


class GoalFootprintTest(unittest.TestCase):
    def setUp(self):
        self.grid = office_corner_grid()
        self.footprint = parse_footprint(DEFAULT_FOOTPRINT)

    def blocked(self, yaw_deg):
        return blocked_footprint_cell(
            self.grid, self.footprint, -0.75, -0.54, math.radians(yaw_deg), 65
        )

    def test_goal_facing_away_from_wall_is_free(self):
        # The axle is 0.33 m from the wall; only 0.09 m of chassis is behind it.
        self.assertIsNone(self.blocked(90.0))

    def test_nose_towards_wall_is_blocked(self):
        # 0.44 m nose pointing south-west reaches into the wall.
        self.assertIsNotNone(self.blocked(-150.0))
        self.assertIsNotNone(self.blocked(-90.0))

    def test_unknown_cells_block(self):
        self.grid.data = [-1] * (self.grid.info.width * self.grid.info.height)
        self.assertIsNotNone(self.blocked(90.0))


if __name__ == "__main__":
    unittest.main()
