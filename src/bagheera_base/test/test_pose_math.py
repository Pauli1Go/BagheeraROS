import math
import unittest

from bagheera_base.pose_math import OdomHistory, compose, map_pose, relative


class PoseAlgebraTest(unittest.TestCase):
    def assertPose(self, actual, expected, places=6):
        self.assertAlmostEqual(actual[0], expected[0], places=places)
        self.assertAlmostEqual(actual[1], expected[1], places=places)
        self.assertAlmostEqual(
            math.atan2(math.sin(actual[2] - expected[2]), math.cos(actual[2] - expected[2])),
            0.0,
            places=places,
        )

    def test_relative_inverts_compose(self):
        a = (1.0, -2.0, 2.5)
        b = (0.3, 0.4, -1.0)
        self.assertPose(relative(a, compose(a, b)), b)

    def test_final_turn_after_last_amcl_pose_is_kept(self):
        # The case that restored a heading 40 deg off: AMCL published its last
        # pose before the final turn, because it only updates after 10 cm or
        # ~6 deg. The odometry since that scan must carry the turn.
        amcl = (2.0, 3.0, math.radians(90.0))
        odom_at_scan = (5.0, 1.0, math.radians(10.0))
        odom_now = (5.0, 1.0, math.radians(50.0))
        self.assertPose(map_pose(amcl, odom_at_scan, odom_now), (2.0, 3.0, math.radians(130.0)))

    def test_motion_is_rotated_into_the_map_frame(self):
        # map and odom frames differ by 90 deg: 1 m forward in odom (+x while
        # the robot faces odom +x) is 1 m along the robot's map heading (+y).
        amcl = (0.0, 0.0, math.radians(90.0))
        odom_at_scan = (0.0, 0.0, 0.0)
        odom_now = (1.0, 0.0, 0.0)
        self.assertPose(map_pose(amcl, odom_at_scan, odom_now), (0.0, 1.0, math.radians(90.0)))

    def test_matches_tf_chain(self):
        # TF: map->base_link = (map->odom) * (odom->base_link now), with
        # map->odom = amcl * (odom at scan)^-1.
        amcl = (-3.2, 4.1, -2.7)
        odom_at_scan = (10.0, -4.0, 0.9)
        odom_now = (10.4, -3.7, 1.6)
        map_to_odom = compose(amcl, relative(odom_at_scan, (0.0, 0.0, 0.0)))
        self.assertPose(map_pose(amcl, odom_at_scan, odom_now), compose(map_to_odom, odom_now))


class OdomHistoryTest(unittest.TestCase):
    def test_interpolates_between_samples(self):
        history = OdomHistory()
        history.add(10.0, (0.0, 0.0, math.radians(170.0)))
        history.add(10.1, (0.1, 0.2, math.radians(-170.0)))
        pose = history.at(10.05)
        self.assertAlmostEqual(pose[0], 0.05)
        self.assertAlmostEqual(pose[1], 0.1)
        # Across +-180 deg the shorter way round.
        self.assertAlmostEqual(abs(pose[2]), math.pi)

    def test_small_lead_uses_latest_sample(self):
        history = OdomHistory(max_extrapolation_s=0.2)
        history.add(10.0, (1.0, 2.0, 0.3))
        self.assertEqual(history.at(10.1), (1.0, 2.0, 0.3))
        self.assertIsNone(history.at(10.5))

    def test_old_stamps_are_not_covered(self):
        history = OdomHistory(max_age_s=1.0)
        for i in range(30):
            history.add(i * 0.1, (i * 0.1, 0.0, 0.0))
        self.assertIsNone(history.at(0.5))
        self.assertAlmostEqual(history.at(2.25)[0], 2.25)

    def test_time_going_backwards_clears_history(self):
        history = OdomHistory()
        history.add(100.0, (1.0, 0.0, 0.0))
        history.add(5.0, (2.0, 0.0, 0.0))
        self.assertEqual(history.latest(), (5.0, (2.0, 0.0, 0.0)))
        self.assertIsNone(history.at(100.0 - 50.0))


if __name__ == "__main__":
    unittest.main()
