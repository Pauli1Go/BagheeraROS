"""Unit tests for the 360-degree heading test analysis."""

import math
import unittest

import numpy as np

from bagheera_base.heading_test_math import (
    FAIL,
    INFO,
    OK,
    WARN,
    Stop,
    assess,
    base_translation,
    fit_proportional,
    grade,
    icp_2d,
    lever_arm_translation,
    rotation,
    scan_to_points,
)

LIDAR_XY = (0.1834, 0.011)
LIDAR_YAW = math.radians(-84.1)


def _room_scan(base_xy, base_yaw, count=500):
    """Ray-cast a 6 x 4 m room with a pillar and a box, seen by the LiDAR."""
    segments = [
        ((-2.0, -1.5), (4.0, -1.5)), ((4.0, -1.5), (4.0, 2.5)),
        ((4.0, 2.5), (-2.0, 2.5)), ((-2.0, 2.5), (-2.0, -1.5)),
        ((1.0, 1.0), (1.4, 1.0)), ((1.4, 1.0), (1.4, 1.3)),
        ((-1.2, -0.8), (-0.6, -0.8)), ((-0.6, -0.8), (-0.6, -0.2)),
    ]
    origin = np.asarray(base_xy) + rotation(base_yaw) @ np.asarray(LIDAR_XY)
    heading = base_yaw + LIDAR_YAW
    increment = 2.0 * math.pi / count
    ranges = []
    for index in range(count):
        angle = heading - math.pi + index * increment
        direction = np.array([math.cos(angle), math.sin(angle)])
        best = math.inf
        for start, end in segments:
            start, end = np.asarray(start), np.asarray(end)
            edge = end - start
            matrix = np.column_stack((direction, -edge))
            if abs(np.linalg.det(matrix)) < 1e-12:
                continue
            distance, fraction = np.linalg.solve(matrix, start - origin)
            if distance > 0.0 and 0.0 <= fraction <= 1.0:
                best = min(best, distance)
        ranges.append(best)
    return scan_to_points(ranges, -math.pi, increment, 0.2, 16.0)


class HeadingTestMathTest(unittest.TestCase):
    def test_icp_recovers_pivot_with_lever_arm(self):
        reference = _room_scan((0.0, 0.0), 0.0)
        for degrees in (45.0, 135.0, 225.0, 360.0):
            true_yaw = math.radians(degrees)
            current = _room_scan((0.0, 0.0), true_yaw)
            guess = true_yaw + math.radians(6.0)  # gyro-like initial error
            result = icp_2d(reference, current, guess,
                            lever_arm_translation(guess, LIDAR_XY, LIDAR_YAW))
            self.assertIsNotNone(result)
            self.assertAlmostEqual(result.yaw, true_yaw, delta=math.radians(0.3))
            self.assertLess(result.rms, 0.02)
            self.assertLess(base_translation(result, LIDAR_XY, LIDAR_YAW), 0.01)

    def test_icp_reports_off_axis_pivot(self):
        reference = _room_scan((0.0, 0.0), 0.0)
        current = _room_scan((0.06, 0.0), math.radians(90.0))
        yaw = math.radians(90.0)
        result = icp_2d(reference, current, yaw,
                        lever_arm_translation(yaw, LIDAR_XY, LIDAR_YAW))
        self.assertAlmostEqual(base_translation(result, LIDAR_XY, LIDAR_YAW), 0.06, delta=0.01)

    def test_grade_and_fit(self):
        self.assertEqual(grade(1.0, (2.0, 5.0)), OK)
        self.assertEqual(grade(-3.0, (2.0, 5.0)), WARN)
        self.assertEqual(grade(6.0, (2.0, 5.0)), FAIL)
        angles = [math.radians(value) for value in (90, 180, 270, 360)]
        self.assertAlmostEqual(fit_proportional([0.01 * a for a in angles], angles), 0.01)
        self.assertIsNone(fit_proportional([0.0], [math.radians(45)]))

    def _stops(self, gyro_scale=0.0, ekf_offset=0.0, wheel_scale=0.0, lidar_valid=True):
        stops = []
        for index in range(9):
            truth = math.radians(45.0 * index)
            gyro = truth * (1.0 + gyro_scale)
            stops.append(Stop(
                elapsed_s=5.0 * index, target_deg=45.0 * index, lidar=truth,
                lidar_valid=lidar_valid or index == 0, gyro=gyro,
                wheel=truth * (1.0 + wheel_scale), ekf=gyro + ekf_offset * index / 8,
            ))
        return stops

    def _status(self, report, check):
        return next(item for item in report["findings"] if item["check"] == check)["status"]

    def test_clean_run_passes(self):
        report = assess(self._stops(), 0.0, 20.0, 0.01, 0.32)
        self.assertEqual(report["verdict"], OK)
        self.assertTrue(report["drift_acceptable"])

    def test_gyro_scale_is_blamed(self):
        report = assess(self._stops(gyro_scale=0.01), 0.0, 20.0, 0.01, 0.32)
        self.assertEqual(self._status(report, "gyro_scale"), WARN)
        self.assertEqual(self._status(report, "ekf_heading_error"), WARN)
        self.assertEqual(self._status(report, "ekf_follows_gyro"), OK)
        self.assertFalse(report["drift_acceptable"])

    def test_ekf_not_following_gyro_is_blamed(self):
        report = assess(self._stops(ekf_offset=math.radians(4.0)), 0.0, 20.0, 0.01, 0.32)
        self.assertEqual(self._status(report, "gyro_scale"), OK)
        self.assertEqual(self._status(report, "ekf_follows_gyro"), FAIL)

    def test_gyro_bias_is_separated_from_scale(self):
        bias = math.radians(1.0) / 60.0  # 1 deg/min
        stops = self._stops()
        for stop in stops:
            stop.gyro += bias * stop.elapsed_s
            stop.ekf = stop.gyro
        report = assess(stops, bias, 20.0, 0.01, 0.32)
        self.assertEqual(self._status(report, "gyro_bias"), WARN)
        self.assertEqual(self._status(report, "gyro_scale"), OK)

    def test_wheel_slip_is_information_only(self):
        report = assess(self._stops(wheel_scale=-0.06), 0.0, 20.0, 0.01, 0.32)
        self.assertEqual(self._status(report, "wheel_yaw"), INFO)
        self.assertEqual(report["verdict"], OK)
        advice = next(i for i in report["findings"] if i["check"] == "wheel_yaw")["advice"]
        self.assertIn("0.301", advice)

    def test_unreliable_lidar_is_inconclusive(self):
        report = assess(self._stops(lidar_valid=False), 0.0, 20.0, None, 0.32)
        self.assertEqual(report["verdict"], "INCONCLUSIVE")
        self.assertIsNone(report["drift_acceptable"])


if __name__ == "__main__":
    unittest.main()
