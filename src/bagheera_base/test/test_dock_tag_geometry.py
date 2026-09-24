import math
import unittest

import cv2
import numpy as np

from bagheera_base.dock_tag_geometry import axis_quaternion, solve_tag_pose

# Bagheera's calibrated 1080p equidistant camera (config/camera_fisheye.yaml).
CAMERA_MATRIX = np.array(
    [[1108.3093375101807, 0.0, 936.9551711306665],
     [0.0, 1106.836865563981, 830.0426994757527],
     [0.0, 0.0, 1.0]]
)
DISTORTION = np.array(
    [-0.03022648709212265, 0.03942297830429717, -0.07474156376331077, 0.02869319131518839]
)


def project_tag(size, position, heading_right):
    """Raw fisheye corners of a vertical tag facing the camera.

    heading_right rotates the into-tag direction to the right of the optical
    axis (about the optical y axis, which points down).
    """
    half = size / 2.0
    model = np.array(
        [(-half, half, 0.0), (half, half, 0.0), (half, -half, 0.0), (-half, -half, 0.0)]
    )
    c, s = math.cos(heading_right), math.sin(heading_right)
    rotation = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    rvec, _ = cv2.Rodrigues(rotation)
    tvec = np.array(position, dtype=np.float64).reshape(3, 1)
    corners, _ = cv2.fisheye.projectPoints(
        model.reshape(-1, 1, 3), rvec, tvec, CAMERA_MATRIX, DISTORTION
    )
    return corners.reshape(4, 2)


class DockTagGeometryTest(unittest.TestCase):
    def test_small_tag_position_at_staging_distance(self):
        corners = project_tag(0.026667, (0.0127, -0.0088, 0.707), 0.0)
        pose = solve_tag_pose(corners, CAMERA_MATRIX, DISTORTION, 0.026667)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.position[0], 0.0127, delta=0.002)
        self.assertAlmostEqual(pose.position[2], 0.707, delta=0.01)
        # The staging recording measured 41.6 px at this distance.
        self.assertAlmostEqual(pose.edge_pixels, 41.6, delta=2.0)

    def test_large_tag_heading_sign(self):
        for heading_deg in (-10.0, 0.0, 4.0):
            corners = project_tag(
                0.088889, (0.017, -0.298, 0.793), math.radians(heading_deg)
            )
            pose = solve_tag_pose(corners, CAMERA_MATRIX, DISTORTION, 0.088889)
            self.assertIsNotNone(pose)
            self.assertAlmostEqual(math.degrees(pose.heading_right), heading_deg, delta=0.5)

    def test_axis_quaternion_is_level_and_matches_heading(self):
        # In the optical frame, x is right and z forward. The quaternion's x
        # axis must point along into_tag with its z axis up (-y optical).
        heading = math.radians(7.0)
        quaternion = axis_quaternion((math.sin(heading), 0.05, math.cos(heading)))
        x, y, z, w = quaternion
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        x_axis, z_axis = rotation[:, 0], rotation[:, 2]
        self.assertAlmostEqual(x_axis[1], 0.0, places=9)
        self.assertAlmostEqual(math.atan2(x_axis[0], x_axis[2]), heading, places=9)
        np.testing.assert_allclose(z_axis, (0.0, -1.0, 0.0), atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(rotation), 1.0, places=9)

    def test_axis_yaw_in_base_link(self):
        # camera_optical_frame -> base_link (camera yaw 0): x_b = z_o,
        # y_b = -x_o, z_b = -y_o. A tag direction to the right is negative yaw.
        heading = math.radians(5.0)
        x, y, z, w = axis_quaternion((math.sin(heading), 0.0, math.cos(heading)))
        optical_to_base = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        base = optical_to_base @ rotation
        self.assertAlmostEqual(math.atan2(base[1, 0], base[0, 0]), -heading, places=9)
        self.assertAlmostEqual(base[2, 2], 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
