"""Fisheye AprilTag pose geometry for the charging dock.

apriltag_ros estimates poses with a pinhole model, which is wrong for the
equidistant fisheye camera. This module undistorts the raw corners and solves
the square tag pose itself. Results are in the REP-103 camera optical frame
(x right, y down, z forward).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class TagPose:
    """Tag center and the horizontal direction pointing into the tag plane."""

    position: tuple[float, float, float]
    # Unit vector in the optical frame, perpendicular to the tag and pointing
    # away from the camera. Its horizontal angle is the dock-axis direction.
    into_tag: tuple[float, float, float]
    edge_pixels: float
    # Angle of into_tag right of the optical axis, in radians.
    heading_right: float


def edge_length_pixels(corners: np.ndarray) -> float:
    return float(
        np.mean(
            [np.linalg.norm(corners[(index + 1) % 4] - corners[index]) for index in range(4)]
        )
    )


def solve_tag_pose(
    corners: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    tag_size: float,
) -> TagPose | None:
    """Solve one tag from its four raw 1080p corner pixels.

    AprilTag corners wind around the square; IPPE_SQUARE expects bottom-left,
    bottom-right, top-right, top-left model points in that order.
    """
    corners = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(corners).all():
        return None
    try:
        undistorted = cv2.fisheye.undistortPoints(
            corners.reshape(-1, 1, 2), camera_matrix, distortion[:4]
        ).reshape(-1, 2)
    except cv2.error:
        return None
    if not np.isfinite(undistorted).all():
        return None
    half = tag_size / 2.0
    object_points = np.asarray(
        [(-half, half, 0.0), (half, half, 0.0), (half, -half, 0.0), (-half, -half, 0.0)],
        dtype=np.float64,
    )
    try:
        success, rotation, translation = cv2.solvePnP(
            object_points,
            undistorted,
            np.eye(3, dtype=np.float64),
            None,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
    except cv2.error:
        return None
    if not success or not np.isfinite(translation).all():
        return None
    translation = translation.reshape(3)
    if translation[2] <= 0.0:
        return None
    normal = cv2.Rodrigues(rotation)[0][:, 2]
    # Make the normal point into the tag, away from the camera.
    if normal[2] < 0.0:
        normal = -normal
    norm = float(np.linalg.norm(normal))
    if not math.isfinite(norm) or norm <= 0.0:
        return None
    into_tag = normal / norm
    return TagPose(
        position=(float(translation[0]), float(translation[1]), float(translation[2])),
        into_tag=(float(into_tag[0]), float(into_tag[1]), float(into_tag[2])),
        edge_pixels=edge_length_pixels(corners),
        heading_right=math.atan2(float(into_tag[0]), float(into_tag[2])),
    )


def axis_quaternion(into_tag: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Orientation (x, y, z, w) whose x axis points into the tag, z axis up.

    In the optical frame "up" is -y. Roll and pitch of the tag are discarded
    by the dock plugin, which uses only the yaw of this x axis.
    """
    x_axis = np.asarray(into_tag, dtype=np.float64)
    up = np.asarray((0.0, -1.0, 0.0))
    x_axis = x_axis - up * float(np.dot(x_axis, up))
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(up, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    matrix = np.column_stack((x_axis, y_axis, z_axis))
    return _matrix_to_quaternion(matrix)


def _matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)
