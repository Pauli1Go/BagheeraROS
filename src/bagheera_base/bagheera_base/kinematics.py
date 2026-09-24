"""Rigid-body twist helpers shifting axle-centred odometry to base_link."""

import math


def axle_to_base_link_twist(
    linear_m_s: float, angular_rad_s: float, axle_to_base_link_m: float
) -> tuple[float, float]:
    """Convert axle-frame twist to the twist of a point ahead of the axle.

    The wheel odometry describes the axle midpoint (the pivot center), but
    base_link sits at the LiDAR position, axle_to_base_link_m ahead of the
    axle. A rotating rigid body moves that point laterally with wz x r, so
    the point velocity gains a y component while x stays the same.
    """
    if not all(
        math.isfinite(v)
        for v in (linear_m_s, angular_rad_s, axle_to_base_link_m)
    ):
        return 0.0, 0.0
    return linear_m_s, angular_rad_s * axle_to_base_link_m


def shift_twist_covariance_x(covariance: list[float], offset_x: float) -> list[float]:
    """Shift a ROS 6D twist covariance to a point ``offset_x`` ahead.

    For a planar rigid body ``vy' = vy + offset_x * wz``. Applying the same
    Jacobian to the covariance keeps the y/yaw variance and correlation
    consistent when axle-centred wheel odometry is labelled as base_link.
    """
    if len(covariance) != 36 or not math.isfinite(offset_x):
        raise ValueError("twist covariance must be 6x6 and offset_x finite")
    matrix = [list(covariance[row * 6:(row + 1) * 6]) for row in range(6)]
    jacobian = [[float(row == column) for column in range(6)] for row in range(6)]
    jacobian[1][5] = offset_x
    left = [
        [sum(jacobian[row][k] * matrix[k][column] for k in range(6))
         for column in range(6)]
        for row in range(6)
    ]
    shifted = [
        [sum(left[row][k] * jacobian[column][k] for k in range(6))
         for column in range(6)]
        for row in range(6)
    ]
    return [shifted[row][column] for row in range(6) for column in range(6)]
