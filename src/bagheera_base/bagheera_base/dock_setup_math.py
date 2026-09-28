"""ROS-free geometry of bagheera_dock_setup (see dock_setup.py).

Poses are (x, y, yaw) tuples. The robot stands on the unpowered dock
(``docked_*``), then reverses straight to a point where both tags are in view
(``now_*``). Odometry is continuous between the two.
"""

from __future__ import annotations

import math

from .pose_math import Pose2D, compose, normalize_angle, relative


def circular_mean(values: list[float]) -> float:
    return math.atan2(sum(map(math.sin, values)), sum(map(math.cos, values)))


def mean_pose(samples: list[Pose2D]) -> Pose2D:
    if not samples:
        raise ValueError("no samples")
    return (
        sum(pose[0] for pose in samples) / len(samples),
        sum(pose[1] for pose in samples) / len(samples),
        circular_mean([pose[2] for pose in samples]),
    )


def offsets_from_sample(
    dock: Pose2D, tag: tuple[float, float], axis_yaw: float
) -> tuple[float, float, float]:
    """Plugin parameters that map this tag sample onto the true dock pose.

    dock is the docked base_link pose, tag the ID 1 centre and axis_yaw the raw
    ID 0 direction, all in the same fixed frame. Returns (translation_x,
    translation_y, axis_yaw_offset) in TagChargingDock's conventions.
    """
    dock_x, dock_y, dock_yaw = dock
    dx = dock_x - tag[0]
    dy = dock_y - tag[1]
    translation_x = math.cos(dock_yaw) * dx + math.sin(dock_yaw) * dy
    translation_y = -math.sin(dock_yaw) * dx + math.cos(dock_yaw) * dy
    return translation_x, translation_y, normalize_angle(dock_yaw - axis_yaw)


def docked_map_pose_from_now(map_now: Pose2D, odom_now: Pose2D, odom_docked: Pose2D) -> Pose2D:
    """Docked pose in the map, carried back from the current pose by odometry.

    The yaw of this is the reliable dock yaw: AMCL's heading is poor right at
    the dock, better with both walls in view, and the gyro-fused odometry
    carries it back exactly. x/y suffer from wheel slip while reversing.
    """
    return compose(map_now, relative(odom_now, odom_docked))


def staging_map_pose(dock: Pose2D, offsets: tuple[float, float, float]) -> Pose2D:
    """Map pose of the staging point, given the plugin's staging offsets."""
    return compose(dock, offsets)


def check_undock(reverse_distance_m: float, turn_angle_deg: float) -> None:
    if not math.isfinite(reverse_distance_m) or reverse_distance_m <= 0.0:
        raise ValueError("the reverse distance must be a positive number of metres")
    if not math.isfinite(turn_angle_deg) or abs(turn_angle_deg) > 180.0:
        raise ValueError("the turn must be between -180 and +180 degrees")
