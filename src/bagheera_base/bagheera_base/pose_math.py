"""Planar pose algebra for following the map pose without a TF listener.

AMCL publishes ``/amcl_pose`` only after it moved the filter (10 cm or ~6 deg)
and stamps it with the scan it used. The map pose at a later time is that pose
plus the odometry motion since the scan: exactly what the TF chain
``map -> odom -> base_link`` yields, because AMCL's ``map -> odom`` is derived
from the same pose and the odometry at the scan time.
"""

from __future__ import annotations

from collections import deque
import math

Pose2D = tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """Apply the relative pose b in the frame of a."""
    cos_a = math.cos(a[2])
    sin_a = math.sin(a[2])
    return (
        a[0] + cos_a * b[0] - sin_a * b[1],
        a[1] + sin_a * b[0] + cos_a * b[1],
        normalize_angle(a[2] + b[2]),
    )


def relative(a: Pose2D, b: Pose2D) -> Pose2D:
    """Pose of b seen from a, i.e. a^-1 * b."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    cos_a = math.cos(a[2])
    sin_a = math.sin(a[2])
    return (
        cos_a * dx + sin_a * dy,
        -sin_a * dx + cos_a * dy,
        normalize_angle(b[2] - a[2]),
    )


def map_pose(amcl: Pose2D, odom_at_amcl: Pose2D, odom_now: Pose2D) -> Pose2D:
    """Current map pose from an AMCL pose and the odometry since its scan."""
    return compose(amcl, relative(odom_at_amcl, odom_now))


class OdomHistory:
    """Short odometry history, interpolated at an AMCL scan stamp."""

    def __init__(self, max_age_s: float = 5.0, max_extrapolation_s: float = 0.2) -> None:
        self._max_age = max_age_s
        self._max_extrapolation = max_extrapolation_s
        self._samples: deque[tuple[float, Pose2D]] = deque()

    def clear(self) -> None:
        self._samples.clear()

    def add(self, stamp: float, pose: Pose2D) -> None:
        if self._samples and stamp < self._samples[-1][0]:
            # Time went backwards (restarted clock): old samples lie.
            self._samples.clear()
        self._samples.append((stamp, pose))
        while self._samples and stamp - self._samples[0][0] > self._max_age:
            self._samples.popleft()

    def latest(self) -> tuple[float, Pose2D] | None:
        return self._samples[-1] if self._samples else None

    def at(self, stamp: float) -> Pose2D | None:
        """Odometry pose at stamp, or None outside the covered time."""
        if not self._samples:
            return None
        last_stamp, last_pose = self._samples[-1]
        if stamp >= last_stamp:
            # AMCL stamps are scan times, so they are normally older than the
            # newest odometry; a small lead only means the odometry lags.
            return last_pose if stamp - last_stamp <= self._max_extrapolation else None
        if stamp < self._samples[0][0]:
            return None
        previous_stamp, previous_pose = self._samples[0]
        for sample_stamp, sample_pose in self._samples:
            if sample_stamp >= stamp:
                span = sample_stamp - previous_stamp
                if span <= 0.0:
                    return sample_pose
                ratio = (stamp - previous_stamp) / span
                return (
                    previous_pose[0] + ratio * (sample_pose[0] - previous_pose[0]),
                    previous_pose[1] + ratio * (sample_pose[1] - previous_pose[1]),
                    normalize_angle(
                        previous_pose[2]
                        + ratio * normalize_angle(sample_pose[2] - previous_pose[2])
                    ),
                )
            previous_stamp, previous_pose = sample_stamp, sample_pose
        return last_pose
