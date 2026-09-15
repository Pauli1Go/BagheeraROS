"""Pure differential-drive kinematics used by the ROS node and unit tests."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def twist_to_wheels(
    linear_m_s: float, angular_rad_s: float, wheel_track_m: float, max_wheel_m_s: float
) -> Tuple[int, int]:
    if not all(math.isfinite(value) for value in (linear_m_s, angular_rad_s)):
        raise ValueError("twist values must be finite")
    if wheel_track_m <= 0.0 or max_wheel_m_s <= 0.0:
        raise ValueError("wheel geometry and limit must be positive")
    left = linear_m_s - angular_rad_s * wheel_track_m / 2.0
    right = linear_m_s + angular_rad_s * wheel_track_m / 2.0
    peak = max(abs(left), abs(right))
    if peak > max_wheel_m_s:
        scale = max_wheel_m_s / peak
        left *= scale
        right *= scale
    return round(left * 1000.0), round(right * 1000.0)


def signed_delta32(current: int, previous: int) -> int:
    return ((current - previous + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class OdometryUpdate:
    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float


class DifferentialOdometry:
    def __init__(self, ticks_per_meter: int, wheel_track_m: float) -> None:
        if ticks_per_meter <= 0 or wheel_track_m <= 0.0:
            raise ValueError("invalid differential-drive geometry")
        self.ticks_per_meter = ticks_per_meter
        self.wheel_track_m = wheel_track_m
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._left_ticks: Optional[int] = None
        self._right_ticks: Optional[int] = None

    def reset_ticks(self, left_ticks: int, right_ticks: int) -> None:
        self._left_ticks = left_ticks
        self._right_ticks = right_ticks

    def update(
        self,
        left_ticks: int,
        right_ticks: int,
        measured_left_m_s: float,
        measured_right_m_s: float,
    ) -> OdometryUpdate:
        if self._left_ticks is None or self._right_ticks is None:
            self.reset_ticks(left_ticks, right_ticks)
        else:
            left_distance = signed_delta32(left_ticks, self._left_ticks) / self.ticks_per_meter
            right_distance = signed_delta32(right_ticks, self._right_ticks) / self.ticks_per_meter
            distance = (left_distance + right_distance) / 2.0
            angle = (right_distance - left_distance) / self.wheel_track_m
            midpoint = self.yaw + angle / 2.0
            self.x += distance * math.cos(midpoint)
            self.y += distance * math.sin(midpoint)
            self.yaw = normalize_angle(self.yaw + angle)
            self.reset_ticks(left_ticks, right_ticks)

        linear = (measured_left_m_s + measured_right_m_s) / 2.0
        angular = (measured_right_m_s - measured_left_m_s) / self.wheel_track_m
        return OdometryUpdate(self.x, self.y, self.yaw, linear, angular)


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
