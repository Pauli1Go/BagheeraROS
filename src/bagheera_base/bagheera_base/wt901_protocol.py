"""Decode motion registers from a WIT Motion WT901 over I2C."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


STANDARD_GRAVITY = 9.80665
ACCEL_FULL_SCALE_G = 16.0
GYRO_FULL_SCALE_DPS = 2000.0
MOTION_REGISTER = 0x34
INERTIAL_BLOCK_LENGTH = 12
MOTION_BLOCK_LENGTH = 18
MAG_SENSOR_REGISTER = 0x72

# WITMotion's official conversion table, in microtesla per register count.
MAG_SCALE_UT_PER_COUNT = {
    2: 0.15,
    3: 0.013,
    4: 0.058,
    5: 0.098,
    6: 1.0 / 120.0,
    7: 0.020,
}


@dataclass(frozen=True)
class MotionSample:
    """WT901 acceleration and angular velocity in REP-103 SI units."""

    acceleration: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    magnetic_raw: tuple[int, int, int]


def decode_motion_block(data: bytes | bytearray | list[int]) -> MotionSample:
    """Decode AX..GZ and, when present, HX..HZ."""
    if len(data) not in (INERTIAL_BLOCK_LENGTH, MOTION_BLOCK_LENGTH):
        raise ValueError(
            "WT901 motion block must contain "
            f"{INERTIAL_BLOCK_LENGTH} or {MOTION_BLOCK_LENGTH} bytes, "
            f"got {len(data)}"
        )
    words = struct.unpack(f"<{len(data) // 2}h", bytes(data))
    acceleration_scale = ACCEL_FULL_SCALE_G * STANDARD_GRAVITY / 32768.0
    gyro_scale = GYRO_FULL_SCALE_DPS * math.pi / (180.0 * 32768.0)
    return MotionSample(
        acceleration=tuple(value * acceleration_scale for value in words[0:3]),
        angular_velocity=tuple(value * gyro_scale for value in words[3:6]),
        magnetic_raw=tuple(words[6:9]) if len(words) == 9 else (0, 0, 0),
    )


def magnetic_raw_to_tesla(
    raw: tuple[int, int, int], sensor_type: int
) -> tuple[float, float, float]:
    """Convert HX/HY/HZ registers using WITMotion's sensor-type table."""
    try:
        scale = MAG_SCALE_UT_PER_COUNT[sensor_type] * 1.0e-6
    except KeyError as error:
        raise ValueError(f"unsupported WT901 magnetometer type {sensor_type}") from error
    return tuple(float(value) * scale for value in raw)


class GyroBiasEstimator:
    """Average a fixed number of stationary samples before publishing."""

    def __init__(self, required_samples: int):
        if required_samples < 0:
            raise ValueError("required_samples must not be negative")
        self.required_samples = required_samples
        self.sample_count = 0
        self._sum = [0.0, 0.0, 0.0]
        self.bias = (0.0, 0.0, 0.0)

    @property
    def ready(self) -> bool:
        return self.sample_count >= self.required_samples

    def update(self, angular_velocity: tuple[float, float, float]) -> bool:
        """Consume a calibration sample and return whether calibration is done."""
        if self.ready:
            return True
        for index, value in enumerate(angular_velocity):
            self._sum[index] += value
        self.sample_count += 1
        if self.ready and self.required_samples:
            self.bias = tuple(value / self.required_samples for value in self._sum)
        return self.ready

    def correct(
        self, angular_velocity: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        if not self.ready:
            raise RuntimeError("gyro bias calibration is not complete")
        return tuple(
            value - bias for value, bias in zip(angular_velocity, self.bias)
        )
