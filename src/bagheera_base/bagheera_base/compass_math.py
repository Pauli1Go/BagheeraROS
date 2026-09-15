"""Pure calibration and heading helpers for the WT901 magnetometer."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import statistics

import yaml


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class CompassCalibration:
    bias_t: tuple[float, float, float]
    matrix: tuple[tuple[float, float, float], ...]
    horizontal_field_t: float
    map_yaw_offset_rad: float
    sample_count: int


def load_calibration(path: str | Path) -> CompassCalibration:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("version") != 1 or not data.get("valid"):
        raise ValueError("compass calibration is missing version=1 and valid=true")
    bias = tuple(float(value) for value in data["bias_t"])
    matrix = tuple(tuple(float(value) for value in row) for row in data["matrix"])
    field = float(data["horizontal_field_t"])
    offset = float(data.get("map_yaw_offset_rad", 0.0))
    sample_count = int(data.get("sample_count", 0))
    values = [*bias, *(value for row in matrix for value in row), field, offset]
    if len(bias) != 3 or len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("compass calibration bias/matrix dimensions are invalid")
    if not all(math.isfinite(value) for value in values) or field <= 0.0:
        raise ValueError("compass calibration contains invalid values")
    return CompassCalibration(bias, matrix, field, offset, sample_count)


def apply_calibration(
    vector: tuple[float, float, float], calibration: CompassCalibration
) -> tuple[float, float, float]:
    centered = tuple(vector[index] - calibration.bias_t[index] for index in range(3))
    return tuple(
        sum(calibration.matrix[row][column] * centered[column] for column in range(3))
        for row in range(3)
    )


def rotate_z(
    vector: tuple[float, float, float], yaw: float
) -> tuple[float, float, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x, y, z = vector
    return cosine * x - sine * y, sine * x + cosine * y, z


def tilt_compensated_heading(
    magnetic: tuple[float, float, float],
    acceleration: tuple[float, float, float],
) -> tuple[float, float]:
    """Return magnetic yaw and horizontal field strength in body axes."""
    ax, ay, az = acceleration
    if not all(math.isfinite(value) for value in (*magnetic, *acceleration)):
        raise ValueError("heading inputs must be finite")
    if math.sqrt(ax * ax + ay * ay + az * az) < 1.0e-6:
        raise ValueError("acceleration vector is zero")
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    mx, my, mz = magnetic
    horizontal_x = mx * math.cos(pitch) + mz * math.sin(pitch)
    horizontal_y = (
        mx * math.sin(roll) * math.sin(pitch)
        + my * math.cos(roll)
        - mz * math.sin(roll) * math.cos(pitch)
    )
    strength = math.hypot(horizontal_x, horizontal_y)
    if strength < 1.0e-9:
        raise ValueError("horizontal magnetic field is too small")
    return math.atan2(-horizontal_y, horizontal_x), strength


def fit_planar_calibration(samples: list[tuple[float, float, float]]) -> dict:
    """Fit hard-/soft-iron correction from a full planar rotation."""
    if len(samples) < 20 or any(len(sample) != 3 for sample in samples):
        raise ValueError("at least 20 three-axis samples are required")
    if not all(math.isfinite(value) for sample in samples for value in sample):
        raise ValueError("magnetometer samples must be finite")

    def percentile(values: list[float], fraction: float) -> float:
        ordered = sorted(values)
        position = fraction * (len(ordered) - 1)
        lower_index = int(math.floor(position))
        upper_index = int(math.ceil(position))
        weight = position - lower_index
        return ordered[lower_index] * (1.0 - weight) + ordered[upper_index] * weight

    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    bias_x = (percentile(xs, 0.01) + percentile(xs, 0.99)) / 2.0
    bias_y = (percentile(ys, 0.01) + percentile(ys, 0.99)) / 2.0
    centered = [(x - bias_x, y - bias_y) for x, y in zip(xs, ys)]
    angles = [math.atan2(y, x) for x, y in centered]
    occupied = len(
        {
            int((angle + math.pi) / (2.0 * math.pi) * 36) % 36
            for angle in angles
        }
    )
    coverage = occupied / 36.0
    if coverage < 0.75:
        raise ValueError(f"rotation coverage only {coverage:.0%}; complete a full 360 degrees")

    divisor = len(centered) - 1
    cov_xx = sum(x * x for x, _ in centered) / divisor
    cov_xy = sum(x * y for x, y in centered) / divisor
    cov_yy = sum(y * y for _, y in centered) / divisor
    half_trace = (cov_xx + cov_yy) / 2.0
    radius = math.hypot((cov_xx - cov_yy) / 2.0, cov_xy)
    eigen_max = half_trace + radius
    eigen_min = half_trace - radius
    if eigen_min <= 1.0e-18 or eigen_max / eigen_min > 100.0:
        raise ValueError("magnetometer ellipse is degenerate")
    target_variance = math.sqrt(eigen_min * eigen_max)
    angle = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    scale_max = math.sqrt(target_variance / eigen_max)
    scale_min = math.sqrt(target_variance / eigen_min)
    correction = (
        (
            cosine * cosine * scale_max + sine * sine * scale_min,
            cosine * sine * (scale_max - scale_min),
        ),
        (
            cosine * sine * (scale_max - scale_min),
            sine * sine * scale_max + cosine * cosine * scale_min,
        ),
    )
    corrected_xy = [
        (
            correction[0][0] * x + correction[0][1] * y,
            correction[1][0] * x + correction[1][1] * y,
        )
        for x, y in centered
    ]
    field = statistics.median(math.hypot(x, y) for x, y in corrected_xy)
    return {
        "bias_t": [bias_x, bias_y, 0.0],
        "matrix": [
            [correction[0][0], correction[0][1], 0.0],
            [correction[1][0], correction[1][1], 0.0],
            [0.0, 0.0, 1.0],
        ],
        "horizontal_field_t": field,
        "coverage": coverage,
    }
