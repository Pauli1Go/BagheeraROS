"""Pure helpers used by the interactive compass characterization tool."""

from __future__ import annotations

import math
import statistics


class TrapezoidIntegrator:
    """Incrementally integrate a sampled rate while rejecting long gaps."""

    def __init__(self, maximum_gap: float = 0.5) -> None:
        if maximum_gap <= 0.0:
            raise ValueError("maximum gap must be positive")
        self.maximum_gap = maximum_gap
        self.last_stamp: float | None = None
        self.last_value = 0.0
        self.total = 0.0

    def update(self, stamp: float, value: float) -> float:
        if self.last_stamp is not None:
            elapsed = stamp - self.last_stamp
            if 0.0 < elapsed <= self.maximum_gap:
                self.total += elapsed * (self.last_value + value) / 2.0
        self.last_stamp = stamp
        self.last_value = value
        return self.total


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def circular_mean(values: list[float]) -> float:
    if not values:
        raise ValueError("at least one angle is required")
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) < 1.0e-12:
        raise ValueError("angles have no defined circular mean")
    return math.atan2(sine, cosine)


def circular_std(values: list[float]) -> float:
    """Return circular standard deviation in radians."""
    if not values:
        raise ValueError("at least one angle is required")
    sine = sum(math.sin(value) for value in values) / len(values)
    cosine = sum(math.cos(value) for value in values) / len(values)
    resultant = min(1.0, max(1.0e-12, math.hypot(sine, cosine)))
    return math.sqrt(-2.0 * math.log(resultant))


def unwrap_near(angle: float, reference: float) -> float:
    """Choose the 2*pi-equivalent of angle nearest reference."""
    return reference + normalize_angle(angle - reference)


def integrate_trapezoid(samples: list[tuple[float, float]], start: float, end: float) -> float:
    """Integrate timestamped samples over [start, end]."""
    if end <= start:
        return 0.0
    ordered = sorted((stamp, value) for stamp, value in samples if start <= stamp <= end)
    if len(ordered) < 2:
        return 0.0
    return sum(
        (right_time - left_time) * (left_value + right_value) / 2.0
        for (left_time, left_value), (right_time, right_value)
        in zip(ordered, ordered[1:])
    )


def scan_yaw_from_signatures(
    reference: list[float | None],
    current: list[float | None],
    expected_rad: float,
    angle_increment: float,
    search_half_width_rad: float = math.radians(25.0),
    minimum_pairs: int = 80,
) -> tuple[float, float, int] | None:
    """Estimate planar yaw from two circular range signatures.

    This intentionally searches near the physically marked checkpoint.  It is
    a diagnostic cross-check, not a general-purpose scan matcher.  The score
    is the median absolute range residual in metres.
    """
    if len(reference) != len(current) or not reference:
        raise ValueError("scan signatures must be non-empty and equal in size")
    expected_shift = round(expected_rad / angle_increment)
    half_width = max(1, round(search_half_width_rad / angle_increment))
    best = None
    count = len(reference)
    for shift in range(expected_shift - half_width, expected_shift + half_width + 1):
        residuals = []
        for index, value in enumerate(current):
            other = reference[(index + shift) % count]
            if value is not None and other is not None:
                residuals.append(abs(value - other))
        if len(residuals) < minimum_pairs:
            continue
        score = statistics.median(residuals)
        if best is None or score < best[0]:
            best = score, shift, len(residuals)
    if best is None:
        return None
    score, shift, pairs = best
    return shift * angle_increment, score, pairs
