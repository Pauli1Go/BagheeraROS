"""Voltage-based battery level for Bagheera's 7S Li-Ion pack.

The pack (ARTS 7 ICR/INR 19/66, 25.2 V nominal, 29.4 V full) has no fuel
gauge, and the mainboard measures current only in the charge path. The state
of charge is therefore estimated from the averaged pack voltage alone. The
percentage is an approximate resting-voltage mapping (about +-5..10 %): under
driving load it reads low, while charging it reads high.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque


NORMAL = "NORMAL"
LOW = "LOW"
CRITICAL = "CRITICAL"
FULL = "FULL"

# Typical 18650 resting voltage per cell -> state of charge in percent.
CELL_OCV_TABLE = (
    (3.00, 0.0),
    (3.40, 5.0),
    (3.50, 10.0),
    (3.61, 20.0),
    (3.66, 30.0),
    (3.71, 40.0),
    (3.76, 50.0),
    (3.82, 60.0),
    (3.89, 70.0),
    (3.98, 80.0),
    (4.08, 90.0),
    (4.20, 100.0),
)


def percentage_from_voltage(pack_voltage: float, cell_count: int) -> float:
    """Linear interpolation in CELL_OCV_TABLE, clamped to 0..100 %."""
    cell = pack_voltage / cell_count
    voltages = [row[0] for row in CELL_OCV_TABLE]
    if cell <= voltages[0]:
        return 0.0
    if cell >= voltages[-1]:
        return 100.0
    index = bisect_left(voltages, cell)
    (v0, p0), (v1, p1) = CELL_OCV_TABLE[index - 1], CELL_OCV_TABLE[index]
    return p0 + (cell - v0) / (v1 - v0) * (p1 - p0)


class TimeWindowAverage:
    """Mean of the samples received during the last window_s seconds."""

    def __init__(self, window_s: float) -> None:
        self._window = window_s
        self._samples: deque[tuple[float, float]] = deque()
        self._sum = 0.0

    def reset(self) -> None:
        self._samples.clear()
        self._sum = 0.0

    def add(self, now: float, value: float) -> None:
        self._samples.append((now, value))
        self._sum += value
        while now - self._samples[0][0] > self._window:
            self._sum -= self._samples.popleft()[1]

    def span(self) -> float:
        """Time covered by the stored samples."""
        if not self._samples:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    def mean(self) -> float | None:
        if not self._samples:
            return None
        return self._sum / len(self._samples)


class BatteryLevelTracker:
    """Averages the pack voltage and derives NORMAL / LOW / CRITICAL / FULL.

    LOW and CRITICAL latch: they are only cleared once the robot actually
    charges in the dock, so a voltage recovering at rest cannot cancel a
    pending return to the dock. FULL needs the dock, a high averaged voltage
    and a small charge current, all held for full_hold_s.
    """

    def __init__(
        self,
        *,
        cell_count: int = 7,
        window_s: float = 60.0,
        warmup_s: float = 10.0,
        low_voltage: float = 25.3,
        critical_voltage: float = 24.5,
        charging_current: float = 0.2,
        full_voltage: float = 28.4,
        full_current: float = 0.15,
        full_hold_s: float = 120.0,
        full_release_voltage: float = 28.0,
    ) -> None:
        if not critical_voltage < low_voltage:
            raise ValueError("critical_voltage must be below low_voltage")
        if not full_release_voltage < full_voltage:
            raise ValueError("full_release_voltage must be below full_voltage")
        self.cell_count = cell_count
        self._warmup = warmup_s
        self._low = low_voltage
        self._critical = critical_voltage
        self._charging_current = charging_current
        self._full_voltage = full_voltage
        self._full_current = full_current
        self._full_hold = full_hold_s
        self._full_release = full_release_voltage
        self._average = TimeWindowAverage(window_s)
        self._docked: bool | None = None
        self._full_candidate_since: float | None = None
        self.level = NORMAL
        self.voltage: float | None = None

    @property
    def percentage(self) -> float | None:
        if self.voltage is None:
            return None
        return percentage_from_voltage(self.voltage, self.cell_count)

    def update(self, now: float, voltage: float, docked: bool, charge_current: float) -> str:
        """Feed one corrected pack-voltage sample; returns the current level."""
        if docked != self._docked:
            # Charging lifts the terminal voltage at once; never mix docked
            # and undocked samples in one average.
            self._average.reset()
            self._full_candidate_since = None
            if self._docked and self.level == FULL:
                self.level = NORMAL
            self._docked = docked
        self._average.add(now, voltage)
        if self._average.span() < self._warmup:
            return self.level
        self.voltage = self._average.mean()

        if docked:
            self._update_docked(now, charge_current)
        else:
            self._apply_thresholds()
        return self.level

    def _apply_thresholds(self) -> None:
        if self.voltage < self._critical:
            self.level = CRITICAL
        elif self.voltage < self._low and self.level != CRITICAL:
            self.level = LOW

    def _update_docked(self, now: float, charge_current: float) -> None:
        charging = charge_current >= self._charging_current
        if self.level in (LOW, CRITICAL):
            if charging:
                self.level = NORMAL
            else:
                # In the dock without charge current: keep warning.
                self._apply_thresholds()
            return
        if not charging and self.level != FULL:
            self._apply_thresholds()
            if self.level != NORMAL:
                return
        if self.level == FULL:
            if self.voltage < self._full_release:
                self.level = NORMAL
            return
        if self.voltage >= self._full_voltage and charge_current < self._full_current:
            if self._full_candidate_since is None:
                self._full_candidate_since = now
            if now - self._full_candidate_since >= self._full_hold:
                self.level = FULL
        else:
            self._full_candidate_since = None
