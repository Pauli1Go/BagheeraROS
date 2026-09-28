"""ROS-free decisions of the waypoint patrol (see patrol.py).

Two modes drive the same closed waypoint loop:

- ``charge``: patrol until the battery is LOW (finish the current waypoint,
  then dock) or CRITICAL (dock at once), charge until FULL, resume.
- ``dock_cycle``: dock after every lap, pause, undock and continue. A LOW or
  CRITICAL battery additionally charges until FULL before continuing.

A Nav2 abort is not always an unreachable waypoint. Costmap update timeouts,
TF/extrapolation errors, rejected goals and planner failures also happen when
the navigation data is late (CPU load, right after a wake-up); such a waypoint
is retried a few times before it counts as skipped. Controller patience,
missing progress and collisions still skip at once: there the robot really
tried to drive, and an obstacle is the likely cause.
"""

from __future__ import annotations

import math

from .battery_math import CRITICAL, LOW

CHARGE = "charge"
DOCK_CYCLE = "dock_cycle"
MODES = (CHARGE, DOCK_CYCLE)

# Next steps returned by PatrolPlan.
NAVIGATE = "navigate"
DOCK = "dock"
DOCK_AND_STOP = "dock_and_stop"
WAIT_FOR_FULL = "wait_for_full"
PAUSE = "pause"

# Lower-case fragments of Nav2 error messages that point to late or missing
# navigation data rather than to an unreachable waypoint.
TRANSIENT_ERRORS = (
    "costmap timed out",
    "timed out waiting for update",
    "transform",
    "extrapolation",
    "failed to plan",
    "failed to create plan",
    "legal potential",
    "goal rejected",
)


def is_transient_failure(detail: str) -> bool:
    """True when a Nav2 failure is worth retrying the same waypoint."""
    text = detail.lower()
    return any(fragment in text for fragment in TRANSIENT_ERRORS)


def parse_waypoints(values: list[float]) -> list[tuple[float, float, float]]:
    """Flat [x, y, yaw, x, y, yaw, ...] parameter list -> (x, y, yaw) tuples."""
    if not values or len(values) % 3:
        raise ValueError("waypoints must be a non-empty list of x, y, yaw triples")
    waypoints = [
        (float(values[i]), float(values[i + 1]), float(values[i + 2]))
        for i in range(0, len(values), 3)
    ]
    if not all(math.isfinite(v) for waypoint in waypoints for v in waypoint):
        raise ValueError("waypoints contain non-finite values")
    return waypoints


class PatrolPlan:
    """Waypoint index, lap counting and the dock/charge decisions."""

    def __init__(
        self,
        count: int,
        mode: str,
        max_consecutive_failures: int = 3,
        max_transient_retries: int = 2,
    ) -> None:
        if count < 1:
            raise ValueError("a patrol needs at least one waypoint")
        if mode not in MODES:
            raise ValueError(f"unknown patrol mode '{mode}'")
        self.count = count
        self.mode = mode
        self.max_consecutive_failures = max_consecutive_failures
        self.max_transient_retries = max_transient_retries
        self.index = 0
        self.laps = 0
        self.reached = 0
        self.skipped = 0
        self.consecutive_failures = 0
        # Retries of the current waypoint, and of the whole run.
        self.retries = 0
        self.retried = 0
        # Set when a battery warning sent the robot to the dock. The monitor
        # clears LOW/CRITICAL as soon as charging starts, so the dock phase
        # must remember why it docked.
        self.charge_to_full = False

    def start(self, level: str, docked: bool) -> str:
        """First step after a start trigger."""
        if level in (LOW, CRITICAL):
            self.charge_to_full = True
            return WAIT_FOR_FULL if docked else DOCK
        return NAVIGATE

    def retry_after(self, detail: str) -> bool:
        """Nav2 aborted the current waypoint: True to drive to it again."""
        if not is_transient_failure(detail) or self.retries >= self.max_transient_retries:
            return False
        self.retries += 1
        self.retried += 1
        return True

    def waypoint_done(self, reached: bool, level: str) -> str:
        """The current waypoint was reached or given up (Nav2 aborted)."""
        self.retries = 0
        if reached:
            self.reached += 1
            self.consecutive_failures = 0
        else:
            self.skipped += 1
            self.consecutive_failures += 1
        self.index += 1
        lap_complete = self.index >= self.count
        if lap_complete:
            self.index = 0
            self.laps += 1
        if self.consecutive_failures >= self.max_consecutive_failures:
            return DOCK_AND_STOP
        if level in (LOW, CRITICAL):
            self.charge_to_full = True
            return DOCK
        if self.mode == DOCK_CYCLE and lap_complete:
            return DOCK
        return NAVIGATE

    def critical_during_navigation(self) -> str:
        """CRITICAL while driving: abandon the waypoint, retry it later."""
        self.retries = 0
        self.charge_to_full = True
        return DOCK

    def docked(self) -> str:
        """Docking succeeded; how long to stay."""
        if self.charge_to_full or self.mode == CHARGE:
            return WAIT_FOR_FULL
        return PAUSE

    def charged(self) -> None:
        self.charge_to_full = False
