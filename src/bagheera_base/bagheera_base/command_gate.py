"""Small state helpers for command publication."""

from __future__ import annotations


class StopCommandGate:
    """Allow one stop command per transition while passing motion commands."""

    def __init__(self, epsilon: float = 1.0e-9) -> None:
        self._epsilon = epsilon
        self._stopped = False

    def should_publish(self, linear: float, angular: float) -> bool:
        stopped = abs(linear) <= self._epsilon and abs(angular) <= self._epsilon
        if stopped and self._stopped:
            return False
        self._stopped = stopped
        return True


class ProgressWatchdog:
    """Detect a stall: less than min_progress within any window_s."""

    def __init__(self, window_s: float, min_progress: float) -> None:
        self._window = window_s
        self._min_progress = min_progress
        self._window_start = 0.0
        self._window_progress = 0.0

    def reset(self, now: float, progress: float = 0.0) -> None:
        self._window_start = now
        self._window_progress = progress

    def stalled(self, now: float, progress: float) -> bool:
        """Call periodically; True once a full window gained too little."""
        if now - self._window_start < self._window:
            return False
        if progress - self._window_progress < self._min_progress:
            return True
        self.reset(now, progress)
        return False


def undock_turn_speed(
    target: float,
    progress: float,
    tolerance: float,
    gain: float,
    min_speed: float,
    max_speed: float,
) -> float | None:
    """Angular speed for the undock turn, or None once it is done.

    target is signed (+ left, - right, 0 = no turn); progress is the signed
    rotation so far. The speed keeps the target's sign.
    """
    direction = 1.0 if target >= 0.0 else -1.0
    remaining = abs(target) - direction * progress
    if remaining <= tolerance:
        return None
    return direction * min(max_speed, max(min_speed, gain * remaining))


def turn_description(angle: float) -> str:
    """Human-readable turn, e.g. '90.0 deg left', '45.0 deg right', 'no turn'."""
    degrees = abs(angle) * 57.29577951308232
    if degrees < 0.05:
        return "no turn"
    return "%.1f deg %s" % (degrees, "left" if angle > 0.0 else "right")
