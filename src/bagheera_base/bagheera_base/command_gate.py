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
