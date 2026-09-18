"""Automatically rotate Bagheera by one gyro-measured revolution."""

from __future__ import annotations

import argparse
from collections import deque
import math
import statistics
import threading
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class GyroTurn(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_gyro_turn_test")
        self._lock = threading.Lock()
        self._samples: deque[tuple[float, float]] = deque(maxlen=500)
        self._last_stamp: float | None = None
        self._angle = 0.0
        self._bias = 0.0
        self._integrating = False
        self._command = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self.create_subscription(
            Imu, "/imu/wt901/data_raw", self._on_imu, qos_profile_sensor_data
        )

    def _on_imu(self, message: Imu) -> None:
        now = time.monotonic()
        angular = float(message.angular_velocity.z)
        with self._lock:
            self._samples.append((now, angular))
            if self._integrating and self._last_stamp is not None:
                elapsed = now - self._last_stamp
                if 0.0 < elapsed <= 0.25:
                    self._angle += (angular - self._bias) * elapsed
            self._last_stamp = now

    def ready(self) -> bool:
        with self._lock:
            gyro_fresh = (
                bool(self._samples)
                and time.monotonic() - self._samples[-1][0] < 0.5
            )
        return gyro_fresh and self._command.get_subscription_count() > 0

    def estimate_bias(self, duration: float) -> float:
        start = time.monotonic()
        while time.monotonic() - start < duration:
            if not self.ready():
                raise RuntimeError("WT901 gyro data or command path became unavailable")
            time.sleep(0.05)
        with self._lock:
            values = [value for stamp, value in self._samples if stamp >= start]
        if len(values) < 10:
            raise RuntimeError("not enough stationary gyro samples")
        self._bias = statistics.median(values)
        return self._bias

    def begin(self) -> None:
        with self._lock:
            self._angle = 0.0
            self._last_stamp = self._samples[-1][0]
            self._integrating = True

    def angle(self) -> float:
        with self._lock:
            return self._angle

    def publish(self, angular: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.angular.z = angular
        self._command.publish(message)

    def stop(self) -> None:
        with self._lock:
            self._integrating = False
        for _ in range(30):
            self.publish(0.0)
            time.sleep(0.025)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Gyro-controlled 360 degree turn")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--clockwise", action="store_true")
    parser.add_argument("--maximum-speed", type=float, default=0.30)
    parser.add_argument("--minimum-speed", type=float, default=0.18)
    parser.add_argument("--gain", type=float, default=0.7)
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    parser.add_argument("--bias-time", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--countdown", type=int, default=5)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("automatic motion is disabled unless --execute is supplied")
    if not 0.0 < args.minimum_speed <= args.maximum_speed <= 0.5:
        parser.error("speeds must satisfy 0 < minimum <= maximum <= 0.5 rad/s")

    rclpy.init()
    node = GyroTurn()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    failed = False
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not node.ready():
            time.sleep(0.1)
        if not node.ready():
            raise RuntimeError("WT901 gyro or /cmd_vel_tuning is unavailable")

        print(f"Gyro-Offset wird {args.bias_time:.1f} s im Stillstand gemessen.")
        bias = node.estimate_bias(args.bias_time)
        print(f"Gyro-Offset: {bias:+.5f} rad/s")
        for remaining in range(max(0, args.countdown), 0, -1):
            print(f"Start in {remaining} ...", flush=True)
            time.sleep(1.0)

        direction = -1.0 if args.clockwise else 1.0
        target = direction * 2.0 * math.pi
        tolerance = math.radians(args.tolerance_deg)
        node.begin()
        deadline = time.monotonic() + args.timeout
        next_report = 45.0
        while time.monotonic() < deadline:
            if not node.ready():
                raise RuntimeError("WT901 gyro or command path became unavailable")
            angle = node.angle()
            remaining = target - angle
            if abs(remaining) <= tolerance or direction * remaining <= 0.0:
                break
            speed = min(
                args.maximum_speed,
                max(args.minimum_speed, args.gain * abs(remaining)),
            )
            node.publish(math.copysign(speed, remaining))
            progress = abs(math.degrees(angle))
            if progress >= next_report:
                print(f"Gyro-Winkel: {math.degrees(angle):+.1f} deg")
                next_report += 45.0
            time.sleep(0.04)
        else:
            raise TimeoutError(f"360 degrees not reached within {args.timeout:.1f} s")
        node.stop()
        print(f"Fertig: Gyro-Winkel {math.degrees(node.angle()):+.1f} deg")
    except KeyboardInterrupt:
        failed = True
        print("\nAbgebrochen.")
    except Exception as error:
        failed = True
        print(f"Test fehlgeschlagen: {error}")
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=2.0)
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
