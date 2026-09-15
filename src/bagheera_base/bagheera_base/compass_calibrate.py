"""Collect one planar rotation and persist WT901 compass calibration."""

from __future__ import annotations

import math
import os
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import MagneticField
import yaml

from .compass_math import (
    CompassCalibration,
    apply_calibration,
    fit_planar_calibration,
    normalize_angle,
    rotate_z,
)


class CompassCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_compass_calibrate")
        self.declare_parameter("duration_s", 60.0)
        self.declare_parameter("minimum_samples", 500)
        self.declare_parameter("output_file", "/bagheera_ws/maps/compass_calibration.yaml")
        self.declare_parameter("sensor_to_base_yaw", -math.pi / 2.0)
        self._duration_ns = int(float(self.get_parameter("duration_s").value) * 1.0e9)
        self._minimum_samples = int(self.get_parameter("minimum_samples").value)
        self._output = Path(str(self.get_parameter("output_file").value))
        self._sensor_yaw = float(self.get_parameter("sensor_to_base_yaw").value)
        self._samples: list[tuple[float, float, float]] = []
        self._start_ns = None
        self._odom_yaw = None
        self.done = False
        self.succeeded = False
        self.create_subscription(
            MagneticField,
            "/imu/wt901/mag_raw",
            self._on_magnetic,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)
        self.get_logger().info(
            "Ready: rotate Bagheera through at least one complete 360 degree turn"
        )

    def _on_odom(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_magnetic(self, message: MagneticField) -> None:
        if self.done:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._start_ns is None:
            self._start_ns = now_ns
        self._samples.append((
            message.magnetic_field.x,
            message.magnetic_field.y,
            message.magnetic_field.z,
        ))
        if now_ns - self._start_ns < self._duration_ns:
            return
        try:
            self._finish()
            self.succeeded = True
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Compass calibration failed: {error}")
        self.done = True

    def _finish(self) -> None:
        if len(self._samples) < self._minimum_samples:
            raise ValueError(
                f"received {len(self._samples)} samples, need {self._minimum_samples}"
            )
        result = fit_planar_calibration(self._samples)
        temporary = CompassCalibration(
            tuple(result["bias_t"]),
            tuple(tuple(row) for row in result["matrix"]),
            float(result["horizontal_field_t"]),
            0.0,
            len(self._samples),
        )
        corrected = rotate_z(
            apply_calibration(self._samples[-1], temporary), self._sensor_yaw
        )
        magnetic_yaw = math.atan2(-corrected[1], corrected[0])
        offset = 0.0 if self._odom_yaw is None else normalize_angle(
            self._odom_yaw - magnetic_yaw
        )
        document = {
            "version": 1,
            "valid": True,
            "bias_t": result["bias_t"],
            "matrix": result["matrix"],
            "horizontal_field_t": result["horizontal_field_t"],
            "map_yaw_offset_rad": offset,
            "sample_count": len(self._samples),
            "coverage": result["coverage"],
        }
        self._output.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self._output.with_suffix(self._output.suffix + ".tmp")
        temporary_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        os.replace(temporary_path, self._output)
        self.get_logger().info(
            f"Saved {len(self._samples)} samples with {result['coverage']:.0%} "
            f"coverage to {self._output}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompassCalibrator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        succeeded = node.succeeded
        node.destroy_node()
        rclpy.try_shutdown()
    if not succeeded:
        raise SystemExit(1)
