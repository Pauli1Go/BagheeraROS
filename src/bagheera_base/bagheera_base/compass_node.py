"""Calibrate, validate and publish absolute WT901 compass yaw."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float64

from .compass_math import (
    apply_calibration,
    load_calibration,
    normalize_angle,
    rotate_z,
    tilt_compensated_heading,
)


class CompassNode(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_compass")
        self.declare_parameter("calibration_file", "/bagheera_ws/maps/compass_calibration.yaml")
        self.declare_parameter("sensor_to_base_yaw", -math.pi / 2.0)
        self.declare_parameter("field_tolerance_fraction", 0.35)
        self.declare_parameter("acceleration_tolerance_fraction", 0.30)
        self.declare_parameter("heading_variance", 0.12)
        self.declare_parameter("filter_gain", 0.12)
        self.declare_parameter("max_heading_rate_rad_s", 1.5)
        self.declare_parameter("imu_max_age", 0.25)
        self.declare_parameter("output_frame", "base_link")

        self._path = Path(str(self.get_parameter("calibration_file").value))
        self._sensor_yaw = float(self.get_parameter("sensor_to_base_yaw").value)
        self._field_tolerance = float(
            self.get_parameter("field_tolerance_fraction").value
        )
        self._accel_tolerance = float(
            self.get_parameter("acceleration_tolerance_fraction").value
        )
        self._yaw_variance = float(self.get_parameter("heading_variance").value)
        self._gain = float(self.get_parameter("filter_gain").value)
        self._max_rate = float(self.get_parameter("max_heading_rate_rad_s").value)
        self._imu_max_age_ns = int(
            float(self.get_parameter("imu_max_age").value) * 1.0e9
        )
        self._frame = str(self.get_parameter("output_frame").value)
        if not 0.0 < self._gain <= 1.0:
            raise ValueError("filter_gain must be in (0, 1]")

        self._calibration = None
        self._calibration_mtime_ns = None
        self._acceleration = None
        self._acceleration_stamp_ns = 0
        self._filtered_heading = None
        self._last_heading_stamp_ns = 0
        self._warned_missing = False

        self._imu_pub = self.create_publisher(Imu, "/imu/compass", 10)
        self._heading_pub = self.create_publisher(Float64, "/imu/compass/heading", 10)
        self._valid_pub = self.create_publisher(Bool, "/imu/compass/valid", 10)
        self.create_subscription(
            Imu, "/imu/wt901/data_raw", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            MagneticField,
            "/imu/wt901/mag_raw",
            self._on_magnetic,
            qos_profile_sensor_data,
        )
        self.create_timer(2.0, self._reload_calibration)
        self._reload_calibration()

    def _reload_calibration(self) -> None:
        try:
            mtime = self._path.stat().st_mtime_ns
            if mtime == self._calibration_mtime_ns:
                return
            self._calibration = load_calibration(self._path)
            self._calibration_mtime_ns = mtime
            self._filtered_heading = None
            self._warned_missing = False
            self.get_logger().info(
                f"Loaded compass calibration ({self._calibration.sample_count} samples)"
            )
        except (OSError, KeyError, TypeError, ValueError) as error:
            self._calibration = None
            if not self._warned_missing:
                self.get_logger().warning(
                    f"Compass disabled until calibration is valid: {error}"
                )
                self._warned_missing = True

    def _on_imu(self, message: Imu) -> None:
        self._acceleration = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        self._acceleration_stamp_ns = self.get_clock().now().nanoseconds

    def _invalid(self) -> None:
        message = Bool()
        message.data = False
        self._valid_pub.publish(message)

    def _on_magnetic(self, message: MagneticField) -> None:
        if self._calibration is None or self._acceleration is None:
            self._invalid()
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._acceleration_stamp_ns > self._imu_max_age_ns:
            self._invalid()
            return
        raw = (
            message.magnetic_field.x,
            message.magnetic_field.y,
            message.magnetic_field.z,
        )
        magnetic = rotate_z(apply_calibration(raw, self._calibration), self._sensor_yaw)
        acceleration = rotate_z(self._acceleration, self._sensor_yaw)
        accel_norm = math.sqrt(sum(value * value for value in acceleration))
        if abs(accel_norm - 9.80665) > 9.80665 * self._accel_tolerance:
            self._invalid()
            return
        try:
            magnetic_yaw, field = tilt_compensated_heading(magnetic, acceleration)
        except ValueError:
            self._invalid()
            return
        reference = self._calibration.horizontal_field_t
        if abs(field - reference) > reference * self._field_tolerance:
            self._invalid()
            return
        heading = normalize_angle(
            magnetic_yaw + self._calibration.map_yaw_offset_rad
        )
        if self._filtered_heading is not None:
            elapsed = max(1.0e-3, (now_ns - self._last_heading_stamp_ns) / 1.0e9)
            difference = normalize_angle(heading - self._filtered_heading)
            if abs(difference) > 0.12 + self._max_rate * elapsed:
                self._invalid()
                return
            heading = normalize_angle(self._filtered_heading + self._gain * difference)
        self._filtered_heading = heading
        self._last_heading_stamp_ns = now_ns

        orientation = Imu()
        orientation.header = message.header
        orientation.header.frame_id = self._frame
        orientation.orientation = Quaternion(
            z=math.sin(heading / 2.0), w=math.cos(heading / 2.0)
        )
        orientation.orientation_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, self._yaw_variance,
        ]
        orientation.angular_velocity_covariance[0] = -1.0
        orientation.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(orientation)
        heading_message = Float64()
        heading_message.data = heading
        self._heading_pub.publish(heading_message)
        valid = Bool()
        valid.data = True
        self._valid_pub.publish(valid)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
