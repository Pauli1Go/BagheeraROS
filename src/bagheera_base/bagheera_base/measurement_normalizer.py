"""Normalize MowgliNext measurements for standard ROS consumers."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Imu

from .kinematics import axle_to_base_link_twist, shift_twist_covariance_x


def _all_zero(values) -> bool:
    return all(value == 0.0 for value in values)


def _diagonal_covariance(diagonal: list[float]) -> list[float]:
    covariance = [0.0] * 36
    for index, value in enumerate(diagonal):
        covariance[index * 6 + index] = value
    return covariance


def _imu_covariance(diagonal: list[float]) -> list[float]:
    covariance = [0.0] * 9
    for index, value in enumerate(diagonal):
        covariance[index * 3 + index] = value
    return covariance


class MeasurementNormalizer(Node):
    """Fill required frames, valid quaternions and non-zero covariances."""

    def __init__(self) -> None:
        super().__init__("bagheera_measurement_normalizer")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("axle_to_base_link_m", 0.0)
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("camera_frame_id", "camera_optical_frame")
        self.declare_parameter("camera_width", 1920)
        self.declare_parameter("camera_height", 1080)
        self.declare_parameter("camera_nominal_fx", 960.0)
        self.declare_parameter("camera_nominal_fy", 960.0)
        self.declare_parameter("camera_nominal_cx", 960.0)
        self.declare_parameter("camera_nominal_cy", 540.0)
        self.declare_parameter(
            "wheel_pose_variance", [0.04, 0.09, 1.0e6, 1.0e6, 1.0e6, 0.09]
        )
        self.declare_parameter(
            "wheel_twist_variance", [0.01, 0.01, 1.0e6, 1.0e6, 1.0e6, 0.0009]
        )
        self.declare_parameter("imu_orientation_variance", [0.0027, 0.0027, 0.01])
        self.declare_parameter("imu_angular_velocity_variance", [0.0004] * 3)
        self.declare_parameter("imu_linear_acceleration_variance", [0.04] * 3)

        self._odom_frame = str(self.get_parameter("odom_frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame_id").value)
        self._axle_to_base_link_m = float(
            self.get_parameter("axle_to_base_link_m").value
        )
        if not math.isfinite(self._axle_to_base_link_m):
            raise ValueError("axle_to_base_link_m must be finite")
        self._imu_frame = str(self.get_parameter("imu_frame_id").value)
        self._camera_frame = str(self.get_parameter("camera_frame_id").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_nominal_fx = float(self.get_parameter("camera_nominal_fx").value)
        self._camera_nominal_fy = float(self.get_parameter("camera_nominal_fy").value)
        self._camera_nominal_cx = float(self.get_parameter("camera_nominal_cx").value)
        self._camera_nominal_cy = float(self.get_parameter("camera_nominal_cy").value)
        if self._camera_width <= 0 or self._camera_height <= 0:
            raise ValueError("camera_width and camera_height must be positive")
        if self._camera_nominal_fx <= 0.0 or self._camera_nominal_fy <= 0.0:
            raise ValueError("camera_nominal_fx and camera_nominal_fy must be positive")
        self._wheel_pose_covariance = self._read_covariance("wheel_pose_variance", 6)
        self._wheel_twist_covariance = self._read_covariance("wheel_twist_variance", 6)
        self._imu_orientation_covariance = self._read_covariance(
            "imu_orientation_variance", 3
        )
        self._imu_angular_velocity_covariance = self._read_covariance(
            "imu_angular_velocity_variance", 3
        )
        self._imu_linear_acceleration_covariance = self._read_covariance(
            "imu_linear_acceleration_variance", 3
        )

        self._wheel_publisher = self.create_publisher(Odometry, "/wheel_odom", 20)
        self._imu_publisher = self.create_publisher(Imu, "/imu/data", 20)
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, "/camera/camera_info", 10
        )
        self._wheel_subscription = self.create_subscription(
            Odometry, "/wheel_odom_raw", self._normalize_wheel, 20
        )
        self._imu_subscription = self.create_subscription(
            Imu, "/imu/data_raw", self._normalize_imu, 20
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/camera/camera_info_raw",
            self._normalize_camera_info,
            10,
        )

    def _read_covariance(self, name: str, length: int) -> list[float]:
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != length or any(
            not math.isfinite(value) or value < 0.0 for value in values
        ):
            raise ValueError(f"{name} must contain {length} finite non-negative values")
        return values

    def _ensure_stamp(self, header) -> None:
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            header.stamp = self.get_clock().now().to_msg()

    def _normalize_wheel(self, message: Odometry) -> None:
        self._ensure_stamp(message.header)
        if not message.header.frame_id:
            message.header.frame_id = self._odom_frame
        if not message.child_frame_id:
            message.child_frame_id = self._base_frame

        quaternion = message.pose.pose.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm <= 1.0e-12 or not math.isfinite(norm):
            quaternion.x = quaternion.y = quaternion.z = 0.0
            quaternion.w = 1.0
        else:
            quaternion.x /= norm
            quaternion.y /= norm
            quaternion.z /= norm
            quaternion.w /= norm

        if _all_zero(message.pose.covariance):
            message.pose.covariance = _diagonal_covariance(
                self._wheel_pose_covariance
            )
        if _all_zero(message.twist.covariance):
            message.twist.covariance = _diagonal_covariance(
                self._wheel_twist_covariance
            )
        if self._axle_to_base_link_m != 0.0:
            twist = message.twist.twist
            axle_vx = twist.linear.x
            axle_vy = twist.linear.y
            base_vx, offset_vy = axle_to_base_link_twist(
                axle_vx, twist.angular.z, self._axle_to_base_link_m
            )
            twist.linear.x = base_vx
            twist.linear.y = axle_vy + offset_vy
            message.twist.covariance = shift_twist_covariance_x(
                list(message.twist.covariance), self._axle_to_base_link_m
            )
        self._wheel_publisher.publish(message)

    def _normalize_imu(self, message: Imu) -> None:
        self._ensure_stamp(message.header)
        if not message.header.frame_id:
            message.header.frame_id = self._imu_frame

        quaternion = message.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm <= 1.0e-12 or not math.isfinite(norm):
            quaternion.x = quaternion.y = quaternion.z = 0.0
            quaternion.w = 1.0
            message.orientation_covariance = [-1.0] + [0.0] * 8
        else:
            quaternion.x /= norm
            quaternion.y /= norm
            quaternion.z /= norm
            quaternion.w /= norm
            if _all_zero(message.orientation_covariance):
                message.orientation_covariance = _imu_covariance(
                    self._imu_orientation_covariance
                )

        if _all_zero(message.angular_velocity_covariance):
            message.angular_velocity_covariance = _imu_covariance(
                self._imu_angular_velocity_covariance
            )
        if _all_zero(message.linear_acceleration_covariance):
            message.linear_acceleration_covariance = _imu_covariance(
                self._imu_linear_acceleration_covariance
            )
        self._imu_publisher.publish(message)

    def _normalize_camera_info(self, message: CameraInfo) -> None:
        """Fill required image dimensions while preserving real calibration data.

        camera_ros publishes an otherwise valid uncalibrated CameraInfo message
        with width and height set to zero when no calibration file is installed.
        Foxglove needs those dimensions and non-zero focal lengths to display
        the image. Nominal pinhole intrinsics are inserted only for generic
        visualization; metric vision must use an actual fisheye calibration.
        """
        self._ensure_stamp(message.header)
        if not message.header.frame_id:
            message.header.frame_id = self._camera_frame
        if message.width == 0:
            message.width = self._camera_width
        if message.height == 0:
            message.height = self._camera_height
        if message.k[0] == 0.0 or message.k[4] == 0.0:
            message.distortion_model = "plumb_bob"
            message.d = [0.0] * 5
            message.k = [
                self._camera_nominal_fx,
                0.0,
                self._camera_nominal_cx,
                0.0,
                self._camera_nominal_fy,
                self._camera_nominal_cy,
                0.0,
                0.0,
                1.0,
            ]
            message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            message.p = [
                self._camera_nominal_fx,
                0.0,
                self._camera_nominal_cx,
                0.0,
                0.0,
                self._camera_nominal_fy,
                self._camera_nominal_cy,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
        self._camera_info_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MeasurementNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
