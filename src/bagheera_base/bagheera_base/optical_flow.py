"""ROS 2 publisher for the downward-facing Bagheera PMW3901."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from rclpy.node import Node

from .pmw3901 import Pmw3901, transform_counts


class OpticalFlowNode(Node):
    """Poll PMW3901 motion bursts and publish raw and metric flow."""

    def __init__(self) -> None:
        super().__init__("bagheera_optical_flow")
        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_chip_select", 0)
        self.declare_parameter("spi_speed_hz", 400_000)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("mount_height_m", 0.09)
        self.declare_parameter("radians_per_count", 0.002057)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("minimum_quality", 0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._meters_per_count = float(self.get_parameter("mount_height_m").value) * float(
            self.get_parameter("radians_per_count").value
        )
        self._swap_xy = bool(self.get_parameter("swap_xy").value)
        self._invert_x = bool(self.get_parameter("invert_x").value)
        self._invert_y = bool(self.get_parameter("invert_y").value)
        self._minimum_quality = int(self.get_parameter("minimum_quality").value)

        self._sensor = Pmw3901(
            bus=int(self.get_parameter("spi_bus").value),
            chip_select=int(self.get_parameter("spi_chip_select").value),
            speed_hz=int(self.get_parameter("spi_speed_hz").value),
        )
        product, revision = self._sensor.identity()
        self._raw_publisher = self.create_publisher(Vector3Stamped, "/optical_flow/raw", 20)
        self._twist_publisher = self.create_publisher(
            TwistWithCovarianceStamped, "/optical_flow/twist", 20
        )
        self._last_stamp_ns = self.get_clock().now().nanoseconds
        rate = float(self.get_parameter("publish_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._poll)
        self.get_logger().info(
            "PMW3901 ready on SPI%d.%d (id=0x%02x revision=0x%02x, %.6f m/count)"
            % (
                int(self.get_parameter("spi_bus").value),
                int(self.get_parameter("spi_chip_select").value),
                product,
                revision,
                self._meters_per_count,
            )
        )

    def destroy_node(self):
        self._sensor.close()
        return super().destroy_node()

    def _poll(self) -> None:
        try:
            motion = self._sensor.read_motion()
        except OSError as exc:
            self.get_logger().error(f"PMW3901 SPI read failed: {exc}")
            return
        sensor_x, sensor_y, quality = motion
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        elapsed = (now_ns - self._last_stamp_ns) / 1_000_000_000.0
        self._last_stamp_ns = now_ns

        raw = Vector3Stamped()
        raw.header.stamp = now.to_msg()
        raw.header.frame_id = "pmw3901_sensor"
        raw.vector.x = float(sensor_x)
        raw.vector.y = float(sensor_y)
        raw.vector.z = float(quality)
        self._raw_publisher.publish(raw)

        if quality < self._minimum_quality or elapsed <= 0.0 or not math.isfinite(elapsed):
            return
        robot_x, robot_y = transform_counts(
            sensor_x,
            sensor_y,
            swap_xy=self._swap_xy,
            invert_x=self._invert_x,
            invert_y=self._invert_y,
        )
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = raw.header.stamp
        twist.header.frame_id = self._frame_id
        twist.twist.twist.linear.x = robot_x * self._meters_per_count / elapsed
        twist.twist.twist.linear.y = robot_y * self._meters_per_count / elapsed
        variance = max(0.0025, 0.25 / max(1, quality))
        twist.twist.covariance[0] = variance
        twist.twist.covariance[7] = variance
        twist.twist.covariance[14] = 1_000_000.0
        twist.twist.covariance[21] = 1_000_000.0
        twist.twist.covariance[28] = 1_000_000.0
        twist.twist.covariance[35] = 1_000_000.0
        self._twist_publisher.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
