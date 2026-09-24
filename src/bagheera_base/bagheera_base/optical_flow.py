"""ROS 2 publisher for the downward-facing Bagheera PMW3901."""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

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
        self.declare_parameter("lever_arm_x", 0.0)
        self.declare_parameter("lever_arm_y", 0.0)
        self.declare_parameter("imu_topic", "/imu/wt901/data_raw")
        self.declare_parameter("imu_max_age", 0.5)
        # The PMW3901 reports phantom translation while the robot pivots (the
        # ground image swirls, its average is not zero in practice). Such flow
        # readings get published with an inflated covariance so the EKF's
        # Mahalanobis gate drops them instead of integrating the phantom.
        self.declare_parameter("rotation_gate_rad_s", 0.25)
        self.declare_parameter("rotation_variance", 1000000.0)
        self.declare_parameter("rotation_gate_enabled", True)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._meters_per_count = float(self.get_parameter("mount_height_m").value) * float(
            self.get_parameter("radians_per_count").value
        )
        self._swap_xy = bool(self.get_parameter("swap_xy").value)
        self._invert_x = bool(self.get_parameter("invert_x").value)
        self._invert_y = bool(self.get_parameter("invert_y").value)
        self._minimum_quality = int(self.get_parameter("minimum_quality").value)
        # Lever arm of the sensor relative to the base_link rotation center.
        # While rotating, the sensor sweeps with wz x r, which would otherwise
        # be fused as a phantom base velocity (vy) and smear the odometry.
        self._lever_x = float(self.get_parameter("lever_arm_x").value)
        self._lever_y = float(self.get_parameter("lever_arm_y").value)
        self._rotation_gate = float(self.get_parameter("rotation_gate_rad_s").value)
        self._rotation_gate_enabled = bool(
            self.get_parameter("rotation_gate_enabled").value
        )
        self._rotation_variance = float(
            self.get_parameter("rotation_variance").value
        )
        imu_max_age = float(self.get_parameter("imu_max_age").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        # Recent gyro samples for integrating wz over the flow window.
        self._wz_history: deque[tuple[int, float]] = deque(maxlen=256)
        self._imu_max_age_s = imu_max_age
        self._imu_max_age_ns = int(imu_max_age * 1_000_000_000)
        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST
        self._imu_subscription = self.create_subscription(
            Imu, imu_topic, self._on_imu, sensor_qos
        )

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
        self._sleeping = False
        sleep_qos = QoSProfile(depth=1)
        sleep_qos.reliability = ReliabilityPolicy.RELIABLE
        sleep_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/dock/sleep_state", self._on_sleep_state, sleep_qos)
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

    def _on_sleep_state(self, message: String) -> None:
        sleeping = message.data == "sleeping"
        if sleeping == self._sleeping:
            return
        if sleeping:
            self._timer.cancel()
            try:
                self._sensor.shutdown()
            except OSError as exc:
                self.get_logger().error(f"PMW3901 shutdown failed: {exc}")
            self._sleeping = True
            self.get_logger().info("PMW3901 asleep (LED off) while docked")
            return
        try:
            self._sensor.power_up()
            # Drop whatever the first burst after the reset reports.
            self._sensor.read_motion()
        except (OSError, RuntimeError) as exc:
            # Stay asleep; the next state message retries.
            self.get_logger().error(f"PMW3901 wake-up failed: {exc}")
            return
        self._sleeping = False
        self._last_stamp_ns = self.get_clock().now().nanoseconds
        self._timer.reset()
        self.get_logger().info("PMW3901 awake")

    def _on_imu(self, message: Imu) -> None:
        wz = message.angular_velocity.z
        if math.isfinite(wz):
            self._wz_history.append((self.get_clock().now().nanoseconds, wz))

    def _mean_wz_between(self, start_ns: int, end_ns: int) -> float | None:
        """Average gyro wz over the flow integration window."""
        samples = [wz for stamp, wz in self._wz_history
                   if start_ns <= stamp <= end_ns]
        if not samples:
            return None
        return sum(samples) / len(samples)

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

        stamp = now.to_msg()
        if self._raw_publisher.get_subscription_count() > 0:
            raw = Vector3Stamped()
            raw.header.stamp = stamp
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
        twist.header.stamp = stamp
        twist.header.frame_id = self._frame_id
        vx = robot_x * self._meters_per_count / elapsed
        vy = robot_y * self._meters_per_count / elapsed

        # Remove the rotation-induced sweep velocity so the twist describes
        # the base_link motion, not the sensor point motion.
        wz = None
        if self._lever_x != 0.0 or self._lever_y != 0.0:
            wz = self._mean_wz_between(now_ns - int(elapsed * 1_000_000_000), now_ns)
            if wz is None and self._wz_history and \
                    now_ns - self._wz_history[-1][0] <= self._imu_max_age_ns:
                wz = self._wz_history[-1][1]
            if wz is None:
                # Normal for 4 s while the WT901 measures its bias.
                self.get_logger().warning(
                    "No gyro data within %.1f s; publishing uncorrected flow"
                    % self._imu_max_age_s,
                    throttle_duration_sec=5.0,
                )
            else:
                # v_flow = v_base + wz x lever_arm  =>  v_base = v_flow - wz x r
                vx += wz * self._lever_y
                vy -= wz * self._lever_x

        variance = max(0.0025, 0.25 / max(1, quality))
        # This optional gate is only correct when base_link is at the axle and
        # the flow sensor is offset from it. Bagheera's base_link is at the
        # sensor, so its lateral motion during a pivot is real and the gate is
        # disabled in sensors.yaml.
        if wz is None:
            wz = self._mean_wz_between(
                now_ns - int(0.3 * 1_000_000_000), now_ns
            )
        if (
            self._rotation_gate_enabled
            and wz is not None
            and abs(wz) > self._rotation_gate
        ):
            variance = self._rotation_variance

        twist.twist.twist.linear.x = vx
        twist.twist.twist.linear.y = vy
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
