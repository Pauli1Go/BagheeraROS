"""Gate autonomous velocity commands and leave the charging dock first."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import Power
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class AutonomyDockGuard(Node):
    """Run Bagheera's fixed undock manoeuvre before autonomous movement.

    Teleoperation uses a separate twist_mux lane and is intentionally not
    routed through this node.
    """

    def __init__(self) -> None:
        super().__init__("bagheera_autonomy_dock_guard")
        self.declare_parameter("input_topic", "/cmd_vel_automatic_raw")
        self.declare_parameter("output_topic", "/cmd_vel_monitored")
        self.declare_parameter("power_topic", "/hardware_bridge/power")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("dock_voltage_threshold", 10.0)
        self.declare_parameter("dock_debounce_s", 1.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("reverse_distance_m", 0.80)
        self.declare_parameter("reverse_speed_mps", 0.08)
        self.declare_parameter("reverse_timeout_s", 18.0)
        self.declare_parameter("reverse_heading_kp", 1.5)
        self.declare_parameter("reverse_cross_track_kp", 1.0)
        self.declare_parameter("reverse_max_angular_rps", 0.25)
        self.declare_parameter("turn_angle_rad", math.pi / 2.0)
        self.declare_parameter("turn_max_speed_rps", 0.30)
        self.declare_parameter("turn_min_speed_rps", 0.16)
        self.declare_parameter("turn_gain", 0.8)
        self.declare_parameter("turn_tolerance_rad", math.radians(2.0))
        self.declare_parameter("turn_timeout_s", 10.0)
        self.declare_parameter("settle_time_s", 0.30)

        self._dock_voltage = float(
            self.get_parameter("dock_voltage_threshold").value
        )
        self._dock_debounce = float(self.get_parameter("dock_debounce_s").value)
        self._command_timeout = float(
            self.get_parameter("command_timeout_s").value
        )
        self._reverse_distance = float(
            self.get_parameter("reverse_distance_m").value
        )
        self._reverse_speed = float(self.get_parameter("reverse_speed_mps").value)
        self._reverse_timeout = float(
            self.get_parameter("reverse_timeout_s").value
        )
        self._reverse_heading_kp = float(
            self.get_parameter("reverse_heading_kp").value
        )
        self._reverse_cross_track_kp = float(
            self.get_parameter("reverse_cross_track_kp").value
        )
        self._reverse_max_angular = float(
            self.get_parameter("reverse_max_angular_rps").value
        )
        self._turn_angle = float(self.get_parameter("turn_angle_rad").value)
        self._turn_max_speed = float(
            self.get_parameter("turn_max_speed_rps").value
        )
        self._turn_min_speed = float(
            self.get_parameter("turn_min_speed_rps").value
        )
        self._turn_gain = float(self.get_parameter("turn_gain").value)
        self._turn_tolerance = float(
            self.get_parameter("turn_tolerance_rad").value
        )
        self._turn_timeout = float(self.get_parameter("turn_timeout_s").value)
        self._settle_time = float(self.get_parameter("settle_time_s").value)

        self._state = "WAITING_FOR_POWER"
        self._dock_candidate: bool | None = None
        self._dock_candidate_since = time.monotonic()
        self._docked: bool | None = None
        self._last_command: TwistStamped | None = None
        self._last_command_time = 0.0
        self._odom: Odometry | None = None
        self._odom_time = 0.0
        self._phase_started = 0.0
        self._start_x = 0.0
        self._start_y = 0.0
        self._start_yaw = 0.0
        self._reverse_path = 0.0
        self._reverse_last_x = 0.0
        self._reverse_last_y = 0.0
        self._last_yaw = 0.0
        self._turn_progress = 0.0
        self._localization_violation = False

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        power_topic = str(self.get_parameter("power_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        dock_qos = QoSProfile(depth=1)
        dock_qos.reliability = ReliabilityPolicy.RELIABLE
        dock_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._dock_publisher = self.create_publisher(Bool, "/docked", dock_qos)
        self.create_subscription(TwistStamped, input_topic, self._on_command, 10)
        self.create_subscription(Power, power_topic, self._on_power, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(
            Bool,
            "/localization_exclusion_violation",
            self._on_localization_violation,
            10,
        )
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_docked)
        self.get_logger().info(
            "Autonomous dock guard active: dock voltage >= %.1f V; "
            "undock %.2f m reverse, then %.1f deg left"
            % (
                self._dock_voltage,
                self._reverse_distance,
                math.degrees(self._turn_angle),
            )
        )

    def _on_command(self, message: TwistStamped) -> None:
        self._last_command = message
        self._last_command_time = time.monotonic()

    def _on_power(self, message: Power) -> None:
        # Mowgli's established electrical dock test is v_charge >= 10 V.
        # Unlike charge current, this also remains valid when the battery is
        # full and the charger has tapered its current.
        candidate = math.isfinite(message.v_charge) and (
            message.v_charge >= self._dock_voltage
        )
        now = time.monotonic()
        if candidate != self._dock_candidate:
            self._dock_candidate = candidate
            self._dock_candidate_since = now
            return
        if now - self._dock_candidate_since < self._dock_debounce:
            return
        if candidate == self._docked:
            return
        self._docked = candidate
        self._publish_docked()
        if candidate:
            self._state = "DOCKED_IDLE"
            self.get_logger().info("Dock connection detected; autonomy is gated")
        elif self._state in ("WAITING_FOR_POWER", "DOCKED_IDLE"):
            self._state = "CLEAR"
            self.get_logger().info("No dock connection; autonomy is released")

    def _on_odom(self, message: Odometry) -> None:
        if self._state == "REVERSING":
            position = message.pose.pose.position
            self._reverse_path += math.hypot(
                position.x - self._reverse_last_x,
                position.y - self._reverse_last_y,
            )
            self._reverse_last_x = position.x
            self._reverse_last_y = position.y
        self._odom = message
        self._odom_time = time.monotonic()

    def _on_localization_violation(self, message: Bool) -> None:
        self._localization_violation = message.data

    def _publish_docked(self) -> None:
        if self._docked is not None:
            self._dock_publisher.publish(Bool(data=self._docked))

    def _command_is_active(self, now: float) -> bool:
        if (
            self._last_command is None
            or now - self._last_command_time > self._command_timeout
        ):
            return False
        twist = self._last_command.twist
        return (
            abs(twist.linear.x) > 1.0e-3
            or abs(twist.linear.y) > 1.0e-3
            or abs(twist.angular.z) > 1.0e-3
        )

    def _odom_is_fresh(self, now: float) -> bool:
        return self._odom is not None and now - self._odom_time <= 0.5

    def _publish(self, linear: float = 0.0, angular: float = 0.0) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self._publisher.publish(message)

    def _start_reverse(self, now: float) -> None:
        assert self._odom is not None
        pose = self._odom.pose.pose
        self._start_x = pose.position.x
        self._start_y = pose.position.y
        self._start_yaw = _yaw(self._odom)
        self._reverse_path = 0.0
        self._reverse_last_x = pose.position.x
        self._reverse_last_y = pose.position.y
        self._phase_started = now
        self._state = "REVERSING"
        self.get_logger().info(
            "Autonomous movement requested while docked: reversing %.2f m"
            % self._reverse_distance
        )

    def _reverse_progress(self) -> float:
        return self._reverse_path

    def _reverse_errors(self) -> tuple[float, float]:
        """Return heading and lateral error relative to the starting line."""
        assert self._odom is not None
        pose = self._odom.pose.pose
        dx = pose.position.x - self._start_x
        dy = pose.position.y - self._start_y
        heading_error = _normalize_angle(self._start_yaw - _yaw(self._odom))
        cross_track = -math.sin(self._start_yaw) * dx + math.cos(
            self._start_yaw
        ) * dy
        return heading_error, cross_track

    def _start_turn(self, now: float) -> None:
        assert self._odom is not None
        self._last_yaw = _yaw(self._odom)
        self._turn_progress = 0.0
        self._phase_started = now
        self._state = "TURNING"
        self.get_logger().info(
            "Reverse complete; turning %.1f deg left"
            % math.degrees(self._turn_angle)
        )

    def _update_turn_progress(self) -> None:
        assert self._odom is not None
        current_yaw = _yaw(self._odom)
        self._turn_progress += _normalize_angle(current_yaw - self._last_yaw)
        self._last_yaw = current_yaw

    def _fail(self, reason: str) -> None:
        self._state = "FAULT"
        self._publish()
        self.get_logger().error(f"Undocking aborted: {reason}; autonomy remains gated")

    def _tick(self) -> None:
        now = time.monotonic()
        if self._localization_violation:
            self._publish()
            return
        if self._state in ("WAITING_FOR_POWER", "FAULT"):
            self._publish()
            return

        if self._state == "CLEAR":
            if self._last_command is not None and (
                now - self._last_command_time <= self._command_timeout
            ):
                self._last_command.header.stamp = self.get_clock().now().to_msg()
                self._publisher.publish(self._last_command)
            else:
                self._publish()
            return

        if self._state == "DOCKED_IDLE":
            self._publish()
            if not self._command_is_active(now):
                return
            if not self._odom_is_fresh(now):
                self.get_logger().warn(
                    "Autonomous command is waiting for fresh odometry",
                    throttle_duration_sec=2.0,
                )
                return
            self._start_reverse(now)
            return

        if not self._odom_is_fresh(now):
            self._fail("odometry became stale")
            return

        if self._state == "REVERSING":
            if now - self._phase_started > self._reverse_timeout:
                self._fail("reverse timeout")
                return
            if self._reverse_progress() >= self._reverse_distance:
                heading_error, cross_track = self._reverse_errors()
                self.get_logger().info(
                    "Reverse result: path %.3f m, lateral %+.3f m, yaw drift %+.1f deg"
                    % (
                        self._reverse_progress(),
                        cross_track,
                        -math.degrees(heading_error),
                    )
                )
                self._state = "SETTLE_AFTER_REVERSE"
                self._phase_started = now
                self._publish()
                return
            heading_error, cross_track = self._reverse_errors()
            angular = (
                self._reverse_heading_kp * heading_error
                + self._reverse_cross_track_kp * cross_track
            )
            angular = max(
                -self._reverse_max_angular,
                min(self._reverse_max_angular, angular),
            )
            self._publish(linear=-self._reverse_speed, angular=angular)
            return

        if self._state == "SETTLE_AFTER_REVERSE":
            self._publish()
            if now - self._phase_started >= self._settle_time:
                self._start_turn(now)
            return

        if self._state == "TURNING":
            self._update_turn_progress()
            remaining = self._turn_angle - self._turn_progress
            if remaining <= self._turn_tolerance:
                self._state = "SETTLE_AFTER_TURN"
                self._phase_started = now
                self._publish()
                self.get_logger().info(
                    "Undock turn complete at %.1f deg"
                    % math.degrees(self._turn_progress)
                )
                return
            if now - self._phase_started > self._turn_timeout:
                self._fail("left-turn timeout")
                return
            speed = min(
                self._turn_max_speed,
                max(self._turn_min_speed, self._turn_gain * remaining),
            )
            self._publish(angular=speed)
            return

        if self._state == "SETTLE_AFTER_TURN":
            self._publish()
            if now - self._phase_started < self._settle_time:
                return
            if self._docked:
                self._fail("dock voltage is still present after the manoeuvre")
                return
            self._state = "CLEAR"
            self.get_logger().info("Undocking complete; autonomous movement released")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomyDockGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
