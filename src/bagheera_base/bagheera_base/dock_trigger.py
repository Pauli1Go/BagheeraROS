"""Bridge Bagheera's Foxglove dock topic to Nav2's DockRobot action.

The TagChargingDock plugin hands over at a pre-dock pose on the dock axis,
where Nav2's curve is finished. During the server's wait-for-charge phase this
node drives the last centimetres straight onto the charging pins with gyro
heading hold, and stops at contact voltage or a small overtravel.

Undocking is not offered here: bagheera_autonomy_dock_guard already reverses
out of the dock before any autonomous movement.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from mowgli_interfaces.msg import Power
from nav2_msgs.action import DockRobot
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


FEEDBACK_STATES = {
    DockRobot.Feedback.NONE: "STARTING",
    DockRobot.Feedback.NAV_TO_STAGING_POSE: "NAV_TO_STAGING_POSE",
    DockRobot.Feedback.INITIAL_PERCEPTION: "INITIAL_PERCEPTION",
    DockRobot.Feedback.CONTROLLING: "CONTROLLING",
    DockRobot.Feedback.WAIT_FOR_CHARGE: "WAIT_FOR_CHARGE",
    DockRobot.Feedback.RETRY: "RETRY",
}
# The camera is only needed once the robot is at the staging pose; keeping it
# off while Nav2 drives there saves the CPU that localization needs.
CAMERA_STATES = {"INITIAL_PERCEPTION", "CONTROLLING", "WAIT_FOR_CHARGE", "RETRY"}


@dataclass(frozen=True)
class FinalApproach:
    speed: float = 0.05
    overtravel: float = 0.03
    heading_gain: float = 1.2
    trim_max: float = math.radians(4.0)
    trim_distance: float = 0.25
    max_angular: float = 0.20
    front_offset: float = 0.47
    # Hand-over limits: the straight drive can turn the heading back, but it
    # cannot remove an axle offset without a curve near the pins.
    max_lateral: float = 0.025
    max_yaw: float = math.radians(10.0)


def final_approach_command(
    along: float, left: float, yaw_error: float, params: FinalApproach
) -> tuple[float, float, str | None]:
    """Straight final approach in the dock frame.

    along/left: base_link relative to the contact pose (along < 0 in front of
    it), yaw_error: robot heading minus dock axis. Returns (linear, angular,
    stop reason). A small bounded heading trim removes the remaining lateral
    offset without a curve near the pins.
    """
    if along >= params.overtravel:
        return 0.0, 0.0, f"{along:.3f} m past the contact pose without voltage"
    desired = max(-params.trim_max, min(params.trim_max, -math.atan2(left, params.trim_distance)))
    angular = params.heading_gain * math.atan2(
        math.sin(desired - yaw_error), math.cos(desired - yaw_error)
    )
    angular = max(-params.max_angular, min(params.max_angular, angular))
    return params.speed, angular, None


def front_offset(left: float, yaw_error: float, params: FinalApproach) -> float:
    """Lateral offset of the charging contacts ahead of the axle."""
    return left + params.front_offset * math.sin(yaw_error)


class DockTrigger(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_dock_trigger")
        self.declare_parameter("dock_id", "home_dock")
        self.declare_parameter("max_staging_time_s", 120.0)
        self.declare_parameter("final_speed_mps", 0.05)
        self.declare_parameter("final_overtravel_m", 0.03)
        self.declare_parameter("final_heading_gain", 1.2)
        self.declare_parameter("final_trim_max_rad", math.radians(4.0))
        self.declare_parameter("final_trim_distance_m", 0.25)
        self.declare_parameter("final_max_angular_rps", 0.20)
        self.declare_parameter("final_front_offset_m", 0.47)
        self.declare_parameter("final_max_lateral_m", 0.025)
        self.declare_parameter("final_max_yaw_rad", math.radians(10.0))
        self.declare_parameter("final_timeout_s", 8.0)
        self.declare_parameter("contact_voltage", 0.5)
        self._dock_id = str(self.get_parameter("dock_id").value)
        self._max_staging_time = float(self.get_parameter("max_staging_time_s").value)
        self._final_params = FinalApproach(
            speed=float(self.get_parameter("final_speed_mps").value),
            overtravel=float(self.get_parameter("final_overtravel_m").value),
            heading_gain=float(self.get_parameter("final_heading_gain").value),
            trim_max=float(self.get_parameter("final_trim_max_rad").value),
            trim_distance=float(self.get_parameter("final_trim_distance_m").value),
            max_angular=float(self.get_parameter("final_max_angular_rps").value),
            front_offset=float(self.get_parameter("final_front_offset_m").value),
            max_lateral=float(self.get_parameter("final_max_lateral_m").value),
            max_yaw=float(self.get_parameter("final_max_yaw_rad").value),
        )
        self._final_timeout = float(self.get_parameter("final_timeout_s").value)
        self._contact_voltage = float(self.get_parameter("contact_voltage").value)
        # None: not started in this wait-for-charge phase; "driving"; "done".
        self._final_state: str | None = None
        self._final_started = 0.0
        self._dock_pose: PoseStamped | None = None
        self._charge_voltage = 0.0
        self._last_event = ""
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        self._state = "IDLE"
        self._detail = "ready"
        self._retries = 0
        self._elapsed_s = 0
        self._docked = False
        self._goal_handle = None
        self._goal_pending = False
        self._action = ""
        self._camera_enabled: bool | None = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(String, "/dock/status", latched)
        self._active_publisher = self.create_publisher(Bool, "/dock/active", latched)
        self._vision_publisher = self.create_publisher(Bool, "/dock/vision_enabled", latched)
        # opennav_docking publishes nothing while it waits for charge; this
        # node owns the docking lane then (final approach, then zero).
        self._docking_velocity_publisher = self.create_publisher(
            TwistStamped, "/cmd_vel_docking", 10
        )
        self._dock_client = ActionClient(self, DockRobot, "/dock_robot")

        self.create_subscription(Bool, "/dock/trigger", self._on_dock, 10)
        self.create_subscription(Bool, "/dock/cancel", self._on_cancel, 10)
        self.create_subscription(Bool, "/docked", self._on_docked, latched)
        self.create_subscription(TwistStamped, "/cmd_vel_teleop", self._on_teleop, 10)
        self.create_subscription(PoseStamped, "/dock_pose", self._on_dock_pose, 5)
        self.create_subscription(Power, "/hardware_bridge/power", self._on_power, 10)
        self.create_subscription(String, "/dock/plugin_event", self._on_plugin_event, 10)
        self.create_timer(0.05, self._final_tick)
        self.create_timer(1.0, self._publish_status)
        self._set_camera(False)
        self._publish_status()

    def _active(self) -> bool:
        return self._goal_pending or self._goal_handle is not None

    def _set_state(self, state: str, detail: str) -> None:
        if state != self._state or detail != self._detail:
            self.get_logger().info(f"Docking {state}: {detail}")
        self._state = state
        self._detail = detail
        self._publish_status()

    def _publish_status(self) -> None:
        payload = {
            "state": self._state,
            "detail": self._detail,
            "active": self._active(),
            "action": self._action,
            "docked": self._docked,
            "retry": self._retries,
            "elapsed_s": self._elapsed_s,
        }
        self._status_publisher.publish(String(data=json.dumps(payload)))
        self._active_publisher.publish(Bool(data=self._active()))

    def _set_camera(self, enabled: bool) -> None:
        if self._camera_enabled != enabled:
            self._camera_enabled = enabled
            self._vision_publisher.publish(Bool(data=enabled))

    def _on_docked(self, message: Bool) -> None:
        self._docked = bool(message.data)

    def _on_plugin_event(self, message: String) -> None:
        self._last_event = message.data
        if message.data.startswith("PRE_DOCK_OFF_AXIS"):
            self._set_state("PRE_DOCK_OFF_AXIS", message.data + "; retrying from staging")

    def _on_dock_pose(self, message: PoseStamped) -> None:
        self._dock_pose = message

    def _on_power(self, message: Power) -> None:
        self._charge_voltage = float(message.v_charge) if math.isfinite(message.v_charge) else 0.0

    def _publish_docking_velocity(self, linear: float = 0.0, angular: float = 0.0) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        command.twist.linear.x = float(linear)
        command.twist.angular.z = float(angular)
        self._docking_velocity_publisher.publish(command)

    def _robot_in_dock_frame(self) -> tuple[float, float, float] | None:
        dock = self._dock_pose
        if dock is None:
            return None
        try:
            transform = self._tf.lookup_transform(dock.header.frame_id, "base_link", Time())
        except Exception:  # noqa: BLE001 - TF raises several exception types
            return None
        t = transform.transform
        q = dock.pose.orientation
        dock_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        r = t.rotation
        robot_yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        dx = t.translation.x - dock.pose.position.x
        dy = t.translation.y - dock.pose.position.y
        along = math.cos(dock_yaw) * dx + math.sin(dock_yaw) * dy
        left = -math.sin(dock_yaw) * dx + math.cos(dock_yaw) * dy
        yaw_error = math.atan2(math.sin(robot_yaw - dock_yaw), math.cos(robot_yaw - dock_yaw))
        return along, left, yaw_error

    def _start_final_approach(self) -> None:
        self._final_started = time.monotonic()
        pose = self._robot_in_dock_frame()
        if pose is None:
            self._final_state = "done"
            self.get_logger().error("Final approach skipped: no dock pose or TF")
            return
        along, left, yaw_error = pose
        if (
            abs(left) > self._final_params.max_lateral
            or abs(yaw_error) > self._final_params.max_yaw
        ):
            self._final_state = "done"
            detail = (
                "final approach skipped at %.3f m before contact: axle left %+.3f m "
                "(max %.3f), yaw %+.1f deg (max %.0f); Nav2 retries after the charge timeout"
                % (-along, left, self._final_params.max_lateral, math.degrees(yaw_error),
                   math.degrees(self._final_params.max_yaw))
            )
            self.get_logger().warn(detail)
            self._set_state("PRE_DOCK_OFF_AXIS", detail)
            return
        self._final_state = "driving"
        detail = "straight from %.3f m: left %+.3f m, yaw %+.1f deg" % (
            -along, left, math.degrees(yaw_error))
        self.get_logger().info(f"Straight final approach {detail}")
        self._set_state("FINAL_APPROACH", detail)

    def _final_tick(self) -> None:
        if self._final_state == "done" and self._state in ("WAIT_FOR_CHARGE", "PRE_DOCK_OFF_AXIS"):
            self._publish_docking_velocity()
            return
        if self._final_state != "driving":
            return
        if self._charge_voltage >= self._contact_voltage:
            self._stop_final("contact voltage %.1f V" % self._charge_voltage)
            return
        if time.monotonic() - self._final_started > self._final_timeout:
            self._stop_final("final approach timeout")
            return
        pose = self._robot_in_dock_frame()
        if pose is None:
            self._stop_final("robot or dock pose unavailable")
            return
        linear, angular, reason = final_approach_command(*pose, self._final_params)
        if reason is not None:
            self._stop_final(reason)
            return
        self._publish_docking_velocity(linear, angular)

    def _stop_final(self, reason: str) -> None:
        self._final_state = "done"
        self._publish_docking_velocity()
        pose = self._robot_in_dock_frame()
        where = ""
        if pose is not None:
            along, left, yaw_error = pose
            where = " at %+.3f m, contacts %+.3f m off axis, yaw %+.1f deg" % (
                along, front_offset(left, yaw_error, self._final_params),
                math.degrees(yaw_error))
        self.get_logger().info(f"Final approach stopped: {reason}{where}")
        if self._state in ("FINAL_APPROACH", "WAIT_FOR_CHARGE"):
            self._set_state("WAIT_FOR_CHARGE", f"{reason}{where}; waiting for /docked")

    def _on_dock(self, message: Bool) -> None:
        if not message.data:
            return
        if self._active():
            self.get_logger().warn("Dock trigger ignored: an action is active")
            return
        if not self._dock_client.server_is_ready():
            self._set_state("FAILED", "Nav2 docking server unavailable")
            return
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self._dock_id
        goal.navigate_to_staging_pose = True
        goal.max_staging_time = self._max_staging_time
        self._send(self._dock_client, goal, "dock")

    def _send(self, client: ActionClient, goal, action: str) -> None:
        self._goal_pending = True
        self._action = action
        self._retries = 0
        self._elapsed_s = 0
        self._last_event = ""
        self._set_state("STARTING", f"{action} goal sent")
        future = client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self._goal_pending = False
        try:
            handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self._finish("FAILED", f"goal error: {error}")
            return
        if not handle.accepted:
            self._finish("FAILED", f"{self._action} goal rejected")
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)
        self._publish_status()

    def _on_feedback(self, message) -> None:
        feedback = message.feedback
        if self._action == "dock":
            state = FEEDBACK_STATES.get(feedback.state, f"STATE_{feedback.state}")
            self._retries = int(feedback.num_retries)
            self._elapsed_s = int(feedback.docking_time.sec)
            self._set_camera(state in CAMERA_STATES)
            if state == "WAIT_FOR_CHARGE":
                if self._final_state is None:
                    self._start_final_approach()
            elif self._final_state is not None:
                # A retry brings the robot back through the curve first.
                if self._final_state == "driving":
                    self._stop_final("docking server left the charge phase")
                self._final_state = None
            if state == "WAIT_FOR_CHARGE" and self._final_state is not None:
                return  # FINAL_APPROACH / WAIT_FOR_CHARGE are set by the final approach
            if state != self._state and self._state != "CANCELLING":
                detail = f"retry {self._retries}"
                if self._retries and self._last_event:
                    detail += f" after {self._last_event.split(' ')[0]}"
                self._set_state(state, detail)

    def _on_result(self, future) -> None:
        self._goal_handle = None
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self._finish("FAILED", f"result error: {error}")
            return
        result = response.result
        self._retries = int(getattr(result, "num_retries", self._retries))
        if response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self._finish("SUCCEEDED", f"{self._action} succeeded")
        elif response.status == GoalStatus.STATUS_CANCELED:
            self._finish("CANCELLED", f"{self._action} cancelled")
        else:
            message = getattr(result, "error_msg", "") or f"error code {result.error_code}"
            self._finish("FAILED", f"{self._action}: {message}")

    def _finish(self, state: str, detail: str) -> None:
        if self._final_state == "driving":
            self._stop_final(f"docking {state.lower()}")
        self._final_state = None
        self._goal_handle = None
        self._goal_pending = False
        self._set_camera(False)
        self._set_state(state, detail)

    def _cancel(self, reason: str) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._set_state("CANCELLING", reason)

    def _on_cancel(self, message: Bool) -> None:
        if message.data:
            self._cancel("cancel requested")

    def _on_teleop(self, message: TwistStamped) -> None:
        twist = message.twist
        if abs(twist.linear.x) > 1.0e-3 or abs(twist.angular.z) > 1.0e-3:
            self._cancel("manual controller override")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockTrigger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
