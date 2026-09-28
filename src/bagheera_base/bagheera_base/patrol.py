"""Drive a closed waypoint loop with battery- or lap-driven docking.

Triggers (std_msgs/Bool, publish ``{"data": true}``):

- ``/patrol/start_charge``: loop until the battery is LOW/CRITICAL, charge
  until FULL, continue.
- ``/patrol/start_dock_cycle``: dock after every lap, pause, continue.
- ``/patrol/cancel``: stop; the robot stays where it is.

Moving the game controller or sending a Foxglove goal also cancels the
patrol. State is published as JSON on ``/patrol/status``; every event is
appended to a CSV file under ``log_dir``.
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from datetime import datetime

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .battery_math import CRITICAL, FULL, NORMAL
from .patrol_logic import (
    CHARGE,
    DOCK,
    DOCK_AND_STOP,
    DOCK_CYCLE,
    NAVIGATE,
    PAUSE,
    WAIT_FOR_FULL,
    PatrolPlan,
    parse_waypoints,
)

IDLE = "IDLE"
WAKING = "WAKING"
NAVIGATING = "NAVIGATING"
RETRY_WAIT = "RETRY_WAIT"  # short pause before driving to the same waypoint again
ABANDONING = "ABANDONING"  # cancelling the Nav2 goal for a CRITICAL return
DOCKING = "DOCKING"
CHARGING = "CHARGING"
DOCK_PAUSE = "DOCK_PAUSE"
FAILED = "FAILED"
RUNNING_STATES = (
    WAKING, NAVIGATING, RETRY_WAIT, ABANDONING, DOCKING, CHARGING, DOCK_PAUSE
)

DOCK_TERMINAL = ("SUCCEEDED", "FAILED", "CANCELLED")


def _quaternion_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _command_is_active(message: TwistStamped) -> bool:
    twist = message.twist
    return abs(twist.linear.x) > 1.0e-3 or abs(twist.angular.z) > 1.0e-3


class Patrol(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_patrol")
        self.declare_parameter("waypoints", [0.0])
        self.declare_parameter("dock_pause_s", 60.0)
        self.declare_parameter("dock_attempts", 3)
        self.declare_parameter("max_consecutive_failures", 3)
        self.declare_parameter("max_transient_retries", 2)
        self.declare_parameter("retry_delay_s", 3.0)
        self.declare_parameter("waypoint_timeout_s", 600.0)
        self.declare_parameter("wake_timeout_s", 90.0)
        self.declare_parameter("dock_start_timeout_s", 5.0)
        self.declare_parameter("max_charge_s", 21600.0)
        self.declare_parameter("log_dir", "/bagheera_ws/test_logs")

        self._waypoints = parse_waypoints(
            list(self.get_parameter("waypoints").get_parameter_value().double_array_value)
        )
        self._dock_pause = float(self.get_parameter("dock_pause_s").value)
        self._dock_attempts = int(self.get_parameter("dock_attempts").value)
        self._max_failures = int(self.get_parameter("max_consecutive_failures").value)
        self._max_retries = int(self.get_parameter("max_transient_retries").value)
        self._retry_delay = float(self.get_parameter("retry_delay_s").value)
        self._waypoint_timeout = float(self.get_parameter("waypoint_timeout_s").value)
        self._wake_timeout = float(self.get_parameter("wake_timeout_s").value)
        self._dock_start_timeout = float(self.get_parameter("dock_start_timeout_s").value)
        self._max_charge = float(self.get_parameter("max_charge_s").value)
        self._log_dir = str(self.get_parameter("log_dir").value)

        self._state = IDLE
        self._detail = "ready"
        self._plan: PatrolPlan | None = None
        self._phase_started = time.monotonic()
        self._run_started = 0.0
        self._docks = 0
        self._dock_failures = 0
        self._dock_attempt = 0
        self._dock_seen_active = False
        self._stop_after_dock = False
        self._last_wake = 0.0
        self._goal_handle = None
        self._goal_token = 0
        self._log_path: str | None = None

        self._level = NORMAL
        self._voltage: float | None = None
        self._docked = False
        self._sleep_state = "awake"
        self._dock_status: dict = {}

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, "/patrol/status", latched)
        self._marker_pub = self.create_publisher(MarkerArray, "/patrol/waypoints", latched)
        self._dock_trigger_pub = self.create_publisher(Bool, "/dock/trigger", 10)
        self._dock_cancel_pub = self.create_publisher(Bool, "/dock/cancel", 10)
        self._wake_pub = self.create_publisher(Bool, "/dock/wake", 10)
        self._nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.create_subscription(Bool, "/patrol/start_charge", self._on_start_charge, 10)
        self.create_subscription(Bool, "/patrol/start_dock_cycle", self._on_start_cycle, 10)
        self.create_subscription(Bool, "/patrol/cancel", self._on_cancel, 10)
        self.create_subscription(String, "/battery/level", self._on_level, latched)
        self.create_subscription(Float32, "/battery/voltage", self._on_voltage, 10)
        self.create_subscription(Bool, "/docked", self._on_docked, latched)
        self.create_subscription(String, "/dock/sleep_state", self._on_sleep_state, latched)
        self.create_subscription(String, "/dock/status", self._on_dock_status, latched)
        self.create_subscription(TwistStamped, "/cmd_vel_teleop", self._on_teleop, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_foreign_goal, 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self._on_foreign_goal, 10
        )
        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_status)
        self._publish_markers()
        self._publish_status()
        self.get_logger().info(
            f"Patrol ready: {len(self._waypoints)} waypoints; start with "
            "/patrol/start_charge or /patrol/start_dock_cycle"
        )

    # ------------------------------------------------------------------ inputs

    def _on_level(self, message: String) -> None:
        self._level = message.data
        if self._level == CRITICAL and self._state == NAVIGATING:
            self._abandon_for_critical()
        elif self._level == CRITICAL and self._state == RETRY_WAIT:
            self._log("waypoint_abandoned", "battery CRITICAL")
            self._do(self._plan.critical_during_navigation())

    def _on_voltage(self, message: Float32) -> None:
        self._voltage = float(message.data)

    def _on_docked(self, message: Bool) -> None:
        docked = bool(message.data)
        if self._docked and not docked and self._state in (CHARGING, DOCK_PAUSE):
            self._fail("left the dock unexpectedly")
        self._docked = docked

    def _on_sleep_state(self, message: String) -> None:
        self._sleep_state = message.data
        if self._state == WAKING:
            self._tick()

    def _on_dock_status(self, message: String) -> None:
        try:
            self._dock_status = json.loads(message.data)
        except ValueError:
            return
        if self._state == DOCKING:
            self._check_docking()

    def _on_teleop(self, message: TwistStamped) -> None:
        if self._state in RUNNING_STATES and _command_is_active(message):
            self._stop("cancelled by manual controller override")

    def _on_foreign_goal(self, _message: PoseStamped) -> None:
        if self._state in RUNNING_STATES:
            self._stop("cancelled by a Foxglove navigation goal")

    def _on_start_charge(self, message: Bool) -> None:
        if message.data:
            self._start(CHARGE)

    def _on_start_cycle(self, message: Bool) -> None:
        if message.data:
            self._start(DOCK_CYCLE)

    def _on_cancel(self, message: Bool) -> None:
        if message.data and self._state in RUNNING_STATES:
            self._stop("cancelled via /patrol/cancel")

    # ----------------------------------------------------------- state changes

    def _start(self, mode: str) -> None:
        if self._state in RUNNING_STATES:
            self.get_logger().warn("Patrol already running; cancel it first")
            return
        self._plan = PatrolPlan(
            len(self._waypoints), mode, self._max_failures, self._max_retries
        )
        self._docks = 0
        self._dock_failures = 0
        self._stop_after_dock = False
        self._run_started = time.monotonic()
        self._open_log(mode)
        self._log("start", f"{len(self._waypoints)} waypoints, level {self._level}")
        self._do(self._plan.start(self._level, self._docked))

    def _do(self, step: str) -> None:
        plan = self._plan
        if step == NAVIGATE:
            self._leave_or_navigate()
        elif step == DOCK:
            self._start_docking()
        elif step == DOCK_AND_STOP:
            self._stop_after_dock = True
            self._log("stopping", f"{plan.consecutive_failures} waypoints in a row unreachable")
            self._start_docking()
        elif step == WAIT_FOR_FULL:
            self._set_state(CHARGING, "charging until the battery is FULL")
        elif step == PAUSE:
            self._set_state(DOCK_PAUSE, "pausing %.0f s in the dock" % self._dock_pause)

    def _leave_or_navigate(self) -> None:
        if self._docked or self._sleep_state != "awake":
            self._last_wake = 0.0
            self._set_state(WAKING, "waking up before leaving the dock")
            self._tick()
        else:
            self._send_goal()

    def _send_goal(self) -> None:
        plan = self._plan
        if not self._nav.server_is_ready():
            self._fail("Nav2 navigate_to_pose server unavailable")
            return
        x, y, yaw = self._waypoints[plan.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = _quaternion_z_w(yaw)
        self._goal_token += 1
        token = self._goal_token
        self._set_state(NAVIGATING, f"driving to waypoint {plan.index + 1}")
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, token))

    def _on_goal_response(self, future, token: int) -> None:
        if token != self._goal_token:
            return
        handle = future.result()
        if not handle.accepted:
            self._waypoint_failed("goal rejected by Nav2")
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, token))
        if self._state == ABANDONING:
            handle.cancel_goal_async()  # CRITICAL arrived before the acceptance

    def _on_result(self, future, token: int) -> None:
        if token != self._goal_token:
            return
        self._goal_handle = None
        response = future.result()
        if self._state == ABANDONING:
            self._log("waypoint_abandoned", "battery CRITICAL")
            self._do(self._plan.critical_during_navigation())
            return
        if self._state != NAVIGATING:
            return
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self._waypoint_finished(True, "")
            return
        result = response.result
        detail = getattr(result, "error_msg", "") or f"error code {result.error_code}"
        if response.status == GoalStatus.STATUS_CANCELED:
            self._stop(f"Nav2 goal cancelled externally ({detail})")
            return
        self._waypoint_failed(detail)

    def _waypoint_failed(self, detail: str) -> None:
        """Retry after late navigation data; otherwise skip the waypoint."""
        plan = self._plan
        if plan.retry_after(detail):
            number = plan.index + 1
            self.get_logger().warn(
                f"Retrying waypoint {number} in {self._retry_delay:.0f} s "
                f"({plan.retries}/{plan.max_transient_retries}): {detail}"
            )
            self._log(
                "waypoint_retry", detail, waypoint=number,
                duration=time.monotonic() - self._phase_started,
            )
            self._set_state(
                RETRY_WAIT,
                f"retrying waypoint {number} ({plan.retries}/{plan.max_transient_retries})",
            )
            return
        self._waypoint_finished(False, detail)

    def _waypoint_finished(self, reached: bool, detail: str) -> None:
        plan = self._plan
        number = plan.index + 1
        duration = time.monotonic() - self._phase_started
        laps_before = plan.laps
        step = plan.waypoint_done(reached, self._level)
        if reached:
            self._log("waypoint_reached", "", waypoint=number, duration=duration)
        else:
            self.get_logger().warn(f"Skipping waypoint {number}: {detail}")
            self._log("waypoint_skipped", detail, waypoint=number, duration=duration)
        if plan.laps != laps_before:
            self._log(
                "lap_complete",
                f"lap {plan.laps}: reached {plan.reached}, skipped {plan.skipped}",
            )
        if step == DOCK and self._level != NORMAL:
            self._log("battery_return", f"battery {self._level}")
        self._do(step)

    def _abandon_for_critical(self) -> None:
        self.get_logger().warn("Battery CRITICAL: abandoning the waypoint, docking now")
        self._set_state(ABANDONING, "battery CRITICAL: cancelling the waypoint")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        # A pending goal response cancels itself in _on_goal_response.

    def _start_docking(self) -> None:
        self._dock_attempt = 0
        if self._docked:
            self._docking_succeeded()
            return
        self._trigger_dock()

    def _trigger_dock(self) -> None:
        self._dock_attempt += 1
        self._dock_seen_active = False
        self._set_state(
            DOCKING, f"docking attempt {self._dock_attempt}/{self._dock_attempts}"
        )
        self._log("dock_start", f"attempt {self._dock_attempt}")
        self._dock_trigger_pub.publish(Bool(data=True))

    def _check_docking(self) -> None:
        status = self._dock_status
        if status.get("active"):
            self._dock_seen_active = True
            return
        if not self._dock_seen_active or status.get("state") not in DOCK_TERMINAL:
            return
        state = status.get("state")
        if state == "SUCCEEDED":
            self._docking_succeeded()
        elif state == "CANCELLED":
            self._stop("docking was cancelled")
        else:
            self._docking_failed(str(status.get("detail", "docking failed")))

    def _docking_failed(self, detail: str) -> None:
        self._dock_failures += 1
        self._log("dock_failed", detail, duration=time.monotonic() - self._phase_started)
        if self._dock_attempt < self._dock_attempts:
            self._trigger_dock()
        else:
            self._fail(f"docking failed {self._dock_attempts} times: {detail}")

    def _docking_succeeded(self) -> None:
        self._docks += 1
        self._log(
            "docked",
            f"attempt {self._dock_attempt}" if self._dock_attempt else "already docked",
            duration=time.monotonic() - self._phase_started,
        )
        if self._stop_after_dock:
            self._fail(
                f"stopped in the dock after {self._max_failures} unreachable waypoints"
            )
            return
        self._do(self._plan.docked())

    def _stop(self, reason: str) -> None:
        self._halt_motion()
        self._log("cancelled", reason)
        self._set_state(IDLE, reason)

    def _fail(self, reason: str) -> None:
        self._halt_motion()
        self.get_logger().error(f"Patrol failed: {reason}")
        self._log("failed", reason)
        self._set_state(FAILED, reason)

    def _halt_motion(self) -> None:
        self._goal_token += 1  # ignore late responses of the current goal
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._state == DOCKING:
            self._dock_cancel_pub.publish(Bool(data=True))

    # -------------------------------------------------------------------- tick

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self._phase_started
        if self._state == WAKING:
            if self._sleep_state == "awake":
                self._log("awake", "", duration=elapsed)
                self._send_goal()
            elif self._sleep_state == "fault":
                self._fail("dock wake-up reported a fault")
            elif elapsed > self._wake_timeout:
                self._fail("dock wake-up timed out")
            elif self._sleep_state == "sleeping" and now - self._last_wake > 5.0:
                self._last_wake = now
                self._wake_pub.publish(Bool(data=True))
        elif self._state == NAVIGATING and elapsed > self._waypoint_timeout:
            self.get_logger().warn("Waypoint timeout; cancelling the goal")
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._goal_token += 1
            self._goal_handle = None
            self._waypoint_finished(False, "waypoint timeout %.0f s" % self._waypoint_timeout)
        elif self._state == DOCKING:
            if not self._dock_seen_active and elapsed > self._dock_start_timeout:
                self._docking_failed("dock trigger was not accepted")
        elif self._state == CHARGING:
            if self._level == FULL:
                self._plan.charged()
                self._log("charged", "", duration=elapsed)
                self._leave_or_navigate()
            elif elapsed > self._max_charge:
                self._fail("battery not FULL after %.1f h" % (self._max_charge / 3600.0))
        elif self._state == RETRY_WAIT and elapsed >= self._retry_delay:
            self._leave_or_navigate()
        elif self._state == DOCK_PAUSE and elapsed >= self._dock_pause:
            self._log("pause_done", "", duration=elapsed)
            self._leave_or_navigate()

    # ------------------------------------------------------------------ output

    def _set_state(self, state: str, detail: str) -> None:
        if state != self._state or detail != self._detail:
            self.get_logger().info(f"Patrol {state}: {detail}")
        if state != self._state or state in (NAVIGATING, DOCKING):
            self._phase_started = time.monotonic()
        self._state = state
        self._detail = detail
        self._publish_status()

    def _publish_status(self) -> None:
        plan = self._plan
        payload = {
            "state": self._state,
            "detail": self._detail,
            "mode": plan.mode if plan else "",
            "waypoint": plan.index + 1 if plan else 0,
            "waypoints": len(self._waypoints),
            "laps": plan.laps if plan else 0,
            "reached": plan.reached if plan else 0,
            "skipped": plan.skipped if plan else 0,
            "retried": plan.retried if plan else 0,
            "docks": self._docks,
            "dock_failures": self._dock_failures,
            "battery": self._level,
            "phase_s": int(time.monotonic() - self._phase_started),
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = stamp
        line.ns = "patrol_loop"
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.03
        line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 0.6, 1.0, 0.8
        line.pose.orientation.w = 1.0
        for x, y, _yaw in self._waypoints + self._waypoints[:1]:
            line.points.append(Point(x=x, y=y, z=0.05))
        markers.markers.append(line)
        for number, (x, y, yaw) in enumerate(self._waypoints, 1):
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = stamp
            arrow.ns = "patrol_waypoints"
            arrow.id = number
            arrow.type = Marker.ARROW
            arrow.pose.position.x, arrow.pose.position.y = x, y
            arrow.pose.orientation.z, arrow.pose.orientation.w = _quaternion_z_w(yaw)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.4, 0.06, 0.06
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.5, 0.0, 1.0
            markers.markers.append(arrow)
            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "patrol_labels"
            label.id = number
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = 0.3
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(number)
            markers.markers.append(label)
        self._marker_pub.publish(markers)

    def _open_log(self, mode: str) -> None:
        self._log_path = None
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            path = os.path.join(
                self._log_dir, f"patrol_{datetime.now():%Y%m%d_%H%M%S}_{mode}.csv"
            )
            with open(path, "w", newline="") as handle:
                csv.writer(handle).writerow([
                    "time", "elapsed_s", "event", "lap", "waypoint", "duration_s",
                    "voltage", "level", "docks", "detail",
                ])
            self._log_path = path
            self.get_logger().info(f"Patrol log: {path}")
        except OSError as error:
            self.get_logger().warn(f"Patrol log disabled: {error}")

    def _log(self, event: str, detail: str, waypoint: int | None = None,
             duration: float | None = None) -> None:
        if self._log_path is None:
            return
        plan = self._plan
        row = [
            datetime.now().isoformat(timespec="seconds"),
            "%.0f" % (time.monotonic() - self._run_started),
            event,
            plan.laps + 1 if plan else "",
            waypoint if waypoint is not None else "",
            "%.1f" % duration if duration is not None else "",
            "%.2f" % self._voltage if self._voltage is not None else "",
            self._level,
            self._docks,
            detail,
        ]
        try:
            with open(self._log_path, "a", newline="") as handle:
                csv.writer(handle).writerow(row)
        except OSError as error:
            self.get_logger().warn(f"Patrol log write failed: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Patrol()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
