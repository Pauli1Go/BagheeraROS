"""AprilTag-guided docking triggered through a ROS topic."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import json
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped
from mowgli_interfaces.msg import Power, WheelTick
from nav_msgs.msg import Odometry
from nav2_msgs.action import BackUp, NavigateToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _quaternion_z(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass(frozen=True)
class TagObservation:
    tag_id: int
    received_at: float
    bearing: float
    heading: float
    lateral_m: float
    range_m: float
    pose_valid: bool
    odom_yaw: float | None
    edge_pixels: float
    decision_margin: float
    heading_source: str


class DockingController(Node):
    """Navigate to the dock entrance and perform the final visual approach."""

    ACTIVE_STATES = {
        "PREPARING",
        "NAVIGATING",
        "ACQUIRING",
        "SMALL_ACQUIRING",
        "SMALL_APPROACHING",
        "AXIS_ALIGNING",
        "APPROACHING",
        "BACKOFF_WAIT",
        "BACKING_UP",
        "TERMINAL_DRIVE",
        "WAITING_DOCK",
    }

    def __init__(self) -> None:
        super().__init__("bagheera_docking_controller")

        self.declare_parameter("dock_pose_x", 1.192)
        self.declare_parameter("dock_pose_y", 1.884)
        self.declare_parameter("dock_pose_yaw", 1.527)
        self.declare_parameter("staging_distance_m", 1.30)
        self.declare_parameter("navigation_timeout_s", 120.0)
        self.declare_parameter("acquire_timeout_s", 8.0)
        self.declare_parameter("tag_timeout_s", 1.0)
        self.declare_parameter("tag_loss_timeout_s", 6.0)
        self.declare_parameter("vision_scale", 0.5)
        self.declare_parameter("minimum_decision_margin", 15.0)
        self.declare_parameter("bearing_gain", 0.5)
        self.declare_parameter("maximum_angular_speed_rps", 0.20)
        self.declare_parameter("maximum_driving_angular_speed_rps", 0.16)
        self.declare_parameter("maximum_coarse_angular_speed_rps", 0.16)
        self.declare_parameter("heading_gain", 1.5)
        self.declare_parameter("maximum_intercept_angle_rad", math.radians(20.0))
        self.declare_parameter("camera_lever_arm_m", 0.4334103944436625)
        self.declare_parameter("small_tag_heading_reference_rad", math.radians(2.33))
        self.declare_parameter("large_tag_heading_reference_rad", math.radians(1.4))
        self.declare_parameter("small_tag_lateral_reference_m", -0.008)
        self.declare_parameter("approach_speed_far_mps", 0.10)
        self.declare_parameter("approach_speed_alignment_mps", 0.08)
        self.declare_parameter("approach_speed_near_mps", 0.05)
        self.declare_parameter("align_tolerance_rad", math.radians(3.0))
        self.declare_parameter("realign_threshold_rad", math.radians(8.0))
        self.declare_parameter("axis_alignment_range_m", 0.85)
        self.declare_parameter("axis_lateral_tolerance_m", 0.07)
        self.declare_parameter("axis_bearing_tolerance_rad", math.radians(90.0))
        self.declare_parameter("axis_alignment_timeout_s", 4.0)
        self.declare_parameter("axis_heading_tolerance_rad", math.radians(3.0))
        self.declare_parameter("straight_gate_range_m", 0.55)
        self.declare_parameter("straight_gate_lateral_tolerance_m", 0.08)
        self.declare_parameter("backoff_distance_m", 0.50)
        self.declare_parameter("backoff_speed_mps", 0.08)
        self.declare_parameter("maximum_retries", 2)
        self.declare_parameter("terminal_lateral_tolerance_m", 0.015)
        self.declare_parameter("terminal_edge_pixels", 300.0)
        self.declare_parameter("terminal_bearing_tolerance_rad", math.radians(6.0))
        self.declare_parameter("terminal_speed_mps", 0.07)
        self.declare_parameter("terminal_distance_m", 0.065)
        self.declare_parameter("terminal_timeout_s", 2.0)
        self.declare_parameter("small_start_min_range_m", 0.12)
        self.declare_parameter("small_start_max_range_m", 1.50)
        self.declare_parameter("small_approach_speed_mps", 0.09)
        self.declare_parameter("small_straighten_range_m", 0.48)
        self.declare_parameter("small_pose_gate_range_m", 0.22)
        self.declare_parameter("small_pose_max_lateral_m", 0.035)
        self.declare_parameter("small_pose_max_heading_rad", math.radians(6.0))
        self.declare_parameter("small_max_travel_m", 1.60)
        self.declare_parameter("electrical_contact_voltage", 0.5)
        self.declare_parameter("dock_wait_timeout_s", 15.0)

        self._dock_x = float(self.get_parameter("dock_pose_x").value)
        self._dock_y = float(self.get_parameter("dock_pose_y").value)
        self._dock_yaw = float(self.get_parameter("dock_pose_yaw").value)
        self._staging_distance = float(
            self.get_parameter("staging_distance_m").value
        )
        self._navigation_timeout = float(
            self.get_parameter("navigation_timeout_s").value
        )
        self._acquire_timeout = float(self.get_parameter("acquire_timeout_s").value)
        self._tag_timeout = float(self.get_parameter("tag_timeout_s").value)
        self._tag_loss_timeout = float(
            self.get_parameter("tag_loss_timeout_s").value
        )
        self._vision_scale = float(self.get_parameter("vision_scale").value)
        if not 0.0 < self._vision_scale <= 1.0:
            raise ValueError("vision_scale must be in (0, 1]")
        self._minimum_margin = float(
            self.get_parameter("minimum_decision_margin").value
        )
        self._bearing_gain = float(self.get_parameter("bearing_gain").value)
        self._maximum_angular = float(
            self.get_parameter("maximum_angular_speed_rps").value
        )
        self._maximum_driving_angular = float(
            self.get_parameter("maximum_driving_angular_speed_rps").value
        )
        self._maximum_coarse_angular = float(
            self.get_parameter("maximum_coarse_angular_speed_rps").value
        )
        self._heading_gain = float(self.get_parameter("heading_gain").value)
        self._maximum_intercept_angle = float(
            self.get_parameter("maximum_intercept_angle_rad").value
        )
        self._camera_lever_arm = float(
            self.get_parameter("camera_lever_arm_m").value
        )
        self._small_heading_reference = float(
            self.get_parameter("small_tag_heading_reference_rad").value
        )
        self._large_heading_reference = float(
            self.get_parameter("large_tag_heading_reference_rad").value
        )
        self._small_lateral_reference = float(
            self.get_parameter("small_tag_lateral_reference_m").value
        )
        self._approach_speed_far = float(
            self.get_parameter("approach_speed_far_mps").value
        )
        self._approach_speed_alignment = float(
            self.get_parameter("approach_speed_alignment_mps").value
        )
        self._approach_speed_near = float(
            self.get_parameter("approach_speed_near_mps").value
        )
        self._align_tolerance = float(
            self.get_parameter("align_tolerance_rad").value
        )
        self._realign_threshold = float(
            self.get_parameter("realign_threshold_rad").value
        )
        self._axis_alignment_range = float(
            self.get_parameter("axis_alignment_range_m").value
        )
        self._axis_lateral_tolerance = float(
            self.get_parameter("axis_lateral_tolerance_m").value
        )
        self._axis_bearing_tolerance = float(
            self.get_parameter("axis_bearing_tolerance_rad").value
        )
        self._axis_alignment_timeout = float(
            self.get_parameter("axis_alignment_timeout_s").value
        )
        self._axis_heading_tolerance = float(
            self.get_parameter("axis_heading_tolerance_rad").value
        )
        self._straight_gate_range = float(
            self.get_parameter("straight_gate_range_m").value
        )
        self._straight_gate_lateral_tolerance = float(
            self.get_parameter("straight_gate_lateral_tolerance_m").value
        )
        self._backoff_distance = float(self.get_parameter("backoff_distance_m").value)
        self._backoff_speed = float(self.get_parameter("backoff_speed_mps").value)
        self._maximum_retries = int(self.get_parameter("maximum_retries").value)
        self._terminal_lateral_tolerance = float(
            self.get_parameter("terminal_lateral_tolerance_m").value
        )
        self._terminal_edge = float(
            self.get_parameter("terminal_edge_pixels").value
        )
        self._terminal_bearing_tolerance = float(
            self.get_parameter("terminal_bearing_tolerance_rad").value
        )
        self._terminal_speed = float(
            self.get_parameter("terminal_speed_mps").value
        )
        self._terminal_distance = float(
            self.get_parameter("terminal_distance_m").value
        )
        self._terminal_timeout = float(
            self.get_parameter("terminal_timeout_s").value
        )
        self._small_start_min = float(self.get_parameter("small_start_min_range_m").value)
        self._small_start_max = float(self.get_parameter("small_start_max_range_m").value)
        self._small_speed = float(self.get_parameter("small_approach_speed_mps").value)
        self._small_straighten_range = float(
            self.get_parameter("small_straighten_range_m").value
        )
        self._small_pose_gate_range = float(
            self.get_parameter("small_pose_gate_range_m").value
        )
        self._small_pose_lateral = float(
            self.get_parameter("small_pose_max_lateral_m").value
        )
        self._small_pose_heading = float(
            self.get_parameter("small_pose_max_heading_rad").value
        )
        self._small_max_travel = float(self.get_parameter("small_max_travel_m").value)
        self._contact_voltage = float(
            self.get_parameter("electrical_contact_voltage").value
        )
        self._dock_wait_timeout = float(
            self.get_parameter("dock_wait_timeout_s").value
        )

        self._state = "IDLE"
        self._state_started = time.monotonic()
        self._attempt_started = 0.0
        self._status_detail = "ready"
        self._docked = False
        self._charge_voltage = 0.0
        self._camera_matrix: np.ndarray | None = None
        self._distortion: np.ndarray | None = None
        self._latest_observations: dict[int, TagObservation] = {}
        self._observation_windows = {0: deque(maxlen=5), 1: deque(maxlen=3)}
        self._small_tag_locked = False
        self._odom_yaw: float | None = None
        self._alignment_since: float | None = None
        self._axis_yaw: float | None = None
        self._approach_yaw_target: float | None = None
        self._approach_turn_direction = 0
        self._last_turn_end = 0.0
        self._staging_lateral_correction = 0.0
        self._staging_yaw_correction = 0.0
        self._best_approach_range = math.inf
        self._last_approach_progress = 0.0
        self._straight_gate_checked = False
        self._straight_gate_started = 0.0
        self._straight_gate_unmeasurable_since = 0.0
        self._axle_lateral_window: deque[tuple[float, float]] = deque(maxlen=9)
        self._last_axle_sample_at = 0.0
        self._axis_samples: deque[tuple[float, float, float]] = deque(maxlen=8)
        self._last_axis_sample_at = 0.0
        self._retry_count = 0
        self._retry_backoff_distance = 0.0
        self._last_tag_seen = 0.0
        self._last_control_log = 0.0
        self._wheel_ticks: WheelTick | None = None
        self._last_wheel_ticks_at = 0.0
        self._terminal_start_ticks: tuple[int, int, float] | None = None
        self._small_start_ticks: tuple[int, int, float] | None = None
        self._small_yaw_start: float | None = None
        self._small_yaw_target: float | None = None
        self._small_straightened = False
        self._small_pose_checked = False
        self._small_pose_gate_started = 0.0
        self._small_turn_direction = 0
        self._small_last_turn_end = 0.0
        self._small_start_range = 0.0
        self._small_tag_xy: tuple[float, float] | None = None
        self._small_last_target_update = 0.0
        self._small_last_visual_progress = 0.0
        self._small_heading_target: float | None = None
        self._small_backoff = False
        self._odom_x: float | None = None
        self._odom_y: float | None = None
        self._last_odom_at = 0.0
        self._odom_linear = 0.0
        self._odom_angular = 0.0
        self._navigation_active = False
        self._nav_goal_handle = None
        self._nav_goal_pending = False
        self._backoff_goal_handle = None
        self._active_subscriptions = []

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._velocity_publisher = self.create_publisher(
            TwistStamped, "/cmd_vel_docking", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/dock/status", latched_qos
        )
        self._active_publisher = self.create_publisher(
            Bool, "/dock/active", latched_qos
        )
        self._staging_pose_publisher = self.create_publisher(
            PoseStamped, "/dock/staging_pose", latched_qos
        )
        self._vision_enabled_publisher = self.create_publisher(
            Bool, "/dock/vision_enabled", latched_qos
        )

        self.create_subscription(Bool, "/dock/trigger", self._on_trigger, 10)
        self.create_subscription(Bool, "/dock/small_trigger", self._on_small_trigger, 10)
        self.create_subscription(Bool, "/dock/cancel", self._on_cancel, 10)
        self.create_subscription(Bool, "/docked", self._on_docked, latched_qos)
        self.create_subscription(Power, "/hardware_bridge/power", self._on_power, 10)
        self._vision_enabled = False
        self.create_subscription(
            TwistStamped, "/cmd_vel_teleop", self._on_teleop, 10
        )
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status",
            self._on_navigation_status, latched_qos,
        )

        self._navigation_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self._backoff_client = ActionClient(self, BackUp, "/backup")
        self._tick_timer = self.create_timer(0.05, self._tick)
        self._tick_timer.cancel()
        self.create_timer(1.0, self._publish_status)
        self._publish_status()
        self._vision_enabled_publisher.publish(Bool(data=False))
        self.get_logger().info(
            "Docking ready: /dock/trigger (Nav2) or /dock/small_trigger (ID 1 only)"
        )

    def _active(self) -> bool:
        return self._state in self.ACTIVE_STATES

    def _set_state(self, state: str, detail: str) -> None:
        if state != self._state or detail != self._status_detail:
            self.get_logger().info(f"Docking {state}: {detail}")
        self._state = state
        self._state_started = time.monotonic()
        self._status_detail = detail
        if self._active():
            self._enable_active_subscriptions()
            self._tick_timer.reset()
        else:
            self._tick_timer.cancel()
            self._disable_active_subscriptions()
        self._publish_status()

    def _enable_active_subscriptions(self) -> None:
        if self._active_subscriptions:
            return
        self._active_subscriptions = [
            self.create_subscription(WheelTick, "/wheel_ticks", self._on_wheel_ticks, 20),
            self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 20),
            self.create_subscription(
                AprilTagDetectionArray, "/dock/tags", self._on_tags, 10
            ),
            self.create_subscription(
                CameraInfo, "/camera/camera_info", self._on_camera_info, 10
            ),
        ]

    def _disable_active_subscriptions(self) -> None:
        for subscription in self._active_subscriptions:
            self.destroy_subscription(subscription)
        self._active_subscriptions.clear()

    def _publish_status(self) -> None:
        payload = {
            "state": self._state,
            "detail": self._status_detail,
            "active": self._active(),
            "docked": self._docked,
            "charge_voltage": round(self._charge_voltage, 3),
            "retry": self._retry_count,
            "maximum_retries": self._maximum_retries,
        }
        self._status_publisher.publish(String(data=json.dumps(payload)))
        self._active_publisher.publish(Bool(data=self._active()))

    def _on_trigger(self, message: Bool) -> None:
        if not message.data:
            return
        if self._active():
            self.get_logger().warn("Docking trigger ignored: an attempt is active")
            return
        if self._docked:
            self._set_state("SUCCEEDED", "robot is already docked")
            return

        self._latest_observations.clear()
        for window in self._observation_windows.values():
            window.clear()
        self._small_tag_locked = False
        self._last_tag_seen = 0.0
        self._alignment_since = None
        self._axis_yaw = None
        self._approach_yaw_target = None
        self._approach_turn_direction = 0
        self._last_turn_end = 0.0
        self._staging_lateral_correction = 0.0
        self._staging_yaw_correction = 0.0
        self._best_approach_range = math.inf
        self._last_approach_progress = 0.0
        self._straight_gate_checked = False
        self._straight_gate_started = 0.0
        self._straight_gate_unmeasurable_since = 0.0
        self._axle_lateral_window.clear()
        self._last_axle_sample_at = 0.0
        self._axis_samples.clear()
        self._last_axis_sample_at = 0.0
        self._retry_count = 0
        self._retry_backoff_distance = 0.0
        self._terminal_start_ticks = None
        self._wheel_ticks = None
        self._odom_yaw = None
        self._attempt_started = time.monotonic()
        self._set_state("PREPARING", "waiting for Nav2 action server")

    def _on_small_trigger(self, message: Bool) -> None:
        if not message.data:
            return
        if self._active():
            self.get_logger().warn("Small-tag trigger ignored: docking is active")
            return
        if self._docked:
            self._set_state("SUCCEEDED", "robot is already docked")
            return
        if self._navigation_active:
            self._set_state("FAILED", "Nav2 goal active; cancel it before small-tag docking")
            return
        self._latest_observations.clear()
        for window in self._observation_windows.values():
            window.clear()
        self._last_tag_seen = 0.0
        self._wheel_ticks = None
        self._terminal_start_ticks = None
        self._small_start_ticks = None
        self._small_yaw_start = None
        self._small_yaw_target = None
        self._small_straightened = False
        self._small_pose_checked = False
        self._small_pose_gate_started = 0.0
        self._small_turn_direction = 0
        self._small_last_turn_end = 0.0
        self._small_start_range = 0.0
        self._small_tag_xy = None
        self._small_last_target_update = 0.0
        self._small_last_visual_progress = 0.0
        self._small_heading_target = None
        self._small_backoff = False
        self._retry_count = 0
        self._best_approach_range = math.inf
        self._last_approach_progress = time.monotonic()
        self._last_control_log = 0.0
        self._attempt_started = time.monotonic()
        self._set_state("SMALL_ACQUIRING", "ID 1 visual approach; collision-checked BackUp if too close")
        self._enable_camera()

    def _on_navigation_status(self, message: GoalStatusArray) -> None:
        self._navigation_active = any(
            item.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
            for item in message.status_list
        )

    def _on_cancel(self, message: Bool) -> None:
        if message.data and self._active():
            self._cancel_navigation()
            self._finish("CANCELLED", "cancel requested")

    def _on_docked(self, message: Bool) -> None:
        self._docked = bool(message.data)
        if self._docked and self._state in {
            "SMALL_ACQUIRING", "SMALL_APPROACHING", "TERMINAL_DRIVE", "WAITING_DOCK"
        }:
            self._finish("SUCCEEDED", "dock voltage confirmed")

    def _on_power(self, message: Power) -> None:
        self._charge_voltage = float(message.v_charge)
        if (
            self._state in {"SMALL_ACQUIRING", "SMALL_APPROACHING", "TERMINAL_DRIVE"}
            and math.isfinite(self._charge_voltage)
            and self._charge_voltage >= self._contact_voltage
        ):
            self._publish_velocity()
            self._set_state("WAITING_DOCK", "electrical contact; waiting for /docked")

    def _on_wheel_ticks(self, message: WheelTick) -> None:
        self._wheel_ticks = message
        self._last_wheel_ticks_at = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self._odom_x = float(message.pose.pose.position.x)
        self._odom_y = float(message.pose.pose.position.y)
        q = message.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._last_odom_at = time.monotonic()
        self._odom_linear = float(message.twist.twist.linear.x)
        self._odom_angular = float(message.twist.twist.angular.z)

    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.width > 0 and message.height > 0 and len(message.k) == 9:
            matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(message.d, dtype=np.float64)
            if matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0 and distortion.size >= 4:
                self._camera_matrix = matrix
                self._distortion = distortion[:4]

    def _enable_camera(self) -> None:
        if not self._vision_enabled:
            self._vision_enabled = True
            self._vision_enabled_publisher.publish(Bool(data=True))

    def _disable_camera(self) -> None:
        if self._vision_enabled:
            self._vision_enabled = False
            self._vision_enabled_publisher.publish(Bool(data=False))

    def _on_teleop(self, message: TwistStamped) -> None:
        twist = message.twist
        if self._active() and (
            abs(twist.linear.x) > 1.0e-3 or abs(twist.angular.z) > 1.0e-3
        ):
            self._cancel_navigation()
            self._finish("CANCELLED", "manual controller override")

    def _observation_from_detection(self, detection) -> TagObservation | None:
        if self._camera_matrix is None or self._distortion is None:
            return None
        if detection.id not in (0, 1):
            return None
        if detection.hamming != 0 or detection.decision_margin < self._minimum_margin:
            return None

        corners = np.asarray(
            [[point.x, point.y] for point in detection.corners], dtype=np.float64
        )
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            return None
        # The control thresholds and original calibration are expressed in
        # full-resolution pixels, independent of the detector image size.
        corners /= self._vision_scale
        edge_pixels = float(
            np.mean(
                [
                    np.linalg.norm(corners[(index + 1) % 4] - corners[index])
                    for index in range(4)
                ]
            )
        )
        if not math.isfinite(edge_pixels) or edge_pixels < 16.0:
            return None
        try:
            undistorted = cv2.fisheye.undistortPoints(
                corners.reshape(-1, 1, 2),
                self._camera_matrix,
                self._distortion,
            ).reshape(-1, 2)
        except cv2.error:
            return None
        if not np.isfinite(undistorted).all():
            return None
        bearing = math.atan2(float(np.mean(undistorted[:, 0])), 1.0)
        tag_size = 0.088889 if detection.id == 0 else 0.026667
        range_m = tag_size * math.sqrt(
            float(self._camera_matrix[0, 0] * self._camera_matrix[1, 1])
        ) / edge_pixels
        heading = 0.0
        lateral = 0.0
        pose_valid = False
        heading_source = "none"

        # ID 1 is too small for a stable plane normal at 0.5 m. ID 0 is
        # visible there and supplies only the dock-axis angle; ID 1 remains
        # authoritative for the target center and the final approach.
        # The large wall tag is ~80 px at the actual 1 m staging range.
        # Use it early, but only after multiple gyro-compensated samples agree.
        minimum_edge = 75.0 if detection.id == 0 else 100.0
        if edge_pixels >= minimum_edge:
            half = tag_size / 2.0
            # AprilTag p[0..3] winds around the square. IPPE_SQUARE uses
            # bottom-left, bottom-right, top-right, top-left model points.
            object_points = np.asarray(
                [
                    (-half, half, 0.0),
                    (half, half, 0.0),
                    (half, -half, 0.0),
                    (-half, -half, 0.0),
                ],
                dtype=np.float64,
            )
            try:
                success, rotation, translation = cv2.solvePnP(
                    object_points,
                    undistorted,
                    np.eye(3, dtype=np.float64),
                    None,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
            except cv2.error:
                success = False
            if success and np.isfinite(translation).all():
                tx, _, tz = translation.reshape(3)
                normal = cv2.Rodrigues(rotation)[0][:, 2]
                if normal[2] > 0.0:
                    normal = -normal
                if 0.04 < tz < 1.7 and np.isfinite(normal).all() and normal[2] < -0.8:
                    # The tag normal is expressed in camera coordinates. Its
                    # apparent yaw changes opposite to the robot's yaw: a
                    # right turn makes raw_heading increase. Steering and the
                    # gyro-compensated observation window require robot yaw.
                    raw_heading = -math.atan2(float(normal[0]), float(-normal[2]))
                    candidate_heading = _angle_delta(
                        raw_heading,
                        self._large_heading_reference
                        if detection.id == 0 else self._small_heading_reference,
                    )
                    if detection.id == 0:
                        if abs(candidate_heading) < math.radians(25.0):
                            heading = candidate_heading
                            pose_valid = True
                            heading_source = "large"
                    else:
                        candidate_bearing = math.atan2(float(tx), float(tz))
                        angle_to_lateral = _angle_delta(candidate_bearing, candidate_heading)
                        if abs(angle_to_lateral) < math.radians(35.0):
                            candidate_lateral = (
                                float(tz) * math.tan(angle_to_lateral)
                                - self._small_lateral_reference
                            )
                            if (
                                abs(candidate_heading) < math.radians(25.0)
                                and abs(candidate_lateral) < 0.15
                            ):
                                heading = candidate_heading
                                lateral = candidate_lateral
                                range_m = float(tz)
                                pose_valid = True
                                heading_source = "small"
        return TagObservation(
            tag_id=int(detection.id),
            received_at=time.monotonic(),
            bearing=bearing,
            heading=heading,
            lateral_m=lateral,
            range_m=range_m,
            pose_valid=pose_valid,
            odom_yaw=self._odom_yaw,
            edge_pixels=edge_pixels,
            decision_margin=float(detection.decision_margin),
            heading_source=heading_source,
        )

    def _on_tags(self, message: AprilTagDetectionArray) -> None:
        if not self._active():
            return
        for detection in message.detections:
            if self._state in {"SMALL_ACQUIRING", "SMALL_APPROACHING"} and detection.id != 1:
                continue
            observation = self._observation_from_detection(detection)
            if observation is None:
                continue
            window = self._observation_windows[observation.tag_id]
            window.append(observation)
            self._latest_observations[observation.tag_id] = observation
            if observation.tag_id == 1:
                self._small_tag_locked = True
            # Both IDs prove that the dock is still visible. ID 0 must remain
            # usable when the much smaller ID 1 temporarily drops out.
            self._last_tag_seen = observation.received_at

    def _current_observation(self, now: float) -> TagObservation | None:
        large = self._latest_observations.get(0)
        if large is not None and now - large.received_at > self._tag_timeout:
            large = None
        for tag_id in (1, 0):
            observation = self._latest_observations.get(tag_id)
            if (
                observation is not None
                and now - observation.received_at <= self._tag_timeout
            ):
                if tag_id == 0:
                    return observation
                if not observation.pose_valid:
                    return self._with_large_heading(observation, large)
                headings = []
                laterals = []
                for item in self._observation_windows[1]:
                    if now - item.received_at > 1.5 or not item.pose_valid:
                        continue
                    predicted = item.heading
                    if self._odom_yaw is not None and item.odom_yaw is not None:
                        predicted += _angle_delta(self._odom_yaw, item.odom_yaw)
                    headings.append(predicted)
                    laterals.append(item.lateral_m)
                if (
                    len(headings) < 3
                    or max(headings) - min(headings) > math.radians(8.0)
                    or max(laterals) - min(laterals) > 0.05
                ):
                    return self._with_large_heading(
                        replace(observation, pose_valid=False), large
                    )
                return replace(observation, heading=float(np.median(headings)))
        return None

    def _with_large_heading(
        self, small: TagObservation, large: TagObservation | None
    ) -> TagObservation:
        if large is None or not large.pose_valid:
            return small
        headings = []
        for item in self._observation_windows[0]:
            if time.monotonic() - item.received_at > 1.5 or not item.pose_valid:
                continue
            predicted = item.heading
            if self._odom_yaw is not None and item.odom_yaw is not None:
                predicted += _angle_delta(self._odom_yaw, item.odom_yaw)
            headings.append(predicted)
        if len(headings) < 3:
            return small
        median = float(np.median(headings))
        inliers = [
            value for value in headings
            if abs(_angle_delta(value, median)) <= math.radians(5.0)
        ]
        if len(inliers) < 3 or max(inliers) - min(inliers) > math.radians(8.0):
            return small
        heading = float(np.median(inliers))
        lateral = small.range_m * math.tan(
            _angle_delta(small.bearing, heading)
        ) - self._small_lateral_reference
        if abs(lateral) >= 0.25:
            return small
        return replace(
            small,
            heading=heading,
            lateral_m=lateral,
            pose_valid=True,
            heading_source="large",
        )

    def _publish_velocity(self, linear: float = 0.0, angular: float = 0.0) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(linear)
        message.twist.angular.z = float(angular)
        self._velocity_publisher.publish(message)

    def _staging_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self._dock_x - self._staging_distance * math.cos(
            self._dock_yaw
        )
        pose.pose.position.y = self._dock_y - self._staging_distance * math.sin(
            self._dock_yaw
        )
        # A visual lateral error is measured relative to the dock axis. On a
        # retry, ask Nav2 to move the staging pose toward the actual tag
        # center; Nav2's costmaps check this repositioning.
        pose.pose.position.x += self._staging_lateral_correction * math.sin(
            self._dock_yaw
        )
        pose.pose.position.y -= self._staging_lateral_correction * math.cos(
            self._dock_yaw
        )
        pose.pose.orientation.z, pose.pose.orientation.w = _quaternion_z(
            self._dock_yaw + self._staging_yaw_correction
        )
        return pose

    def _send_navigation_goal(self) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = self._staging_pose()
        self._staging_pose_publisher.publish(goal.pose)
        self._nav_goal_pending = True
        future = self._navigation_client.send_goal_async(goal)
        future.add_done_callback(self._on_navigation_goal_response)
        self._set_state(
            "NAVIGATING", f"moving to the {self._staging_distance:.2f} m staging pose"
        )

    def _on_navigation_goal_response(self, future) -> None:
        self._nav_goal_pending = False
        if self._state != "NAVIGATING":
            return
        try:
            handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self._finish("FAILED", f"Nav2 goal error: {error}")
            return
        if not handle.accepted:
            self._finish("FAILED", "Nav2 rejected the staging goal")
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        if self._state != "NAVIGATING":
            return
        try:
            result = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self._finish("FAILED", f"Nav2 result error: {error}")
            return
        self._nav_goal_handle = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self._finish(
                "FAILED",
                f"staging navigation ended with status {result.status}",
            )
            return
        self._enable_camera()
        self._set_state("ACQUIRING", "staging pose reached; looking for dock tags")

    def _cancel_navigation(self) -> None:
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        if self._backoff_goal_handle is not None:
            self._backoff_goal_handle.cancel_goal_async()
            self._backoff_goal_handle = None

    def _retry_or_fail(
        self, reason: str, range_m: float, lateral_m: float | None = None,
        heading_rad: float | None = None,
    ) -> None:
        if self._retry_count >= self._maximum_retries:
            self._finish("FAILED", f"{reason}; {self._retry_count} retries exhausted")
            return
        self._retry_count += 1
        # Return near the original visual staging range. A retry from only a
        # few centimetres farther away cannot remove a lateral offset.
        self._retry_backoff_distance = _clamp(
            # Leave room for Nav2 to correct both yaw and lateral position.
            # Backing up only to the staging range forced an almost pure
            # sideways move directly beside the cabinet.
            max(self._backoff_distance, self._staging_distance - range_m + 0.30),
            self._backoff_distance,
            0.90,
        )
        if lateral_m is not None and math.isfinite(lateral_m):
            self._staging_lateral_correction = _clamp(
                self._staging_lateral_correction + lateral_m, -0.18, 0.18
            )
        if heading_rad is not None and math.isfinite(heading_rad):
            # The tag plane measures the robot's yaw relative to the dock.
            # Let Nav2 correct the staging orientation with its costmap rather
            # than pivoting blindly in the narrow visual approach corridor.
            self._staging_yaw_correction = _clamp(
                self._staging_yaw_correction - heading_rad,
                -math.radians(15.0), math.radians(15.0),
            )
        self._axis_yaw = None
        self._approach_yaw_target = None
        self._approach_turn_direction = 0
        self._last_turn_end = 0.0
        self._straight_gate_checked = False
        self._straight_gate_started = 0.0
        self._straight_gate_unmeasurable_since = 0.0
        self._alignment_since = None
        self._best_approach_range = math.inf
        self._last_approach_progress = 0.0
        self._publish_velocity()
        self._disable_camera()
        # Never rotate in place beside the dock/cabinet: that motion used the
        # direct docking lane and had no Nav2 collision check. BackUp keeps
        # its footprint collision checks; if its path is blocked, abort.
        self._set_state(
            "BACKOFF_WAIT",
            f"retry {self._retry_count}/{self._maximum_retries}: {reason}; "
            f"collision-checked BackUp {self._retry_backoff_distance:.2f} m, "
            f"staging shift {self._staging_lateral_correction:+.2f} m, "
            f"yaw {math.degrees(self._staging_yaw_correction):+.1f} deg",
        )

    def _start_backoff(self) -> None:
        goal = BackUp.Goal()
        goal.target.x = -self._retry_backoff_distance
        goal.speed = self._backoff_speed
        goal.time_allowance.sec = 15
        goal.disable_collision_checks = False
        future = self._backoff_client.send_goal_async(goal)
        future.add_done_callback(self._on_backoff_goal_response)
        self._set_state("BACKING_UP", "Nav2 BackUp with collision checking")

    def _on_backoff_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if self._state == "BACKING_UP":
                self._finish("FAILED", f"BackUp goal error: {error}")
            return
        if self._state != "BACKING_UP":
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._finish("FAILED", "Nav2 rejected collision-checked BackUp")
            return
        self._backoff_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_backoff_result)

    def _on_backoff_result(self, future) -> None:
        if self._state != "BACKING_UP":
            return
        try:
            result = future.result()
        except Exception as error:
            self._finish("FAILED", f"BackUp result error: {error}")
            return
        self._backoff_goal_handle = None
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code:
            self._finish(
                "FAILED",
                f"collision-checked BackUp failed: {result.result.error_msg}",
            )
            return
        self._latest_observations.clear()
        for window in self._observation_windows.values():
            window.clear()
        self._axle_lateral_window.clear()
        self._last_axle_sample_at = 0.0
        self._axis_samples.clear()
        self._last_axis_sample_at = 0.0
        self._small_tag_locked = False
        self._last_tag_seen = 0.0
        if self._small_backoff:
            self._small_backoff = False
            self._small_start_ticks = None
            self._small_tag_xy = None
            self._small_last_target_update = 0.0
            self._small_heading_target = None
            self._small_pose_checked = False
            self._best_approach_range = math.inf
            self._last_approach_progress = time.monotonic()
            self._set_state("SMALL_ACQUIRING", "BackUp complete; reacquiring ID 1 for a new arc")
            self._enable_camera()
            return
        self._set_state(
            "PREPARING", "backoff complete; Nav2 will reposition to staging pose"
        )

    def _start_terminal_drive(self) -> None:
        if self._wheel_ticks is None or time.monotonic() - self._last_wheel_ticks_at > 0.5:
            self._finish("FAILED", "wheel ticks unavailable for terminal approach")
            return
        factor = float(self._wheel_ticks.wheel_tick_factor)
        if not math.isfinite(factor) or factor <= 0.0:
            self._finish("FAILED", "invalid wheel tick factor")
            return
        self._terminal_start_ticks = (
            int(self._wheel_ticks.wheel_ticks_rl),
            int(self._wheel_ticks.wheel_ticks_rr),
            factor,
        )
        self._set_state(
            "TERMINAL_DRIVE",
            "ID 1 aligned and close; advancing at most 6.5 cm",
        )

    def _terminal_progress(self) -> float:
        if self._terminal_start_ticks is None or self._wheel_ticks is None:
            return 0.0
        left_start, right_start, factor = self._terminal_start_ticks
        left = abs(int(self._wheel_ticks.wheel_ticks_rl) - left_start)
        right = abs(int(self._wheel_ticks.wheel_ticks_rr) - right_start)
        return (left + right) / (2.0 * factor)

    def _small_progress(self) -> float:
        if self._small_start_ticks is None or self._wheel_ticks is None:
            return 0.0
        left_start, right_start, factor = self._small_start_ticks
        left = abs(int(self._wheel_ticks.wheel_ticks_rl) - left_start)
        right = abs(int(self._wheel_ticks.wheel_ticks_rr) - right_start)
        return (left + right) / (2.0 * factor)

    def _small_update_tag_target(self, observation: TagObservation) -> None:
        if (
            observation.received_at <= self._small_last_target_update
            or self._odom_x is None or self._odom_y is None or self._odom_yaw is None
        ):
            return
        # base_link is the axle midpoint; camera_x is measured from that axle.
        forward = self._camera_lever_arm + observation.range_m * math.cos(observation.bearing)
        left = -observation.range_m * math.sin(observation.bearing)
        cosine, sine = math.cos(self._odom_yaw), math.sin(self._odom_yaw)
        target = (
            self._odom_x + cosine * forward - sine * left,
            self._odom_y + sine * forward + cosine * left,
        )
        if self._small_tag_xy is None:
            self._small_tag_xy = target
        else:
            dx = target[0] - self._small_tag_xy[0]
            dy = target[1] - self._small_tag_xy[1]
            distance = math.hypot(dx, dy)
            scale = min(1.0, 0.06 / distance) if distance > 0.0 else 1.0
            self._small_tag_xy = (
                self._small_tag_xy[0] + 0.4 * scale * dx,
                self._small_tag_xy[1] + 0.4 * scale * dy,
            )
        self._small_last_target_update = observation.received_at
        self._small_last_visual_progress = self._small_progress()

    def _small_target_geometry(self) -> tuple[float, float] | None:
        if (
            self._small_tag_xy is None or self._odom_x is None
            or self._odom_y is None or self._odom_yaw is None
        ):
            return None
        camera_x = self._odom_x + self._camera_lever_arm * math.cos(self._odom_yaw)
        camera_y = self._odom_y + self._camera_lever_arm * math.sin(self._odom_yaw)
        dx = self._small_tag_xy[0] - camera_x
        dy = self._small_tag_xy[1] - camera_y
        bearing = -_angle_delta(math.atan2(dy, dx), self._odom_yaw)
        return math.hypot(dx, dy), bearing

    def _small_steering(self, visible: bool) -> tuple[float, float]:
        geometry = self._small_target_geometry()
        if geometry is None:
            return 0.0, 0.0
        distance, bearing = geometry
        speed = min(self._small_speed, 0.08) if distance < 0.32 else self._small_speed
        if not visible:
            return 0.0, 0.0
        angular = -2.0 * speed * math.sin(bearing) / max(0.20, distance)
        angular = _clamp(angular, -0.14 if distance >= 0.32 else -0.12,
                         0.14 if distance >= 0.32 else 0.12)
        return speed, angular

    def _small_backoff_for_offset(self, lateral_m: float) -> None:
        if self._retry_count >= self._maximum_retries:
            self._finish("FAILED", "ID 1 lateral offset remains after retries")
            return
        self._retry_count += 1
        self._small_backoff = True
        self._retry_backoff_distance = 0.30
        self._publish_velocity()
        self._disable_camera()
        self._set_state(
            "BACKOFF_WAIT",
            f"ID 1 lateral offset {lateral_m:+.3f} m; "
            "collision-checked 0.30 m BackUp before another arc",
        )

    def _finish(self, state: str, detail: str) -> None:
        self._publish_velocity()
        self._cancel_navigation()
        self._disable_camera()
        self._set_state(state, detail)

    def _axle_lateral(self, observation: TagObservation, now: float) -> float:
        # The camera is 43 cm ahead of the differential-drive pivot. A turn
        # moves the camera sideways, but not the axle. Guide the axle to the
        # dock axis so straightening the chassis does not recreate the offset.
        measured = observation.lateral_m - self._camera_lever_arm * math.sin(
            observation.heading
        )
        if observation.received_at > self._last_axle_sample_at:
            self._axle_lateral_window.append((observation.received_at, measured))
            self._last_axle_sample_at = observation.received_at
        recent = [
            value for stamp, value in self._axle_lateral_window
            if now - stamp <= 1.0
        ]
        return float(np.median(recent)) if recent else measured

    def _tick_small(self, now: float) -> None:
        if self._state == "SMALL_ACQUIRING":
            self._publish_velocity()
            if now - self._state_started > 15.0:
                self._finish("FAILED", "ID 1 not measurable at the start")
                return
            if (
                self._odom_yaw is None or now - self._last_odom_at > 0.5
                or self._wheel_ticks is None or now - self._last_wheel_ticks_at > 0.5
                or abs(self._odom_linear) > 0.01
                or abs(self._odom_angular) > 0.03
            ):
                return
            samples = [
                item for item in self._observation_windows[1]
                if now - item.received_at <= 1.0
            ]
            if len(samples) < 3:
                return
            bearings = [item.bearing for item in samples]
            ranges = [item.range_m for item in samples]
            if (
                max(bearings) - min(bearings) > math.radians(3.0)
                or max(ranges) - min(ranges) > 0.10
            ):
                return
            bearing = float(np.median(bearings))
            distance = float(np.median(ranges))
            if not self._small_start_min <= distance <= self._small_start_max:
                self._finish(
                    "FAILED", f"ID 1 range {distance:.2f} m outside measurable approach range"
                )
                return
            factor = float(self._wheel_ticks.wheel_tick_factor)
            if not math.isfinite(factor) or factor <= 0.0:
                self._finish("FAILED", "wheel ticks unavailable for bounded approach")
                return
            self._small_start_ticks = (
                int(self._wheel_ticks.wheel_ticks_rl),
                int(self._wheel_ticks.wheel_ticks_rr),
                factor,
            )
            self._small_yaw_start = self._odom_yaw
            self._small_start_range = distance
            self._best_approach_range = distance
            self._last_approach_progress = now
            self._set_state(
                "SMALL_APPROACHING",
                f"ID 1 at {distance:.2f} m, bearing {math.degrees(bearing):+.1f} deg; "
                "bounded forward-only approach",
            )
            return

        if self._navigation_active:
            self._finish("FAILED", "Nav2 goal started during small-tag approach")
            return
        if (
            now - self._last_odom_at > 0.5 or self._wheel_ticks is None
            or now - self._last_wheel_ticks_at > 0.5
        ):
            self._finish("FAILED", "odometry or wheel ticks became stale")
            return
        if self._small_progress() >= min(
            self._small_max_travel, self._small_start_range + 0.30
        ):
            self._finish("FAILED", "small-tag approach travel limit reached")
            return
        observation = self._current_observation(now)
        if observation is None or now - observation.received_at > 0.6:
            # Never steer toward a cached tag position while it is invisible.
            # The camera is far ahead of the axle and a turn can otherwise
            # create a large lateral sweep very close to the dock.
            self._publish_velocity()
            if now - self._last_tag_seen > 3.0:
                self._finish("FAILED", "ID 1 not reacquired while stopped")
            return
        self._small_update_tag_target(observation)
        if observation.range_m < self._best_approach_range - 0.03:
            self._best_approach_range = observation.range_m
            self._last_approach_progress = now
        if now - self._last_approach_progress > 10.0:
            self._finish("FAILED", "no forward progress toward ID 1")
            return
        if observation.pose_valid and observation.heading_source == "small":
            if observation.range_m < 0.25:
                laterals = [
                    item.lateral_m for item in self._observation_windows[1]
                    if item.pose_valid and now - item.received_at <= 1.0
                ]
                if (
                    len(laterals) >= 3
                    and max(laterals) - min(laterals) <= 0.03
                    and abs(float(np.median(laterals))) > 0.05
                ):
                    self._small_backoff_for_offset(float(np.median(laterals)))
                    return
            if (
                abs(observation.heading) > math.radians(8.0)
                or abs(observation.lateral_m) > 0.04
            ):
                self._small_pose_checked = False
        # Below 30 cm there is no room for a heading correction. Check the
        # *visual* range, not the odom-projected tag distance, which can lag.
        # A bad approach must stop before the charging hardware, not execute
        # a late arc or continue moving without a visible tag.
        if observation.range_m <= 0.30:
            visual_lateral = observation.range_m * math.tan(observation.bearing)
            if abs(visual_lateral) > 0.025:
                self._finish(
                    "FAILED",
                    f"ID 1 outside near corridor: visual lateral {visual_lateral:+.3f} m",
                )
                return
            if (
                observation.pose_valid
                and observation.heading_source == "small"
                and abs(observation.heading) > self._small_pose_heading
            ):
                self._finish(
                    "FAILED",
                    f"ID 1 heading {math.degrees(observation.heading):+.1f} deg "
                    "too large for near approach",
                )
                return
        if observation.range_m <= self._small_pose_gate_range and not self._small_pose_checked:
            if not self._small_pose_gate_started:
                self._small_pose_gate_started = now
            samples = [
                item for item in self._observation_windows[1]
                if item.pose_valid and now - item.received_at <= 1.0
            ]
            if len(samples) >= 3:
                headings = [item.heading for item in samples]
                laterals = [item.lateral_m for item in samples]
                stable = (
                    max(headings) - min(headings) <= math.radians(6.0)
                    and max(laterals) - min(laterals) <= 0.025
                )
                if stable:
                    heading = float(np.median(headings))
                    lateral = float(np.median(laterals))
                    if (
                        abs(heading) <= self._small_pose_heading
                        and abs(lateral) <= self._small_pose_lateral
                    ):
                        self._small_pose_checked = True
                        self.get_logger().info(
                            f"ID 1 near pose valid: heading {math.degrees(heading):+.1f} deg, "
                            f"lateral {lateral:+.3f} m"
                        )
        terminal_ready = (
            self._small_pose_checked
            and observation.edge_pixels >= self._terminal_edge
            and abs(observation.bearing) <= self._terminal_bearing_tolerance
            and observation.decision_margin >= 30.0
        )
        if terminal_ready:
            self._publish_velocity()
            self._start_terminal_drive()
            return
        if observation.edge_pixels >= self._terminal_edge + 30.0:
            self._publish_velocity()
            self._set_state(
                "WAITING_DOCK",
                "ID 1 at contact distance but alignment uncertain; waiting for delayed voltage",
            )
            return
        if observation.range_m <= 0.30:
            # Once the robot is this close, only a straight drive is safe.
            # The terminal pose check still gates the final contact motion.
            self._publish_velocity(linear=min(self._small_speed, 0.08), angular=0.0)
            return
        linear, angular = self._small_steering(visible=True)
        if now - self._last_control_log >= 1.0:
            geometry = self._small_target_geometry()
            self.get_logger().info(
                f"ID 1 only: range={observation.range_m:.3f} m "
                f"bearing={math.degrees(observation.bearing):+.1f} deg "
                f"heading={math.degrees(observation.heading):+.1f} deg "
                f"lateral={observation.lateral_m:+.3f} m "
                f"odom_bearing={math.degrees(geometry[1]) if geometry else float('nan'):+.1f} deg "
                f"speed={linear:.3f} m/s "
                f"yaw_cmd={angular:+.3f} rad/s "
                f"travel={self._small_progress():.3f} m "
                f"edge={observation.edge_pixels:.0f} px"
            )
            self._last_control_log = now
        self._publish_velocity(linear=linear, angular=angular)

    def _tick(self) -> None:
        now = time.monotonic()
        if self._active() and now - self._attempt_started > 180.0:
            self._finish("FAILED", "overall docking timeout")
            return

        if self._state in {"SMALL_ACQUIRING", "SMALL_APPROACHING"}:
            self._tick_small(now)
            return

        if self._state == "PREPARING":
            if self._navigation_client.server_is_ready():
                self._send_navigation_goal()
            elif now - self._state_started > 5.0:
                self._finish("FAILED", "Nav2 action server unavailable")
            return

        if self._state == "NAVIGATING":
            if now - self._state_started > self._navigation_timeout:
                self._finish("FAILED", "staging navigation timeout")
            return

        if self._state == "BACKOFF_WAIT":
            if now - self._state_started < 0.70:
                return
            if self._backoff_client.server_is_ready():
                self._start_backoff()
            elif now - self._state_started > 3.0:
                self._finish("FAILED", "Nav2 collision-checked BackUp unavailable")
            return

        if self._state == "BACKING_UP":
            if now - self._state_started > 17.0:
                self._finish("FAILED", "collision-checked BackUp timed out")
            return

        if self._state == "ACQUIRING":
            self._publish_velocity()
            if self._current_observation(now) is not None:
                if self._odom_yaw is None:
                    return
                self._alignment_since = None
                # Nav2 has just oriented the chassis at the staging pose.
                # Hold that yaw with the gyro instead of chasing the centre
                # of the 20-pixel small tag and steering across the dock.
                self._approach_yaw_target = self._odom_yaw
                self._approach_turn_direction = 0
                self._last_turn_end = 0.0
                self._best_approach_range = math.inf
                self._last_approach_progress = now
                self._set_state("APPROACHING", "dock tag acquired; slow visual approach")
            elif now - self._state_started > self._acquire_timeout:
                self._finish("FAILED", "no dock tag at staging pose")
            return

        if self._state in {"AXIS_ALIGNING", "APPROACHING"}:
            observation = self._current_observation(now)
            if observation is None:
                self._publish_velocity()
                if now - self._last_tag_seen > self._tag_loss_timeout:
                    # A temporary detector backlog must not terminate the
                    # whole attempt. Stay put and give both tags one fresh
                    # acquisition window before declaring them unavailable.
                    self._small_tag_locked = False
                    self._latest_observations.clear()
                    for window in self._observation_windows.values():
                        window.clear()
                    self._set_state("ACQUIRING", "tags lost; waiting to reacquire")
                return

            axle_lateral = None
            if observation.tag_id == 1 and observation.pose_valid:
                axle_lateral = self._axle_lateral(observation, now)
            if observation.pose_valid:
                # The small tag is only ~30 px wide at 1 m. Combining its
                # instantaneous bearing with the large-tag normal makes the
                # inferred axle offset jump by 10-20 cm while pivoting. Do
                # not chase that value with alternating full-speed turns.
                heading_error = observation.heading
                angular = -0.7 * heading_error
            else:
                # ID 0 only acquires ID 1. A distant ID 1 still owns the
                # steering, but its center bearing is reliable long before
                # its 3D plane normal becomes measurable.
                heading_error = observation.bearing
                angular = -self._bearing_gain * heading_error
            # At 0.1 m/s, 0.08 rad/s keeps the visual approach on a gentle
            # arc. Larger corrections are performed by collision-aware Nav2
            # restaging, not by sweeping the chassis next to the cabinet.
            limit = min(self._maximum_driving_angular, 0.08)
            angular = _clamp(angular, -limit, limit)
            if (
                self._state == "APPROACHING"
                and self._axis_yaw is None
                and observation.pose_valid
                and self._odom_yaw is not None
                and self._approach_yaw_target is None
            ):
                # Lock the dock direction once from the tag, then use the
                # gyro/odometry for steering. Re-reading the tiny tag every
                # frame previously reversed the motor command repeatedly.
                self._approach_yaw_target = _angle_delta(
                    self._odom_yaw, observation.heading
                )
            if (
                self._state == "APPROACHING"
                and self._axis_yaw is None
                and self._approach_yaw_target is not None
                and self._odom_yaw is not None
            ):
                yaw_error = _angle_delta(self._odom_yaw, self._approach_yaw_target)
                if self._approach_turn_direction and (
                    abs(yaw_error) <= math.radians(2.0)
                    or math.copysign(1.0, yaw_error) != self._approach_turn_direction
                ):
                    self._approach_turn_direction = 0
                    self._last_turn_end = now
                if (
                    not self._approach_turn_direction
                    and abs(yaw_error) >= math.radians(5.0)
                    and now - self._last_turn_end >= 0.6
                ):
                    self._approach_turn_direction = int(math.copysign(1.0, yaw_error))
                # A 2/5-degree hysteresis and reversal pause prevent the
                # left-right chatter; 0.13 rad/s clears the motor deadband.
                angular = -0.13 * self._approach_turn_direction
            if now - self._last_control_log >= 1.0:
                self.get_logger().info(
                    f"Dock control: id={observation.tag_id} "
                    f"pose={'valid' if observation.pose_valid else 'coarse'} "
                    f"heading_source={observation.heading_source} "
                    f"bearing={math.degrees(observation.bearing):+.1f} deg "
                    f"heading={math.degrees(observation.heading):+.1f} deg "
                    f"lateral={observation.lateral_m:+.3f} m "
                    f"axle_lateral={axle_lateral if axle_lateral is not None else float('nan'):+.3f} m "
                    f"range={observation.range_m:.3f} m "
                    f"yaw_cmd={angular:+.3f} rad/s "
                    f"edge={observation.edge_pixels:.0f} px"
                )
                self._last_control_log = now

            if self._state == "AXIS_ALIGNING":
                self._publish_velocity()
                # Do not turn in place near the cabinet. If the dock axis
                # does not agree with the robot, let Nav2 restage with a
                # corrected goal orientation and collision checking.
                if (
                    axle_lateral is not None
                    and observation.received_at > self._last_axis_sample_at
                    and now - self._state_started >= 0.35
                ):
                    self._axis_samples.append(
                        (observation.received_at, observation.heading, axle_lateral)
                    )
                    self._last_axis_sample_at = observation.received_at
                samples = [
                    (heading, lateral)
                    for stamp, heading, lateral in self._axis_samples
                    if now - stamp <= 1.5
                ]
                if len(samples) < 3:
                    if now - self._state_started > self._axis_alignment_timeout:
                        self._retry_or_fail("dock axis not measurable", observation.range_m)
                    return
                headings = [sample[0] for sample in samples]
                laterals = [sample[1] for sample in samples]
                stable = (
                    max(headings) - min(headings) <= math.radians(5.0)
                    and max(laterals) - min(laterals) <= 0.06
                )
                if not stable:
                    if now - self._state_started > self._axis_alignment_timeout:
                        self._retry_or_fail("dock axis measurement unstable", observation.range_m)
                    return
                heading = float(np.median(headings))
                lateral = float(np.median(laterals))
                if abs(lateral) > self._axis_lateral_tolerance:
                    self._retry_or_fail(
                        "offset too large for axis alignment",
                        observation.range_m, lateral, heading,
                    )
                    return
                if abs(heading) > self._axis_heading_tolerance:
                    self._retry_or_fail(
                        "dock heading too far from axis",
                        observation.range_m, lateral, heading,
                    )
                    return
                if self._odom_yaw is None:
                    self._finish("FAILED", "gyro heading unavailable")
                    return
                self._axis_yaw = self._odom_yaw
                self._best_approach_range = observation.range_m
                self._last_approach_progress = now
                self._set_state(
                    "APPROACHING",
                    f"dock axis verified at {observation.range_m:.2f} m; straight approach",
                )
                return

            if observation.range_m < self._best_approach_range - 0.03:
                self._best_approach_range = observation.range_m
                self._last_approach_progress = now
            if now - self._last_approach_progress > 8.0:
                self._retry_or_fail(
                    "no forward progress during visual approach",
                    observation.range_m,
                )
                return

            if observation.tag_id != 1:
                # ID 0 is centred above ID 1. It can guide the long-range
                # approach. Once the axis is verified, ID 1 must be visible:
                # the wall tag alone cannot safely guide terminal contact.
                if self._axis_yaw is not None:
                    self._publish_velocity()
                    small = self._latest_observations.get(1)
                    if small is None or now - small.received_at > 2.0:
                        self._retry_or_fail("small dock tag lost near dock", observation.range_m)
                    return
                self._publish_velocity(
                    linear=self._approach_speed_alignment,
                    angular=angular,
                )
                return
            if self._axis_yaw is None and observation.range_m <= self._axis_alignment_range:
                self._publish_velocity()
                self._alignment_since = None
                self._axle_lateral_window.clear()
                self._last_axle_sample_at = 0.0
                self._axis_samples.clear()
                self._last_axis_sample_at = 0.0
                self._set_state(
                    "AXIS_ALIGNING",
                    f"stop and verify dock axis at {self._axis_alignment_range:.2f} m",
                )
                return
            if self._axis_yaw is not None:
                if (
                    not self._straight_gate_checked
                    and observation.range_m <= self._straight_gate_range
                ):
                    self._publish_velocity()
                    if (
                        observation.pose_valid
                        and abs(observation.heading) > self._realign_threshold
                    ):
                        self._retry_or_fail(
                            "dock heading drift before 0.55 m gate",
                            observation.range_m, axle_lateral, observation.heading,
                        )
                        return
                    if axle_lateral is None:
                        # A missing/unstable plane estimate is not evidence
                        # of a lateral error. Wait for fresh tag data instead
                        # of repeatedly retreating from a centered position.
                        self._straight_gate_started = 0.0
                        if not self._straight_gate_unmeasurable_since:
                            self._straight_gate_unmeasurable_since = now
                        elif now - self._straight_gate_unmeasurable_since > 6.0:
                            self._finish("FAILED", "dock lateral not measurable at 0.55 m")
                        return
                    self._straight_gate_unmeasurable_since = 0.0
                    if (
                        abs(axle_lateral) > self._straight_gate_lateral_tolerance
                        or abs(observation.bearing) > self._axis_bearing_tolerance
                    ):
                        if not self._straight_gate_started:
                            self._straight_gate_started = now
                        elif now - self._straight_gate_started >= 0.35:
                            self._retry_or_fail(
                                "offset too large at 0.55 m",
                                observation.range_m,
                                axle_lateral,
                            )
                        return
                    self._straight_gate_checked = True
                    self._straight_gate_started = 0.0
                if (
                    observation.range_m < 0.40
                    and abs(observation.bearing) > math.radians(10.0)
                ):
                    self._retry_or_fail(
                        "dock center drifted out of straight corridor",
                        observation.range_m,
                    )
                    return
                # A small bounded visual trim removes the remaining centimetres
                # without returning to a sweeping intercept path near the dock.
                trim = 0.0
                if axle_lateral is not None:
                    trim = _clamp(
                        -math.atan2(axle_lateral, max(observation.range_m, 0.25)),
                        -math.radians(3.0),
                        math.radians(3.0),
                    )
                angular = _clamp(
                    1.5 * _angle_delta(self._axis_yaw + trim, self._odom_yaw),
                    -0.06, 0.06,
                )

            # Use the newest raw close-tag sample for the hand-off. Median
            # filtering is useful for steering, but would retain several
            # smaller historical tag sizes and delay the calibrated 300 px
            # threshold until ID 1 has already left the image.
            terminal_ready = (
                observation.pose_valid
                and observation.heading_source == "small"
                and observation.edge_pixels >= self._terminal_edge
                and abs(observation.bearing) <= self._terminal_bearing_tolerance
                and abs(observation.heading) <= self._align_tolerance
                and abs(observation.lateral_m) <= self._terminal_lateral_tolerance
                and observation.decision_margin >= 30.0
            )
            if terminal_ready:
                self._publish_velocity()
                self._start_terminal_drive()
                return

            if observation.tag_id == 1 and observation.edge_pixels >= 150.0:
                linear = self._approach_speed_near
            elif observation.pose_valid and observation.range_m <= 0.90:
                linear = self._approach_speed_alignment
            else:
                linear = self._approach_speed_far
            self._publish_velocity(linear=linear, angular=angular)
            return

        if self._state == "TERMINAL_DRIVE":
            if now - self._last_wheel_ticks_at > 0.5:
                self._finish("FAILED", "wheel ticks became stale during terminal drive")
                return
            if self._charge_voltage >= self._contact_voltage:
                self._publish_velocity()
                self._set_state(
                    "WAITING_DOCK", "electrical contact; waiting for /docked"
                )
                return
            progress = self._terminal_progress()
            if (
                progress >= self._terminal_distance
                or now - self._state_started >= self._terminal_timeout
            ):
                self._publish_velocity()
                self._set_state(
                    "WAITING_DOCK",
                    f"terminal motion stopped at {progress:.3f} m; waiting for voltage",
                )
                return
            self._publish_velocity(linear=self._terminal_speed)
            return

        if self._state == "WAITING_DOCK":
            self._publish_velocity()
            if self._docked:
                self._finish("SUCCEEDED", "dock voltage confirmed")
            elif now - self._state_started > self._dock_wait_timeout:
                self._finish("FAILED", "dock voltage did not reach the valid threshold")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_velocity()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
