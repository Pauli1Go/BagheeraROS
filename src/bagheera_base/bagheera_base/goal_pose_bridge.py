from __future__ import annotations

import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


DEFAULT_FOOTPRINT = "[[0.4434, 0.18], [0.4434, -0.18], [-0.0866, -0.18], [-0.0866, 0.18]]"


def _yaw(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse_footprint(text: str) -> list[tuple[float, float]]:
    points = json.loads(text)
    if len(points) < 3:
        raise ValueError("footprint needs at least three points")
    return [(float(x), float(y)) for x, y in points]


def _inside(polygon: list[tuple[float, float]], x: float, y: float) -> bool:
    inside = False
    for index, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[index - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def blocked_footprint_cell(
    grid: OccupancyGrid,
    footprint: list[tuple[float, float]],
    x: float,
    y: float,
    yaw: float,
    occupied_threshold: int,
) -> tuple[float, float] | None:
    """First grid cell under the oriented footprint that is occupied/unknown.

    Returns its world coordinates, or None when the whole footprint is free.
    base_link is Bagheera's axle, so the footprint reaches 0.44 m ahead: a goal
    point can be free while the nose of the robot would sit in a wall.
    """
    info = grid.info
    resolution = info.resolution
    origin_yaw = _yaw(info.origin.orientation)
    cos_o, sin_o = math.cos(origin_yaw), math.sin(origin_yaw)
    cos_r, sin_r = math.cos(yaw), math.sin(yaw)
    corners = [
        (x + cos_r * fx - sin_r * fy, y + sin_r * fx + cos_r * fy) for fx, fy in footprint
    ]
    min_x = min(c[0] for c in corners) - resolution
    max_x = max(c[0] for c in corners) + resolution
    min_y = min(c[1] for c in corners) - resolution
    max_y = max(c[1] for c in corners) + resolution
    steps_x = int(math.ceil((max_x - min_x) / resolution * 2.0)) + 1
    steps_y = int(math.ceil((max_y - min_y) / resolution * 2.0)) + 1
    # Sample at half-cell spacing so thin walls under the outline are hit.
    for i in range(steps_x):
        wx = min_x + i * resolution / 2.0
        for j in range(steps_y):
            wy = min_y + j * resolution / 2.0
            dx, dy = wx - x, wy - y
            if not _inside(footprint, cos_r * dx + sin_r * dy, -sin_r * dx + cos_r * dy):
                continue
            gx = wx - info.origin.position.x
            gy = wy - info.origin.position.y
            cell_x = math.floor((cos_o * gx + sin_o * gy) / resolution)
            cell_y = math.floor((-sin_o * gx + cos_o * gy) / resolution)
            if not (0 <= cell_x < info.width and 0 <= cell_y < info.height):
                return wx, wy
            value = grid.data[cell_y * info.width + cell_x]
            if value < 0 or value >= occupied_threshold:
                return wx, wy
    return None


class GoalPoseBridge(Node):
    """Forward Foxglove click-to-publish poses to Nav2's action server."""

    def __init__(self) -> None:
        super().__init__("bagheera_goal_pose_bridge")
        self.declare_parameter("footprint", DEFAULT_FOOTPRINT)
        self.declare_parameter("map_occupied_threshold", 65)
        self._footprint = parse_footprint(str(self.get_parameter("footprint").value))
        self._occupied_threshold = int(self.get_parameter("map_occupied_threshold").value)
        self._map: OccupancyGrid | None = None
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._cancel_in_progress = False
        self._pending_pose: PoseStamped | None = None
        self._keepout_mask: OccupancyGrid | None = None
        # While bagheera_dock_sleep has the LiDAR off, AMCL publishes no
        # map -> odom and bt_navigator aborts at once ("Initial robot pose is
        # not available"). Hold the goal, wake the robot, forward when awake.
        self._sleep_state = "awake"
        self._held_pose: PoseStamped | None = None
        self._held_since = 0.0
        self._wake_publisher = self.create_publisher(Bool, "/dock/wake", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_pose, 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self._on_pose, 10
        )
        mask_qos = QoSProfile(depth=1)
        mask_qos.reliability = ReliabilityPolicy.RELIABLE
        mask_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            "/keepout_filter_mask_live",
            self._on_keepout_mask,
            mask_qos,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, mask_qos)
        self.create_subscription(String, "/dock/sleep_state", self._on_sleep_state, mask_qos)
        self.create_timer(1.0, self._check_held_goal)
        self.get_logger().info(
            "Foxglove goals accepted on /goal_pose and /move_base_simple/goal"
        )

    def _on_sleep_state(self, message: String) -> None:
        self._sleep_state = message.data
        pose = self._held_pose
        if pose is None:
            return
        if self._sleep_state == "awake":
            self._held_pose = None
            self._forward(pose)
        elif self._sleep_state == "fault":
            self._held_pose = None
            self.get_logger().error("Dropping held goal: dock wake-up failed")

    def _check_held_goal(self) -> None:
        if self._held_pose is not None and time.monotonic() - self._held_since > 60.0:
            self._held_pose = None
            self.get_logger().error("Dropping held goal: dock wake-up took over 60 s")

    def _on_keepout_mask(self, message: OccupancyGrid) -> None:
        self._keepout_mask = message

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _goal_is_blocked(self, pose: PoseStamped) -> bool:
        mask = self._keepout_mask
        if mask is None or mask.info.resolution <= 0.0:
            return True
        origin = mask.info.origin
        q = origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx = pose.pose.position.x - origin.position.x
        dy = pose.pose.position.y - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        cell_x = math.floor(local_x / mask.info.resolution)
        cell_y = math.floor(local_y / mask.info.resolution)
        if not (0 <= cell_x < mask.info.width and 0 <= cell_y < mask.info.height):
            return True
        value = mask.data[cell_y * mask.info.width + cell_x]
        return value < 0 or value >= 50

    def _on_pose(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != "map":
            self.get_logger().error(
                f"Ignoring goal in frame '{pose.header.frame_id}'; expected 'map'"
            )
            return
        if self._keepout_mask is None:
            self.get_logger().error(
                "Ignoring goal because the keepout mask is not available"
            )
            return
        if self._goal_is_blocked(pose):
            self.get_logger().warn(
                "Rejecting goal inside keepout/outside map at (%.2f, %.2f)"
                % (pose.pose.position.x, pose.pose.position.y)
            )
            return
        yaw = _yaw(pose.pose.orientation)
        for name, grid, threshold in (
            ("wall", self._map, self._occupied_threshold),
            ("keepout zone", self._keepout_mask, 50),
        ):
            if grid is None:
                continue
            hit = blocked_footprint_cell(
                grid, self._footprint, pose.pose.position.x, pose.pose.position.y,
                yaw, threshold,
            )
            if hit is not None:
                self.get_logger().warn(
                    "Rejecting goal (%.2f, %.2f, %.0f deg): robot footprint would "
                    "overlap a %s/unknown cell at (%.2f, %.2f)"
                    % (pose.pose.position.x, pose.pose.position.y, math.degrees(yaw),
                       name, hit[0], hit[1])
                )
                return
        if self._sleep_state in ("sleeping", "waking", "anchoring"):
            self._held_pose = pose
            self._held_since = time.monotonic()
            self._wake_publisher.publish(Bool(data=True))
            self.get_logger().info(
                "Holding goal at (%.2f, %.2f, %.0f deg) until the dock wake-up is done (%s)"
                % (pose.pose.position.x, pose.pose.position.y, math.degrees(yaw),
                   self._sleep_state)
            )
            return
        self._forward(pose)

    def _forward(self, pose: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error("Nav2 /navigate_to_pose action is not available")
            return

        yaw = _yaw(pose.pose.orientation)
        self.get_logger().info(
            "Forwarding valid goal at (%.2f, %.2f, %.0f deg)"
            % (pose.pose.position.x, pose.pose.position.y, math.degrees(yaw))
        )

        self._pending_pose = pose
        if self._goal_handle is not None:
            if self._cancel_in_progress:
                return
            self._cancel_in_progress = True
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._cancel_done)
            return
        self._send_pending()

    def _cancel_done(self, _future) -> None:
        self._cancel_in_progress = False
        self._goal_handle = None
        self._send_pending()

    def _send_pending(self) -> None:
        pose = self._pending_pose
        self._pending_pose = None
        if pose is None:
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the Foxglove goal")
            return
        self._goal_handle = handle
        self.get_logger().info("Nav2 accepted the Foxglove goal")
        handle.get_result_async().add_done_callback(
            lambda result, goal_handle=handle: self._goal_result(
                result, goal_handle
            )
        )

    def _goal_result(self, future, goal_handle) -> None:
        status = future.result().status
        labels = {
            GoalStatus.STATUS_SUCCEEDED: "reached",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        result_label = labels.get(status, f"ended ({status})")
        self.get_logger().info(f"Navigation goal {result_label}")
        if self._goal_handle == goal_handle:
            self._goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
