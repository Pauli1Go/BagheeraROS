from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GoalPoseBridge(Node):
    """Forward Foxglove click-to-publish poses to Nav2's action server."""

    def __init__(self) -> None:
        super().__init__("bagheera_goal_pose_bridge")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._cancel_in_progress = False
        self._pending_pose: PoseStamped | None = None
        self._keepout_mask: OccupancyGrid | None = None
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
        self.get_logger().info(
            "Foxglove goals accepted on /goal_pose and /move_base_simple/goal"
        )

    def _on_keepout_mask(self, message: OccupancyGrid) -> None:
        self._keepout_mask = message

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
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error("Nav2 /navigate_to_pose action is not available")
            return

        self.get_logger().info(
            "Forwarding valid goal at (%.2f, %.2f)"
            % (pose.pose.position.x, pose.pose.position.y)
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
