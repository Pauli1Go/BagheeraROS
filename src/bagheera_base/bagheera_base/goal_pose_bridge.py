from __future__ import annotations

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):
    """Forward Foxglove click-to-publish poses to Nav2's action server."""

    def __init__(self) -> None:
        super().__init__("bagheera_goal_pose_bridge")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._cancel_in_progress = False
        self._pending_pose: PoseStamped | None = None
        self.create_subscription(PoseStamped, "/goal_pose", self._on_pose, 10)
        self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self._on_pose, 10
        )
        self.get_logger().info(
            "Foxglove goals accepted on /goal_pose and /move_base_simple/goal"
        )

    def _on_pose(self, pose: PoseStamped) -> None:
        if pose.header.frame_id != "map":
            self.get_logger().error(
                f"Ignoring goal in frame '{pose.header.frame_id}'; expected 'map'"
            )
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error("Nav2 /navigate_to_pose action is not available")
            return

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
