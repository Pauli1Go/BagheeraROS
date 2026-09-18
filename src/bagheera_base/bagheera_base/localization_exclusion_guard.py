"""Reject AMCL pose jumps into user-defined impossible regions."""

from __future__ import annotations

import copy
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _yaw(pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class LocalizationExclusionGuard(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_localization_exclusion_guard")
        self.declare_parameter("mask_topic", "/localization_exclusion_mask")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("reset_cooldown_s", 1.0)
        self._threshold = int(self.get_parameter("occupied_threshold").value)
        self._cooldown = float(self.get_parameter("reset_cooldown_s").value)
        self._mask: OccupancyGrid | None = None
        self._last_valid: PoseWithCovarianceStamped | None = None
        self._odom: Odometry | None = None
        self._last_valid_odom: tuple[float, float, float] | None = None
        self._last_reset = 0.0
        self._violation: bool | None = None

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._violation_publisher = self.create_publisher(
            Bool, "/localization_exclusion_violation", latched_qos
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("mask_topic").value),
            self._on_mask,
            latched_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 20
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, 20
        )

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message

    def _set_violation(self, value: bool) -> None:
        if value != self._violation:
            self._violation = value
            self._violation_publisher.publish(Bool(data=value))

    def _on_mask(self, message: OccupancyGrid) -> None:
        self._mask = message
        self.get_logger().info(
            "Localization exclusion mask loaded: %d x %d"
            % (message.info.width, message.info.height)
        )

    def _excluded(self, x: float, y: float) -> bool:
        mask = self._mask
        if mask is None or mask.info.resolution <= 0.0:
            return False
        origin = mask.info.origin
        q = origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx = x - origin.position.x
        dy = y - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        cell_x = math.floor(local_x / mask.info.resolution)
        cell_y = math.floor(local_y / mask.info.resolution)
        if not (0 <= cell_x < mask.info.width and 0 <= cell_y < mask.info.height):
            return True
        value = mask.data[cell_y * mask.info.width + cell_x]
        return value < 0 or value >= self._threshold

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        if not self._excluded(position.x, position.y):
            self._last_valid = message
            if self._odom is not None:
                odom_pose = self._odom.pose.pose
                self._last_valid_odom = (
                    odom_pose.position.x,
                    odom_pose.position.y,
                    _yaw(odom_pose),
                )
            self._set_violation(False)
            return
        now = time.monotonic()
        predicted = self._odometry_predicted_pose()
        if predicted is None:
            self._set_violation(True)
            return
        predicted_position = predicted.pose.pose.position
        if self._excluded(predicted_position.x, predicted_position.y):
            # Odometry says the chassis itself entered the forbidden region.
            # Resetting AMCL would now falsify the physical position; stop
            # autonomous commands and require a new valid pose instead.
            self._set_violation(True)
            self.get_logger().error(
                "Robot odometry entered a localization-exclusion zone; "
                "not resetting AMCL to a false pose"
            )
            return
        self._set_violation(False)
        if now - self._last_reset < self._cooldown:
            return
        restored = predicted
        # Do not let a single previous overconfident solution collapse the
        # reset cloud to one point.
        restored.pose.covariance[0] = max(restored.pose.covariance[0], 0.04)
        restored.pose.covariance[7] = max(restored.pose.covariance[7], 0.04)
        restored.pose.covariance[35] = max(restored.pose.covariance[35], 0.03)
        self._initial_pose.publish(restored)
        self._last_reset = now
        self.get_logger().warn(
            "Rejected AMCL pose in localization-exclusion zone; "
            "restoring odometry-projected valid pose"
        )

    def _odometry_predicted_pose(self) -> PoseWithCovarianceStamped | None:
        if (
            self._last_valid is None
            or self._last_valid_odom is None
            or self._odom is None
        ):
            return None
        old_x, old_y, old_yaw = self._last_valid_odom
        current = self._odom.pose.pose
        current_yaw = _yaw(current)
        dx = current.position.x - old_x
        dy = current.position.y - old_y
        local_x = math.cos(old_yaw) * dx + math.sin(old_yaw) * dy
        local_y = -math.sin(old_yaw) * dx + math.cos(old_yaw) * dy
        valid_pose = self._last_valid.pose.pose
        map_yaw = _yaw(valid_pose)
        predicted = PoseWithCovarianceStamped()
        predicted.header.stamp = self.get_clock().now().to_msg()
        predicted.header.frame_id = "map"
        predicted.pose = copy.deepcopy(self._last_valid.pose)
        predicted.pose.pose.position.x = (
            valid_pose.position.x
            + math.cos(map_yaw) * local_x
            - math.sin(map_yaw) * local_y
        )
        predicted.pose.pose.position.y = (
            valid_pose.position.y
            + math.sin(map_yaw) * local_x
            + math.cos(map_yaw) * local_y
        )
        yaw = map_yaw + math.atan2(
            math.sin(current_yaw - old_yaw), math.cos(current_yaw - old_yaw)
        )
        predicted.pose.pose.orientation.x = 0.0
        predicted.pose.pose.orientation.y = 0.0
        predicted.pose.pose.orientation.z = math.sin(yaw / 2.0)
        predicted.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return predicted


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationExclusionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
