"""Publish fisheye-correct AprilTag poses for the Nav2 docking plugin."""

from __future__ import annotations

import math

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from .dock_tag_geometry import axis_quaternion, solve_tag_pose


class DockTagPose(Node):
    """ID 1 gives the dock position, ID 0 on the wall gives the dock axis.

    Both poses keep the image stamp and camera frame so the docking plugin can
    transform them with TF at exposure time; detection latency is ~0.9 s.
    """

    def __init__(self) -> None:
        super().__init__("bagheera_dock_tag_pose")
        self.declare_parameter("position_tag_id", 1)
        self.declare_parameter("position_tag_size_m", 0.026667)
        self.declare_parameter("axis_tag_id", 0)
        self.declare_parameter("axis_tag_size_m", 0.088889)
        self.declare_parameter("minimum_decision_margin", 15.0)
        self.declare_parameter("minimum_edge_pixels", 16.0)
        self.declare_parameter("minimum_axis_edge_pixels", 60.0)
        self.declare_parameter("maximum_axis_heading_rad", math.radians(30.0))
        self.declare_parameter("camera_frame", "camera_optical_frame")

        self._position_id = int(self.get_parameter("position_tag_id").value)
        self._position_size = float(self.get_parameter("position_tag_size_m").value)
        self._axis_id = int(self.get_parameter("axis_tag_id").value)
        self._axis_size = float(self.get_parameter("axis_tag_size_m").value)
        self._minimum_margin = float(self.get_parameter("minimum_decision_margin").value)
        self._minimum_edge = float(self.get_parameter("minimum_edge_pixels").value)
        self._minimum_axis_edge = float(
            self.get_parameter("minimum_axis_edge_pixels").value
        )
        self._maximum_axis_heading = float(
            self.get_parameter("maximum_axis_heading_rad").value
        )
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._camera_matrix: np.ndarray | None = None
        self._distortion: np.ndarray | None = None

        self._position_publisher = self.create_publisher(
            PoseStamped, "/dock/detected_pose", 5
        )
        self._axis_publisher = self.create_publisher(PoseStamped, "/dock/detected_axis", 5)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_camera_info, 5)
        self.create_subscription(AprilTagDetectionArray, "/dock/tags", self._on_tags, 5)

    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.distortion_model != "equidistant" or len(message.d) < 4:
            return
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0:
            self._camera_matrix = matrix
            self._distortion = np.asarray(message.d[:4], dtype=np.float64)

    def _on_tags(self, message: AprilTagDetectionArray) -> None:
        if self._camera_matrix is None or self._distortion is None:
            return
        for detection in message.detections:
            if detection.id not in (self._position_id, self._axis_id):
                continue
            if detection.hamming != 0 or detection.decision_margin < self._minimum_margin:
                continue
            corners = np.asarray(
                [[point.x, point.y] for point in detection.corners], dtype=np.float64
            )
            size = self._position_size if detection.id == self._position_id else self._axis_size
            pose = solve_tag_pose(corners, self._camera_matrix, self._distortion, size)
            if pose is None or pose.edge_pixels < self._minimum_edge:
                continue

            output = PoseStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = message.header.frame_id or self._camera_frame
            output.pose.position.x, output.pose.position.y, output.pose.position.z = (
                pose.position
            )
            (
                output.pose.orientation.x,
                output.pose.orientation.y,
                output.pose.orientation.z,
                output.pose.orientation.w,
            ) = axis_quaternion(pose.into_tag)

            if detection.id == self._position_id:
                # Only the position is used; ID 1's plane normal is too noisy
                # at the staging distance (several degrees, IPPE flips).
                self._position_publisher.publish(output)
            elif (
                pose.edge_pixels >= self._minimum_axis_edge
                and abs(pose.heading_right) <= self._maximum_axis_heading
            ):
                self._axis_publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockTagPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
