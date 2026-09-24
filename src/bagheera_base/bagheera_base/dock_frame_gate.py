"""Feed apriltag_ros one newest camera frame at a time.

apriltag_ros subscribes with a fixed sensor-data queue of five frames. The
Pi 4 detects ~5 frames/s from the camera's JPEG stream, so that queue stays
full and every detection is computed on a frame ~0.8 s old. This gate forwards
the newest JPEG only after the previous one has produced a /dock/tags result,
so nothing waits in a queue. It does not decode images.

camera_ros encodes JPEG only while the topic has a subscriber. The gate
therefore subscribes only while /dock/vision_enabled is true; a camera running
for the Foxglove video alone then encodes no JPEG.
"""

from __future__ import annotations

import copy
import time

from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool


class DockFrameGate(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_dock_frame_gate")
        self.declare_parameter("result_timeout_s", 1.0)
        self._timeout = float(self.get_parameter("result_timeout_s").value)

        self._latest_only = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # Reliable output also matches apriltag_ros' reliable CameraInfo
        # subscription; a best-effort publisher delivered no CameraInfo.
        latest_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._image_publisher = self.create_publisher(
            CompressedImage, "/dock/camera/image_raw/compressed", latest_reliable
        )
        self._info_publisher = self.create_publisher(
            CameraInfo, "/dock/camera/camera_info", latest_reliable
        )
        self._image_subscription = None
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/dock/vision_enabled", self._on_vision, latched)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_info, 5)
        self.create_subscription(AprilTagDetectionArray, "/dock/tags", self._on_result, 5)
        self.create_timer(0.05, self._on_timer)

        self._info: CameraInfo | None = None
        self._pending: CompressedImage | None = None
        self._busy_since: float | None = None

    def _on_vision(self, message: Bool) -> None:
        if message.data and self._image_subscription is None:
            self._image_subscription = self.create_subscription(
                CompressedImage,
                "/camera/image_raw/compressed",
                self._on_image,
                self._latest_only,
            )
        elif not message.data and self._image_subscription is not None:
            self.destroy_subscription(self._image_subscription)
            self._image_subscription = None
            self._pending = None
            self._busy_since = None

    def _on_info(self, message: CameraInfo) -> None:
        self._info = message

    def _on_image(self, message: CompressedImage) -> None:
        self._pending = message
        self._forward()

    def _on_result(self, _message: AprilTagDetectionArray) -> None:
        self._busy_since = None
        self._forward()

    def _on_timer(self) -> None:
        # A frame without any quad still produces an (empty) result; the
        # timeout only covers a detector restart or a dropped frame.
        if self._busy_since is not None and time.monotonic() - self._busy_since > self._timeout:
            self._busy_since = None
            self._forward()

    def _forward(self) -> None:
        if self._busy_since is not None or self._pending is None or self._info is None:
            return
        image = self._pending
        self._pending = None
        # apriltag_ros pairs image and CameraInfo by identical stamps.
        info = copy.deepcopy(self._info)
        info.header = image.header
        self._info_publisher.publish(info)
        self._image_publisher.publish(image)
        self._busy_since = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockFrameGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
