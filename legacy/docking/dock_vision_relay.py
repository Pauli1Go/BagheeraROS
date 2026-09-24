"""Isolated, on-demand camera transport for AprilTag docking."""

from __future__ import annotations

import copy

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool


class DockVisionRelay(Node):
    """Keep JPEG decoding away from the docking motion executor."""

    def __init__(self) -> None:
        super().__init__("bagheera_dock_vision_relay")
        self.declare_parameter("vision_scale", 0.5)
        self._scale = float(self.get_parameter("vision_scale").value)
        if not 0.0 < self._scale <= 1.0:
            raise ValueError("vision_scale must be in (0, 1]")

        self._camera_info: CameraInfo | None = None
        self._image_subscription = None
        self._received = 0
        self._relayed = 0
        self._source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        enabled_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._image_publisher = self.create_publisher(
            Image, "/dock/camera/image_raw", image_qos
        )
        self._info_publisher = self.create_publisher(
            CameraInfo, "/dock/camera/camera_info", 10
        )
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, 10
        )
        self.create_subscription(
            Bool, "/dock/vision_enabled", self._on_enabled, enabled_qos
        )
        self.create_timer(5.0, self._report_rate)

    def _on_camera_info(self, message: CameraInfo) -> None:
        if (
            message.width > 0
            and message.height > 0
            and len(message.k) == 9
            and message.k[0] > 0.0
            and message.k[4] > 0.0
            and len(message.d) >= 4
        ):
            self._camera_info = copy.deepcopy(message)

    def _on_enabled(self, message: Bool) -> None:
        if message.data and self._image_subscription is None:
            self._image_subscription = self.create_subscription(
                CompressedImage,
                "/camera/image_raw/compressed",
                self._on_image,
                self._source_qos,
            )
            self.get_logger().info("Dock camera relay enabled")
        elif not message.data and self._image_subscription is not None:
            self.destroy_subscription(self._image_subscription)
            self._image_subscription = None
            self.get_logger().info("Dock camera relay disabled")

    def _on_image(self, message: CompressedImage) -> None:
        self._received += 1
        if self._camera_info is None:
            return
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        if self._scale < 1.0:
            gray = cv2.resize(
                gray,
                None,
                fx=self._scale,
                fy=self._scale,
                interpolation=cv2.INTER_AREA,
            )

        image = Image()
        image.header = message.header
        image.height, image.width = gray.shape
        image.encoding = "mono8"
        image.is_bigendian = False
        image.step = image.width
        image.data = gray.tobytes()

        info = copy.deepcopy(self._camera_info)
        scale_x = image.width / info.width
        scale_y = image.height / info.height
        info.header = image.header
        info.width = image.width
        info.height = image.height
        info.k = [
            value * (scale_x if index // 3 == 0 else scale_y)
            if index // 3 < 2 else value
            for index, value in enumerate(info.k)
        ]
        info.p = [
            value * (scale_x if index // 4 == 0 else scale_y)
            if index // 4 < 2 else value
            for index, value in enumerate(info.p)
        ]
        self._info_publisher.publish(info)
        self._image_publisher.publish(image)
        self._relayed += 1

    def _report_rate(self) -> None:
        if self._image_subscription is not None:
            self.get_logger().info(
                f"Dock camera 5 s: source={self._received}, relayed={self._relayed}"
            )
        self._received = 0
        self._relayed = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockVisionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
