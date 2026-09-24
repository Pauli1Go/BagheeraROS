"""Run the camera only while someone watches it or a docking attempt needs it."""

from __future__ import annotations

import subprocess
import time

from foxglove_msgs.msg import CompressedVideo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


VIDEO_TOPIC = "/camera/h264"


class CameraManager(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_camera_manager")
        self.declare_parameter("camera_executable", "/bagheera_ws/install/lib/camera_ros/camera_node")
        self.declare_parameter(
            "camera_parameters", "/bagheera_ws/install/share/bagheera_base/config/sensors.yaml"
        )
        self.declare_parameter("start_enabled", False)
        # Only this node's subscription to the video counts as a viewer.
        self.declare_parameter("viewer_node", "foxglove_bridge")
        # Keep the camera running briefly after the last viewer leaves, so a
        # Foxglove tab or layout switch does not restart it.
        self.declare_parameter("viewer_linger_s", 20.0)
        self._user_enabled = bool(self.get_parameter("start_enabled").value)
        self._viewer_node = str(self.get_parameter("viewer_node").value)
        self._viewer_linger = float(self.get_parameter("viewer_linger_s").value)
        self._dock_enabled = False
        self._sleeping = False
        self._last_viewer = float("-inf")
        self._process: subprocess.Popen | None = None
        self._last_start = 0.0

        dock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/dock/vision_enabled", self._on_dock, dock_qos)
        self.create_subscription(Bool, "/camera/stream_enabled", self._on_user, 10)
        self.create_subscription(String, "/dock/sleep_state", self._on_sleep_state, dock_qos)
        self._active_publisher = self.create_publisher(
            Bool, "/camera/stream_active", dock_qos
        )
        # Keep both topics discoverable in Foxglove while the camera process
        # is asleep; a Foxglove subscription to the video then wakes it. No
        # frames or CameraInfo are fabricated. QoS matches camera_ros.
        self._jpeg_advertisement = self.create_publisher(
            CompressedImage,
            "/camera/image_raw/compressed",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._video_advertisement = self.create_publisher(
            CompressedVideo,
            VIDEO_TOPIC,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_timer(0.5, self._tick)
        self._active_publisher.publish(Bool(data=False))

    def _on_dock(self, message: Bool) -> None:
        self._dock_enabled = bool(message.data)
        self._tick()

    def _on_sleep_state(self, message: String) -> None:
        self._sleeping = message.data == "sleeping"
        self._tick()

    def _on_user(self, message: Bool) -> None:
        self._user_enabled = bool(message.data)
        self._tick()

    def _viewer_wants_video(self) -> bool:
        now = time.monotonic()
        if any(
            info.node_name == self._viewer_node
            for info in self.get_subscriptions_info_by_topic(VIDEO_TOPIC)
        ):
            self._last_viewer = now
        return now - self._last_viewer < self._viewer_linger

    def _stop_camera(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        self._active_publisher.publish(Bool(data=False))
        self.get_logger().info("Camera stopped; no viewer, stream request or docking")

    def _tick(self) -> None:
        reasons = [
            name
            for name, wanted in (
                ("docking", self._dock_enabled),
                ("stream request", self._user_enabled),
                ("Foxglove viewer", self._viewer_wants_video()),
            )
            if wanted
        ]
        if not reasons or self._sleeping:
            # Asleep in the dock: not even a Foxglove viewer wakes the camera.
            self._stop_camera()
            return
        if self._process is not None and self._process.poll() is None:
            return
        if time.monotonic() - self._last_start < 2.0:
            return
        if self._process is not None:
            self.get_logger().error(
                f"Camera exited with code {self._process.returncode}; restarting"
            )
        command = [
            str(self.get_parameter("camera_executable").value),
            "--ros-args",
            "-r", "__node:=camera",
            "--params-file", str(self.get_parameter("camera_parameters").value),
            "-r", "/camera/camera_info:=/camera/camera_info_raw",
        ]
        self._last_start = time.monotonic()
        try:
            self._process = subprocess.Popen(command)
        except OSError as error:
            self.get_logger().error(f"Cannot start camera: {error}")
            return
        self._active_publisher.publish(Bool(data=True))
        self.get_logger().info(f"Camera started for {', '.join(reasons)}")

    def destroy_node(self) -> bool:
        self._stop_camera()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
