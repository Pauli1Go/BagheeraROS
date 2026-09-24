"""Stationary, read-only visual check of the AprilTag docking alignment."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
from statistics import median
import time
from types import SimpleNamespace

from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool
import yaml

from .docking_controller import DockingController


CONFIG = Path("/bagheera_ws/install/share/bagheera_base/config/nav2_navigation.yaml")
OUTPUT_ROOT = Path("/bagheera_ws/maps")


def _stamp(message) -> int:
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class DockAlignmentCheck(Node):
    def __init__(self, parameters: dict) -> None:
        super().__init__("bagheera_dock_alignment_check")
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active: bool | None = None
        self.stream_active: bool | None = None
        self.camera_info: CameraInfo | None = None
        self.motion: tuple[float, float] | None = None
        self.frames: deque[CompressedImage] = deque(maxlen=12)
        self.samples: list[dict] = []
        self.best_frame: CompressedImage | None = None
        self._seen_stamps: set[int] = set()
        self._camera_requested = False
        self._params = parameters

        self.create_subscription(Bool, "/dock/active", self._on_active, latched_qos)
        self.create_subscription(
            Bool, "/camera/stream_active", self._on_stream, latched_qos
        )
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_info, sensor_qos
        )
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self._on_frame,
            sensor_qos,
        )
        self.create_subscription(
            AprilTagDetectionArray, "/dock/tags", self._on_tags, sensor_qos
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, sensor_qos
        )
        self._stream_publisher = self.create_publisher(
            Bool, "/camera/stream_enabled", 10
        )

    def _on_active(self, message: Bool) -> None:
        self.active = bool(message.data)

    def _on_stream(self, message: Bool) -> None:
        self.stream_active = bool(message.data)

    def _on_info(self, message: CameraInfo) -> None:
        if len(message.k) == 9 and len(message.d) >= 4 and message.k[0] > 0:
            self.camera_info = message

    def _on_odom(self, message: Odometry) -> None:
        self.motion = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
        )

    def _on_frame(self, message: CompressedImage) -> None:
        if message.data:
            self.frames.append(message)

    def _on_tags(self, message: AprilTagDetectionArray) -> None:
        stamp = _stamp(message)
        if stamp in self._seen_stamps or self.camera_info is None:
            return
        self._seen_stamps.add(stamp)
        if self.active:
            return
        image = min(
            self.frames,
            key=lambda item: abs(_stamp(item) - stamp),
            default=None,
        )
        if image is None or abs(_stamp(image) - stamp) > 150_000_000:
            return
        if not image.data:
            return
        info = self.camera_info
        proxy = SimpleNamespace(
            _camera_matrix=np.asarray(info.k, dtype=np.float64).reshape(3, 3),
            _distortion=np.asarray(info.d[:4], dtype=np.float64),
            _minimum_margin=float(self._params["minimum_decision_margin"]),
            _vision_scale=float(self._params["vision_scale"]),
            _large_heading_reference=float(
                self._params["large_tag_heading_reference_rad"]
            ),
            _small_heading_reference=float(
                self._params["small_tag_heading_reference_rad"]
            ),
            _small_lateral_reference=float(
                self._params["small_tag_lateral_reference_m"]
            ),
            _odom_yaw=None,
        )
        observations = {}
        detections = {}
        for detection in message.detections:
            observation = DockingController._observation_from_detection(
                proxy, detection
            )
            if observation is not None:
                observations[observation.tag_id] = observation
                detections[observation.tag_id] = detection
        if 0 not in observations or 1 not in observations:
            return
        large, small = observations[0], observations[1]
        if not large.pose_valid:
            return
        heading = large.heading
        camera_lateral = (
            small.range_m * math.tan(small.bearing - heading)
            - proxy._small_lateral_reference
        )
        axle_lateral = camera_lateral - float(
            self._params["camera_lever_arm_m"]
        ) * math.sin(heading)
        self.samples.append(
            {
                "stamp_ns": stamp,
                "range_m": small.range_m,
                "bearing_deg": math.degrees(small.bearing),
                "heading_deg": math.degrees(heading),
                "camera_lateral_m": camera_lateral,
                "axle_lateral_m": axle_lateral,
                "small_edge_px": small.edge_pixels,
                "large_edge_px": large.edge_pixels,
                "small_decision_margin": small.decision_margin,
                "large_decision_margin": large.decision_margin,
                "small_center_px": [
                    float(detections[1].centre.x), float(detections[1].centre.y)
                ],
                "camera_principal_px": [float(info.k[2]), float(info.k[5])],
                "large_raw_heading_deg": math.degrees(
                    heading + proxy._large_heading_reference
                ),
                "linear_speed_mps": self.motion[0] if self.motion else None,
                "angular_speed_rps": self.motion[1] if self.motion else None,
            }
        )
        self.best_frame = image

    def request_camera(self) -> None:
        if self.stream_active:
            return
        self._camera_requested = True
        self._stream_publisher.publish(Bool(data=True))

    def release_camera(self) -> None:
        if self._camera_requested:
            self._stream_publisher.publish(Bool(data=False))
            rclpy.spin_once(self, timeout_sec=0.2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))[
        "bagheera_docking_controller"
    ]["ros__parameters"]
    output = args.output or OUTPUT_ROOT / time.strftime(
        "dock_alignment_%Y%m%d_%H%M%S"
    )
    rclpy.init()
    node = DockAlignmentCheck(params)
    try:
        ready_by = time.monotonic() + 3.0
        while node.active is None and time.monotonic() < ready_by:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.active is None:
            raise RuntimeError("/dock/active unavailable; refusing to control camera")
        if node.active:
            raise RuntimeError("docking is active; stop it before the still-frame check")
        node.request_camera()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and len(node.samples) < 7:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.active:
                raise RuntimeError("docking started during the check")
        if not node.samples:
            raise RuntimeError("no synchronized frame with both valid dock tags")
        samples = node.samples[-7:]
        headings = [item["heading_deg"] for item in samples]
        laterals = [item["axle_lateral_m"] for item in samples]
        stationary = (
            all(
                item["linear_speed_mps"] is not None
                and abs(item["linear_speed_mps"]) <= 0.01
                and abs(item["angular_speed_rps"]) <= 0.03
                for item in samples
            )
        )
        stable = (
            len(samples) >= 5
            and max(headings) - min(headings) <= 5.0
            and max(laterals) - min(laterals) <= 0.06
        )
        heading = median(headings)
        lateral = median(laterals)
        axis_ready = (
            stationary and stable
            and abs(heading) <= math.degrees(params["axis_heading_tolerance_rad"])
            and abs(lateral) <= params["axis_lateral_tolerance_m"]
        )
        report = {
            "axis_ready_at_current_pose": axis_ready,
            "stationary": stationary,
            "measurements_stable": stable,
            "sample_count": len(samples),
            "range_m": median(item["range_m"] for item in samples),
            "small_tag_bearing_deg": median(item["bearing_deg"] for item in samples),
            "dock_heading_deg": heading,
            "camera_lateral_m": median(item["camera_lateral_m"] for item in samples),
            "axle_lateral_m": lateral,
            "heading_spread_deg": max(headings) - min(headings),
            "axle_lateral_spread_m": max(laterals) - min(laterals),
            "limits": {
                "axis_heading_deg": math.degrees(params["axis_heading_tolerance_rad"]),
                "axis_lateral_m": params["axis_lateral_tolerance_m"],
                "straight_gate_lateral_m": params["straight_gate_lateral_tolerance_m"],
            },
            "successful_manual_recording_reference_at_0_87m": {
                "small_tag_bearing_deg": 0.43,
                "large_raw_heading_deg": 2.7,
            },
            "samples": samples,
        }
        output.mkdir(parents=True, exist_ok=False)
        (output / "frame.jpg").write_bytes(bytes(node.best_frame.data))
        (output / "report.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        print(f"Bild und Auswertung: {output}")
        print(
            f"Distanz {report['range_m']:.2f} m | Tag-Bearing "
            f"{report['small_tag_bearing_deg']:+.1f}° | Dock-Winkel "
            f"{heading:+.1f}° | Achsversatz {lateral:+.3f} m"
        )
        print(
            "Referenz des erfolgreichen Hand-Dockings bei 0.87 m: "
            "Bearing +0.4°, großer Tag Rohwinkel +2.7°"
        )
        print(
            "Achs-Check: "
            + ("PASS" if axis_ready else "FAIL / nicht sicher messbar")
        )
        if not stationary:
            print("Hinweis: Roboter war nicht nachweislich im Stillstand.")
        if not stable:
            print("Hinweis: Tagschwankung zu groß oder zu wenige Messungen.")
    finally:
        node.release_camera()
        node.destroy_node()
        rclpy.try_shutdown()
