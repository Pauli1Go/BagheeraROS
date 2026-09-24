"""Persist the last map pose and anchor AMCL to the measured dock pose."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
import yaml


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class PosePersistence(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_pose_persistence")
        self.declare_parameter("pose_file", "/bagheera_ws/maps/last_pose.json")
        self.declare_parameter("map_file", "/bagheera_ws/maps/current.yaml")
        self.declare_parameter("dock_x", 1.192)
        self.declare_parameter("dock_y", 1.884)
        self.declare_parameter("dock_yaw", 1.527)
        self._path = Path(str(self.get_parameter("pose_file").value))
        self._map_file = Path(str(self.get_parameter("map_file").value))
        self._dock_pose = (
            float(self.get_parameter("dock_x").value),
            float(self.get_parameter("dock_y").value),
            float(self.get_parameter("dock_yaw").value),
        )
        self._map_hash = self._hash_map()
        self._saved_pose = self._load_pose()
        self._dock_state: bool | None = None
        self._dock_pose_confirmed = False
        self._restoring = False
        self._restore_started = 0.0
        self._last_initialpose = 0.0
        self._last_save = 0.0
        self._last_good: tuple[float, float, float] | None = None
        self._last_good_odom: tuple[float, float, float] | None = None
        self._odom: tuple[float, float, float] | None = None

        dock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(Bool, "/docked", self._on_docked, dock_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_manual_pose, 10
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, 10
        )
        self.create_timer(1.0, self._tick)
        if self._saved_pose is not None:
            self._publish_initialpose(self._saved_pose)

    def _hash_map(self) -> str | None:
        try:
            map_bytes = self._map_file.read_bytes()
            image_name = yaml.safe_load(map_bytes)["image"]
            image_path = (self._map_file.parent / image_name).resolve()
            digest = hashlib.sha256(map_bytes)
            digest.update(image_path.read_bytes())
            return digest.hexdigest()
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            self.get_logger().error(f"Cannot identify current map: {error}")
            return None

    def _load_pose(self) -> tuple[float, float, float] | None:
        if self._map_hash is None:
            return None
        try:
            record = json.loads(self._path.read_text(encoding="utf-8"))
            if record["map_sha256"] != self._map_hash:
                self.get_logger().warn("Saved pose belongs to another map; ignoring it")
                return None
            pose = tuple(float(record[key]) for key in ("x", "y", "yaw"))
            if not all(math.isfinite(value) for value in pose):
                raise ValueError("non-finite pose")
            self.get_logger().info(f"Loaded last map pose from {self._path}")
            return pose
        except FileNotFoundError:
            self.get_logger().info("No saved map pose yet")
            return None
        except (OSError, KeyError, ValueError, TypeError) as error:
            self.get_logger().warn(f"Ignoring invalid saved pose: {error}")
            return None

    def _save_pose(self, pose: tuple[float, float, float]) -> None:
        if self._map_hash is None:
            return
        record = {
            "schema": 1,
            "map_sha256": self._map_hash,
            "x": pose[0],
            "y": pose[1],
            "yaw": pose[2],
            "saved_at_unix": time.time(),
        }
        temporary = self._path.with_name(f".{self._path.name}.{os.getpid()}.tmp")
        try:
            temporary.write_text(json.dumps(record) + "\n", encoding="utf-8")
            os.replace(temporary, self._path)
            self._saved_pose = pose
            self._last_save = time.monotonic()
        except OSError as error:
            self.get_logger().error(f"Cannot save map pose: {error}")

    def _publish_initialpose(self, pose: tuple[float, float, float]) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        message.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        message.pose.covariance[0] = 0.0004
        message.pose.covariance[7] = 0.0004
        message.pose.covariance[35] = 0.0003
        self._initialpose.publish(message)
        self._last_initialpose = time.monotonic()

    def _on_docked(self, message: Bool) -> None:
        docked = bool(message.data)
        if docked == self._dock_state:
            return
        previous = self._dock_state
        self._dock_state = docked
        if docked:
            self._restoring = False
            self._dock_pose_confirmed = False
            self._last_good = self._dock_pose
            self._last_good_odom = self._odom
            self._save_pose(self._dock_pose)
            self._publish_initialpose(self._dock_pose)
            self.get_logger().info("Docked: AMCL anchored to measured dock pose")
        elif previous is None and self._saved_pose is not None:
            self._restoring = True
            self._restore_started = time.monotonic()
            self._last_good = self._saved_pose
            self._last_good_odom = self._odom
            self._publish_initialpose(self._saved_pose)
            self.get_logger().info("Undocked startup: restoring saved map pose")
        elif previous is None:
            self.get_logger().error(
                "Undocked startup without saved pose; set /initialpose manually"
            )
        else:
            self.get_logger().info("Undocked: following AMCL and saving map pose")

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self._odom = (pose.position.x, pose.position.y, _yaw(pose.orientation))
        if self._dock_state and self._last_good_odom is None:
            self._last_good_odom = self._odom

    def _on_manual_pose(self, message: PoseWithCovarianceStamped) -> None:
        if self._dock_state is not False or self._restoring:
            return
        if message.header.frame_id != "map":
            return
        position = message.pose.pose.position
        pose = (position.x, position.y, _yaw(message.pose.pose.orientation))
        if not all(math.isfinite(value) for value in pose):
            return
        self._last_good = pose
        self._last_good_odom = self._odom
        self._save_pose(pose)
        self.get_logger().info("Saved manually supplied /initialpose")

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        candidate = (pose.position.x, pose.position.y, _yaw(pose.orientation))
        if not all(math.isfinite(value) for value in candidate):
            return
        if self._dock_state:
            self._dock_pose_confirmed = (
                math.hypot(
                    candidate[0] - self._dock_pose[0],
                    candidate[1] - self._dock_pose[1],
                ) <= 0.04
                and abs(_angle_delta(candidate[2], self._dock_pose[2]))
                <= math.radians(3.0)
            )
            return
        if self._dock_state is None:
            return
        if self._restoring:
            saved = self._saved_pose
            if saved is not None and (
                math.hypot(candidate[0] - saved[0], candidate[1] - saved[1]) < 0.25
                and abs(_angle_delta(candidate[2], saved[2])) < 0.35
                and time.monotonic() > self._last_initialpose + 0.1
            ):
                self._restoring = False
                self.get_logger().info("Saved initial pose accepted by AMCL")
            else:
                return
        if self._last_good is not None and self._last_good_odom is not None:
            if self._odom is None:
                return
            odom_travel = math.hypot(
                self._odom[0] - self._last_good_odom[0],
                self._odom[1] - self._last_good_odom[1],
            )
            map_travel = math.hypot(
                candidate[0] - self._last_good[0],
                candidate[1] - self._last_good[1],
            )
            odom_turn = abs(_angle_delta(self._odom[2], self._last_good_odom[2]))
            map_turn = abs(_angle_delta(candidate[2], self._last_good[2]))
            if map_travel > odom_travel + 0.50 or map_turn > odom_turn + 0.50:
                self.get_logger().warn(
                    "AMCL pose jump rejected for persistence (not saved)",
                    throttle_duration_sec=5.0,
                )
                return
        self._last_good = candidate
        self._last_good_odom = self._odom
        if time.monotonic() - self._last_save >= 2.0:
            self._save_pose(candidate)

    def _tick(self) -> None:
        now = time.monotonic()
        if self._dock_state is None and self._saved_pose is not None:
            if now - self._last_initialpose >= 1.0:
                self._publish_initialpose(self._saved_pose)
        elif self._dock_state:
            if not self._dock_pose_confirmed and now - self._last_initialpose >= 2.0:
                self._publish_initialpose(self._dock_pose)
        elif self._restoring and self._saved_pose is not None:
            if now - self._restore_started > 15.0:
                self._restoring = False
                self.get_logger().error(
                    "AMCL did not confirm saved pose within 15 s; "
                    "manual localization required"
                )
            elif now - self._last_initialpose >= 1.0:
                self._publish_initialpose(self._saved_pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PosePersistence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
