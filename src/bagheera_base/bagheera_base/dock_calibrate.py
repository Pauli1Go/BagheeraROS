"""Calibrate the dock plugin's tag offsets against the real docked pose.

No motor command is sent. Procedure:

1. Start with the robot charging in the dock. The averaged odom pose of
   base_link is the ground-truth dock pose.
2. Reverse out with the game controller (0.3-0.8 m, any angle with both tags
   in view) and stop. Each stationary stop with both tags visible yields one
   sample of external_detection_translation_x/y and axis_yaw_offset.
3. Repeat from a few positions, then press Ctrl-C for the averages.

Odom must stay continuous between steps 1 and 2. In practice the wheels slip
while leaving the dock, which corrupts translation_x by several centimetres;
translation_y and axis_yaw_offset are the trustworthy outputs, as long as the
samples agree with each other (a common jump means an odom yaw jump).
"""

from __future__ import annotations

from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped transforms


OUTPUT_ROOT = Path("/bagheera_ws/maps")
FIXED_FRAME = "odom"


def _yaw(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _circular_mean(values: list[float]) -> float:
    return math.atan2(sum(map(math.sin, values)), sum(map(math.cos, values)))


def offsets_from_sample(
    dock: tuple[float, float, float],
    tag: tuple[float, float],
    axis_yaw: float,
) -> tuple[float, float, float]:
    """Plugin parameters that map this tag sample onto the true dock pose.

    dock is the docked base_link pose, tag the ID 1 centre and axis_yaw the raw
    ID 0 direction, all in the same fixed frame. Returns (translation_x,
    translation_y, axis_yaw_offset) in the plugin's conventions.
    """
    dock_x, dock_y, dock_yaw = dock
    dx = dock_x - tag[0]
    dy = dock_y - tag[1]
    translation_x = math.cos(dock_yaw) * dx + math.sin(dock_yaw) * dy
    translation_y = -math.sin(dock_yaw) * dx + math.cos(dock_yaw) * dy
    return translation_x, translation_y, _angle(dock_yaw - axis_yaw)


class DockCalibrate(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_dock_calibrate")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._vision = self.create_publisher(Bool, "/dock/vision_enabled", latched)
        self.create_subscription(Bool, "/docked", self._on_docked, latched)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 20)
        self.create_subscription(PoseStamped, "/dock/detected_pose", self._on_tag, 10)
        self.create_subscription(PoseStamped, "/dock/detected_axis", self._on_axis, 10)
        self.docked: bool | None = None
        self.moving_since = time.monotonic()
        self.still_since: float | None = None
        self.tags: deque[tuple[float, float, float]] = deque(maxlen=60)
        self.axes: deque[tuple[float, float]] = deque(maxlen=60)

    def _on_docked(self, message: Bool) -> None:
        self.docked = bool(message.data)

    def _on_odom(self, message: Odometry) -> None:
        twist = message.twist.twist
        moving = abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.02
        now = time.monotonic()
        if moving:
            self.moving_since = now
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def _to_fixed(self, message: PoseStamped) -> PoseStamped | None:
        try:
            return self._tf.transform(message, FIXED_FRAME, timeout=Duration(seconds=0.3))
        except Exception:  # noqa: BLE001 - TF raises several exception types
            return None

    def _on_tag(self, message: PoseStamped) -> None:
        fixed = self._to_fixed(message)
        if fixed is not None:
            self.tags.append((time.monotonic(), fixed.pose.position.x, fixed.pose.position.y))

    def _on_axis(self, message: PoseStamped) -> None:
        fixed = self._to_fixed(message)
        if fixed is not None:
            self.axes.append((time.monotonic(), _yaw(fixed.pose.orientation)))

    def robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf.lookup_transform(FIXED_FRAME, "base_link", Time())
        except Exception:  # noqa: BLE001
            return None
        t = transform.transform
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_camera(self, enabled: bool) -> None:
        self._vision.publish(Bool(data=enabled))


def _measure_dock(node: DockCalibrate) -> tuple[float, float, float]:
    print("Waiting for /docked ...", flush=True)
    while node.docked is None:
        node.spin_for(0.2)
    if not node.docked:
        raise SystemExit("Robot is not docked. Put it on the charger first.")
    samples = []
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        node.spin_for(0.1)
        pose = node.robot_pose()
        if pose is not None:
            samples.append(pose)
    if len(samples) < 10:
        raise SystemExit("No odom -> base_link transform.")
    dock = (
        sum(p[0] for p in samples) / len(samples),
        sum(p[1] for p in samples) / len(samples),
        _circular_mean([p[2] for p in samples]),
    )
    print(
        f"Docked pose in {FIXED_FRAME}: x={dock[0]:.4f} y={dock[1]:.4f} "
        f"yaw={math.degrees(dock[2]):.2f} deg",
        flush=True,
    )
    return dock


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockCalibrate()
    results: list[dict] = []
    dock: tuple[float, float, float] | None = None
    try:
        node.spin_for(1.0)
        dock = _measure_dock(node)
        print(
            "Now reverse out of the dock with the controller and stop with both "
            "tags in view. Each stop gives one sample; Ctrl-C to finish.",
            flush=True,
        )
        node.set_camera(True)
        waiting_for_motion = True
        while True:
            node.spin_for(0.2)
            now = time.monotonic()
            if node.docked:
                continue
            if node.still_since is None:
                waiting_for_motion = False
                continue
            if waiting_for_motion or now - node.still_since < 2.0:
                continue
            # Only detections taken after the robot came to rest.
            tags = [t for t in node.tags if t[0] >= node.still_since + 1.0]
            axes = [a for a in node.axes if a[0] >= node.still_since + 1.0]
            if len(tags) < 5 or len(axes) < 5:
                if now - node.still_since > 8.0:
                    print(
                        f"  no sample: ID 1 {len(tags)}, ID 0 {len(axes)} detections "
                        "(both tags must be visible)",
                        flush=True,
                    )
                    waiting_for_motion = True
                continue
            tag = (
                sum(t[1] for t in tags) / len(tags),
                sum(t[2] for t in tags) / len(tags),
            )
            axis = _circular_mean([a[1] for a in axes])
            robot = node.robot_pose()
            tx, ty, yaw_offset = offsets_from_sample(dock, tag, axis)
            distance = math.hypot(robot[0] - dock[0], robot[1] - dock[1]) if robot else float("nan")
            result = {
                "robot_distance_m": distance,
                "robot_yaw_to_dock_deg": math.degrees(_angle(robot[2] - dock[2])) if robot else None,
                "translation_x": tx,
                "translation_y": ty,
                "axis_yaw_offset": yaw_offset,
                "id1_samples": len(tags),
                "id0_samples": len(axes),
            }
            results.append(result)
            print(
                f"  sample {len(results)} at {distance:.2f} m: "
                f"translation_x={tx:+.4f} translation_y={ty:+.4f} "
                f"axis_yaw_offset={yaw_offset:+.4f} rad ({math.degrees(yaw_offset):+.2f} deg)",
                flush=True,
            )
            waiting_for_motion = True
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.set_camera(False)
            node.spin_for(0.3)
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - second Ctrl-C
            pass
        if results and dock is not None:
            tx = sum(r["translation_x"] for r in results) / len(results)
            ty = sum(r["translation_y"] for r in results) / len(results)
            yaw = _circular_mean([r["axis_yaw_offset"] for r in results])
            spread = (
                max(r["translation_y"] for r in results) - min(r["translation_y"] for r in results)
            )
            print(
                "\nAverage over "
                f"{len(results)} samples (translation_y spread {spread * 100:.1f} cm):\n"
                f"      external_detection_translation_x: {tx:.4f}\n"
                f"      external_detection_translation_y: {ty:.4f}\n"
                f"      axis_yaw_offset: {yaw:.5f}  # {math.degrees(yaw):+.2f} deg",
                flush=True,
            )
            output = OUTPUT_ROOT / f"dock_calibration_{datetime.now():%Y%m%d_%H%M%S}.json"
            try:
                output.write_text(json.dumps(
                    {"docked_pose_odom": dock, "samples": results,
                     "average": {"translation_x": tx, "translation_y": ty,
                                 "axis_yaw_offset": yaw}},
                    indent=2,
                ))
                print(f"Saved {output}", flush=True)
            except OSError as error:
                print(f"Could not save {output}: {error}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
