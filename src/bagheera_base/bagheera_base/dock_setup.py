"""Measure a (moved) charging dock and write maps/dock.yaml.

Procedure (docs/docking.md, "Moving the dock"):

1. Unplug the dock's power supply. Without charge voltage there is no
   /docked, so bagheera_pose_persistence does not pin AMCL to the old dock
   pose.
2. Localize the robot (Foxglove 2D pose estimate if needed) and put it onto
   the dock by hand or with the game controller until the pins touch.
3. Run this tool with --execute. It
   - asks for the undock manoeuvre (reverse distance, turn),
   - refines AMCL on the spot and averages the docked map and odom pose,
   - reverses straight (slowly, heading hold) until both tags are in view,
   - measures ID 1 and ID 0 while standing still,
   - computes the dock pose: x/y from AMCL on the dock, yaw from AMCL out
     there carried back by odometry (AMCL's heading is poor right at the
     dock), and the wall tag's axis_yaw_offset,
   - checks ID 1's offsets against nav2_navigation.yaml,
   - writes maps/dock.yaml (the old file is kept as a backup) and a report.
4. Plug the dock in again, `docker compose restart`, test a docking run.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped transforms
import yaml

from .dock_setup_math import (
    check_undock,
    circular_mean,
    docked_map_pose_from_now,
    mean_pose,
    offsets_from_sample,
    staging_map_pose,
)
from .dock_site import SITE_FILE, DockSite, dump_dock_site, load_dock_site
from .pose_math import normalize_angle

MAPS = Path("/bagheera_ws/maps")


def _yaw(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockSetup(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_dock_setup")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.command = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self._vision = self.create_publisher(Bool, "/dock/vision_enabled", latched)
        self._nomotion = self.create_client(Empty, "/request_nomotion_update")
        self.create_subscription(Bool, "/docked", self._on_docked, latched)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )
        self.create_subscription(PoseStamped, "/dock/detected_pose", self._on_tag, 10)
        self.create_subscription(PoseStamped, "/dock/detected_axis", self._on_axis, 10)
        self.docked: bool | None = None
        self.amcl: PoseWithCovarianceStamped | None = None
        self.amcl_count = 0
        self.tags: list[tuple[float, float, float]] = []
        self.axes: list[tuple[float, float]] = []

    # -- inputs ---------------------------------------------------------------

    def _on_docked(self, message: Bool) -> None:
        self.docked = bool(message.data)

    def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
        self.amcl = message
        self.amcl_count += 1

    def _to_odom(self, message: PoseStamped) -> PoseStamped | None:
        try:
            return self._tf.transform(message, "odom", timeout=Duration(seconds=0.3))
        except Exception:  # noqa: BLE001 - TF raises several exception types
            return None

    def _on_tag(self, message: PoseStamped) -> None:
        pose = self._to_odom(message)
        if pose is not None:
            self.tags.append((time.monotonic(), pose.pose.position.x, pose.pose.position.y))

    def _on_axis(self, message: PoseStamped) -> None:
        pose = self._to_odom(message)
        if pose is not None:
            self.axes.append((time.monotonic(), _yaw(pose.pose.orientation)))

    # -- helpers --------------------------------------------------------------

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, frame: str) -> tuple[float, float, float] | None:
        try:
            transform = self._tf.lookup_transform(frame, "base_link", Time())
        except Exception:  # noqa: BLE001
            return None
        t = transform.transform
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def average_poses(self, seconds: float) -> tuple[tuple, tuple]:
        """Mean map and odom pose of base_link over a still period."""
        map_samples, odom_samples = [], []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin_for(0.1)
            map_pose = self.pose("map")
            odom_pose = self.pose("odom")
            if map_pose is not None and odom_pose is not None:
                map_samples.append(map_pose)
                odom_samples.append(odom_pose)
        if len(map_samples) < 5:
            raise RuntimeError("no map/odom -> base_link transform (localization running?)")
        return mean_pose(map_samples), mean_pose(odom_samples)

    def refine_amcl(self) -> tuple[float, float]:
        """Let AMCL update without motion; returns (xy std [m], yaw std [rad])."""
        if not self._nomotion.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("AMCL's /request_nomotion_update is not available")
        start = self.amcl_count
        for _ in range(8):
            self._nomotion.call_async(Empty.Request())
            self.spin_for(0.6)
        if self.amcl is None or self.amcl_count == start:
            raise RuntimeError("AMCL published no pose; is the robot localized?")
        covariance = self.amcl.pose.covariance
        return math.sqrt(max(covariance[0], covariance[7])), math.sqrt(covariance[35])

    def drive(self, linear: float = 0.0, angular: float = 0.0) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.command.publish(message)

    def stop(self) -> None:
        for _ in range(5):
            self.drive()
            self.spin_for(0.05)

    def set_camera(self, enabled: bool) -> None:
        self._vision.publish(Bool(data=enabled))

    def reverse(self, distance: float, speed: float, timeout: float) -> float:
        """Drive straight back by distance with odometry heading hold."""
        start = self.pose("odom")
        if start is None:
            raise RuntimeError("no odom -> base_link transform")
        started = time.monotonic()
        travelled = 0.0
        while travelled < distance:
            if time.monotonic() - started > timeout:
                raise RuntimeError(f"reverse timed out after {travelled:.2f} m")
            if self.docked:
                raise RuntimeError("/docked became true: is the dock powered?")
            now = self.pose("odom")
            if now is None:
                raise RuntimeError("odometry lost while reversing")
            travelled = math.hypot(now[0] - start[0], now[1] - start[1])
            heading_error = normalize_angle(start[2] - now[2])
            angular = max(-0.2, min(0.2, 1.5 * heading_error))
            self.drive(-speed, angular)
            self.spin_for(0.05)
        self.stop()
        return travelled


def _ask(prompt: str, default: float) -> float:
    while True:
        answer = input(f"{prompt} [{default:g}]: ").strip()
        if not answer:
            return default
        try:
            return float(answer.replace(",", "."))
        except ValueError:
            print("  please enter a number")


def _ask_undock(args, current: DockSite) -> tuple[float, float]:
    reverse = args.undock_reverse
    turn = args.undock_turn_deg
    if reverse is None or turn is None:
        print(
            "\nUndocking: the robot reverses straight out of the dock, then turns on\n"
            "the spot (+ left, - right, 0 = no turn, Nav2 turns onto its path).\n"
            "Pick values that leave it in free space at this dock.",
            flush=True,
        )
    if reverse is None:
        reverse = current.reverse_distance_m if args.yes else _ask(
            "Reverse distance in m", current.reverse_distance_m)
    if turn is None:
        default_turn = round(math.degrees(current.turn_angle_rad), 1)
        turn = default_turn if args.yes else _ask("Turn after reversing in deg", default_turn)
    check_undock(reverse, turn)
    return reverse, turn


def _confirm(prompt: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    return input(f"{prompt} [Y/n]: ").strip().lower() in ("", "y", "yes", "j", "ja")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        prog="bagheera_dock_setup",
        description="Measure a moved dock from the unpowered dock and write maps/dock.yaml.",
    )
    parser.add_argument("--execute", action="store_true",
                        help="required: the robot reverses out of the dock by itself")
    parser.add_argument("--measure-distance", type=float, default=0.60,
                        help="reverse this far (m) to see both tags (default 0.60)")
    parser.add_argument("--speed", type=float, default=0.05, help="reverse speed in m/s")
    parser.add_argument("--undock-reverse", type=float, default=None,
                        help="undock reverse distance in m (asked if omitted)")
    parser.add_argument("--undock-turn-deg", type=float, default=None,
                        help="undock turn in deg, + left, - right, 0 none (asked if omitted)")
    parser.add_argument("--yes", action="store_true",
                        help="no questions: keep current undock values unless given, write the file")
    parser.add_argument("--site-file", default=SITE_FILE)
    parser.add_argument("--max-xy-std", type=float, default=0.10,
                        help="abort if AMCL's position std is larger (m)")
    parser.add_argument("--countdown", type=int, default=3)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("the robot drives backwards by itself; confirm with --execute")
    if not 0.2 <= args.measure_distance <= 1.2:
        parser.error("--measure-distance must be between 0.2 and 1.2 m")
    if not 0.02 <= args.speed <= 0.10:
        parser.error("--speed must be between 0.02 and 0.10 m/s")

    nav2_config = str(
        Path(get_package_share_directory("bagheera_base")) / "config" / "nav2_navigation.yaml"
    )
    with open(nav2_config, encoding="utf-8") as handle:
        docking = yaml.safe_load(handle)["docking_server"]["ros__parameters"]
    plugin = docking[docking["dock_plugins"][0]]
    current = load_dock_site(nav2_config, args.site_file)

    rclpy.init()
    node = DockSetup()
    report: dict = {"started": datetime.now().isoformat(timespec="seconds")}
    try:
        node.spin_for(1.5)
        while node.docked is None:
            print("Waiting for /docked ...", flush=True)
            node.spin_for(1.0)
        if node.docked:
            raise SystemExit(
                "The robot reports /docked: the dock is powered. Unplug the dock's\n"
                "power supply first, otherwise the dock lock pins AMCL to the old pose."
            )
        reverse, turn = _ask_undock(args, current)

        print("\nRefining AMCL on the spot ...", flush=True)
        xy_std, yaw_std = node.refine_amcl()
        print(f"  AMCL std: {xy_std * 100:.1f} cm, {math.degrees(yaw_std):.1f} deg", flush=True)
        if xy_std > args.max_xy_std:
            raise SystemExit(
                "AMCL is not localized well enough. Set the pose with Foxglove's\n"
                "2D pose estimate and run the tool again."
            )
        map_docked, odom_docked = node.average_poses(3.0)
        print(
            f"  on the dock: map x={map_docked[0]:.3f} y={map_docked[1]:.3f} "
            f"yaw={math.degrees(map_docked[2]):.1f} deg",
            flush=True,
        )

        if node.command.get_subscription_count() == 0:
            raise RuntimeError("/cmd_vel_tuning has no subscriber (twist_mux running?)")
        node.set_camera(True)
        for remaining in range(max(0, args.countdown), 0, -1):
            print(f"Reversing {args.measure_distance:.2f} m in {remaining} ...", flush=True)
            node.spin_for(1.0)
        travelled = node.reverse(args.measure_distance, args.speed, timeout=40.0)
        print(f"  reversed {travelled:.2f} m; measuring the tags ...", flush=True)
        node.spin_for(2.5)  # settle, camera and AprilTag latency
        since = time.monotonic()
        map_now, odom_now = node.average_poses(4.0)
        tags = [t for t in node.tags if t[0] >= since]
        axes = [a for a in node.axes if a[0] >= since]
        if len(tags) < 5 or len(axes) < 5:
            raise RuntimeError(
                f"too few tag detections (ID 1: {len(tags)}, ID 0: {len(axes)}); "
                "are both tags in the camera image?"
            )
        tag = (sum(t[1] for t in tags) / len(tags), sum(t[2] for t in tags) / len(tags))
        axis = circular_mean([a[1] for a in axes])

        carried = docked_map_pose_from_now(map_now, odom_now, odom_docked)
        dock_pose = (map_docked[0], map_docked[1], carried[2])
        translation_x, translation_y, axis_offset = offsets_from_sample(odom_docked, tag, axis)
        staging = staging_map_pose(dock_pose, (
            float(plugin["staging_x_offset"]),
            float(plugin["staging_y_offset"]),
            float(plugin["staging_yaw_offset"]),
        ))
        xy_check = math.hypot(carried[0] - map_docked[0], carried[1] - map_docked[1])
        yaw_check = normalize_angle(map_docked[2] - carried[2])
        config_x = float(plugin["external_detection_translation_x"])
        config_y = float(plugin["external_detection_translation_y"])

        print("\nResult", flush=True)
        print(
            f"  dock pose        [{dock_pose[0]:.4f}, {dock_pose[1]:.4f}, {dock_pose[2]:.4f}]"
            f"  (yaw {math.degrees(dock_pose[2]):.1f} deg)", flush=True)
        print(f"  axis_yaw_offset  {axis_offset:+.5f} rad ({math.degrees(axis_offset):+.2f} deg)",
              flush=True)
        print(f"  undock           reverse {reverse:.2f} m, turn {turn:+.1f} deg", flush=True)
        print(f"  staging (map)    x={staging[0]:.3f} y={staging[1]:.3f} "
              f"yaw={math.degrees(staging[2]):.1f} deg", flush=True)
        print("Checks", flush=True)
        print(f"  AMCL on the dock vs. carried back by odometry: {xy_check * 100:.1f} cm, "
              f"{math.degrees(yaw_check):+.1f} deg (yaw from out here is used)", flush=True)
        print(f"  ID 1 offsets measured x={translation_x:+.3f} y={translation_y:+.3f} m, "
              f"configured x={config_x:+.3f} y={config_y:+.3f} m", flush=True)
        warnings = []
        if xy_check > 0.10:
            warnings.append("AMCL moved more than 10 cm while reversing: check localization")
        if abs(translation_y - config_y) > 0.02:
            warnings.append(
                "ID 1 sits more than 2 cm sideways from where nav2_navigation.yaml expects it: "
                "was ID 1 moved on the dock? (external_detection_translation_y)")
        for warning in warnings:
            print(f"  WARNING: {warning}", flush=True)

        site = DockSite(dock_pose, reverse, math.radians(turn), axis_offset, args.site_file)
        report.update({
            "map_on_dock": map_docked, "odom_on_dock": odom_docked,
            "map_measured": map_now, "odom_measured": odom_now,
            "reversed_m": travelled, "amcl_std": [xy_std, yaw_std],
            "id1_odom": tag, "id0_yaw_odom": axis,
            "id1_samples": len(tags), "id0_samples": len(axes),
            "dock_pose": dock_pose, "axis_yaw_offset": axis_offset,
            "translation_measured": [translation_x, translation_y],
            "translation_configured": [config_x, config_y],
            "undock": {"reverse_distance_m": reverse, "turn_angle_deg": turn},
            "staging_map": staging, "warnings": warnings,
        })
        if not _confirm(f"\nWrite {args.site_file}?", args.yes):
            print("Nothing written.", flush=True)
            return
        site_path = Path(args.site_file)
        if site_path.is_file():
            backup = site_path.with_name(f"{site_path.name}.{datetime.now():%Y%m%d_%H%M%S}.bak")
            shutil.copy2(site_path, backup)
            print(f"  previous file kept as {backup}", flush=True)
        site_path.write_text(dump_dock_site(site), encoding="utf-8")
        print(f"  wrote {site_path}", flush=True)
        print(
            "\nNext: plug the dock in again, run `docker compose restart` on the host\n"
            "and test a docking run (/dock/trigger).",
            flush=True,
        )
    except KeyboardInterrupt:
        print("\nAborted.", flush=True)
    except RuntimeError as error:
        print(f"\nFailed: {error}", flush=True)
    finally:
        try:
            node.stop()
            node.set_camera(False)
            node.spin_for(0.3)
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        if len(report) > 1:
            output = MAPS / f"dock_setup_{datetime.now():%Y%m%d_%H%M%S}.json"
            try:
                output.write_text(json.dumps(report, indent=2))
                print(f"Report: {output}", flush=True)
            except OSError as error:
                print(f"Could not save the report: {error}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
