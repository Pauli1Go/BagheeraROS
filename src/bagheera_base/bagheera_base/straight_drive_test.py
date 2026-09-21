"""Record all motion-relevant data during a three metre straight-line run."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, TwistWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _angle_delta(angle: float, reference: float) -> float:
    return math.atan2(math.sin(angle - reference), math.cos(angle - reference))


def _pose(message: Odometry | None) -> tuple[float, float, float]:
    if message is None:
        return math.nan, math.nan, math.nan
    pose = message.pose.pose
    return pose.position.x, pose.position.y, _yaw(pose.orientation)


def _twist(message: TwistStamped | None) -> tuple[float, float]:
    if message is None:
        return math.nan, math.nan
    return message.twist.linear.x, message.twist.angular.z


class StraightDriveRecorder(Node):
    """Publish the direct command when requested and sample sensor state."""

    def __init__(self) -> None:
        super().__init__("bagheera_straight_drive_test")
        self.lock = threading.Lock()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.direct_speed: float | None = None
        self.started_at = time.monotonic()
        self.rows: list[dict] = []
        self.latest_odom: dict[str, Odometry] = {}
        self.latest_imu: dict[str, Imu] = {}
        self.latest_command: dict[str, TwistStamped] = {}
        self.latest_flow: TwistWithCovarianceStamped | None = None
        self.latest_scan: LaserScan | None = None
        self.last_scan_monotonic = 0.0

        for topic in (
            "/odom",
            "/wheel_odom_raw",
            "/wheel_odom",
            "/odometry/filtered",
            "/odometry/filtered_map",
        ):
            self.create_subscription(
                Odometry,
                topic,
                lambda message, name=topic: self._on_odom(name, message),
                qos_profile_sensor_data,
            )
        for topic in (
            "/imu/wt901/data_raw",
            "/imu/data_raw",
            "/imu/data",
            "/imu_onboard/data_raw",
        ):
            self.create_subscription(
                Imu,
                topic,
                lambda message, name=topic: self._on_imu(name, message),
                qos_profile_sensor_data,
            )
        for topic in (
            "/cmd_vel",
            "/cmd_vel_tuning",
            "/cmd_vel_nav",
            "/cmd_vel_automatic_raw",
            "/cmd_vel_monitored",
        ):
            self.create_subscription(
                TwistStamped,
                topic,
                lambda message, name=topic: self._on_command(name, message),
                20,
            )
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/optical_flow/twist",
            self._on_flow,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self._tick)

    def _on_odom(self, name: str, message: Odometry) -> None:
        with self.lock:
            self.latest_odom[name] = message

    def _on_imu(self, name: str, message: Imu) -> None:
        with self.lock:
            self.latest_imu[name] = message

    def _on_command(self, name: str, message: TwistStamped) -> None:
        with self.lock:
            self.latest_command[name] = message

    def _on_flow(self, message: TwistWithCovarianceStamped) -> None:
        with self.lock:
            self.latest_flow = message

    def _on_scan(self, message: LaserScan) -> None:
        with self.lock:
            self.latest_scan = message
            self.last_scan_monotonic = time.monotonic()

    def sensors_ready(self) -> bool:
        now = time.monotonic()
        with self.lock:
            ready = (
                "/odometry/filtered" in self.latest_odom
                and "/wheel_odom" in self.latest_odom
                and "/imu/wt901/data_raw" in self.latest_imu
                and self.latest_flow is not None
                and self.latest_scan is not None
                and now - self.last_scan_monotonic < 0.5
            )
        return ready

    def direct_ready(self) -> bool:
        # One match is this recorder's own /cmd_vel_tuning observer. A second
        # match proves that twist_mux is present and can forward the command.
        return self.sensors_ready() and self.command.get_subscription_count() > 1

    def current_ekf_pose(self) -> tuple[float, float, float]:
        with self.lock:
            return _pose(self.latest_odom.get("/odometry/filtered"))

    def latest_map_pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            _yaw(transform.transform.rotation),
        )

    def start_direct(self, speed: float) -> None:
        with self.lock:
            self.direct_speed = speed
        self._publish_direct(speed)

    def stop_direct(self) -> None:
        with self.lock:
            was_direct = self.direct_speed is not None
            self.direct_speed = None
        if was_direct:
            for _ in range(20):
                self._publish_direct(0.0)
                time.sleep(0.025)

    def _publish_direct(self, speed: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = speed
        message.twist.angular.z = 0.0
        self.command.publish(message)

    def _transform(self, parent: str, child: str) -> tuple[float, float, float]:
        try:
            transform = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return math.nan, math.nan, math.nan
        value = transform.transform
        return value.translation.x, value.translation.y, _yaw(value.rotation)

    def _tick(self) -> None:
        with self.lock:
            speed = self.direct_speed
        if speed is not None:
            self._publish_direct(speed)
        self._capture()

    def _capture(self) -> None:
        with self.lock:
            odometry = dict(self.latest_odom)
            imu = dict(self.latest_imu)
            commands = dict(self.latest_command)
            flow = self.latest_flow
            scan = self.latest_scan

        ekf_x, ekf_y, ekf_yaw = _pose(odometry.get("/odometry/filtered"))
        map_ekf_x, map_ekf_y, map_ekf_yaw = _pose(
            odometry.get("/odometry/filtered_map")
        )
        wheel_x, wheel_y, wheel_yaw = _pose(odometry.get("/wheel_odom"))
        raw_x, raw_y, raw_yaw = _pose(odometry.get("/odom"))
        map_x, map_y, map_yaw = self._transform("map", "base_link")
        map_odom_x, map_odom_y, map_odom_yaw = self._transform("map", "odom")

        ekf = odometry.get("/odometry/filtered")
        wheel = odometry.get("/wheel_odom")
        wt901 = imu.get("/imu/wt901/data_raw")
        onboard = imu.get("/imu_onboard/data_raw")
        flow_vx = flow_vy = flow_variance_x = flow_variance_y = math.nan
        if flow is not None:
            flow_vx = flow.twist.twist.linear.x
            flow_vy = flow.twist.twist.linear.y
            flow_variance_x = flow.twist.covariance[0]
            flow_variance_y = flow.twist.covariance[7]
        scan_min = scan_front = math.nan
        if scan is not None:
            valid = [value for value in scan.ranges if math.isfinite(value)]
            if valid:
                scan_min = min(valid)
            front = []
            for index, value in enumerate(scan.ranges):
                angle = scan.angle_min + index * scan.angle_increment
                if abs(angle) <= math.radians(10.0) and math.isfinite(value):
                    front.append(value)
            if front:
                scan_front = min(front)

        cmd_x, cmd_wz = _twist(commands.get("/cmd_vel"))
        tune_x, tune_wz = _twist(commands.get("/cmd_vel_tuning"))
        nav_x, nav_wz = _twist(commands.get("/cmd_vel_nav"))
        auto_x, auto_wz = _twist(commands.get("/cmd_vel_automatic_raw"))
        monitored_x, monitored_wz = _twist(commands.get("/cmd_vel_monitored"))
        self.rows.append(
            {
                "elapsed_s": time.monotonic() - self.started_at,
                "map_base_x": map_x,
                "map_base_y": map_y,
                "map_base_yaw_rad": map_yaw,
                "map_odom_x": map_odom_x,
                "map_odom_y": map_odom_y,
                "map_odom_yaw_rad": map_odom_yaw,
                "ekf_x": ekf_x,
                "ekf_y": ekf_y,
                "ekf_yaw_rad": ekf_yaw,
                "map_ekf_x": map_ekf_x,
                "map_ekf_y": map_ekf_y,
                "map_ekf_yaw_rad": map_ekf_yaw,
                "wheel_x": wheel_x,
                "wheel_y": wheel_y,
                "wheel_yaw_rad": wheel_yaw,
                "raw_odom_x": raw_x,
                "raw_odom_y": raw_y,
                "raw_odom_yaw_rad": raw_yaw,
                "ekf_vx": math.nan if ekf is None else ekf.twist.twist.linear.x,
                "ekf_vy": math.nan if ekf is None else ekf.twist.twist.linear.y,
                "ekf_wz": math.nan if ekf is None else ekf.twist.twist.angular.z,
                "wheel_vx": math.nan if wheel is None else wheel.twist.twist.linear.x,
                "wheel_vy": math.nan if wheel is None else wheel.twist.twist.linear.y,
                "wheel_wz": math.nan if wheel is None else wheel.twist.twist.angular.z,
                "gyro_wz": math.nan if wt901 is None else wt901.angular_velocity.z,
                "accel_x": math.nan if wt901 is None else wt901.linear_acceleration.x,
                "accel_y": math.nan if wt901 is None else wt901.linear_acceleration.y,
                "onboard_gyro_wz": (
                    math.nan if onboard is None else onboard.angular_velocity.z
                ),
                "flow_vx": flow_vx,
                "flow_vy": flow_vy,
                "flow_variance_x": flow_variance_x,
                "flow_variance_y": flow_variance_y,
                "cmd_vx": cmd_x,
                "cmd_wz": cmd_wz,
                "tuning_vx": tune_x,
                "tuning_wz": tune_wz,
                "nav_vx": nav_x,
                "nav_wz": nav_wz,
                "automatic_vx": auto_x,
                "automatic_wz": auto_wz,
                "monitored_vx": monitored_x,
                "monitored_wz": monitored_wz,
                "scan_min_m": scan_min,
                "scan_front_m": scan_front,
            }
        )


def _start_bag(output: Path, include_camera: bool) -> subprocess.Popen:
    topics = [
        "/tf",
        "/tf_static",
        "/map",
        "/map_metadata",
        "/scan",
        "/odom",
        "/wheel_odom_raw",
        "/wheel_odom",
        "/odometry/filtered",
        "/odometry/filtered_map",
        "/wheel_ticks",
        "/optical_flow/raw",
        "/optical_flow/twist",
        "/imu/wt901/data_raw",
        "/imu/data_raw",
        "/imu/data",
        "/imu_onboard/data_raw",
        "/imu_onboard/temperature",
        "/battery_state",
        "/hardware_bridge/status",
        "/hardware_bridge/power",
        "/hardware_bridge/emergency",
        "/cmd_vel",
        "/cmd_vel_tuning",
        "/cmd_vel_teleop",
        "/cmd_vel_nav",
        "/cmd_vel_automatic_raw",
        "/cmd_vel_monitored",
        "/pose",
        "/amcl_pose",
        "/particle_cloud",
        "/diagnostics",
        "/local_costmap/costmap",
        "/local_costmap/costmap_updates",
        "/global_costmap/costmap",
        "/global_costmap/costmap_updates",
        "/plan",
        "/local_plan",
    ]
    if include_camera:
        topics.extend(
            [
                "/camera/image_raw/compressed",
                "/camera/camera_info",
            ]
        )
    return subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(output / "rosbag"), *topics],
        stdout=(output / "rosbag.log").open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop_bag(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5.0)


def _sign_changes(values: list[float], deadband: float) -> int:
    signs = [1 if value > deadband else -1 for value in values if abs(value) > deadband]
    return sum(left != right for left, right in zip(signs, signs[1:]))


def _trajectory_metrics(rows: list[dict], prefix: str) -> dict:
    points = [
        (row[f"{prefix}_x"], row[f"{prefix}_y"], row[f"{prefix}_yaw_rad"])
        for row in rows
        if all(
            math.isfinite(row[name])
            for name in (f"{prefix}_x", f"{prefix}_y", f"{prefix}_yaw_rad")
        )
    ]
    if len(points) < 2:
        return {"sample_count": len(points)}
    start_x, start_y, start_yaw = points[0]
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    forward = []
    lateral = []
    yaw_error = []
    for x, y, yaw in points:
        dx = x - start_x
        dy = y - start_y
        forward.append(cosine * dx + sine * dy)
        lateral.append(-sine * dx + cosine * dy)
        yaw_error.append(_angle_delta(yaw, start_yaw))
    return {
        "sample_count": len(points),
        "forward_m": float(forward[-1]),
        "end_lateral_m": float(lateral[-1]),
        "max_abs_lateral_m": float(max(abs(value) for value in lateral)),
        "lateral_rms_m": float(
            math.sqrt(sum(value * value for value in lateral) / len(lateral))
        ),
        "end_yaw_error_deg": float(math.degrees(yaw_error[-1])),
        "max_abs_yaw_error_deg": float(
            math.degrees(max(abs(value) for value in yaw_error))
        ),
    }


def _write_results(output: Path, rows: list[dict], metadata: dict) -> None:
    if rows:
        with (output / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    moving = [row for row in rows if abs(row["cmd_vx"]) > 0.01]
    source = moving if moving else rows
    angular_commands = [row["cmd_wz"] for row in source if math.isfinite(row["cmd_wz"])]
    gyro = [row["gyro_wz"] for row in source if math.isfinite(row["gyro_wz"])]
    flow_y = [row["flow_vy"] for row in source if math.isfinite(row["flow_vy"])]
    report = {
        **metadata,
        "sample_count": len(rows),
        "moving_sample_count": len(moving),
        "trajectories": {
            name: _trajectory_metrics(source, name)
            for name in ("map_base", "ekf", "wheel", "raw_odom", "map_ekf")
        },
        "command": {
            "max_abs_angular_rps": (
                None if not angular_commands else float(max(map(abs, angular_commands)))
            ),
            "angular_sign_changes": _sign_changes(angular_commands, 0.01),
        },
        "gyro": {
            "max_abs_wz_rps": None if not gyro else float(max(map(abs, gyro))),
            "wz_sign_changes": _sign_changes(gyro, 0.01),
        },
        "optical_flow": {
            "max_abs_vy_mps": None if not flow_y else float(max(map(abs, flow_y))),
            "vy_sign_changes": _sign_changes(flow_y, 0.01),
        },
    }
    (output / "report.yaml").write_text(
        yaml.safe_dump(report, sort_keys=False), encoding="utf-8"
    )


def _copy_configs(output: Path) -> None:
    destination = output / "config"
    destination.mkdir()
    source = Path("/bagheera_ws/install/share/bagheera_base/config")
    for name in (
        "base.yaml",
        "robot.yaml",
        "sensors.yaml",
        "localization.yaml",
        "nav2_localization.yaml",
        "nav2_navigation.yaml",
        "slam.yaml",
    ):
        path = source / name
        if path.exists():
            shutil.copy2(path, destination / name)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Record a direct or Nav2-controlled three metre straight run"
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--mode", choices=("direct", "nav2"), default="direct")
    parser.add_argument("--distance", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=0.16)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--cooldown", type=float, default=5.0)
    parser.add_argument("--include-camera", action="store_true")
    parser.add_argument("--output", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("automatic motion is disabled unless --execute is supplied")
    if not 0.1 <= args.distance <= 10.0:
        parser.error("distance must be between 0.1 and 10.0 metres")
    if not 0.02 <= args.speed <= 0.5:
        parser.error("speed must be between 0.02 and 0.5 m/s")

    output = Path(args.output) if args.output else Path("/bagheera_ws/maps") / (
        "straight_test_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output.mkdir(parents=True, exist_ok=False)
    _copy_configs(output)

    rclpy.init()
    node = StraightDriveRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_stopping = threading.Event()

    def spin_executor() -> None:
        try:
            executor.spin()
        except Exception:
            if not executor_stopping.is_set():
                raise

    thread = threading.Thread(target=spin_executor, daemon=True)
    thread.start()
    bag = None
    goal_handle = None
    result_label = "not_started"
    failed = False
    start_pose = (math.nan, math.nan, math.nan)
    target_pose = None
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            mode_ready = (
                node.direct_ready()
                if args.mode == "direct"
                else node.sensors_ready() and node.action.wait_for_server(timeout_sec=0.0)
            )
            if mode_ready:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("required sensors or command/action path are unavailable")

        bag = _start_bag(output, args.include_camera)
        time.sleep(2.0)
        print(f"Modus {args.mode}: {args.distance:.2f} m geradeaus.")
        for remaining in range(max(0, args.countdown), 0, -1):
            print(f"Start in {remaining} ...", flush=True)
            time.sleep(1.0)

        deadline = time.monotonic() + args.timeout
        if args.mode == "direct":
            start_pose = node.current_ekf_pose()
            if not all(math.isfinite(value) for value in start_pose):
                raise RuntimeError("no finite EKF start pose")
            node.start_direct(args.speed)
            next_report = 0.5
            while time.monotonic() < deadline:
                x, y, _ = node.current_ekf_pose()
                dx = x - start_pose[0]
                dy = y - start_pose[1]
                progress = math.cos(start_pose[2]) * dx + math.sin(start_pose[2]) * dy
                if progress >= args.distance:
                    result_label = "distance_reached"
                    break
                if progress >= next_report:
                    print(f"EKF-Fortschritt: {progress:.2f} m", flush=True)
                    next_report += 0.5
                time.sleep(0.05)
            else:
                raise TimeoutError(f"did not reach {args.distance:.2f} m")
            node.stop_direct()
        else:
            start_pose = node.latest_map_pose()
            target_pose = (
                start_pose[0] + args.distance * math.cos(start_pose[2]),
                start_pose[1] + args.distance * math.sin(start_pose[2]),
                start_pose[2],
            )
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x = target_pose[0]
            goal.pose.pose.position.y = target_pose[1]
            goal.pose.pose.orientation.z = math.sin(target_pose[2] / 2.0)
            goal.pose.pose.orientation.w = math.cos(target_pose[2] / 2.0)
            future = node.action.send_goal_async(goal)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done() or not future.result().accepted:
                raise RuntimeError("Nav2 did not accept the generated goal")
            goal_handle = future.result()
            result_future = goal_handle.get_result_async()
            while not result_future.done() and time.monotonic() < deadline:
                time.sleep(0.1)
            if not result_future.done():
                goal_handle.cancel_goal_async()
                raise TimeoutError("Nav2 goal timed out")
            status = result_future.result().status
            labels = {
                GoalStatus.STATUS_SUCCEEDED: "succeeded",
                GoalStatus.STATUS_CANCELED: "canceled",
                GoalStatus.STATUS_ABORTED: "aborted",
            }
            result_label = labels.get(status, f"status_{status}")
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"Nav2 goal {result_label}")

        print(f"Fahrt beendet; zeichne noch {args.cooldown:.1f} s auf.")
        time.sleep(args.cooldown)
    except KeyboardInterrupt:
        failed = True
        result_label = "interrupted"
        print("\nAbgebrochen.")
    except Exception as error:
        failed = True
        result_label = f"failed: {error}"
        print(f"Test fehlgeschlagen: {error}")
    finally:
        node.stop_direct()
        if goal_handle is not None and result_label not in ("succeeded",):
            goal_handle.cancel_goal_async()
            time.sleep(0.2)
        _stop_bag(bag)
        with node.lock:
            rows = list(node.rows)
        metadata = {
            "mode": args.mode,
            "requested_distance_m": float(args.distance),
            "requested_direct_speed_mps": float(args.speed),
            "include_camera": bool(args.include_camera),
            "result": result_label,
            "start_pose_xy_yaw_rad": [float(value) for value in start_pose],
            "target_pose_xy_yaw_rad": (
                None if target_pose is None else [float(value) for value in target_pose]
            ),
        }
        _write_results(output, rows, metadata)
        executor_stopping.set()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"Auswertung gespeichert: {output}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
