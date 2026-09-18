"""Measure LiDAR-to-map translation drift during a gyro-controlled pivot."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import shutil
import subprocess
import threading
import time

import cv2
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _pose(message: Odometry | None) -> tuple[float, float, float]:
    if message is None:
        return math.nan, math.nan, math.nan
    pose = message.pose.pose
    return pose.position.x, pose.position.y, _yaw(pose.orientation)


class RotationShiftRecorder(Node):
    def __init__(self, analysis_period: float, max_range: float) -> None:
        super().__init__("bagheera_rotation_shift_test")
        self.lock = threading.Lock()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self.desired_angular = 0.0
        self.command_timer = self.create_timer(0.05, self._republish_command)
        self.gyro: list[tuple[float, float]] = []
        self.angle = 0.0
        self.bias = 0.0
        self.last_gyro_stamp: float | None = None
        self.integrating = False
        self.latest_scan: LaserScan | None = None
        self.latest_map: OccupancyGrid | None = None
        self.reference_map: dict | None = None
        self.latest_ekf: Odometry | None = None
        self.latest_wheel: Odometry | None = None
        self.latest_flow: TwistWithCovarianceStamped | None = None
        self.last_capture = 0.0
        self.analysis_period = analysis_period
        self.max_range = max_range
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/wt901/data_raw", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, qos_profile_sensor_data)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/optical_flow/twist",
            self._on_flow,
            qos_profile_sensor_data,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        with self.lock:
            self.latest_map = message

    def _on_scan(self, message: LaserScan) -> None:
        with self.lock:
            self.latest_scan = message

    def _on_imu(self, message: Imu) -> None:
        now = time.monotonic()
        angular = float(message.angular_velocity.z)
        with self.lock:
            self.gyro.append((now, angular))
            if len(self.gyro) > 5000:
                del self.gyro[:1000]
            if self.integrating and self.last_gyro_stamp is not None:
                elapsed = now - self.last_gyro_stamp
                if 0.0 < elapsed <= 0.25:
                    self.angle += (angular - self.bias) * elapsed
            self.last_gyro_stamp = now

    def _on_ekf(self, message: Odometry) -> None:
        with self.lock:
            self.latest_ekf = message

    def _on_wheel(self, message: Odometry) -> None:
        with self.lock:
            self.latest_wheel = message

    def _on_flow(self, message: TwistWithCovarianceStamped) -> None:
        with self.lock:
            self.latest_flow = message

    def ready(self) -> bool:
        with self.lock:
            fresh_gyro = bool(self.gyro) and time.monotonic() - self.gyro[-1][0] < 0.5
            messages = self.latest_map is not None and self.latest_scan is not None
        try:
            self.tf_buffer.lookup_transform("map", "lidar_link", Time())
            transform_ready = True
        except TransformException:
            transform_ready = False
        return (
            fresh_gyro
            and messages
            and transform_ready
            and self.command.get_subscription_count() > 0
        )

    def estimate_bias(self, duration: float) -> float:
        start = time.monotonic()
        while time.monotonic() - start < duration:
            if not self.ready():
                raise RuntimeError("required sensor, map, TF or command path became unavailable")
            time.sleep(0.05)
        with self.lock:
            samples = [value for stamp, value in self.gyro if stamp >= start]
        if len(samples) < 20:
            raise RuntimeError("not enough stationary gyro samples")
        self.bias = float(np.median(samples))
        return self.bias

    def freeze_reference_map(self, output: Path) -> None:
        with self.lock:
            message = self.latest_map
        if message is None:
            raise RuntimeError("no map available")
        width = message.info.width
        height = message.info.height
        occupancy = np.asarray(message.data, dtype=np.int16).reshape(height, width)
        occupied = occupancy >= 50
        if int(np.count_nonzero(occupied)) < 20:
            raise RuntimeError("map has too few occupied cells for scan matching")
        free_from_walls = np.where(occupied, 0, 255).astype(np.uint8)
        distance = cv2.distanceTransform(free_from_walls, cv2.DIST_L2, 5)
        origin = message.info.origin
        self.reference_map = {
            "distance": distance * message.info.resolution,
            "resolution": message.info.resolution,
            "origin_x": origin.position.x,
            "origin_y": origin.position.y,
            "origin_yaw": _yaw(origin.orientation),
            "width": width,
            "height": height,
        }
        np.savez_compressed(
            output / "reference_map.npz",
            occupancy=occupancy,
            resolution=message.info.resolution,
            origin=np.asarray([
                origin.position.x,
                origin.position.y,
                self.reference_map["origin_yaw"],
            ]),
        )

    def begin(self) -> None:
        with self.lock:
            self.angle = 0.0
            self.last_gyro_stamp = self.gyro[-1][0]
            self.integrating = True

    def gyro_angle(self) -> float:
        with self.lock:
            return self.angle

    def publish(self, angular: float) -> None:
        with self.lock:
            self.desired_angular = angular
        self._publish_command(angular)

    def _republish_command(self) -> None:
        with self.lock:
            angular = self.desired_angular
        self._publish_command(angular)

    def _publish_command(self, angular: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.angular.z = angular
        self.command.publish(message)

    def stop(self) -> None:
        with self.lock:
            self.integrating = False
            self.desired_angular = 0.0
        for _ in range(30):
            self.publish(0.0)
            time.sleep(0.025)

    def _grid_coordinates(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        reference = self.reference_map
        assert reference is not None
        dx = x - reference["origin_x"]
        dy = y - reference["origin_y"]
        cosine = math.cos(reference["origin_yaw"])
        sine = math.sin(reference["origin_yaw"])
        gx = (cosine * dx + sine * dy) / reference["resolution"]
        gy = (-sine * dx + cosine * dy) / reference["resolution"]
        return gx, gy

    def _sample_grid(self, gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
        reference = self.reference_map
        assert reference is not None
        return cv2.remap(
            reference["distance"].astype(np.float32, copy=False),
            gx.astype(np.float32),
            gy.astype(np.float32),
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.75,
        )

    def _score(self, x: np.ndarray, y: np.ndarray) -> float:
        gx, gy = self._grid_coordinates(x, y)
        distances = np.minimum(
            self._sample_grid(gx.reshape(1, -1), gy.reshape(1, -1))[0], 0.75
        )
        count = max(1, int(0.8 * len(distances)))
        return float(np.mean(np.partition(distances, count - 1)[:count]))

    def _translation_scores(
        self,
        x: np.ndarray,
        y: np.ndarray,
        translations: np.ndarray,
    ) -> np.ndarray:
        reference = self.reference_map
        assert reference is not None
        gx, gy = self._grid_coordinates(x, y)
        cosine = math.cos(reference["origin_yaw"])
        sine = math.sin(reference["origin_yaw"])
        shift_x = (
            cosine * translations[:, 0] + sine * translations[:, 1]
        ) / reference["resolution"]
        shift_y = (
            -sine * translations[:, 0] + cosine * translations[:, 1]
        ) / reference["resolution"]
        sample_x = gx[None, :] + shift_x[:, None]
        sample_y = gy[None, :] + shift_y[:, None]
        distances = np.minimum(self._sample_grid(sample_x, sample_y), 0.75)
        count = max(1, int(0.8 * distances.shape[1]))
        return np.mean(np.partition(distances, count - 1, axis=1)[:, :count], axis=1)

    def _match(self, world_x: np.ndarray, world_y: np.ndarray, lidar_x: float, lidar_y: float):
        best = (self._score(world_x, world_y), 0.0, 0.0, 0.0)
        coarse_axis = np.arange(-0.30, 0.301, 0.05)
        coarse_x, coarse_y = np.meshgrid(coarse_axis, coarse_axis)
        coarse_translations = np.column_stack((coarse_x.ravel(), coarse_y.ravel()))
        for yaw_deg in np.arange(-6.0, 6.01, 1.5):
            yaw = math.radians(float(yaw_deg))
            cosine, sine = math.cos(yaw), math.sin(yaw)
            rel_x, rel_y = world_x - lidar_x, world_y - lidar_y
            rotated_x = lidar_x + cosine * rel_x - sine * rel_y
            rotated_y = lidar_y + sine * rel_x + cosine * rel_y
            scores = self._translation_scores(rotated_x, rotated_y, coarse_translations)
            index = int(np.argmin(scores))
            if scores[index] < best[0]:
                best = (
                    float(scores[index]),
                    float(coarse_translations[index, 0]),
                    float(coarse_translations[index, 1]),
                    yaw,
                )
        _, coarse_x, coarse_y, coarse_yaw = best
        fine_x, fine_y = np.meshgrid(
            np.arange(coarse_x - 0.05, coarse_x + 0.051, 0.01),
            np.arange(coarse_y - 0.05, coarse_y + 0.051, 0.01),
        )
        fine_translations = np.column_stack((fine_x.ravel(), fine_y.ravel()))
        for yaw in np.arange(coarse_yaw - math.radians(1.5), coarse_yaw + math.radians(1.51), math.radians(0.5)):
            cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
            rel_x, rel_y = world_x - lidar_x, world_y - lidar_y
            rotated_x = lidar_x + cosine * rel_x - sine * rel_y
            rotated_y = lidar_y + sine * rel_x + cosine * rel_y
            scores = self._translation_scores(rotated_x, rotated_y, fine_translations)
            index = int(np.argmin(scores))
            if scores[index] < best[0]:
                best = (
                    float(scores[index]),
                    float(fine_translations[index, 0]),
                    float(fine_translations[index, 1]),
                    float(yaw),
                )
        return best

    def capture_latest(self) -> dict | None:
        now = time.monotonic()
        if now - self.last_capture < self.analysis_period:
            return None
        self.last_capture = now
        with self.lock:
            scan = self.latest_scan
            ekf = self.latest_ekf
            wheel = self.latest_wheel
            flow = self.latest_flow
            angle = self.angle
        if scan is None or self.reference_map is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=float)
        ray_angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= max(scan.range_min, 0.30))
            & (ranges <= min(scan.range_max, self.max_range))
        )
        indices = np.flatnonzero(valid)
        if len(indices) < 30:
            return None
        if len(indices) > 300:
            indices = indices[np.linspace(0, len(indices) - 1, 300).astype(int)]
        local_x = ranges[indices] * np.cos(ray_angles[indices])
        local_y = ranges[indices] * np.sin(ray_angles[indices])
        stamp = Time.from_msg(scan.header.stamp)
        try:
            map_lidar = self.tf_buffer.lookup_transform(
                "map", scan.header.frame_id or "lidar_link", stamp,
                timeout=Duration(seconds=0.10),
            )
            map_odom = self.tf_buffer.lookup_transform("map", "odom", stamp)
            odom_base = self.tf_buffer.lookup_transform("odom", "base_link", stamp)
        except TransformException:
            return None
        transform = map_lidar.transform
        lidar_yaw = _yaw(transform.rotation)
        cosine, sine = math.cos(lidar_yaw), math.sin(lidar_yaw)
        world_x = transform.translation.x + cosine * local_x - sine * local_y
        world_y = transform.translation.y + sine * local_x + cosine * local_y
        ekf_x, ekf_y, ekf_yaw = _pose(ekf)
        wheel_x, wheel_y, wheel_yaw = _pose(wheel)
        ekf_vx = ekf_vy = wheel_vx = wheel_wz = math.nan
        if ekf is not None:
            ekf_vx = ekf.twist.twist.linear.x
            ekf_vy = ekf.twist.twist.linear.y
        if wheel is not None:
            wheel_vx = wheel.twist.twist.linear.x
            wheel_wz = wheel.twist.twist.angular.z
        flow_vx = flow_vy = flow_variance = math.nan
        if flow is not None:
            flow_vx = flow.twist.twist.linear.x
            flow_vy = flow.twist.twist.linear.y
            flow_variance = flow.twist.covariance[0]
        row = {
            "monotonic_s": now,
            "scan_stamp_s": stamp.nanoseconds / 1.0e9,
            "gyro_angle_deg": math.degrees(angle),
            "map_lidar_x": transform.translation.x,
            "map_lidar_y": transform.translation.y,
            "map_lidar_yaw_deg": math.degrees(lidar_yaw),
            "match_dx_m": math.nan,
            "match_dy_m": math.nan,
            "match_dyaw_deg": math.nan,
            "score_before_m": math.nan,
            "score_after_m": math.nan,
            "map_odom_x": map_odom.transform.translation.x,
            "map_odom_y": map_odom.transform.translation.y,
            "map_odom_yaw_deg": math.degrees(_yaw(map_odom.transform.rotation)),
            "odom_base_x": odom_base.transform.translation.x,
            "odom_base_y": odom_base.transform.translation.y,
            "odom_base_yaw_deg": math.degrees(_yaw(odom_base.transform.rotation)),
            "ekf_x": ekf_x,
            "ekf_y": ekf_y,
            "ekf_yaw_deg": math.degrees(ekf_yaw),
            "ekf_vx": ekf_vx,
            "ekf_vy": ekf_vy,
            "wheel_x": wheel_x,
            "wheel_y": wheel_y,
            "wheel_yaw_deg": math.degrees(wheel_yaw),
            "wheel_vx": wheel_vx,
            "wheel_wz": wheel_wz,
            "flow_vx": flow_vx,
            "flow_vy": flow_vy,
            "flow_variance": flow_variance,
            "scan_points": len(indices),
        }
        return {
            "row": row,
            "world_x": world_x,
            "world_y": world_y,
            "lidar_x": transform.translation.x,
            "lidar_y": transform.translation.y,
        }

    def analyse_capture(self, capture: dict) -> dict:
        row = capture["row"]
        world_x = capture["world_x"]
        world_y = capture["world_y"]
        row["score_before_m"] = self._score(world_x, world_y)
        score, correction_x, correction_y, correction_yaw = self._match(
            world_x, world_y, capture["lidar_x"], capture["lidar_y"]
        )
        row["match_dx_m"] = correction_x
        row["match_dy_m"] = correction_y
        row["match_dyaw_deg"] = math.degrees(correction_yaw)
        row["score_after_m"] = score
        return row


def _span(rows: list[dict], x_name: str, y_name: str) -> float:
    points = np.asarray([[row[x_name], row[y_name]] for row in rows], dtype=float)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 2:
        return math.nan
    center = np.median(points, axis=0)
    return float(np.max(np.linalg.norm(points - center, axis=1)))


def _report(rows: list[dict], bias: float) -> dict:
    configured_lidar = (0.1834103944436625, 0.011005001769871877)
    moving_rows = [
        row for row in rows if 5.0 <= abs(row["gyro_angle_deg"]) <= 355.0
    ]
    fit_rows = moving_rows if len(moving_rows) >= 8 else rows
    angles = np.radians(np.asarray([row["gyro_angle_deg"] for row in fit_rows]))
    corrections = np.asarray([
        [row["match_dx_m"], row["match_dy_m"]] for row in fit_rows
    ])
    offset = [math.nan, math.nan]
    fit_rms = math.nan
    if len(rows) >= 8:
        matrices = []
        values = []
        for angle, correction in zip(angles, corrections):
            rotation = np.asarray([
                [math.cos(angle), -math.sin(angle)],
                [math.sin(angle), math.cos(angle)],
            ])
            matrices.append(rotation - np.eye(2))
            values.append(correction - corrections[0])
        matrix = np.vstack(matrices)
        vector = np.concatenate(values)
        estimate, _, _, _ = np.linalg.lstsq(matrix, vector, rcond=None)
        residual = matrix @ estimate - vector
        offset = [float(estimate[0]), float(estimate[1])]
        fit_rms = float(np.sqrt(np.mean(residual ** 2)))
    correction_norm = np.linalg.norm(corrections, axis=1) if len(rows) else np.asarray([])
    motion_rows = moving_rows if moving_rows else rows
    odom_span = _span(motion_rows, "odom_base_x", "odom_base_y")
    map_odom_span = _span(motion_rows, "map_odom_x", "map_odom_y")
    flow_variances = [
        row["flow_variance"] for row in motion_rows
        if math.isfinite(row["flow_variance"])
    ]
    ungated_fraction = (
        math.nan if not flow_variances else
        float(sum(value < 1000.0 for value in flow_variances) / len(flow_variances))
    )
    flow_vy = [abs(row["flow_vy"]) for row in motion_rows if math.isfinite(row["flow_vy"])]
    ekf_vy = [abs(row["ekf_vy"]) for row in motion_rows if math.isfinite(row.get("ekf_vy", math.nan))]
    conclusions = []
    offset_norm = math.hypot(*offset)
    if math.isfinite(offset_norm) and offset_norm > 0.02 and fit_rms < 0.04:
        conclusions.append("shift follows a rotating lever-arm pattern; inspect lidar_x/lidar_y")
    if math.isfinite(odom_span) and odom_span > 0.03:
        conclusions.append("odom->base_link translates during the pivot; local fusion contributes to the shift")
    if math.isfinite(map_odom_span) and map_odom_span > 0.05:
        conclusions.append("SLAM applies large map->odom translation corrections during the pivot")
    median_ekf_vy = None if not ekf_vy else float(np.median(ekf_vy))
    if (
        math.isfinite(ungated_fraction)
        and ungated_fraction > 0.01
        and (
            (median_ekf_vy is not None and median_ekf_vy > 0.01)
            or (math.isfinite(odom_span) and odom_span > 0.03)
        )
    ):
        conclusions.append(
            "optical flow intermittently remained ungated during the pivot and can seed false lateral EKF velocity"
        )
    return {
        "gyro_bias_rad_s": bias,
        "sample_count": len(rows),
        "moving_sample_count": len(moving_rows),
        "max_scan_match_translation_m": (
            None if not len(correction_norm) else float(np.max(correction_norm))
        ),
        "max_scan_match_yaw_deg": (
            None if not rows else max(abs(row["match_dyaw_deg"]) for row in rows)
        ),
        "odom_base_translation_span_m": odom_span,
        "map_odom_translation_span_m": map_odom_span,
        "optical_flow_ungated_fraction": ungated_fraction,
        "median_abs_optical_flow_vy_mps": (
            None if not flow_vy else float(np.median(flow_vy))
        ),
        "median_abs_ekf_vy_mps": (
            median_ekf_vy
        ),
        "lever_arm_fit_parameter_base_xy_m": offset,
        # The fitted parameter is the effective static-transform delta that
        # must be applied to base_link->lidar_link.
        "recommended_lidar_offset_delta_base_xy_m": [
            offset[0], offset[1]
        ],
        "recommended_lidar_offset_base_xy_m": [
            configured_lidar[0] + offset[0], configured_lidar[1] + offset[1]
        ],
        "lever_arm_fit_rms_m": fit_rms,
        "conclusions": conclusions or ["no single dominant cause crossed the automatic thresholds"],
    }


def _start_bag(output: Path) -> subprocess.Popen:
    topics = [
        "/map", "/map_metadata", "/scan", "/tf", "/tf_static",
        "/odometry/filtered", "/odometry/filtered_map", "/wheel_odom",
        "/wheel_odom_raw", "/wheel_ticks", "/optical_flow/raw",
        "/optical_flow/twist", "/imu/wt901/data_raw", "/imu/data_raw",
        "/cmd_vel", "/cmd_vel_tuning", "/pose", "/diagnostics",
        "/slam_toolbox/scan_visualization",
    ]
    return subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(output / "rosbag"), *topics],
        stdout=(output / "rosbag.log").open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Measure LiDAR/map shift over a gyro-controlled 360 degree pivot")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--clockwise", action="store_true")
    parser.add_argument("--maximum-speed", type=float, default=0.30)
    parser.add_argument("--minimum-speed", type=float, default=0.18)
    parser.add_argument("--gain", type=float, default=0.7)
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--cooldown", type=float, default=5.0)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--output", default=None)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("automatic motion is disabled unless --execute is supplied")

    output = Path(args.output) if args.output else Path("/bagheera_ws/maps") / (
        "rotation_shift_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output.mkdir(parents=True, exist_ok=False)
    config_output = output / "config"
    config_output.mkdir()
    for name in ("robot.yaml", "sensors.yaml", "localization.yaml", "slam.yaml"):
        source = Path("/bagheera_ws/install/share/bagheera_base/config") / name
        if source.exists():
            shutil.copy2(source, config_output / name)

    rclpy.init()
    node = RotationShiftRecorder(analysis_period=0.20, max_range=8.0)
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
    bag: subprocess.Popen | None = None
    captures: list[dict] = []
    rows: list[dict] = []
    failed = False
    failure_reason = None
    bias = math.nan
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not node.ready():
            time.sleep(0.2)
        if not node.ready():
            raise RuntimeError("mapping must publish /map and map->lidar_link TF before this test")
        node.freeze_reference_map(output)
        print("Referenzkarte fixiert. Gyro-Offset wird 2.0 s im Stillstand gemessen.")
        bias = node.estimate_bias(2.0)
        print(f"Gyro-Offset: {bias:+.5f} rad/s")
        bag = _start_bag(output)
        time.sleep(2.0)
        if bag.poll() is not None:
            raise RuntimeError("rosbag recorder failed to start; see rosbag.log")
        for remaining in range(max(0, args.countdown), 0, -1):
            print(f"Start in {remaining} ...", flush=True)
            time.sleep(1.0)

        initial_capture = node.capture_latest()
        if initial_capture:
            captures.append(initial_capture)

        direction = -1.0 if args.clockwise else 1.0
        target = direction * 2.0 * math.pi
        tolerance = math.radians(args.tolerance_deg)
        node.begin()
        deadline = time.monotonic() + args.timeout
        next_report = 45.0
        while time.monotonic() < deadline:
            if not node.ready():
                raise RuntimeError("sensor, map, TF or command path became unavailable")
            angle = node.gyro_angle()
            remaining = target - angle
            if abs(remaining) <= tolerance or direction * remaining <= 0.0:
                break
            speed = min(args.maximum_speed, max(args.minimum_speed, args.gain * abs(remaining)))
            node.publish(math.copysign(speed, remaining))
            capture = node.capture_latest()
            if capture:
                captures.append(capture)
            progress = abs(math.degrees(angle))
            if progress >= next_report:
                print(
                    f"Gyro {math.degrees(angle):+.1f} deg, "
                    f"{len(captures)} Scan/TF-Paare gespeichert"
                )
                next_report += 45.0
            time.sleep(0.04)
        else:
            raise TimeoutError("gyro did not reach 360 degrees")
        node.stop()
        print(f"Drehung fertig bei {math.degrees(node.gyro_angle()):+.1f} deg; beobachte SLAM noch {args.cooldown:.1f} s.")
        cooldown_end = time.monotonic() + args.cooldown
        while time.monotonic() < cooldown_end:
            capture = node.capture_latest()
            if capture:
                captures.append(capture)
            time.sleep(0.05)
    except KeyboardInterrupt:
        failed = True
        failure_reason = "interrupted"
        print("\nAbgebrochen.")
    except Exception as error:
        failed = True
        failure_reason = str(error)
        print(f"Test fehlgeschlagen: {error}")
    finally:
        node.stop()
        if bag is not None and bag.poll() is None:
            bag.send_signal(2)
            try:
                bag.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                bag.terminate()
        if captures:
            print(f"Werte {len(captures)} zeitgestempelte Scan/TF-Paare aus ...")
            next_progress = 10
            for index, capture in enumerate(captures, start=1):
                rows.append(node.analyse_capture(capture))
                progress = int(100 * index / len(captures))
                if progress >= next_progress:
                    print(f"  Auswertung {progress}%", flush=True)
                    next_progress += 10
        if rows:
            with (output / "shift_samples.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        report = _report(rows, bias)
        report.update({
            "complete": not failed,
            "failure_reason": failure_reason,
            "created": datetime.now().isoformat(timespec="seconds"),
            "configured_lidar_offset_base_xy_m": [
                0.1834103944436625, 0.011005001769871877
            ],
        })
        (output / "report.yaml").write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        executor_stopping.set()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
        print(f"Auswertung gespeichert: {output}")
        for conclusion in report["conclusions"]:
            print(f"  - {conclusion}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
