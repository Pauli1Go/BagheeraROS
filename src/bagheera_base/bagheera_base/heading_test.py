"""Automatic 360-degree turn that judges Bagheera's heading drift and its cause.

The robot stands still, turns once on the spot with a stop every 45 degrees
and stands still again. The LiDAR is the reference: every stop is
scan-matched against the first scan, which measures the true rotation
independently of all motion sensors. Gyro, wheel odometry, EKF and (if
calibrated) compass are compared with it; heading_test_math grades the EKF
drift and each possible cause as OK / WARN / FAIL.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import gzip
import json
import math
from pathlib import Path
import statistics
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, MagneticField
import yaml

from .compass_math import apply_calibration, load_calibration, rotate_z
from .compass_test_math import (
    circular_mean,
    integrate_trapezoid,
    TrapezoidIntegrator,
    unwrap_near,
)
from .heading_test_math import (
    assess,
    base_translation,
    icp_2d,
    IcpResult,
    Limits,
    lever_arm_translation,
    scan_to_points,
    Stop,
)


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _degrees(value: float | None) -> float | None:
    return None if value is None else math.degrees(value)


@dataclass
class ScanSample:
    stamp: float
    ranges: list[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float

    def points(self) -> np.ndarray:
        return scan_to_points(
            self.ranges, self.angle_min, self.angle_increment,
            self.range_min, self.range_max,
        )


@dataclass
class Checkpoint:
    target_deg: float
    stamp: float
    wheel: float | None
    ekf: float | None
    compass: float | None
    field: float | None
    scan: ScanSample | None


class HeadingRecorder(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_heading_test")
        self.lock = threading.Lock()
        self.mag: list[tuple[float, float, float, float]] = []
        self.compass: list[tuple[float, float]] = []
        self.gyro: list[tuple[float, float]] = []
        self._gyro_integrator = TrapezoidIntegrator(maximum_gap=0.5)
        self.gyro_angle: tuple[float, float] | None = None
        self.wheel: list[tuple[float, float, float]] = []
        self._wheel_integrator = TrapezoidIntegrator(maximum_gap=0.5)
        self.ekf: list[tuple[float, float]] = []
        self.latest_scan: ScanSample | None = None
        self.command = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self.create_subscription(
            MagneticField, "/imu/wt901/mag_raw", self._on_mag, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/imu/compass", self._on_compass, 20)
        self.create_subscription(
            Imu, "/imu/wt901/data_raw", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_wheel, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_ekf, qos_profile_sensor_data
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_mag(self, message: MagneticField) -> None:
        vector = message.magnetic_field
        with self.lock:
            self.mag.append((self._now(), vector.x, vector.y, vector.z))

    def _on_compass(self, message: Imu) -> None:
        with self.lock:
            self.compass.append((self._now(), _yaw(message.orientation)))

    def _on_imu(self, message: Imu) -> None:
        stamp = self._now()
        rate = message.angular_velocity.z
        with self.lock:
            self.gyro.append((stamp, rate))
            self.gyro_angle = stamp, self._gyro_integrator.update(stamp, rate)

    def _on_wheel(self, message: Odometry) -> None:
        stamp = self._now()
        angular = message.twist.twist.angular.z
        with self.lock:
            self.wheel.append((stamp, self._wheel_integrator.update(stamp, angular), angular))

    def _on_ekf(self, message: Odometry) -> None:
        with self.lock:
            self.ekf.append((self._now(), _yaw(message.pose.pose.orientation)))

    def _on_scan(self, message: LaserScan) -> None:
        sample = ScanSample(
            self._now(), list(message.ranges), message.angle_min,
            message.angle_increment, message.range_min, message.range_max,
        )
        with self.lock:
            self.latest_scan = sample

    def ready(self) -> dict[str, bool]:
        with self.lock:
            return {
                "gyro (/imu/wt901/data_raw)": bool(self.gyro),
                "wheel odometry (/wheel_odom)": bool(self.wheel),
                "EKF (/odometry/filtered)": bool(self.ekf),
                "LiDAR (/scan)": self.latest_scan is not None,
            }

    def has_compass(self) -> bool:
        with self.lock:
            return bool(self.compass) and bool(self.mag)

    def checkpoint(self, target_deg: float, start: float, end: float,
                   calibration, sensor_yaw: float) -> Checkpoint:
        with self.lock:
            wheel = [value for stamp, value, _ in self.wheel if start <= stamp <= end]
            ekf = [value for stamp, value in self.ekf if start <= stamp <= end]
            compass = [value for stamp, value in self.compass if start <= stamp <= end]
            magnetic = [sample for sample in self.mag if start <= sample[0] <= end]
            scan = self.latest_scan
        fields = []
        if calibration is not None:
            for _, x_value, y_value, z_value in magnetic:
                corrected = rotate_z(
                    apply_calibration((x_value, y_value, z_value), calibration), sensor_yaw
                )
                fields.append(math.hypot(corrected[0], corrected[1]))
        if scan is not None and not start - 0.5 <= scan.stamp <= end + 0.5:
            scan = None
        return Checkpoint(
            target_deg=target_deg,
            stamp=end,
            # Wheel yaw is an unwrapped integral, so a plain mean is correct.
            wheel=statistics.mean(wheel) if wheel else None,
            ekf=circular_mean(ekf) if ekf else None,
            compass=circular_mean(compass) if compass else None,
            field=statistics.mean(fields) if fields else None,
            scan=scan,
        )

    def gyro_integral(self, start: float, end: float) -> float:
        with self.lock:
            samples = list(self.gyro)
        return integrate_trapezoid(samples, start, end)

    def latest_angle(self, source: str) -> tuple[float, float] | None:
        with self.lock:
            if source == "gyro":
                return self.gyro_angle
            return (self.wheel[-1][0], self.wheel[-1][1]) if self.wheel else None

    def publish_angular(self, angular: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.angular.z = angular
        self.command.publish(message)

    def stop(self, repetitions: int = 20) -> None:
        for _ in range(repetitions):
            self.publish_angular(0.0)
            time.sleep(0.025)

    def raw_samples(self) -> dict[str, list[tuple]]:
        with self.lock:
            return {
                "mag": list(self.mag),
                "compass": list(self.compass),
                "gyro": list(self.gyro),
                "wheel": list(self.wheel),
                "ekf": list(self.ekf),
            }


def _write_raw(output: Path, recorder: HeadingRecorder) -> None:
    headers = {
        "mag": ("monotonic_s", "x_t", "y_t", "z_t"),
        "compass": ("monotonic_s", "yaw_rad"),
        "gyro": ("monotonic_s", "angular_z_rad_s"),
        "wheel": ("monotonic_s", "integrated_yaw_rad", "angular_z_rad_s"),
        "ekf": ("monotonic_s", "yaw_rad"),
    }
    for name, samples in recorder.raw_samples().items():
        if not samples:
            continue
        with (output / f"raw_{name}.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(headers[name])
            writer.writerows(samples)


def _relative(values: list[float | None], unwrap: bool) -> list[float | None]:
    """Angles relative to the first available one, unwrapped along the turn."""
    base = next((value for value in values if value is not None), None)
    previous = base
    result = []
    for value in values:
        if value is None or base is None:
            result.append(None)
            continue
        if unwrap:
            value = unwrap_near(value, previous)
        previous = value
        result.append(value - base)
    return result


def _analyse(
    checkpoints: list[Checkpoint],
    recorder: HeadingRecorder,
    lidar_xy: tuple[float, float],
    lidar_yaw: float,
    limits: Limits,
    calibration,
) -> tuple[list[Stop], list[dict], float | None]:
    first = checkpoints[0]
    wheel = _relative([point.wheel for point in checkpoints], unwrap=False)
    ekf = _relative([point.ekf for point in checkpoints], unwrap=True)
    compass = _relative([point.compass for point in checkpoints], unwrap=True)
    reference = first.scan.points() if first.scan is not None else None
    stops, rows, translations = [], [], []
    for index, point in enumerate(checkpoints):
        gyro = recorder.gyro_integral(first.stamp, point.stamp)
        result: IcpResult | None = None
        valid = index == 0
        lidar = 0.0 if index == 0 else None
        translation = None
        if index and reference is not None and point.scan is not None:
            result = icp_2d(reference, point.scan.points(), gyro,
                            lever_arm_translation(gyro, lidar_xy, lidar_yaw))
        if result is not None:
            lidar = result.yaw
            translation = base_translation(result, lidar_xy, lidar_yaw)
            valid = (
                result.rms <= limits.lidar_max_rms_m
                and result.inlier_fraction >= limits.lidar_min_inlier_fraction
                and abs(math.degrees(result.yaw - gyro)) <= limits.lidar_max_gyro_disagreement_deg
            )
            if valid:
                translations.append(translation)
        stops.append(Stop(
            elapsed_s=point.stamp, target_deg=point.target_deg, lidar=lidar,
            lidar_valid=valid, gyro=gyro, wheel=wheel[index], ekf=ekf[index],
            compass=compass[index],
        ))

        def error(value):
            return None if value is None or lidar is None or not valid else math.degrees(value - lidar)

        rows.append({
            "target_deg": point.target_deg,
            "elapsed_s": round(point.stamp - first.stamp, 3),
            "lidar_deg": _degrees(lidar),
            "lidar_valid": valid,
            "lidar_rms_m": None if result is None else result.rms,
            "lidar_inlier_fraction": None if result is None else result.inlier_fraction,
            "pivot_translation_m": translation,
            "gyro_deg": math.degrees(gyro),
            "gyro_error_deg": error(gyro),
            "wheel_deg": _degrees(wheel[index]),
            "wheel_error_deg": error(wheel[index]),
            "ekf_deg": _degrees(ekf[index]),
            "ekf_error_deg": error(ekf[index]),
            "ekf_minus_gyro_deg": (
                None if ekf[index] is None else math.degrees(ekf[index] - gyro)
            ),
            "compass_deg": _degrees(compass[index]),
            "compass_error_deg": error(compass[index]),
            "field_deviation_percent": (
                None if point.field is None or calibration is None else
                100.0 * (point.field / calibration.horizontal_field_t - 1.0)
            ),
        })
    return stops, rows, max(translations, default=None)


def _drive_to_angle(
    recorder: HeadingRecorder,
    source: str,
    baseline: float,
    target: float,
    maximum_speed: float,
    minimum_speed: float,
    gain: float,
    tolerance: float,
    timeout: float,
) -> float:
    deadline = time.monotonic() + timeout
    initial = recorder.latest_angle(source)
    if initial is None:
        raise RuntimeError(f"{source} angle disappeared before rotation")
    direction = math.copysign(1.0, target - (initial[1] - baseline))
    moved = False
    try:
        while time.monotonic() < deadline:
            latest = recorder.latest_angle(source)
            if latest is None or time.monotonic() - latest[0] > 0.5:
                raise RuntimeError(f"{source} data became stale during rotation")
            relative = latest[1] - baseline
            error = target - relative
            if abs(error) <= tolerance or (moved and direction * error <= 0.0):
                return relative
            recorder.publish_angular(math.copysign(
                min(maximum_speed, max(minimum_speed, gain * abs(error))), error
            ))
            moved = True
            time.sleep(0.05)
    finally:
        recorder.stop()
    raise TimeoutError(
        f"did not reach {math.degrees(target):.1f} degrees within {timeout:.1f} s"
    )


def _load_robot(path: str) -> dict:
    values = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    geometry = values["bagheera"]["ros__parameters"]
    return {
        "lidar_xy": (float(geometry["lidar_x"]), float(geometry["lidar_y"])),
        "lidar_yaw": float(geometry["lidar_yaw"]),
        "wheel_track": float(geometry["wheel_track"]),
    }


def _print_report(report: dict) -> None:
    labels = {
        "lidar_reference": "LiDAR reference",
        "ekf_heading_error": "EKF heading error",
        "gyro_bias": "gyro bias",
        "gyro_scale": "gyro scale",
        "ekf_follows_gyro": "EKF follows gyro",
        "wheel_yaw": "wheel yaw",
        "pivot_translation": "pivot translation",
        "compass": "compass",
    }
    print(f"\nHeading test result: {report['verdict']}")
    drift = report["drift_acceptable"]
    print("Heading drift acceptable: " + ("unknown" if drift is None else "yes" if drift else "no"))
    for item in report["findings"]:
        value = item["value"]
        text = value if isinstance(value, str) else (
            "n/a" if value is None else f"{value:+.3f}" if abs(value) < 1 else f"{value:+.2f}"
        )
        print(f"  {item['status']:<7} {labels.get(item['check'], item['check']):<18} "
              f"{text} {item['unit']}  ({item['text']})")
        if item["advice"]:
            print(f"          -> {item['advice']}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Automatic 360 degree heading test with a LiDAR reference"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="required acknowledgement that the robot may rotate automatically",
    )
    parser.add_argument("--clockwise", action="store_true")
    parser.add_argument("--still", type=float, default=10.0,
                        help="standing time before and after the turn (gyro bias)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="stationary time at every 45 degree stop")
    parser.add_argument("--window", type=float, default=1.5,
                        help="stable tail of each stop used for statistics")
    parser.add_argument("--drive-source", choices=("gyro", "wheel"), default="gyro",
                        help="angle that closes the turn loop")
    parser.add_argument("--maximum-speed", type=float, default=0.30)
    parser.add_argument("--minimum-speed", type=float, default=0.18)
    parser.add_argument("--angular-gain", type=float, default=0.8)
    parser.add_argument("--angle-tolerance-deg", type=float, default=2.5)
    parser.add_argument("--step-timeout", type=float, default=15.0)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--robot-config", default=None,
                        help="robot.yaml with lidar_x/y/yaw and wheel_track")
    parser.add_argument("--calibration",
                        default="/bagheera_ws/maps/compass_calibration.yaml",
                        help="compass calibration; the compass is skipped without it")
    parser.add_argument("--sensor-yaw", type=float, default=-math.pi / 2.0)
    parser.add_argument("--output", default=None)
    parser.add_argument("--discovery-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("automatic motion is disabled unless --execute is supplied")
    if args.settle <= 0.0 or not 0.0 < args.window <= args.settle:
        parser.error("window must be positive and no longer than settle")
    if args.still < args.window:
        parser.error("still must be at least as long as window")
    if not 0.0 < args.minimum_speed <= args.maximum_speed <= 0.5:
        parser.error("speeds must satisfy 0 < minimum <= maximum <= 0.5 rad/s")
    if args.angle_tolerance_deg <= 0.0 or args.step_timeout <= 0.0:
        parser.error("angle tolerance and step timeout must be positive")

    robot = _load_robot(args.robot_config or str(
        Path(get_package_share_directory("bagheera_base")) / "config" / "robot.yaml"
    ))
    try:
        calibration = load_calibration(args.calibration)
    except (OSError, ValueError, KeyError, TypeError):
        calibration = None
    limits = Limits()
    output = Path(args.output) if args.output else Path(
        "/bagheera_ws/maps" if Path("/bagheera_ws/maps").exists() else "."
    ) / f"heading_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    recorder = HeadingRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    checkpoints: list[Checkpoint] = []
    report: dict | None = None
    failure_reason = None
    try:
        deadline = time.monotonic() + args.discovery_timeout
        while time.monotonic() < deadline:
            if all(recorder.ready().values()) and recorder.command.get_subscription_count():
                break
            time.sleep(0.2)
        missing = [name for name, available in recorder.ready().items() if not available]
        if missing:
            raise RuntimeError("no data from: " + ", ".join(missing))
        if recorder.command.get_subscription_count() == 0:
            raise RuntimeError("/cmd_vel_tuning has no subscriber (twist_mux running?)")
        compass_note = "with compass" if recorder.has_compass() and calibration else "without compass"

        direction = -1.0 if args.clockwise else 1.0
        targets = [direction * value for value in range(45, 361, 45)]
        print(f"\nAll sensors and the tuning drive path are ready ({compass_note}).")
        print(f"Bagheera stands still for {args.still:.0f} s, turns once through 360 degrees "
              f"with a stop every 45 degrees and stands still for {args.still:.0f} s again.")
        print("Keep the area clear and do not walk around the robot (LiDAR reference).")
        print("Abort any time with Ctrl-C; the node then actively sends zero commands.")
        for remaining in range(max(0, args.countdown), 0, -1):
            print(f"Start in {remaining} ...", flush=True)
            time.sleep(1.0)

        recorder.stop()
        still_start = time.monotonic()
        time.sleep(args.still)
        still_before = (still_start, time.monotonic())
        checkpoints.append(recorder.checkpoint(
            0.0, still_before[1] - args.window, still_before[1], calibration, args.sensor_yaw
        ))
        baseline = recorder.latest_angle(args.drive_source)
        if baseline is None:
            raise RuntimeError(f"{args.drive_source} angle unavailable at start")
        for target_deg in targets:
            reached = _drive_to_angle(
                recorder, args.drive_source, baseline[1], math.radians(target_deg),
                args.maximum_speed, args.minimum_speed, args.angular_gain,
                math.radians(args.angle_tolerance_deg), args.step_timeout,
            )
            time.sleep(args.settle)
            end = time.monotonic()
            checkpoints.append(recorder.checkpoint(
                target_deg, end - args.window, end, calibration, args.sensor_yaw
            ))
            print(f"  stop {target_deg:+4.0f} deg ({args.drive_source} "
                  f"{math.degrees(reached):+.1f} deg)", flush=True)
        after_start = time.monotonic()
        time.sleep(args.still)
        still_after = (after_start, time.monotonic())

        duration = sum(end - start for start, end in (still_before, still_after))
        still_rate = sum(
            recorder.gyro_integral(start, end) for start, end in (still_before, still_after)
        ) / duration
        print("Evaluating scans ...", flush=True)
        stops, rows, pivot = _analyse(
            checkpoints, recorder, robot["lidar_xy"], robot["lidar_yaw"], limits, calibration
        )
        report = assess(stops, still_rate, duration, pivot, robot["wheel_track"], limits)
        report.update({
            "complete": True,
            "created": datetime.now().isoformat(timespec="seconds"),
            "direction": "clockwise" if args.clockwise else "counter-clockwise",
            "drive_source": args.drive_source,
            "robot_lidar_xy_m": list(robot["lidar_xy"]),
            "robot_lidar_yaw_rad": robot["lidar_yaw"],
            "compass_calibration": args.calibration if calibration else None,
        })
        with (output / "checkpoints.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        _print_report(report)
    except (EOFError, KeyboardInterrupt):
        failure_reason = "aborted by the user"
        print("\nTest aborted; saving the raw data.")
    except Exception as error:
        failure_reason = str(error)
        print(f"\nTest failed: {error}")
    finally:
        recorder.stop(repetitions=30)
        _write_raw(output, recorder)
        if report is None:
            report = {"complete": False, "failure_reason": failure_reason,
                      "created": datetime.now().isoformat(timespec="seconds")}
        (output / "report.yaml").write_text(
            yaml.safe_dump(report, sort_keys=False), encoding="utf-8"
        )
        scans = [
            {"target_deg": point.target_deg, "angle_min": point.scan.angle_min,
             "angle_increment": point.scan.angle_increment, "ranges": point.scan.ranges}
            for point in checkpoints if point.scan is not None
        ]
        with gzip.open(output / "scans.json.gz", "wt", encoding="utf-8") as handle:
            json.dump(scans, handle, allow_nan=True)
        print(f"Report saved: {output}")
        executor.shutdown()
        recorder.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=2.0)
    if failure_reason is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
