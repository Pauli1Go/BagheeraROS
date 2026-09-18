"""Interactive, non-driving characterization of Bagheera's heading sensors."""

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

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, MagneticField
from std_msgs.msg import Bool
import yaml

from .compass_math import apply_calibration, load_calibration, rotate_z
from .compass_test_math import (
    circular_mean,
    circular_std,
    integrate_trapezoid,
    scan_yaw_from_signatures,
    TrapezoidIntegrator,
    unwrap_near,
)


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _mean(values: list[float]) -> float | None:
    return statistics.mean(values) if values else None


def _degrees(value: float | None) -> float | None:
    return None if value is None else math.degrees(value)


def _error_degrees(actual: float | None, expected: float) -> float | None:
    return None if actual is None else math.degrees(actual - expected)


@dataclass
class Checkpoint:
    phase: str
    expected_deg: float
    stamp: float
    compass: float | None
    compass_std: float | None
    field: float | None
    mag_z: float | None
    valid_fraction: float | None
    wheel: float | None
    ekf: float | None
    scan: list[float | None] | None


class HeadingRecorder(Node):
    def __init__(self, scan_bins: int) -> None:
        super().__init__("bagheera_compass_test")
        self.lock = threading.Lock()
        self.scan_bins = scan_bins
        self.mag: list[tuple[float, float, float, float]] = []
        self.compass: list[tuple[float, float]] = []
        self.valid: list[tuple[float, bool]] = []
        self.gyro: list[tuple[float, float]] = []
        self.wheel: list[tuple[float, float, float]] = []
        self.wheel_unwrapped: list[tuple[float, float]] = []
        self._wheel_integrator = TrapezoidIntegrator(maximum_gap=0.5)
        self.ekf: list[tuple[float, float]] = []
        self.latest_scan: tuple[float, list[float | None]] | None = None
        self.command = self.create_publisher(TwistStamped, "/cmd_vel_tuning", 10)
        self.create_subscription(
            MagneticField, "/imu/wt901/mag_raw", self._on_mag, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/imu/compass", self._on_compass, 20)
        self.create_subscription(Bool, "/imu/compass/valid", self._on_valid, 20)
        self.create_subscription(
            Imu, "/imu/wt901/data_raw", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_wheel, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_ekf, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )

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

    def _on_valid(self, message: Bool) -> None:
        with self.lock:
            self.valid.append((self._now(), bool(message.data)))

    def _on_imu(self, message: Imu) -> None:
        with self.lock:
            self.gyro.append((self._now(), message.angular_velocity.z))

    def _on_wheel(self, message: Odometry) -> None:
        stamp = self._now()
        angular = message.twist.twist.angular.z
        with self.lock:
            integrated = self._wheel_integrator.update(stamp, angular)
            self.wheel.append((stamp, integrated, angular))
            self.wheel_unwrapped.append((stamp, integrated))

    def _on_ekf(self, message: Odometry) -> None:
        with self.lock:
            self.ekf.append((self._now(), _yaw(message.pose.pose.orientation)))

    def _on_scan(self, message: LaserScan) -> None:
        signature: list[float | None] = [None] * self.scan_bins
        for index, distance in enumerate(message.ranges):
            if (
                not math.isfinite(distance)
                or not message.range_min <= distance <= message.range_max
            ):
                continue
            angle = message.angle_min + index * message.angle_increment
            target = int(
                ((angle + math.pi) % (2.0 * math.pi))
                / (2.0 * math.pi)
                * self.scan_bins
            )
            previous = signature[target]
            signature[target] = distance if previous is None else min(previous, distance)
        with self.lock:
            self.latest_scan = self._now(), signature

    def ready(self) -> dict[str, bool]:
        with self.lock:
            return {
                "mag": bool(self.mag),
                "compass": bool(self.compass),
                "valid": bool(self.valid),
                "gyro": bool(self.gyro),
                "wheel": bool(self.wheel),
                "ekf": bool(self.ekf),
                "scan": self.latest_scan is not None,
            }

    def checkpoint(
        self,
        phase: str,
        expected_deg: float,
        start: float,
        end: float,
        calibration,
        sensor_yaw: float,
    ) -> Checkpoint:
        with self.lock:
            compass_values = [value for stamp, value in self.compass if start <= stamp <= end]
            valid_values = [value for stamp, value in self.valid if start <= stamp <= end]
            wheel_values = [
                value for stamp, value in self.wheel_unwrapped if start <= stamp <= end
            ]
            ekf_values = [value for stamp, value in self.ekf if start <= stamp <= end]
            magnetic = [sample for sample in self.mag if start <= sample[0] <= end]
            latest_scan = self.latest_scan
        fields = []
        z_values = []
        for _, x_value, y_value, z_value in magnetic:
            corrected = rotate_z(
                apply_calibration((x_value, y_value, z_value), calibration), sensor_yaw
            )
            fields.append(math.hypot(corrected[0], corrected[1]))
            z_values.append(corrected[2])
        scan = None
        if latest_scan is not None and end - latest_scan[0] <= 1.0:
            scan = latest_scan[1]
        return Checkpoint(
            phase=phase,
            expected_deg=expected_deg,
            stamp=end,
            compass=circular_mean(compass_values) if compass_values else None,
            compass_std=circular_std(compass_values) if compass_values else None,
            field=_mean(fields),
            mag_z=_mean(z_values),
            valid_fraction=(sum(valid_values) / len(valid_values)) if valid_values else None,
            wheel=circular_mean(wheel_values) if wheel_values else None,
            ekf=circular_mean(ekf_values) if ekf_values else None,
            scan=scan,
        )

    def gyro_integral(self, start: float, end: float) -> float:
        with self.lock:
            samples = list(self.gyro)
        return integrate_trapezoid(samples, start, end)

    def latest_wheel_angle(self) -> tuple[float, float] | None:
        with self.lock:
            return self.wheel_unwrapped[-1] if self.wheel_unwrapped else None

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

    def phase_quality(self, start: float, end: float, calibration, sensor_yaw: float) -> dict:
        with self.lock:
            valid = [value for stamp, value in self.valid if start <= stamp <= end]
            magnetic = [sample for sample in self.mag if start <= sample[0] <= end]
        fields = []
        for _, x_value, y_value, z_value in magnetic:
            corrected = rotate_z(
                apply_calibration((x_value, y_value, z_value), calibration),
                sensor_yaw,
            )
            fields.append(math.hypot(corrected[0], corrected[1]))
        return {
            "motion_valid_percent": (
                None if not valid else 100.0 * sum(valid) / len(valid)
            ),
            "motion_field_min_ut": None if not fields else min(fields) * 1.0e6,
            "motion_field_median_ut": (
                None if not fields else statistics.median(fields) * 1.0e6
            ),
            "motion_field_max_ut": None if not fields else max(fields) * 1.0e6,
            "motion_max_field_deviation_percent": (
                None
                if not fields
                else 100.0
                * max(
                    abs(value / calibration.horizontal_field_t - 1.0)
                    for value in fields
                )
            ),
        }

    def raw_samples(self) -> dict[str, list[tuple]]:
        with self.lock:
            return {
                "mag": list(self.mag),
                "compass": list(self.compass),
                "valid": list(self.valid),
                "gyro": list(self.gyro),
                "wheel": list(self.wheel),
                "ekf": list(self.ekf),
            }


def _write_raw(output: Path, recorder: HeadingRecorder) -> None:
    headers = {
        "mag": ("monotonic_s", "x_t", "y_t", "z_t"),
        "compass": ("monotonic_s", "yaw_rad"),
        "valid": ("monotonic_s", "valid"),
        "gyro": ("monotonic_s", "angular_z_rad_s"),
        "wheel": ("monotonic_s", "integrated_yaw_rad", "angular_z_rad_s"),
        "ekf": ("monotonic_s", "yaw_rad"),
    }
    for name, samples in recorder.raw_samples().items():
        with (output / f"raw_{name}.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(headers[name])
            writer.writerows(samples)


def _analyse_phase(
    checkpoints: list[Checkpoint],
    recorder: HeadingRecorder,
    calibration,
    sensor_yaw: float,
    scan_increment: float,
) -> tuple[list[dict], dict]:
    rows = []
    first = checkpoints[0]
    base = {}
    base_expected = {}
    previous = {}
    for name in ("compass", "wheel", "ekf"):
        reference = next(
            (point for point in checkpoints if getattr(point, name) is not None),
            None,
        )
        base[name] = None if reference is None else getattr(reference, name)
        previous[name] = base[name]
        base_expected[name] = (
            None if reference is None else math.radians(reference.expected_deg)
        )
    for checkpoint in checkpoints:
        expected = math.radians(checkpoint.expected_deg)
        relative = {}
        expected_relative = {}
        for name in ("compass", "wheel", "ekf"):
            value = getattr(checkpoint, name)
            if value is None or base[name] is None:
                relative[name] = None
                expected_relative[name] = None
                continue
            if previous[name] is not None:
                value = unwrap_near(value, previous[name])
            previous[name] = value
            relative[name] = value - base[name]
            expected_relative[name] = expected - base_expected[name]
        gyro = recorder.gyro_integral(first.stamp, checkpoint.stamp)
        lidar = None
        if first.scan is not None and checkpoint.scan is not None:
            lidar = scan_yaw_from_signatures(
                first.scan, checkpoint.scan, expected, scan_increment
            )
        rows.append({
            "phase": checkpoint.phase,
            "expected_deg": checkpoint.expected_deg,
            "compass_deg": _degrees(relative["compass"]),
            "compass_error_deg": _error_degrees(
                relative["compass"], expected_relative["compass"]
            ),
            "compass_std_deg": _degrees(checkpoint.compass_std),
            "gyro_deg": math.degrees(gyro),
            "gyro_error_deg": math.degrees(gyro - expected),
            "wheel_deg": _degrees(relative["wheel"]),
            "wheel_error_deg": _error_degrees(
                relative["wheel"], expected_relative["wheel"]
            ),
            "ekf_deg": _degrees(relative["ekf"]),
            "ekf_error_deg": _error_degrees(
                relative["ekf"], expected_relative["ekf"]
            ),
            "lidar_deg": None if lidar is None else math.degrees(lidar[0]),
            "lidar_error_deg": (
                None if lidar is None else math.degrees(lidar[0] - expected)
            ),
            "lidar_residual_m": None if lidar is None else lidar[1],
            "lidar_pairs": None if lidar is None else lidar[2],
            "field_ut": (
                None if checkpoint.field is None else checkpoint.field * 1.0e6
            ),
            "field_deviation_percent": (
                None if checkpoint.field is None else
                100.0 * (checkpoint.field / calibration.horizontal_field_t - 1.0)
            ),
            "mag_z_ut": (
                None if checkpoint.mag_z is None else checkpoint.mag_z * 1.0e6
            ),
            "valid_percent": (
                None if checkpoint.valid_fraction is None else 100.0 * checkpoint.valid_fraction
            ),
        })
    def maximum_error(name: str) -> float | None:
        values = [abs(row[name]) for row in rows if row[name] is not None]
        return max(values, default=None)

    valid = [row["valid_percent"] for row in rows if row["valid_percent"] is not None]
    summary = {
        "phase": first.phase,
        "max_compass_error_deg": maximum_error("compass_error_deg"),
        "max_gyro_error_deg": maximum_error("gyro_error_deg"),
        "max_wheel_error_deg": maximum_error("wheel_error_deg"),
        "max_ekf_error_deg": maximum_error("ekf_error_deg"),
        "max_lidar_error_deg": maximum_error("lidar_error_deg"),
        "max_field_deviation_percent": maximum_error("field_deviation_percent"),
        "minimum_valid_percent": min(valid, default=None),
        "return_error_deg": rows[-1]["compass_error_deg"],
    }
    summary.update(
        recorder.phase_quality(
            checkpoints[0].stamp,
            checkpoints[-1].stamp,
            calibration,
            sensor_yaw,
        )
    )
    return rows, summary


def _write_results(
    output: Path,
    rows: list[dict],
    summary: dict,
    checkpoints: list[Checkpoint],
) -> None:
    if rows:
        with (output / "checkpoints.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    (output / "report.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8"
    )
    scans = [
        {"phase": item.phase, "expected_deg": item.expected_deg, "ranges": item.scan}
        for item in checkpoints if item.scan is not None
    ]
    with gzip.open(output / "scans.json.gz", "wt", encoding="utf-8") as handle:
        json.dump(scans, handle)


def _drive_to_angle(
    recorder: HeadingRecorder,
    baseline: float,
    target: float,
    maximum_speed: float,
    minimum_speed: float,
    gain: float,
    tolerance: float,
    timeout: float,
) -> float:
    deadline = time.monotonic() + timeout
    initial = recorder.latest_wheel_angle()
    if initial is None:
        raise RuntimeError("wheel odometry disappeared before rotation")
    initial_error = target - (initial[1] - baseline)
    direction = math.copysign(1.0, initial_error)
    moved = False
    try:
        while time.monotonic() < deadline:
            latest = recorder.latest_wheel_angle()
            if latest is None or time.monotonic() - latest[0] > 0.5:
                raise RuntimeError("wheel odometry became stale during rotation")
            relative = latest[1] - baseline
            error = target - relative
            if abs(error) <= tolerance or (moved and direction * error <= 0.0):
                return relative
            angular = math.copysign(
                min(maximum_speed, max(minimum_speed, gain * abs(error))),
                error,
            )
            recorder.publish_angular(angular)
            moved = True
            time.sleep(0.05)
    finally:
        recorder.stop()
    raise TimeoutError(
        f"did not reach {math.degrees(target):.1f} degrees within {timeout:.1f} s"
    )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Automatic 360 degree compass/gyro/odometry/LiDAR test"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="required acknowledgement that the robot may rotate automatically",
    )
    parser.add_argument("--clockwise", action="store_true")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="stationary sampling time at every 45 degree stop")
    parser.add_argument("--window", type=float, default=1.5,
                        help="stable tail of each stop used for statistics")
    parser.add_argument("--maximum-speed", type=float, default=0.30)
    parser.add_argument("--minimum-speed", type=float, default=0.18)
    parser.add_argument("--angular-gain", type=float, default=0.8)
    parser.add_argument("--angle-tolerance-deg", type=float, default=2.5)
    parser.add_argument("--step-timeout", type=float, default=15.0)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--calibration",
                        default="/bagheera_ws/maps/compass_calibration.yaml")
    parser.add_argument("--sensor-yaw", type=float, default=-math.pi / 2.0)
    parser.add_argument("--output", default=None)
    parser.add_argument("--discovery-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("automatic motion is disabled unless --execute is supplied")
    if args.settle <= 0.0 or not 0.0 < args.window <= args.settle:
        parser.error("window must be positive and no longer than settle")
    if not 0.0 < args.minimum_speed <= args.maximum_speed <= 0.5:
        parser.error("speeds must satisfy 0 < minimum <= maximum <= 0.5 rad/s")
    if args.angle_tolerance_deg <= 0.0 or args.step_timeout <= 0.0:
        parser.error("angle tolerance and step timeout must be positive")

    calibration = load_calibration(args.calibration)
    output = Path(args.output) if args.output else Path(
        "/bagheera_ws/maps" if Path("/bagheera_ws/maps").exists() else "."
    ) / f"compass_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    recorder = HeadingRecorder(scan_bins=720)
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    checkpoints: list[Checkpoint] = []
    rows: list[dict] = []
    summaries = []
    interrupted = False
    failure_reason = None
    try:
        deadline = time.monotonic() + args.discovery_timeout
        while time.monotonic() < deadline:
            state = recorder.ready()
            sensors_ready = all(
                available for name, available in state.items() if name != "compass"
            )
            command_ready = recorder.command.get_subscription_count() > 0
            if sensors_ready and command_ready:
                break
            time.sleep(0.2)
        ready = recorder.ready()
        missing = [
            name
            for name, available in ready.items()
            if name != "compass" and not available
        ]
        if missing:
            raise RuntimeError("no data from: " + ", ".join(missing))
        if recorder.command.get_subscription_count() == 0:
            raise RuntimeError("/cmd_vel_tuning has no subscriber")
        if not ready["compass"]:
            print(
                "WARNUNG: /imu/compass liefert aktuell nichts; der Test laeuft "
                "weiter und bewertet valid/raw magnetometer."
            )

        direction = -1.0 if args.clockwise else 1.0
        phase_name = "motor_cw" if args.clockwise else "motor_ccw"
        targets = [direction * value for value in range(0, 361, 45)]
        print("\nAlle Sensoren und der Tuning-Fahrpfad sind bereit.")
        print("Bagheera dreht automatisch einmal um 360 Grad und stoppt alle 45 Grad.")
        print("Abbruch jederzeit mit Ctrl-C; der Node sendet danach aktiv Nullbefehle.")
        for remaining in range(max(0, args.countdown), 0, -1):
            print(f"Start in {remaining} ...", flush=True)
            time.sleep(1.0)

        recorder.stop()
        time.sleep(args.settle)
        first_wheel = recorder.latest_wheel_angle()
        if first_wheel is None:
            raise RuntimeError("wheel odometry unavailable at start")
        wheel_baseline = first_wheel[1]
        phase_points = []
        for index, expected_deg in enumerate(targets):
            if index:
                reached = _drive_to_angle(
                    recorder,
                    wheel_baseline,
                    math.radians(expected_deg),
                    args.maximum_speed,
                    args.minimum_speed,
                    args.angular_gain,
                    math.radians(args.angle_tolerance_deg),
                    args.step_timeout,
                )
                print(
                    f"Ziel {expected_deg:+.0f} deg, Rad-Odometrie "
                    f"{math.degrees(reached):+.1f} deg"
                )
                time.sleep(args.settle)
            end = time.monotonic()
            point = recorder.checkpoint(
                phase_name,
                expected_deg,
                end - args.window,
                end,
                calibration,
                args.sensor_yaw,
            )
            phase_points.append(point)
            checkpoints.append(point)
            field_text = (
                "n/a" if point.field is None else f"{point.field * 1e6:.1f} uT"
            )
            valid_text = (
                "n/a" if point.valid_fraction is None else f"{point.valid_fraction:.0%}"
            )
            print(f"  Messpunkt: Feld {field_text}, Compass valid {valid_text}")

        phase_rows, phase_summary = _analyse_phase(
            phase_points,
            recorder,
            calibration,
            args.sensor_yaw,
            2.0 * math.pi / recorder.scan_bins,
        )
        rows.extend(phase_rows)
        summaries.append(phase_summary)

        failures = []
        for phase_summary in summaries:
            phase = phase_summary["phase"]
            limits = (
                ("max_compass_error_deg", 5.0),
                ("max_field_deviation_percent", 10.0),
                ("motion_max_field_deviation_percent", 10.0),
                ("return_error_deg", 3.0),
            )
            for field, limit in limits:
                value = phase_summary[field]
                if value is not None and abs(value) > limit:
                    failures.append(f"{phase}: {field}={value:.2f} exceeds {limit:.2f}")
            valid = phase_summary["minimum_valid_percent"]
            if valid is not None and valid < 95.0:
                failures.append(
                    f"{phase}: minimum_valid_percent={valid:.2f} is below 95.00"
                )
            motion_valid = phase_summary["motion_valid_percent"]
            if motion_valid is not None and motion_valid < 95.0:
                failures.append(
                    f"{phase}: motion_valid_percent={motion_valid:.2f} is below 95.00"
                )
        summary = {
            "complete": not interrupted,
            "created": datetime.now().isoformat(timespec="seconds"),
            "assessment": "PASS" if not failures else "FAIL",
            "failures": failures,
            "calibration_file": args.calibration,
            "reference_horizontal_field_ut": calibration.horizontal_field_t * 1.0e6,
            "phases": summaries,
            "recommended_limits": {
                "max_heading_error_deg": 5.0,
                "max_return_error_deg": 3.0,
                "max_field_deviation_percent": 10.0,
                "minimum_valid_percent": 95.0,
            },
        }
        _write_raw(output, recorder)
        _write_results(output, rows, summary, checkpoints)
        print(f"\nAuswertung gespeichert: {output}")
        print(f"Ergebnis: {summary['assessment']}")
        for failure in failures:
            print(f"  - {failure}")
        for phase in summaries:
            print(
                f"  {phase['phase']}: max Compass-Fehler "
                f"{phase['max_compass_error_deg']} deg, max Feldabweichung "
                f"{phase['max_field_deviation_percent']} %"
            )
    except (EOFError, KeyboardInterrupt):
        interrupted = True
        print("\nTest abgebrochen; Rohdaten werden gespeichert.")
    except Exception as error:
        interrupted = True
        failure_reason = str(error)
        print(f"\nTest fehlgeschlagen: {error}")
    finally:
        recorder.stop(repetitions=30)
        _write_raw(output, recorder)
        if interrupted:
            if not rows and len(checkpoints) >= 2:
                partial_rows, partial_summary = _analyse_phase(
                    checkpoints,
                    recorder,
                    calibration,
                    args.sensor_yaw,
                    2.0 * math.pi / recorder.scan_bins,
                )
                rows.extend(partial_rows)
                summaries.append(partial_summary)
            partial = {
                "complete": False,
                "created": datetime.now().isoformat(timespec="seconds"),
                "failure_reason": failure_reason,
                "phases": summaries,
            }
            _write_results(output, rows, partial, checkpoints)
        executor.shutdown()
        recorder.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=2.0)
    if interrupted:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
