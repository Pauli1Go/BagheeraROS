"""ROS-free analysis for the 360-degree heading test (see heading_test.py).

The LiDAR is the reference: every stop is scan-matched against the first scan
with a 2D ICP, which measures the true rotation independently of all motion
sensors. Gyro, wheel, EKF and compass headings are then compared with it and
their errors are split into causes, each graded OK / WARN / FAIL.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

OK, WARN, FAIL, INFO, UNKNOWN = "OK", "WARN", "FAIL", "INFO", "UNKNOWN"
_SEVERITY = {OK: 0, INFO: 0, UNKNOWN: 1, WARN: 2, FAIL: 3}


@dataclass(frozen=True)
class Limits:
    """OK up to the first value, WARN up to the second, FAIL above."""

    ekf_error_deg: tuple[float, float] = (2.0, 5.0)
    gyro_bias_deg_min: tuple[float, float] = (0.5, 2.0)
    gyro_scale_percent: tuple[float, float] = (0.5, 1.5)
    ekf_vs_gyro_deg: tuple[float, float] = (1.0, 3.0)
    # Bagheera's pivot centre really wanders ~10 cm on its casters; only more
    # than that points to slip or a wrong LiDAR offset.
    pivot_translation_m: tuple[float, float] = (0.12, 0.17)
    compass_error_deg: tuple[float, float] = (5.0, 10.0)
    # A stop counts as a valid LiDAR reference only with enough overlap.
    lidar_max_rms_m: float = 0.05
    lidar_min_inlier_fraction: float = 0.5
    lidar_max_gyro_disagreement_deg: float = 15.0


@dataclass
class IcpResult:
    yaw: float
    tx: float
    ty: float
    rms: float
    inlier_fraction: float
    pairs: int


def rotation(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine], [sine, cosine]])


def scan_to_points(
    ranges: list[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_range: float = 8.0,
) -> np.ndarray:
    """Valid returns of a LaserScan as an (N, 2) array in the LiDAR frame."""
    values = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(values.size)
    usable = np.isfinite(values) & (values >= range_min) & (values <= min(range_max, max_range))
    return np.column_stack((values[usable] * np.cos(angles[usable]),
                            values[usable] * np.sin(angles[usable])))


def lever_arm_translation(
    yaw: float, lidar_xy: tuple[float, float], lidar_yaw: float
) -> np.ndarray:
    """Translation of the ICP model after a pure pivot of base_link by yaw.

    The model maps current LiDAR points into the first LiDAR frame:
    p_ref = R(yaw) p_cur + t, with t = R(-lidar_yaw) (R(yaw) - I) l.
    """
    lever = np.asarray(lidar_xy, dtype=float)
    return rotation(-lidar_yaw) @ ((rotation(yaw) - np.eye(2)) @ lever)


def base_translation(
    result: IcpResult, lidar_xy: tuple[float, float], lidar_yaw: float
) -> float:
    """Distance base_link moved during the pivot, as implied by the ICP."""
    measured = rotation(lidar_yaw) @ np.array([result.tx, result.ty])
    lever = np.asarray(lidar_xy, dtype=float)
    return float(np.linalg.norm(measured - (rotation(result.yaw) - np.eye(2)) @ lever))


def _normals(points: np.ndarray, max_gap: float = 0.15) -> tuple[np.ndarray, np.ndarray]:
    """Unit normals from the scan-order neighbours; mask of points that have one."""
    previous = np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0)
    tangent = following - previous
    gaps = np.maximum(np.linalg.norm(points - previous, axis=1),
                      np.linalg.norm(following - points, axis=1))
    length = np.linalg.norm(tangent, axis=1)
    usable = (gaps <= max_gap) & (length > 1e-6)
    normals = np.zeros_like(points)
    normals[usable, 0] = -tangent[usable, 1] / length[usable]
    normals[usable, 1] = tangent[usable, 0] / length[usable]
    return normals, usable


def icp_2d(
    reference: np.ndarray,
    current: np.ndarray,
    yaw0: float,
    translation0: np.ndarray,
    distances: tuple[float, ...] = (0.5, 0.35, 0.25, 0.15, 0.10, 0.08, 0.06, 0.05),
    iterations_per_distance: int = 3,
) -> IcpResult | None:
    """Trimmed point-to-line ICP: find R, t with R current + t ≈ reference.

    Both point sets must be in scan order (as from scan_to_points), so the
    reference normals can be taken from neighbouring returns.
    """
    normals, has_normal = _normals(reference)
    reference, normals = reference[has_normal], normals[has_normal]
    if len(reference) < 20 or len(current) < 20:
        return None
    yaw = float(yaw0)
    translation = np.asarray(translation0, dtype=float).copy()
    inliers = np.zeros(len(current), dtype=bool)
    squared = np.zeros(len(current))
    for maximum in distances:
        for _ in range(iterations_per_distance):
            moved = current @ rotation(yaw).T + translation
            difference = moved[:, None, :] - reference[None, :, :]
            distance = np.einsum("ijk,ijk->ij", difference, difference)
            nearest = distance.argmin(axis=1)
            squared = distance[np.arange(len(current)), nearest]
            inliers = squared <= maximum * maximum
            if inliers.sum() < 10:
                return None
            source = moved[inliers]
            normal = normals[nearest[inliers]]
            residual = np.einsum("ij,ij->i", source - reference[nearest[inliers]], normal)
            # Linearized rotation about the origin: d(R p)/d(yaw) = (-p_y, p_x).
            jacobian = np.column_stack((
                normal[:, 1] * source[:, 0] - normal[:, 0] * source[:, 1],
                normal[:, 0],
                normal[:, 1],
            ))
            step, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
            step_rotation = rotation(float(step[0]))
            yaw += float(step[0])
            translation = step_rotation @ translation + step[1:]
    final = squared[inliers]
    return IcpResult(
        yaw=yaw,
        tx=float(translation[0]),
        ty=float(translation[1]),
        rms=float(math.sqrt(final.mean())) if final.size else math.inf,
        inlier_fraction=float(inliers.mean()),
        pairs=int(inliers.sum()),
    )


def grade(value: float | None, limits: tuple[float, float]) -> str:
    if value is None or not math.isfinite(value):
        return UNKNOWN
    magnitude = abs(value)
    if magnitude <= limits[0]:
        return OK
    return WARN if magnitude <= limits[1] else FAIL


def fit_proportional(errors: list[float], angles: list[float]) -> float | None:
    """Least-squares s in error = s * angle (through the origin)."""
    denominator = sum(angle * angle for angle in angles)
    if denominator < math.radians(90.0) ** 2:
        return None
    return sum(error * angle for error, angle in zip(errors, angles)) / denominator


@dataclass
class Stop:
    """One stationary checkpoint, all angles relative to the first stop [rad]."""

    elapsed_s: float
    target_deg: float
    lidar: float | None
    lidar_valid: bool
    gyro: float
    wheel: float | None
    ekf: float | None
    compass: float | None = None


def _degrees(value: float | None) -> float | None:
    return None if value is None else math.degrees(value)


def _finding(name: str, status: str, value, unit: str, text: str, advice: str = "") -> dict:
    return {"check": name, "status": status, "value": value, "unit": unit,
            "text": text, "advice": advice}


def assess(
    stops: list[Stop],
    still_gyro_rate: float | None,
    still_duration_s: float,
    max_pivot_translation_m: float | None,
    wheel_track_m: float | None,
    limits: Limits = Limits(),
) -> dict:
    """Grade the heading sources against the LiDAR reference."""
    valid = [stop for stop in stops[1:] if stop.lidar_valid and stop.lidar is not None]
    final = stops[-1] if stops and stops[-1].lidar_valid else None
    findings: list[dict] = []
    reference_ok = final is not None and len(valid) >= max(2, (len(stops) - 1) // 2)

    findings.append(_finding(
        "lidar_reference",
        OK if reference_ok else FAIL,
        f"{len(valid)}/{max(0, len(stops) - 1)}", "stops",
        "LiDAR scan matching usable as reference" if reference_ok else
        "LiDAR scan matching is not reliable enough to judge the other sensors",
        "" if reference_ok else
        "Repeat the test in a place with several walls or furniture within 6 m "
        "and nobody walking around the robot.",
    ))

    ekf_error = None
    if final is not None and final.ekf is not None:
        ekf_error = final.ekf - final.lidar
    ekf_status = grade(_degrees(ekf_error), limits.ekf_error_deg) if reference_ok else UNKNOWN
    findings.append(_finding(
        "ekf_heading_error", ekf_status, _degrees(ekf_error), "deg per turn",
        "EKF heading error after the full turn (what localization has to correct)",
    ))

    bias_deg_min = None if still_gyro_rate is None else math.degrees(still_gyro_rate) * 60.0
    bias_status = grade(bias_deg_min, limits.gyro_bias_deg_min)
    findings.append(_finding(
        "gyro_bias", bias_status, bias_deg_min, "deg/min",
        f"WT901 rate while standing still for {still_duration_s:.0f} s",
        "" if bias_status in (OK, UNKNOWN) else
        "The gyro offset is measured in the first 4 s after start and after every "
        "dock wake-up. Restart with the robot completely still; if the drift "
        "returns, the sensor warms up or is loose.",
    ))

    scale = None
    if reference_ok and valid:
        bias = still_gyro_rate or 0.0
        errors = [stop.gyro - stop.lidar - bias * (stop.elapsed_s - stops[0].elapsed_s)
                  for stop in valid]
        scale = fit_proportional(errors, [stop.lidar for stop in valid])
    scale_percent = None if scale is None else 100.0 * scale
    scale_status = grade(scale_percent, limits.gyro_scale_percent) if reference_ok else UNKNOWN
    findings.append(_finding(
        "gyro_scale", scale_status, scale_percent, "%",
        "WT901 rotation per true rotation, bias removed"
        + ("" if scale_percent is None else f" ({3.6 * scale_percent:+.1f} deg per turn)"),
        "" if scale_status in (OK, UNKNOWN) else
        "There is no scale parameter for the WT901. Check that it is mounted "
        "level and firmly; a large scale error points to a faulty sensor.",
    ))

    ekf_gyro = [stop.ekf - stop.gyro for stop in stops[1:] if stop.ekf is not None]
    ekf_gyro_max = max(ekf_gyro, key=abs, default=None)
    ekf_gyro_status = grade(_degrees(ekf_gyro_max), limits.ekf_vs_gyro_deg)
    findings.append(_finding(
        "ekf_follows_gyro", ekf_gyro_status, _degrees(ekf_gyro_max), "deg",
        "largest difference between EKF and integrated gyro",
        "" if ekf_gyro_status in (OK, UNKNOWN) else
        "The EKF does not follow the gyro. Check imu0 in localization.yaml "
        "(only vyaw fused, no low rejection threshold), that wheel yaw is not "
        "fused, and that /imu/wt901/data_raw arrives at ~25 Hz.",
    ))

    wheel_scale = None
    wheel_valid = [stop for stop in valid if stop.wheel is not None]
    if reference_ok and wheel_valid:
        wheel_scale = fit_proportional(
            [stop.wheel - stop.lidar for stop in wheel_valid],
            [stop.lidar for stop in wheel_valid],
        )
    wheel_advice = ""
    if wheel_scale is not None and wheel_track_m and abs(wheel_scale) > 0.02:
        wheel_advice = (
            f"If this is not slip, wheel_track {wheel_track_m:.3f} m -> "
            f"{wheel_track_m * (1.0 + wheel_scale):.3f} m in robot.yaml "
            "(URDF and hardware_bridge)."
        )
    findings.append(_finding(
        "wheel_yaw", INFO if wheel_scale is not None else UNKNOWN,
        None if wheel_scale is None else 100.0 * wheel_scale, "%",
        "wheel-odometry rotation error (not fused by the EKF; slip on smooth "
        "floors is normal)",
        wheel_advice,
    ))

    pivot_status = grade(max_pivot_translation_m, limits.pivot_translation_m) \
        if reference_ok else UNKNOWN
    findings.append(_finding(
        "pivot_translation", pivot_status, max_pivot_translation_m, "m",
        "largest base_link translation during the pivot, from scan matching",
        "" if pivot_status in (OK, UNKNOWN) else
        "Either the robot does not turn around base_link (wheel slip, caster) or "
        "lidar_x/lidar_y in robot.yaml are wrong. Run bagheera_rotation_shift_test.",
    ))

    compass_stops = [stop for stop in valid if stop.compass is not None]
    if compass_stops:
        compass_error = max((stop.compass - stop.lidar for stop in compass_stops), key=abs)
        compass_status = grade(math.degrees(compass_error), limits.compass_error_deg)
        findings.append(_finding(
            "compass", compass_status, math.degrees(compass_error), "deg",
            "largest compass heading error (not fused by the EKF)",
            "" if compass_status == OK else
            "Recalibrate with bagheera_compass_calibrate or keep the compass off.",
        ))

    graded = [item for item in findings if item["check"] != "compass"]
    if not reference_ok:
        verdict = "INCONCLUSIVE"
    else:
        verdict = max((item["status"] for item in graded), key=_SEVERITY.__getitem__)
        verdict = {UNKNOWN: WARN, INFO: OK}.get(verdict, verdict)
    return {
        "verdict": verdict,
        "drift_acceptable": None if not reference_ok else ekf_status == OK,
        "findings": findings,
        "limits": asdict(limits),
    }
