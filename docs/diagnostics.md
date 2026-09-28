# Diagnostics and test tools

Two kinds of tools exist:

- **ROS executables** in `bagheera_base` (`ros2 run bagheera_base …`), run
  inside the container.
- **Scripts** in `tools/`, run on the host or copied into the container
  (only `maps/` and `test_logs/` are mounted; copy a script to `maps/` to run
  it inside).

> Tools that take `--execute` **move the robot**. They publish on
> `/cmd_vel_tuning`, which overrides the game controller in `twist_mux`, and
> they bypass all Nav2 obstacle handling. Run them only in a clear area, with
> the stop button in reach. Every tool sends zero commands on exit, error and
> Ctrl-C.

Run the ROS tools like this (the container name is `bagheera-mapping` while
in mapping mode):

```bash
docker exec -it bagheera-base /bagheera_entrypoint.sh \
  ros2 run bagheera_base <tool> [arguments]
```

Results go to `maps/<tool>_YYYYMMDD_HHMMSS/` on the host.

## 360° turn test

`bagheera_heading_test --execute` answers two questions: **is the heading
drift acceptable**, and if not, **which sensor causes it**. (The former name
`bagheera_compass_test` still works.)

Sequence: stand still for 10 s, turn once through 360° on the spot with a
2 s stop every 45°, stand still for 10 s again. The gyro closes the turn
loop (`--drive-source wheel` uses wheel odometry instead). Takes about one
minute. Keep the area clear and do not walk around the robot.

**Reference.** At every stop the LiDAR scan is matched against the first scan
(2D point-to-line ICP). This measures the true rotation independently of all
motion sensors. The matching accounts for the LiDAR sitting ahead of the
axle (`lidar_x/y/yaw` from `robot.yaml`). A stop counts as valid when the
match RMS is ≤ 5 cm, ≥ 50 % of the points overlap and the result is within
15° of the gyro.

Against this reference it compares:

| Source | Topic | Used by the EKF |
|---|---|---|
| gyro (integrated) | `/imu/wt901/data_raw` | yes, yaw rate |
| EKF | `/odometry/filtered` | – |
| wheel odometry | `/wheel_odom` | no (yaw not fused) |
| compass (only if calibrated and running) | `/imu/compass`, `/imu/wt901/mag_raw` | no |

**Verdict.** The console and `report.yaml` show one line per check:

| Check | Meaning | OK | WARN | FAIL above |
|---|---|---:|---:|---:|
| LiDAR reference | enough valid stops, including the final one | – | – | otherwise the result is `INCONCLUSIVE` |
| EKF heading error | EKF error after the full turn: **the drift** (`drift_acceptable`) | ≤ 2° | ≤ 5° | 5° |
| gyro bias | WT901 rate while standing still (both still periods) | ≤ 0.5°/min | ≤ 2°/min | 2°/min |
| gyro scale | gyro rotation per true rotation, bias removed | ≤ 0.5 % | ≤ 1.5 % | 1.5 % |
| EKF follows gyro | largest EKF − gyro difference | ≤ 1° | ≤ 3° | 3° |
| pivot translation | how far `base_link` moved while pivoting (Bagheera's pivot centre wanders ~10 cm on its casters, which is normal) | ≤ 12 cm | ≤ 17 cm | 17 cm |
| wheel yaw | wheel rotation error, INFO only | – | – | – |
| compass | largest compass error, not part of the verdict | ≤ 5° | ≤ 10° | 10° |

The overall result is the worst of the graded checks: `OK`, `WARN`, `FAIL`,
or `INCONCLUSIVE` when the LiDAR reference is unusable. Every failed check
prints what to do, for example:

```text
Heading test result: WARN
Heading drift acceptable: no
  OK      LiDAR reference    8/8 stops
  WARN    EKF heading error  +3.80 deg per turn
  OK      gyro bias          +0.300 deg/min
  WARN    gyro scale         +1.00 %  (+3.6 deg per turn)
          -> There is no scale parameter for the WT901. Check that it is mounted level ...
  OK      EKF follows gyro   -0.000 deg
  INFO    wheel yaw          -5.00 %
          -> If this is not slip, wheel_track 0.320 m -> 0.304 m in robot.yaml ...
  OK      pivot translation  +0.000 m
```

How to read the causes:

| Result | Cause | What to do |
|---|---|---|
| gyro bias WARN/FAIL | the robot moved during the 4 s gyro calibration after start or wake-up, or the sensor warms up | restart with the robot still; check the mounting |
| gyro scale WARN/FAIL | the WT901 measures too much or too little rotation | mount it level and firmly; a large error means a faulty sensor |
| EKF follows gyro WARN/FAIL | the EKF rejects or delays IMU samples, or fuses wheel yaw | check `imu0` in `localization.yaml`, IMU rate |
| pivot translation WARN/FAIL | wheel slip while pivoting, or `lidar_x/y` wrong | run the rotation shift test below |
| wheel yaw large | wheel slip (normal on smooth floors) or `wheel_track` wrong | use the suggested `wheel_track` only if the floor has grip |
| INCONCLUSIVE | too few features in LiDAR range, or people moving | repeat in a furnished area, stay away from the robot |

Output in `maps/heading_test_YYYYMMDD_HHMMSS/`: `report.yaml` (verdict,
findings, limits), `checkpoints.csv` (one row per stop: every source, its
error against the LiDAR, match quality), `raw_*.csv` (all samples) and
`scans.json.gz` (the scans of every stop). Run it once in each direction
(`--clockwise`) to see direction-dependent errors.

Options: `--still` (standing time before/after, default 10 s), `--settle`,
`--window` (stop duration and averaging window), `--drive-source`,
`--maximum-speed`, `--minimum-speed` (rad/s), `--angle-tolerance-deg`,
`--step-timeout`, `--countdown`, `--robot-config`, `--calibration`,
`--output`. The limits are defined in `heading_test_math.Limits`.

## Gyro turn test

`bagheera_gyro_turn_test --execute` measures the gyro offset for 2 s, then
turns exactly 360° as measured by the WT901 gyro alone. Mark the robot's
start heading on the floor: the difference to the real end heading is the
gyro's scale error. `--clockwise` for the other direction.

## Rotation shift test

`bagheera_rotation_shift_test --execute` needs mapping mode with SLAM
running. It freezes the current map, pivots once with gyro control and
matches every scan against the frozen map. The report separates:

- EKF translation during the pivot (should be ~0),
- slam_toolbox's `map → odom` correction,
- a rotation-dependent error that fits a wrong LiDAR lever arm; the report
  suggests a corrected LiDAR x/y (`recommended_lidar_offset_base_xy_m`),
- optical flow that leaked through the rotation gate.

It also records a rosbag of all relevant topics. Output: `report.yaml`,
`shift_samples.csv`, `reference_map.npz`, configuration snapshots, bag.

## Straight drive test

`bagheera_straight_drive_test --execute` drives 3 m straight (`--distance`,
`--speed`) and records wheel odometry, optical flow, EKF, map pose and all
velocity stages to CSV and a rosbag. `--mode direct` drives on the tuning
lane; `--mode nav2` sends a Nav2 goal instead, to see what the controller
does. Use it for `ticks_per_meter`, optical-flow scale and heading-hold
checks.

## Dock calibration

`bagheera_dock_calibrate` (no motion): see
[docking.md](docking.md#4-tag-offsets).

## Scripts in `tools/`

| Script | Runs | Purpose |
|---|---|---|
| `make_masks.py` | host | Create empty keepout / exclusion masks for a map ([mapping.md](mapping.md#8-masks)). |
| `make_apriltags.py` | host | Print-ready AprilTag PDF with exact sizes ([docking.md](docking.md#2-camera-calibration-and-tags)). |
| `calibrate_fisheye.py` | host with OpenCV | Fisheye camera calibration from checkerboard images. |
| `save_calib_images.py` | container | Save camera frames for the calibration. |
| `calib_checkerboard_*.pdf` | print | Checkerboards for the camera calibration. |
| `amcl_hold_test.py` | container | Set a pose and measure whether AMCL holds it, optionally while turning or driving (`--spin`, `--drive`). |
| `scan_match_probe.py` | host | Score a dumped scan against a map and find the best-matching poses; shows whether a map is ambiguous. |
| `scan_health.py` | host | Which angle sectors of a dumped scan are usable (dead zones, blocked view). |
| `drive_probe.py` | container | Command a constant velocity and record every stage of the command chain (mux, wheels, EKF) to find where motion is lost. Moves the robot. |
| `nav_chain_probe.py` | container | Record every stage of the Nav2 velocity chain while Nav2 drives. |
| `perf_snapshot.sh` | host | Read-only CPU/ROS load snapshot to `test_logs/perf/`, e.g. `tools/perf_snapshot.sh sleep 60`. |
| `check_nav2_bringup.sh` | throwaway container | Nav2 lifecycle and BT parse check with fake TF, without hardware ([development.md](development.md)). |
| `pmw3901_probe.c` | host | Read the PMW3901's ID registers over SPI ([hardware.md](hardware.md#pmw3901-optical-flow)). |

## Quick health checks

ROS runs on the robot's loopback interface only, so run these inside the
container (`docker exec -it bagheera-base bash`), not on a laptop:

```bash
ros2 topic hz /scan                           # ~9.6 Hz
ros2 topic hz /imu/wt901/data_raw             # ~25 Hz
ros2 topic hz /odometry/filtered              # 25 Hz
ros2 topic echo /hardware_bridge/emergency --once
ros2 topic echo /dock/sleep_state --once
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /docking_server
ros2 run tf2_ros tf2_echo map base_link
```
