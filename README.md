# BagheeraROS

ROS 2 overlay for the Bagheera office robot. MowgliNext v1.1.0 provides the
STM32 wire protocol, hardware bridge and velocity multiplexer. This repository
provides Bagheera's own robot model, measurement normalization, local sensor
fusion, teleoperation and indoor LiDAR navigation. AprilTag docking remains a
later milestone.

The current platform bringup deliberately starts no GNSS, coverage or mower
behavior. LiDAR, optical flow, the front camera, static-map localization and
Nav2 point-to-point navigation are part of the base bringup.

## Architecture

```text
game controller -> /cmd_vel_teleop -> twist_mux -> /cmd_vel
                                                  |
                                      mowgli_hardware bridge
                                        |                    |
                                  wheel odom raw        USB protocol
                                        |                    |
                                  measurement normalizer     |
                                        |                    |
                                   wheel odom                 |
                                        \                    |
WT901 over Raspberry Pi I2C -> /imu/wt901/data_raw            |
                                         \                   |
                                      robot_localization      |
                                              |               |
                                      /odometry/filtered      |
                                              |               |
                                      map_server + AMCL       |
                                              |               |
                                      /map + map -> odom      |
                                                              |
                                      COBS + CRC16 over USB CDC
                                                  |
                                   MowgliNext STM32 firmware v1.1.0
```

The STM32 runtime connection is `/dev/mowgli` at 115200 baud. The ST-Link is
only needed to flash or debug firmware.

## Raspberry Pi requirements

- Ubuntu Server 24.04 LTS, ARM64
- Docker Engine with Compose v2
- MowgliNext firmware from release v1.1.0, protocol version 6
- stable `/dev/mowgli` symlink
- controller visible below `/dev/input`

Install the udev rule if `/dev/mowgli` does not already exist:

```bash
sudo install -m 0644 config/99-bagheera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Build and start

The image is derived from the released multi-architecture MowgliNext ROS 2
image. ROS itself is not installed on the Ubuntu host.

```bash
git clone https://github.com/Pauli1Go/BagheeraROS.git
cd BagheeraROS
docker compose build
docker compose up
```

The launch starts:

- Bagheera `robot_state_publisher` model and complete static sensor TF tree
- MowgliNext `hardware_bridge_node`
- MowgliNext `twist_mux`
- `bagheera_manual_mode`
- `bagheera_controller`
- measurement normalization (`/wheel_odom`, `/imu/data`, camera metadata)
- `robot_localization` EKF (`/odometry/filtered`, `odom -> base_link`)
- YDLidar G2 driver (`/scan`)
- PMW3901 optical-flow driver (`/optical_flow/raw`, `/optical_flow/twist`)
- WT901 I2C IMU driver (`/imu/wt901/data_raw`)
- front camera driver (`/camera/image_raw`, `/camera/camera_info`)
- static map server (`/map`) and AMCL localization (`map -> odom`)
- Nav2 planner, rotation shim, regulated-pure-pursuit controller and velocity smoother
- bounded Clear/BackUp recovery without Spin; separate LiDAR hardstop disabled
- Foxglove `/goal_pose` bridge to Nav2's `NavigateToPose` action
- Foxglove bridge on `ws://bagheera.local:8765`

The saved map selected by `maps/current.yaml` is loaded at boot. Online mapping
is deliberately not started. AMCL needs an initial robot pose after startup;
publish a `geometry_msgs/msg/PoseWithCovarianceStamped` on `/initialpose` from
Foxglove at the robot's real position in the map. It then keeps `map -> odom`
aligned while the robot moves.

## Navigate from Foxglove

First set the approximate robot pose using the 3D panel's `2D pose estimate`
publisher on `/initialpose`. For a driving goal, configure `2D pose` in the
same panel to publish `geometry_msgs/msg/PoseStamped` on `/goal_pose` (the
legacy Foxglove default `/move_base_simple/goal` is also accepted). Click the
target position and drag the arrow into the desired final heading. The
Bagheera goal bridge forwards it to Nav2's `/navigate_to_pose` action.

The first navigation tuning is capped at 0.16 m/s, matching controller teleop.
Both global and local costmaps consume `/scan`; mapped and live obstacles are
inflated around the physical chassis footprint. The global costmap removes
saved-map obstacle artifacts smaller than six connected 5 cm cells before
adding current LiDAR obstacles. The final velocity chain is:

```text
Nav2/RPP or BackUp -> velocity_smoother -> twist_mux -> STM32
controller teleop ----------------------> twist_mux -> STM32
```

The collision monitor is not launched. Teleop bypasses all Nav2 filtering;
firmware stop conditions and watchdog remain unchanged. Nav2 retains live
obstacle layers, RPP collision detection and collision-checked BackUp.
`base_link` is the midpoint of the 0.32 m differential-drive axle. The common
LiDAR/WT901/PMW3901 origin is approximately 0.1834 m ahead and 0.011 m left of
it. The measured chassis bounds relative to the axle are x=[-0.0766, 0.4334]
and y=[-0.17, 0.17]. Nav2 and the dormant monitor box use x=[-0.0866, 0.4434]
and y=[-0.18, 0.18], incorporating the requested
1 cm safety margin directly (`footprint_padding=0.0`). Both costmaps use
0.25 m inflation.

The installed `behavior_trees/navigate_no_spin.xml` is selected through
`default_nav_to_pose_bt_xml` using the package share path. Four recovery
attempts alternate clear-local/global + replan and BackUp + replan.
BackUp travels 0.20 m at 0.08 m/s with a 6 s timeout. If blocked, it stops,
waits 1 s, clears both costmaps and replans. After the bounded retries the
goal aborts. Normal RPP path/goal alignment can still turn the robot.

Recovery tuning does not fix the separately observed ~4.2 m AMCL error.
Before physical goal tests, verify the initial pose and scan/map alignment;
do not automatically restore an old pose after the robot has moved.

For a new mapping session, stop the normal Compose service, start the base
without static-map localization, and then start SLAM Toolbox:

```bash
docker compose stop bagheera-base
docker compose run --rm --name bagheera-mapping bagheera-base \
  ros2 launch bagheera_base manual_control.launch.py \
  use_map_localization:=false use_navigation:=false use_foxglove:=true
# In a second shell:
docker exec -d bagheera-mapping /bagheera_entrypoint.sh \
  ros2 launch bagheera_base mapping.launch.py
```

In Foxglove, add a 3D panel, select `map` as the fixed frame and enable
`/map`, `/scan`, the robot model and `/odometry/filtered`. The occupancy map
then updates every two seconds while the robot is driven manually. A normal
container or host restart returns to the saved-map localization mode.

Hold controller button 4 while driving. The left stick (axis 1) controls
forward/reverse and the right stick (axis 2) controls in-place rotation.
Steering is ignored while the left stick commands translation, so manual
turning is possible only from a standstill.
For manual mapping, the current limits are 0.16 m/s linear and 0.30 rad/s
angular. These conservative values reduce motion and scan distortion between
successive 9.6 Hz LiDAR scans.

Keep both drive wheels clear of the floor for the first test. Releasing the
deadman button sends zero velocity; loss of commands is additionally bounded
by the MowgliNext firmware watchdog.

Run in the background after validation:

```bash
docker compose up -d
docker compose logs -f bagheera-base
```

Stop explicitly with:

```bash
docker compose down
```

## Checks

Open a shell in the running container:

```bash
docker exec -it bagheera-base bash
```

Then inspect the firmware link and telemetry:

```bash
ros2 topic echo /hardware_bridge/status --once
ros2 topic echo /hardware_bridge/emergency --once
ros2 topic echo /wheel_odom --once
ros2 topic echo /imu/wt901/data_raw --once
ros2 topic echo /imu/wt901/mag_raw --once
ros2 topic echo /imu/compass/valid --once
ros2 topic echo /optical_flow/twist --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo /camera/camera_info --once
ros2 topic echo /map --once
ros2 topic hz /cmd_vel_teleop
```

The EKF fuses encoder forward velocity, PMW3901 planar ground velocity and the
WT901's bias-corrected Z angular velocity. Wheel-derived yaw is deliberately
excluded so wheel slip cannot override the gyro. The WT901 also publishes its
magnetometer on `/imu/wt901/mag_raw`. After a valid calibration exists, the
compass node supplies slowly filtered absolute yaw on `/imu/compass`; disturbed
field and acceleration samples are withheld from the EKF. Without that file it
publishes only `valid=false`, so gyro-only operation continues unchanged.

Create the persistent calibration while slowly completing at least one full
rotation (two rotations over 60 seconds are preferable):

```bash
docker exec -it bagheera-mapping /bagheera_entrypoint.sh \
  ros2 run bagheera_base bagheera_compass_calibrate
```

The result is stored in the maps volume as `compass_calibration.yaml`; the
running compass node reloads it automatically. The calibration tool aligns the
magnetic yaw to the current `/odometry/filtered` yaw when it finishes, avoiding
an immediate heading jump.

Characterize the calibrated compass before fusing it into mapping odometry.
Place the robot in a clear area where it can pivot in place, keep the controller
released, and explicitly enable the automatic test run:

```bash
docker exec -it bagheera-mapping /bagheera_entrypoint.sh \
  ros2 run bagheera_base bagheera_compass_test -- --execute
```

After a five-second countdown the tool commands `/cmd_vel_tuning` directly and
performs one counter-clockwise 360-degree rotation. Wheel odometry closes the
position loop; the robot stops for two seconds at every 45-degree point and
sends repeated zero commands after every step, timeout, error or Ctrl-C.
Compass, corrected magnetic-field strength, gyro integration, wheel yaw, EKF
yaw and a LiDAR range-signature yaw are recorded together. The tuning input
intentionally has priority over an idle controller and bypasses the
navigation-only collision monitor, so the test must only be run in a clear
area. Use `--clockwise` for the opposite direction.

Results are written persistently below
`maps/compass_test_YYYYMMDD_HHMMSS/`. `report.yaml` contains the summary,
`checkpoints.csv` contains every angle comparison, `raw_*.csv` preserves the
source measurements and `scans.json.gz` preserves the LiDAR signatures. Angle
error, gyro/LiDAR disagreement and field deviation expose non-linearity,
timing errors and office-field or motor-current distortion. A second run with
`--clockwise` makes direction-dependent hysteresis directly comparable.

For a simple turn that closes its 360-degree position loop directly from the
WT901 Z gyro, independently of wheel yaw and compass, run:

```bash
docker exec -it bagheera-mapping /bagheera_entrypoint.sh \
  ros2 run bagheera_base bagheera_gyro_turn_test -- --execute
```

The robot measures its stationary gyro offset for two seconds, counts down and
then turns counter-clockwise. Add `--clockwise` for the opposite direction.

To diagnose LiDAR/map translation shift during an in-place turn, start online
mapping first and run the gyro-controlled rotation diagnostic:

```bash
docker exec -it bagheera-mapping /bagheera_entrypoint.sh \
  ros2 run bagheera_base bagheera_rotation_shift_test -- --execute
```

It freezes the current occupancy map as its reference, records all relevant
LiDAR, TF, odometry, gyro, optical-flow, wheel and SLAM topics in a rosbag, and
estimates the map-alignment correction for every sampled scan. Results are
stored below `maps/rotation_shift_YYYYMMDD_HHMMSS/` as `report.yaml`,
`shift_samples.csv`, `reference_map.npz`, configuration snapshots and the raw
bag. The report separates local odometry translation, SLAM `map->odom`
correction and a rotation-dependent LiDAR lever-arm error.

Robot dimensions and all sensor poses are centralized in
`src/bagheera_base/config/robot.yaml`. The PMW height is measured; the current
LiDAR, camera and IMU X/Y offsets are usable provisional defaults and should be
replaced with measured values before precision mapping.

## Save a map

The host directory `maps/` is mounted writable at `/bagheera_ws/maps` inside
the container. Save both the ordinary occupancy map and the serialized SLAM
pose graph after completing a mapping run:

```bash
docker exec -it bagheera-base bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /bagheera_ws/maps/office}}"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: /bagheera_ws/maps/office}"
```

This produces the map files in the repository's `maps/` directory on the Pi.
The serialized pose graph is what `slam_toolbox` later uses to continue mapping
or run in localization mode; the YAML/PGM pair is useful for Nav2 map tooling.
Select the occupancy map used at boot with a stable symlink:

```bash
cd maps
ln -sfn office.yaml current.yaml
```

The camera, LiDAR and optical-flow wiring checks, detected hardware revisions
and reproducible standalone probes are documented in
[`docs/hardware-bringup.md`](docs/hardware-bringup.md).

The old Python `base_driver.py` and its `MW` protocol implementation remain in
the repository only as migration reference. `manual_control.launch.py` never
starts them; the only process opening `/dev/mowgli` is MowgliNext's C++ bridge.

Pure-Python tests can also run on a development machine without ROS installed:

```bash
PYTHONPATH=src/bagheera_base python3.11 -m unittest discover \
  -s src/bagheera_base/test -v
```

## Next milestones

1. Measure the remaining LiDAR, camera and IMU offsets.
2. Drive a slow closed loop, verify the WT901 yaw sign and SLAM loop closure,
   then save the first office map.
3. Tune Nav2 goal following and collision distances in the real office.
4. Add camera-based AprilTag docking and charging verification.

Autonomous velocity commands pass through `bagheera_autonomy_dock_guard`.
When the STM32 reports at least 10 V on the charging input, `/docked` is true
and the first non-zero autonomous command is held back while Bagheera reverses
0.80 m and turns 90 degrees left. Teleoperation uses its separate higher-
priority mux lane and is not gated. The map editor's `keepout_mask.yaml` is
loaded as a Nav2 keepout costmap filter; poses inside
`localization_exclusion_mask.yaml` are rejected and reset to the last valid
AMCL pose.
