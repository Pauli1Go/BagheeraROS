# BagheeraROS

ROS 2 overlay for the Bagheera office robot. MowgliNext v1.1.0 provides the
STM32 wire protocol, hardware bridge and velocity multiplexer. This repository
provides Bagheera's own robot model, measurement normalization, local sensor
fusion, teleoperation, indoor LiDAR navigation and AprilTag docking.

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
- battery monitor (`/battery/voltage`, `/battery/percentage`, latched
  `/battery/level`: NORMAL/LOW/CRITICAL/FULL from the averaged, offset-corrected
  pack voltage; reporting only, no automatic return to the dock yet)
- `robot_localization` EKF (`/odometry/filtered`, `odom -> base_link`)
- YDLidar G2 driver (`/scan`)
- PMW3901 optical-flow driver (`/optical_flow/raw`, `/optical_flow/twist`)
- WT901 I2C IMU driver (`/imu/wt901/data_raw`)
- on-demand front camera driver (`/camera/h264`, `/camera/image_raw/compressed`,
  `/camera/camera_info`)
- static map server (`/map`) and AMCL localization (`map -> odom`)
- composable Nav2 localization, NavFn planner, rotation shim,
  regulated-pure-pursuit controller and velocity smoother
- wait-and-replan recovery without Spin or BackUp; separate LiDAR hardstop
  disabled
- Foxglove `/goal_pose` bridge to Nav2's `NavigateToPose` action
- Nav2 `opennav_docking` server with the Bagheera AprilTag dock plugin and a
  Foxglove trigger
- Foxglove bridge on `ws://bagheera.local:8765`

The saved map selected by `maps/current.yaml` is loaded at boot. Online mapping
is deliberately not started. `bagheera_pose_persistence` stores the most recent
plausible AMCL map position **and heading** in `maps/last_pose.json` and restores
it through `/initialpose` after an undocked restart. The record is rejected if
`current.yaml` or its map image has changed. While `/docked` is true, the
measured dock pose `(1.192, 1.884, 1.624 rad)` overrides the saved pose and
keeps AMCL anchored there. If the robot was physically moved while powered
off, set `/initialpose` manually; no software can infer that displacement.

`bagheera_dock_sleep` puts the sensors to sleep 3 s after docking without a
motion command: it stops the LiDAR driver (it runs the driver as a child
process), the WT901 stops polling, the PMW3901 goes into shutdown with its LED
off, and the camera cannot be started, not even by a Foxglove viewer. The
state is latched on `/dock/sleep_state` (`awake`, `sleeping`, `waking`,
`anchoring`, `fault`). An autonomous command in the dock, the controller's
deadman button or `/dock/wake` wakes it: LiDAR on and a steady `/scan`, WT901
gyro bias re-measured (4 s, the robot stands still in the dock), flow sensor
back, EKF reset at its current pose, and AMCL re-anchored to the dock pose.
Only in `awake` does the dock guard start undocking and does teleop drive; a
failed wake-up ends in `fault` with autonomy gated (manual driving allowed),
and the next wake request retries. After a wake-up it stays awake for at least 30 s
(`wake_grace_s`), so a held goal or the teleop driver can leave the dock first. `/dock/sleep_request` (`true`) sends a
docked robot to sleep at once.

The camera process is off at idle. To watch it, show `/camera/h264` in a
Foxglove Image panel: `bagheera_camera_manager` starts the camera as soon as
`foxglove_bridge` subscribes and stops it 20 s after the last viewer leaves.
The stream is colour H.264 (1080p, 15 FPS, ~2 Mbit/s, one keyframe per second)
from the Pi 4 hardware encoder and costs ~17 % of one CPU core. The grayscale
JPEG topic `/camera/image_raw/compressed` is encoded in software (7.5 FPS,
~70 % of one core) and meant for docking only. Publishing `{"data": true}` as
`std_msgs/msg/Bool` on `/camera/stream_enabled` still keeps the camera on
without a viewer; `/camera/stream_active` reports whether the camera process
is running. Docking enables and disables the camera on its own.

## Navigate from Foxglove

Verify that the restored pose matches the robot's real position. If it does
not, set it with the 3D panel's `2D pose estimate` publisher on `/initialpose`.
For a driving goal, configure `2D pose` in the
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
Nav2/RPP -> velocity_smoother -> dock guard -> twist_mux -> STM32
controller teleop ----------------------> twist_mux -> STM32
```

The collision monitor is not launched. Teleop bypasses all Nav2 filtering;
firmware stop conditions and watchdog remain unchanged. Nav2 retains live
obstacle layers and RPP collision detection.
`base_link` is the midpoint of the 0.32 m differential-drive axle. The common
LiDAR/WT901/PMW3901 origin is approximately 0.1834 m ahead and 0.011 m left of
it. The measured chassis bounds relative to the axle are x=[-0.0766, 0.4334]
and y=[-0.17, 0.17]. Nav2 and the dormant monitor box use x=[-0.0866, 0.4434]
and y=[-0.18, 0.18], incorporating the requested
1 cm safety margin directly (`footprint_padding=0.0`). Both costmaps use
0.25 m inflation.

Because `base_link` is the rear axle, the footprint reaches 0.44 m ahead but
only 0.09 m behind. NavFn plans for a small circle around the axle, so near
walls RPP's footprint check can report "collision ahead". A state-lattice
planner (Smac) was tried on 2026-09-23 and reverted: its paths turn in place
mid-route, which RPP cannot follow, and the robot got stuck at doors. NavFn
plus the wait-and-replan recovery below is the preferred combination.

The installed `behavior_trees/navigate_no_spin.xml` is selected through
`default_nav_to_pose_bt_xml` using the package share path. A failed plan or
"collision ahead" is treated as temporary: recoveries cycle through a quick
clear-both-costmaps + replan, a collision-checked 20 cm BackUp (a turn toward
the path can be blocked by the 0.44 m nose while 10 cm further back it is
free) and a 20 s wait + clear + replan. Only 2 retries are allowed, so a goal
aborts after the clear/replan and the BackUp; the 20 s wait is only reached
with more retries. There is no Spin recovery. RPP turns on the spot first when the
carrot is more than 20 degrees off (`rotate_to_heading_min_angle` 0.35 rad);
at 0.50 rad a long straight test arc clipped door frames from standstill.

`bagheera_goal_pose_bridge` rejects a Foxglove goal whose *oriented* footprint
would overlap a wall, unknown map cell or keepout cell, and logs where. Only
the final pose is checked; live obstacles are left to the wait-and-replan
recovery.

Recovery tuning does not fix the separately observed ~4.2 m AMCL error.
Before physical goal tests, verify the initial pose and scan/map alignment;
do not automatically restore an old pose after the robot has moved.

## Dock from Foxglove

Docking uses Nav2's `opennav_docking` server with Bagheera's own dock plugin
`bagheera_docking::TagChargingDock`. Add a Publish panel for
`std_msgs/msg/Bool` on `/dock/trigger` and publish `{"data": true}`.
`bagheera_dock_trigger` turns this into a `DockRobot` goal for `home_dock`:

1. **Staging.** The staging pose is map `(1.218, 1.252, 1.607 rad)`, about
   0.63 m in front of the dock with both tags in view. It is configured as a
   rigid offset from `home_dock` (`staging_x/y/yaw_offset`). If the robot is
   more than 15 cm away, Nav2 drives there first. That drive uses
   `behavior_trees/navigate_to_staging.xml` with a 5 cm / 5 deg goal checker
   (normal goals keep 10 cm / 8.6 deg). While the camera starts,
   `bagheera_dock_trigger` then turns the last degrees onto the staging
   heading: 1.2 x the remaining angle, 0.06-0.30 rad/s, down to 1.5 deg
   (`/dock/status` `STAGING_ALIGN`). Nav2's own turn cannot do this: its
   braking and start-up share one acceleration.
2. **Initial perception.** The camera is switched on only now. Two
   `tagStandard41h12` tags are used: ID 1 (48 mm printed, 26.67 mm pose edge)
   on the dock gives the dock *position*; ID 0 (160 mm printed, 88.89 mm pose
   edge) on the wall above it gives the dock *axis angle*. At the staging pose
   ID 1 is only ~42 px wide and its plane angle scatters by 4.4 degrees
   (IPPE ambiguity), while ID 0 measures the angle to 0.35 degrees. Without
   ID 0 the attempt stops instead of guessing the angle.
3. **Approach.** Nav2's graceful docking controller converges position *and*
   heading onto the dock axis at 0.05-0.10 m/s, with costmap collision
   checking except for the last 15 cm in front of the dock. Its target is a
   pre-dock pose 17 cm in front of the contact pose, so the curve is finished
   before the charging pins (at the pins it still turned by +-14 degrees).
4. **Straight final approach and contact.** At the pre-dock pose the server
   waits for charge while `bagheera_dock_trigger` drives the last 17 cm
   straight at 0.05 m/s with gyro heading hold and a trim of at most 4 degrees.
   It stops at contact voltage (`v_charge >= 0.5 V`) or 3 cm past the contact
   pose. The heading may still be off by up to 10 degrees at hand-over; the
   heading hold turns it back. An axle offset over 2.5 cm cannot be fixed
   straight: the plugin then reports the dock as lost at the pre-dock line,
   so Nav2 retries from the staging pose immediately. Nav2's log line
   "Made contact with dock" only means the pre-dock hand-over.
   `/dock/status` shows `FINAL_APPROACH`, `WAIT_FOR_CHARGE` (after contact)
   and `PRE_DOCK_OFF_AXIS` with the measured offsets. Success is `/docked`
   (debounced 10 V) within 20 s.
5. **Retry.** A failure drives back to the staging pose with the same
   controller and tries again, at most twice.

`bagheera_dock_tag_pose` solves both tags from the raw fisheye corners
(apriltag_ros' own poses assume a pinhole camera and are isolated on
`/dock/tag_tf_unused`) and publishes them in `camera_optical_frame` with the
image stamp on `/dock/detected_pose` (ID 1) and `/dock/detected_axis`
(ID 0). The plugin transforms them into `odom` with TF at exposure time; the
~0.9 s AprilTag latency on the Pi therefore does not shift the target. The
dock is static in `odom`, so the filtered pose is held when ID 0 leaves the
image near the dock and when ID 1 disappears in the last ~4 cm before
contact (`hold_distance` 0.30 m).

The contact geometry comes from the successful manual docking recording
`docking_record_20260921_115633`: ID 1 reaches 300 px edge at 9.9 cm camera
depth, 4.3 cm of wheel travel before contact, so base_link sits 0.489 m
behind ID 1 when docked. ID 0's direction plus 1.4 degrees is the robot
heading in the dock.

Display `/dock/status` for the JSON state (`NAV_TO_STAGING_POSE`,
`INITIAL_PERCEPTION`, `CONTROLLING`, `WAIT_FOR_CHARGE`, `SUCCEEDED`,
`FAILED` with Nav2's error message). Publish `{"data": true}` on
`/dock/cancel` to stop. Moving the game controller also cancels docking; its
teleop lane has priority over the docking lane anyway. Leaving the dock is
not a separate command: `bagheera_autonomy_dock_guard` reverses out of the
dock before any autonomous Nav2 movement. Debug poses are published on
`/dock_pose` (refined dock), `/staging_pose` and `/docking_trajectory`; the
docking server logs one `Dock estimate:` line per second with the fused tag
position, axis angle and the robot's along/left/yaw offset to the dock.

```bash
docker exec -it bagheera-base bash -lc 'source /opt/ros/kilted/setup.bash && source /bagheera_ws/install/setup.bash && ros2 topic pub --once /dock/trigger std_msgs/msg/Bool "{data: true}"'
```

To calibrate the tag offsets against the real dock, start with the robot
charging in the dock and run the tool below. It stores the docked odom pose,
then waits while you reverse out with the game controller (it sends no
motion command itself). Every stop with both tags in view gives a sample of
`external_detection_translation_x/y` and `axis_yaw_offset`; Ctrl-C prints
the averages for `nav2_navigation.yaml` and saves them under
`/bagheera_ws/maps/dock_calibration_*.json`. Trust its lateral and yaw
results only: the wheels slip while leaving the dock, so its `translation_x`
is off by centimetres. The contact distance comes from the docked camera image
instead (ID 1 ~3.5 cm in front of the camera).

```bash
docker exec -it bagheera-base bash -lc 'source /opt/ros/kilted/setup.bash && source /bagheera_ws/install/setup.bash && ros2 run bagheera_base bagheera_dock_calibrate'
```

The previous hand-written docking state machine (including the ID 1-only
`/dock/small_trigger` mode and the stationary alignment check) is kept for
reference under `legacy/docking/` and is no longer built or launched.

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

The only process opening `/dev/mowgli` is MowgliNext's C++ bridge.

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
4. Validate Nav2 AprilTag docking repeatedly from the staging pose.

Autonomous velocity commands pass through `bagheera_autonomy_dock_guard`.
When the STM32 reports at least 10 V on the charging input, `/docked` is true
and the first non-zero autonomous command is held back while Bagheera reverses
0.80 m and turns 90 degrees left. The guard already gates at the first contact
voltage (0.5 V): the charger needs several seconds to reach 10 V, and Nav2
must not move the robot inside the dock meanwhile. Teleoperation uses its separate higher-
priority mux lane and is not gated. The map editor's `keepout_mask.yaml` is
loaded as a Nav2 keepout costmap filter; poses inside
`localization_exclusion_mask.yaml` are rejected and reset to the last valid
AMCL pose.
