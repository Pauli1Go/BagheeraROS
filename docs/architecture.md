# Architecture

This page explains how the pieces of BagheeraROS fit together: which process
runs what, which configuration file feeds which node, how velocity commands
reach the wheels and how the state machines (dock sleep, docking, patrol)
interact. The per-parameter reference lives in [`config/`](config/README.md).

## Layers

```text
┌──────────────────────────────────────────────────────────────────────────┐
│ Foxglove Studio (laptop)  ── ws://<robot>:8765 ──┐                       │
└──────────────────────────────────────────────────┼───────────────────────┘
                                                   │
┌─ Raspberry Pi 4, Ubuntu 24.04 host ──────────────┼───────────────────────┐
│  Docker container "bagheera-base" (ROS 2 Kilted, host network)           │
│   ├─ MowgliNext: hardware_bridge_node, twist_mux  (from the base image)  │
│   ├─ BagheeraROS: bagheera_base (Python nodes, config, launch, BTs)      │
│   ├─ BagheeraROS: bagheera_docking (C++ Nav2 dock plugin)                │
│   ├─ BagheeraROS: bagheera_sensors (C++ PMW3901/WT901/normalizer)        │
│   └─ third party: Nav2, robot_localization, slam_toolbox, apriltag_ros,  │
│      YDLidar driver, camera_ros (patched), foxglove_bridge               │
│                                                                          │
│  I2C1: WT901 IMU   SPI0: PMW3901 flow   USB: YDLidar G2   CSI: camera    │
└───────────────────────────┬──────────────────────────────────────────────┘
                            │ USB CDC /dev/mowgli (COBS + CRC16, protocol v6)
┌───────────────────────────┴──────────────────────────────────────────────┐
│ YardForce GForce mainboard, MowgliNext firmware 1.9.10                   │
│ drive motors + encoders, charging input, emergency inputs, watchdog      │
└──────────────────────────────────────────────────────────────────────────┘
```

[MowgliNext](https://github.com/mowglinext/mowglinext) provides everything
that talks to the mainboard: the STM32 firmware, the C++
`hardware_bridge_node`, the `twist_mux` configuration and the ROS 2 Docker
image this project builds on. BagheeraROS never opens `/dev/mowgli` itself.
The firmware remains the final safety authority: its command watchdog and
emergency inputs stop the motors regardless of what ROS does.

## Launch structure

Everything starts from one launch file:

```text
manual_control.launch.py            (compose.yaml default command)
├─ robot_state_publisher             robot.yaml → URDF (xacro)
├─ hardware_bridge (MowgliNext)      MowgliNext hardware_bridge.yaml + robot.yaml
├─ twist_mux (MowgliNext)            MowgliNext twist_mux.yaml
├─ bagheera_mode                     keeps the firmware in manual (no blade) mode
├─ bagheera_controller               base.yaml               [base.yaml enabled, off]
├─ bagheera_battery_monitor          base.yaml
├─ sensor_container (C++ components from bagheera_sensors, one process)
│   ├─ bagheera_measurement_normalizer   sensors.yaml
│   ├─ bagheera_optical_flow             sensors.yaml        [use_optical_flow]
│   └─ bagheera_wt901                    sensors.yaml        [use_wt901]
├─ bagheera_dock_sleep               sensors.yaml (starts the YDLidar driver as child)
├─ bagheera_compass                  sensors.yaml            [use_compass, off]
├─ bagheera_camera_manager           sensors.yaml (camera section) [use_camera]
├─ ekf_filter_node                   localization.yaml       [use_sensor_fusion]
├─ nav2_container                    nav2_localization.yaml + nav2_navigation.yaml
│   └─ odometry_throttle (topic_tools) /odometry/filtered → /odometry/filtered_throttled (5 Hz)
├─ nav2 localization_launch.py       nav2_localization.yaml  [use_map_localization]
│   (map_server + amcl in nav2_container)
├─ bagheera_pose_persistence         nav2_localization.yaml + dock pose [use_map_localization]
├─ navigation.launch.py                                      [use_navigation]
│   ├─ Nav2 components in nav2_container: controller, planner, behaviors,
│   │  bt_navigator, velocity_smoother, keepout + exclusion mask servers
│   ├─ docking_server + lifecycle_manager_docking (own process)
│   ├─ dock_apriltag, bagheera_dock_frame_gate, bagheera_dock_tag_pose
│   ├─ bagheera_dock_trigger, bagheera_autonomy_dock_guard
│   ├─ bagheera_goal_pose_bridge, bagheera_localization_exclusion_guard,
│   │  bagheera_keepout_mask_relay
│   └─ bagheera_patrol                patrol.yaml
└─ foxglove_bridge                                           [use_foxglove]

mapping.launch.py                    slam.yaml (started separately, see mapping.md)
```

Everything in `nav2_navigation.yaml` is read by `navigation.launch.py`;
`manual_control.launch.py` additionally passes it to the Nav2 container
(costmap child nodes need it at process level) and reads the dock pose from it
for `bagheera_pose_persistence`. The launch arguments are listed in
[`config/README.md`](config/README.md#launch-arguments).

## Configuration files at a glance

| File | Consumers | Reference |
|---|---|---|
| `config/robot.yaml` | URDF (robot_state_publisher), MowgliNext hardware bridge | [robot.md](config/robot.md) |
| `config/base.yaml` | battery monitor, game-controller teleop | [base.md](config/base.md) |
| `config/sensors.yaml` | LiDAR driver, dock sleep, optical flow, WT901, compass, normalizer, camera | [sensors.md](config/sensors.md) |
| `config/camera_fisheye.yaml` | camera_ros `CameraInfo`, dock tag pose | [sensors.md](config/sensors.md#camera_fisheyeyaml) |
| `config/localization.yaml` | robot_localization EKF | [localization.md](config/localization.md#localizationyaml-ekf) |
| `config/nav2_localization.yaml` | map_server, AMCL, pose persistence | [localization.md](config/localization.md#nav2_localizationyaml) |
| `config/slam.yaml` | slam_toolbox (mapping only) | [localization.md](config/localization.md#slamyaml) |
| `config/nav2_navigation.yaml` | Nav2 servers, costmaps, masks, guards, docking | [navigation.md](config/navigation.md), [docking.md](config/docking.md) |
| `config/patrol.yaml` | waypoint patrol | [patrol.md](config/patrol.md) |
| `config/collision_monitor.yaml` | not launched (kept for reference) | [navigation.md](config/navigation.md#collision_monitoryaml) |
| `behavior_trees/*.xml` | bt_navigator, docking_server | [navigation.md](config/navigation.md#behavior-trees) |

Config, launch files, behavior trees and the Python package are bind-mounted
into the container (see `compose.yaml`), so editing them needs only
`docker compose restart`. See [development.md](development.md).

## Frames (TF)

```text
map ──(AMCL, or slam_toolbox while mapping)──► odom ──(EKF)──► base_link
                                                               ├─ base_footprint
                                                               ├─ base_axle, wheel and caster links
                                                               ├─ imu_link             (WT901)
                                                               ├─ lidar_link           (YDLidar)
                                                               ├─ pmw3901_sensor       (PMW3901)
                                                               └─ camera_link → camera_optical_frame
```

- `base_link` is the **midpoint of the drive axle** (REP-103: x forward,
  y left, z up). The chassis therefore reaches far ahead of `base_link` and
  only a little behind it; every footprint in the configuration uses this
  origin.
- The EKF is the only publisher of `odom → base_link`. AMCL (normal operation)
  or slam_toolbox (mapping) publishes `map → odom`; never both.
- All sensor poses come from `robot.yaml` through the xacro URDF.

## Localization chain

```text
/wheel_odom_raw ─► measurement_normalizer ─► /wheel_odom ──(vx, vy=0)──┐
/optical_flow/twist ───────────────────────────────(vx)──────────────► EKF ─► /odometry/filtered
/imu/wt901/data_raw ───────────────────────(yaw rate only)───────────┘      odom → base_link
                                                                               │
/scan + /map ─► AMCL ─► map → odom ◄── /initialpose ◄── pose_persistence ─────┘
                  ▲                                    (restore / dock anchor)
                  └── localization_exclusion_guard rejects poses inside the exclusion mask
```

- The EKF fuses forward velocity from wheels and optical flow and yaw rate
  from the WT901 gyro only. Wheel yaw is excluded so wheel slip cannot rotate
  the estimate. The compass node exists but is disabled and not fused by
  default.
- `bagheera_pose_persistence` stores the map pose continuously in
  `maps/last_pose.json` and restores it after a restart (if the map did not
  change). While the robot is docked it forces AMCL onto the configured dock
  pose.
- `bagheera_localization_exclusion_guard` resets AMCL to the last valid pose
  when it jumps into a region painted in `localization_exclusion_mask.pgm`.

## Velocity chain

```text
Nav2 controller / behaviors ─► /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel_automatic_raw
    ─► bagheera_autonomy_dock_guard ─► /cmd_vel_monitored ─┐
docking_server + bagheera_dock_trigger ─► /cmd_vel_docking ─┤
bagheera_controller (game controller) ─► /cmd_vel_teleop ───┼─► twist_mux ─► /cmd_vel ─► hardware_bridge ─► STM32
test tools (--execute) ─► /cmd_vel_tuning ──────────────────┤
/cmd_vel_emergency ─────────────────────────────────────────┘
```

MowgliNext's `twist_mux` priorities (higher wins while the lane is active):

| Lane | Topic | Priority | Timeout |
|---|---|---:|---:|
| navigation | `/cmd_vel_monitored` | 10 | 0.6 s |
| docking | `/cmd_vel_docking` | 15 | 0.5 s |
| teleop | `/cmd_vel_teleop` | 20 | 0.5 s |
| tuning | `/cmd_vel_tuning` | 30 | 0.5 s |
| emergency | `/cmd_vel_emergency` | 100 | 0.2 s |

Consequences:

- Teleop always overrides autonomy; moving the controller also cancels docking
  and the patrol.
- Test tools drive on the tuning lane and override an idle controller. Run
  them only in a clear area.
- The Nav2 collision monitor is **not** launched. Obstacle handling is done by
  the costmaps, RPP's collision check, the collision-checked BackUp recovery
  and replanning. Teleop is never filtered.
- `bagheera_autonomy_dock_guard` sits in the navigation lane only: when the
  robot is on the charger it holds back the first autonomous command, reverses
  out of the dock and turns, then passes commands through.

## State machines

### Dock detection

The hardware bridge publishes `v_charge` on `/hardware_bridge/power`.
`bagheera_autonomy_dock_guard` debounces it (≥ 10 V for 1 s) and publishes
the latched `/docked`. Autonomy is already gated at first contact (≥ 0.5 V)
because the charger needs a few seconds to ramp up.

### Dock sleep (`/dock/sleep_state`)

```text
awake ──(docked + idle for sleep_delay_s)──► sleeping
sleeping ──(autonomous command, deadman button, /dock/wake, held goal)──► waking
waking ──(LiDAR at ≥ 8 Hz, WT901 bias re-measured, flow sensor back)──► anchoring
anchoring ──(EKF reset, AMCL anchored to the dock pose)──► resuming
resuming ──(Nav2 lifecycle resumed, obstacle layers on)──► awake
any wake step failing ──► fault   (autonomy gated, teleop allowed, next wake retries)
```

While sleeping, the LiDAR driver process is stopped (after the costmap
obstacle layers are confirmed off, so they never see a stale `/scan`), the
WT901 stops polling, the PMW3901 is shut down (LED off), the camera cannot
start and the Nav2 navigation lifecycle is paused. Localization, map servers and the docking
server keep running. Undocking and teleop only drive in `awake`.

### Docking (`/dock/status`)

`/dock/trigger` → `bagheera_dock_trigger` sends a `DockRobot` goal →
`docking_server` drives to the staging pose (Nav2, precise goal checker) →
`STAGING_ALIGN` (final heading) → `INITIAL_PERCEPTION` (camera on, both tags)
→ `CONTROLLING` (graceful controller to the pre-dock pose) →
`FINAL_APPROACH` (straight drive with gyro heading hold) → `WAIT_FOR_CHARGE`
→ `SUCCEEDED` when `/docked` becomes true. Details in [docking.md](docking.md).

### Patrol (`/patrol/status`)

Drives the waypoint loop from `patrol.yaml`, docks according to the mode and
the battery level (`/battery/level`: NORMAL, LOW, CRITICAL, FULL), wakes the
robot, lets the dock guard reverse out (`UNDOCKING`, `/dock/undock`) before
the first goal and skips waypoints Nav2 cannot reach. Details
in [operation.md](operation.md#waypoint-patrol).

## Packages

| Package | Language | Content |
|---|---|---|
| `src/bagheera_base` | Python (ament_python) | all Bagheera nodes, config, launch files, URDF, behavior trees, tests |
| `src/bagheera_docking` | C++ | `bagheera_docking::TagChargingDock`, an `opennav_docking` plugin using two AprilTags |
| `src/bagheera_sensors` | C++ | PMW3901 (SPI) and WT901 (I2C) drivers and the measurement normalizer as `rclcpp` components; ROS-free decoding in `sensor_math.hpp` with gtests |
| `docker/` | Dockerfile + patches | image build on top of the MowgliNext image; YDLidar and camera_ros patches |
| `tools/` | Python, shell, C | calibration, probes and diagnostics outside the launch |
| `legacy/docking/` | Python | the previous hand-written docking controller, reference only, not built |

Pure logic lives in ROS-free modules (`*_math.py`, `*_logic.py`, …, and
`bagheera_sensors/sensor_math.hpp` for the C++ drivers) so it can be unit
tested without ROS. The sensor drivers are C++ because every message into a
Python node costs ~2–3 ms of executor overhead on the Pi 4.
