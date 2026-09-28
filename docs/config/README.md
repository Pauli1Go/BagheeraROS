# Configuration reference

All runtime configuration lives in `src/bagheera_base/config/`. The files are
bind-mounted into the container, so a change needs only
`docker compose restart` (see [development.md](../development.md)).

| Page | Files |
|---|---|
| [robot.md](robot.md) | `robot.yaml`: robot geometry, sensor poses, MowgliNext hardware-bridge overrides |
| [base.md](base.md) | `base.yaml`: battery monitor, game-controller teleop |
| [sensors.md](sensors.md) | `sensors.yaml`, `camera_fisheye.yaml`: LiDAR, dock sleep, optical flow, IMU, compass, normalizer, camera |
| [localization.md](localization.md) | `localization.yaml` (EKF), `nav2_localization.yaml` (AMCL, map server, pose persistence), `slam.yaml` |
| [navigation.md](navigation.md) | `nav2_navigation.yaml` navigation part, behavior trees, launch speed overrides, dock guard, goal bridge, masks |
| [docking.md](docking.md) | `nav2_navigation.yaml` docking part: AprilTag detector, docking server, dock plugin, trigger |
| [patrol.md](patrol.md) | `patrol.yaml` |

Units are SI throughout: metres, seconds, radians, m/s, rad/s, volts,
amperes. Map poses are `[x, y, yaw]` in the `map` frame.

## What you must adapt for your robot

Values marked **site** depend on your map, values marked **robot** on your
hardware. Everything else is a tuned default that should work for a similar
robot.

| What | Where | Kind |
|---|---|---|
| Chassis dimensions, wheel radius and track, sensor poses | `robot.yaml` (`bagheera`) | robot |
| Encoder scale `ticks_per_meter`, `wheel_track` | `robot.yaml` (`hardware_bridge`) | robot |
| Robot footprint (4 copies, keep identical) | `nav2_navigation.yaml`: `local_costmap`, `global_costmap`, `bagheera_goal_pose_bridge`; `collision_monitor.yaml` | robot |
| LiDAR serial port | `sensors.yaml` → `ydlidar_ros2_driver_node.port` | robot |
| Optical-flow scale and height | `sensors.yaml` → `bagheera_optical_flow` | robot |
| Camera calibration | `camera_fisheye.yaml` | robot |
| Battery voltage offset and thresholds | `base.yaml` → `bagheera_battery_monitor` | robot |
| Game controller on/off (off by default), axes / buttons | `base.yaml` → `bagheera_controller` (`enabled`), axes/buttons also as launch arguments | robot |
| Map, keepout and exclusion masks | `maps/` (not in git) | site |
| Dock pose and undock manoeuvre | `maps/dock.yaml` (`dock_pose`, `undock`); fallback `nav2_navigation.yaml` | site |
| Staging offset and tag offsets | `nav2_navigation.yaml` → `docking_server.bagheera_dock` | robot + site |
| AprilTag IDs and sizes | `nav2_navigation.yaml` → `dock_apriltag`, `bagheera_dock_tag_pose` | site |
| Patrol paths (waypoints, mode, closed) | `maps/paths/<name>.yaml`, made with `tools/paths.sh` | site |

Order of work for a new robot: [installation](../installation.md) →
[calibration](../calibration.md) → [mapping](../mapping.md) →
[docking](../docking.md) → [operation](../operation.md).

## Launch arguments

`manual_control.launch.py` (the container's default command):

| Argument | Default | Effect |
|---|---|---|
| `serial_port` | `/dev/mowgli` | Serial device of the MowgliNext firmware. |
| `joystick_index` | `0` | Pygame joystick index of the game controller (only used with `bagheera_controller.enabled: true` in `base.yaml`). |
| `deadman_button` | `4` | Button that must be held for teleop to send motion. |
| `throttle_axis` | `1` | Axis for forward/reverse. |
| `steering_axis` | `2` | Axis for in-place rotation. |
| `use_lidar` | `true` | LiDAR driver (started and stopped by `bagheera_dock_sleep`). |
| `use_optical_flow` | `true` | PMW3901 driver. |
| `use_wt901` | `true` | WT901 IMU driver. The EKF needs it for yaw. |
| `use_compass` | `false` | Magnetometer publishing and the compass node. Not fused by the EKF. |
| `use_camera` | `true` | On-demand camera manager. Docking needs it. |
| `use_sensor_fusion` | `true` | robot_localization EKF (`odom → base_link`). |
| `use_map_localization` | `true` | map server, AMCL and pose persistence. `false` for mapping. |
| `use_navigation` | `true` | Everything in `navigation.launch.py` (Nav2, docking, patrol). |
| `enable_higher_speeds` | `true` | Autonomous Nav2 speed: 0.32 m/s and 0.45 rad/s instead of 0.16 m/s and 0.30 rad/s. Docking, undocking and teleop keep their own limits. See [navigation.md](navigation.md#speed-overrides-from-the-launch-file). |
| `map` | `/bagheera_ws/maps/current.yaml` | Occupancy map loaded at boot. |
| `use_foxglove` | `true` | foxglove_bridge. |
| `foxglove_address` | `0.0.0.0` | Bind address of foxglove_bridge. |
| `foxglove_port` | `8765` | WebSocket port of foxglove_bridge. |

Override them in `compose.yaml` (`command:`) or with `docker compose run`,
for example `use_map_localization:=false use_navigation:=false` for mapping.
