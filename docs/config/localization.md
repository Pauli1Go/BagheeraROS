# Localization: `localization.yaml`, `nav2_localization.yaml`, `slam.yaml`

How the pieces connect is shown in
[architecture.md](../architecture.md#localization-chain). In short: the EKF
produces smooth local odometry (`odom → base_link`). AMCL matches the LiDAR
against the saved map and publishes the global correction (`map → odom`).
During mapping, slam_toolbox takes AMCL's place.

## `localization.yaml` (EKF)

`robot_localization`'s `ekf_filter_node`, publishing `/odometry/filtered` and
`odom → base_link`.

### General

| Parameter | Value | Meaning |
|---|---|---|
| `frequency` | 25 Hz | Filter output rate. |
| `sensor_timeout` | 0.20 s | Predict without updates after this long. |
| `two_d_mode` | true | Planar robot: z, roll and pitch are fixed. |
| `publish_tf` | true | The EKF is the only `odom → base_link` publisher. |
| `map_frame`, `odom_frame`, `base_link_frame`, `world_frame` | `map`, `odom`, `base_link`, `odom` | Local (odom) filter. |
| `transform_time_offset`, `transform_timeout` | 0.0, 0.10 s | TF stamping and lookup. |
| `predict_to_current_time` | true | Output is predicted to "now", not to the last measurement. |
| `reset_on_time_jump` | true | Reset if the clock jumps. |
| `publish_acceleration` | false | Not needed. |
| `print_diagnostics`, `debug` | true, false | Diagnostics on `/diagnostics`. |

### Inputs

Each `*_config` is a 15-element mask in the order
`x y z | roll pitch yaw | vx vy vz | vroll vpitch vyaw | ax ay az`.

| Input | Topic | Fused | Why |
|---|---|---|---|
| `odom0` | `/wheel_odom` | `vx`, `vy` | Encoder forward speed, plus `vy = 0` (a differential drive cannot slide sideways). Wheel yaw is **not** fused: it is wrong whenever a wheel slips. |
| `twist0` | `/optical_flow/twist` | `vx` | Floor-relative forward speed that catches wheel slip. Its lateral component is a false velocity during pivots. |
| `imu0` | `/imu/wt901/data_raw` | `vyaw` | Bias-corrected gyro Z rate: the only rotation source. No magnetometer orientation and no accelerations. |

| Parameter | Value | Meaning |
|---|---|---|
| `*_queue_size` | 10–20 | Message buffer per input. |
| `*_nodelay` | false / true | TCP_NODELAY for the subscription (on for the IMU). |
| `*_differential`, `*_relative` | false | Absolute measurements. |
| `odom0_pose_rejection_threshold`, `odom0_twist_rejection_threshold` | 5.0, 2.0 | Mahalanobis gates for wheel odometry. |
| `twist0_rejection_threshold` | 2.0 | Gate for optical flow. |
| `imu0_pose_rejection_threshold`, `imu0_linear_acceleration_rejection_threshold` | 0.8, 1.5 | Not relevant for the fused yaw rate. **Do not add** a low `imu0_twist_rejection_threshold`: with the small gyro covariance it rejected normal turns, and the yaw stood still while the robot rotated. |
| `imu0_remove_gravitational_acceleration` | true | Irrelevant while no acceleration is fused. |
| `use_control`, `control_*`, `stamped_control` | off | Commanded velocity is not used as an input. |
| `acceleration_limits`, `deceleration_limits`, `*_gains` | see file | Only used with `use_control`. |
| `process_noise_covariance` | diagonal | How fast the state is allowed to change between measurements. Larger → trusts measurements more. |
| `initial_estimate_covariance` | diagonal | Initial uncertainty after start or reset. |

## `nav2_localization.yaml`

Loaded by Nav2's `localization_launch.py` (map server + AMCL in the Nav2
container) and by `bagheera_pose_persistence`.

### `amcl`

| Parameter | Value | Meaning |
|---|---|---|
| `base_frame_id`, `odom_frame_id`, `global_frame_id` | `base_link`, `odom`, `map` | Frames. |
| `scan_topic` | `/scan` | LiDAR input. |
| `tf_broadcast`, `transform_tolerance` | true, 0.5 s | AMCL publishes `map → odom`, future-dated by the tolerance. |
| `robot_model_type` | `DifferentialMotionModel` | |
| `alpha1`–`alpha5` | 0.02 … 0.01 | Odometry noise per motion. Small, because the EKF output is already good; large values smear the particle cloud. |
| `laser_model_type` | `likelihood_field` | |
| `laser_min_range`, `laser_max_range` | 0.28, 6.0 m | Beyond 6 m, beams through glass or into unmapped rooms pulled the pose into another room. |
| `laser_likelihood_max_dist` | 0.5 m | A wide field (2 m) made almost every pose look plausible. |
| `max_beams` | 180 | Beams used per update. |
| `sigma_hit` | 0.15 m | Expected scan-to-map error. |
| `z_hit`, `z_rand`, `z_max`, `z_short`, `lambda_short` | 0.65, 0.35, 0, 0, 0.1 | Mixture weights. The large `z_rand` accounts for chairs, people and moved objects. |
| `do_beamskip`, `beam_skip_*` | true, 0.5 m, 0.3, 0.9 | Ignore beams that most particles disagree with (unmapped obstacles). |
| `min_particles`, `max_particles` | 1000, 3000 | Particle count (KLD adaptive). |
| `pf_err`, `pf_z` | 0.05, 0.99 | KLD sampling parameters. |
| `resample_interval` | 2 | Resample every second update. |
| `recovery_alpha_fast`, `recovery_alpha_slow` | 0.1, 0.001 | Inject random particles when the match quality drops (kidnapped robot). |
| `update_min_d`, `update_min_a` | 0.10 m, 0.10 rad | Update only after this much motion. |
| `save_pose_rate` | 0.5 Hz | AMCL's own pose parameter storage (not used for restarts; see pose persistence). |
| `set_initial_pose`, `always_reset_initial_pose` | false | The initial pose comes from `bagheera_pose_persistence` or from you (`/initialpose`), never from a fixed parameter. |
| `first_map_only` | false | Accept map updates. |

### `map_server`

Publishes the map given by the `map` launch argument (default
`/bagheera_ws/maps/current.yaml`) on `/map` in frame `map`.

### `lifecycle_manager_localization`

Brings up map server and AMCL (`autostart: true`). The bond settings restart
the pair if one of them dies.

### `bagheera_pose_persistence`

Remembers where the robot is across restarts and pins AMCL to the dock while
the robot is docked.

- Undocked: saves the current map pose (the last `/amcl_pose` plus the EKF
  motion since that scan) to `pose_file` whenever it moved more than
  `save_min_distance_m` or turned more than `save_min_angle_rad`. A sudden
  AMCL jump is saved only if it persists for 10 s. After a restart the saved
  pose is published on `/initialpose`, but only if the map file is unchanged
  (hash check).
- Docked (`/docked` true): publishes the dock pose on `/initialpose` until
  AMCL confirms it, and re-anchors on request of `bagheera_dock_sleep` after
  a wake-up.

| Parameter | Value | Meaning |
|---|---|---|
| `pose_file` | `/bagheera_ws/maps/last_pose.json` | Saved pose (in the `maps/` volume). |
| `map_file` | `/bagheera_ws/maps/current.yaml` | Map whose hash must match the saved record. |
| `dock_x`, `dock_y`, `dock_yaw` | from `maps/dock.yaml` (else `docking_server.home_dock.pose`) | Set by `manual_control.launch.py`; there are no defaults. See [docking.md](docking.md#dock-pose). |
| `save_min_distance_m`, `save_min_angle_rad` | 0.02 m, 1° | Save thresholds. |
| `odom_topic` | `/odometry/filtered_throttled` | 5 Hz copy of the EKF output (`odometry_throttle` in `nav2_container`). The pose is interpolated between samples; the full 25 Hz only costs Python executor time. |

If the robot was carried somewhere while powered off, no software can know.
Set the pose manually with Foxglove's *2D pose estimate* ([operation.md](../operation.md#set-the-initial-pose)).

## `slam.yaml`

slam_toolbox (`online_async`) for creating a map. It is only started by
`mapping.launch.py`, see [mapping.md](../mapping.md).

| Parameter | Value | Meaning |
|---|---|---|
| `mode` | `mapping` | Build a new map. |
| `odom_frame`, `map_frame`, `base_frame`, `scan_topic` | `odom`, `map`, `base_link`, `/scan` | slam_toolbox publishes `map → odom` while mapping. |
| `resolution` | 0.05 m | Map cell size. The masks must use the same size. |
| `min_laser_range`, `max_laser_range` | 0.30, 15.5 m | Range window used for mapping. |
| `map_update_interval` | 2.0 s | How often `/map` is republished. |
| `transform_publish_period`, `transform_timeout`, `tf_buffer_duration` | 0.02 s, 0.2 s, 30 s | TF handling. |
| `throttle_scans`, `scan_queue_size`, `minimum_time_interval` | 1, 1, 0.1 s | Process every scan, never queue old ones. |
| `minimum_travel_distance`, `minimum_travel_heading` | 0.05 m, 0.05 rad | Low, so slow manual motion still adds scans. |
| `use_scan_matching`, `use_scan_barycenter` | true | Refine odometry by scan matching. |
| `scan_buffer_size`, `scan_buffer_maximum_scan_distance` | 10, 10 m | Local scan chain. |
| `link_match_minimum_response_fine`, `link_scan_maximum_distance` | 0.1, 1.5 m | Linking scans into the chain. |
| `do_loop_closing`, `loop_search_maximum_distance`, `loop_match_*` | true, 3.0 m, see file | Loop closure: how far to search and how good a match must be. |
| `correlation_search_space_*`, `loop_search_space_*` | see file | Search window sizes and resolution of the scan matcher. |
| `distance_variance_penalty`, `angle_variance_penalty`, `minimum_*_penalty` | see file | Penalties for matches far from the odometry guess. |
| `fine_search_angle_offset`, `coarse_search_angle_offset`, `coarse_angle_resolution` | see file | Angular search. |
| `use_response_expansion`, `min_pass_through`, `occupancy_threshold` | true, 2, 0.1 | Map rendering. |
| `solver_plugin`, `ceres_*` | Ceres, sparse Cholesky | Pose-graph optimizer. |
| `enable_interactive_mode`, `use_map_saver`, `stack_size_to_use` | true, true, 40 MB | Interactive graph editing, map saver service, stack for large maps. |
| `restamp_tf`, `debug_logging`, `use_sim_time` | false | |
