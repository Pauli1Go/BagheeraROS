# Navigation: `nav2_navigation.yaml`

`nav2_navigation.yaml` holds all Nav2 servers, costmaps and masks, and the
parameters of the Bagheera nodes that sit around Nav2. The docking sections
(`dock_apriltag`, `docking_server`, `bagheera_dock_*`) are described in
[docking.md](docking.md).

Design decisions that shape this file:

- **NavFn + Rotation Shim + Regulated Pure Pursuit (RPP).** A state-lattice
  planner (Smac) was tried and reverted: its paths turn in place mid-route,
  which RPP cannot follow, and the robot got stuck in doorways.
- **No Spin recovery.** Recoveries are clear/replan, a collision-checked
  BackUp and waiting ([behavior trees](#behavior-trees)).
- **No collision monitor in the chain.** Costmaps, RPP's collision check and
  replanning handle obstacles; the firmware watchdog and emergency inputs
  remain the last line.
- `base_link` is the rear axle. The footprint reaches 0.44 m ahead but only
  0.09 m behind. Near walls a turn can therefore be blocked by the nose,
  which the BackUp recovery handles.

## Footprint

```text
footprint: "[[0.4434, 0.18], [0.4434, -0.18], [-0.0866, -0.18], [-0.0866, 0.18]]"
```

This is the measured chassis (x = −0.0766…0.4334 m, y = ±0.17 m around the
axle) plus 1 cm on every side; `footprint_padding` is therefore 0. The same
string appears in `local_costmap`, `global_costmap`,
`bagheera_goal_pose_bridge` and `collision_monitor.yaml`. **Change all four
together.**

## Speed overrides from the launch file

`navigation.launch.py` overrides some values in this file depending on the
`enable_higher_speeds` launch argument (default `true`):

| Parameter | `false` (as in the file) | `true` (default) |
|---|---:|---:|
| `FollowPath.desired_linear_vel` | 0.16 m/s | 0.32 m/s |
| `FollowPath.rotate_to_heading_angular_vel` | 0.30 rad/s | 0.45 rad/s |
| `FollowPath.min_approach_linear_velocity`, `regulated_linear_scaling_min_speed` | 0.05 m/s | 0.10 m/s |
| `behavior_server.max_rotational_vel` / `min_rotational_vel` | 0.30 / 0.10 rad/s | 0.45 / 0.15 rad/s |
| `velocity_smoother.max_velocity` | [0.16, 0, 0.30] | [0.32, 0, 0.45] |
| `velocity_smoother.min_velocity` | [−0.08, 0, −0.30] | [−0.16, 0, −0.45] |

The base values are the constants `BASE_LINEAR_SPEED` and
`BASE_ANGULAR_SPEED` at the top of `navigation.launch.py`. Docking,
undocking and teleop use their own limits.

## `bt_navigator`

| Parameter | Value | Meaning |
|---|---|---|
| `global_frame`, `robot_base_frame`, `odom_topic` | `map`, `base_link`, `/odometry/filtered` | |
| `bt_loop_duration` | 100 ms | BT tick. Every tick also publishes action feedback, which each Python client (patrol, goal bridge) receives at ~2–3 ms of executor overhead; 10 ms cost noticeable CPU. The controller keeps its own 15 Hz. |
| `filter_duration` | 0.3 s | Speed filter for feedback. |
| `default_server_timeout`, `wait_for_service_timeout` | 100 ms, 5000 ms | BT node timeouts. |
| `navigators` | `navigate_to_pose` | Only NavigateToPose is used. |
| `default_nav_to_pose_bt_xml` | set by the launch file | `behavior_trees/navigate_no_spin.xml`. |
| `bt_search_directories` | Nav2 default | |
| `error_code_name_prefixes` | backup, compute_path, follow_path, nav_to_pose, wait | Error codes reported in the action result. |
| `enable_groot_monitoring`, `groot_server_port` | false, 1667 | Groot live view off. |

## `controller_server`

| Parameter | Value | Meaning |
|---|---|---|
| `controller_frequency` | 15 Hz | Control loop rate. |
| `costmap_update_timeout` | 0.5 s | Fail the cycle if the local costmap is older. |
| `min_x/y/theta_velocity_threshold` | 0.001, 0.5, 0.001 | Odometry values below this count as zero. |
| `failure_tolerance` | 0.3 s | Tolerated controller failures before aborting. |
| `enable_stamped_cmd_vel` | true | Kilted uses `TwistStamped`. |

### `progress_checker` (PoseProgressChecker)

| Parameter | Value | Meaning |
|---|---|---|
| `required_movement_radius` | 0.08 m | … or |
| `required_movement_angle` | 0.10 rad | … must be moved/turned within … |
| `movement_time_allowance` | 20 s | … this time, otherwise the goal fails. Turns count as progress, so the final heading alignment does not trigger recovery. |

### Goal checkers

| Checker | xy / yaw tolerance | Used by |
|---|---|---|
| `goal_checker` | 0.10 m / 0.15 rad | normal goals (`navigate_no_spin.xml`) |
| `precise_goal_checker` | 0.05 m / 5° | docking staging pose (`navigate_to_staging.xml`) |

`stateful: true`: once the position is reached, only the heading is checked.

### `FollowPath` (Rotation Shim → RPP)

Rotation shim (turns in place before following a new path):

| Parameter | Value | Meaning |
|---|---|---|
| `angular_dist_threshold` | 0.20 rad | Rotate in place first if the path starts more than this off the current heading. |
| `angular_disengage_threshold` | 0.10 rad | Hand over to RPP below this. |
| `forward_sampling_distance` | 0.30 m | Path point used for the initial heading. |
| `rotate_to_heading_angular_vel` | 0.30 rad/s | In-place turn speed (overridden by the launch file). |
| `closed_loop` | false | Open-loop turn: the closed loop tracked the delayed measured yaw rate and made small turns take 10 s. |
| `max_angular_accel` | 8.0 rad/s² | Lets the command jump to speed at once; the velocity smoother limits the real acceleration. |
| `simulate_ahead_time` | 1.0 s | Collision check of the turn. |
| `rotate_to_goal_heading` | false | RPP owns the final heading. |
| `rotate_to_heading_once` | true | Rotate only for the initial alignment, not after every 1 Hz replan. |
| `use_path_orientations` | false | Use path geometry, not planner orientations. |

Regulated Pure Pursuit:

| Parameter | Value | Meaning |
|---|---|---|
| `desired_linear_vel` | 0.16 m/s | Cruise speed (overridden by the launch file). |
| `lookahead_time`, `use_velocity_scaled_lookahead_dist` | 3.0 s, true | Lookahead grows with speed … |
| `lookahead_dist`, `min_lookahead_dist`, `max_lookahead_dist` | 0.55, 0.45, 0.75 m | … within these limits. Long lookahead calms the steering, which reacts with ~1.2 s delay. |
| `use_rotate_to_heading`, `rotate_to_heading_min_angle` | true, 0.35 rad | Turn on the spot when the carrot is more than 20° off. At 0.5 rad a robot next to a door frame projected an arc whose corner touched the frame and reported "collision ahead" on every retry. |
| `stateful` | true | Keep rotating to the goal heading once the position is reached. |
| `allow_reversing` | false | NavFn paths never need reversing. |
| `use_collision_detection`, `max_allowed_time_to_collision_up_to_carrot` | true, 1.5 s | Project the current arc and stop before a collision. |
| `approach_velocity_scaling_dist`, `min_approach_linear_velocity` | 0.40 m, 0.05 m/s | Slow down near the goal. |
| `use_regulated_linear_velocity_scaling`, `regulated_linear_scaling_min_radius`, `regulated_linear_scaling_min_speed` | true, 0.60 m, 0.05 m/s | Slow down in tight curves. |
| `use_cost_regulated_linear_velocity_scaling`, `cost_scaling_dist`, `cost_scaling_gain`, `inflation_cost_scaling_factor` | true, 0.20 m, 1.0, 5.0 | Slow down only when an obstacle is really close. 0.35 m made the robot crawl through every door. |
| `transform_tolerance` | 0.30 s | |
| `max_robot_pose_search_dist` | 5.0 m | Path search window. |
| `use_interpolation` | true | Interpolate the carrot between path points. |

## `local_costmap`

A 3 × 3 m rolling window in `odom` at 5 cm resolution, for the controller.

| Parameter | Value | Meaning |
|---|---|---|
| `update_frequency`, `publish_frequency` | 5 Hz, 2 Hz | |
| `plugins` | `obstacle_layer`, `inflation_layer` | No static map: only live obstacles. |
| `filters` | `keepout_filter` | Keepout mask applied on top. |
| `obstacle_layer.scan.*` | see below | |
| `inflation_layer.inflation_radius`, `cost_scaling_factor` | 0.25 m, 5.0 | Cost falloff around obstacles. |

Obstacle layer (`scan` source; the global costmap uses the same values except
where noted):

| Parameter | Value | Meaning |
|---|---|---|
| `topic`, `data_type` | `/scan`, `LaserScan` | |
| `marking`, `clearing` | true | Mark hits, clear free space along the rays. |
| `min_obstacle_height`, `max_obstacle_height` | 0.0, 1.0 m | Kilted's per-source default of 0 m silently dropped all returns (the scan plane is 0.30 m high). |
| `obstacle_min_range`, `obstacle_max_range` | 0.0, 2.5 m (global 3.0 m) | Marking range. |
| `raytrace_min_range`, `raytrace_max_range` | 0.0, 4.0 m (global 5.0 m) | Clearing range. |
| `observation_persistence` | 0.0 | Only the latest scan counts. |
| `expected_update_rate` | 0.20 s | Warn if scans are older. |
| `inf_is_valid` | true | `inf` returns clear space. |

While the robot sleeps in the dock, `bagheera_dock_sleep` disables the
obstacle layers and pauses the Nav2 lifecycle; both come back before the
robot reports `awake`.

## `global_costmap`

The whole map in `map`, for the planner.

| Parameter | Value | Meaning |
|---|---|---|
| `update_frequency`, `publish_frequency` | 2 Hz, 1 Hz | |
| `track_unknown_space` | true | Unknown cells are not free; with `allow_unknown: false` the planner avoids them. |
| `plugins` | `static_layer`, `denoise_layer`, `obstacle_layer`, `inflation_layer` | |
| `static_layer.map_subscribe_transient_local` | true | Receive the latched `/map`. |
| `denoise_layer.minimal_group_size`, `group_connectivity_type` | 6, 8 | Remove saved-map specks smaller than 6 connected cells. Runs before the live obstacle layer, so current scans are untouched. |
| `obstacle_layer.combination_method` | 1 (max) | Live obstacles are added on top of the static map. |
| `inflation_layer.inflation_radius`, `cost_scaling_factor` | 0.50 m, 3.0 | Wide, gentle falloff keeps NavFn paths in the middle of corridors; a steep one made paths hug walls. |
| `filters` | `keepout_filter` | Keepout mask. |

## `planner_server`

| Parameter | Value | Meaning |
|---|---|---|
| `GridBased.plugin` | `nav2_navfn_planner::NavfnPlanner` | |
| `use_astar` | true | A* instead of Dijkstra. |
| `tolerance` | 0.20 m | Plan to the nearest free cell within this radius of a blocked goal. |
| `allow_unknown` | false | Never plan through unknown space. |
| `expected_planner_frequency`, `costmap_update_timeout` | 1 Hz, 1.0 s | The BT replans every second. |

## `behavior_server`

Only `backup` and `wait` are loaded (no spin).

| Parameter | Value | Meaning |
|---|---|---|
| `backup.acceleration_limit`, `deceleration_limit`, `minimum_speed` | 0.20, −0.30 m/s², 0.05 m/s | BackUp motion profile. Distance and speed come from the BT (0.20 m at 0.08 m/s). |
| `simulate_ahead_time` | 1.5 s | Collision check of the BackUp. |
| `max_rotational_vel`, `min_rotational_vel`, `rotational_acc_lim` | 0.30, 0.10 rad/s, 2.0 rad/s² | (Launch overrides apply.) |
| `local_frame`, `global_frame`, `robot_base_frame`, `transform_tolerance` | `odom`, `map`, `base_link`, 0.30 s | |
| `cycle_frequency` | 10 Hz | |

## `velocity_smoother`

| Parameter | Value | Meaning |
|---|---|---|
| `smoothing_frequency` | 20 Hz | |
| `feedback` | `OPEN_LOOP` | Smooth against the last command, not the measured speed. |
| `max_velocity`, `min_velocity` | see [speed overrides](#speed-overrides-from-the-launch-file) | Hard caps for all autonomous motion. |
| `max_accel`, `max_decel` | [0.25, 0, 2.0], [−0.35, 0, −2.5] | Acceleration limits (linear m/s², angular rad/s²). |
| `scale_velocities` | false | Axes are limited independently. |
| `velocity_timeout` | 0.5 s | Output zero if the controller stops publishing. |
| `deadband_velocity` | 0 | |

The output goes to `/cmd_vel_automatic_raw`, then through the dock guard.

## `bagheera_autonomy_dock_guard`

Sits between the velocity smoother and `twist_mux`. It publishes the debounced
`/docked`. When the robot is on the charger, it holds back the first
non-zero autonomous command, wakes the robot if it sleeps, reverses out along
the dock axis, turns and only then passes Nav2's commands through.

| Parameter | Value | Meaning |
|---|---|---|
| `input_topic`, `output_topic` | `/cmd_vel_automatic_raw`, `/cmd_vel_monitored` | |
| `power_topic`, `odom_topic` | `/hardware_bridge/power`, `/odometry/filtered` | |
| `dock_voltage_threshold`, `dock_debounce_s` | 10 V, 1.0 s | `/docked` becomes true after `v_charge` ≥ 10 V for 1 s. |
| `contact_voltage_threshold` | 0.5 V | Autonomy is already gated at first contact; the charger needs seconds to reach 10 V. |
| `command_timeout_s` | 0.5 s | Input older than this counts as stopped. |
| `reverse_distance_m`, `reverse_speed_mps` | 0.80 m, 0.08 m/s | Undock reverse. |
| `reverse_heading_kp`, `reverse_cross_track_kp`, `reverse_max_angular_rps` | 1.5, 1.0, 0.25 rad/s | Keep the reverse straight on the dock axis. |
| `reverse_stall_window_s`, `reverse_stall_min_progress_m` | 5 s, 0.05 m | Abort when the reverse makes less than 5 cm in 5 s. |
| `reverse_timeout_s` | 60 s | Absolute limit. |
| `turn_angle_rad` | 90° | Turn left after reversing. |
| `turn_max_speed_rps`, `turn_min_speed_rps`, `turn_gain`, `turn_tolerance_rad`, `turn_timeout_s` | 0.30, 0.30, 0.8, 2°, 15 s | Turn controller. |
| `settle_time_s` | 0.30 s | Pause between the phases. |

## `bagheera_goal_pose_bridge`

Turns Foxglove's `/goal_pose` (and the legacy `/move_base_simple/goal`) into
a `NavigateToPose` action. It rejects a goal whose **oriented footprint**
would overlap a wall, unknown space or a keepout cell, and logs where. It
wakes a sleeping robot first and holds the goal until it is awake.

| Parameter | Value | Meaning |
|---|---|---|
| `footprint` | same as the costmaps | Checked at the goal pose. |
| `map_occupied_threshold` | 65 | Map cells at or above this value count as blocked. |

## Masks

Two masks with the same size, resolution and origin as the map live in
`maps/`. Black (0) marks a forbidden cell, white (254) a free one. Create
them with `tools/make_masks.py` ([mapping.md](../mapping.md#8-masks)).

| Node | Parameter | Value |
|---|---|---|
| `keepout_filter_mask_server` | `yaml_filename`, `topic_name`, `frame_id` | `/bagheera_ws/maps/keepout_mask.yaml`, `/keepout_filter_mask`, `map` |
| `localization_exclusion_mask_server` | same | `/bagheera_ws/maps/localization_exclusion_mask.yaml`, `/localization_exclusion_mask`, `map` |
| `costmap_filter_info_server` | `type`, `filter_info_topic`, `mask_topic`, `base`, `multiplier` | 0 (keepout), `/costmap_filter_info`, `/keepout_filter_mask_live`, 0.0, 1.0 |
| `bagheera_keepout_mask_relay` | `input_topic`, `output_topic`, `repeat_topics` | Latches the keepout mask for the costmaps and re-sends both masks to late subscribers (Foxglove often connects before the map servers publish). |
| `bagheera_localization_exclusion_guard` | `mask_topic`, `occupied_threshold`, `reset_cooldown_s`, `odom_topic` | `/localization_exclusion_mask`, 50, 1.0 s, `/odometry/filtered_throttled`: an AMCL pose on a mask value ≥ 50 is replaced by the last valid pose projected forward with odometry (5 Hz is enough for that), at most once per second. |

- **Keepout**: the costmaps treat these cells as lethal. Nav2 never plans
  or drives there (stairs, cable areas, rooms the robot should not enter).
- **Localization exclusion**: places the robot can never physically be (the
  other side of a glass wall, inside furniture). If AMCL jumps there, the pose
  is reset.

## Behavior trees

`behavior_trees/navigate_no_spin.xml` (normal goals) and
`navigate_to_staging.xml` (docking staging pose; identical except for the
precise goal checker).

```text
RecoveryNode (number_of_retries = 2)
├─ PipelineSequence
│   ├─ RateController 1 Hz → ComputePathToPose (GridBased)
│   └─ FollowPath (FollowPath, goal checker)
└─ RoundRobin (one recovery per failure, in turn)
    ├─ ClearAndReplan: clear local + global costmap, wait 0.5 s
    ├─ BackUpOrSkip: BackUp 0.20 m at 0.08 m/s (collision checked), else wait 0.5 s
    └─ WaitThenReplan: wait 20 s, clear both costmaps, wait 0.5 s
```

- The path is recomputed every second, so new obstacles are avoided without
  a failure.
- With 2 retries, a goal aborts after clear/replan and one BackUp. The 20 s
  wait is only reached if `number_of_retries` is raised.
- BackUp helps when the long nose blocks a turn next to a door frame: 10 cm
  further back the turn is often free.

## `collision_monitor.yaml`

Not launched. It is kept as a ready configuration (slow-down zone and stop
polygon with the same footprint) in case a collision monitor is added between
the velocity smoother and the dock guard again.
