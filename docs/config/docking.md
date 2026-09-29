# Docking: `nav2_navigation.yaml` docking sections

Procedure, dock hardware and calibration steps are in
[../docking.md](../docking.md). This page lists the parameters.

## Dock pose

```yaml
docking_server:
  ros__parameters:
    docks: ["home_dock"]
    home_dock:
      type: bagheera_dock
      frame: map
      pose: [1.192, 1.884, 1.624]   # EXAMPLE, replace with yours
```

`home_dock.pose` is only the **fallback** for the dock pose `[x, y, yaw]` in
the map frame. The site file `maps/dock.yaml` (`dock_pose`, see
[../docking.md](../docking.md#3-dock-pose-and-staging-pose)) overrides it;
the launch files pass the result to the docking server and to
`bagheera_pose_persistence` (as `dock_x/dock_y/dock_yaw`), which anchors AMCL
there while the robot is docked. The shipped value belongs to the author's
map and is meaningless for yours.

- x/y: the AMCL position of `base_link` while the robot sits in the dock.
- yaw: the direction the robot faces in the dock. Measured best from the
  tags (see [../docking.md](../docking.md#3-dock-pose-and-staging-pose)).

The docking server uses the pose only to compute the staging pose; the final
approach follows the tags.

## `docking_server`

| Parameter | Value | Meaning |
|---|---|---|
| `fixed_frame` | `odom` | Detections are filtered in `odom`: the dock does not move there, while AMCL corrections would move it in `map`. |
| `base_frame`, `odom_topic` | `base_link`, `/odometry/filtered` | |
| `controller_frequency` | 20 Hz | |
| `initial_perception_timeout` | 10 s | Camera start (~2.4 s) plus AprilTag latency (~0.9 s) on a Pi 4. |
| `dock_approach_timeout` | 40 s | Approach from staging to the pre-dock pose. |
| `wait_charge_timeout` | 20 s | From the pre-dock pose until `/docked`, including the straight final drive and the charger ramp to 10 V. |
| `undock_linear_tolerance`, `undock_angular_tolerance` | 0.05 | Not used (undocking is done by the dock guard). |
| `max_retries` | 3 | On failure, return to staging and try again, up to three times. |
| `dock_prestaging_tolerance` | 0.15 m | Closer than this to the staging pose: skip the Nav2 drive. |
| `navigator_bt_xml` | `…/navigate_to_staging.xml` | BT for the staging drive (precise goal checker). |
| `dock_plugins` | `bagheera_dock` | |

### `controller` (Nav2 graceful controller)

| Parameter | Value | Meaning |
|---|---|---|
| `k_phi`, `k_delta`, `beta`, `lambda` | 3.0, 2.0, 0.4, 2.0 | Graceful-controller gains (heading convergence, curvature, speed reduction). |
| `v_linear_min`, `v_linear_max`, `v_angular_max` | 0.05, 0.10 m/s, 0.30 rad/s | Approach speed limits. |
| `slowdown_radius` | 0.25 m | Slow down near the target. |
| `use_collision_detection`, `costmap_topic`, `footprint_topic` | true, local costmap | Collision check of the approach. |
| `projection_time`, `simulation_time_step`, `transform_tolerance` | 5.0 s, 0.1 s, 0.3 s | |
| `dock_collision_threshold` | 0.40 m | No collision check for the last part (measured from the controller target 0.25 m beyond the dock pose, so about 15 cm before contact): the dock and wall are right in front of the chassis there. |

## `bagheera_dock` (plugin `bagheera_docking::TagChargingDock`)

Two AprilTags of family `tagStandard41h12`:

- **ID 1 (small, on the dock)** gives the dock **position**. At the staging
  distance its plane angle is too noisy to use.
- **ID 0 (large, on the wall above)** gives the dock **axis angle**.

| Parameter | Value | Meaning |
|---|---|---|
| `staging_x_offset`, `staging_y_offset`, `staging_yaw_offset` | −0.6325 m, −0.0794 m, −0.017 rad | Staging pose relative to the dock pose (in the dock frame: x along the dock axis, y left). About 0.63 m in front of the dock with both tags in view. y was moved 8.7 cm right because the robot consistently reached staging that far left of the tag-measured axis. |
| `external_detection_translation_x` | −0.468 m | Offset from tag ID 1 to `base_link` at contact, along the dock axis. |
| `external_detection_translation_y` | 0.002 m | Same, sideways. |
| `axis_yaw_offset` | 0.0055 rad | Angle between ID 0's direction and the robot heading in the dock. Depends on the wall ID 0 hangs on: `maps/dock.yaml` (`axis_yaw_offset_rad`, written by `bagheera_dock_setup`) overrides it. |
| `require_axis_tag` | true | Without ID 0, stop instead of guessing the angle. |
| `detection_timeout` | 3.0 s | A detection older than this counts as lost. Detections arrive 0.9–1.1 s after exposure. |
| `position_filter_coef`, `yaw_filter_coef` | 0.3, 0.2 | Low-pass filters on the fused dock pose. |
| `max_position_jump`, `max_yaw_jump` | 0.10 m, 0.17 rad | Ignore single detections that jump more. |
| `hold_distance` | 0.30 m | Closer than this, keep the last pose (ID 1 leaves the image ~4 cm before contact; ID 0 leaves earlier). |
| `pre_dock_distance` | 0.17 m | The graceful controller ends here, in front of the contact pose; `bagheera_dock_trigger` drives the rest straight. |
| `pre_dock_max_lateral`, `pre_dock_max_yaw` | 0.025 m, 10° | Larger offsets at the pre-dock pose report the dock as lost, so Nav2 retries from staging at once. |
| `contact_voltage` | 0.5 V | `v_charge` counted as contact. |
| `position_topic`, `axis_topic` | `/dock/detected_pose`, `/dock/detected_axis` | From `bagheera_dock_tag_pose`. |
| `power_topic`, `docked_topic`, `event_topic` | `/hardware_bridge/power`, `/docked`, `/dock/plugin_event` | |

Debug output: `/dock_pose` (refined dock), `/staging_pose`, and one
`Dock estimate:` log line per second.

## `dock_apriltag` (apriltag_ros)

| Parameter | Value | Meaning |
|---|---|---|
| `image_transport` | `compressed` | Reads the camera's grayscale JPEG, one frame at a time through `bagheera_dock_frame_gate`. |
| `family` | `Standard41h12` | Tag family. |
| `size`, `tag.sizes` | 0.026667; [0.088889, 0.026667] m | **Detected border size**, not the printed size: for tagStandard41h12 the black border is 5/9 of the full 9-cell tag (48 mm printed → 26.67 mm, 160 mm printed → 88.89 mm). |
| `tag.ids`, `tag.frames` | [0, 1]; `dock_tag_far`, `dock_tag_near` | Only these IDs are reported. |
| `max_hamming` | 0 | No bit errors accepted. |
| `pose_estimation_method` | `pnp` | Required by the Kilted node. Its pinhole pose is ignored (remapped to `/dock/tag_tf_unused`). |
| `detector.threads`, `decimate`, `blur`, `refine`, `sharpening` | 2, 2.0, 0.0, true, 0.25 | Detector tuning. Decimation 2 on 1080p keeps the Pi 4 from saturating. |
| `qos_profile` | `sensor_data` | |

## `bagheera_dock_frame_gate`

Forwards only the newest camera frame and waits for the detector's result
before sending the next one (a queue added ~0.8 s latency). Active only while
docking has vision enabled (`/dock/vision_enabled`).

| Parameter | Value | Meaning |
|---|---|---|
| `result_timeout_s` | 1.0 s | Send the next frame even if no result arrived. |

## `bagheera_dock_tag_pose`

Solves the tag poses from the raw corners with the fisheye model of
`camera_fisheye.yaml` and publishes them in `camera_optical_frame` with the
image timestamp.

| Parameter | Value | Meaning |
|---|---|---|
| `position_tag_id`, `position_tag_size_m` | 1, 0.026667 m | Dock tag → `/dock/detected_pose`. |
| `axis_tag_id`, `axis_tag_size_m` | 0, 0.088889 m | Wall tag → `/dock/detected_axis`. |
| `minimum_decision_margin` | 15 | Reject weak detections. |
| `minimum_edge_pixels` | 16 px | Minimum tag edge length in the image. |
| `minimum_axis_edge_pixels` | 60 px | Minimum for the axis tag (it measured 113 px and 0.35° scatter at the staging pose). |
| `maximum_axis_heading_rad` | 30° | Reject axis solutions viewed more obliquely. |
| `camera_frame` | `camera_optical_frame` | |

## `bagheera_dock_trigger`

Turns `/dock/trigger` into a `DockRobot` goal, publishes `/dock/status`,
cancels on `/dock/cancel` or controller input, aligns the heading at the
staging pose, and drives the straight final approach.

| Parameter | Value | Meaning |
|---|---|---|
| `dock_id` | `home_dock` | Dock to use. |
| `max_staging_time_s` | 120 s | Maximum time for the staging drive. |
| `staging_align_gain`, `staging_align_min_angular_rps`, `staging_align_max_angular_rps` | 1.2, 0.06, 0.30 rad/s | Final heading turn at the staging pose (code defaults, not in the YAML). |
| `staging_align_tolerance_rad`, `staging_align_timeout_s` | 1.5°, 8 s | |
| `final_speed_mps` | 0.05 m/s | Straight final approach speed. |
| `final_overtravel_m` | 0.03 m | Stop this far past the expected contact if no voltage appears. |
| `final_heading_gain`, `final_max_angular_rps` | 1.2, 0.20 rad/s | Gyro heading hold during the final approach. |
| `final_trim_max_rad`, `final_trim_distance_m` | 4°, 0.25 m | Small heading trim toward the dock axis. |
| `final_front_offset_m` | 0.47 m | Charging contacts ahead of `base_link` (for logging). |
| `final_max_lateral_m`, `final_max_yaw_rad` | 0.025 m, 10° | Hand-over limits; larger offsets retry from staging. |
| `final_timeout_s` | 12 s | Upper limit for the straight final approach. |
| `contact_voltage` | 0.5 V | Stop at first contact. |
