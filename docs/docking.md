# Docking

Bagheera docks with Nav2's `opennav_docking` server and its own dock plugin
`bagheera_docking::TagChargingDock`. The plugin finds the dock with two
AprilTags seen by the front fisheye camera. All parameters:
[config/docking.md](config/docking.md).

## How a docking run works

1. **Trigger.** `/dock/trigger` → `bagheera_dock_trigger` sends a
   `DockRobot` goal for `home_dock`.
2. **Staging.** If the robot is more than 15 cm from the staging pose (about
   0.63 m in front of the dock, both tags in view), Nav2 drives there with a
   5 cm / 5° goal checker. `bagheera_dock_trigger` then turns the last degrees
   slowly onto the staging heading (`STAGING_ALIGN`).
3. **Initial perception.** Only now is the camera switched on. The docking
   server waits for both tags:
   - **ID 1** (small, on the dock) gives the dock **position**;
   - **ID 0** (large, on the wall above the dock) gives the dock **axis
     angle**. The small tag's own angle is far too noisy at 0.6 m; without
     ID 0 the attempt stops instead of guessing.
4. **Approach.** Nav2's graceful controller converges position and heading
   onto the dock axis at 0.05–0.10 m/s and ends at the pre-dock pose 17 cm
   in front of contact. The dock is static in `odom`, so detections (which
   arrive ~0.9 s late on a Pi 4) are transformed with the TF of their exposure
   time and filtered. Close to the dock the last pose is held (the tags leave
   the image).
5. **Straight final approach.** At the pre-dock pose `bagheera_dock_trigger`
   drives straight at 0.05 m/s with gyro heading hold until the charge
   voltage appears (≥ 0.5 V) or 3 cm past the expected contact. If the axle is
   more than 2.5 cm off the axis at the hand-over, the plugin reports the dock
   as lost and the server retries from staging immediately.
6. **Charge.** Success is `/docked` (≥ 10 V for 1 s) within 20 s. On failure
   the robot returns to staging and tries again (3 retries, `max_retries`).

`/dock/status` shows the phase as JSON: `NAV_TO_STAGING_POSE`,
`STAGING_ALIGN`, `INITIAL_PERCEPTION`, `CONTROLLING`, `FINAL_APPROACH`,
`PRE_DOCK_OFF_AXIS`, `WAIT_FOR_CHARGE`, `SUCCEEDED` or `FAILED` (with Nav2's
error). The docking server logs one `Dock estimate:` line per second with the
fused tag position, the axis angle and the robot's offset to the dock
(`along`, `left`, `yaw`).

Leaving the dock is done by `bagheera_autonomy_dock_guard` before the next
autonomous goal: reverse 0.80 m along the axis, turn 90° left.

## Setting up a dock

### 1. Dock hardware

- Charging contacts that the robot's contacts reach when it drives straight
  into the dock, wired to the mainboard's charge input. `v_charge` in
  `/hardware_bridge/power` must rise when the robot is pushed onto the
  contacts by hand.
- A dock the robot can enter head-first. The last part of the approach is
  not collision-checked.

### 2. Camera calibration and tags

1. Calibrate the camera ([calibration.md](calibration.md#7-camera)). Tag
   geometry depends on `camera_fisheye.yaml`.
2. Print the tags (tagStandard41h12):

   ```bash
   python3 tools/make_apriltags.py --out apriltags.pdf
   ```

   Defaults: ID 0 at 160 mm, ID 1 at 48 mm, `--print-scale 0.96` (the
   author's printer printed 4 % small). Print at 100 % / actual size and
   measure the outer black edge with a ruler; adjust `--print-scale` until it
   matches.
3. Mount **ID 1** on the dock, centred on the dock axis, where the camera
   sees it from the staging pose until a few centimetres before contact.
   Mount **ID 0** flat on the wall above the dock, facing along the dock
   axis, visible from the staging pose.
4. If you change the sizes, update the **detected border size** (5/9 of the
   printed size) in `dock_apriltag` and `bagheera_dock_tag_pose`.

### 3. Dock pose and staging pose

The dock pose is a map coordinate. Measure it again after every new map.

1. **Position.** Put the robot on the charger (localized; `/docked` true)
   and read the AMCL pose:

   ```bash
   ros2 topic echo /amcl_pose --once
   ```

   Its x/y are the dock position. Its heading is a first guess for the dock
   yaw. Enter `[x, y, yaw]` as `docking_server.home_dock.pose` in
   `nav2_navigation.yaml`. This is the only place; `bagheera_pose_persistence`
   gets it from there.
2. **Staging pose.** Drive the robot to where it should start the approach:
   about 0.6 m in front of the dock, on the axis, facing the dock, with both
   tags in the camera image (check `/camera/h264`). Read `/amcl_pose` again:
   `(sx, sy, syaw)`. Convert it into the dock frame with the dock pose
   `(dx, dy, dyaw)`:

   ```text
   staging_x_offset   =  cos(dyaw)·(sx−dx) + sin(dyaw)·(sy−dy)
   staging_y_offset   = −sin(dyaw)·(sx−dx) + cos(dyaw)·(sy−dy)
   staging_yaw_offset =  syaw − dyaw
   ```

3. **Better dock yaw (recommended).** AMCL's heading is least reliable right
   at the dock. Trigger a docking run from the staging pose and read the first
   `Dock estimate:` line while the robot stands there: `yaw` is the robot's
   heading relative to the tag-derived dock axis. Then
   `dock_yaw = syaw − yaw` (with `syaw` from `/amcl_pose` at the same
   moment). Enter it and recompute the staging offsets so the staging map
   pose stays the same.
4. `docker compose restart`.

### 4. Tag offsets

The plugin needs to know where `base_link` is relative to tag ID 1 when the
robot is docked (`external_detection_translation_x/y`) and how ID 0's
direction relates to the docked heading (`axis_yaw_offset`).

```bash
docker exec -it bagheera-base bash -lc \
  'source /opt/ros/kilted/setup.bash && source /bagheera_ws/install/setup.bash && ros2 run bagheera_base bagheera_dock_calibrate'
```

1. Start with the robot charging in the dock. The tool records the docked
   odometry pose. It sends no motion commands.
2. Reverse out with the controller (0.3–0.8 m) and stop with both tags in
   view. Every stationary stop gives one sample.
3. Repeat from a few positions and press Ctrl-C. The tool prints the averages
   and saves them to `maps/dock_calibration_*.json`.

Trust `external_detection_translation_y` and `axis_yaw_offset` if the
samples agree. `translation_x` is usually off by centimetres because the
wheels slip while leaving the dock. Determine x from the docked camera image
instead: the distance from the camera to ID 1 at contact plus the camera's x
in `robot.yaml`, as a negative value. If the robot stops short of the
contacts or pushes the dock, correct `external_detection_translation_x` in
small steps.

### 5. Test

Place the robot about 1 m in front of the dock and trigger docking. Watch
`/dock/status`, the `Dock estimate:` lines and `/dock_pose` in Foxglove.
Then test from farther away, so the Nav2 staging drive is included.

| Symptom | Check |
|---|---|
| `FAILED` in `INITIAL_PERCEPTION` | Both tags visible from the staging pose? ID 0 large enough (`minimum_axis_edge_pixels`)? Light: the camera meters the centre of the image. |
| Repeated `PRE_DOCK_OFF_AXIS` | Staging pose not on the axis, or `external_detection_translation_y` / `axis_yaw_offset` wrong. |
| Stops short of the contacts | `external_detection_translation_x` too negative, or `final_overtravel_m` too small. |
| Pushes the dock | `external_detection_translation_x` not negative enough. |
| Drives to a wrong staging pose | Dock pose or staging offsets wrong, or AMCL mislocalized. |

## Legacy controller

`legacy/docking/` contains the earlier hand-written docking state machine.
It is kept for reference only; it is not built or launched.
