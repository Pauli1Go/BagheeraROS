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
autonomous goal: reverse along the dock axis, then turn, both per site in
`maps/dock.yaml` (default 0.80 m and 90° left). With `turn_angle_deg: 0` it
only reverses and Nav2 turns onto its path. Pick the turn so that the robot
ends up in free space at your dock.

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
3. Mount the tags (values measured on Bagheera; camera at 12 cm, level):

   ```text
   side view                                  front view (from the robot)

   wall │                                         ┌─────────┐
        │ ┌──┐  ID 0, 160 mm                      │  ID 0   │  centre ~42 cm
        │ │  │  centre ~42 cm above the floor     │ 160 mm  │  above floor
        │ └──┘                                    └─────────┘
        │          ~29 cm centre to centre             │ same vertical axis
        │      ┌┐  ID 1, 48 mm, ~11 cm in front        ┌┐
        │      └┘  of the wall, centre at camera   ●   └┘   ●   pins
        │ dock ██  height (~12–13 cm)                  ID 1
   ─────┴──────██──────────── floor
   ```

   - **ID 1** (small): on the dock, centred **between the charging pins**, its
     centre at **camera height** (~12–13 cm above the floor), facing the
     robot along the dock axis. The camera must see it from the staging pose
     until a few centimetres before contact.
   - **ID 0** (large): flat on the wall behind the dock, **vertically above
     ID 1** on the same axis, ~29 cm centre to centre (its centre ~42 cm above
     the floor).
   - Both upright (as printed, top edge up) and flat; they must not move
     afterwards.
   - The exact height of ID 0 is not critical: only its left-right facing is
     used, and a fixed rotation is absorbed by `axis_yaw_offset`. Mount it as
     low as possible without ID 1 or the dock covering it. Seen steeply from
     below it gets foreshortened, lands at the fisheye edge and its angle
     gets noisier; its shortest edge must stay ≥ 60 px
     (`minimum_axis_edge_pixels`) from the staging pose.
4. If you change the sizes, update the **detected border size** (5/9 of the
   printed size) in `dock_apriltag` and `bagheera_dock_tag_pose`.

### 3. Dock pose and staging pose

The dock pose and the undock manoeuvre belong to the site, like the map.
They live in `maps/dock.yaml` (mounted, not in git); without that file the
example values in `nav2_navigation.yaml` apply. The staging pose, the tag
offsets and everything else are relative to the dock and stay the same when
the dock moves.

```yaml
# maps/dock.yaml: charging dock of this site (not in git)
dock_pose: [1.192, 1.884, 1.624]   # map x, y [m], yaw [rad]
undock:
  reverse_distance_m: 0.80         # straight back out of the dock
  turn_angle_deg: 90.0             # + left, - right, 0 = no turn
axis_yaw_offset_rad: 0.0055        # ID 0 (wall) direction -> docked heading
```

`axis_yaw_offset_rad` is here because ID 0 hangs on the site's wall. ID 1
sits on the dock, so its offsets (`external_detection_translation_x/y`) stay
in `nav2_navigation.yaml` and move with the dock.

The launch files pass it to the docking server, `bagheera_pose_persistence`
and the dock guard (the start-up log shows `Dock site from …`). Measure the
dock pose again after every new map and every time the dock is moved.

**Dock lock.** While `/docked` is true, `bagheera_pose_persistence` pins AMCL
to the *configured* dock pose. After moving the dock the robot therefore
snaps to the old place as soon as it charges, and reading `/amcl_pose` in
the dock only returns the old value. It warns (`Docked … m away from the
configured dock pose`) if it docks more than 0.5 m from it. Hence the dock
pose is measured with the dock **unpowered**.

#### Measuring the dock (new site or moved dock)

`bagheera_dock_setup` measures everything from the unpowered dock and writes
`maps/dock.yaml`:

1. Place the dock with its tags ([step 2](#2-camera-calibration-and-tags))
   and **unplug its power supply**. Without charge voltage there is no
   `/docked` and no dock lock.
2. Localize the robot (Foxglove *2D pose estimate* if needed; check that
   `/scan` lies on the walls of `/map`).
3. Put the robot onto the dock until the pins touch: push it by hand or drive
   it with the game controller (`enabled: true` in `base.yaml`).
4. Run the tool (keep 0.6 m behind the robot free):

   ```bash
   docker exec -it bagheera-base bash -lc \
     'source /opt/ros/kilted/setup.bash && source /bagheera_ws/install/setup.bash && ros2 run bagheera_base bagheera_dock_setup --execute'
   ```

   It
   - **asks for the undock manoeuvre**: reverse distance and turn after
     reversing (+ left, − right, 0 = no turn and Nav2 turns onto its path),
     proposing the current values. Choose them so that the robot ends up in
     free space at this dock;
   - refines AMCL on the spot (aborts if its position std exceeds 10 cm)
     and averages the docked pose;
   - reverses 0.6 m straight at 5 cm/s with heading hold, switches the dock
     camera on and measures both tags standing still;
   - computes the dock pose: x/y from AMCL on the dock, yaw from AMCL out
     there carried back by odometry (AMCL's heading is poorest right at the
     dock), and ID 0's `axis_yaw_offset`;
   - checks ID 1's measured offsets against `nav2_navigation.yaml` and warns
     if ID 1 seems to have moved on the dock;
   - prints the result and the resulting staging pose, asks before writing,
     keeps the previous file as `dock.yaml.<time>.bak` and saves a report
     `maps/dock_setup_<time>.json`.

   Options: `--undock-reverse`, `--undock-turn-deg` (skip the questions),
   `--yes` (no questions at all), `--measure-distance` (default 0.6 m),
   `--speed` (default 0.05 m/s).
5. Plug the dock in again, `docker compose restart`, and test a docking run
   ([step 5](#5-test)).

The staging pose does not have to be set again: it is defined relative to
the dock (`staging_*_offset`) and moves with it.

#### Staging offsets (first setup of a dock design only)

The staging pose is where the approach starts: about 0.6 m in front of the
dock, on the axis, facing the dock, with both tags in the camera image. The
shipped offsets fit Bagheera's dock and tags. For a different dock or camera,
drive the robot there, read `/amcl_pose` `(sx, sy, syaw)` and convert it into
the dock frame with the dock pose `(dx, dy, dyaw)`:

```text
staging_x_offset   =  cos(dyaw)·(sx−dx) + sin(dyaw)·(sy−dy)
staging_y_offset   = −sin(dyaw)·(sx−dx) + cos(dyaw)·(sy−dy)
staging_yaw_offset =  syaw − dyaw
```

Enter them in the `bagheera_dock` block of `nav2_navigation.yaml` and
restart.

### 4. Tag offsets

The plugin needs to know where `base_link` is relative to tag ID 1 when the
robot is docked (`external_detection_translation_x/y`) and how ID 0's
direction relates to the docked heading (`axis_yaw_offset`). The ID 1 offsets
belong to the dock design and only need measuring once (the shipped values
fit Bagheera's dock). `axis_yaw_offset` is measured by `bagheera_dock_setup`
at every site; the tool below is for a new dock design.

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
