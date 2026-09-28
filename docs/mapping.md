# Mapping

A map is a pair of files: an occupancy image (`.pgm`) and its metadata
(`.yaml`). The normal stack loads `maps/current.yaml` at boot and localizes
against it with AMCL. `maps/` is bind-mounted to `/bagheera_ws/maps` in the
container and is **not** tracked by git: maps, masks, calibrations and saved
poses belong to your site.

A complete setup in `maps/` looks like this:

```text
maps/
├─ office.pgm, office.yaml                 saved map (any name)
├─ current.yaml -> office.yaml             symlink: the map loaded at boot
├─ keepout_mask.pgm/.yaml                  areas Nav2 must never enter
├─ localization_exclusion_mask.pgm/.yaml   areas the robot can never be in
├─ last_pose.json                          written by bagheera_pose_persistence
└─ compass_calibration.yaml, …             optional calibrations and test results
```

## 1. Prepare

- Calibrate first: wheel odometry, LiDAR yaw and optical flow
  ([calibration.md](calibration.md)). A map made with a wrong `lidar_yaw` or
  wheel scale is bent.
- Open doors you want in the map and clear the floor of loose objects.
- Plan a route that returns to already mapped areas (loop closure).

## 2. Start mapping mode

Stop the normal service and start the base without map localization and
navigation, then start SLAM Toolbox in the same container:

```bash
docker compose stop bagheera-base
docker compose run --rm --name bagheera-mapping bagheera-base \
  ros2 launch bagheera_base manual_control.launch.py \
  use_map_localization:=false use_navigation:=false
```

Second shell:

```bash
docker exec -d bagheera-mapping /bagheera_entrypoint.sh \
  ros2 launch bagheera_base mapping.launch.py
```

slam_toolbox now publishes `/map` and `map → odom` (see
[config/localization.md](config/localization.md#slamyaml)).

## 3. Watch it in Foxglove

Connect Foxglove to `ws://<robot>:8765`, add a **3D** panel with fixed frame
`map` and enable `/map`, `/scan`, the robot model and `/odometry/filtered`.
The map updates every 2 s.

## 4. Drive

Enable the game controller first if you have not (`enabled: true` under
`bagheera_controller` in `config/base.yaml`, then `docker compose restart`).
Hold the deadman button and drive with the controller. Forward/reverse and
rotation are exclusive, and speed is limited to 0.16 m/s and 0.30 rad/s.

- Drive slowly and smoothly. Stop before you turn.
- Keep walls within a few metres of the LiDAR; large empty halls give scan
  matching nothing to hold on to.
- Close loops: come back through an area you already mapped and watch the
  map snap into place.
- Walk behind the robot, not in front of it: people end up in the map.

## 5. Save

Save both the occupancy map and the pose graph (the pose graph lets
slam_toolbox continue this map later):

```bash
docker exec -it bagheera-mapping /bagheera_entrypoint.sh bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /bagheera_ws/maps/office}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /bagheera_ws/maps/office}"
```

This writes `office.pgm`, `office.yaml`, `office.posegraph` and
`office.data` to `maps/` on the host. Files created by the container belong
to root; use `sudo` for the following steps if needed.

## 6. Clean up the map (optional)

Open `office.pgm` in an image editor (GIMP keeps 8-bit greyscale PGM):
white = free, black = occupied, grey = unknown. Remove people, chairs and
speckles; close gaps in walls behind which the robot must not plan. Keep the
image size unchanged, otherwise the origin in `office.yaml` no longer fits.
Saving under a new name (for example `office_edited.pgm`) needs a matching
YAML whose `image:` points to it.

## 7. Select the map

```bash
cd maps
ln -sfn office.yaml current.yaml
```

`bagheera_pose_persistence` discards the saved pose when the selected map
changes, so after switching maps you must set the initial pose once.

## 8. Masks

Nav2 needs both mask files at boot. Create them empty (all free) with
exactly the map's size, resolution and origin:

```bash
python3 tools/make_masks.py maps/current.yaml
```

Then paint forbidden areas **black** into the `.pgm` files (keep the image
size):

- `keepout_mask.pgm`: areas Nav2 must never plan or drive into (stairs,
  cable areas, rooms the robot should not enter).
- `localization_exclusion_mask.pgm`: places the robot can never physically
  be, such as the other side of a glass wall. If AMCL jumps there, the pose is
  reset to the last valid one.

Run the tool again with `--force` only if you want to reset a mask. After a
new map, recreate the masks: they must match the map geometry.

## 9. Back to normal operation

End the mapping container (Ctrl-C in the first shell) and start the normal
stack:

```bash
docker compose up -d
```

Set the initial pose in Foxglove ([operation.md](operation.md#set-the-initial-pose))
and check that `/scan` lies on the walls of `/map` while you drive a bit.
Then measure the dock pose for this map and put it into `maps/dock.yaml`
([docking.md](docking.md#3-dock-pose-and-staging-pose)): it is a map
coordinate and changes with every new map.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Walls appear double or bent after turns | `lidar_yaw` or LiDAR x/y offset wrong; check with `bagheera_rotation_shift_test` ([diagnostics.md](diagnostics.md#rotation-shift-test)). |
| Map stretched or compressed along corridors | `ticks_per_meter` or optical-flow scale wrong. |
| Map rotates slowly while driving straight | Gyro bias: the robot moved during the first 4 s after start. Restart and keep it still. |
| No loop closure | Loop too long or too few features; drive the loop again more slowly. |
| Nav2 does not start after mapping | `current.yaml` or a mask file missing, or mask size different from the map. |
