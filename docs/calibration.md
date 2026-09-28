# Calibration

Calibrate in this order: each step relies on the ones before. All values
live in `src/bagheera_base/config/`; a `docker compose restart` applies them.

| Step | Result | Parameter |
|---|---|---|
| 1. Robot geometry | footprint, sensor poses | `robot.yaml`, footprints in `nav2_navigation.yaml` |
| 2. Wheel odometry | encoder scale | `robot.yaml` → `hardware_bridge.ticks_per_meter`, `wheel_track` |
| 3. IMU mounting | yaw sign | `robot.yaml` → `imu_yaw` |
| 4. LiDAR mounting | scan zero direction | `robot.yaml` → `lidar_yaw` |
| 5. Optical flow | metric scale | `sensors.yaml` → `bagheera_optical_flow.radians_per_count` |
| 6. Battery voltage | offset | `base.yaml` → `voltage_offset_v` |
| 7. Camera | fisheye intrinsics | `camera_fisheye.yaml` |
| 8. Compass (optional) | hard/soft iron | `maps/compass_calibration.yaml` |

Afterwards run the [360° turn test](diagnostics.md#360-turn-test) to check
that all heading sources agree.

## 1. Robot geometry

`base_link` must be the midpoint of the drive axle. Measure from there:

- the chassis outline (front, rear, sides) → the four footprint strings
  (add 1 cm margin; see [config/navigation.md](config/navigation.md#footprint));
- wheel radius and track;
- x/y/z of every sensor.

Sensors that share one mount (as on Bagheera) get identical x/y values.

## 2. Wheel odometry

`ticks_per_meter` is used both for odometry and by the firmware's wheel
speed loop.

1. Mark a start line and drive straight with the controller (or
   `bagheera_straight_drive_test`, below) over 1–3 m at the normal speed.
2. Compare the measured distance with the encoder distance
   (`/wheel_odom` x) and scale:
   `ticks_per_meter_new = ticks_per_meter_old × encoder_distance / measured_distance`.
3. Repeat at 0.08 and 0.16 m/s and average.

Bagheera: 206.0 ticks over 0.80 m and 201.5 ticks over 0.72 m →
(206.0 + 201.5) / (0.80 + 0.72) = **268.1 ticks/m**. Validation with 0.5 m
targets was within 3 cm at 0.08 and 0.16 m/s. At 0.5 m/s the robot ran on
noticeably, which is one reason the normal speeds stay low.

For `wheel_track`, the [360° turn test](diagnostics.md#360-turn-test) reports
the wheel yaw error against the LiDAR and suggests a corrected value. If the
wheels report more rotation than the robot really turned, the track is too
small. On a slippery floor the error is slip, not geometry.

## 3. IMU mounting

The EKF uses only the WT901's Z rate. Turn the robot counter-clockwise by
hand: `angular_velocity.z` in `/imu/wt901/data_raw` must be positive. If it
is negative, the sensor Z axis points down; fix `imu_roll`/`imu_pitch`. Set
`imu_yaw` so the sensor axis that points forward maps to `base_link` X+
(Bagheera: sensor Y+ forward → −90°).

The WT901 measures its gyro bias during the first 4 s after start (and after
every dock wake-up). Keep the robot still then.

## 4. LiDAR mounting

`inverted: true` fixes the G2's clockwise scan order; `lidar_yaw` rotates the
scan so its zero points forward.

1. Place a narrow target (a box, a table leg) straight in front of the robot
   at ~0.5 m.
2. Look at `/scan` in Foxglove's 3D panel with fixed frame `base_link`: the
   target must lie on the +x axis. Adjust `lidar_yaw` until it does.
3. Refine with a mapped turn: `bagheera_rotation_shift_test`
   ([diagnostics.md](diagnostics.md#rotation-shift-test)) reports a
   residual rotation or a lever-arm error of the LiDAR offset.

Bagheera: the target cluster was centred at +85.5°, so first −85.5°; a
mapped 360° turn corrected it by +1.43° to **−84.1°**.

## 5. Optical flow

1. Drive straight at 0.16 m/s over a measured distance (≥ 0.5 m) and record
   the summed X counts on `/optical_flow/raw`.
2. `radians_per_count = (distance / counts) / mount_height_m`.
3. If the forward motion appears on sensor Y or with a negative sign, use
   `swap_xy`, `invert_x`, `invert_y`.

Bagheera: 2755 counts over 0.510 m at 0.09 m height →
0.510 / 2755 / 0.09 = **0.002057 rad/count** (a repeat gave 2723 counts).
The scale depends on the floor surface and the exact mounting height.

## 6. Battery voltage

The mainboard's battery reading can have an offset. Compare a multimeter at
the battery pack with `v_battery` in `/hardware_bridge/power` at two or three
charge levels, off the dock and while charging, and enter the mean difference
as `voltage_offset_v`.

Bagheera (firmware 1.9.10): 25.71/26.49, 28.10/28.90 and 28.37/29.20 V
(real/reported) → constant **−0.80 V**, no gain error. Details:
[config/base.md](config/base.md#battery-voltage-offset).

## 7. Camera

Needed for docking. The calibration is valid only for the configured
resolution and `sensor_mode`.

1. Print a checkerboard (`tools/calib_checkerboard_10x7_25mm_*.pdf`; 10 × 7
   squares → 9 × 6 inner corners) and measure the printed square size.
2. Record images inside the container while you move the board through the
   whole image, including the edges and corners:

   ```bash
   cp tools/save_calib_images.py maps/
   docker exec -it bagheera-base bash -lc 'source /opt/ros/kilted/setup.bash && \
     python3 /bagheera_ws/maps/save_calib_images.py --outdir /bagheera_ws/maps/calib'
   ```

   Keep the camera running meanwhile (Foxglove viewer on `/camera/h264` or
   `/camera/stream_enabled` true).
3. Compute the fisheye model on a machine with OpenCV:

   ```bash
   pip install opencv-python numpy pyyaml
   python3 tools/calibrate_fisheye.py --images maps/calib --pattern 9x6 \
     --square <measured square size in m> \
     --out src/bagheera_base/config/camera_fisheye.yaml
   ```

   Check the printed RMS reprojection error (well below 1 px) and the corner
   coverage grid.

## 8. Compass (optional)

The compass is not fused by default, because indoor steel and motor currents
disturb it. To evaluate it, start with `use_compass:=true`, then:

```bash
docker exec -it bagheera-base /bagheera_entrypoint.sh \
  ros2 run bagheera_base bagheera_compass_calibrate
```

Rotate the robot slowly through at least one full turn (two turns over 60 s
are better). The result is written to `maps/compass_calibration.yaml` and
loaded automatically. Then run the [360° turn test](diagnostics.md#360-turn-test).
