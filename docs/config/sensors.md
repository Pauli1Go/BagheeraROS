# `sensors.yaml`: sensor drivers

Each section is the parameter namespace of one node. Wiring, device
identification and probes are described in [hardware.md](../hardware.md);
how the calibrated values were measured is in
[calibration.md](../calibration.md).

## `ydlidar_ros2_driver_node`: YDLidar G2/G2B

The driver is not started by the launch file directly: `bagheera_dock_sleep`
runs it as a child process so it can stop the LiDAR motor while docked. It
publishes `sensor_msgs/LaserScan` on `/scan` in frame `lidar_link`, about
9.6 scans/s.

| Parameter | Value | Meaning |
|---|---|---|
| `port` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_…` | **Change to your adapter's `by-id` path** (`ls /dev/serial/by-id/`). Unlike `/dev/ttyUSB0` it survives re-enumeration. |
| `frame_id` | `lidar_link` | Must match the URDF. |
| `baudrate` | 230400 | G2 default. |
| `lidar_type`, `device_type` | 1, 0 | Triangulation LiDAR over serial. |
| `sample_rate` | 5 | 5 kHz. |
| `intensity`, `intensity_bit` | true, 8 | The G2 sends 8-bit intensities. |
| `isSingleChannel`, `support_motor_dtr` | false, false | G2 settings. |
| `abnormal_check_count` | 4 | Start-up retries before the driver gives up. |
| `fixed_resolution` | false | Keep every measured point. The G2B varies around 500 points/revolution, and a fixed buffer would drop valid points. |
| `reversion` | false | No extra 180° rotation; the mounting yaw is in `robot.yaml` (`lidar_yaw`). |
| `inverted` | true | The G2 scans clockwise; ROS expects counter-clockwise angles. |
| `auto_reconnect` | true | Reopen the port after an error. |
| `angle_min`, `angle_max` | −180°, 180° | Full circle. |
| `range_min`, `range_max` | 0.20, 16.0 m | The nominal minimum is 0.28 m; 0.20 m was a trial. AMCL and SLAM apply their own minimums. |
| `frequency` | 10 Hz | Requested motor speed. |
| `invalid_range_is_inf` | true | Invalid returns become `inf` (free space for raytracing). |
| `ignore_array` | `""` | Angle ranges to blank out, e.g. for chassis parts in view. |

## `bagheera_dock_sleep`

Puts the sensors to sleep while the robot idles on the charger and wakes them
in a fixed order. States and triggers: [architecture.md](../architecture.md#dock-sleep-docksleep_state).

| Parameter | Value | Meaning |
|---|---|---|
| `sleep_delay_s` | 3.0 s | Docked (debounced `/docked`) and no motion command for this long → sleep. |
| `wake_grace_s` | 30 s | After a wake-up, stay awake at least this long so a held goal or the driver can leave the dock. |
| `lidar_executable` | `…/ydlidar_ros2_driver_node` | Driver binary started as a child process. |
| `lidar_parameters` | `…/config/sensors.yaml` | Parameter file passed to it (this file). |
| `scan_window_s`, `scan_min_rate_hz` | 2.0 s, 8 Hz | Wake-up requires `/scan` at ≥ 8 Hz for 2 s. |
| `sensor_timeout_s` | 30 s | A wake step that takes longer → `fault`. |
| `ekf_settle_s` | 0.5 s | Pause after resetting the EKF before anchoring AMCL. |
| `anchor_timeout_s` | 15 s | Maximum wait for AMCL to accept the dock pose. |
| `startup_nodes` | `/bt_navigator`, `/docking_server` | The first sleep waits until these are active (otherwise Nav2 bring-up would starve without scans). |
| `navigation_manager` | `/lifecycle_manager_navigation` | Paused while asleep, resumed on wake. |
| `navigation_resume_timeout_s` | 30 s | Wait for Nav2 to become active again. |
| `lidar_enabled`, `wt901_enabled`, `optical_flow_enabled`, `ekf_reset_enabled`, `anchor_enabled`, `navigation_enabled` | from launch arguments | Skip the corresponding step when that part is not launched. |

## `bagheera_optical_flow`: PMW3901

C++ component (`bagheera_sensors::OpticalFlowNode`) in `sensor_container`.

A downward-facing PMW3901 on SPI0, polled without an interrupt pin. It
publishes raw counts and quality on `/optical_flow/raw` and a metric
`TwistWithCovarianceStamped` for `base_link` on `/optical_flow/twist`. The EKF
fuses only its forward component.

| Parameter | Value | Meaning |
|---|---|---|
| `spi_bus`, `spi_chip_select`, `spi_speed_hz` | 0, 0, 400 kHz | `/dev/spidev0.0`. |
| `frame_id` | `base_link` | The twist is already transformed to `base_link`. |
| `publish_rate` | 25 Hz | Matches the EKF rate. Counts accumulate between reads, so nothing is lost. |
| `mount_height_m` | 0.09 m | Sensor-to-floor distance. The metric scale is proportional to it. |
| `radians_per_count` | 0.002057 | **Calibrated**: 2755 counts over a measured 0.510 m at 0.09 m height. |
| `swap_xy`, `invert_x`, `invert_y` | false | Map sensor axes to robot axes (sensor X is forward on Bagheera). |
| `minimum_quality` | 0 | Samples with a lower surface-quality value are dropped. |
| `lever_arm_x`, `lever_arm_y` | 0.1834, 0.0110 m | Sensor offset from `base_link`. Removes the circular motion the sensor sees while the robot pivots. |
| `rotation_gate_enabled`, `rotation_gate_rad_s` | true, 0.08 rad/s | While the gyro reports more than this yaw rate, the twist is published with a huge variance, so the EKF effectively ignores it. During turns it otherwise seeded a false EKF velocity. |
| `imu_topic`, `imu_max_age` | `/imu/wt901/data_raw`, 0.5 s | Gyro used for the lever-arm correction and the gate. If no gyro sample is younger than `imu_max_age`, the flow is published uncorrected (normal for the 4 s WT901 bias measurement). |

## `bagheera_wt901`: WT901 IMU

C++ component (`bagheera_sensors::Wt901Node`) in `sensor_container`.

A WIT Motion WT901 on I2C1. It publishes `sensor_msgs/Imu` on
`/imu/wt901/data_raw` and, with `use_compass:=true`, `MagneticField` on
`/imu/wt901/mag_raw`. The EKF uses only its Z angular velocity.

| Parameter | Value | Meaning |
|---|---|---|
| `i2c_bus`, `i2c_address` | 1, 80 (0x50) | `i2cdetect -y 1` must show `50`. |
| `publish_rate` | 25 Hz | Register poll rate, matching the 25 Hz EKF. 50 Hz doubled the work in this driver, the optical-flow gyro subscription and the EKF for no gain. |
| `frame_id` | `imu_link` | URDF frame (mounting rotation in `robot.yaml`). |
| `gyro_calibration_samples` | 100 | Bias measured over the first 4 s after start and after every dock wake-up. **Keep the robot still** during this time. |
| `angular_velocity_variance` | 1e-4 | Covariance reported to the EKF. |
| `linear_acceleration_variance` | 0.04 | Covariance of the (unused) acceleration. |
| `mag_sensor_type` | 6 | WITMotion magnetometer type from register 0x72; selects the raw-to-tesla conversion. |
| `magnetic_field_variance` | 2.5e-11 | Magnetometer covariance. |

## `bagheera_compass` (optional, off by default)

Applies a hard-/soft-iron calibration, tilt compensation and plausibility
gates to the WT901 magnetometer and publishes `/imu/compass`,
`/imu/compass/heading` and `/imu/compass/valid`. It is **not fused** by the
EKF in the default `localization.yaml`: indoor steel and motor currents
disturb the field too much. Calibration and tests: [diagnostics.md](../calibration.md#8-compass-optional).

| Parameter | Value | Meaning |
|---|---|---|
| `calibration_file` | `/bagheera_ws/maps/compass_calibration.yaml` | Written by `bagheera_compass_calibrate`, reloaded automatically. Without it, only `valid=false` is published. |
| `sensor_to_base_yaw` | −90° | Same mounting rotation as `imu_yaw`. |
| `output_frame` | `base_link` | Frame of the published orientation. |
| `field_tolerance_fraction` | 0.35 | Reject samples whose field strength deviates more than 35 % from the calibration. |
| `acceleration_tolerance_fraction` | 0.30 | Reject samples during acceleration (tilt compensation invalid). |
| `heading_variance` | 0.12 | Reported yaw variance. |
| `filter_gain` | 0.12 | Low-pass gain of the heading filter. |
| `max_heading_rate_rad_s` | 1.5 | Reject heading jumps faster than this. |
| `imu_max_age` | 0.25 s | Maximum age of the matching IMU sample. |

## `bagheera_measurement_normalizer`

C++ component (`bagheera_sensors::MeasurementNormalizerNode`) in
`sensor_container`. Turns MowgliNext's raw messages into what standard ROS consumers expect.

| Parameter | Value | Meaning |
|---|---|---|
| `odom_frame_id`, `base_frame_id` | `odom`, `base_link` | Frames written into `/wheel_odom`. |
| `axle_to_base_link_m` | 0.0 | Shift of the wheel twist from the axle to `base_link` (0 because they coincide). |
| `imu_enabled` | false | Republish the mainboard IMU as `/imu/data`. Off: the EKF does not use it. |
| `imu_frame_id` | `imu_link` | Frame for `/imu/data` when enabled. |
| `camera_frame_id`, `camera_width`, `camera_height` | `camera_optical_frame`, 1920, 1080 | Fallback `CameraInfo` for visualization. |
| `camera_nominal_fx/fy/cx/cy` | 960, 960, 960, 540 | Provisional pinhole model used only for Foxglove. The real calibration is `camera_fisheye.yaml`. |
| `wheel_pose_variance`, `wheel_twist_variance` | see file | Covariances for `/wheel_odom` (x, y, z, roll, pitch, yaw). Unobserved axes are set huge instead of zero. |
| `imu_orientation_variance`, `imu_angular_velocity_variance`, `imu_linear_acceleration_variance` | see file | Covariances for `/imu/data`. |

## `camera`: camera_ros

The camera process is started on demand by `bagheera_camera_manager`: while a
Foxglove client subscribes `/camera/h264`, while docking needs vision, or
while `/camera/stream_enabled` is true. It stops 20 s after the last viewer
leaves and never starts while the robot sleeps in the dock. The node is a
patched `camera_ros` (see `docker/*.patch`).

| Parameter | Value | Meaning |
|---|---|---|
| `camera` | 0 | libcamera camera index. |
| `role` | `viewfinder` | libcamera stream role. |
| `format` | `NV21` | Only the luminance plane is used: `/camera/image_raw` is `mono8`, and JPEG is encoded straight from it. |
| `width`, `height` | 1920, 1080 | Output size. `camera_fisheye.yaml` is calibrated for this size. |
| `sensor_mode` | `2592:1944` | Full sensor area, so the fisheye field of view is kept. |
| `FrameDurationLimits` | 66666 µs | 15 FPS, the limit of the full-area mode. |
| `orientation` | 180 | The camera is mounted upside down. |
| `jpeg_quality`, `jpeg_frame_divisor` | 20, 2 | Grayscale JPEG on `/camera/image_raw/compressed` at 7.5 FPS for AprilTag detection only (~70 % of a core while subscribed). Foxglove cannot subscribe it (hidden by `foxglove_bridge`). |
| `h264_device` | `/dev/video11` | Raspberry Pi 4 hardware H.264 encoder. |
| `h264_bitrate`, `h264_intra_period`, `h264_framerate` | 2 Mbit/s, 15, 15 | Colour H.264 for Foxglove on `/camera/h264` (~17 % of a core); one keyframe per second so new viewers start quickly. |
| `camera_info_url` | `file://…/camera_fisheye.yaml` | Calibration. |
| `frame_id` | `camera_optical_frame` | URDF frame. |
| `use_node_time` | false | Use the sensor timestamp (needed so docking can transform detections at exposure time). |
| `AeMeteringMode`, `ExposureValue` | 1, 1.0 | Centre-weighted metering one stop brighter, so sunlight near the dock does not darken the tags. |
| `AwbEnable`, `AwbMode` | true, 0 | Automatic white balance. |

## `camera_fisheye.yaml`

A standard ROS `CameraInfo` calibration file (`equidistant` fisheye model,
1920 × 1080) for the OV5647 fisheye module. `bagheera_dock_tag_pose` uses it
to undistort the AprilTag corners. Create your own with
`tools/calibrate_fisheye.py` ([calibration.md](../calibration.md#7-camera)):
another lens or another `sensor_mode` invalidates it.
