# `robot.yaml`: geometry and hardware bridge

`robot.yaml` has two sections:

- `bagheera`: robot geometry and sensor poses. `manual_control.launch.py`
  reads it and passes every value as a xacro argument to
  `urdf/bagheera.urdf.xacro`. `robot_state_publisher` then publishes the
  static TF tree. The Nav2 footprint is **not** derived from it; see
  [navigation.md](navigation.md#footprint).
- `hardware_bridge`: overrides for the
  [MowgliNext](https://github.com/mowglinext/mowglinext) `hardware_bridge_node`.
  The launch file loads MowgliNext's own `hardware_bridge.yaml` first and this
  section on top, so every value here wins. Many values are pushed to the
  STM32 at connect time. The firmware clamps them to its compiled limits: the
  wire can only make protection stronger, never weaker.

All poses are relative to `base_link`, the midpoint of the drive axle
(REP-103: x forward, y left, z up).

## `bagheera`: geometry

| Parameter | Value | Meaning |
|---|---|---|
| `chassis_length`, `chassis_width`, `chassis_height` | 0.51, 0.34, 0.19 m | Box used for the URDF visual/collision model. |
| `chassis_center_x` | 0.1784 m | Box centre ahead of `base_link`. With the length this gives the measured outline: 0.4334 m ahead of and 0.0766 m behind the axle. |
| `chassis_mass_kg` | 8.76 | URDF inertia only. |
| `wheel_radius` | 0.04475 m | Drive wheel radius. Also sets the height of `base_link` above the floor. |
| `wheel_width` | 0.04 m | URDF visual only. |
| `wheel_track` | 0.32 m | Distance between the wheel centres. Keep it identical to `hardware_bridge.wheel_track`. |
| `axle_x` | 0.0 | Axle position relative to `base_link`; 0 because `base_link` is the axle. |
| `caster_radius`, `caster_track` | 0.03, 0.36 m | URDF visual only. |
| `imu_x/y/z`, `imu_roll/pitch/yaw` | see file | WT901 pose (`imu_link`). The WT901's Y+ points forward, hence yaw −90°. |
| `lidar_x/y/z`, `lidar_yaw` | see file | LiDAR pose (`lidar_link`). `lidar_yaw` maps the scan's zero direction to the robot's front; calibrated with a target and a mapped 360° turn (−84.1°). |
| `optical_flow_x/y/z` | see file | PMW3901 pose (`pmw3901_sensor`). z = mounting height 0.09 m minus the wheel radius. |
| `camera_x/y/z`, `camera_roll/pitch/yaw` | see file | Front camera pose (`camera_link`; the optical frame is rotated from it in the URDF). |

On Bagheera the WT901, PMW3901 and LiDAR share one sensor cluster 0.1834 m
ahead of and 0.011 m left of the axle, so their x/y values are identical. How
to measure these values: [calibration.md](../calibration.md).

## `hardware_bridge`: MowgliNext overrides

### Link

| Parameter | Value | Meaning |
|---|---|---|
| `serial_port` | `/dev/mowgli` | Replaced by the `serial_port` launch argument. |
| `baud_rate` | 115200 | USB CDC baud rate. |
| `heartbeat_rate` | 4.0 Hz | Keep-alive to the firmware. Without it the firmware watchdog stops the motors. |
| `serial_rx_timeout_s` | 2.0 s | Reopen the port if no bytes arrive (board reset, reflash). |
| `publish_rate` | 100 Hz | Bridge read tick. |
| `high_level_rate` | 2.0 Hz | Rate at which the high-level mode (from `bagheera_mode`) is sent to the firmware. |

### Kinematics and drive control

| Parameter | Value | Meaning |
|---|---|---|
| `wheel_track` | 0.32 m | Wheel distance used for odometry and sent to the STM32. |
| `ticks_per_meter` | 268.1 | Encoder scale for odometry and for the firmware's wheel-speed loop. **Calibrate per robot** ([calibration.md](../calibration.md#2-wheel-odometry)). |
| `max_mps` | 0.5 m/s | Wheel-speed cap sent to the firmware (the firmware can only lower its own limit). |
| `min_linear_vel` | 0.05 m/s | Forward commands with a magnitude below this are set to 0 (motor deadband). |
| `wheel_pid_kp/ki/kd` | 80 / 400 / 0.01 | Per-wheel velocity PID in the firmware (PWM per m/s, per m/s·s, per m/s²). MowgliNext's pre-calibration defaults 0.2 / 0.092 leave the loop open: at 4–7 cm/s the wheels reached only 40–60 % of the commanded speed. |
| `wheel_pid_integral_limit` | 40.0 | Anti-windup limit of the integral term (PWM). |
| `wheel_pid_pwm_per_mps` | 282.135 | Feed-forward PWM per m/s. |
| `yaw_loop_enabled` | true | Firmware yaw-rate loop on the mainboard gyro. It trims the wheel speeds so the robot turns at the commanded rate. |
| `yaw_kp`, `yaw_ki` | 0.30, 0.0 | Yaw-loop gains. Bagheera uses P only: an integral term caused a 5–6 s weave on its floor. |
| `yaw_trim_limit_mps` | 0.15 m/s | Maximum wheel-speed trim of the yaw loop. |
| `yaw_gyro_sign` | 1 | Sign of the mainboard gyro Z relative to the robot. Flip it if the yaw loop makes turns worse. |

### Mainboard IMU calibration

These concern the IMU on the mainboard (`/imu/data_raw`), which Bagheera's
EKF does not use. Its yaw loop above does.

| Parameter | Value | Meaning |
|---|---|---|
| `imu_cal_samples` | 200 | Samples averaged for the gyro/accel offset (~4 s). |
| `imu_cal_persist_path` | `/ros2_ws/maps/imu_calibration.txt` | Where the bridge stores the last calibration inside the container. |
| `imu_cal_auto_rest_sec` | 15 s | Calibrate automatically after this long at rest if no calibration exists. |
| `imu_cal_periodic_recal_sec` | 600 s | Recalibrate this often while docked and stationary. |

### Charging and emergency inputs (sent to the firmware)

| Parameter | Value | Meaning |
|---|---|---|
| `max_charge_voltage` | 29.4 V | Charge cut-off as measured by the board. Bagheera's board reads 0.8 V high, so this is about 28.6 V real (see [base.md](base.md#battery-voltage-offset)). |
| `max_charge_current` | 1.2 A | Charge current limit. |
| `one_wheel_lift_emergency_ms` | 2000 | Emergency after one wheel-lift switch is active this long. |
| `both_wheels_lift_emergency_ms` | 1000 | Same for both wheels. |
| `tilt_emergency_ms` | 500 | Emergency after tilt is detected this long. |
| `stop_button_emergency_ms` | 100 | Stop-button debounce. |
| `play_button_clear_emergency_ms` | 2000 | Holding the play button this long clears a latched emergency. |

### Mower features that stay inactive on Bagheera

| Parameter | Value | Why it has no effect here |
|---|---|---|
| `dock_pose_x/y/yaw` | 0.0 | MowgliNext's GNSS dock pose. Bagheera's dock pose lives in `nav2_navigation.yaml` ([docking.md](docking.md#dock-pose)). |
| `lift_recovery_mode`, `lift_blade_resume_delay_sec` | false, 1.0 | Blade handling after a lift. Bagheera never enables a blade. |
| `dig_*` | see file | Wheel-slip "dig" detection. It compares wheel travel with `/odometry/filtered_map`, which Bagheera does not publish, so it stands down after `dig_pose_timeout_s`. |
