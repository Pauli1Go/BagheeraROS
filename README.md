# BagheeraROS

ROS 2 Jazzy host workspace for the Bagheera office robot. The STM32 runs the
ROS-independent Mowgli USB protocol; this workspace translates it into standard
ROS 2 topics and provides a game-controller proof of concept.

## Current scope

- USB CDC driver for the Mowgli protocol
- differential-drive conversion from `/cmd_vel`
- encoder odometry on `/odom` and `odom -> base_link` TF
- battery, external IMU, onboard accelerometer, panel, and diagnostics topics
- controller teleoperation with a deadman button
- safe stop on stale commands, disconnect, shutdown, or emergency telemetry

Lidar, Nav2, robot description, external sensors, and AprilTag docking are
deliberately postponed until the base can be driven and observed reliably.

## Requirements

The target platform is Ubuntu 24.04 with ROS 2 Jazzy. Install ROS 2 first, then:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-serial python3-pygame
cd ~/BagheeraROS
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Install `config/99-bagheera.rules` on the robot computer to get a stable
`/dev/mowgli` device name:

```bash
sudo install -m 0644 config/99-bagheera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Manual-control PoC

First keep the drive wheels clear of the floor. The currently built Mowgli
firmware advertises that its periodic safety controller is disabled, so motion
requires an explicit test override:

```bash
ros2 launch bagheera_base manual_control.launch.py \
  serial_port:=/dev/mowgli \
  allow_unsafe_firmware:=true
```

Hold controller button 4 (usually the left shoulder button) while driving:

- axis 3: forward/reverse
- axis 0: steering
- release the deadman button: publish zero once, then the base driver stops and
  disables the drive latch after its host timeout

Controller layouts differ. Override the parameters when necessary:

```bash
ros2 launch bagheera_base manual_control.launch.py \
  allow_unsafe_firmware:=true joystick_index:=0 deadman_button:=4 \
  throttle_axis:=3 steering_axis:=0
```

The blade motor is not addressed anywhere in this workspace.

## ROS interfaces

| Interface | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | subscribed |
| `/odom` | `nav_msgs/msg/Odometry` | published |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | published |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | published |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | published |
| `/imu/mag_raw` | `sensor_msgs/msg/MagneticField` | published |
| `/imu_onboard/data_raw` | `sensor_msgs/msg/Imu` | published |
| `/imu_onboard/temperature` | `sensor_msgs/msg/Temperature` | published |
| `/bagheera/panel_buttons` | `std_msgs/msg/UInt16MultiArray` | published |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | published |
| `/bagheera/stop` | `std_srvs/srv/Trigger` | service |

The base driver is intentionally the only process that opens the STM32 serial
device. Teleoperation and future Nav2 integration both use `/cmd_vel`.

## Development checks

Pure protocol and kinematics tests run without ROS:

```bash
python3.11 -m unittest discover -s src/bagheera_base/test -v
```

A full ROS build is done with `colcon build` on Jazzy (or in the provided
development container command documented in `docs/development.md`).
