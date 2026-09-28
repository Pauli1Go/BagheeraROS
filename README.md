# BagheeraROS

ROS 2 software for **Bagheera**, an indoor robot built around a YardForce
robot-mower mainboard. It adds indoor LiDAR mapping and navigation,
AprilTag docking and battery-aware patrols on top of the mower's own
electronics.

BagheeraROS is built on **[MowgliNext](https://github.com/mowglinext/mowglinext)**.
MowgliNext's STM32 firmware runs on the mainboard, and its C++
`hardware_bridge_node`, `twist_mux` and ROS 2 Docker image are the base this
project extends. BagheeraROS adds the robot model, sensor drivers, sensor
fusion, Nav2 configuration, docking and operating logic for an indoor robot.
It never starts the blade, GNSS or mowing behavior.

| Tested with | Version |
|---|---|
| MowgliNext release | [v1.1.0](https://github.com/mowglinext/mowglinext/releases/tag/v1.1.0) |
| Mainboard firmware | MowgliNext 1.9.10, protocol 6 |
| ROS 2 | Kilted (inside the MowgliNext image) |
| Host | Raspberry Pi 4, Ubuntu Server 24.04 ARM64, Docker Compose |

## Features

- **Teleop** with any Linux gamepad (deadman button, conservative limits;
  enable it in `base.yaml`, off by default to save CPU).
- **Sensor fusion**: wheel encoders, PMW3901 optical flow (catches wheel
  slip) and WT901 gyro in a robot_localization EKF.
- **Mapping** with slam_toolbox, **localization** with AMCL on a saved map,
  pose restored across restarts.
- **Navigation** with Nav2: NavFn, rotation shim + regulated pure pursuit,
  keepout zones, "impossible pose" exclusion zones, wait/replan recoveries
  without spinning.
- **Docking** with Nav2 `opennav_docking` and a custom plugin that uses two
  AprilTags and a fisheye camera, a straight final approach with gyro heading
  hold, and automatic undocking before the next goal.
- **Dock sleep**: LiDAR, IMU, optical flow, camera and Nav2 sleep while
  charging; a wake-up re-anchors the pose before driving.
- **Battery monitor** for a 7S Li-Ion pack (voltage-based, measured offset
  correction) and a **waypoint patrol** that docks when the battery is low.
- **Foxglove** as the user interface: map, goals, camera (hardware H.264),
  status topics.
- Calibration and diagnostic tools (360° heading test, rotation-shift test,
  straight-drive test, AMCL and scan probes).

## Architecture

```text
Foxglove ──WebSocket──► Raspberry Pi 4 / Docker (ROS 2 Kilted)
                        ├─ MowgliNext hardware_bridge + twist_mux
                        ├─ bagheera_base: drivers, EKF, AMCL, Nav2, docking, patrol
                        ├─ bagheera_sensors: C++ optical-flow/IMU drivers
                        └─ bagheera_docking: AprilTag dock plugin
                               │ USB /dev/mowgli
                        YardForce mainboard, MowgliNext firmware 1.9.10
```

Details: [docs/architecture.md](docs/architecture.md).

## Quick start

```bash
git clone https://github.com/Pauli1Go/BagheeraROS.git
cd BagheeraROS
sudo install -m 0644 config/99-bagheera.rules /etc/udev/rules.d/ && sudo udevadm trigger
docker compose build
```

A fresh checkout contains **no map** and robot-specific values from
Bagheera. Follow the guides in order:

1. [Installation](docs/installation.md): hardware, MowgliNext firmware, host,
   first start with the wheels in the air.
2. [Calibration](docs/calibration.md): geometry, wheel odometry, IMU, LiDAR,
   optical flow, battery offset, camera.
3. [Mapping](docs/mapping.md): create and save a map, keepout and exclusion
   masks.
4. [Docking](docs/docking.md): tags, dock pose, staging pose, tag offsets.
5. [Operation](docs/operation.md): Foxglove, goals, docking, dock sleep,
   battery, patrol.

Reference:

- [Configuration reference](docs/config/README.md): every parameter of every
  config file, what to adapt, launch arguments.
- [Diagnostics](docs/diagnostics.md): test tools and scripts.
- [Hardware](docs/hardware.md): wiring and standalone probes.
- [Development](docs/development.md): repository layout, applying changes,
  tests.
- [Roadmap](ROADMAP.md).

## Safety

This software drives real motors. Test new setups with the wheels off the
floor, keep the physical stop button within reach, and do not leave the
robot unattended while it drives autonomously. Test tools with `--execute`
override the game controller and bypass obstacle handling. The MowgliNext
firmware's command watchdog and emergency inputs remain the final safety
layer. If you reuse a mower mainboard, disconnect the blade motor.

## License

BagheeraROS is licensed under the [Apache License 2.0](LICENSE).

It builds on MowgliNext, which has its own license (GPLv3 for
non-commercial use, commercial license on request; see the
[MowgliNext repository](https://github.com/mowglinext/mowglinext)). The
Docker image built from this repository contains MowgliNext software and is
subject to MowgliNext's terms. The patched third-party drivers keep their
upstream licenses.

## Acknowledgements

- [MowgliNext](https://github.com/mowglinext/mowglinext): firmware, hardware
  bridge and ROS 2 base image.
- [Mowgli](https://github.com/cloudn1ne/Mowgli): reverse engineering of the
  YardForce mainboard.
- [Nav2](https://nav2.org), [robot_localization](https://github.com/cra-ros-pkg/robot_localization),
  [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox),
  [apriltag_ros](https://github.com/christianrauch/apriltag_ros),
  [camera_ros](https://github.com/christianrauch/camera_ros),
  [YDLidar](https://github.com/YDLIDAR), [Foxglove](https://foxglove.dev).
