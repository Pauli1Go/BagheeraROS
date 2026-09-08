# Roadmap

## Milestone 1: MowgliNext-backed manual base

- pin MowgliNext release v1.1.0 and protocol-v6 firmware
- reuse `mowgli_hardware`, `mowgli_interfaces`, robot model and `twist_mux`
- publish an explicit manual high-level mode without sending blade commands
- drive through `/cmd_vel_teleop` with deadman and conservative limits
- validate firmware handshake, emergency, USB reconnect, wheel direction,
  encoders, IMU and battery on real hardware

## Milestone 2: stable Bagheera platform

- calibrate wheel track, encoder scale and drive/yaw controllers
- remove the retired custom `MW` driver after hardware migration is accepted
- migrate the deployment from short-lived ROS 2 Kilted to a supported LTS
- add service supervision and boot-time health reporting

## Milestone 3: office navigation

- integrate the YDLidar G2, PMW3901 and front fisheye camera
- create a Bagheera URDF without mower-specific tool geometry
- fuse wheel odometry, IMU yaw rate and optical-flow velocity
- add `slam_toolbox` for mapping and AMCL for operation on a maintained map
- configure Nav2 footprint, costmaps and indoor velocity/acceleration limits
- integrate additional bump, cliff and proximity safety inputs

## Milestone 4: docking

- add camera driver, calibration and fixed transform
- detect an AprilTag at the dock and publish a stable dock pose
- integrate an `opennav_docking` plugin or equivalent docking action
- verify final alignment, charging detection, timeout and safe abort behavior
