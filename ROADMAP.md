# Roadmap

## Milestone 1: mobile ROS 2 base (implemented)

- Mowgli USB protocol driver with reconnect and protocol/feature validation
- stamped `/cmd_vel` input and differential wheel-speed conversion
- two-stage drive enable plus host and STM32 command timeouts
- encoder odometry and `odom -> base_link` transform
- battery, IMU, magnetometer, temperature, panel, and diagnostics output
- controller PoC with deadman button and no blade support
- pure protocol/kinematics tests and ROS/PTY integration test

Hardware validation with the real STM32 and controller remains necessary. The
software cannot prove wheel polarity, encoder scale, USB identity, controller
button numbering, or the physical emergency inputs without the robot.

## Milestone 2: robot model and base calibration

- migrate and simplify the useful URDF/Xacro geometry from `MowgliRover`
- publish static sensor transforms
- measure wheel track and encoder scale, then add odometry covariance
- add a twist multiplexer before more than one velocity source is enabled
- restore or replace the STM32 periodic safety controller so the unsafe test
  override is no longer required

## Milestone 3: office navigation

- add the selected 2D lidar driver
- configure `ros2_control`/Nav2 only after the direct base driver is stable
- add SLAM or a maintained office map, localization, footprint, costmaps, and
  velocity/acceleration limits
- add bump/cliff/proximity inputs according to the final hardware safety design

## Milestone 4: docking

- add camera driver, calibration, and fixed camera transform
- detect an AprilTag at the dock and publish its pose
- implement a docking action/state machine with search, approach, final align,
  charge verification, timeout, and abort behavior

Maintenance-only protocol commands (`CONFIG_GET`, `CONFIG_SET`, panel LED, and
reboot) are intentionally not ROS services in milestone 1. They should be added
through typed interfaces or a separate diagnostic CLI, not overloaded onto the
drive path. `SET_BLADE_ENABLE` will not be exposed because Bagheera has no blade
motor.
