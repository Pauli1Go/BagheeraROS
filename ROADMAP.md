# Roadmap

## Done

- Manual base on MowgliNext v1.1.0 (firmware 1.9.10, protocol 6): hardware
  bridge, `twist_mux`, explicit manual mode without blade commands, game
  controller teleop with deadman.
- Robot model, wheel-odometry and sensor calibration; WT901, PMW3901,
  YDLidar G2 and fisheye camera drivers.
- EKF fusion of wheel speed, optical flow and gyro yaw rate.
- Mapping with slam_toolbox, localization with AMCL, pose persistence,
  keepout and localization-exclusion masks.
- Nav2 point-to-point navigation with wait/replan recoveries.
- AprilTag docking with Nav2 `opennav_docking` and a custom dock plugin;
  automatic undocking.
- Dock sleep and wake-up sequence, voltage-based battery monitor, waypoint
  patrol.
- 360° heading test with a LiDAR reference: verdict whether the heading
  drift is acceptable and which sensor causes it.

## Next

- Return to the dock automatically on `LOW` battery outside the patrol.
- Reduce CPU load on the Raspberry Pi 4 while driving.
- Bump, cliff or proximity inputs as an additional safety layer.
- Move from ROS 2 Kilted to the next LTS release together with MowgliNext.
