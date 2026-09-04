# Development

The workspace targets ROS 2 Jazzy on Ubuntu 24.04. On a non-ROS development
machine, run the pure Python tests directly. For a reproducible complete build
with Docker:

```bash
docker run --rm -v "$PWD:/ws" -w /ws ros:jazzy-ros-base \
  bash -lc 'apt-get update && apt-get install -y python3-colcon-common-extensions python3-serial python3-pygame ros-jazzy-tf2-ros-py && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && colcon test && colcon test-result --verbose'
```

Hardware smoke-test order:

1. Start only `bagheera_base_driver` without the unsafe override. Confirm that
   `/battery_state`, `/diagnostics`, and available IMU topics update while drive
   remains blocked.
2. Put the robot on blocks and enable `allow_unsafe_firmware` explicitly.
3. Call `/bagheera/stop` and verify both wheels remain stopped.
4. Start controller teleoperation, hold the deadman button briefly, and verify
   wheel directions at low `max_linear_speed` and `max_angular_speed`.
5. Release the deadman, unplug USB, and stop the teleop process separately;
   every case must stop motion.
6. Only after direction and stop behavior are correct, test on the floor at low
   speed and compare `/odom` against measured travel and rotation.
