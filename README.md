# BagheeraROS

ROS 2 overlay for the Bagheera office robot. MowgliNext v1.1.0 provides the
STM32 wire protocol, hardware bridge and velocity multiplexer. This repository
provides Bagheera's own robot model, measurement normalization, local sensor
fusion and teleoperation, and will later add indoor LiDAR navigation and
AprilTag docking.

The current platform bringup deliberately starts no GNSS, Nav2, coverage or
mower behavior. LiDAR, optical flow, the front camera and online 2D SLAM are
part of the base bringup. Nav2 autonomous navigation remains the next layer.

## Architecture

```text
game controller -> /cmd_vel_teleop -> twist_mux -> /cmd_vel
                                                  |
                                      mowgli_hardware bridge
                                        |                    |
                                  wheel odom raw        USB protocol
                                        |                    |
                                  measurement normalizer     |
                                        |                    |
                                   wheel odom                 |
                                        \                    |
WT901 over Raspberry Pi I2C -> /imu/wt901/data_raw            |
                                         \                   |
                                      robot_localization      |
                                              |               |
                                      /odometry/filtered      |
                                              |               |
                                     slam_toolbox + /scan     |
                                              |               |
                                      /map + map -> odom      |
                                                              |
                                      COBS + CRC16 over USB CDC
                                                  |
                                   MowgliNext STM32 firmware v1.1.0
```

The STM32 runtime connection is `/dev/mowgli` at 115200 baud. The ST-Link is
only needed to flash or debug firmware.

## Raspberry Pi requirements

- Ubuntu Server 24.04 LTS, ARM64
- Docker Engine with Compose v2
- MowgliNext firmware from release v1.1.0, protocol version 6
- stable `/dev/mowgli` symlink
- controller visible below `/dev/input`

Install the udev rule if `/dev/mowgli` does not already exist:

```bash
sudo install -m 0644 config/99-bagheera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Build and start

The image is derived from the released multi-architecture MowgliNext ROS 2
image. ROS itself is not installed on the Ubuntu host.

```bash
git clone https://github.com/Pauli1Go/BagheeraROS.git
cd BagheeraROS
docker compose build
docker compose up
```

The launch starts:

- Bagheera `robot_state_publisher` model and complete static sensor TF tree
- MowgliNext `hardware_bridge_node`
- MowgliNext `twist_mux`
- `bagheera_manual_mode`
- `bagheera_controller`
- measurement normalization (`/wheel_odom`, `/imu/data`, camera metadata)
- `robot_localization` EKF (`/odometry/filtered`, `odom -> base_link`)
- asynchronous `slam_toolbox` mapping (`/map`, `map -> odom`)
- YDLidar G2 driver (`/scan`)
- PMW3901 optical-flow driver (`/optical_flow/raw`, `/optical_flow/twist`)
- WT901 I2C IMU driver (`/imu/wt901/data_raw`)
- front camera driver (`/camera/image_raw`, `/camera/camera_info`)
- Foxglove bridge on `ws://bagheera.local:8765`

In Foxglove, add a 3D panel, select `map` as the fixed frame and enable
`/map`, `/scan`, the robot model and `/odometry/filtered`. The occupancy map
updates every two seconds while the robot is driven manually.

Hold controller button 4 while driving. The default mapping is axis 3 for
forward/reverse and axis 2 for steering, so the right stick controls both.
For manual mapping, the current limits are 0.16 m/s linear and 0.50 rad/s
angular. These conservative values reduce motion and scan distortion between
successive 9.6 Hz LiDAR scans.

Keep both drive wheels clear of the floor for the first test. Releasing the
deadman button sends zero velocity; loss of commands is additionally bounded
by the MowgliNext firmware watchdog.

Run in the background after validation:

```bash
docker compose up -d
docker compose logs -f bagheera-base
```

Stop explicitly with:

```bash
docker compose down
```

## Checks

Open a shell in the running container:

```bash
docker exec -it bagheera-base bash
```

Then inspect the firmware link and telemetry:

```bash
ros2 topic echo /hardware_bridge/status --once
ros2 topic echo /hardware_bridge/emergency --once
ros2 topic echo /wheel_odom --once
ros2 topic echo /imu/wt901/data_raw --once
ros2 topic echo /optical_flow/twist --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo /camera/camera_info --once
ros2 topic echo /map --once
ros2 topic hz /cmd_vel_teleop
```

The EKF fuses encoder forward velocity, PMW3901 planar ground velocity and the
WT901's bias-corrected Z angular velocity. Wheel-derived yaw is deliberately
excluded so wheel slip cannot override the gyro. The WT901 node publishes SI
units and explicitly marks orientation as unavailable; magnetometer yaw and
gravity-contaminated acceleration are not fused. It keeps running with the
available inputs if one source is absent. Sensor dimensions, frames and
conservative non-zero covariances are populated for standard ROS 2 consumers.

Robot dimensions and all sensor poses are centralized in
`src/bagheera_base/config/robot.yaml`. The PMW height is measured; the current
LiDAR, camera and IMU X/Y offsets are usable provisional defaults and should be
replaced with measured values before precision mapping.

## Save a map

The host directory `maps/` is mounted writable at `/bagheera_ws/maps` inside
the container. Save both the ordinary occupancy map and the serialized SLAM
pose graph after completing a mapping run:

```bash
docker exec -it bagheera-base bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /bagheera_ws/maps/office}}"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: /bagheera_ws/maps/office}"
```

This produces the map files in the repository's `maps/` directory on the Pi.
The serialized pose graph is what `slam_toolbox` later uses to continue mapping
or run in localization mode; the YAML/PGM pair is useful for Nav2 map tooling.

The camera, LiDAR and optical-flow wiring checks, detected hardware revisions
and reproducible standalone probes are documented in
[`docs/hardware-bringup.md`](docs/hardware-bringup.md).

The old Python `base_driver.py` and its `MW` protocol implementation remain in
the repository only as migration reference. `manual_control.launch.py` never
starts them; the only process opening `/dev/mowgli` is MowgliNext's C++ bridge.

Pure-Python tests can also run on a development machine without ROS installed:

```bash
PYTHONPATH=src/bagheera_base python3.11 -m unittest discover \
  -s src/bagheera_base/test -v
```

## Next milestones

1. Measure the remaining LiDAR, camera and IMU offsets.
2. Drive a slow closed loop, verify the WT901 yaw sign and SLAM loop closure,
   then save the first office map.
3. Add Nav2 using the saved map and the existing `/odometry/filtered` output.
4. Add camera-based AprilTag docking and charging verification.
