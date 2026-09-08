# BagheeraROS

ROS 2 overlay for the Bagheera office robot. MowgliNext v1.1.0 provides the
STM32 wire protocol, hardware bridge, robot model and velocity multiplexer;
this repository contains Bagheera-specific teleoperation and will later add
indoor LiDAR navigation and AprilTag docking.

The current platform bringup deliberately starts no GNSS, Nav2, coverage or
mower behavior. LiDAR, optical flow and the front camera are part of the base
bringup so their ROS interfaces are available before autonomous navigation is
enabled.

## Architecture

```text
game controller -> /cmd_vel_teleop -> twist_mux -> /cmd_vel
                                                  |
                                      mowgli_hardware bridge
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

- MowgliNext `robot_state_publisher`
- MowgliNext `hardware_bridge_node`
- MowgliNext `twist_mux`
- `bagheera_manual_mode`
- `bagheera_controller`
- YDLidar G2 driver (`/scan`)
- PMW3901 optical-flow driver (`/optical_flow/raw`, `/optical_flow/twist`)
- front camera driver (`/camera/image_raw`, `/camera/camera_info`)

Hold controller button 4 while driving. The default mapping is axis 3 for
forward/reverse and axis 2 for steering, so the right stick controls both.
The current limits are 0.50 m/s linear and 1.50 rad/s angular.

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
ros2 topic echo /imu/data --once
ros2 topic hz /cmd_vel_teleop
```

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

1. Validate USB reconnect, emergency inputs, controller deadman, wheel polarity,
   encoder direction and odometry scale on the real base.
2. Move to a long-supported ROS 2 base while keeping the MowgliNext protocol.
3. Add the selected 2D LiDAR, `slam_toolbox`/AMCL and Nav2 for indoor use.
4. Add camera-based AprilTag docking and charging verification.
