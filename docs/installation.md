# Installation

This guide takes a robot from bare hardware to a running container. Continue
with [calibration.md](calibration.md) and [mapping.md](mapping.md)
afterwards.

> **Safety.** BagheeraROS drives real motors. Keep the drive wheels off the
> floor for the first start, keep the physical stop button in reach, and never
> run autonomy or test tools unattended. If you use a mower mainboard, make
> sure no blade motor is connected.

## 1. Hardware

| Part | Bagheera uses | Notes |
|---|---|---|
| Mainboard | YardForce GForce mower mainboard (STM32F103) | Must run MowgliNext firmware. Drives two wheel motors with encoders, measures battery and charge voltage, handles emergency inputs. |
| Computer | Raspberry Pi 4 (4 GB or more) | Other ARM64/AMD64 Linux computers work if they provide I2C, SPI and a camera. |
| LiDAR | YDLidar G2 (G2B) over a CP2102 USB-UART adapter | Other YDLidar models need driver parameter changes. |
| IMU | WIT Motion WT901 on I2C1 | Provides the yaw rate for the EKF. |
| Optical flow | PMW3901 on SPI0, 9 cm above the floor | Detects wheel slip. |
| Camera | OV5647 fisheye module (CSI) | Only needed for docking. |
| Battery | 7S Li-Ion (25.2 V nominal) | Thresholds in `base.yaml` assume 7S. |
| Controller | any gamepad supported by Linux | Needed for teleop and mapping; enable it in `base.yaml` (off by default). |
| Dock | charging contacts wired to the mainboard's charge input, two printed AprilTags | See [docking.md](docking.md). |

Wiring and probe commands: [hardware.md](hardware.md).

## 2. Mainboard firmware (MowgliNext)

BagheeraROS builds on **[MowgliNext](https://github.com/mowglinext/mowglinext)**.
MowgliNext provides the STM32 firmware, the USB protocol (COBS + CRC16,
protocol version 6), the C++ `hardware_bridge_node`, `twist_mux` and the ROS 2
image. BagheeraROS is tested with:

| Component | Version |
|---|---|
| MowgliNext release | [v1.1.0](https://github.com/mowglinext/mowglinext/releases/tag/v1.1.0) |
| Firmware | 1.9.10, protocol 6 (the `mowgli-fw_…_p6_v1.9.10_….bin` assets of that release) |
| ROS 2 image | `ghcr.io/mowglinext/mowglinext/mowgli-ros2:1.1.0` (ROS 2 Kilted) |

Flash the firmware build that matches your mainboard, following the
MowgliNext documentation. Back up the original firmware first. Firmware and
image versions must match: the bridge refuses a different protocol version.

## 3. Host operating system

1. Install **Ubuntu Server 24.04 LTS (ARM64)** on the Pi. ROS is not
   installed on the host; everything runs in Docker.
2. Enable I2C and SPI in `/boot/firmware/config.txt`:

   ```ini
   dtparam=i2c_arm=on
   dtparam=spi=on
   ```

   Make sure the camera is detected (`camera_auto_detect=1` is the Ubuntu
   default). Reboot.
3. Install Docker Engine with the Compose plugin
   ([docs.docker.com/engine/install/ubuntu](https://docs.docker.com/engine/install/ubuntu/))
   and add your user to the `docker` group.
4. Install the udev rule that creates `/dev/mowgli` for the mainboard:

   ```bash
   sudo install -m 0644 config/99-bagheera.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

5. Check the devices:

   ```bash
   ls -l /dev/mowgli /dev/serial/by-id/ /dev/spidev0.0 /dev/i2c-1 /dev/input/js0
   ```

## 4. Get and build BagheeraROS

```bash
git clone https://github.com/Pauli1Go/BagheeraROS.git
cd BagheeraROS
docker compose build
```

The first build takes a while on a Pi 4. It compiles the YDLidar SDK, the
patched YDLidar and camera drivers and the C++ dock plugin on top of the
MowgliNext image. See `docker/Dockerfile`.

Before the first start, adapt at least:

- `src/bagheera_base/config/sensors.yaml` → `ydlidar_ros2_driver_node.port`:
  your adapter's path from `ls /dev/serial/by-id/`.
- `src/bagheera_base/config/robot.yaml`: wheel radius, wheel track and
  sensor poses of your robot.

The full checklist is in [config/README.md](config/README.md#what-you-must-adapt-for-your-robot).

## 5. What the container gets

`compose.yaml` runs one container, `bagheera-base`:

- `network_mode: host`, `ipc: host`: Foxglove on port 8765 (WebSocket, works
  over WiFi).
- ROS 2 (CycloneDDS) runs on the loopback interface only
  (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, interface `lo`). A WiFi drop
  then no longer freezes the traffic between the nodes on the robot. The
  flip side: `ros2` commands from another computer do not see the robot; run
  them inside the container (`docker exec -it bagheera-base bash`).
- `privileged: true` with `/dev` and `/run/udev` mounted: access to USB, I2C,
  SPI, the camera and the controller. The container therefore has full
  hardware access to the host; run it only on the robot's own computer.
- CycloneDDS as RMW, bound to `lo`, up to 120 participants (processes).
- Bind mounts: `config/`, `launch/`, `behavior_trees/` and the Python
  package are read-only overlays over the installed copies (a restart applies
  changes). `maps/` and `test_logs/` are writable.
- `restart: unless-stopped`: the stack starts again after a reboot.

## 6. First start (without a map)

A fresh checkout has no map, so start without map localization and
navigation. **Wheels off the floor.**

```bash
docker compose run --rm --name bagheera-mapping bagheera-base \
  ros2 launch bagheera_base manual_control.launch.py \
  use_map_localization:=false use_navigation:=false
```

In a second shell, check the basics:

```bash
docker exec -it bagheera-mapping /bagheera_entrypoint.sh bash
ros2 topic echo /hardware_bridge/status --once      # firmware link, protocol v6
ros2 topic echo /hardware_bridge/emergency --once   # no active emergency
ros2 topic echo /hardware_bridge/power --once       # battery and charge voltage
ros2 topic hz /scan                                  # ~9.6 Hz
ros2 topic hz /imu/wt901/data_raw                    # ~25 Hz
ros2 topic echo /optical_flow/twist --once
ros2 topic echo /odometry/filtered --once
```

Then, still with the wheels in the air, enable the game controller
(`enabled: true` under `bagheera_controller` in `config/base.yaml`, then
`docker compose restart`; it is off by default to save CPU) and:

1. Hold the deadman button (default button 4) and push the left stick
   forward. Both wheels must turn forward and `/wheel_odom` must count up.
2. Release the deadman: the wheels must stop.
3. Trigger the physical stop button: `/hardware_bridge/emergency` must show
   it and commands must be blocked.

Only then put the robot on the floor, [calibrate](calibration.md) it and
[create a map](mapping.md). Once `maps/current.yaml` and the masks exist, the
normal start is:

```bash
docker compose up -d
docker compose logs -f bagheera-base
```

Stop with `docker compose down`.
