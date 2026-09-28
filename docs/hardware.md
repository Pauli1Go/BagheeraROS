# Hardware and wiring

How Bagheera's sensors are connected and how to check each one without ROS.
Parameters: [config/sensors.md](config/sensors.md). Values measured on
Bagheera are marked as such; your hardware may differ.

## Mainboard

A YardForce GForce mower mainboard running
[MowgliNext](https://github.com/mowglinext/mowglinext) firmware 1.9.10
(protocol 6) is connected over USB. `config/99-bagheera.rules` creates the
stable link `/dev/mowgli` from the USB product name `Mowgli`:

```text
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE:="0660", TAG+="uaccess"
```

Only MowgliNext's `hardware_bridge_node` opens this port. An ST-Link is only
needed to flash or debug the firmware.

## YDLidar G2 / G2B

Connected through a Silicon Labs CP2102 USB-UART adapter. Use the
`/dev/serial/by-id/…` path in `sensors.yaml`; unlike `/dev/ttyUSB0` it does
not depend on the USB enumeration order.

Standalone check with the official SDK (on the host):

```bash
git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/ydlidar-sdk
cmake -S /tmp/ydlidar-sdk -B /tmp/ydlidar-sdk/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/ydlidar-sdk/build -j2
/tmp/ydlidar-sdk/build/tri_test     # port, baud 230400, two-way, 10 Hz
```

Bagheera's unit: model code 15 (G2B), firmware 3.5, hardware 3, 5 kHz,
about 500 points per revolution, about 9.66 scans/s in ROS. One checksum
warning while the stream starts is harmless.

## WT901 IMU (I2C1)

| Signal | Raspberry Pi |
|---|---|
| SDA | GPIO 2 / pin 3 (3.3 V logic) |
| SCL | GPIO 3 / pin 5 (3.3 V logic) |
| VCC | 5 V |
| GND | GND |

`i2cdetect -y 1` must show `50` (default address 0x50). At rest the
acceleration should read about `[0, 0, +1] g`; if Z is −1 g, the sensor is
upside down. On Bagheera the sensor's Y+ axis points forward, hence
`imu_yaw: −90°` in `robot.yaml`. Register 0x72 (`MAGSENSOR`) reported type 6,
which selects the magnetometer conversion (`mag_sensor_type`).

## PMW3901 optical flow

| Signal | BCM GPIO | Function |
|---|---:|---|
| CS | 8 | SPI0 CE0 |
| SCK | 11 | SPI0 SCLK |
| MOSI | 10 | SPI0 MOSI |
| MISO | 9 | SPI0 MISO |
| INT | – | not needed (polled) |

Mounted facing down, 9 cm above the floor (Bagheera). With
`dtparam=spi=on` it appears as `/dev/spidev0.0`. Read-only identity probe:

```bash
gcc -O2 -Wall tools/pmw3901_probe.c -o /tmp/pmw3901_probe
/tmp/pmw3901_probe /dev/spidev0.0
```

Expected: product ID `0x49`, revision `0x00`, inverse product ID `0xb6`.

## Front camera

An OV5647 fisheye module on the CSI port, detected by libcamera as
`ov5647`. Ubuntu 24.04's system libcamera (0.2) aborts on the first frame on
current Pi kernels. The image therefore uses the newer libcamera shipped with
`ros-kilted-camera-ros`, patched by `docker/camera-*.patch`:

- only the NV21 luminance plane is used (`mono8`), and JPEG is encoded
  directly from it by a single latest-frame worker;
- hardware H.264 through `/dev/video11` for Foxglove, encoded only while
  subscribed;
- startup controls from the YAML are merged instead of discarded.

The container mounts `/run/udev` read-only so libcamera can enumerate the
media graph. The full 2592 × 1944 sensor mode is forced before scaling, so the
fisheye field of view and the calibration stay valid. The module is mounted
upside down (`orientation: 180`).

## Game controller

Any controller the Linux joystick driver exposes as `/dev/input/js*`. Check
axes and buttons with `jstest /dev/input/js0` (package `joystick`) and set
them in `base.yaml` or with the launch arguments.

## Docker patches

| Patch | Upstream | Change |
|---|---|---|
| `ydlidar-reliable-scan.patch` | YDLIDAR/ydlidar_ros2_driver | publish `/scan` with reliable QoS (keep last 5) |
| `camera-latest-frame.patch` | christianrauch/camera_ros | mono8 from NV21, JPEG from luminance by a single latest-frame worker, `jpeg_frame_divisor`, hardware H.264 on `~/h264` |
| `camera-merge-pending-controls.patch` | christianrauch/camera_ros | merge control batches set before the first request instead of dropping earlier ones |

Upstream commits are pinned in `docker/Dockerfile`.
