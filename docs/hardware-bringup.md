# Sensor bring-up

This page records the first hardware identification tests on the Bagheera
Raspberry Pi 4. The tests deliberately run independently of ROS so that USB,
SPI and wiring can be checked before adding either sensor to the main launch.

## Front fisheye camera

The CSI camera is detected as an OmniVision OV5647 and registered through the
Raspberry Pi Unicam and ISP devices. A successful hardware probe looks like:

```text
Available cameras:
1: 'ov5647' (/base/soc/i2c0mux/i2c@1/ov5647@36)
```

Ubuntu Noble's system `libcamera` 0.2 detects this camera but aborts on the
first frame with a `prepareIsp()` buffer assertion against the current Pi
kernel. Bagheera therefore uses `ros-kilted-camera-ros`, which brings the newer
ROS-packaged libcamera runtime into the container. The container also mounts
`/run/udev` read-only so libcamera can enumerate the media graph.

The ROS node publishes a 1920 x 1080 OpenCV-compatible `bgr8` view on
`/camera/image_raw`, rotated by 180 degrees to match the physical installation.
It forces the 2592 x 1944 sensor mode before scaling instead of selecting a
low-resolution sensor crop. Automatic white balance is explicitly enabled.

Without a calibration file, `camera_ros` reports a `CameraInfo` size of 0 x 0,
which Foxglove rejects as `invalid image size 0x0`. The measurement normalizer
therefore publishes the known stream dimensions and `camera_optical_frame` on
`/camera/camera_info`. Foxglove also rejects zero focal lengths, so the default
configuration supplies a provisional pinhole model with `fx=fy=960` and the
principal point at the 1920 x 1080 image centre. This is suitable only for
visualization. A real fisheye calibration must replace it before AprilTag pose,
bearing or distance estimation.

## YDLIDAR G2B

The LiDAR is connected through a Silicon Labs CP2102 USB-to-UART adapter. On
the tested robot it appears as:

```text
/dev/ttyUSB0
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Use the `by-id` path for deployment. Unlike `ttyUSB0`, it does not depend on
USB enumeration order.

The sensor was identified and scanned with the official YDLIDAR SDK:

```bash
git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/ydlidar-sdk
cmake -S /tmp/ydlidar-sdk -B /tmp/ydlidar-sdk/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/ydlidar-sdk/build -j2
/tmp/ydlidar-sdk/build/tri_test
```

Select the CP2102 port and the following values when prompted:

```text
port: /dev/ttyUSB0
baud rate: 230400
one-way communication: no
scan frequency: 10 Hz
```

Observed identification:

```text
model code: 15
model: G2B
firmware: 3.5
hardware: 3
sample rate: 5 kHz
points per revolution: approximately 500
```

The test confirmed a healthy device and continuous scan data. One checksum
warning occurred while the stream was starting; subsequent scans continued.
The ROS integration is enabled in `manual_control.launch.py` and publishes
`sensor_msgs/msg/LaserScan` on `/scan` with frame `lidar_link`. The tested
G2B delivers about 9.66 scans/s. `fixed_resolution` is deliberately disabled:
the sensor varies slightly around its nominal samples-per-revolution and a
fixed 530-point buffer would discard valid points.

The physical scan zero was calibrated using a static target centred directly
in front of the robot at about 0.44 m. Across 20 scans it formed the largest
nearby cluster, centred at +85.5 degrees with a width of 34.75 degrees.
`robot.yaml` therefore sets `lidar_yaw` to -85.5 degrees, mapping the cluster
centre to REP-103 `base_link` X+ (forward). This static correction is
independent of the `inverted` setting, which fixes the scan's
clockwise/counter-clockwise order.

## PMW3901 optical-flow sensor

The downward-facing PMW3901 uses Raspberry Pi SPI0:

| Signal | BCM GPIO | Linux function |
|---|---:|---|
| CS | 8 | SPI0 CE0 |
| SCK | 11 | SPI0 SCLK |
| MOSI | 10 | SPI0 MOSI |
| MISO | 9 | SPI0 MISO |
| INT | not connected | not required for polling |

SPI is enabled by `dtparam=spi=on` in `/boot/firmware/config.txt`. The sensor
is therefore available at `/dev/spidev0.0`. The `ubuntu` user and the ROS
container need access to the `dialout` group or the device must be passed into
the container; the current privileged Bagheera container can access it.

Build and run the non-destructive identity probe from inside an environment
with a C compiler and the Linux SPI headers:

```bash
gcc -O2 -Wall tools/pmw3901_probe.c -o /tmp/pmw3901_probe
/tmp/pmw3901_probe /dev/spidev0.0
```

Expected and observed identity:

```text
product ID:         0x49
revision:           0x00
inverse product ID: 0xb6
```

The probe only reads registers; it does not initialize motion tracking.
`bagheera_optical_flow` polls motion bursts without an interrupt pin. It
publishes sensor counts and quality on `/optical_flow/raw` and a calibrated
height-scaled `TwistWithCovarianceStamped` on `/optical_flow/twist`. The
mounting height is configured as 0.09 m.

After the final installation, a straight run at 0.16 m/s produced 2755 counts
on sensor X and -60 counts on sensor Y over a measured 0.510 m. Therefore X is
the robot's forward axis without inversion. The calibrated scale is:

```text
0.510 m / 2755 counts = 0.00018512 m/count
0.00018512 m/count / 0.09 m = 0.002057 rad/count
```

The immediately preceding repeat produced 2723 X-counts, confirming that the
new installation gives stable longitudinal flow measurements.

## WT901 IMU

The WT901 is connected directly to Raspberry Pi I2C1 and was detected at its
default 7-bit address `0x50`:

| Signal | Raspberry Pi connection | Electrical level |
|---|---|---|
| SDA | GPIO 2 / pin 3 | measured 3.3 V logic |
| SCL | GPIO 3 / pin 5 | measured 3.3 V logic |
| VCC | 5 V | module supply |
| GND | GND | common ground |

`i2cdetect -y 1` must show `50`. A live register probe returned approximately
`[-0.052, -0.021, +1.007] g` while stationary, confirming communication and
that the sensor Z axis points upward in the current mounting. The physical
WT901 Y+ axis points toward the front of the robot. `robot.yaml` therefore
defines an IMU yaw of -90 degrees so sensor Y+ maps to REP-103 `base_link` X+.

`bagheera_wt901` reads acceleration and angular velocity at 50 Hz, converts
them to m/s² and rad/s, and publishes `sensor_msgs/msg/Imu` on
`/imu/wt901/data_raw`. During its first four seconds it averages the stationary
gyro samples and then removes that bias. Keep the robot still during this
period after every bring-up restart.

The message marks orientation as unavailable. The local EKF intentionally
fuses only `angular_velocity.z`; it does not consume the WT901 magnetometer
orientation or linear acceleration. Consequently, a horizontal rotation of
the module and its roughly 1 cm displacement from the robot centre do not
affect the currently used measurement. The exact pose must still be entered in
`config/robot.yaml` before using acceleration or full 3D orientation.

Check the live integration inside the running container:

```bash
i2cdetect -y 1
ros2 topic hz /imu/wt901/data_raw
ros2 topic info --verbose /imu/wt901/data_raw
ros2 topic echo /imu/wt901/data_raw --once
```

The topic should run at about 50 Hz and list `ekf_filter_node` as a subscriber.

## Online SLAM

`slam_toolbox` consumes `/scan` and the existing `odom -> base_link ->
lidar_link` transform chain. It publishes the occupancy grid on `/map` and the
global correction as `map -> odom`. The EKF remains the sole publisher of
`odom -> base_link`; feeding the SLAM pose back into that local EKF would create
a circular dependency.

The WT901 is intentionally not connected straight to `slam_toolbox`. Its Z
angular velocity enters the EKF, the EKF integrates it into
`odom -> base_link`, and SLAM uses that transform as the motion estimate for
each LiDAR scan. A controlled turn verified that `map -> base_link` and
`odom -> base_link` rotate together while `map -> odom` remains the independent
scan-matching correction.

Wheel odometry contributes forward velocity but not angular velocity. The
WT901 is the sole EKF yaw-rate source: wheel-derived yaw becomes unreliable
during slip and reports zero when the robot is turned manually. Do not add a
low `imu0_twist_rejection_threshold`; with the WT901 covariance, the previous
1.5-sigma threshold rejected ordinary turns as outliers.

Mapping is disabled by default in `manual_control.launch.py`, so a normal boot
does not create or modify a map. Start the mapping-only launch explicitly with
`ros2 launch bagheera_base mapping.launch.py`. Parameters are in
`src/bagheera_base/config/slam.yaml`. The 5 cm map resolution matches the first
indoor proof of concept; travel thresholds are deliberately low enough to
accept the robot's slow manual movements.

## Drive calibration

Bagheera overrides the MowgliNext wheel scale in
`src/bagheera_base/config/robot.yaml`. The hardware bridge uses `ticks_per_meter` for
wheel odometry and sends the same value to the STM32 velocity controller.

The initial floor runs produced 206.0 ticks over 0.80 m at 0.08 m/s and 201.5
ticks over 0.72 m at 0.16 m/s. Their combined estimate is:

```text
(206.0 + 201.5) ticks / (0.80 + 0.72) m = 268.09 ticks/m
```

The configured starting calibration is therefore `268.1` ticks/m. Further
runs at multiple speeds should be recorded separately to expose speed-related
slip before replacing this combined estimate.

Validation with the new value used a 0.50 m encoder target at three speeds:

| Commanded speed | Encoder distance after stopping | Measured distance |
|---:|---:|---:|
| 0.08 m/s | 0.505 m | 0.47 m |
| 0.16 m/s | 0.511 m | 0.53 m |
| 0.50 m/s | 0.519 m | 0.60 m |

The 0.08 and 0.16 m/s runs are within 3 cm of the 0.50 m target. The maximum
speed shows substantially more run-on and is not intended as the normal indoor
navigation speed. Keep `268.1` ticks/m and use the lower speeds for the first
navigation milestone.
