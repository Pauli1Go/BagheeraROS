"""ROS 2 driver for a WIT Motion WT901 connected to Raspberry Pi I2C."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String

try:
    import smbus
except ImportError as error:  # pragma: no cover - exercised by deployment
    smbus = None
    SMBUS_IMPORT_ERROR = error

from .wt901_protocol import (
    GyroBiasEstimator,
    INERTIAL_BLOCK_LENGTH,
    MAG_SENSOR_REGISTER,
    MOTION_BLOCK_LENGTH,
    MOTION_REGISTER,
    decode_motion_block,
    magnetic_raw_to_tesla,
)


def _diagonal_covariance(value: float) -> list[float]:
    return [value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value]


class Wt901Node(Node):
    """Poll the WT901 registers and publish un-oriented SI-unit IMU data."""

    def __init__(self) -> None:
        super().__init__("bagheera_wt901")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x50)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("gyro_calibration_samples", 200)
        self.declare_parameter("angular_velocity_variance", 0.0001)
        self.declare_parameter("linear_acceleration_variance", 0.04)
        self.declare_parameter("magnetic_field_variance", 2.5e-11)
        self.declare_parameter("mag_sensor_type", -1)
        self.declare_parameter("publish_magnetometer", True)

        bus_number = int(self.get_parameter("i2c_bus").value)
        self._address = int(self.get_parameter("i2c_address").value)
        rate = float(self.get_parameter("publish_rate").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        calibration_samples = int(
            self.get_parameter("gyro_calibration_samples").value
        )
        angular_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )
        acceleration_variance = float(
            self.get_parameter("linear_acceleration_variance").value
        )
        magnetic_variance = float(
            self.get_parameter("magnetic_field_variance").value
        )
        self._publish_magnetometer = bool(
            self.get_parameter("publish_magnetometer").value
        )
        self._motion_block_length = (
            MOTION_BLOCK_LENGTH
            if self._publish_magnetometer
            else INERTIAL_BLOCK_LENGTH
        )
        if smbus is None:
            raise RuntimeError("python3-smbus is required") from SMBUS_IMPORT_ERROR
        if not 0 < self._address < 0x80:
            raise ValueError("i2c_address must be a 7-bit address")
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if (
            angular_variance <= 0.0
            or acceleration_variance <= 0.0
            or magnetic_variance <= 0.0
        ):
            raise ValueError("IMU variances must be positive")

        self._bus = smbus.SMBus(bus_number)
        self._mag_sensor_type: int | None = None
        if self._publish_magnetometer:
            configured_mag_type = int(self.get_parameter("mag_sensor_type").value)
            if configured_mag_type >= 0:
                self._mag_sensor_type = configured_mag_type
            else:
                type_data = self._bus.read_i2c_block_data(
                    self._address, MAG_SENSOR_REGISTER, 2
                )
                self._mag_sensor_type = int.from_bytes(
                    bytes(type_data), byteorder="little", signed=True
                )
            # Validate the reported sensor type before starting the timer.
            magnetic_raw_to_tesla((0, 0, 0), self._mag_sensor_type)
        self._calibration_samples = calibration_samples
        self._bias = GyroBiasEstimator(calibration_samples)
        self._angular_covariance = _diagonal_covariance(angular_variance)
        self._acceleration_covariance = _diagonal_covariance(acceleration_variance)
        self._magnetic_covariance = _diagonal_covariance(magnetic_variance)
        self._publisher = self.create_publisher(
            Imu, "/imu/wt901/data_raw", qos_profile_sensor_data
        )
        self._mag_publisher = (
            self.create_publisher(
                MagneticField, "/imu/wt901/mag_raw", qos_profile_sensor_data
            )
            if self._publish_magnetometer
            else None
        )
        self._consecutive_errors = 0
        self._calibration_announced = False
        self._timer = self.create_timer(1.0 / rate, self._poll)
        self._sleeping = False
        sleep_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/dock/sleep_state", self._on_sleep_state, sleep_qos)
        self.get_logger().info(
            f"WT901 on /dev/i2c-{bus_number} address 0x{self._address:02x}; "
            f"magnetometer {'enabled' if self._publish_magnetometer else 'disabled'}"
            f"{f' (type {self._mag_sensor_type})' if self._publish_magnetometer else ''}; "
            "keep robot still for "
            f"{calibration_samples / rate:.1f} s"
        )

    def _on_sleep_state(self, message: String) -> None:
        sleeping = message.data == "sleeping"
        if sleeping == self._sleeping:
            return
        self._sleeping = sleeping
        if sleeping:
            self._timer.cancel()
            self.get_logger().info("WT901 polling stopped while docked")
            return
        # The robot still stands in the dock: measure the gyro bias again.
        # Nothing is published until that is done, which dock_sleep waits for.
        self._bias = GyroBiasEstimator(self._calibration_samples)
        self._calibration_announced = False
        self._timer.reset()
        self.get_logger().info("WT901 polling resumed; re-measuring gyro bias")

    def _poll(self) -> None:
        try:
            data = self._bus.read_i2c_block_data(
                self._address, MOTION_REGISTER, self._motion_block_length
            )
            sample = decode_motion_block(data)
            self._consecutive_errors = 0
        except (OSError, ValueError) as error:
            self._consecutive_errors += 1
            if self._consecutive_errors == 1 or self._consecutive_errors % 50 == 0:
                self.get_logger().error(
                    f"WT901 I2C read failed ({self._consecutive_errors}): {error}"
                )
            return

        stamp = self.get_clock().now().to_msg()
        if self._mag_publisher is not None and self._mag_sensor_type is not None:
            magnetic = MagneticField()
            magnetic.header.stamp = stamp
            magnetic.header.frame_id = self._frame_id
            (
                magnetic.magnetic_field.x,
                magnetic.magnetic_field.y,
                magnetic.magnetic_field.z,
            ) = magnetic_raw_to_tesla(sample.magnetic_raw, self._mag_sensor_type)
            magnetic.magnetic_field_covariance = self._magnetic_covariance
            self._mag_publisher.publish(magnetic)

        if not self._bias.update(sample.angular_velocity):
            return
        if not self._calibration_announced:
            bias = self._bias.bias
            self.get_logger().info(
                "WT901 gyro bias ready: "
                f"[{bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f}] rad/s"
            )
            self._calibration_announced = True

        angular_velocity = self._bias.correct(sample.angular_velocity)
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        # The WT901 orientation may use magnetometer yaw. Leave it explicitly
        # unavailable; robot_localization consumes only angular_velocity.z.
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ) = angular_velocity
        message.angular_velocity_covariance = self._angular_covariance
        (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ) = sample.acceleration
        message.linear_acceleration_covariance = self._acceleration_covariance
        self._publisher.publish(message)

    def destroy_node(self):
        try:
            self._bus.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Wt901Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
