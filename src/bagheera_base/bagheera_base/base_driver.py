"""ROS 2 driver for the Mowgli STM32 USB CDC protocol."""

import math
import struct
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, MagneticField, Temperature
import serial
from std_msgs.msg import UInt16MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .kinematics import DifferentialOdometry, twist_to_wheels
from .protocol import (
    ACK,
    ACK_NAMES,
    ACK_OK,
    DRIVE,
    EXTERNAL_IMU,
    FEATURE_DRIVE,
    FEATURE_SAFETY_DISABLED,
    GET_INFO,
    INFO,
    ONBOARD_IMU,
    PANEL,
    SET_DRIVE_ENABLE,
    SET_WHEEL_SPEEDS,
    STATUS,
    STOP,
    Frame,
    Info,
    StreamParser,
    decode_ack,
    decode_drive,
    decode_external_imu,
    decode_info,
    decode_onboard_imu,
    decode_panel,
    decode_status,
    drive_enable_payload,
    encode_frame,
    wheel_speed_payload,
)


def yaw_quaternion(yaw: float) -> Quaternion:
    result = Quaternion()
    result.z = math.sin(yaw / 2.0)
    result.w = math.cos(yaw / 2.0)
    return result


class BagheeraBaseDriver(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("bagheera_base_driver", **kwargs)
        self.declare_parameter("port", "/dev/mowgli")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("reconnect_interval", 2.0)
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("allow_unsafe_firmware", False)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("external_imu_frame", "imu_link")
        self.declare_parameter("onboard_imu_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self._port = self.get_parameter("port").value
        self._baud_rate = self.get_parameter("baud_rate").value
        self._reconnect_interval = self.get_parameter("reconnect_interval").value
        self._command_rate = self.get_parameter("command_rate").value
        self._command_timeout = self.get_parameter("command_timeout").value
        self._allow_unsafe = self.get_parameter("allow_unsafe_firmware").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._external_imu_frame = self.get_parameter("external_imu_frame").value
        self._onboard_imu_frame = self.get_parameter("onboard_imu_frame").value
        self._publish_tf = self.get_parameter("publish_tf").value
        if self._command_rate < 10.0:
            raise ValueError("command_rate must be at least 10 Hz for the STM32 watchdog")
        if not 0.0 < self._command_timeout < 1.0:
            raise ValueError("command_timeout must be between 0 and the 1 s STM32 timeout")

        self._serial: Optional[serial.Serial] = None
        self._parser = StreamParser()
        self._next_sequence = 1
        self._last_connect_attempt = 0.0
        self._last_info_request = 0.0
        self._info: Optional[Info] = None
        self._odometry: Optional[DifferentialOdometry] = None
        self._drive_enabled = False
        self._motion_inhibited = False
        self._last_twist: Optional[TwistStamped] = None
        self._last_twist_time = 0.0
        self._last_status = None
        self._last_drive = None
        self._last_drive_uptime: Optional[int] = None

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._battery_pub = self.create_publisher(
            BatteryState, "/battery_state", qos_profile_sensor_data
        )
        self._external_imu_pub = self.create_publisher(
            Imu, "/imu/data_raw", qos_profile_sensor_data
        )
        self._mag_pub = self.create_publisher(MagneticField, "/imu/mag", qos_profile_sensor_data)
        self._raw_mag_pub = self.create_publisher(
            MagneticField, "/imu/mag_raw", qos_profile_sensor_data
        )
        self._onboard_imu_pub = self.create_publisher(
            Imu, "/imu_onboard/data_raw", qos_profile_sensor_data
        )
        self._temperature_pub = self.create_publisher(
            Temperature, "/imu_onboard/temperature", qos_profile_sensor_data
        )
        self._panel_pub = self.create_publisher(UInt16MultiArray, "/bagheera/panel_buttons", 10)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_twist, 10)
        self.create_service(Trigger, "/bagheera/stop", self._on_stop)
        self._io_timer = self.create_timer(0.01, self._io_tick)
        self._command_timer = self.create_timer(1.0 / self._command_rate, self._command_tick)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info("Bagheera base driver waiting for %s" % self._port)

    def _new_sequence(self) -> int:
        result = self._next_sequence
        self._next_sequence = (self._next_sequence + 1) & 0xFFFF
        return result

    def _send(self, message_type: int, payload: bytes = b"") -> None:
        if self._serial is None:
            return
        try:
            self._serial.write(encode_frame(message_type, self._new_sequence(), payload))
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as error:
            self._disconnect("serial write failed: %s" % error)

    def _connect(self) -> None:
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_interval:
            return
        self._last_connect_attempt = now
        try:
            self._serial = serial.Serial(
                self._port,
                self._baud_rate,
                timeout=0,
                write_timeout=0.1,
                exclusive=True,
            )
        except (serial.SerialException, OSError) as error:
            self.get_logger().warning(
                "Cannot open %s: %s" % (self._port, error), throttle_duration_sec=5.0
            )
            self._serial = None
            return
        self._parser.reset()
        self._info = None
        self._odometry = None
        self._drive_enabled = False
        self._motion_inhibited = False
        self._last_twist = None
        self._last_drive_uptime = None
        self.get_logger().info("Connected to %s" % self._port)
        self._last_info_request = now
        self._send(GET_INFO)

    def _best_effort_stop(self) -> None:
        if self._serial is None:
            return
        try:
            self._serial.write(encode_frame(STOP, self._new_sequence()))
            self._serial.write(
                encode_frame(SET_DRIVE_ENABLE, self._new_sequence(), drive_enable_payload(False))
            )
            self._serial.flush()
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            pass
        self._drive_enabled = False

    def _disconnect(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._best_effort_stop()
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None
        self._info = None
        self._odometry = None
        self._last_twist = None
        self._drive_enabled = False

    def _io_tick(self) -> None:
        if self._serial is None:
            self._connect()
            return
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(min(max(waiting, 1), 4096)) if waiting else b""
        except (serial.SerialException, OSError) as error:
            self._disconnect("serial read failed: %s" % error)
            return
        for frame in self._parser.feed(data):
            try:
                self._handle_frame(frame)
            except (ValueError, struct.error) as error:
                self.get_logger().warning(
                    "Discarding malformed 0x%02x payload: %s" % (frame.message_type, error)
                )
        now = time.monotonic()
        if self._info is None and now - self._last_info_request >= 1.0:
            self._last_info_request = now
            self._send(GET_INFO)

    def _handle_frame(self, frame: Frame) -> None:
        if frame.message_type == INFO:
            self._handle_info(decode_info(frame.payload))
        elif frame.message_type == ACK:
            ack = decode_ack(frame.payload)
            if ack.result != ACK_OK:
                if ack.command == SET_DRIVE_ENABLE:
                    self._drive_enabled = False
                self.get_logger().warning(
                    "STM32 rejected command 0x%02x: %s (detail %d)"
                    % (ack.command, ACK_NAMES.get(ack.result, str(ack.result)), ack.detail)
                )
        elif frame.message_type == DRIVE:
            self._handle_drive(decode_drive(frame.payload))
        elif frame.message_type == STATUS:
            self._handle_status(decode_status(frame.payload))
        elif frame.message_type == EXTERNAL_IMU:
            self._handle_external_imu(decode_external_imu(frame.payload))
        elif frame.message_type == ONBOARD_IMU:
            self._handle_onboard_imu(decode_onboard_imu(frame.payload))
        elif frame.message_type == PANEL:
            _, buttons = decode_panel(frame.payload)
            message = UInt16MultiArray()
            message.data = list(buttons)
            self._panel_pub.publish(message)

    def _handle_info(self, info: Info) -> None:
        if info.protocol_version != 1:
            self._disconnect("unsupported STM32 protocol version %d" % info.protocol_version)
            return
        if not info.features & FEATURE_DRIVE:
            self._disconnect("STM32 firmware has no drive feature")
            return
        if info.ticks_per_meter <= 0 or info.wheel_track_mm <= 0 or info.max_wheel_mm_s <= 0:
            self._disconnect("STM32 reported invalid drive geometry")
            return
        self._info = info
        self._odometry = DifferentialOdometry(info.ticks_per_meter, info.wheel_track_mm / 1000.0)
        version = ".".join(str(value) for value in info.firmware)
        self.get_logger().info(
            "Mowgli firmware %s: track=%d mm, encoder=%d ticks/m, limit=%d mm/s"
            % (version, info.wheel_track_mm, info.ticks_per_meter, info.max_wheel_mm_s)
        )
        if info.features & FEATURE_SAFETY_DISABLED:
            if self._allow_unsafe:
                self.get_logger().warning(
                    "UNSAFE TEST OVERRIDE ACTIVE: STM32 periodic safety controller is disabled"
                )
            else:
                self.get_logger().error(
                    "Motion blocked: STM32 periodic safety controller is disabled"
                )

    def _motion_allowed(self) -> bool:
        return bool(
            self._info
            and (
                not self._info.features & FEATURE_SAFETY_DISABLED
                or self._allow_unsafe
            )
            and not self._motion_inhibited
        )

    def _on_twist(self, message: TwistStamped) -> None:
        linear = message.twist.linear.x
        angular = message.twist.angular.z
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().error("Ignoring non-finite /cmd_vel")
            self._stop_and_disable()
            return
        if self._motion_inhibited:
            if abs(linear) < 1e-6 and abs(angular) < 1e-6:
                self._motion_inhibited = False
                self.get_logger().info("Motion inhibit cleared by a zero velocity command")
            else:
                return
        self._last_twist = message
        self._last_twist_time = time.monotonic()

    def _command_tick(self) -> None:
        if self._info is None or self._serial is None:
            return
        fresh = (
            self._last_twist is not None
            and time.monotonic() - self._last_twist_time <= self._command_timeout
        )
        if not fresh or not self._motion_allowed():
            if self._drive_enabled:
                self._stop_and_disable()
            return
        assert self._last_twist is not None
        left, right = twist_to_wheels(
            self._last_twist.twist.linear.x,
            self._last_twist.twist.angular.z,
            self._info.wheel_track_mm / 1000.0,
            self._info.max_wheel_mm_s / 1000.0,
        )
        if not self._drive_enabled:
            self._send(SET_DRIVE_ENABLE, drive_enable_payload(True))
            self._drive_enabled = True
        self._send(SET_WHEEL_SPEEDS, wheel_speed_payload(left, right))

    def _stop_and_disable(self) -> None:
        self._send(STOP)
        self._send(SET_DRIVE_ENABLE, drive_enable_payload(False))
        self._drive_enabled = False
        self._last_twist = None

    def _on_stop(self, _request, response):
        self._motion_inhibited = True
        self._stop_and_disable()
        response.success = True
        response.message = "Drive stopped; send zero /cmd_vel to clear the motion inhibit"
        return response

    def _handle_drive(self, drive) -> None:
        self._last_drive = drive
        if drive.flags & (1 << 3):
            if not self._motion_inhibited:
                self.get_logger().error("STM32 emergency state active; motion inhibited")
            self._motion_inhibited = True
            self._last_twist = None
            self._drive_enabled = False
        if self._odometry is None:
            return
        if self._last_drive_uptime is not None:
            uptime_delta = (drive.uptime_ms - self._last_drive_uptime) & 0xFFFFFFFF
            if uptime_delta > 0x80000000:
                self.get_logger().warning("STM32 restart detected; resetting encoder baseline")
                self._odometry.reset_ticks(drive.left_ticks, drive.right_ticks)
        self._last_drive_uptime = drive.uptime_ms
        update = self._odometry.update(
            drive.left_ticks,
            drive.right_ticks,
            drive.measured_left_mm_s / 1000.0,
            drive.measured_right_mm_s / 1000.0,
        )
        stamp = self.get_clock().now().to_msg()
        orientation = yaw_quaternion(update.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = update.x
        odom.pose.pose.position.y = update.y
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = update.linear_velocity
        odom.twist.twist.angular.z = update.angular_velocity
        self._odom_pub.publish(odom)
        if self._publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = update.x
            transform.transform.translation.y = update.y
            transform.transform.rotation = orientation
            self._tf.sendTransform(transform)

    def _handle_status(self, status) -> None:
        self._last_status = status
        emergency = bool(status.system_flags & (1 << 4))
        if emergency:
            if not self._motion_inhibited:
                self.get_logger().error("STM32 safety condition active; motion inhibited")
            self._motion_inhibited = True
            self._last_twist = None
            self._drive_enabled = False
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = status.battery_mv / 1000.0
        # BatteryState defines discharge current as positive and charge current as negative.
        battery.current = -status.charge_ma / 1000.0 if status.system_flags & 1 else math.nan
        battery.percentage = math.nan
        battery.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if status.system_flags & 1
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery.present = status.battery_mv > 0
        self._battery_pub.publish(battery)

    def _handle_external_imu(self, telemetry) -> None:
        stamp = self.get_clock().now().to_msg()
        if telemetry.validity & 0x03:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self._external_imu_frame
            imu.orientation_covariance[0] = -1.0
            if telemetry.validity & 0x01:
                (
                    imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                ) = telemetry.acceleration
            else:
                imu.linear_acceleration_covariance[0] = -1.0
            if telemetry.validity & 0x02:
                (
                    imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z,
                ) = telemetry.angular_velocity
            else:
                imu.angular_velocity_covariance[0] = -1.0
            self._external_imu_pub.publish(imu)
        if telemetry.validity & 0x04:
            self._publish_magnetic(stamp, telemetry.magnetic_field, self._mag_pub)
        if telemetry.validity & 0x08:
            self._publish_magnetic(stamp, telemetry.raw_magnetic_field, self._raw_mag_pub)

    def _publish_magnetic(self, stamp, values, publisher) -> None:
        message = MagneticField()
        message.header.stamp = stamp
        message.header.frame_id = self._external_imu_frame
        message.magnetic_field.x, message.magnetic_field.y, message.magnetic_field.z = values
        publisher.publish(message)

    def _handle_onboard_imu(self, telemetry) -> None:
        stamp = self.get_clock().now().to_msg()
        if telemetry.validity & 0x01:
            message = Imu()
            message.header.stamp = stamp
            message.header.frame_id = self._onboard_imu_frame
            message.orientation_covariance[0] = -1.0
            message.angular_velocity_covariance[0] = -1.0
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ) = telemetry.acceleration
            self._onboard_imu_pub.publish(message)
        if telemetry.validity & 0x02:
            temperature = Temperature()
            temperature.header.stamp = stamp
            temperature.header.frame_id = self._onboard_imu_frame
            temperature.temperature = telemetry.temperature_c
            temperature.variance = 0.0
            self._temperature_pub.publish(temperature)

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "Bagheera base/STM32"
        status.hardware_id = self._port
        if self._serial is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "Disconnected"
        elif self._info is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Waiting for firmware information"
        elif self._motion_inhibited:
            status.level = DiagnosticStatus.ERROR
            status.message = "Motion inhibited"
        elif self._info.features & FEATURE_SAFETY_DISABLED:
            status.level = DiagnosticStatus.WARN if self._allow_unsafe else DiagnosticStatus.ERROR
            status.message = (
                "Safety disabled; test override active"
                if self._allow_unsafe
                else "Motion blocked: safety disabled"
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Connected"
        values = {
            "drive_enabled": self._drive_enabled,
            "motion_inhibited": self._motion_inhibited,
            "host_crc_errors": self._parser.crc_errors,
            "host_length_errors": self._parser.length_errors,
        }
        if self._last_status is not None:
            values.update(
                battery_voltage_v=self._last_status.battery_mv / 1000.0,
                charge_voltage_v=self._last_status.charge_mv / 1000.0,
                charge_current_a=self._last_status.charge_ma / 1000.0,
                firmware_crc_errors=self._last_status.crc_errors,
                firmware_length_errors=self._last_status.length_errors,
                firmware_rx_overflows=self._last_status.rx_overflows,
                firmware_tx_drops=self._last_status.tx_drops,
                safety_flags="0x%04x" % self._last_status.safety_flags,
                system_flags="0x%04x" % self._last_status.system_flags,
            )
        if self._last_drive is not None:
            values.update(
                command_age_ms=self._last_drive.command_age_ms,
                left_motor_power=self._last_drive.left_power,
                right_motor_power=self._last_drive.right_power,
            )
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        array.status = [status]
        self._diagnostic_pub.publish(array)

    def destroy_node(self) -> bool:
        self._best_effort_stop()
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        return super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BagheeraBaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
