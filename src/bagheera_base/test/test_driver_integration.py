"""ROS/PTY integration check for the real Bagheera base-driver node."""

import fcntl
import os
import pty
import struct
import time
import tty
import unittest

try:
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState

    from bagheera_base.base_driver import BagheeraBaseDriver
    from bagheera_base.protocol import (
        DRIVE,
        FEATURE_DRIVE,
        FEATURE_SAFETY_DISABLED,
        GET_INFO,
        INFO,
        SET_DRIVE_ENABLE,
        SET_WHEEL_SPEEDS,
        STATUS,
        StreamParser,
        encode_frame,
    )

    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 runtime is not installed")
class DriverIntegrationTest(unittest.TestCase):
    """Exercise serial transport and ROS interfaces without robot hardware."""

    def setUp(self):
        """Create a raw pseudo-terminal, driver, and ROS probe node."""
        rclpy.init()
        self.master_fd, slave_fd = pty.openpty()
        tty.setraw(self.master_fd)
        tty.setraw(slave_fd)
        flags = fcntl.fcntl(self.master_fd, fcntl.F_GETFL)
        fcntl.fcntl(self.master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        slave_path = os.ttyname(slave_fd)
        os.close(slave_fd)

        self.driver = BagheeraBaseDriver(
            parameter_overrides=[
                Parameter("port", value=slave_path),
                Parameter("reconnect_interval", value=0.01),
                Parameter("allow_unsafe_firmware", value=True),
            ]
        )
        self.probe = Node("bagheera_test_probe")
        self.velocity_pub = self.probe.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.odometry = []
        self.batteries = []
        self.probe.create_subscription(Odometry, "/odom", self.odometry.append, 10)
        self.probe.create_subscription(
            BatteryState,
            "/battery_state",
            self.batteries.append,
            qos_profile_sensor_data,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.driver)
        self.executor.add_node(self.probe)
        self.parser = StreamParser()

    def tearDown(self):
        """Stop the driver and release all local test resources."""
        self.executor.remove_node(self.probe)
        self.executor.remove_node(self.driver)
        self.probe.destroy_node()
        self.driver.destroy_node()
        self.executor.shutdown()
        os.close(self.master_fd)
        rclpy.try_shutdown()

    def _spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def _read_frames(self):
        result = []
        while True:
            try:
                chunk = os.read(self.master_fd, 4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            result.extend(self.parser.feed(chunk))
        return result

    def test_velocity_commands_and_telemetry_topics(self):
        """Handshake, drive, then publish firmware telemetry as ROS messages."""
        self._spin(0.1)
        requests = self._read_frames()
        self.assertIn(GET_INFO, [frame.message_type for frame in requests])

        info_payload = struct.pack(
            "<BBBBIhHHHH",
            1,
            2,
            0,
            0,
            FEATURE_DRIVE | FEATURE_SAFETY_DISABLED,
            600,
            250,
            285,
            1000,
            324,
        )
        os.write(self.master_fd, encode_frame(INFO, 1, info_payload))
        self._spin(0.3)

        command = TwistStamped()
        command.header.stamp = self.probe.get_clock().now().to_msg()
        command.twist.linear.x = 0.2
        self.velocity_pub.publish(command)
        self._spin(0.3)
        command_frames = self._read_frames()
        command_types = [frame.message_type for frame in command_frames]
        self.assertIn(SET_DRIVE_ENABLE, command_types)
        self.assertIn(SET_WHEEL_SPEEDS, command_types)
        wheel_frame = next(
            frame for frame in command_frames if frame.message_type == SET_WHEEL_SPEEDS
        )
        self.assertEqual(struct.unpack("<hh", wheel_frame.payload), (200, 200))

        drive_payload = struct.pack(
            "<IiihhhhHHBB", 1000, 250, 250, 200, 200, 200, 200, 10, 7, 20, 20
        )
        status_payload = struct.pack(
            "<IHHHHHHhBBBBHHHH",
            1000,
            24500,
            0,
            0,
            0,
            0,
            0,
            2500,
            2,
            0,
            0,
            1,
            0,
            0,
            0,
            0,
        )
        os.write(self.master_fd, encode_frame(DRIVE, 2, drive_payload))
        os.write(self.master_fd, encode_frame(STATUS, 3, status_payload))
        self._spin(0.3)
        self.assertTrue(self.odometry)
        self.assertTrue(self.batteries)
        self.assertAlmostEqual(self.odometry[-1].twist.twist.linear.x, 0.2)
        self.assertAlmostEqual(self.batteries[-1].voltage, 24.5)


if __name__ == "__main__":
    unittest.main()
