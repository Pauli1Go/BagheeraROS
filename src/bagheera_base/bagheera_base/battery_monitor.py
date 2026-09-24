"""Averaged battery voltage, percentage and level for Bagheera.

Reads ``/hardware_bridge/power`` (MowgliNext firmware), corrects the pack
voltage by the offset measured on this robot, averages it and publishes:

- ``/battery/voltage`` (std_msgs/Float32, V): averaged, corrected pack voltage
- ``/battery/percentage`` (std_msgs/Float32, 0..100): voltage-based estimate
- ``/battery/level`` (std_msgs/String, latched): NORMAL, LOW, CRITICAL, FULL

This node only reports; reacting to LOW/CRITICAL (returning to the dock) is
left to the behavior tree.
"""

from __future__ import annotations

import math
import time

from mowgli_interfaces.msg import Power
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from .battery_math import BatteryLevelTracker


class BatteryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_battery_monitor")
        self.declare_parameter("power_topic", "/hardware_bridge/power")
        # Added to the firmware's v_battery. Measured on Bagheera on
        # 2026-09-24 (MowgliNext firmware 1.9.10) against a multimeter at the
        # pack: 25.71/28.10/28.37 V real vs. 26.49/28.90/29.20 V reported, a
        # constant offset without gain error. Specific to this robot's board.
        self.declare_parameter("voltage_offset_v", -0.80)
        self.declare_parameter("cell_count", 7)
        self.declare_parameter("filter_window_s", 60.0)
        self.declare_parameter("warmup_s", 10.0)
        self.declare_parameter("low_voltage", 25.3)
        self.declare_parameter("critical_voltage", 24.5)
        # Same electrical dock test as the autonomy dock guard.
        self.declare_parameter("dock_voltage_threshold", 10.0)
        self.declare_parameter("charging_current_a", 0.2)
        self.declare_parameter("full_voltage", 28.4)
        self.declare_parameter("full_current_a", 0.15)
        self.declare_parameter("full_hold_s", 120.0)
        self.declare_parameter("full_release_voltage", 28.0)
        self.declare_parameter("publish_rate", 1.0)

        self._offset = float(self.get_parameter("voltage_offset_v").value)
        self._dock_voltage = float(self.get_parameter("dock_voltage_threshold").value)
        self._tracker = BatteryLevelTracker(
            cell_count=int(self.get_parameter("cell_count").value),
            window_s=float(self.get_parameter("filter_window_s").value),
            warmup_s=float(self.get_parameter("warmup_s").value),
            low_voltage=float(self.get_parameter("low_voltage").value),
            critical_voltage=float(self.get_parameter("critical_voltage").value),
            charging_current=float(self.get_parameter("charging_current_a").value),
            full_voltage=float(self.get_parameter("full_voltage").value),
            full_current=float(self.get_parameter("full_current_a").value),
            full_hold_s=float(self.get_parameter("full_hold_s").value),
            full_release_voltage=float(self.get_parameter("full_release_voltage").value),
        )
        self._published_level: str | None = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._level_pub = self.create_publisher(String, "/battery/level", latched)
        self._voltage_pub = self.create_publisher(Float32, "/battery/voltage", 10)
        self._percentage_pub = self.create_publisher(Float32, "/battery/percentage", 10)
        self.create_subscription(
            Power, str(self.get_parameter("power_topic").value), self._on_power, 10
        )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)
        self.get_logger().info(
            "Battery monitor: offset %+.2f V, LOW < %.2f V, CRITICAL < %.2f V"
            % (
                self._offset,
                self.get_parameter("low_voltage").value,
                self.get_parameter("critical_voltage").value,
            )
        )

    def _on_power(self, message: Power) -> None:
        if not math.isfinite(message.v_battery) or message.v_battery <= 0.0:
            return
        docked = math.isfinite(message.v_charge) and message.v_charge >= self._dock_voltage
        current = message.charge_current if math.isfinite(message.charge_current) else 0.0
        self._tracker.update(
            time.monotonic(), message.v_battery + self._offset, docked, current
        )
        self._publish_level()

    def _publish_level(self) -> None:
        level = self._tracker.level
        if self._tracker.voltage is None or level == self._published_level:
            return
        log = (
            self.get_logger().warning
            if level in ("LOW", "CRITICAL")
            else self.get_logger().info
        )
        log("Battery level %s at %.2f V (~%.0f %%)"
            % (level, self._tracker.voltage, self._tracker.percentage))
        self._level_pub.publish(String(data=level))
        self._published_level = level

    def _publish(self) -> None:
        if self._tracker.voltage is None:
            return
        self._voltage_pub.publish(Float32(data=float(self._tracker.voltage)))
        self._percentage_pub.publish(Float32(data=float(self._tracker.percentage)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
