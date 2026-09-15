#!/usr/bin/env python3
"""Command a constant velocity and measure what survives the command chain.

Publishes on a teleop-style topic and records every stage of the chain
(twist_mux output, collision monitor output, wheel odometry, EKF output) so a
stuttering or missing motion can be traced to one specific stage.

Run inside the robot container:
  python3 tools/drive_probe.py --topic /cmd_vel_teleop --angular 0.5 --seconds 6
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover - optional diagnostics
    CollisionMonitorState = None


def stats(values):
    if not values:
        return "no data"
    return (f"n={len(values):4d} min={min(values):+.3f} mean={sum(values) / len(values):+.3f} "
            f"max={max(values):+.3f}")


class DriveProbe(Node):
    def __init__(self, args):
        super().__init__("drive_probe")
        self.args = args
        self.log = {name: [] for name in (
            "mux", "monitored", "wheel_odom", "ekf", "monitor_state")}
        self.timestamps = {name: [] for name in self.log}

        self.publisher = self.create_publisher(TwistStamped, args.topic, 10)
        for topic, key in (("/cmd_vel_requested", "mux"), ("/cmd_vel", "monitored")):
            self.create_subscription(TwistStamped, topic,
                                     lambda msg, key=key: self.on_twist(msg, key), 20)
        odom_qos = QoSProfile(depth=50)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Odometry, "/wheel_odom",
                                 lambda msg: self.on_odom(msg, "wheel_odom"), odom_qos)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda msg: self.on_odom(msg, "ekf"), odom_qos)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     self.on_monitor, 10)

    def on_twist(self, message, key):
        self.log[key].append(message.twist.angular.z)
        self.log[key + "_lin"] = self.log.get(key + "_lin", []) + [message.twist.linear.x]
        self.timestamps[key].append(time.monotonic())

    def on_odom(self, message, key):
        self.log[key].append(message.twist.twist.angular.z)
        self.timestamps[key].append(time.monotonic())

    def on_monitor(self, message):
        self.log["monitor_state"].append(getattr(message, "action_type", -99))
        self.timestamps["monitor_state"].append(time.monotonic())

    def run(self):
        command = TwistStamped()
        command.header.frame_id = "base_link"
        command.twist.linear.x = self.args.linear
        command.twist.angular.z = self.args.angular
        print(f"publishing on {self.args.topic}: linear {self.args.linear} m/s, "
              f"angular {self.args.angular} rad/s for {self.args.seconds} s", flush=True)

        end = time.monotonic() + self.args.seconds
        sent = 0
        while time.monotonic() < end:
            command.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(command)
            sent += 1
            rclpy.spin_once(self, timeout_sec=0.05)

        stop = TwistStamped()
        stop.header.frame_id = "base_link"
        for _ in range(30):
            stop.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)

        print(f"sent {sent} commands")
        for key in self.log:
            values = self.log[key]
            if key == "monitor_state":
                summary = sorted(set(values)) if values else "no data"
                print(f"  {key:14s}: {summary}")
                continue
            print(f"  {key:14s}: {stats(values)}")

        for key in ("mux", "monitored", "wheel_odom", "ekf"):
            stamps = self.timestamps[key]
            if len(stamps) > 2:
                span = stamps[-1] - stamps[0]
                print(f"  {key:14s}: rate {len(stamps) / span:.1f} Hz" if span > 0 else "")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cmd_vel_teleop")
    parser.add_argument("--linear", type=float, default=0.0)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=6.0)
    args = parser.parse_args()

    rclpy.init()
    node = DriveProbe(args)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
