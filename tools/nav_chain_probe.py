#!/usr/bin/env python3
"""Record every stage of the Nav2 velocity chain while the robot drives.

Stages: controller output, velocity smoother, twist_mux, collision monitor and
the wheel odometry that shows what the robot actually did. The strip chart
makes it obvious which stage drops or stalls the command.

Run inside the robot container:
  python3 tools/nav_chain_probe.py --seconds 60
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

STAGES = ("nav", "smooth", "mux", "cmd", "wheel")


class ChainProbe(Node):
    def __init__(self):
        super().__init__("nav_chain_probe")
        self.start = None
        self.samples = {stage: [] for stage in STAGES}
        self.ranges = {}

        for topic, stage in (("/cmd_vel_nav", "nav"),
                             ("/cmd_vel_monitored", "smooth"),
                             ("/cmd_vel_requested", "mux"),
                             ("/cmd_vel", "cmd")):
            self.create_subscription(TwistStamped, topic,
                                     lambda msg, stage=stage: self.on_twist(msg, stage), 30)
        odom_qos = QoSProfile(depth=50)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Odometry, "/wheel_odom", self.on_wheel, odom_qos)

    def stamp(self):
        if self.start is None:
            self.start = time.monotonic()
        return time.monotonic() - self.start

    def on_twist(self, message, stage):
        self.samples[stage].append((self.stamp(), message.twist.angular.z,
                                    message.twist.linear.x))

    def on_wheel(self, message):
        self.samples["wheel"].append((self.stamp(), message.twist.twist.angular.z,
                                      message.twist.twist.linear.x))

    def at(self, stage, moment, window=0.30):
        values = [value for stamp, value, _ in self.samples[stage]
                  if abs(stamp - moment) <= window]
        return max(values, key=abs) if values else None

    def summary(self):
        for stage in STAGES:
            samples = self.samples[stage]
            if not samples:
                print(f"  {stage:6s}: NO DATA")
                continue
            stamps = [stamp for stamp, _, _ in samples]
            gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b - a > 0.25]
            angular = [abs(value) for _, value, _ in samples]
            span = stamps[-1] - stamps[0]
            moving = sum(1 for value in angular if value > 0.05) / len(angular)
            rate = len(samples) / span if span > 0 else 0.0
            print(f"  {stage:6s}: n={len(samples):5d} rate={rate:5.1f} Hz "
                  f"|w| mean={sum(angular) / len(angular):.3f} max={max(angular):.3f} "
                  f"moving={100 * moving:5.1f}%  gaps>0.25s={len(gaps)} "
                  f"total_gap={sum(gaps):.1f}s")

    def strip(self, seconds, step=0.2):
        print("\n  t | " + " | ".join(f"{stage:>5s}" for stage in STAGES))
        moment = 0.0
        while moment <= seconds:
            cells = []
            for stage in STAGES:
                value = self.at(stage, moment)
                cells.append("  ---" if value is None else f"{value:+.2f}")
            print(f"{moment:5.1f} " + " | ".join(f"{cell:>5s}" for cell in cells))
            moment += step


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--strip-seconds", type=float, default=40.0)
    parser.add_argument("--step", type=float, default=0.25)
    args = parser.parse_args()

    rclpy.init()
    node = ChainProbe()
    print(f"recording the velocity chain for {args.seconds:.0f} s - send the goal now", flush=True)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.summary()
    node.strip(min(args.strip_seconds, args.seconds), args.step)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
