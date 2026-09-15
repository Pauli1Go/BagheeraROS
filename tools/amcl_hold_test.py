#!/usr/bin/env python3
"""Set an initial pose and measure whether AMCL actually holds it.

Prints the pose track, the biggest single-update jump and the drift from the
pose that was set. Optionally spins or drives the robot while recording, which
is when a badly tuned filter usually starts jumping between rooms.

Run inside the robot container:
  python3 tools/amcl_hold_test.py --pose -1.29,0.96,316 --spin 0.4 6
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
    return math.atan2(siny, cosy)


def angle_difference(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class HoldTest(Node):
    def __init__(self, args):
        super().__init__("amcl_hold_test")
        self.args = args
        self.samples = []
        odom_qos = QoSProfile(depth=20)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos.history = HistoryPolicy.KEEP_LAST
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_requested", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.on_pose, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom,
                                 odom_qos)
        self.odom = None

    def on_pose(self, message):
        pose = message.pose.pose
        self.samples.append((time.monotonic(), pose.position.x, pose.position.y,
                             yaw_from_quaternion(pose.orientation),
                             math.sqrt(max(0.0, message.pose.covariance[0])),
                             math.sqrt(max(0.0, message.pose.covariance[7])),
                             math.sqrt(max(0.0, message.pose.covariance[35]))))

    def on_odom(self, message):
        pose = message.pose.pose
        self.odom = (pose.position.x, pose.position.y,
                     yaw_from_quaternion(pose.orientation))

    def publish_initial_pose(self, deadline=1.0):
        x, y, yaw_deg = self.args.pose
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        message.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        covariance = [0.0] * 36
        covariance[0] = self.args.sigma_xy ** 2
        covariance[7] = self.args.sigma_xy ** 2
        covariance[35] = self.args.sigma_yaw ** 2
        message.pose.covariance = covariance
        end = time.monotonic() + deadline
        while time.monotonic() < end:
            message.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
        print(f"set pose ({x:.3f}, {y:.3f}, {yaw_deg:.1f} deg) with sigma_xy="
              f"{self.args.sigma_xy} m, sigma_yaw={self.args.sigma_yaw} rad",
              flush=True)

    def drive(self, linear, angular, duration):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.cmd_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
        stop = Twist()
        for _ in range(20):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def report(samples, args, label):
    if len(samples) < 3:
        print(f"[{label}] no AMCL pose received")
        return
    worst_jump, worst_index = 0.0, 1
    worst_turn, turn_index = 0.0, 1
    for index in range(1, len(samples)):
        _, x0, y0, yaw0, _, _, _ = samples[index - 1]
        _, x1, y1, yaw1, _, _, _ = samples[index]
        jump = math.hypot(x1 - x0, y1 - y0)
        turn = angle_difference(yaw1, yaw0)
        if jump > worst_jump:
            worst_jump, worst_index = jump, index
        if turn > worst_turn:
            worst_turn, turn_index = turn, index
    first, last = samples[0], samples[-1]
    drift = math.hypot(last[1] - first[1], last[2] - first[2])
    final = samples[-1]
    print(f"[{label}] samples {len(samples)}, duration {last[0] - first[0]:.1f} s")
    print(f"[{label}] max single jump {worst_jump:.3f} m at t={samples[worst_index][0] - first[0]:.1f} s; "
          f"max single turn {math.degrees(worst_turn):.1f} deg at t={samples[turn_index][0] - first[0]:.1f} s")
    print(f"[{label}] moved {drift:.3f} m, final pose ({final[1]:.2f}, {final[2]:.2f}, "
          f"{math.degrees(final[3]):.1f} deg), 1-sigma {final[4]:.3f} m / {math.degrees(final[6]):.1f} deg")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", required=True, help="x,y,yaw_deg")
    parser.add_argument("--sigma-xy", type=float, default=0.10)
    parser.add_argument("--sigma-yaw", type=float, default=0.05)
    parser.add_argument("--settle", type=float, default=8.0)
    parser.add_argument("--spin", nargs=2, type=float, metavar=("SPEED", "SECONDS"))
    parser.add_argument("--drive", nargs=2, type=float, metavar=("SPEED", "SECONDS"))
    parser.add_argument("--after", type=float, default=5.0)
    args = parser.parse_args()
    args.pose = tuple(float(value) for value in args.pose.split(","))

    rclpy.init()
    node = HoldTest(args)
    node.publish_initial_pose()

    node.samples = []
    node.wait(args.settle)
    report(node.samples, args, "standstill")

    if args.spin:
        node.samples = []
        node.drive(0.0, args.spin[0], args.spin[1])
        node.wait(args.after)
        report(node.samples, args, f"spin {args.spin[0]} rad/s")

    if args.drive:
        node.samples = []
        node.drive(args.drive[0], 0.0, args.drive[1])
        node.wait(args.after)
        report(node.samples, args, f"drive {args.drive[0]} m/s")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
