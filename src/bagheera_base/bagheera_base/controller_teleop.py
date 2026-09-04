"""Pygame controller teleoperation node publishing standard ROS 2 velocity."""

import os
from typing import Optional

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from .controller_mapping import axes_to_twist


class ControllerTeleop(Node):
    def __init__(self) -> None:
        super().__init__("bagheera_controller")
        self.declare_parameter("joystick_index", 0)
        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("throttle_axis", 3)
        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("publish_rate", 20.0)

        self._joystick_index = self.get_parameter("joystick_index").value
        self._steering_axis = self.get_parameter("steering_axis").value
        self._throttle_axis = self.get_parameter("throttle_axis").value
        self._deadman_button = self.get_parameter("deadman_button").value
        self._deadzone = self.get_parameter("deadzone").value
        self._max_linear = self.get_parameter("max_linear_speed").value
        self._max_angular = self.get_parameter("max_angular_speed").value
        publish_rate = self.get_parameter("publish_rate").value
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        pygame.init()
        pygame.display.init()
        pygame.joystick.init()
        self._joystick: Optional[pygame.joystick.Joystick] = None
        self._last_deadman = False
        self._missing_logged = False
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(
            "Controller teleop ready; hold button %d to drive" % self._deadman_button
        )

    def _discover_joystick(self) -> bool:
        pygame.event.pump()
        if self._joystick is not None and self._joystick.get_init():
            return True
        if pygame.joystick.get_count() <= self._joystick_index:
            if not self._missing_logged:
                self.get_logger().warning(
                    "Joystick %d not available; waiting" % self._joystick_index
                )
                self._missing_logged = True
            return False
        self._joystick = pygame.joystick.Joystick(self._joystick_index)
        self._joystick.init()
        self._missing_logged = False
        self.get_logger().info("Using controller: %s" % self._joystick.get_name())
        return True

    def _publish(self, linear: float, angular: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self._publisher.publish(message)

    def _tick(self) -> None:
        try:
            if not self._discover_joystick():
                if self._last_deadman:
                    self._publish(0.0, 0.0)
                self._last_deadman = False
                return
            pygame.event.pump()
            assert self._joystick is not None
            required_axis = max(self._steering_axis, self._throttle_axis)
            if self._joystick.get_numaxes() <= required_axis:
                self.get_logger().error(
                    "Controller has %d axes, but axis %d is configured"
                    % (self._joystick.get_numaxes(), required_axis),
                    throttle_duration_sec=5.0,
                )
                return
            if self._joystick.get_numbuttons() <= self._deadman_button:
                self.get_logger().error(
                    "Controller has %d buttons, but button %d is configured"
                    % (self._joystick.get_numbuttons(), self._deadman_button),
                    throttle_duration_sec=5.0,
                )
                return

            deadman = bool(self._joystick.get_button(self._deadman_button))
            if deadman:
                linear, angular = axes_to_twist(
                    self._joystick.get_axis(self._throttle_axis),
                    self._joystick.get_axis(self._steering_axis),
                    self._deadzone,
                    self._max_linear,
                    self._max_angular,
                )
                self._publish(linear, angular)
            elif self._last_deadman:
                self._publish(0.0, 0.0)
            self._last_deadman = deadman
        except pygame.error as error:
            if self._last_deadman:
                self._publish(0.0, 0.0)
            self._last_deadman = False
            self._joystick = None
            self.get_logger().warning("Controller disconnected: %s" % error)

    def destroy_node(self) -> bool:
        if self._last_deadman:
            self._publish(0.0, 0.0)
        pygame.quit()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerTeleop()
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
