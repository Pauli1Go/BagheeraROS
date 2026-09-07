"""Advertise the explicit MowgliNext mode used by Bagheera teleoperation."""

import rclpy
from mowgli_interfaces.msg import HighLevelStatus
from rclpy.node import Node


class ManualModePublisher(Node):
    """Keep the firmware in manual-drive mode without ever commanding a blade."""

    def __init__(self) -> None:
        super().__init__("bagheera_mode")
        self._publisher = self.create_publisher(
            HighLevelStatus, "/behavior_tree_node/high_level_status", 10
        )
        self._timer = self.create_timer(0.25, self._publish_manual)
        self._publish_manual()

    def _message(self, state: int, name: str) -> HighLevelStatus:
        message = HighLevelStatus()
        message.state = state
        message.state_name = name
        message.sub_state_name = "CONTROLLER_TELEOP"
        message.current_area = -1
        message.current_path = -1
        message.current_path_index = -1
        return message

    def _publish_manual(self) -> None:
        self._publisher.publish(
            self._message(HighLevelStatus.HIGH_LEVEL_STATE_MANUAL_MOWING, "BAGHEERA_MANUAL")
        )

    def destroy_node(self) -> bool:
        self._publisher.publish(
            self._message(HighLevelStatus.HIGH_LEVEL_STATE_IDLE, "BAGHEERA_IDLE")
        )
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualModePublisher()
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
