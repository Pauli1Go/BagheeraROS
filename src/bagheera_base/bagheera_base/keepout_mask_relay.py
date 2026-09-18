"""Latch the static keepout mask for Nav2 costmap filters."""

from __future__ import annotations

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class KeepoutMaskRelay(Node):
    """Avoid a Nav2/CycloneDDS late-join race seen on the Raspberry Pi.

    The relay starts with the navigation nodes and publishes once when the map
    server delivers the mask. TRANSIENT_LOCAL durability retains that sample
    for later subscriptions without periodically sending the large grid.
    """

    def __init__(self) -> None:
        super().__init__("bagheera_keepout_mask_relay")
        self.declare_parameter("input_topic", "/keepout_filter_mask")
        self.declare_parameter("output_topic", "/keepout_filter_mask_live")
        qos = _latched_qos()
        self._mask: OccupancyGrid | None = None
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("input_topic").value),
            self._on_mask,
            qos,
        )

    def _on_mask(self, message: OccupancyGrid) -> None:
        self._mask = message
        self._mask.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._mask)
        self.get_logger().info(
            "Keepout mask relay loaded %d x %d mask"
            % (message.info.width, message.info.height)
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeepoutMaskRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
