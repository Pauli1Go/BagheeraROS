"""Latch the static keepout mask for Nav2 and re-send masks to late viewers."""

from __future__ import annotations

import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def _same_grid(a: OccupancyGrid, b: OccupancyGrid) -> bool:
    return a.info == b.info and a.data == b.data


class _MaskRepeater:
    """Re-publish one latched mask whenever a new subscriber appears.

    foxglove_bridge picks its subscription durability from the publishers it
    knows at that moment. Right after boot it often does not know the map
    servers yet, subscribes VOLATILE and misses their single publication, so
    the mask stays missing in Foxglove until the next restart.
    """

    def __init__(self, node: Node, topic: str, on_mask=None) -> None:
        self._node = node
        self._topic = topic
        self._on_mask = on_mask
        self._mask: OccupancyGrid | None = None
        self._known: set[bytes] = set()
        self._new_since: float | None = None
        qos = _latched_qos()
        self._publisher = node.create_publisher(OccupancyGrid, topic, qos)
        node.create_subscription(OccupancyGrid, topic, self._receive, qos)

    def _receive(self, message: OccupancyGrid) -> None:
        # Our own re-publication comes back here as well.
        if self._mask is not None and _same_grid(message, self._mask):
            return
        self._mask = message
        if self._on_mask is not None:
            self._on_mask(message)

    def tick(self, now: float) -> None:
        own = self._node.get_name()
        subscribers = {
            bytes(info.endpoint_gid)
            for info in self._node.get_subscriptions_info_by_topic(self._topic)
            if info.node_name != own
        }
        if subscribers - self._known:
            self._new_since = now
        self._known = subscribers
        # Give discovery a moment to match the new subscription first.
        if self._new_since is None or now - self._new_since < 1.0 or self._mask is None:
            return
        self._new_since = None
        self._publisher.publish(self._mask)
        self._node.get_logger().info(f"Re-sent {self._topic} to new subscribers")


class KeepoutMaskRelay(Node):
    """Avoid a Nav2/CycloneDDS late-join race seen on the Raspberry Pi.

    The relay starts with the navigation nodes and publishes once when the map
    server delivers the mask. TRANSIENT_LOCAL durability retains that sample
    for later subscriptions without periodically sending the large grid.
    Additionally, the masks in ``repeat_topics`` are re-sent once whenever a
    new subscriber (typically a reconnecting Foxglove) appears.
    """

    def __init__(self) -> None:
        super().__init__("bagheera_keepout_mask_relay")
        self.declare_parameter("input_topic", "/keepout_filter_mask")
        self.declare_parameter("output_topic", "/keepout_filter_mask_live")
        self.declare_parameter(
            "repeat_topics", ["/keepout_filter_mask", "/localization_exclusion_mask"]
        )
        input_topic = str(self.get_parameter("input_topic").value)
        self._publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("output_topic").value), _latched_qos()
        )
        repeat_topics = [str(topic) for topic in self.get_parameter("repeat_topics").value]
        self._repeaters = [
            _MaskRepeater(self, topic, self._on_mask if topic == input_topic else None)
            for topic in repeat_topics
        ]
        if input_topic not in repeat_topics:
            self.create_subscription(
                OccupancyGrid, input_topic, self._on_mask, _latched_qos()
            )
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        now = time.monotonic()
        for repeater in self._repeaters:
            repeater.tick(now)

    def _on_mask(self, message: OccupancyGrid) -> None:
        mask = OccupancyGrid()
        mask.header = message.header
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = message.info
        mask.data = message.data
        self._publisher.publish(mask)
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
