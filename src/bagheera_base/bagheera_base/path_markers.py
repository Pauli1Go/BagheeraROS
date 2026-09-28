"""Foxglove/RViz markers for a patrol path (patrol and bagheera_paths)."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# Blue line / orange arrows for the patrol, yellow while editing.
PATROL_COLORS = ((0.2, 0.6, 1.0, 0.8), (1.0, 0.5, 0.0, 1.0))
EDIT_COLORS = ((1.0, 0.85, 0.1, 0.9), (1.0, 0.85, 0.1, 1.0))


def path_markers(
    waypoints: list[tuple[float, float, float]],
    closed: bool,
    stamp: Time,
    colors=PATROL_COLORS,
) -> MarkerArray:
    """Numbered arrows plus a connecting line; clears the previous markers."""
    markers = MarkerArray()
    clear = Marker()
    clear.header.frame_id = "map"
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    if not waypoints:
        return markers
    line_color, arrow_color = colors
    line = Marker()
    line.header.frame_id = "map"
    line.header.stamp = stamp
    line.ns = "patrol_loop"
    line.type = Marker.LINE_STRIP
    line.scale.x = 0.03
    line.color.r, line.color.g, line.color.b, line.color.a = line_color
    line.pose.orientation.w = 1.0
    points = list(waypoints) + (list(waypoints[:1]) if closed and len(waypoints) > 1 else [])
    for x, y, _yaw in points:
        line.points.append(Point(x=x, y=y, z=0.05))
    markers.markers.append(line)
    for number, (x, y, yaw) in enumerate(waypoints, 1):
        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = stamp
        arrow.ns = "patrol_waypoints"
        arrow.id = number
        arrow.type = Marker.ARROW
        arrow.pose.position.x, arrow.pose.position.y = x, y
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.4, 0.06, 0.06
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = arrow_color
        markers.markers.append(arrow)
        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = stamp
        label.ns = "patrol_labels"
        label.id = number
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y = x, y
        label.pose.position.z = 0.3
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = str(number)
        markers.markers.append(label)
    return markers
