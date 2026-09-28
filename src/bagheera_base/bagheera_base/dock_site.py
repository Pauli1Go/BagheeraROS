"""Site data of the charging dock: its map pose and the undock manoeuvre.

Where the dock stands belongs to the site like the map, so it lives in
``maps/dock.yaml`` (mounted, not in git)::

    dock_pose: [x, y, yaw]        # map frame, yaw in rad
    undock:
      reverse_distance_m: 0.80    # straight back out of the dock
      turn_angle_deg: 90.0        # + left, - right, 0 = no turn

Every key is optional. Missing keys, or a missing file, fall back to the
example values in ``nav2_navigation.yaml`` (``docking_server.home_dock.pose``
and the ``bagheera_autonomy_dock_guard`` parameters). The launch files pass
the result to the docking server, pose persistence and the dock guard.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

SITE_FILE = "/bagheera_ws/maps/dock.yaml"


@dataclass(frozen=True)
class DockSite:
    pose: tuple[float, float, float]
    reverse_distance_m: float
    turn_angle_rad: float
    # Where the values came from, for the start-up log.
    source: str


def _finite(value, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def load_dock_site(nav2_navigation_config: str, site_file: str = SITE_FILE) -> DockSite:
    with open(nav2_navigation_config, encoding="utf-8") as handle:
        nav2 = yaml.safe_load(handle)
    docking = nav2["docking_server"]["ros__parameters"]
    guard = nav2["bagheera_autonomy_dock_guard"]["ros__parameters"]
    pose = tuple(float(value) for value in docking[docking["docks"][0]]["pose"])
    reverse = float(guard.get("reverse_distance_m", 0.80))
    turn = float(guard.get("turn_angle_rad", math.pi / 2.0))
    source = nav2_navigation_config

    path = Path(site_file)
    if path.is_file():
        site = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(site, dict):
            raise ValueError(f"{site_file} must contain a mapping")
        if "dock_pose" in site:
            values = site["dock_pose"]
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                raise ValueError(f"{site_file}: dock_pose must be [x, y, yaw]")
            pose = tuple(_finite(value, "dock_pose") for value in values)
        undock = site.get("undock") or {}
        if "reverse_distance_m" in undock:
            reverse = _finite(undock["reverse_distance_m"], "undock.reverse_distance_m")
        if "turn_angle_deg" in undock:
            turn = math.radians(_finite(undock["turn_angle_deg"], "undock.turn_angle_deg"))
        source = site_file

    if reverse <= 0.0:
        raise ValueError("undock reverse distance must be positive")
    if abs(turn) > math.pi:
        raise ValueError("undock turn angle must be within +-180 deg")
    return DockSite(pose, reverse, turn, source)


def dump_dock_site(site: DockSite) -> str:
    """YAML text for maps/dock.yaml (used by the setup tool)."""
    x, y, yaw = site.pose
    return (
        "# Charging dock of this site (see docs/docking.md).\n"
        f"dock_pose: [{x:.4f}, {y:.4f}, {yaw:.4f}]\n"
        "undock:\n"
        f"  reverse_distance_m: {site.reverse_distance_m:.2f}\n"
        "  # + left, - right, 0 = no turn (Nav2 turns onto its path)\n"
        f"  turn_angle_deg: {math.degrees(site.turn_angle_rad):.1f}\n"
    )
