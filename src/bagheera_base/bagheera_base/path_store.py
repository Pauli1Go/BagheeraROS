"""Saved patrol paths in maps/paths/<name>.yaml (site data, not in git).

One file per path::

    mode: dock_cycle        # once | dock_cycle | charge
    closed: true            # after the last waypoint drive back to the first
    waypoints:              # map x [m], y [m], yaw [rad], in driving order
      - [-2.002, 4.656, 1.604]
      - [-2.227, 5.788, -3.055]

``bagheera_paths`` (paths_cli.py) creates and edits them; bagheera_patrol
starts a path on ``/patrol/<name>``. The name is therefore a ROS topic token
and must not collide with the patrol's own topics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
import re

import yaml

PATHS_DIR = "/bagheera_ws/maps/paths"
MODES = ("once", "dock_cycle", "charge")
# Topics of bagheera_patrol under /patrol/ (plus a few spare words).
RESERVED_NAMES = frozenset({
    "cancel", "status", "waypoints", "reload", "add_waypoint", "edit_markers",
    "start", "stop", "all",
})
_NAME = re.compile(r"^[a-z][a-z0-9_]{0,39}$")

Waypoint = tuple[float, float, float]


@dataclass
class PatrolPath:
    name: str
    mode: str = "dock_cycle"
    closed: bool = True
    waypoints: list[Waypoint] = field(default_factory=list)

    def route(self) -> list[Waypoint]:
        """Waypoints of one lap; a closed path ends at its first waypoint again."""
        if self.closed and len(self.waypoints) > 1:
            return list(self.waypoints) + [self.waypoints[0]]
        return list(self.waypoints)


def validate_name(name: str) -> str:
    if not _NAME.match(name):
        raise ValueError(
            f"invalid path name '{name}': use a-z, 0-9 and _, starting with a letter "
            "(it becomes the topic /patrol/<name>)"
        )
    if name in RESERVED_NAMES:
        raise ValueError(f"'{name}' is reserved for a patrol topic; choose another name")
    return name


def validate_mode(mode: str) -> str:
    if mode not in MODES:
        raise ValueError(f"unknown mode '{mode}', expected one of {', '.join(MODES)}")
    return mode


def _file(directory: str | Path, name: str) -> Path:
    return Path(directory) / f"{validate_name(name)}.yaml"


def parse_path(name: str, data) -> PatrolPath:
    if not isinstance(data, dict):
        raise ValueError(f"{name}: the file must contain a mapping")
    waypoints: list[Waypoint] = []
    for index, value in enumerate(data.get("waypoints") or [], 1):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"{name}: waypoint {index} must be [x, y, yaw]")
        point = tuple(float(v) for v in value)
        if not all(math.isfinite(v) for v in point):
            raise ValueError(f"{name}: waypoint {index} is not finite")
        waypoints.append(point)
    closed = data.get("closed", True)
    if not isinstance(closed, bool):
        raise ValueError(f"{name}: closed must be true or false")
    return PatrolPath(
        name=validate_name(name),
        mode=validate_mode(str(data.get("mode", "dock_cycle"))),
        closed=closed,
        waypoints=waypoints,
    )


def dump_path(path: PatrolPath) -> str:
    lines = [
        f"# Patrol path '{path.name}': start with /patrol/{path.name} (docs/operation.md).",
        f"mode: {path.mode}",
        f"closed: {'true' if path.closed else 'false'}",
        "waypoints:" if path.waypoints else "waypoints: []",
    ]
    for number, (x, y, yaw) in enumerate(path.waypoints, 1):
        lines.append(f"  - [{x:.3f}, {y:.3f}, {yaw:.3f}]   # {number}")
    return "\n".join(lines) + "\n"


def load_path(directory: str | Path, name: str) -> PatrolPath:
    file = _file(directory, name)
    if not file.is_file():
        raise FileNotFoundError(f"no path '{name}' in {directory}")
    return parse_path(name, yaml.safe_load(file.read_text(encoding="utf-8")) or {})


def list_paths(directory: str | Path) -> tuple[list[PatrolPath], list[str]]:
    """All valid paths, sorted by name, and error messages for broken files."""
    paths, errors = [], []
    folder = Path(directory)
    if not folder.is_dir():
        return paths, errors
    for file in sorted(folder.glob("*.yaml")):
        try:
            paths.append(load_path(folder, file.stem))
        except (ValueError, yaml.YAMLError) as error:
            errors.append(f"{file.name}: {error}")
    return paths, errors


def save_path(directory: str | Path, path: PatrolPath) -> Path:
    """Write atomically, so the patrol never reads a half-written file."""
    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    file = _file(folder, path.name)
    validate_mode(path.mode)
    temporary = folder / f".{file.name}.{os.getpid()}.tmp"
    temporary.write_text(dump_path(path), encoding="utf-8")
    os.replace(temporary, file)
    return file


def delete_path(directory: str | Path, name: str) -> None:
    file = _file(directory, name)
    if not file.is_file():
        raise FileNotFoundError(f"no path '{name}' in {directory}")
    file.unlink()


def rename_path(directory: str | Path, old: str, new: str) -> None:
    source = _file(directory, old)
    target = _file(directory, new)
    if not source.is_file():
        raise FileNotFoundError(f"no path '{old}' in {directory}")
    if target.exists():
        raise FileExistsError(f"path '{new}' exists already")
    path = load_path(directory, old)
    path.name = new
    save_path(directory, path)
    source.unlink()


def directory_stamp(directory: str | Path) -> tuple:
    """Cheap change detector: names and modification times of the files."""
    folder = Path(directory)
    if not folder.is_dir():
        return ()
    return tuple(sorted((file.name, file.stat().st_mtime_ns) for file in folder.glob("*.yaml")))
