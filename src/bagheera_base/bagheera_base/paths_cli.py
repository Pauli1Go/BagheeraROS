"""bagheera_paths: create, edit and manage the saved patrol paths.

Without arguments it starts an interactive menu (on the robot host simply
``tools/paths.sh``). The sub-commands do the same non-interactively:

    ros2 run bagheera_base bagheera_paths list
    ros2 run bagheera_base bagheera_paths show NAME
    ros2 run bagheera_base bagheera_paths create NAME --mode {once,dock_cycle,charge} (--closed | --open)
    ros2 run bagheera_base bagheera_paths edit NAME [--mode MODE] [--closed | --open] [--no-record]
    ros2 run bagheera_base bagheera_paths rename OLD NEW
    ros2 run bagheera_base bagheera_paths delete NAME

``create`` and ``edit`` record waypoints: in Foxglove's 3D panel set the topic
of the *Publish pose* tool to ``/patrol/add_waypoint`` and click-drag each
waypoint (position and heading) in driving order. Each click is checked
against the map and the keepout mask with the robot's footprint, printed,
shown on ``/patrol/edit_markers`` and saved at once. Type ``u`` + Enter to
remove the last waypoint, ``q`` + Enter (or Ctrl-C) to finish. The patrol
picks up changes within 5 s; start a path with ``/patrol/<name>``.

Nothing here moves the robot.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import select
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
import yaml

from .goal_pose_bridge import DEFAULT_FOOTPRINT, blocked_footprint_cell, parse_footprint
from .path_markers import EDIT_COLORS, path_markers
from .path_store import (
    MODES,
    PATHS_DIR,
    PatrolPath,
    delete_path,
    list_paths,
    load_path,
    rename_path,
    save_path,
    validate_mode,
    validate_name,
)

ADD_TOPIC = "/patrol/add_waypoint"
MARKER_TOPIC = "/patrol/edit_markers"


def _yaw(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _describe(path: PatrolPath) -> str:
    shape = "closed" if path.closed else "open"
    return f"{path.name:<20} {path.mode:<11} {shape:<7} {len(path.waypoints):>3} waypoints"


def _print_waypoints(path: PatrolPath) -> None:
    for number, (x, y, yaw) in enumerate(path.waypoints, 1):
        print(f"  {number:>3}: x={x:8.3f}  y={y:8.3f}  yaw={math.degrees(yaw):7.1f} deg")
    if path.closed and len(path.waypoints) > 1:
        print("       then back to waypoint 1 (closed)")


def _footprint() -> list[tuple[float, float]]:
    config = Path(get_package_share_directory("bagheera_base")) / "config" / "nav2_navigation.yaml"
    try:
        data = yaml.safe_load(config.read_text(encoding="utf-8"))
        text = data["bagheera_goal_pose_bridge"]["ros__parameters"]["footprint"]
        return parse_footprint(str(text))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return parse_footprint(DEFAULT_FOOTPRINT)


class Recorder(Node):
    """Collects Foxglove poses into a path while the command line runs."""

    def __init__(self, directory: str, path: PatrolPath) -> None:
        super().__init__("bagheera_paths")
        self._directory = directory
        self.path = path
        self._footprint = _footprint()
        self._map: OccupancyGrid | None = None
        self._keepout: OccupancyGrid | None = None
        # Clicks are only accepted once the grids for the footprint check are
        # there (or the wait for them timed out).
        self.ready = False
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._markers = self.create_publisher(MarkerArray, MARKER_TOPIC, latched)
        self._reload = self.create_publisher(Bool, "/patrol/reload", 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(
            OccupancyGrid, "/keepout_filter_mask_live", self._on_keepout, latched
        )
        self.create_subscription(PoseStamped, ADD_TOPIC, self._on_pose, 10)
        self.publish_markers()

    def wait_for_grids(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while (self._map is None or self._keepout is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [name for name, grid in (("map", self._map), ("keepout mask", self._keepout))
                   if grid is None]
        if missing:
            print(f"  warning: no {' and '.join(missing)} received; waypoints are not "
                  "checked against it", flush=True)
        self.ready = True

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _on_keepout(self, message: OccupancyGrid) -> None:
        self._keepout = message

    def _on_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id not in ("map", "/map"):
            print(f"  rejected: pose in frame '{message.header.frame_id}', use the map frame",
                  flush=True)
            return
        x = message.pose.position.x
        y = message.pose.position.y
        yaw = _yaw(message.pose.orientation)
        if not self.ready:
            print("  rejected: map and keepout mask not received yet, click again", flush=True)
            return
        for name, grid, threshold in (("wall", self._map, 65), ("keepout zone", self._keepout, 50)):
            if grid is None:
                print(f"  warning: no {name} data, not checked", flush=True)
                continue
            hit = blocked_footprint_cell(grid, self._footprint, x, y, yaw, threshold)
            if hit is not None:
                print(
                    f"  rejected ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg): the robot "
                    f"would overlap a {name}/unknown cell at ({hit[0]:.2f}, {hit[1]:.2f})",
                    flush=True,
                )
                return
        self.path.waypoints.append((x, y, yaw))
        self.save()
        print(
            f"  + {len(self.path.waypoints):>3}: x={x:8.3f}  y={y:8.3f}  "
            f"yaw={math.degrees(yaw):7.1f} deg",
            flush=True,
        )

    def undo(self) -> None:
        if not self.path.waypoints:
            print("  nothing to remove", flush=True)
            return
        x, y, _yaw_ = self.path.waypoints.pop()
        self.save()
        print(f"  - removed waypoint {len(self.path.waypoints) + 1} ({x:.2f}, {y:.2f})",
              flush=True)

    def save(self) -> None:
        save_path(self._directory, self.path)
        self.publish_markers()
        self._reload.publish(Bool(data=True))

    def publish_markers(self, clear: bool = False) -> None:
        waypoints = [] if clear else self.path.waypoints
        self._markers.publish(path_markers(
            waypoints, self.path.closed, self.get_clock().now().to_msg(), EDIT_COLORS
        ))


_pending = ""


def _read_line(timeout: float | None) -> str | None:
    """One line from stdin, None after timeout; EOFError at end of input.

    The menu and the recording loop both read through this one buffer: a
    reader thread would keep consuming input after a recording, and mixing
    select() with Python's buffered stdin can hide lines in that buffer.
    """
    global _pending
    deadline = None if timeout is None else time.monotonic() + timeout
    while "\n" not in _pending:
        remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
        ready, _, _ = select.select([sys.stdin.fileno()], [], [], remaining)
        if not ready:
            return None
        chunk = os.read(sys.stdin.fileno(), 1024)
        if not chunk:
            if _pending:
                line, _pending = _pending, ""
                return line
            raise EOFError
        _pending += chunk.decode(errors="replace")
    line, _pending = _pending.split("\n", 1)
    return line


def _poll_line() -> str | None:
    """A typed command if one is waiting, "q" at end of input, else None."""
    try:
        line = _read_line(0.0)
    except EOFError:
        return "q"
    return None if line is None else line.strip().lower()


def _record(directory: str, path: PatrolPath) -> None:
    rclpy.init()
    node = Recorder(directory, path)
    print("Waiting for the map and the keepout mask ...", flush=True)
    node.wait_for_grids(10.0)
    print(
        f"\nRecording '{path.name}' ({path.mode}, {'closed' if path.closed else 'open'}).\n"
        f"In Foxglove's 3D panel set the Publish pose topic to {ADD_TOPIC} and\n"
        "click-drag each waypoint in driving order (arrow = heading).\n"
        f"Markers: {MARKER_TOPIC}. Commands: u + Enter = remove last, "
        "l = list, q = finish.",
        flush=True,
    )
    if path.waypoints:
        _print_waypoints(path)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            command = _poll_line()
            if command is None:
                continue
            if command in ("q", "quit", "exit"):
                break
            if command in ("u", "undo"):
                node.undo()
            elif command in ("l", "list"):
                _print_waypoints(node.path)
            elif command:
                print("  commands: u = remove last, l = list, q = finish", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_markers(clear=True)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.try_shutdown()
    count = len(path.waypoints)
    print(f"\nSaved '{path.name}' with {count} waypoints. Start it with:", flush=True)
    print(f"  ros2 topic pub --once /patrol/{path.name} std_msgs/msg/Bool '{{data: true}}'",
          flush=True)
    if count == 0:
        print("  (no waypoints yet: the patrol refuses to start it)", flush=True)


MODE_HELP = {
    "once": "drive the path once, then dock and stop",
    "dock_cycle": "dock after every lap, pause, continue",
    "charge": "loop until the battery is low, charge, continue",
}


def _ask(prompt: str) -> str:
    print(prompt, end="", flush=True)
    try:
        return (_read_line(None) or "").strip()
    except EOFError:
        print()
        raise SystemExit(0) from None


def _ask_new_name(directory: str) -> str | None:
    while True:
        name = _ask("Name (a-z, 0-9, _; empty = back): ").lower()
        if not name:
            return None
        try:
            validate_name(name)
            load_path(directory, name)
        except ValueError as error:
            print(f"  {error}")
            continue
        except FileNotFoundError:
            return name
        print(f"  '{name}' exists already")


def _ask_mode(current: str | None = None) -> str:
    for number, mode in enumerate(MODES, 1):
        marker = "  (current)" if mode == current else ""
        print(f"  {number}  {mode:<11} {MODE_HELP[mode]}{marker}")
    while True:
        answer = _ask(f"Mode [1-{len(MODES)}]{' (Enter = keep)' if current else ''}: ")
        if not answer and current:
            return current
        if answer.isdigit() and 1 <= int(answer) <= len(MODES):
            return MODES[int(answer) - 1]
        if answer in MODES:
            return answer


def _ask_closed(current: bool | None = None) -> bool:
    hint = {None: "y/n", True: "Y/n", False: "y/N"}[current]
    while True:
        answer = _ask(f"Closed: drive back to waypoint 1 after the last one? [{hint}]: ").lower()
        if not answer and current is not None:
            return current
        if answer in ("y", "yes", "j", "ja"):
            return True
        if answer in ("n", "no", "nein"):
            return False


def _pick(paths: list[PatrolPath]) -> PatrolPath | None:
    if not paths:
        print("  no paths yet")
        return None
    answer = _ask("Which path (number or name, empty = back): ").lower()
    if answer.isdigit() and 1 <= int(answer) <= len(paths):
        return paths[int(answer) - 1]
    for path in paths:
        if path.name == answer:
            return path
    if answer:
        print("  no such path")
    return None


def _edit_menu(directory: str, path: PatrolPath) -> None:
    while True:
        print(f"\n{_describe(path)}   trigger /patrol/{path.name}")
        print("  [a] add waypoints  [c] clear and record again  [m] mode  "
              "[o] open/closed  [s] show  [b] back")
        choice = _ask("> ").lower()
        if choice in ("", "b"):
            return
        if choice == "a":
            _record(directory, path)
        elif choice == "c":
            if _ask(f"Remove all {len(path.waypoints)} waypoints? [y/N]: ").lower() in ("y", "j"):
                path.waypoints = []
                save_path(directory, path)
                _record(directory, path)
        elif choice == "m":
            path.mode = _ask_mode(path.mode)
            save_path(directory, path)
        elif choice == "o":
            path.closed = _ask_closed(path.closed)
            save_path(directory, path)
        elif choice == "s":
            _print_waypoints(path)


def _menu(directory: str) -> None:
    print("Bagheera patrol paths: start a path with /patrol/<name>, stop with "
          "/patrol/cancel. Nothing here moves the robot.")
    while True:
        paths, errors = list_paths(directory)
        print(f"\nPaths in {directory}:")
        if not paths:
            print("  (none)")
        for number, path in enumerate(paths, 1):
            print(f"  {number:>2}  {_describe(path)}")
        for error in errors:
            print(f"  broken file: {error}")
        print("[n] new  [e] edit  [s] show  [r] rename  [d] delete  [q] quit")
        choice = _ask("> ").lower()
        try:
            if choice in ("q", "quit", "exit"):
                return
            if choice == "n":
                name = _ask_new_name(directory)
                if name is None:
                    continue
                path = PatrolPath(name, _ask_mode(), _ask_closed(), [])
                save_path(directory, path)
                print(f"Created {_describe(path).strip()}; trigger /patrol/{name}")
                _record(directory, path)
            elif choice == "e":
                path = _pick(paths)
                if path is not None:
                    _edit_menu(directory, path)
            elif choice == "s":
                path = _pick(paths)
                if path is not None:
                    print(_describe(path))
                    _print_waypoints(path)
            elif choice == "r":
                path = _pick(paths)
                if path is not None:
                    new = _ask_new_name(directory)
                    if new is not None:
                        rename_path(directory, path.name, new)
                        print(f"Renamed: trigger is now /patrol/{new}")
            elif choice == "d":
                path = _pick(paths)
                if path is not None and _ask(
                        f"Delete '{path.name}'? [y/N]: ").lower() in ("y", "yes", "j", "ja"):
                    delete_path(directory, path.name)
                    print(f"Deleted '{path.name}'.")
        except (FileNotFoundError, FileExistsError, ValueError) as error:
            print(f"  {error}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        prog="bagheera_paths", description="Create, edit and manage saved patrol paths."
    )
    parser.add_argument("--dir", default=PATHS_DIR, help=f"paths folder (default {PATHS_DIR})")
    commands = parser.add_subparsers(dest="command")
    commands.add_parser("list", help="list all paths")
    show = commands.add_parser("show", help="print a path's waypoints")
    show.add_argument("name")
    for command in ("create", "edit"):
        sub = commands.add_parser(
            command, help="new path, then record waypoints" if command == "create"
            else "change a path and/or add waypoints")
        sub.add_argument("name")
        sub.add_argument("--mode", choices=MODES, required=command == "create",
                         help="once: one lap then dock; dock_cycle: dock after every lap; "
                              "charge: loop until the battery is low, charge, continue")
        shape = sub.add_mutually_exclusive_group(required=command == "create")
        shape.add_argument("--closed", dest="closed", action="store_true", default=None,
                           help="after the last waypoint drive back to the first")
        shape.add_argument("--open", dest="closed", action="store_false",
                           help="the lap ends at the last waypoint")
        sub.add_argument("--no-record", action="store_true",
                         help="only save the settings, do not wait for waypoints")
        if command == "edit":
            sub.add_argument("--clear", action="store_true",
                             help="remove all waypoints before recording")
    rename = commands.add_parser("rename", help="rename a path (its trigger changes)")
    rename.add_argument("old")
    rename.add_argument("new")
    delete = commands.add_parser("delete", help="delete a path")
    delete.add_argument("name")
    delete.add_argument("--yes", action="store_true", help="do not ask")
    args = parser.parse_args(argv)
    directory = args.dir

    try:
        if args.command is None:
            _menu(directory)
        elif args.command == "list":
            paths, errors = list_paths(directory)
            if not paths:
                print(f"No paths in {directory}. Create one with: "
                      "ros2 run bagheera_base bagheera_paths create NAME --mode ... --closed")
            for path in paths:
                print(f"{_describe(path)}   trigger /patrol/{path.name}")
            for error in errors:
                print(f"broken file: {error}")
        elif args.command == "show":
            path = load_path(directory, args.name)
            print(_describe(path))
            _print_waypoints(path)
        elif args.command == "create":
            validate_name(args.name)
            try:
                load_path(directory, args.name)
            except FileNotFoundError:
                pass
            else:
                raise SystemExit(f"path '{args.name}' exists already; use edit")
            path = PatrolPath(args.name, validate_mode(args.mode), args.closed, [])
            save_path(directory, path)
            print(f"Created {_describe(path).strip()}; trigger /patrol/{path.name}")
            if not args.no_record:
                _record(directory, path)
        elif args.command == "edit":
            path = load_path(directory, args.name)
            if args.mode is not None:
                path.mode = validate_mode(args.mode)
            if args.closed is not None:
                path.closed = args.closed
            if args.clear:
                path.waypoints = []
            save_path(directory, path)
            print(_describe(path))
            if not args.no_record:
                _record(directory, path)
        elif args.command == "rename":
            rename_path(directory, args.old, args.new)
            print(f"Renamed: trigger is now /patrol/{args.new}")
        elif args.command == "delete":
            load_path(directory, args.name)
            if not args.yes and _ask(f"Delete path '{args.name}'? [y/N]: ").lower() \
                    not in ("y", "yes", "j", "ja"):
                print("Nothing deleted.")
                return
            delete_path(directory, args.name)
            print(f"Deleted '{args.name}'.")
    except (FileNotFoundError, FileExistsError, ValueError) as error:
        raise SystemExit(str(error)) from None


if __name__ == "__main__":
    main()
