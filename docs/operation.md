# Operation

Day-to-day use of a calibrated robot with a map: start, localize, send goals,
dock, patrol. All interaction happens through
[Foxglove](https://foxglove.dev) and a few ROS topics.

## Start and stop

```bash
docker compose up -d                     # start (also after every reboot)
docker compose logs -f bagheera-base     # follow the log
docker compose restart                   # after config/Python/launch changes
docker compose down                      # stop
```

The stack is ready when `ros2 lifecycle get /bt_navigator` reports `active`,
usually ~30 s after the start. Run ROS commands inside the container:

```bash
docker exec -it bagheera-base bash
```

## Connect Foxglove

Open a connection to `ws://<robot>:8765` (Foxglove WebSocket). A useful
layout:

| Panel | Content |
|---|---|
| 3D | fixed frame `map`; `/map`, `/scan`, robot model, `/global_costmap/costmap`, `/plan`, `/keepout_filter_mask_live`, `/patrol/waypoints`, `/dock_pose`, `/staging_pose` |
| Image | `/camera/h264` (starts the camera on demand) |
| Raw Messages | `/dock/status`, `/dock/sleep_state`, `/patrol/status`, `/battery/level` |
| Plot | `/battery/voltage` |
| Publish | `std_msgs/msg/Bool` `{"data": true}` for the trigger topics below |

## Set the initial pose

After a restart `bagheera_pose_persistence` restores the last pose (or the
dock pose while docked). Check that `/scan` lies on the walls of `/map`. If
it does not, for example because the robot was carried somewhere while off,
use the 3D panel's **2D pose estimate** tool (publishes `/initialpose`): click
the robot's position and drag in its heading direction.

## Send a navigation goal

Configure the 3D panel's **2D pose** tool to publish
`geometry_msgs/msg/PoseStamped` on `/goal_pose`, click the target and drag
the final heading. `bagheera_goal_pose_bridge` forwards it to Nav2's
`/navigate_to_pose` action.

- A goal whose oriented footprint would overlap a wall, unknown space or a
  keepout cell is rejected. The container log says where.
- If the robot is docked, the first goal wakes it and
  `bagheera_autonomy_dock_guard` reverses out of the dock and turns as set
  in `maps/dock.yaml` (default 0.80 m, 90° left) before Nav2 takes over.
- Autonomous speed is 0.32 m/s and 0.45 rad/s by default
  (`enable_higher_speeds`).
- When the path is blocked, Nav2 clears the costmaps and replans, then tries
  a 20 cm BackUp; after that the goal aborts.
- Taking over with the controller always wins over Nav2.

## Dock

Publish `{"data": true}` on `/dock/trigger` (`std_msgs/msg/Bool`), or from
a shell:

```bash
docker exec -it bagheera-base bash -lc \
  'source /opt/ros/kilted/setup.bash && ros2 topic pub --once /dock/trigger std_msgs/msg/Bool "{data: true}"'
```

The robot drives to the staging pose, switches the camera on, approaches using
the two AprilTags and drives the last 17 cm straight until the contacts
touch. Progress is on `/dock/status` (JSON). `/dock/cancel` or any controller
movement stops it. How it works and how to set it up: [docking.md](docking.md).

There is no separate undock command: the next autonomous goal leaves the dock
first.

## Dock sleep

3 s after docking without motion commands, the robot goes to sleep:
LiDAR, IMU polling, optical flow (LED off) and camera are off and the Nav2
navigation servers are paused. `/dock/sleep_state` shows `sleeping`.

It wakes on an autonomous goal, the controller's deadman button (if the
controller is enabled in `base.yaml`) or
`/dock/wake`. Waking takes about 7 s: LiDAR up, gyro bias re-measured (the
robot must stay still), EKF reset, AMCL anchored to the dock pose, Nav2
resumed. Only then does it drive. `/dock/sleep_request` sends a docked robot
to sleep at once.

## Battery

| Topic | Content |
|---|---|
| `/battery/voltage` | averaged, offset-corrected pack voltage |
| `/battery/percentage` | approximate state of charge from the voltage |
| `/battery/level` | `NORMAL`, `LOW` (~20 %), `CRITICAL` (~10 %), `FULL` |

Only the patrol reacts to the level. Normal goals do not return to the dock
by themselves. Thresholds and the measured voltage offset:
[config/base.md](config/base.md).

## Waypoint patrol

`bagheera_patrol` drives saved **paths**: waypoints (position and heading,
in order) plus a mode and whether the lap is closed. Each path is one file
`maps/paths/<name>.yaml` (site data, not in git) and has its own trigger:
publish `{"data": true}` on `/patrol/<name>`.

| Mode | What a start does |
|---|---|
| `once` | Drive the path once, then dock and stop. |
| `dock_cycle` | Dock after every lap, wait `dock_pause_s` (60 s), continue. A low battery charges until `FULL` instead. |
| `charge` | Loop until `/battery/level` is `LOW` (finish the current waypoint, then dock) or `CRITICAL` (cancel and dock at once). Charge until `FULL`, then continue with the next waypoint. |

A **closed** path drives back to its first waypoint at the end of each lap;
an **open** one ends the lap at its last waypoint (then docks in `once` and
`dock_cycle`).

`/patrol/cancel` stops the current goal or docking; the robot stays where it
is. A controller movement or a Foxglove goal cancels too. `cancel` and the
patrol's other topic names (`status`, `waypoints`, `reload`, …) cannot be
path names.

### Creating and editing paths

On the robot host start the path editor (it enters the container and sets up
ROS by itself; it does not move the robot):

```bash
~/BagheeraROS/tools/paths.sh
```

```text
Paths in /bagheera_ws/maps/paths:
   1  start_charge         charge      open      11 waypoints
   2  start_dock_cycle     dock_cycle  open      11 waypoints
[n] new  [e] edit  [s] show  [r] rename  [d] delete  [q] quit
```

- **new**: asks for the name, the mode (1 `once`, 2 `dock_cycle`, 3
  `charge`) and whether the path is closed, then records the waypoints.
- **edit**: add waypoints at the end, clear and record again, change the
  mode, switch open/closed, show.
- **rename** changes the trigger (`/patrol/<new name>`); **delete** asks
  first. Paths are picked by number or name.

For scripts the same works non-interactively
(`tools/paths.sh <command>`, or `ros2 run bagheera_base bagheera_paths
<command>` inside the container):

```bash
tools/paths.sh create lager --mode dock_cycle --closed   # new path, then record
tools/paths.sh edit lager                                # add waypoints at the end
tools/paths.sh edit lager --mode once --open --no-record # change settings only
tools/paths.sh edit lager --clear                        # record again from scratch
tools/paths.sh list
tools/paths.sh show lager
tools/paths.sh rename lager halle                        # trigger becomes /patrol/halle
tools/paths.sh delete halle
```

Recording waypoints (new path, edit → add, or `create`/`edit`):

1. Localize the robot; in Foxglove's 3D panel use the fixed/display frame
   `map`.
2. In the 3D panel's publish settings set the **Publish pose** topic to
   `/patrol/add_waypoint` (Foxglove keeps it in the layout).
3. Click-drag each waypoint in driving order: the click is the position, the
   drag direction the heading. Nothing is sent to Nav2 and AMCL stays put.
4. Each waypoint is checked with the robot's footprint against the map and
   the keepout mask; a rejected one prints why. Accepted waypoints print in
   the terminal, appear as yellow numbered arrows on `/patrol/edit_markers`
   and are saved at once.
5. `u` + Enter removes the last waypoint, `l` lists them, `q` (or Ctrl-C)
   finishes.

The patrol picks up new, changed, renamed and deleted paths within 5 s
(or at once on `/patrol/reload`); no restart. The path being driven is shown
on `/patrol/waypoints`.

### Behaviour while driving

- A waypoint Nav2 aborts is skipped; three skips in a row dock and stop the
  patrol. Aborts caused by late data (costmap timeout, TF error, planner
  failure) retry the same waypoint twice after 3 s first.
- In the dock, the patrol wakes the robot and lets the dock guard reverse out
  first (`UNDOCKING`); the first Nav2 goal follows once it is clear.
- Docking is retried up to `dock_attempts` times.
- `/patrol/status` carries JSON with state, waypoint, laps and counters.
- Every run writes a CSV (one row per event) to `test_logs/` on the host.

## Camera

Show `/camera/h264` in an Image panel. The camera starts when the first
viewer subscribes and stops 20 s after the last one leaves (colour H.264,
1080p, 15 FPS, hardware encoded). It cannot start while the robot sleeps in
the dock. `/camera/stream_enabled` (`true`) keeps it on without a viewer.

## Useful checks

```bash
ros2 topic echo /hardware_bridge/status --once
ros2 topic echo /hardware_bridge/emergency --once
ros2 topic echo /dock/sleep_state --once
ros2 topic echo /amcl_pose --once
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /docking_server
```

More tools: [diagnostics.md](diagnostics.md).
