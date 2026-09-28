# `patrol.yaml`: waypoint patrol

`bagheera_patrol` drives a closed loop of waypoints for endurance and
docking tests. Usage: [operation.md](../operation.md#waypoint-patrol).

| Parameter | Value | Meaning |
|---|---|---|
| `waypoints` | flat list `[x, y, yaw, x, y, yaw, …]` | Map poses, driven in order and back to the first. **Site-specific**: the shipped list belongs to the author's map. Shown in Foxglove as `/patrol/waypoints`. |
| `dock_pause_s` | 60 s | `dock_cycle` mode: time in the dock between two laps. |
| `dock_attempts` | 3 | `/dock/trigger` attempts per docking (each includes the docking server's own retries) before the patrol stops. |
| `max_consecutive_failures` | 3 | Waypoints skipped in a row before the patrol docks and stops. |
| `max_transient_retries`, `retry_delay_s` | 2, 3 s | Aborts caused by late navigation data (costmap timeout, TF/extrapolation error, rejected goal, planner failure) retry the same waypoint this often, after this delay, before counting as a skip. Missing progress and collisions skip at once. |
| `waypoint_timeout_s` | 600 s | Safety limit per waypoint on top of Nav2's own abort. |
| `wake_timeout_s` | 90 s | Maximum wait for `/dock/sleep_state` = `awake` before leaving the dock. |
| `dock_start_timeout_s` | 5 s | Maximum wait for the docking server to accept a trigger. |
| `max_charge_s` | 21600 s (6 h) | Stop in the dock if `FULL` is not reached. |
| `log_dir` | `/bagheera_ws/test_logs` | One CSV per run with one row per event. Mounted to `./test_logs` on the host. |

## Collecting waypoints

1. Localize the robot and open the 3D panel in Foxglove.
2. Use the *2D pose estimate* tool to click each waypoint and read the pose
   from the published `/initialpose` message (`ros2 topic echo /initialpose`
   in the container). This moves AMCL briefly. Do it while docked:
   `bagheera_pose_persistence` then puts the pose back on the dock.
3. Check that every waypoint's oriented footprint is free. The goal bridge
   logs a rejection if you send one as a Foxglove goal.
