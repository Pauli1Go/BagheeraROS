# `patrol.yaml`: waypoint patrol

`bagheera_patrol` drives saved paths for endurance and docking tests. The
paths themselves are site data in `maps/paths/<name>.yaml`, made with the
path editor `tools/paths.sh` (`bagheera_paths`); this file only sets the
behaviour. Usage and the path editor:
[operation.md](../operation.md#waypoint-patrol).

| Parameter | Value | Meaning |
|---|---|---|
| `paths_dir` | `/bagheera_ws/maps/paths` | Folder of the path files (`maps/paths/` on the host). Each file becomes the trigger `/patrol/<name>`; re-read within 5 s of a change. |
| `dock_pause_s` | 60 s | `dock_cycle` mode: time in the dock between two laps. |
| `dock_attempts` | 3 | `/dock/trigger` attempts per docking (each includes the docking server's own retries) before the patrol stops. |
| `max_consecutive_failures` | 3 | Waypoints skipped in a row before the patrol docks and stops. |
| `max_transient_retries`, `retry_delay_s` | 2, 3 s | Aborts caused by late navigation data (costmap timeout, TF/extrapolation error, rejected goal, planner failure) retry the same waypoint this often, after this delay, before counting as a skip. Missing progress and collisions skip at once. |
| `waypoint_timeout_s` | 600 s | Safety limit per waypoint on top of Nav2's own abort. |
| `wake_timeout_s` | 90 s | Maximum wait for `/dock/sleep_state` = `awake` before leaving the dock. |
| `undock_timeout_s` | 60 s | When docked, the patrol asks the dock guard to reverse out (`/dock/undock`) and sends the first goal only after `/dock/guard_state` is `CLEAR`. Fails after this time. |
| `dock_start_timeout_s` | 5 s | Maximum wait for the docking server to accept a trigger. |
| `max_charge_s` | 21600 s (6 h) | Stop in the dock if `FULL` is not reached. |
| `log_dir` | `/bagheera_ws/test_logs` | One CSV per run with one row per event. Mounted to `./test_logs` on the host. |

## Path files

`tools/paths.sh` writes them; editing by hand works as well (the patrol
re-reads the folder within 5 s):

```yaml
# maps/paths/lager.yaml, started with /patrol/lager
mode: dock_cycle        # once | dock_cycle | charge
closed: true            # true: back to waypoint 1 at the end of each lap
waypoints:              # map x [m], y [m], yaw [rad], in driving order
  - [-2.002, 4.656, 1.604]   # 1
  - [-2.227, 5.788, -3.055]  # 2
```

Names: lower-case letters, digits and `_`, starting with a letter (they
become topic names). `cancel`, `status`, `waypoints`, `reload`,
`add_waypoint`, `edit_markers`, `start`, `stop` and `all` are reserved.
