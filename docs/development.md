# Development

## Repository layout

```text
compose.yaml                 one container, bind mounts, launch command
docker/                      Dockerfile (on the MowgliNext image) and driver patches
config/99-bagheera.rules     udev rule for /dev/mowgli
src/bagheera_base/           Python package: nodes, config/, launch/, urdf/,
                             behavior_trees/, test/
src/bagheera_docking/        C++ opennav_docking plugin (TagChargingDock)
src/bagheera_sensors/        C++ PMW3901, WT901 and normalizer components, gtests
tools/                       calibration, probes, diagnostics (not installed)
maps/                        site data, mounted into the container, not in git
test_logs/                   patrol CSVs and perf snapshots, not in git
legacy/docking/              former docking controller, reference only
```

Keep ROS-free logic in its own module (`*_math.py`, `*_logic.py`; in C++
`sensor_math.hpp`) so it can be tested without ROS. The node
module only wires topics, parameters and timers.

## Applying changes on the robot

The container runs the files from the checkout on the robot. What a change
needs:

| Changed | Needed | Time on a Pi 4 |
|---|---|---|
| `src/bagheera_base/config/*.yaml` | `docker compose restart` | ~30 s |
| Python nodes in `src/bagheera_base/bagheera_base/` | `docker compose restart` | ~30 s |
| `launch/`, `behavior_trees/` | `docker compose restart` | ~30 s |
| `compose.yaml` | `docker compose up -d` | ~30 s |
| New executable in `setup.py`, `urdf/` | `docker compose build && docker compose up -d` | ~1 min |
| C++ plugin `src/bagheera_docking/` | `docker compose build && docker compose up -d` | ~3 min |
| C++ sensor drivers `src/bagheera_sensors/` | `docker compose build && docker compose up -d` | ~5 min |
| `docker/Dockerfile`, apt packages, patches | `docker compose build && docker compose up -d` | long |

Why:

- `compose.yaml` mounts `config/`, the Python package, `launch/` and
  `behavior_trees/` read-only over the installed copies, so a restart picks
  up changes.
- A new executable needs a build because `ros2 run` looks it up in the
  installed package index; the module itself is mounted, its entry point is
  not.
- The Dockerfile builds the C++ plugin and the Python package in separate
  layers, so a Python-only rebuild reuses the compiled plugin.

If you develop on another computer, copy the changed files to the robot's
checkout (for example with `rsync`, without `--delete`, so `maps/` and
`test_logs/` on the robot stay untouched) and apply the step above.

The stack is ready when `ros2 lifecycle get /docking_server` (or
`/bt_navigator`) reports `active`.

## Tests

Pure-Python tests run anywhere with Python ≥ 3.11 and `pyyaml`, without
ROS (`numpy` and `opencv-python` additionally enable the tag-geometry tests):

```bash
PYTHONPATH=src/bagheera_base python3 -m unittest discover -s src/bagheera_base/test -v
```

Three test modules need ROS message packages (`test_dock_calibrate`,
`test_dock_final_approach`, `test_goal_footprint`) and fail to import
outside the container; `test_dock_sleep` skips itself there.

Full build and all tests in a throwaway container of the robot image,
without devices or network, so the running robot is not affected:

```bash
mkdir -p /tmp/bagheera_test && cp -r src /tmp/bagheera_test/
docker run --rm --network none -v /tmp/bagheera_test:/ws bagheera-ros:local bash -lc \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && cd /ws &&
   colcon build --merge-install --packages-select bagheera_docking bagheera_sensors bagheera_base &&
   colcon test --merge-install --packages-select bagheera_sensors && colcon test-result &&
   source install/setup.bash && cd src/bagheera_base && python3 -m pytest -q test'
```

`tools/check_nav2_bringup.sh` starts the Nav2 part with fake TF in such a
disposable container and checks lifecycle activation and BT parsing. No
goals are sent.

## Hardware smoke test after changes to drive code

1. Wheels off the floor, deadman not held; start the container.
2. Protocol v6 handshake and live `/hardware_bridge/status`, emergency, IMU
   and odometry topics.
3. Stop/lift/tilt inputs appear in `/hardware_bridge/emergency` and block
   commands.
4. Command forward briefly: wheel and encoder directions are right.
5. Release the deadman, unplug the controller, stop the teleop node: every
   case must stop the motors.
6. Only then test on the floor.

## Conventions

- Units SI, frames REP-103/REP-105, `base_link` at the axle.
- Parameters in YAML with a comment explaining **why** a value was chosen,
  not only what it is. The references in `docs/config/` mirror them; update
  both.
- Keep the four footprint copies identical (see
  [config/navigation.md](config/navigation.md#footprint)).
- The dock pose exists once, in `docking_server.home_dock.pose`.
- Never commit anything from `maps/`.
