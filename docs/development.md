# Development

The deployment image is pinned to the MowgliNext 1.1.0 ROS 2 image and its
protocol-v6 firmware. The image currently contains ROS 2 Kilted; Ubuntu on the
Raspberry Pi remains only the Docker host.

Build the complete overlay with:

```bash
docker compose build
```

## Deploying changes to the robot

`~/BagheeraROS` on the Pi is a plain copy of this repository (not a git
checkout). Copy changed files with `rsync` from the workstation, then pick the
cheapest step that covers the change. Do not use `rsync --delete`: the Pi keeps
local files such as `maps/` recordings.

Reach the Pi with `ssh -4 ubuntu@bagheera.local`; without `-4` the name can
resolve to an unreachable IPv6 link-local address.

| Changed | What the robot needs | Time |
|---|---|---|
| `src/bagheera_base/config/*.yaml` | `docker compose restart` | ~30 s |
| Python nodes in `src/bagheera_base/bagheera_base/` | `docker compose restart` | ~30 s |
| `src/bagheera_base/launch/`, `behavior_trees/` | `docker compose restart` | ~30 s |
| `compose.yaml` (mounts, environment) | `docker compose up -d` | ~30 s |
| New executable in `setup.py`, `urdf/` (not mounted) | `docker compose build` + `docker compose up -d` | ~1 min |
| C++ dock plugin `src/bagheera_docking/` | `docker compose build` + `docker compose up -d` | ~3 min |
| `docker/Dockerfile`, apt packages | `docker compose build` + `docker compose up -d` | longer |

Why this works:

- **Mounted, no build:** `compose.yaml` mounts `config`, the Python package,
  `launch` and `behavior_trees` of `bagheera_base` read-only over the installed
  copies. The container runs the files from the Pi's checkout, so a restart
  picks up the change.
- **Partial build:** the Dockerfile builds `bagheera_docking` (C++) and
  `bagheera_base` (Python) in separate layers. A Python-only change reuses the
  cached plugin layer; only the C++ package takes ~2 min on the Pi.
- **Full build:** anything before those layers (base image, apt, YDLidar SDK,
  camera_ros) invalidates everything after it.

Example for a Python/config change:

```bash
rsync -a -e "ssh -4" --exclude __pycache__ --exclude .pytest_cache src/ ubuntu@bagheera.local:BagheeraROS/src/
ssh -4 ubuntu@bagheera.local 'cd ~/BagheeraROS && docker compose restart'
```

and for the C++ plugin or a new executable:

```bash
rsync -a -e "ssh -4" --exclude __pycache__ --exclude .pytest_cache src/ ubuntu@bagheera.local:BagheeraROS/src/
ssh -4 ubuntu@bagheera.local 'cd ~/BagheeraROS && docker compose build && docker compose up -d'
```

The stack is ready when `ros2 lifecycle get /docking_server` (or
`/bt_navigator`) inside the container reports `active`, usually ~30 s after the
restart.

A new executable only appears after a build, because `ros2 run` looks it up in
the installed package index; its Python module is mounted, but its entry point
is not.

To build and test without touching the running robot, use a throwaway
container of the same image with the sources copied to `/tmp`:

```bash
docker run --rm --network none -v /tmp/bagheera_test:/ws bagheera-ros:local bash -lc \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && cd /ws &&
   colcon build --merge-install --packages-select bagheera_docking bagheera_base &&
   source install/setup.bash && cd src/bagheera_base && python3 -m pytest -q test'
```

Pure controller mapping tests remain independent of ROS:

```bash
PYTHONPATH=src/bagheera_base \
  python3.11 -m unittest discover -s src/bagheera_base/test -v
```

Hardware smoke-test order:

1. Start the container with the wheels clear of the floor and do not hold the
   controller deadman.
2. Confirm protocol v6 handshake and live status, emergency, IMU and odometry
   topics.
3. Confirm that physical stop/lift/tilt inputs appear in the firmware emergency
   message and block commands.
4. Briefly command forward; verify both wheel and encoder directions.
5. Release the deadman, disconnect the controller and stop the teleop process
   separately; every case must stop motion.
6. Only then test on the floor and calibrate `ticks_per_meter`, `wheel_track`
   and drive/yaw PID parameters.

Do not run the old `bagheera_base_driver` against MowgliNext firmware. It speaks
the retired `MW` protocol and is retained only to document the previous PoC.
