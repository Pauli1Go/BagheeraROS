# Development

The deployment image is pinned to the MowgliNext 1.1.0 ROS 2 image and its
protocol-v6 firmware. The image currently contains ROS 2 Kilted; Ubuntu on the
Raspberry Pi remains only the Docker host.

Build the complete overlay with:

```bash
docker compose build
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
