#!/usr/bin/env bash
# Patrol path editor, run on the robot host: interactive menu without
# arguments, or the bagheera_paths sub-commands (list, show, create, ...).
# The container's entrypoint sets up the ROS environment first.
exec docker exec -it bagheera-base /bagheera_entrypoint.sh \
  ros2 run bagheera_base bagheera_paths "$@"
