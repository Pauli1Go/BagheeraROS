#!/bin/bash
set -e

source /opt/ros/kilted/setup.bash

if [ -f /opt/ublox_msgs/setup.bash ]; then
  source /opt/ublox_msgs/setup.bash
fi

source /ros2_ws/install/setup.bash
source /bagheera_ws/install/setup.bash

exec "$@"
