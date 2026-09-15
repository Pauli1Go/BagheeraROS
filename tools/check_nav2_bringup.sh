#!/usr/bin/env bash
# Run ONLY in a disposable container without robot devices or host networking.
# Fake TF permits lifecycle activation and BT parsing; no goals are submitted.
set -eo pipefail
source /opt/ros/kilted/setup.bash
source /bagheera_ws/install/setup.bash
export ROS_DOMAIN_ID=94
export ROS2CLI_DISABLE_DAEMON=1
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom >/tmp/test-map-tf.log 2>&1 &
map_tf_pid=$!
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link >/tmp/test-base-tf.log 2>&1 &
base_tf_pid=$!
ros2 launch bagheera_base navigation.launch.py >/tmp/test-nav2.log 2>&1 &
nav_pid=$!
trap 'kill "$nav_pid" "$map_tf_pid" "$base_tf_pid" 2>/dev/null || true' EXIT
for attempt in {1..45}; do
  if grep -q 'Managed nodes are active' /tmp/test-nav2.log; then break; fi
  if grep -qE 'Failed to bring up|Error loading XML|process has died' /tmp/test-nav2.log; then
    cat /tmp/test-nav2.log
    exit 1
  fi
  sleep 1
done
grep 'Managed nodes are active' /tmp/test-nav2.log
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /behavior_server behavior_plugins
ros2 action list -t
if ros2 action list | grep -qx /spin; then exit 1; fi
echo "BT parsed and navigation active in isolated container; no Spin action."
