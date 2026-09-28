#!/usr/bin/env bash
# Read-only CPU/ROS load snapshot on the robot host (not in the container).
# Sends no commands to the robot; only observes. Usage:
#   tools/perf_snapshot.sh <label> [seconds]
# e.g. "sleep-before", "driving-after". Results land in
# test_logs/perf/<timestamp>_<label>/ next to this repository's compose.yaml.
set -uo pipefail

label="${1:?usage: $0 <label> [seconds]}"
seconds="${2:-60}"
container="bagheera-base"
root="$(cd "$(dirname "$0")/.." && pwd)"
out="$root/test_logs/perf/$(date +%Y%m%d_%H%M%S)_$label"
mkdir -p "$out"
since="$(date --iso-8601=seconds)"

ros() {
  docker exec "$container" bash -c \
    "source /opt/ros/kilted/setup.bash; source /bagheera_ws/install/setup.bash; $1"
}

echo "Snapshot '$label' for $seconds s -> $out"
{
  date --iso-8601=seconds
  uptime
  free -h
  for kind in cpu io memory; do echo "pressure $kind:"; cat "/proc/pressure/$kind"; done
  vcgencmd measure_temp
  vcgencmd get_throttled
  docker stats --no-stream "$container"
  echo "sleep_state: $(ros 'timeout 5 ros2 topic echo --once --field data /dock/sleep_state std_msgs/msg/String' 2>/dev/null | head -1)"
} >"$out/start.txt" 2>&1

# The samplers run in parallel over the same window.
vmstat 1 "$seconds" >"$out/vmstat.txt" 2>&1 &
pidstat -u -p ALL 1 "$seconds" >"$out/pidstat.txt" 2>&1 &
iostat -xz 1 "$seconds" >"$out/iostat.txt" 2>&1 &
wait

# Topic rates afterwards: `ros2 topic hz` itself costs CPU and would skew the
# process numbers above.
for topic in /tf /odometry/filtered /wheel_odom_raw /imu/data_raw /hardware_bridge/power \
             /scan /amcl_pose /cmd_vel_teleop /camera/camera_info \
             /navigate_to_pose/_action/feedback; do
  # SIGINT: `ros2 topic hz` ignores SIGTERM and hangs on silent topics.
  rate="$(ros "timeout -s INT -k 3 8 ros2 topic hz $topic" 2>/dev/null | grep -m1 'average rate' || true)"
  echo "$topic: ${rate:-no messages}"
done >"$out/topic_rates.txt"

{
  uptime
  for kind in cpu io memory; do echo "pressure $kind:"; cat "/proc/pressure/$kind"; done
  vcgencmd measure_temp
  vcgencmd get_throttled
  docker stats --no-stream "$container"
} >"$out/end.txt" 2>&1

docker logs --since "$since" "$container" >"$out/container.log" 2>&1
{
  echo "ERROR lines:                $(grep -c '\[ERROR\]' "$out/container.log")"
  echo "WARN lines:                 $(grep -c '\[WARN\]' "$out/container.log")"
  echo "TF extrapolation:           $(grep -c 'extrapolation' "$out/container.log")"
  echo "EKF missed update rate:     $(grep -c 'Failed to meet update rate' "$out/container.log")"
  echo "Nav2 control loop missed:   $(grep -c 'Control loop missed' "$out/container.log")"
  echo "Stale /scan buffer:         $(grep -c 'observation buffer' "$out/container.log")"
} >"$out/log_counts.txt"

# Mean CPU per process, highest first (pidstat's "Average:" lines).
awk '/^Average:/ && $NF != "Command" {printf "%7.2f %%  %s\n", $(NF-2), $NF}' \
  "$out/pidstat.txt" | sort -rn | head -30 >"$out/top_processes.txt"

cat "$out/top_processes.txt" "$out/log_counts.txt" "$out/topic_rates.txt"
echo "Done: $out"
