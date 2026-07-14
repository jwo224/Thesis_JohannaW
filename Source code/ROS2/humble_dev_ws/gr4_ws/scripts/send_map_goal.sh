#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "Usage: $0 X Y YAW_RADIANS"
  echo "Example: $0 1.8 0.8 1.57"
  exit 2
fi

x="$1"
y="$2"
yaw="$3"

source /opt/ros/humble/setup.bash
source "$HOME/rocket_ws/install/setup.bash"
if [ -f "$HOME/gr4_ws/install/setup.bash" ]; then
  source "$HOME/gr4_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-138}"

goal_yaml="$(python3 - "$x" "$y" "$yaw" <<'PY'
import math
import sys

x = float(sys.argv[1])
y = float(sys.argv[2])
yaw = float(sys.argv[3])
qz = math.sin(yaw / 2.0)
qw = math.cos(yaw / 2.0)

print(
    "{header: {frame_id: 'map'}, pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
    "orientation: {z: %.6f, w: %.6f}}}" % (x, y, qz, qw)
)
PY
)"

ros2 topic pub --once /goal_pose_raw geometry_msgs/msg/PoseStamped "$goal_yaml"
