#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 MAP_NAME_OR_ABSOLUTE_BASE_PATH"
  echo "Example: $0 mekagangen"
  echo "Example: $0 /home/rocket/maps/mekagangen"
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$HOME/rocket_ws/install/setup.bash"
if [ -f "$HOME/gr4_ws/install/setup.bash" ]; then
  source "$HOME/gr4_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-138}"

name="$1"
if [[ "$name" != /* ]]; then
  name="/home/rocket/maps/$name"
fi

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$name'}}"
