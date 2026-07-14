#!/usr/bin/env bash
set -euo pipefail

map_path="${1:-/home/rocket/maps/mekagangen.yaml}"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/laser_merger_ws/install/setup.bash" ]; then
  source "$HOME/laser_merger_ws/install/setup.bash"
fi
source "$HOME/rocket_ws/install/setup.bash"
if [ -f "$HOME/gr4_ws/install/setup.bash" ]; then
  source "$HOME/gr4_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-138}"

ros2 launch articubot_one nav.launch.py map:="$map_path"
