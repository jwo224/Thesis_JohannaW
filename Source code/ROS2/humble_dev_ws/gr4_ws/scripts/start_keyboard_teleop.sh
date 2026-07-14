#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/rocket_ws/install/setup.bash"
if [ -f "$HOME/gr4_ws/install/setup.bash" ]; then
  source "$HOME/gr4_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-138}"

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=false \
  -p speed:=0.12 \
  -p turn:=0.5
