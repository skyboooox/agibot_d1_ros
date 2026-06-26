#!/usr/bin/env bash
set -eo pipefail

echo "Setup agibot_d1_ros environment"
: "${ROS_DISTRO:=jazzy}"
: "${AGIBOT_D1_ROS2_ROOT:=$HOME/agibot_d1_ros}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -f "${AGIBOT_D1_ROS2_ROOT}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${AGIBOT_D1_ROS2_ROOT}/install/setup.bash"
fi

export AGIBOT_D1_LOCAL_IP="${AGIBOT_D1_LOCAL_IP:-192.168.168.100}"
export AGIBOT_D1_ROBOT_IP="${AGIBOT_D1_ROBOT_IP:-192.168.168.168}"
export AGIBOT_D1_LOCAL_PORT="${AGIBOT_D1_LOCAL_PORT:-43988}"
