#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/franka_ws/install/setup.bash
if [[ -f /workspace/ros_ws/install/setup.bash ]]; then
  source /workspace/ros_ws/install/setup.bash
fi

exec "$@"

