#!/usr/bin/env bash
set -euo pipefail

cd /workspace/ros_ws
colcon build --symlink-install --packages-select omega7_servo_controller omega7_fresh \
  --event-handlers console_direct+
set +u
source /workspace/ros_ws/install/setup.bash
set -u
exec ros2 run omega7_fresh keyboard "$@"
