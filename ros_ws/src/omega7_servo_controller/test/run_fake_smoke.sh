#!/usr/bin/env bash
set -euo pipefail

# Keep fake controller names and commands isolated from any real FR3 stack
# already running on the host network.
export ROS_DOMAIN_ID="${OMEGA7_TEST_ROS_DOMAIN_ID:-171}"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /workspace/ros_ws
colcon build --symlink-install --packages-select omega7_servo_controller \
  --event-handlers console_direct+ >/tmp/omega7_servo_build.log
set +u
source install/setup.bash
set -u
ros2 launch omega7_servo_controller servo_control.launch.py \
  use_fake_hardware:=true >/tmp/omega7_servo_fake_launch.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  sleep 2
  kill -TERM "$launch_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 5
if ! python3 src/omega7_servo_controller/test/fake_servo_smoke.py; then
  tail -80 /tmp/omega7_servo_fake_launch.log
  exit 1
fi
