#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
robot_ip="${1:-}"

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null
fi

launch_args=(
  ros2 launch single_fr3_bringup single_fr3_rviz.launch.py
  use_fake_hardware:=false
  robot_config:=/workspace/config/robot.yaml
  use_rviz:=true
)
if [[ -n "$robot_ip" ]]; then
  launch_args+=("robot_ip:=$robot_ip")
fi

docker compose --env-file .env -f docker/compose.yaml run --rm single_fr3 \
  "${launch_args[@]}"
