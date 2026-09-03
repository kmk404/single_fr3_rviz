#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null
fi

docker compose --env-file .env -f docker/compose.yaml run --rm single_fr3 \
  ros2 launch single_fr3_bringup single_fr3_rviz.launch.py \
  use_fake_hardware:=true use_rviz:=true
