#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if [[ ! -f .env ]]; then
  cp docker/.env.example .env
  sed -i "s/^USER_UID=.*/USER_UID=$(id -u)/" .env
  sed -i "s/^USER_GID=.*/USER_GID=$(id -g)/" .env
fi

mkdir -p reports
docker compose --env-file .env -f docker/compose.yaml build 2>&1 | tee reports/docker_build.log
docker compose --env-file .env -f docker/compose.yaml run --rm single_fr3 \
  bash -lc 'cd /workspace/ros_ws && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /workspace/reports/colcon_build.log'
