#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
mode="${1:-fake}"
if [[ "$mode" != fake && "$mode" != real ]]; then
  echo "Usage: $0 fake | real ROBOT_IP" >&2
  exit 2
fi
if [[ ! -t 0 ]]; then
  echo "Run directly in an interactive terminal" >&2
  exit 2
fi
running_containers="$(docker ps --filter ancestor=single-fr3-rviz:jazzy --format '{{.Names}}')"
if [[ -n "$running_containers" ]]; then
  echo "Stop the existing FR3 container before starting servo teleop:" >&2
  echo "$running_containers" >&2
  exit 2
fi
args=(omega7_fresh omega7_servo_teleop.launch.py use_rviz:=true)
if [[ "$mode" == real ]]; then
  if [[ -z "${2:-}" ]]; then
    echo "Real mode requires ROBOT_IP" >&2
    exit 2
  fi
  args+=(use_fake_hardware:=false "robot_ip:=$2")
else
  args+=(use_fake_hardware:=true)
fi
if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null || true
fi
docker compose --env-file .env -f docker/compose.yaml -f docker/omega7_fresh.compose.yaml \
  run --rm --interactive --tty single_fr3 \
  bash /workspace/ops/omega7_fresh_container.sh "${args[@]}"
