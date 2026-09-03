#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

python3 - <<'PY'
from pathlib import Path

for path in Path("ros_ws/src/single_fr3_bringup/launch").glob("*.py"):
    compile(path.read_text(encoding="utf-8"), str(path), "exec")
PY

expected='fr3_joint1 fr3_joint2 fr3_joint3 fr3_joint4 fr3_joint5 fr3_joint6 fr3_joint7'
actual="$(sed -n 's/.*name="\(fr3_joint[1-7]\)".*/\1/p' ros_ws/src/single_fr3_moveit_config/config/fr3.srdf | sort -u | tr '\n' ' ' | sed 's/ $//')"
[[ "$actual" == "$expected" ]]

if rg -n -i '(gello|manus|pico|vive|wuji|o30i|g20|left_fr3|right_fr3|udp[^[:alnum:]]*556[01])' \
  ros_ws/src config docker; then
  echo 'Legacy naming or port reference found' >&2
  exit 1
fi

grep -q '^FROM ros:jazzy-ros-base-noble$' docker/Dockerfile
grep -q 'type: FollowJointTrajectory' ros_ws/src/single_fr3_moveit_config/config/moveit_controllers.yaml
echo 'STATIC_TESTS=PASS'
