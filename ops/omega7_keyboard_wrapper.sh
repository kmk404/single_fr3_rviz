#!/usr/bin/env bash
set -euo pipefail

restore_terminal() {
  stty sane 2>/dev/null || true
}

stop_launch() {
  restore_terminal
  if [[ -n "${launch_pid:-}" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM -- "-$launch_pid" 2>/dev/null || kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}

setsid ros2 launch "$@" &
launch_pid=$!
trap stop_launch EXIT INT TERM

if [[ ! -t 0 ]]; then
  echo "Omega.7 keyboard control requires an interactive terminal" >&2
  wait "$launch_pid"
  exit $?
fi

stty -icanon -echo min 1 time 0
printf 'Press SPACE to toggle Omega.7 motion; Ctrl-C to stop.\n'
while kill -0 "$launch_pid" 2>/dev/null; do
  key=''
  IFS= read -r -n 1 key || break
  if [[ "$key" == ' ' ]]; then
    ros2 service call /omega7_teleop/toggle_motion std_srvs/srv/Trigger '{}'
  fi
done

wait "$launch_pid"