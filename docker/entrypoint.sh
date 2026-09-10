#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/franka_ws/install/setup.bash

# The SDK remains outside the repository. Extract its read-only archive into the
# ephemeral container so the teleop package can load libdhd with ctypes.
sdk_archive="${FORCE_DIMENSION_SDK_ARCHIVE:-/opt/forcedimension_sdk/sdk-3.17.7-linux-x86_64-gcc.tar.gz}"
sdk_runtime_root="/tmp/forcedimension_sdk"
if [[ -f "$sdk_archive" ]]; then
  mkdir -p "$sdk_runtime_root"
  tar -xzf "$sdk_archive" -C "$sdk_runtime_root" --strip-components=1
  export OMEGA7_SDK_LIBRARY="$sdk_runtime_root/lib/release/lin-x86_64-gcc/libdhd.so.3.17.7"
  export LD_LIBRARY_PATH="$(dirname "$OMEGA7_SDK_LIBRARY"):${LD_LIBRARY_PATH:-}"
fi

if [[ -f /workspace/ros_ws/install/setup.bash ]]; then
  source /workspace/ros_ws/install/setup.bash
fi

exec "$@"
