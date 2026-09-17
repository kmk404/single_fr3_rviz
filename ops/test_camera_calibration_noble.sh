#!/usr/bin/env bash
# Test only camera calibration in Ubuntu 24.04/Jazzy, without host devices/ROS traffic.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
docker run --rm \
  --env ROS_LOCALHOST_ONLY=1 --env PYTHONNOUSERSITE=1 \
  --mount "type=bind,src=${repo_root}/ros_ws/src/camera_extrinsic_calibration,dst=/source,readonly" \
  --entrypoint /bin/bash ros:jazzy-ros-base-noble -c '
set -eo pipefail
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  python3-opencv python3-numpy python3-scipy python3-yaml python3-pytest \
  python3-colcon-common-extensions ros-jazzy-cv-bridge ros-jazzy-std-srvs
source /opt/ros/jazzy/setup.bash
python3 -c "import sys, cv2, numpy, scipy; print(sys.version); print(\"OpenCV\", cv2.__version__, \"NumPy\", numpy.__version__, \"SciPy\", scipy.__version__)"
mkdir -p /tmp/calibration_ws/src
cp -a /source /tmp/calibration_ws/src/camera_extrinsic_calibration
cd /tmp/calibration_ws
colcon build --symlink-install --packages-select camera_extrinsic_calibration
source install/setup.bash
colcon test --packages-select camera_extrinsic_calibration --event-handlers console_direct+
colcon test-result --verbose
ros2 launch camera_extrinsic_calibration camera_extrinsic_calibration.launch.py --show-args
'
