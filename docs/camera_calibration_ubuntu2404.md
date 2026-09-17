# Ubuntu 24.04 工控机部署：相机外参标定包

本说明只涉及 `camera_extrinsic_calibration`，不启动或重构机器人控制模块。
推荐环境为 Ubuntu 24.04 x86_64、ROS 2 Jazzy、系统 Python 3.12 与 apt OpenCV 4.6。
Ubuntu 22.04/Humble 的 build/install 产物不能直接复制到这里使用，必须重新构建。

## 已完成的兼容性检查（2026-09-16）

在 Ubuntu 24.04.4 x86_64 / ROS 2 Jazzy 的隔离容器中实测：

| 项目 | 结果 |
| --- | --- |
| Python | 3.12.3 |
| OpenCV / NumPy / SciPy | 4.6.0 / 1.26.4 / 1.11.4 |
| colcon build | 通过 |
| 完整测试 | 45 通过，0 失败，0 跳过 |
| 安装后的 launch、参数读取、recollect ROS 服务 | 通过 |
| 无相机输入时禁止保存 | 通过 |

发现并修复：原仓库 Dockerfile 缺少 OpenCV、SciPy、cv_bridge 等标定运行依赖；
package.xml 补全直接使用的 NumPy、launch、launch_ros 声明。
OpenCV 4.6 的旧版 ArUco API 已由代码的兼容分支及实际测试覆盖。
容器验证的是用户空间软件兼容性，尚未连接目标工控机和相机。

## 传输当前修改

本次实现仍在开发机本地工作区，尚未提交/推送；工控机仅执行 git pull 不会获得这些修改。
可以先按项目流程提交推送，也可直接同步本次包源码（将占位地址、路径替换为实际值）：

```bash
# 在开发机执行；不会传输 build/install/log，不使用 --delete。
rsync -av --exclude='__pycache__' --exclude='.pytest_cache' \
  /mnt/data/Projects/26summer/single_fr3_rviz/ros_ws/src/camera_extrinsic_calibration/ \
  USER@IPC:/ABS/PATH/single_fr3_rviz/ros_ws/src/camera_extrinsic_calibration/
```

## 安装依赖及隔离构建

以下命令在已安装 ROS 2 Jazzy 的工控机普通 shell 中执行，不进入 conda/venv。
ROS 安装步骤见 [Jazzy 官方说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-opencv python3-numpy python3-scipy \
  python3-yaml python3-pytest ros-jazzy-cv-bridge ros-jazzy-std-srvs \
  ros-jazzy-launch-ros ros-jazzy-ament-index-python

source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
# 修改成实际仓库绝对路径。
export FR3_REPO=/ABS/PATH/single_fr3_rviz
# 使用独立 workspace，避免污染或重新构建既有机器人控制安装。
export FR3_CALIB_WS="$FR3_REPO/../fr3_calibration_ws"
mkdir -p "$FR3_CALIB_WS/src"
ln -sfn "$FR3_REPO/ros_ws/src/camera_extrinsic_calibration" \
  "$FR3_CALIB_WS/src/camera_extrinsic_calibration"
cd "$FR3_CALIB_WS"
colcon build --symlink-install --packages-select camera_extrinsic_calibration
source install/setup.bash
colcon test --packages-select camera_extrinsic_calibration --event-handlers console_direct+
colcon test-result --verbose
```

`PYTHONNOUSERSITE=1` 排除用户目录 pip 包；它不能隔离 conda 或系统 `/usr/local` 里的替代库。
如果下面输出来自 conda、venv 或 `/usr/local`，先检查 shell/PYTHONPATH，不要将 pip 的
`opencv-python`/`opencv-contrib-python` 混入 apt 的 ROS cv_bridge 环境。

```bash
which python3
python3 - <<'PY'
import sys, cv2, numpy, scipy
from cv_bridge import CvBridge
print(sys.version)
for module in (cv2, numpy, scipy):
    print(module.__name__, module.__version__, module.__file__)
assert hasattr(cv2, 'aruco')
assert hasattr(cv2, 'solvePnPGeneric') and hasattr(cv2, 'solvePnPRefineLM')
image = numpy.zeros((720, 1280, 3), dtype=numpy.uint8)
bridge = CvBridge()
assert numpy.array_equal(image, bridge.imgmsg_to_cv2(
    bridge.cv2_to_imgmsg(image, encoding='bgr8'), desired_encoding='bgr8'))
print('cv_bridge round trip OK')
PY
```

## 启动及现场检查

ZED wrapper/SDK 必须在工控机上独立可用。标定节点只订阅 ROS 图像，不直接链接 ZED SDK、
CUDA 或 libfranka；通过以上测试并不能证明 GPU 驱动、相机 USB、SDK 或机器人硬件兼容。
使用已有的 ZED 左 rect 图像发布配置，单眼尺寸 1280×720。

```bash
source /opt/ros/jazzy/setup.bash
source "$FR3_CALIB_WS/install/setup.bash"
export PYTHONNOUSERSITE=1
mkdir -p "$FR3_REPO/reports"
ros2 launch camera_extrinsic_calibration camera_extrinsic_calibration.launch.py \
  layout_file:="$FR3_REPO/ros_ws/src/camera_extrinsic_calibration/config/extrinsic_calibration.yaml" \
  node_config:="$FR3_REPO/ros_ws/src/camera_extrinsic_calibration/config/node.yaml" \
  output_yaml:="$FR3_REPO/reports/camera_extrinsic.yaml" \
  debug_image_output:="$FR3_REPO/reports/camera_calibration_debug.png"
```

确认启动日志的 layout/output 绝对路径，然后在同一 ROS 环境的新终端检查：

```bash
ros2 topic hz /zed/zed_node/left/color/rect/image
ros2 topic echo /zed/zed_node/left/color/rect/camera_info --once
ros2 topic echo /camera_extrinsic_calibrator/quality
ros2 service call /camera_extrinsic_calibrator/recollect std_srvs/srv/Trigger '{}'
```

相机采样期间固定；至少两张支持板，默认连续 2 秒且至少 30 帧，通过共同位姿验收后才保存。
若一直等待，检查 Image/CameraInfo 时间戳、frame、尺寸及实际 P/R，而不是放宽质量阈值。
ZED CameraInfo 若以高于图像的频率发布，多出的未配对 CameraInfo 会从有界缓存中丢弃，
不会触发稳定窗口重置；只有未配对图像溢出才报告 `synchronization_queue_overflow`。
所有阈值及状态语义见仓库 README。

## Docker 复验

仓库 Dockerfile 已补充标定依赖；旧镜像不会自动更新，需要重新构建。
只验证相机包，可运行下列独立容器测试，不挂载设备、不使用 host network：

```bash
./ops/test_camera_calibration_noble.sh
```

容器以只读方式读取包源码，在临时路径构建，运行结束自动删除；首次需要下载 apt 依赖。
它不检查工控机硬件，也不会启动机器人控制。
