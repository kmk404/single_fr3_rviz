# Single FR3 RViz + MoveIt 2 + Omega.7

单台 Franka FR3 的 ROS 2 Jazzy 工作区，包含 RViz/MoveIt、fake/real 控制、Omega.7
笛卡尔遥操作、Franka Hand 夹爪控制和 ZED 2i 外参标定。

## 环境

- Ubuntu 24.04 x86_64、Docker Engine、Docker Compose v2、X11/XWayland
- `franka_ros2` v3.5.3、`franka_description` 2.9.0、`libfranka` 0.20.5
- 真机额外需要兼容的机器人系统版本、FCI、专用有线网卡和实时内核

## 构建与基础验证

```bash
cd /home/shixingma/single_fr3_rviz
./ops/build.sh
./ops/test.sh
./ops/test_fake.sh
```

fake RViz：

```bash
./ops/run_fake.sh
```

真机 MoveIt/RViz：

```bash
./ops/run_real.sh 172.16.0.2
```

连接真机前必须先在 fake 模式确认控制器、`/joint_states`、规划和执行均正常，并在
Franka Desk 中解除停止、解锁关节、启用 FCI。工作区内不得有人或障碍物，操作员须能触及急停。

## Omega.7 遥操作

当前使用 `omega7_fresh` 与 `omega7_servo_controller`，旧 `omega7_teleop` 不参与此入口：

```bash
./ops/run_omega7_servo.sh fake
./ops/run_omega7_servo.sh real 172.16.0.2
```

```text
Omega.7 (200 Hz) → FR3 base 相对末端目标 → MoveIt IK
  → 关节限速 → omega7_servo_controller → 1 kHz Ruckig → FR3
```

启动脚本必须直接运行在交互式终端。空格键切换机械臂运动使能并在启用时锁存主从零点；
夹爪不受该开关控制。离手前再次按空格确认停用，异常时使用实体急停。

当前映射与夹爪行为：

- DHD raw X/Y/Z 同向映射到 FR3 base X/Y/Z。
- 平移比例为前后 `2.0`、左右 `2.5`、上下 `2.5`。
- 夹爪普通开合直接发送完整 Move 目标，避免短段 action 造成顿挫。
- Move 明确因接触失败或 Omega 全闭时，切换到带持续夹持力的 Grasp。

完整参数、安全门、夹爪状态机和真机诊断见
[`omega7_fresh/README.md`](ros_ws/src/omega7_fresh/README.md)。

### Omega.7 SDK 与 USB

SDK 默认从 `/mnt/data/Projects/omega7/SDK3.17.7` 只读挂载。路径不同时在 `.env` 设置：

```bash
OMEGA7_SDK_DIR=/absolute/path/to/SDK3.17.7
```

首次连接执行 `./ops/install_omega7_udev.sh`，重新插拔后用 `lsusb -d 1451:0402`
检查设备。容器仅挂载 `/dev/bus/usb` 并放行 USB 字符设备，无需启用 `privileged`。

## RViz 操作

1. 等待终端显示规划已就绪，并确认控制器为 `active`。
2. 在 MotionPlanning 面板选择 `fr3_arm`，Start State 设为 Current State。
3. 拖动末端目标，先 Plan 检查轨迹，再在 fake 模式 Execute。

```bash
ros2 control list_controllers
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
ros2 topic echo /joint_states --once
```

## ZED 2i 外参标定

标定包根据平台四角 ArUco marker 估计左目 optical frame 到 `fr3_link0` 的变换：

```text
p_base = T_base_camera @ p_camera_optical
```

构建：

```bash
cd /home/shixingma/single_fr3_rviz/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to camera_extrinsic_calibration zed_wrapper
source install/setup.bash
```

启动相机与标定：

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  param_overrides:='video.publish_left_right:=true;video.publish_rgb:=false;depth.depth_mode:=NONE;pos_tracking.pos_tracking_enabled:=false'

ros2 launch camera_extrinsic_calibration camera_extrinsic_calibration.launch.py \
  debug:=true \
  output_yaml:=/home/shixingma/single_fr3_rviz/camera_extrinsic.yaml \
  debug_image_output:=/home/shixingma/single_fr3_rviz/reports/camera_calibration_debug.png
```

marker 布局和 `T_table_base` 位于
[`extrinsic_calibration.yaml`](ros_ws/src/camera_extrinsic_calibration/config/extrinsic_calibration.yaml)。
安装估算仅适合初始集成，高精度使用前必须实测替换。输出 camera 指左目 optical frame。

## 故障排除

- RViz 无窗口：检查 `DISPLAY`、`/tmp/.X11-unix` 和 `xhost +local:docker`。
- 控制器不可用：运行 `ros2 control list_controllers`，检查启动日志。
- 真机连接超时：检查 IP、专用网卡、路由、防火墙、实时内核和 FCI 状态。
- `dhdOpen failed`：安装 udev 规则、重新插拔 Omega.7，并重新创建容器。
- Desk 显示停止或 reflex：先停止脚本，排查原因并在 Desk 中恢复；程序不会自动清错。

## 目录

```text
config/                         # 机器人与控制器参数
docker/                         # Jazzy/Noble 镜像和 Compose
ops/                            # 构建、启动和测试脚本
reports/                        # 本地日志与测试产物
ros_ws/src/
  omega7_fresh/                 # Omega 输入、IK、夹爪调度
  omega7_servo_controller/      # 1 kHz 连续关节控制器
  single_fr3_description/       # FR3 描述
  single_fr3_moveit_config/     # MoveIt 配置
  single_fr3_bringup/           # RViz/MoveIt 启动
  camera_extrinsic_calibration/ # ZED 外参标定
```

正常停止使用 `Ctrl-C`。真实硬件出现异常运动、碰撞预警或网络抖动时立即使用实体急停。
