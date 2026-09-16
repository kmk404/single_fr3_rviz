# Single FR3 RViz + MoveIt 2 + Omega.7

独立的单臂 Franka FR3 控制仓库，目标平台为 Ubuntu 24.04 Noble 和 ROS 2 Jazzy。

控制链固定为：

```text
RViz 2 → MoveIt 2 / move_group
       → FollowJointTrajectory
       → joint_trajectory_controller
       → ros2_control
       → mock_components/GenericSystem（fake）或 franka_hardware（real）
       → FR3
```

仓库只包含单台 FR3，7 个关节统一为 `fr3_joint1` … `fr3_joint7`，并支持 Franka Hand 和 Force Dimension Omega.7 笛卡尔遥操作。不包含原项目的双臂或自定义 UDP 控制链。

## 系统要求

- Ubuntu 24.04 x86_64 工控机（容器同样固定为 Ubuntu 24.04 Noble）
- Docker Engine 和 Docker Compose v2
- X11 显示；Wayland 桌面需要提供 XWayland
- 真实机器人模式额外需要实时内核、专用有线网卡、可用的 FCI、正确的机器人系统版本与 libfranka 兼容性

容器基础镜像是 `ros:jazzy-ros-base-noble`。Franka 依赖固定在：

- `franka_ros2` v3.5.3
- `franka_description` 2.9.0
- `libfranka` 0.20.5

## 构建

```bash
cd /mnt/data/Projects/26summer/single_fr3_rviz
./ops/build.sh
```

脚本会生成 `.env`、构建 Jazzy 容器和 workspace，并把日志写入 `reports/docker_build.log` 与 `reports/colcon_build.log`。

## fake 模式（必须先验证）

一键启动：

```bash
./ops/run_fake.sh
```

或进入已构建容器后直接启动：

```bash
source /workspace/ros_ws/install/setup.bash
ros2 launch single_fr3_bringup single_fr3_rviz.launch.py use_fake_hardware:=true
```

fake 分支在创建 URDF 前强制清空 IP，只生成 `mock_components/GenericSystem` 插件，并选用 position command interface；因此不会实例化 `franka_hardware/FrankaHardwareInterface`，也不会连接真实机器人。

无 GUI 自动冒烟测试：

```bash
./ops/test.sh
./ops/test_fake.sh
```

测试会确认控制器 active、`/joint_states` 含全部 7 个关节、标准 Action 存在，先发送一条小幅度控制器轨迹，再通过 `move_group` 完成一次规划与执行。结果写入 `reports/fake_mode.log`。

## Omega.7 遥操作

SDK 默认从 `/mnt/data/Projects/omega7/SDK3.17.7` 只读挂载。路径不同时，在 `.env` 中设置：

```bash
OMEGA7_SDK_DIR=/absolute/path/to/SDK3.17.7
```

容器启动时会把 Linux x86_64 SDK 解压到容器的临时目录，不会复制 SDK 到仓库。Omega.7 通过 `/dev/bus/usb` 传入容器；宿主机用户必须具有对应 USB 设备的读写权限。

首次连接 Omega.7 前，在宿主机安装 USB 权限规则：

```bash
./ops/install_omega7_udev.sh
```

脚本会把 `udev/99-omega7.rules` 安装到 `/etc/udev/rules.d/`，重新加载 udev 规则，并触发已连接 USB 设备的规则匹配。安装后仍须重新插拔 Omega.7，再用 `lsusb -d 1451:0402` 和 `ls -l /dev/bus/usb/BBB/DDD` 确认设备存在且权限为 `crw-rw-rw-`。Compose 会继续挂载整个 `/dev/bus/usb`，并仅通过 `device_cgroup_rules` 放行 USB 总线的字符设备（major 189）；无需且不应启用容器 `privileged` 模式。

先构建并在 fake 模式验证：

```bash
./ops/build.sh
./ops/check_omega7_compat.sh
./ops/run_omega7_fake.sh
```

兼容性脚本会检查容器确为 Ubuntu 24.04 x86_64、SDK 3.17.7 动态库依赖完整，并确认遥操作所需的 DHD 符号可以加载。当前 SDK 包名为 `sdk-3.17.7-linux-x86_64-gcc.tar.gz`，与工控机架构必须一致。

设备连接后，在运行 `./ops/run_omega7_fake.sh` 或 `./ops/run_omega7_real.sh` 的终端中按一次空格键，同时启用机械臂遥操作和夹爪控制；再按一次空格键停止两者。每次启用都会锁存 Omega.7 与 `fr3_hand_tcp` 的当前位姿作为零点，避免机械臂跳变。空格键由直接占有终端的启动包装脚本读取，再通过 `/omega7_teleop/toggle_motion` 服务通知节点，不依赖 `ros2 launch` 子进程的标准输入。启动脚本必须直接运行在交互式终端中，不能重定向标准输入。

Omega.7 夹爪只控制 Franka Hand，不再参与使能；未按空格时不会发布夹爪命令。使能后，完全张开对应 `robot_gripper_open_width`，逐渐捏合会连续经过中间开度，合拢对应 `robot_gripper_closed_width`。夹爪命令每 `150 ms` 最多更新一次，小于 `2 mm` 的 Franka 宽度变化会被忽略以抑制抖动。注意：空格键是锁存开关；离手前应再按一次空格并确认终端显示 `arm motion disabled by SPACE`，异常时使用实体急停。

默认轴向已按实际操作方向校正：Omega 的前后轴与左右轴互换后，两条水平轴同时反向，上下轴取反；平移与手腕旋转使用同一个右手正交映射。MoveIt 对每个受限末端目标进行碰撞感知 IK，J1–J7 全部参与求解，以保留 FR3 的冗余自由度并减少奇异构型附近的无解。每个关节的单周期变化均受 `max_joint_velocity` 限制，其中 J7 默认采用更保守的 `0.25 rad/s`；连续遥操作使用短时域 `JointTrajectory` 平滑跟踪，不会为每个设备采样运行耗时的 OMPL 全局路径规划。

重新标定会关闭运动使能。运行：

```bash
ros2 service call /omega7_teleop/recalibrate std_srvs/srv/Trigger '{}'
```

然后把 Omega.7 放到希望作为零点的位置，在启动终端按一次空格键重新启用。

主要参数位于 `ros_ws/src/omega7_teleop/config/omega7_teleop.yaml`：

- `position_scale`、`orientation_scale`：平移三轴与旋转缩放。
- `axis_mapping`：Omega 局部坐标到 FR3 工具局部坐标的正交 3×3 行主序矩阵。
- `position_deadzone`、`orientation_deadzone`：零点附近死区。
- `max_linear_speed`、`max_angular_speed`、`max_joint_velocity`：笛卡尔与关节限速。
- `workspace_min`、`workspace_max`：`fr3_link0` 坐标系下的 XYZ 工作空间边界。
- `device_timeout`、`joint_state_timeout`、`max_ik_failures`：输入与求解看门狗。
- `omega_gripper_closed_gap`、`omega_gripper_open_gap`：Omega 夹持间距标定范围。
- `robot_gripper_closed_width`、`robot_gripper_open_width`：Franka 夹爪总宽度范围。

连接真机前把缩放、轴向、工作空间和夹爪范围在 fake 模式逐项核对。真机启动方式为：

```bash
./ops/run_omega7_real.sh 172.16.0.2
```

设备断连、SDK 返回异常/非有限值、数据超时、关节状态超时、连续 IK 失败或松开夹持器都会停止遥操作。重新连接后必须先松开再捏住，防止带着旧零点自动恢复运动。

## RViz 操作

1. 等待终端出现 `You can start planning now!`，并确认控制器为 active。
2. 在 RViz 的 MotionPlanning 面板中选择 `fr3_arm`。
3. 将 Start State 设为 Current State。
4. 拖动末端交互标记，点击 **Plan**；先检查轨迹没有碰撞或跳变。
5. fake 模式下点击 **Execute**，确认机器人模型平滑到达目标。
6. 保存截图为 `reports/rviz_fake.png`，作为真实硬件启用前的审核材料。

接口核对：

```bash
ros2 control list_controllers
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
ros2 topic echo /joint_states --once
```

## real 模式

> 不要在 fake 构建、Action 冒烟测试和 RViz 规划验证全部通过前启动 real 模式。

可直接传 IP：

```bash
ros2 launch single_fr3_bringup single_fr3_rviz.launch.py \
  robot_ip:=172.16.0.2 use_fake_hardware:=false
```

也可编辑 `config/robot.yaml` 后运行：

```bash
./ops/run_real.sh
```

或用脚本参数覆盖配置：

```bash
./ops/run_real.sh 172.16.0.2
```

real 模式在启动任何 ROS 节点之前验证 IPv4。未提供 IP、格式错误、loopback、multicast 或 `0.0.0.0` 都会立即失败。命令行 `robot_ip` 优先于 YAML。

### 真实硬件启动前必须确认

- FR3 的实际 IPv4 地址和机器人型号确为 FR3（不是 FR3v2）
- 主机网卡静态地址、子网、MTU、直连网线和 `ping` 连通性
- Franka Desk 中已解锁关节、释放急停并启用 FCI
- 机器人系统版本与 libfranka 0.20.5 兼容
- 主机已配置低延迟/实时内核、线程优先级和 memlock
- 工作区清空，负载和末端工具配置正确，操作员可触及急停

## 安全机制

- 官方 `franka_description` 提供机械关节、速度、力矩和碰撞几何限制。
- MoveIt 配置使用保守的速度/加速度上限、起点容差和执行超时。
- SRDF 保留官方单臂自碰撞矩阵；MoveIt 在规划和执行前检查碰撞。
- `joint_trajectory_controller` 禁止 partial goal，并设置逐关节轨迹/目标容差、停止速度容差及 goal timeout。
- real 模式使用 effort 控制及官方增益；Franka 控制柜/FCI 的碰撞与反射安全机制仍然有效。本项目不会自动放宽控制柜碰撞阈值。

正常停止使用 `Ctrl-C`。真实硬件发生异常运动、碰撞预警或网络抖动时应立即使用实体急停，不要只依赖终端。

## 故障排除

- **RViz 无窗口**：确认 `DISPLAY` 正确，`/tmp/.X11-unix` 存在，并执行 `xhost +local:docker`。
- **MoveIt 等不到控制器**：运行 `ros2 control list_controllers`，确认 `joint_trajectory_controller` 和 `joint_state_broadcaster` 为 `active`。
- **没有 joint states**：检查 `ros2 topic echo /joint_states --once` 和 controller manager 日志。
- **real 模式立即退出**：先检查 IP 是否已通过参数或 `config/robot.yaml` 提供；该失败是预期的安全保护。
- **连接超时/UDP receive timeout**：检查专用网卡、路由、防火墙、实时内核和 CPU 调度。不要反复执行轨迹，先解决网络抖动。
- **libfranka 版本不兼容**：按 Franka 官方兼容表核对机器人系统版本；不要绕过版本检查。
- **Omega.7 报 `dhdOpen failed: no device found`**：运行 `./ops/install_omega7_udev.sh` 后重新插拔设备，确认 `1451:0402` 对应的 `/dev/bus/usb/BBB/DDD` 权限为 `0666`；然后重新创建容器，使 Compose 的 USB cgroup 规则生效。
- **NVIDIA 图形加速**：安装 NVIDIA Container Toolkit，并在 `docker/compose.yaml` 中取消 `gpus: all` 注释。软件渲染不影响控制接口验证。

## 目录

```text
single_fr3_rviz/
├── config/                     # IP 与 fake/real 控制器配置
├── docker/                     # Jazzy/Noble 镜像与 Compose
├── ops/                        # 构建、启动、测试脚本
├── reports/                    # 本地测试产物（日志/截图不提交）
├── udev/                       # Omega.7 USB 权限规则
└── ros_ws/src/
    ├── single_fr3_description/
    ├── single_fr3_moveit_config/
    ├── single_fr3_bringup/
    └── omega7_teleop/
```

官方依赖通过 `franka_jazzy.repos` 固定版本，未复制或修改原仓库 `/mnt/data/Projects/26summer/Gello_armhand_teleop12`。

## ZED 2i 自动外参标定

`camera_extrinsic_calibration` 使用平台四角的固定 ArUco marker 估计可移动
ZED 2i 左目光学坐标系相对 FR3 基座的位姿。核心逻辑不依赖 marker 的固定
布局；更换标记或实测基座位姿时只需编辑
`ros_ws/src/camera_extrinsic_calibration/config/extrinsic_calibration.yaml`。

当前物理配置为 `DICT_5X5_50`、黑框边长 `0.077 m`，marker ID/角点对应为
BL=1、BR=0、TL=2、TR=5。配置中的 `T_table_base` 是安装估算值：以孔
(14,29) 和 (19,24) 的中点作为 `fr3_link0` 原点投影、Front 指向 table
`-y`，并忽略 M6/M8 间隙、安装高度和倾斜。因此输出可用于初始集成，但在
需要高精度 base-frame 外参前必须实测并替换该矩阵。

本机已按 `zed_jazzy.repos` 固定并从源码构建官方 ZED ROS 2 wrapper v5.4.1。
重新拉取依赖时使用：

```bash
vcs import ros_ws/src < zed_jazzy.repos
```

系统仍需 ZED SDK 5.4.1，以及 wrapper 的 rosdep/GeographicLib 开发依赖。启动
wrapper 时必须显式启用 left/right 发布；官方默认配置只发布内容相同的 `rgb`
左目别名。推荐的低负载标定启动方式为：

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  param_overrides:='video.publish_left_right:=true;video.publish_rgb:=false;depth.depth_mode:=NONE;pos_tracking.pos_tracking_enabled:=false'
```

另开终端运行标定节点：

```bash
source /workspace/ros_ws/install/setup.bash
ros2 launch camera_extrinsic_calibration camera_extrinsic_calibration.launch.py \
  output_yaml:=/workspace/camera_extrinsic.yaml
```

默认订阅 wrapper 5.1+ 的：

- `/zed/zed_node/left/color/rect/image`
- `/zed/zed_node/left/color/rect/camera_info`

默认并严格检查 optical frame `zed_left_camera_frame_optical`。若实际
`camera_name`、namespace 或 wrapper 版本不同，应在 `config/node.yaml` 中同步
修改 topics 和 frame。这里使用 rectified 左彩色图对应的真实 `CameraInfo`；
不会使用写死内参。

有效结果原子写入 `camera_extrinsic.yaml`。其中 `T_base_camera` 的 camera 明确
指图像消息的 **左目 optical frame**，矩阵语义是把该 optical frame 中的点变换
到 `fr3_link0`：

```text
p_base = T_base_camera @ p_camera_optical
```

节点同时发布：

- `~/quality`：JSON 质量指标，包括检测 ID、有效 marker 数、重投影误差、
  平移/旋转 spread、confidence 和 calibration_valid。
- `~/debug_image`：marker 四角、ID、坐标轴和各 marker 重投影误差。

每个 marker 会独立产生 `T_table_camera`。估计先按重投影误差过滤，再以
translation 与 SO(3) 测地角建立最大一致集合；剩余平移按重投影误差加权，
rotation 使用 SciPy 的 quaternion/SO(3) mean，不平均 Euler angle。一个 marker
也可输出，但 confidence 上限自动降为多 marker 情况的一半。默认只在本次运行
首次得到 `calibration_valid=true` 时写文件；持续覆盖可设置
`save_continuously:=true`。
