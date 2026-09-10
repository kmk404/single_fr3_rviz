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

先构建并在 fake 模式验证：

```bash
./ops/build.sh
./ops/check_omega7_compat.sh
./ops/run_omega7_fake.sh
```

兼容性脚本会检查容器确为 Ubuntu 24.04 x86_64、SDK 3.17.7 动态库依赖完整，并确认遥操作所需的 DHD 符号可以加载。当前 SDK 包名为 `sdk-3.17.7-linux-x86_64-gcc.tar.gz`，与工控机架构必须一致。

设备连接后，节点依次调用 `dhdEnableForce(DHD_ON)` 和 `dhdEmulateButton(DHD_ON)`，并通过 `dhdGetButton(0)` 把 Omega.7 夹爪模拟成使能按钮。启动后先完全松开夹持器；只在模拟按钮按下期间发送机械臂运动指令。按钮按下沿锁存 Omega.7 当前位姿和 `fr3_hand_tcp` 当前位姿作为位置/姿态零点；松开沿立即发布保持轨迹并打开 Franka 夹爪。再次按下会重新对齐双端零点，避免机械臂跳变。

Omega.7 的局部 XYZ 和三轴旋转默认一一对应 FR3 末端局部轴。MoveIt 对每个受限末端目标进行碰撞感知 IK，J1–J6参与求解；首次完整 `/joint_states` 中的 J7 被锁存，IK 请求、结果检查和每条控制器命令都会强制保持该角度。连续遥操作使用短时域 `JointTrajectory`，不会为每个设备采样运行耗时的 OMPL 全局路径规划。

重新标定前先松开夹持器，然后运行：

```bash
ros2 service call /omega7_teleop/recalibrate std_srvs/srv/Trigger '{}'
```

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
- **NVIDIA 图形加速**：安装 NVIDIA Container Toolkit，并在 `docker/compose.yaml` 中取消 `gpus: all` 注释。软件渲染不影响控制接口验证。

## 目录

```text
single_fr3_rviz/
├── config/                     # IP 与 fake/real 控制器配置
├── docker/                     # Jazzy/Noble 镜像与 Compose
├── ops/                        # 构建、启动、测试脚本
├── reports/                    # 本地测试产物（日志/截图不提交）
└── ros_ws/src/
    ├── single_fr3_description/
    ├── single_fr3_moveit_config/
    ├── single_fr3_bringup/
    └── omega7_teleop/
```

官方依赖通过 `franka_jazzy.repos` 固定版本，未复制或修改原仓库 `/mnt/data/Projects/26summer/Gello_armhand_teleop12`。
