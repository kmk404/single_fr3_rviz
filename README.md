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

构建标定包（只构建该包，不启动机器人控制）：

```bash
cd /mnt/data/Projects/26summer/single_fr3_rviz
source /opt/ros/jazzy/setup.bash
cd ros_ws
colcon build --symlink-install --packages-select camera_extrinsic_calibration
source install/setup.bash
cd ..
```

另开终端启动标定。显式传入源码布局和绝对输出路径，避免读到安装目录的旧 YAML：

```bash
cd /mnt/data/Projects/26summer/single_fr3_rviz
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros2 launch camera_extrinsic_calibration camera_extrinsic_calibration.launch.py \
  layout_file:="$PWD/ros_ws/src/camera_extrinsic_calibration/config/extrinsic_calibration.yaml" \
  node_config:="$PWD/ros_ws/src/camera_extrinsic_calibration/config/node.yaml" \
  output_yaml:="$PWD/reports/camera_extrinsic.yaml" \
  debug_image_output:="$PWD/reports/camera_calibration_debug.png"
```

容器内将上述仓库路径替换为 `/workspace`。启动日志会打印实际读取布局的绝对路径、
SHA-256 和输出绝对路径；不传 `layout_file` 时仍默认使用安装目录的配置。

默认订阅 `/zed/zed_node/left/color/rect/image` 和对应 `camera_info`，严格核对
`zed_left_camera_frame_optical`。Image/CameraInfo 必须具有**相同时间戳**，各自缓存最多
10 条，不复用旧 CameraInfo；多余的高频 CameraInfo 只会被丢弃，不会清空稳定窗口。
未配对图像溢出或数据中断会阻止稳定验收。若 wrapper
使用不同名称，在 `node.yaml` 修改 topics/frame。请先核查实际消息，不要放宽检测条件。

现场左眼尺寸为 **1280×720**，`node.yaml` 的 `expected_image_width/height` 默认严格检查
此尺寸，并核对 CameraInfo 尺寸及解码后的数组尺寸。**2560×720 是左右拼接图**，不能
作为左眼输入。合法切换单眼分辨率时需同步修改这两个参数，并重新采集。
rect 图使用 `CameraInfo.P[:3,:3]`，PnP 畸变为零，不使用原图的 K/D，也不引入双目基线。
左眼 P 的第四列必须为零；非默认 ROI/binning 暂不支持，直接报告失败，避免错误缩放。
CameraInfo.R 将原始 optical 坐标旋转到 rectified 坐标；若非单位阵，保存时显式换回
消息声明的 optical frame，并保留 rectified PnP 矩阵以便审计。参考内参
fx=fy=528.8273315429688、cx=630.1070556640625、cy=356.42413330078125 仅用于现场核对，
没有写死到求解器。实现依据 [CameraInfo 消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg)
与 [OpenCV PnP 文档](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)。

### 联合求解与保存条件

角点始终按 ArUco 解码后的印刷 TL/TR/BR/BL 顺序配对。marker 的 +x 向印刷右方、+y 向
印刷下方，四角通过现有 `T_table_marker` 变换到 table；上排 180° 旋转已包含在 YAML
中，绝不按图像上下左右重新排序。每次拼接至少两张板的角点，用 `SOLVEPNP_IPPE`
生成共面候选（不是 `IPPE_SQUARE`），检查全部支持角点正深度、相机在 table 上方，
再使用 `solvePnPRefineLM` 联合优化。两板必须同时合格；三至四板枚举至少两板的子集，
对所有检测板按整板评分，优先最多支持板，再按联合 RMS 排序、优化并重新核对支持集。
同等支持数、RMS 差不超过 0.15 px、位姿差超过 5 mm 或 0.5° 时，保守报告候选歧义。
这些歧义阈值也在 `quality` 中配置；不会按 IPPE 返回顺序选择。

PnP 得到 `T_camera_table`，取逆得到 `T_table_camera`，再计算
`T_base_camera = inverse(T_table_base) @ T_table_camera`。
当 CameraInfo.R 非单位阵时，先完成上述 rectified→optical 换算。
最终误差均由**同一个联合位姿**重新投影计算：角点二维欧氏距离的 RMS。
全局 RMS 覆盖支持板的全部角点，每板 RMS 使用该板四角；不再平均单板独立拟合误差，
不再平均单板位姿，也不输出可替代验收条件的 confidence 分数。单板不能正式保存。

`extrinsic_calibration.yaml` 的 `quality` 初始验收阈值如下，**不是精度保证**，不会自动放宽：

| 条件 | 默认值 |
| --- | --- |
| 每帧支持板数 | ≥2 |
| 全局角点 RMS / 每张支持板 RMS | ≤1.0 px / ≤1.5 px |
| 连续窗口时长 / 不同时间戳帧数 | ≥2 秒 / ≥30 帧，两者同时满足 |
| 相对整个窗口中心的最大平移 / SO(3) 旋转偏差 | ≤5 mm / ≤0.5° |
| 数据中断（消息时间差及墙钟看门狗） | >0.5 秒清空 |
| 窗口容量保护 | 1000 帧，未满足时长则重置，绝不滑动截断 |

状态依次为 `waiting`、`collecting`、`finalizing`、`saved`。窗口不断增长，不滑动丢弃早期帧；
平移中心与 SO(3) rotation mean 用于整窗稳定性检查，另外以首帧锚定同样的运动阈值，
避免慢漂移随着中心移动而被掩盖。不合格帧、歧义、明显移动、失去检测、重复/倒退/零
时间戳、长时间中断、尺寸或内参变化都会清空当前窗口。固定窗口只能约束观测期间的
运动；小于阈值的运动不代表绝对静止，相机移动后必须重新标定。

窗口通过后，汇总各帧支持板角点，LM 优化**一个共同位姿**；逐帧、逐板重新检查残差、
正深度、桌面上方、支持集及相对最终位姿的波动。全部合格才在同目录临时文件中写入、
flush/fsync 并原子替换结果；写入失败不锁定、不破坏旧文件。默认成功一次后锁定。
`save_continuously:=true` 的兼容语义改为：每次覆盖前必须重新收集一个完整、互不重叠的
稳定窗口，不再逐帧覆盖。旧配置中的单板融合阈值已移除，迁移时使用新版 `quality`。

### 诊断、重新采集与输出

```bash
ros2 topic echo /camera_extrinsic_calibrator/quality
ros2 run rqt_image_view rqt_image_view /camera_extrinsic_calibrator/debug_image
ros2 run rqt_image_view rqt_image_view /camera_extrinsic_calibrator/raw_image
ros2 service call /camera_extrinsic_calibrator/recollect std_srvs/srv/Trigger '{}'
ros2 param get /camera_extrinsic_calibrator layout_file
ros2 param get /camera_extrinsic_calibrator output_yaml
realpath reports/camera_extrinsic.yaml
```

重新采集立即解除锁定并清空缓存，旧 YAML 保留到新结果通过；不会删除已有标定。
`quality` JSON 包含检测 ID、支持 ID、各板拒绝原因、联合全局/每板 RMS、候选数量及歧义、
窗口帧数/时长、平移/旋转最大波动和状态；`calibration_valid` 表示当前帧几何合格，
**不等于已经保存**，应同时看 `state=saved`/`saved_once`。未能求得有效候选时 RMS 可为空；
若存在被拒绝的联合拟合，`residual_scope` 会注明诊断误差的范围。

检测开启 `CORNER_REFINE_SUBPIX`。调试图绿色表示实际支持板、红色表示拒绝的已配置板、
黄色表示未配置检测 ID，紫色显示 ArUco 未通过解码的 rejected 四边形。排查右下 ID0 时
同时观察原图和紫色候选，检查遮挡、反光、模糊及黑框完整性；不通过降低阈值掩盖问题。
`raw_image` 保留原消息，叠加仅在副本进行；保存时另外写出 `*_raw.png` 和调试 PNG。

结果记录 `T_camera_table`、`T_table_camera`、`T_base_camera`（兼容 `matrix_4x4`）、平移/
四元数、坐标约定、支持 ID、逐帧时间戳与支持集/残差、窗长/帧数/稳定性、图像尺寸、实际
CameraInfo 与 PnP 内参、质量阈值、布局快照/校验和及路径。
`table_to_base_is_approximate` 保留；时间稳定和低重投影误差不能消除板布局测量误差、
内参偏差或基座安装估算误差，**不代表机器人基座绝对精度**。

### 验证

```bash
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 PYTHONPATH="$PWD/ros_ws/src/camera_extrinsic_calibration:$PYTHONPATH" \
  python3 -m pytest -q ros_ws/src/camera_extrinsic_calibration/test
cd ros_ws
colcon test --packages-select camera_extrinsic_calibration --event-handlers console_direct+
colcon test-result --verbose
```

已在 Ubuntu 24.04.4 x86_64 / ROS 2 Jazzy / Python 3.12.3 / OpenCV 4.6.0 容器中完成
colcon 构建和 45 项测试（全部通过，无跳过），并验证安装后的 launch、参数读取、重新采集服务
及无输入时不保存。系统 ROS 2 Humble / OpenCV 4.5.4 也已通过同一套测试；纯数学部分另在
OpenCV 5.0.0 验证。用户级 OpenCV 5 与本机 Humble 的 cv_bridge 不兼容，完整节点测试使用
`PYTHONNOUSERSITE=1` 选择系统匹配依赖，没有修改系统依赖。测试覆盖无噪声多板恢复、
变换方向、上排 180° 顺序、单板拒绝、两板矛盾、三/四板异常、同等支持歧义、稳定及
带噪序列、移动/慢漂移/旋转、丢失/时间戳/间断/内参尺寸变化、逐帧最终残差、一次锁定/
连续模式/重新采集、精确配对、非单位 R 换算、原图保护及原子写入失败。
针对现场 CameraInfo 约 59 Hz、图像约 20 Hz 的速率差，随后补充了两项同步缓存回归测试；
本机 Humble / 系统 OpenCV 环境现为 47 项通过。该补丁仍需在工控机 Jazzy 环境复验。

仍需现场验证：工控机 ZED wrapper 的消息配对、实际 P/R/分辨率、光照与右下 ID0
检出率、实际静止/移动序列、保存路径权限，以及用独立实测基准评估绝对精度。
未连接相机或机器人，没有运行或修改机器人控制模块。

Ubuntu 24.04 工控机的源码传输、依赖安装、独立工作区构建和启动步骤见
[部署与兼容性说明](docs/camera_calibration_ubuntu2404.md)。
可运行 `./ops/test_camera_calibration_noble.sh` 在隔离的 Noble/Jazzy 容器中复验标定包；
仓库 Dockerfile 已补齐标定依赖，旧镜像需重新构建。
