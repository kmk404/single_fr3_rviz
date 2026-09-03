# single_fr3_rviz 测试报告

日期：2026-09-03  
目标环境：Ubuntu 24.04 Noble 容器、ROS 2 Jazzy  
镜像：`single-fr3-rviz:jazzy`（构建 SHA `603ebb480a6d`）

## 结论

fake 模式的完整软件控制链已通过本机验收：

```text
RViz 2 / MoveIt 2 → FollowJointTrajectory
→ joint_trajectory_controller → ros2_control GenericSystem
```

真实 FR3 未连接、未执行真实硬件测试。real 模式在没有机器人 IP 时会在创建任何 ROS 节点前失败。

## 结果

| 检查项 | 结果 | 证据 |
|---|---:|---|
| Docker Jazzy/Noble 镜像构建 | PASS | `docker_build.log` |
| workspace `colcon build` | PASS（3/3 包） | `colcon_build.log` |
| ament/colcon tests | PASS（0 failures） | `colcon_test.log` |
| 静态架构与遗留依赖扫描 | PASS | `ops/test.sh` 输出 `STATIC_TESTS=PASS` |
| fake 硬件隔离 | PASS | 日志仅加载 `mock_components/GenericSystem`，不含真实硬件插件 |
| `joint_trajectory_controller` | PASS（active） | `fake_mode.log` |
| `FollowJointTrajectory` | PASS（goal accepted，`error_code=0`） | `fake_mode.log` |
| MoveIt `move_group` 规划并执行 | PASS（`MoveItErrorCodes.SUCCESS=1`） | `fake_mode.log` |
| `/joint_states` 七关节 | PASS | `fake_mode.log` |
| ros2_control 软关节限位 | PASS（enabled） | `fake_mode.log` |
| real 模式缺 IP | PASS（exit 1、0 个节点启动） | `real_no_ip.log` |
| RViz + MoveIt MotionPlanning | PASS（可视化确认） | `rviz_fake.png` |

## 可复现命令

```bash
cd /mnt/data/Projects/26summer/single_fr3_rviz
./ops/build.sh
./ops/test.sh
./ops/test_fake.sh
./ops/run_fake.sh
```

## 真实硬件测试前置条件

- FR3 IPv4 地址与专用网卡的静态 IPv4/子网配置
- Franka Desk 中的 FCI 权限、机器人解锁状态和急停状态
- 机器人系统版本与固定的 `libfranka 0.20.5` 兼容
- 实时/低延迟内核、线程优先级、memlock 和稳定的直连以太网
- 已确认负载、末端工具、工作区、碰撞风险与实体急停可达性
