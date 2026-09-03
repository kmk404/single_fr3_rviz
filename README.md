# Single FR3 RViz Control

Single-arm Franka FR3 robot control using RViz 2 + MoveIt 2 on ROS 2 Jazzy.

## Architecture

RViz 2 → MoveIt 2 → FollowJointTrajectory → joint_trajectory_controller → ros2_control + libfranka → FR3

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Docker & Docker Compose
- X11 forwarding for RViz

## Quick Start

### Fake Hardware Mode
```bash
./ops/run_fake.sh
```

### Real Hardware Mode
```bash
./ops/run_real.sh <robot_ip>
```

See full documentation below for setup and usage.

