# Copyright 2026 single_fr3_rviz contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ipaddress
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _as_bool(value, argument_name):
    normalized = value.strip().lower()
    if normalized not in ("true", "false"):
        raise ValueError(f"{argument_name} must be 'true' or 'false', got: {value!r}")
    return normalized == "true"


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _resolve_real_ip(cli_ip, robot_config):
    configured_ip = ""
    if robot_config:
        if not os.path.isfile(robot_config):
            raise ValueError(f"robot_config does not exist: {robot_config}")
        configured_ip = str(_load_yaml(robot_config).get("robot_ip", "")).strip()

    candidate = cli_ip.strip() or configured_ip
    if not candidate:
        raise ValueError(
            "Real mode requires robot_ip:=<IPv4> or a robot_config YAML containing robot_ip."
        )

    try:
        address = ipaddress.ip_address(candidate)
    except ValueError as error:
        raise ValueError(f"robot_ip must be a valid IPv4 address, got: {candidate!r}") from error
    if (
        address.version != 4
        or address.is_unspecified
        or address.is_loopback
        or address.is_multicast
    ):
        raise ValueError(f"robot_ip is not a usable FR3 IPv4 address: {candidate!r}")
    return candidate


def _launch_setup(context):
    use_fake_hardware = _as_bool(
        LaunchConfiguration("use_fake_hardware").perform(context), "use_fake_hardware"
    )
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    robot_config = LaunchConfiguration("robot_config").perform(context).strip()
    cli_robot_ip = LaunchConfiguration("robot_ip").perform(context)

    # This branch is resolved before xacro or ros2_control_node is created. In fake
    # mode no real plugin is emitted and the configured IP is intentionally ignored.
    robot_ip = "" if use_fake_hardware else _resolve_real_ip(cli_robot_ip, robot_config)

    description_share = get_package_share_directory("single_fr3_description")
    moveit_share = get_package_share_directory("single_fr3_moveit_config")
    bringup_share = get_package_share_directory("single_fr3_bringup")

    xacro_path = os.path.join(description_share, "urdf", "fr3.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_path,
        mappings={
            "arm_prefix": "",
            "connected_to": "base",
            "fake_sensor_commands": "false",
            "hand": "false",
            "robot_ip": robot_ip,
            "use_fake_hardware": str(use_fake_hardware).lower(),
            "with_sc": "false",
        },
    ).toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_description_xml}

    srdf_path = os.path.join(moveit_share, "config", "fr3.srdf")
    with open(srdf_path, "r", encoding="utf-8") as stream:
        robot_description_semantic = {"robot_description_semantic": stream.read()}

    kinematics = {
        "robot_description_kinematics": _load_yaml(
            os.path.join(moveit_share, "config", "kinematics.yaml")
        )
    }
    planning_limits = {
        "robot_description_planning": _load_yaml(
            os.path.join(moveit_share, "config", "joint_limits.yaml")
        )
    }

    ompl_parameters = {
        "planning_plugins": ["ompl_interface/OMPLPlanner"],
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
        "start_state_max_bounds_error": 0.05,
    }
    ompl_parameters.update(
        _load_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml"))
    )
    ompl_pipeline = {"move_group": ompl_parameters}

    simple_controller_config = _load_yaml(
        os.path.join(moveit_share, "config", "moveit_controllers.yaml")
    )
    moveit_controllers = {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "moveit_simple_controller_manager": simple_controller_config,
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": True,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    controller_override = LaunchConfiguration("controllers_file").perform(context).strip()
    if controller_override:
        controllers_file = controller_override
    else:
        filename = "controllers.fake.yaml" if use_fake_hardware else "controllers.yaml"
        controllers_file = os.path.join(bringup_share, "config", filename)
    if not os.path.isfile(controllers_file):
        raise ValueError(f"controllers_file does not exist: {controllers_file}")

    common_moveit_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics,
        planning_limits,
        ompl_pipeline,
    ]

    nodes = [
        LogInfo(
            msg=(
                "single_fr3_rviz mode=FAKE (real hardware disabled)"
                if use_fake_hardware
                else f"single_fr3_rviz mode=REAL robot_ip={robot_ip}"
            )
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[robot_description, controllers_file],
            on_exit=Shutdown(),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager-timeout",
                "60",
                "--service-call-timeout",
                "60",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "--controller-manager-timeout",
                "60",
                "--service-call-timeout",
                "60",
                "--switch-timeout",
                "60",
            ],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=(
                common_moveit_parameters
                + [moveit_controllers, trajectory_execution, planning_scene_monitor]
            ),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(moveit_share, "rviz", "single_fr3.rviz")],
            parameters=common_moveit_parameters,
            condition=IfCondition(use_rviz),
        ),
    ]

    if not use_fake_hardware:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "franka_robot_state_broadcaster",
                    "--controller-manager-timeout",
                    "60",
                    "--service-call-timeout",
                    "60",
                ],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    bringup_share = get_package_share_directory("single_fr3_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use isolated ros2_control mock hardware (true/false).",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="",
                description="Real FR3 IPv4 address; overrides robot_config.",
            ),
            DeclareLaunchArgument(
                "robot_config",
                default_value=os.path.join(bringup_share, "config", "robot.yaml"),
                description="YAML containing robot_ip for real mode.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description="Optional mode-specific ros2_control YAML override.",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Start RViz 2."
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
