"""Omega.7 teleoperation with the independent continuous FR3 servo controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def read_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def launch_setup(context):
    fake = LaunchConfiguration("use_fake_hardware").perform(context).lower() == "true"
    robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
    rviz = LaunchConfiguration("use_rviz").perform(context).lower() == "true"
    start_teleop = LaunchConfiguration("start_teleop").perform(context).lower() == "true"
    if not fake and not robot_ip:
        raise ValueError("real hardware requires robot_ip")
    servo_share = get_package_share_directory("omega7_servo_controller")
    fresh_share = get_package_share_directory("omega7_fresh")
    description_share = get_package_share_directory("single_fr3_description")
    moveit_share = get_package_share_directory("single_fr3_moveit_config")
    description = xacro.process_file(
        os.path.join(description_share, "urdf", "fr3.urdf.xacro"),
        mappings={"arm_prefix": "", "connected_to": "base",
                  "fake_sensor_commands": "false", "hand": "true",
                  "robot_ip": "" if fake else robot_ip,
                  "use_fake_hardware": str(fake).lower(), "with_sc": "false"},
    ).toxml()
    with open(os.path.join(moveit_share, "config", "fr3.srdf"), encoding="utf-8") as stream:
        semantic = stream.read()
    ompl = read_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml"))
    ompl.update({
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
        ],
    })
    common = [
        {"robot_description": description},
        {"robot_description_semantic": semantic},
        {"robot_description_kinematics": read_yaml(
            os.path.join(moveit_share, "config", "kinematics.yaml"))},
        {"robot_description_planning": read_yaml(
            os.path.join(moveit_share, "config", "joint_limits.yaml"))},
        {"move_group": ompl},
    ]
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                servo_share, "launch", "servo_control.launch.py")),
            launch_arguments={"use_fake_hardware": str(fake).lower(),
                              "robot_ip": robot_ip}.items()),
        Node(package="moveit_ros_move_group", executable="move_group", name="move_group",
             output="screen", parameters=common + [{"allow_trajectory_execution": False}]),
    ]
    if start_teleop:
        nodes.append(Node(package="omega7_fresh", executable="teleop", name="omega7_fresh",
                          output="screen", parameters=[
                              os.path.join(fresh_share, "config", "omega7_fresh.yaml"),
                              {"fake_hardware": fake}]))
    if rviz:
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", os.path.join(moveit_share, "rviz", "single_fr3.rviz")],
            parameters=common))
    if not fake:
        gripper_share = get_package_share_directory("franka_gripper")
        nodes.append(Node(
            package="franka_gripper", executable="franka_gripper_node",
            name="franka_gripper", output="screen",
            parameters=[{"robot_ip": robot_ip,
                         "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"]},
                        os.path.join(gripper_share, "config", "franka_gripper_node.yaml")],
            remappings=[("~/joint_states", "/joint_states")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("robot_ip", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_teleop", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
