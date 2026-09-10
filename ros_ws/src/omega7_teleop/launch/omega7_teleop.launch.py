"""Start the existing FR3 stack plus Omega.7 Cartesian teleoperation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("single_fr3_bringup")
    teleop_share = get_package_share_directory("omega7_teleop")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("robot_ip", default_value=""),
            DeclareLaunchArgument(
                "robot_config",
                default_value=os.path.join(bringup_share, "config", "robot.yaml"),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "teleop_config",
                default_value=os.path.join(
                    teleop_share, "config", "omega7_teleop.yaml"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share, "launch", "single_fr3_rviz.launch.py"
                    )
                ),
                launch_arguments={
                    "use_fake_hardware": use_fake_hardware,
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "robot_config": LaunchConfiguration("robot_config"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "load_gripper": "true",
                }.items(),
            ),
            Node(
                package="omega7_teleop",
                executable="omega7_teleop_node",
                name="omega7_teleop",
                output="screen",
                parameters=[
                    LaunchConfiguration("teleop_config"),
                    {
                        "fake_hardware": ParameterValue(
                            use_fake_hardware, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
