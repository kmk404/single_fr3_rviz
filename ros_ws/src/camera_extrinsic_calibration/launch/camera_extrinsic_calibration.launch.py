import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("camera_extrinsic_calibration")
    return LaunchDescription([
        DeclareLaunchArgument(
            "layout_file",
            default_value=os.path.join(share, "config", "extrinsic_calibration.yaml"),
        ),
        DeclareLaunchArgument(
            "node_config",
            default_value=os.path.join(share, "config", "node.yaml"),
        ),
        DeclareLaunchArgument("output_yaml", default_value="camera_extrinsic.yaml"),
        DeclareLaunchArgument(
            "debug_image_output",
            default_value="reports/camera_calibration_debug.png",
        ),
        DeclareLaunchArgument("debug", default_value="true"),
        DeclareLaunchArgument("save_continuously", default_value="false"),
        Node(
            package="camera_extrinsic_calibration",
            executable="camera_extrinsic_calibrator",
            name="camera_extrinsic_calibrator",
            output="screen",
            parameters=[
                LaunchConfiguration("node_config"),
                {
                    "layout_file": LaunchConfiguration("layout_file"),
                    "output_yaml": LaunchConfiguration("output_yaml"),
                    "debug_image_output": LaunchConfiguration("debug_image_output"),
                    "debug": LaunchConfiguration("debug"),
                    "save_continuously": LaunchConfiguration("save_continuously"),
                },
            ],
        ),
    ])
