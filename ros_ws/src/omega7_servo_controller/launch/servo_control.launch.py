"""Independent FR3 servo bringup. It never starts Omega or sends a target."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    fake = LaunchConfiguration("use_fake_hardware").perform(context).lower() == "true"
    robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
    if not fake and not robot_ip:
        raise ValueError("real hardware requires robot_ip")
    description_share = get_package_share_directory("single_fr3_description")
    servo_share = get_package_share_directory("omega7_servo_controller")
    model = xacro.process_file(
        os.path.join(description_share, "urdf", "fr3.urdf.xacro"),
        mappings={"arm_prefix": "", "connected_to": "base",
                  "fake_sensor_commands": "false", "hand": "true",
                  "robot_ip": "" if fake else robot_ip,
                  "use_fake_hardware": str(fake).lower(), "with_sc": "false"},
    ).toxml()
    robot_description = {"robot_description": model}
    controllers_file = os.path.join(
        servo_share, "config", "controllers.fake.yaml" if fake else "controllers.real.yaml")
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", parameters=[robot_description]),
        Node(package="controller_manager", executable="ros2_control_node",
             name="controller_manager", output="screen",
             parameters=[robot_description, controllers_file], on_exit=Shutdown()),
    ]

    def spawner(controller):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[controller, "--controller-manager-timeout", "60",
                               "--service-call-timeout", "60"], output="screen")

    state_spawner = spawner("joint_state_broadcaster")
    servo_spawner = spawner("omega7_servo_controller")

    def after_state_spawner(event, _context):
        if event.returncode == 0:
            return [servo_spawner]
        reason = "failed to activate required controller joint_state_broadcaster"
        return [EmitEvent(event=ShutdownEvent(reason=reason))]

    def after_servo_spawner(event, _context):
        if event.returncode == 0:
            return []
        reason = "failed to activate required controller omega7_servo_controller"
        return [EmitEvent(event=ShutdownEvent(reason=reason))]

    # Configure controllers sequentially. Concurrent spawners can race while
    # controller_manager assigns interfaces and intermittently fail activation.
    nodes.extend([
        state_spawner,
        RegisterEventHandler(OnProcessExit(
            target_action=state_spawner, on_exit=after_state_spawner)),
        RegisterEventHandler(OnProcessExit(
            target_action=servo_spawner, on_exit=after_servo_spawner)),
    ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("robot_ip", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
