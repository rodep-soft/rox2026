import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    socketcan_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_socketcan"),
                "launch",
                "socket_can_bridge.launch.xml",
            )
        ),
        launch_arguments={
            "interface": LaunchConfiguration("can_interface"),
            "enable_can_fd": "false",
            "from_can_bus_topic": "/socketcan_bridge/rx",
            "to_can_bus_topic": "/socketcan_bridge/tx",
        }.items(),
    )

    vesc_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "vesc_driver.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by ros2_socketcan",
            ),
            socketcan_launch,
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_driver",
                output="screen",
                parameters=[vesc_parameter_file],
            ),
        ]
    )
