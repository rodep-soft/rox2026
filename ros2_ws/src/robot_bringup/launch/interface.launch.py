import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "joy_conversion.yaml"
    )
    config_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "receive_stm32.yml"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros2_socketcan"),
                            "launch",
                            "socket_can_bridge.launch.xml",
                        ]
                    )
                )
            ),
            Node(
                package="receive_stm32",
                executable="limit_sw_node",
                name="limit_sw_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": "/dev/input/js0",
                        "autorepeat_rate": 100.0,
                    }
                ],
            ),
            Node(
                package="joy_conversion",
                executable="joy_conversion_node",
                name="joy_conversion_node",
                parameters=[parameter_file],
                output="screen",
            ),
        ]
    )
