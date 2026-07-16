import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("robot_bringup")
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "joy_conversion.yaml"
    )
    config_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "receive_stm32.yml"
    )

    return LaunchDescription(
        [
            Node(
                package="ros2socketcan_bridge",
                executable="ros2socketcan",
                output="screen",
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
