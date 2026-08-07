import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "edulite05_driver_v2.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="hardware_driver",
                executable="edulite05_node",
                name="edulite05_driver",
                output="screen",
                parameters=[parameter_file],
            )
        ]
    )
