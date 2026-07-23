import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "joy.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="joy_controller",
                executable="joy_controller_node",
                name="joy_controller",
                output="screen",
                parameters=[parameter_file],
            ),
        ]
    )
