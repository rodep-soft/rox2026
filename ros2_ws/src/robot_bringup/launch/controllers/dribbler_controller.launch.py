import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
    )
    parameter_file = os.path.join(config_dir, "dribbler_controller.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_controller",
                executable="dribbler_controller_node",
                name="dribbler_controller_node",
                output="screen",
                parameters=[parameter_file],
            ),
        ]
    )
