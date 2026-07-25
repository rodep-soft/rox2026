import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
    )
    belt_parameter_file = os.path.join(config_dir, "belt_controller.yaml")
    dribble_parameter_file = os.path.join(config_dir, "dribble_controller.yaml")

    return LaunchDescription(
        [
            Node(
                package="robot_controller",
                executable="belt_controller_node",
                name="belt_controller_node",
                output="screen",
                parameters=[belt_parameter_file],
            ),
            Node(
                package="robot_controller",
                executable="dribble_controller_node",
                name="dribble_controller_node",
                output="screen",
                parameters=[dribble_parameter_file],
            ),
        ]
    )
