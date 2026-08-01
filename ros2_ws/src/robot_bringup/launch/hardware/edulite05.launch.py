import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "edulite05_driver.yaml",
    )

    node_names = [
        "edulite05_fl_driver",
        "edulite05_fr_driver",
        "edulite05_rl_driver",
        "edulite05_rr_driver",
        "edulite05_spring_driver",
        "edulite05_dribble_position_driver",
    ]

    nodes = []
    for node_name in node_names:
        nodes.append(
            Node(
                package="hardware_driver",
                executable="edulite05_node",
                name=node_name,
                output="screen",
                parameters=[parameter_file],
            )
        )

    return LaunchDescription(nodes)
