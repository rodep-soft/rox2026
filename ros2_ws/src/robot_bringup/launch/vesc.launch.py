from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    vesc_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "vesc_driver.yaml"
    )

    node_names = [
        "vesc_upper_belt_driver",
        "vesc_under_belt_driver",
        "vesc_dribble_belt_driver",
    ]

    nodes = []
    for node_name in node_names:
        nodes.append(
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name=node_name,
                output="screen",
                parameters=[vesc_parameter_file],
            )
        )
    return LaunchDescription(nodes)
