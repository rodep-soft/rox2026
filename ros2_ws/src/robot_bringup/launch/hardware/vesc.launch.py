from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    vesc_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "vesc_driver.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_driver",
                output="screen",
                parameters=[vesc_parameter_file],
            )
        ]
    )