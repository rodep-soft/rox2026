from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    stm32_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "stm32_node_param.yaml",
    )

    edulite05_parameter_file = os.path.join(
            get_package_share_directory("robot_bringup"),
            "config",
            "edulite05_node_param.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="hardware_driver",
                executable="stm32_node",
                name="stm32_driver_node",
                output="screen",
                parameters=[stm32_parameter_file],
            )

            Node(
                package="hardware_driver",
                executable="edulite05_node",
                name="edulite05_driver_node",
                output="screen",
                parameters=[edulite05_parameter_file],
            )
        ]
    )
