import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_directory = get_package_share_directory("robot_bringup")
    joy_parameter_file = os.path.join(
        bringup_directory,
        "config",
        "joy_controller.yaml",
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_directory, "launch", "hardware.launch.py")
        )
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": "/dev/input/js0", "autorepeat_rate": 100.0}],
            ),
            Node(
                package="joy_controller",
                executable="joy_controller_node",
                name="joy_controller",
                output="screen",
                parameters=[joy_parameter_file],
            ),
            hardware_launch,
        ]
    )
