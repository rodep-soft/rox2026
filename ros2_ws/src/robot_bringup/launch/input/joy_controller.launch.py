import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
    )
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "joy_controller.yaml",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "input", "joy.launch.py")
                ),
            ),
            Node(
                package="joy_controller",
                executable="joy_controller_node",
                name="joy_controller",
                output="screen",
                parameters=[parameter_file],
            ),
        ]
    )
