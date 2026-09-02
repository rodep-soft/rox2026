import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
    )
    parameter_file = PathJoinSubstitution(
        [
            get_package_share_directory("robot_bringup"),
            "config",
            LaunchConfiguration("config_file"),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="joy_controller.yaml",
                description="Joy controller parameter file in robot_bringup/config",
            ),
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
