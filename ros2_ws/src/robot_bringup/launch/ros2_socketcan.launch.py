import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    canonical_launch = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
        "hardware",
        "ros2_socketcan.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("can_startup_timeout_sec", default_value="5.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical_launch),
                launch_arguments={
                    "can_interface": LaunchConfiguration("can_interface"),
                    "can_startup_timeout_sec": LaunchConfiguration(
                        "can_startup_timeout_sec"
                    ),
                }.items(),
            ),
        ]
    )
