import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "stm32.launch.py",
            )
        )
    )
    edulite05_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "edulite05.launch.py",
            )
        )
    )
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "vesc.launch.py",
            )
        )
    )
    socketcan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "ros2_socketcan.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            stm32_launch,
            edulite05_launch,
            vesc_launch,
            socketcan_launch,
        ]
    )
