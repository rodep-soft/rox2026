import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
    )

    def include(launch_file, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
        )

    controller_launch_files = [
        "controllers/belt_controller.launch.py",
        "controllers/dribbler_controller.launch.py",
        "controllers/arm_position_controller.launch.py",
        "controllers/spring_controller.launch.py",
        "controllers/mecanum_controller.launch.py",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Enable BNO055 IMU & Heading PID Controller",
            ),
            include(
                "hardware.launch.py",
                {
                    "can_interface": LaunchConfiguration("can_interface"),
                    "enable_imu": LaunchConfiguration("enable_imu"),
                }.items(),
            ),
            include("input/joy_controller.launch.py"),
            *[include(launch_file) for launch_file in controller_launch_files],
        ]
    )