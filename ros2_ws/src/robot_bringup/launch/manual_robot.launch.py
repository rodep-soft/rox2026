import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        "controllers/dribble_controller.launch.py",
        "controllers/spring_controller.launch.py",
        "controllers/led_controller.launch.py",
        "controllers/mecanum_controller.launch.py",
        "ekf.launch.py",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface",
            ),
            DeclareLaunchArgument(
                "enable_foxglove",
                default_value="true",
                description="Enable Foxglove WebSocket Bridge",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove WebSocket port",
            ),
            include(
                "hardware.launch.py",
                list({"can_interface": LaunchConfiguration("can_interface")}.items()),
            ),
            include("input/joy_controller.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "foxglove_bridge.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("enable_foxglove")),
                launch_arguments={
                    "port": LaunchConfiguration("foxglove_port"),
                }.items(),
            ),
            *[include(launch_file) for launch_file in controller_launch_files],
        ]
    )
