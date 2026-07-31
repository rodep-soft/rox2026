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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by ros2_socketcan",
            ),
            # belt・dribble回転用VESCだけを起動する。
            include(
                "hardware.launch.py",
                launch_arguments={
                    "can_interface": LaunchConfiguration("can_interface"),
                    "use_vesc": "true",
                    "use_stm32": "false",
                    "use_edulite_mecanum": "false",
                    "use_edulite_spring": "false",
                    "use_edulite_dribble_position": "false",
                }.items(),
            ),
            include("input/joy_controller.launch.py"),
            # この構成の対象controller(belt + dribble)
            include("controllers/belt_dribble.launch.py"),
        ]
    )
