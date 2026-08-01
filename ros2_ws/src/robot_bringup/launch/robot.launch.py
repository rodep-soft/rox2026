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

    # hardware(driver類)。belt・dribble用のVESC 3台も起動する。
    hardware_launch = include(
        "hardware.launch.py"
    )

    # controllerと操作系。調整値はrobot_bringup/config配下のyamlで管理する。
    # beltとdribbleはbelt_dribble.launch.pyで一緒に起動する。
    launch_files = [
        "controllers/belt_dribble.launch.py",
        "controllers/mecanum_controller.launch.py",
        "controllers/spring_dribble_position.launch.py",
        "input/joy_controller.launch.py",
    ]

    return LaunchDescription(
        [
            hardware_launch,
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
