import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
    )

    def include(launch_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file))
        )

    # hardware (ドライバ類・VESC・CAN等)
    hardware_launch = include("hardware.launch.py")

    # 分割された5つの独立コントローラーノード＋入力を一括起動
    launch_files = [
        "controllers/belt_controller.launch.py",
        "controllers/dribbler_controller.launch.py",
        "controllers/arm_position_controller.launch.py",
        "controllers/spring_controller.launch.py",
        "controllers/mecanum_controller.launch.py",
        "input/joy_controller.launch.py",
    ]

    return LaunchDescription(
        [
            hardware_launch,
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
