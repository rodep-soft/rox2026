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

    def include(launch_file, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            **kwargs,
        )

    # hardware (CAN, STM32 IMU, VESC and EduLite drivers)
    hardware_launch = include("hardware/hardware.launch.py")

    # 拡張カルマンフィルタ (EKF) ノード
    ekf_launch = include(
        "ekf.launch.py",
        condition=IfCondition(LaunchConfiguration("enable_ekf")),
    )

    # game3 は専用のベルト設定を使用する
    launch_files = [
        "controllers/game3_belt_controller.launch.py",
        "controllers/dribble_controller.launch.py",
        "controllers/spring_controller.launch.py",
        "controllers/mecanum_controller.launch.py",
        "controllers/led_controller.launch.py",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_ekf",
                default_value="true",
                description="Enable Extended Kalman Filter (robot_localization EKF)",
            ),
            DeclareLaunchArgument(
                "enable_foxglove",
                default_value="true",
                description="Enable Foxglove WebSocket Bridge (port 8765)",
            ),
            hardware_launch,
            ekf_launch,
            include(
                "input/joy_controller.launch.py",
                launch_arguments={"config_file": "game3_joy_controller.yaml"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "foxglove_bridge.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("enable_foxglove")),
            ),
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
