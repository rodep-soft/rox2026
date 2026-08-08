import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")

    def include(launch_file, launch_arguments=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
            condition=condition,
        )

    socketcan_launch = include(
        "hardware/ros2_socketcan.launch.py",
        {"can_interface": LaunchConfiguration("can_interface")}.items(),
    )
    stm32_launch = include("hardware/stm32.launch.py")
    edulite05_launch = include("hardware/edulite05.launch.py")
    vesc_launch = include("hardware/vesc.launch.py")

    # libbno055_linux 高性能 IMU ドライバ ＋ 角度補正 (Heading PID Controller)
    bno055_share = get_package_share_directory("libbno055_linux")
    imu_heading_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bno055_share, "launch", "heading_control_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_imu")),
        launch_arguments={"use_composition": "true"}.items(),
    )

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
                description="Enable BNO055 IMU & Heading PID Controller (libbno055_linux)",
            ),
            socketcan_launch,
            stm32_launch,
            edulite05_launch,
            vesc_launch,
            imu_heading_launch,
        ]
    )