import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")

    hardware_container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "hardware_container.launch.py")
        )
    )

    # libbno055_linux 高性能 IMU ドライバ ＋ 角度補正 (Heading PID Controller)
    bno055_share = get_package_share_directory("libbno055_linux")
    
    imu_heading_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bno055_share, "launch", "heading_control_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_imu")),
        launch_arguments={
            "use_composition": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Enable BNO055 IMU & Heading PID Controller (libbno055_linux)",
            ),
            hardware_container_launch,
            imu_heading_launch,
        ]
    )
