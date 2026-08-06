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

    # libbno055_linux IMU ドライバ (RDK X5 /dev/i2c-5 バス対応)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("libbno055_linux"),
                "launch",
                "bno055_launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_imu")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Enable BNO055 IMU driver node (libbno055_linux)",
            ),
            hardware_container_launch,
            imu_launch,
        ]
    )
