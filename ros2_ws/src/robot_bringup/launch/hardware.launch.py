import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")

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
                description="SocketCAN interface",
            ),
            include(
                "hardware/ros2_socketcan.launch.py",
                list({"can_interface": LaunchConfiguration("can_interface")}.items()),
            ),
            # STM32 publishes the complete sensor_msgs/Imu message on /imu/data.
            include("hardware/stm32.launch.py"),
            include("hardware/edulite05.launch.py"),
            include("hardware/vesc.launch.py"),
        ]
    )
