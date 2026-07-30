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

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        launch_dir, "controllers", "spring_controller.launch.py"))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        launch_dir, "controllers", "dribble_position_controller.launch.py"))),
        ]
    )
