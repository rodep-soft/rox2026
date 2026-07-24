import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_directory = get_package_share_directory("robot_bringup")
    mecanum_parameter_file = os.path.join(
        bringup_directory,
        "config",
        "mecanum_controller.yaml",
    )

    controller_launches = [
        "spring_controller.launch.py",
        "dribble_controller.launch.py",
        "belt_controller.launch.py",
        "dribble_position_controller.launch.py",
    ]

    controller_includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_directory, "launch", launch_file)
            )
        )
        for launch_file in controller_launches
    ]

    return LaunchDescription(
        [
            Node(
                package="robot_controller",
                executable="mecanum_controller_node",
                name="mecanum_controller_node",
                output="screen",
                parameters=[mecanum_parameter_file],
            ),
            *controller_includes,
        ]
    )
