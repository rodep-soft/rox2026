from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='mecanum_controller',
                executable='mecanum_controller_node',
            ),
            Node(
                package='mecanum_controller',
                executable='base_controller',
            ),
        ]
    )
