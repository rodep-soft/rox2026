import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'edulite05_driver.yaml',
    )

    nodes = []
    for i in range(1, 7):
        nodes.append(
            Node(
                package='hardware_driver',
                executable='edulite05_node',
                name=f'edulite05_driver_node_{i}',
                output='screen',
                parameters=[parameter_file],
            )
        )

    return LaunchDescription(nodes)
