import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    stm32_parameter_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'stm32_driver.yaml',
    )

    edulite05_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'launch',
                'edulite05.launch.py',
            )
        )
    )

    return LaunchDescription(
        [
            Node(
                package='hardware_driver',
                executable='stm32_node',
                name='stm32_driver_node',
                output='screen',
                parameters=[stm32_parameter_file],
            ),
            edulite05_launch,
        ]
    )
