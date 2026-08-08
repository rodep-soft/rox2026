from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{
                    'device_name': 'PS5 Controller',
                    "autorepeat_rate": 100.0,
                    "coalesce_interval_ms": 10
                }],
            ),
        ]
    )
