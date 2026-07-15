import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2socketcan_bridge",
            executable="ros2socketcan",
            output="screen",
        ),
    ])