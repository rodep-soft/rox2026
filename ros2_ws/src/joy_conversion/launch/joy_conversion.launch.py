import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy_conversion",
                executable="joy_conversion_node",
                name="joy_conversion_node",
                parameters=[
                    os.path.join(
                        get_package_share_directory("joy_conversion"),
                        "config",
                        "joy_conversion_node.yaml",
                    )
                ],
            )
        ]
    )
