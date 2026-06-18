from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ros2_practice",
                executable="keyboard_pub",
                name="keyboard_publisher",
            ),
            Node(
                package="ros2_practice",
                executable="keyboard_sub",
                name="text_subscriber",
            ),
        ]
    )
