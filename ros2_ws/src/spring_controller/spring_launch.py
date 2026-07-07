from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="spring_controller",
            executable="subscriber",
            name="joy_sub",
            output="screen",
        ),

        Node(
            package="spring_controller",
            executable="motor",
            name="spr_mot",
            output="screen",
        ),
    ])