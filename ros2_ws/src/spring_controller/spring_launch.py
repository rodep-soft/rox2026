from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="spring_controller",
            executable="spring_joy_sub",
            name="joy_sub",
            output="screen",
        ),

        Node(
            package="spring_controller",
            executable="spring_cal_motor_pub",
            name="spr_mot",
            output="screen",
        ),
    ])