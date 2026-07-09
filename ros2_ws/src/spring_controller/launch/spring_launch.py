from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare("spring_controller"),
        "config",
        "spring_param.yaml"
    ])

    return LaunchDescription([
        Node(
            package="spring_controller",
            executable="spring_joy_sub",
            name="joy_sub",
            parameters=[config_file],
            output="screen",
        ),

        Node(
            package="spring_controller",
            executable="spring_cal_motor_pub",
            name="spr_mot",
            parameters=[config_file],
            output="screen",
        ),
    ])