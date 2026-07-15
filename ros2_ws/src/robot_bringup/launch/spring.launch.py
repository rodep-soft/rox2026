from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "spring_param.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="spring_controller",
                executable="spring_joy_sub",
                name="spring_joy_sub",
                parameters=[config_file],
                output="screen",
            ),
            Node(
                package="spring_controller",
                executable="spring_edulite_controller",
                name="spring_edulite_controller",
                parameters=[config_file],
                output="screen",
            ),
        ]
    )
