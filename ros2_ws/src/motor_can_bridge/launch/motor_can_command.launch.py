from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("motor_can_bridge"), "config", "config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="motor_can_bridge",
                executable="motor_can_command_node",
                name="mabuchi_can_command_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="motor_can_bridge",
                executable="motor_can_command_node",
                name="mad_motor_can_command_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
