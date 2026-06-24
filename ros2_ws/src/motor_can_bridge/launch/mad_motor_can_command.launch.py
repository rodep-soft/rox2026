from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NODE_NAME = "mad_motor_can_command_node"


def create_motor_can_command_node(config_file):
    return Node(
        package="motor_can_bridge",
        executable="motor_can_command_node",
        name=NODE_NAME,
        output="screen",
        parameters=[config_file],
    )


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("motor_can_bridge"), "config", "config.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description=f"{NODE_NAME}に渡すparameter yaml",
            ),
            create_motor_can_command_node(config_file),
        ]
    )
