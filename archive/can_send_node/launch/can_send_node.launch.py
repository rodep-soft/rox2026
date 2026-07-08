from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("can_send_node"), "config", "config.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to can_send_node config file",
            ),
            Node(
                package="can_send_node",
                executable="can_send_node",
                name="can_send_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
