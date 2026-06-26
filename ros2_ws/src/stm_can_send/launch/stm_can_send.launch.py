from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("stm_can_send"), "config", "config.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to stm_can_send config file",
            ),
            Node(
                package="stm_can_send",
                executable="stm_can_send_node",
                name="stm_can_send_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
