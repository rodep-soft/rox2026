from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
                description="motor_can_packer_nodeに渡すparameter yaml",
            ),
            Node(
                package="motor_can_bridge",
                executable="motor_can_packer_node",
                name="motor_can_packer_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
