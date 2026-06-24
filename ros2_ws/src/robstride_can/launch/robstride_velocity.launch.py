from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robstride_can"), "config", "robstride_velocity.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="robstride_velocity_nodeに渡すparameter yaml",
            ),
            Node(
                package="robstride_can",
                executable="robstride_velocity_node",
                name="robstride_velocity_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
