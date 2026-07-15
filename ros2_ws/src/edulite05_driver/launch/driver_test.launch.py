from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("edulite05_driver"),
        "config",
        "edulite05_single_velocity.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the EDULITE05 parameter YAML file.",
            ),
            Node(
                package="edulite05_driver",
                executable="edulite05_single_velocity_node",
                name="edulite05_single_velocity_node",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
