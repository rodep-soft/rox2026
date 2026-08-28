import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    default_config = os.path.join(pkg_bringup, "config", "game2_auto_aim.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the game2 auto aim config yaml file",
    )

    game2_node = Node(
        package="robot_controller",
        executable="game2_aim",
        name="game2_aim",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_file_arg, game2_node])
