import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("robot_bringup")
    config_path = os.path.join(bringup_dir, "config", "game1_shooter.yaml")

    node = Node(
        package="robot_controller",
        executable="game1_auto_shooter_node",
        name="game1_auto_shooter_node",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([node])
