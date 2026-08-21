import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("robot_bringup")
    game1_config = os.path.join(bringup_dir, "config", "game1.yaml")
    tag_map_config = os.path.join(bringup_dir, "config", "apriltag_tag_map.yaml")

    side_arg = DeclareLaunchArgument(
        "side",
        default_value="left",
        description="Field side orientation: 'left' (red) or 'right' (blue) for auto Y/Yaw mirroring",
    )

    game1_node = Node(
        package="robot_controller",
        executable="game1_auto_node",
        name="game1_auto_node",
        output="screen",
        parameters=[game1_config, {"field_side": LaunchConfiguration("side")}],
    )

    localizer_node = Node(
        package="robot_controller",
        executable="apriltag_localizer_node",
        name="apriltag_localizer_node",
        output="screen",
        parameters=[tag_map_config, {"field_side": LaunchConfiguration("side")}],
    )

    return LaunchDescription([side_arg, game1_node, localizer_node])
