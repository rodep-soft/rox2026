import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    default_config = os.path.join(pkg_bringup, "config", "game2_auto_aim.yaml")
    belt_config = os.path.join(pkg_bringup, "config", "belt_controller.yaml")
    joy_config = os.path.join(pkg_bringup, "config", "joy_controller.yaml")

    belt_params = {}
    if os.path.exists(belt_config):
        with open(belt_config, "r") as f:
            data = yaml.safe_load(f)
            if data and "/belt_controller_node" in data:
                belt_params = data["/belt_controller_node"].get("ros__parameters", {})

    joy_params = {}
    if os.path.exists(joy_config):
        with open(joy_config, "r") as f:
            data = yaml.safe_load(f)
            if data and "/joy_controller" in data:
                joy_params = data["/joy_controller"].get("ros__parameters", {})

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
        parameters=[LaunchConfiguration("config_file"), belt_params, joy_params],
    )

    return LaunchDescription([config_file_arg, game2_node])
