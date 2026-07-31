import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "edulite05_driver.yaml",
    )

    node_definitions = [
        ("edulite05_fl_driver", "use_mecanum"),
        ("edulite05_fr_driver", "use_mecanum"),
        ("edulite05_rl_driver", "use_mecanum"),
        ("edulite05_rr_driver", "use_mecanum"),
        ("edulite05_spring_driver", "use_spring"),
        ("edulite05_dribble_position_driver", "use_dribble_position"),
    ]

    actions = [
        DeclareLaunchArgument("use_mecanum", default_value="true"),
        DeclareLaunchArgument("use_spring", default_value="true"),
        DeclareLaunchArgument("use_dribble_position", default_value="true"),
    ]
    for node_name, condition_name in node_definitions:
        actions.append(
            Node(
                package="hardware_driver",
                executable="edulite05_node",
                name=node_name,
                output="screen",
                condition=IfCondition(LaunchConfiguration(condition_name)),
                parameters=[parameter_file],
            )
        )

    return LaunchDescription(actions)
