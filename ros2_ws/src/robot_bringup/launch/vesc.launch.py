import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vesc_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "vesc_driver.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_vesc",
                default_value="true",
                description="Launch the underbelt and upperbelt VESC driver nodes",
            ),
            DeclareLaunchArgument(
                "use_dribble_vesc",
                default_value="false",
                description="Launch the dribble VESC driver node",
            ),
            DeclareLaunchArgument(
                "vesc_1_controller_id",
                default_value="51",
                description="CAN controller ID of the underbelt VESC",
            ),
            DeclareLaunchArgument(
                "vesc_2_controller_id",
                default_value="52",
                description="CAN controller ID of the upperbelt VESC",
            ),
            DeclareLaunchArgument(
                "vesc_3_controller_id",
                default_value="50",
                description="CAN controller ID of the dribble VESC",
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_under_belt_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_vesc")),
                parameters=[
                    vesc_parameter_file,
                    {"controller_id": LaunchConfiguration("vesc_1_controller_id")},
                ],
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_upper_belt_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_vesc")),
                parameters=[
                    vesc_parameter_file,
                    {"controller_id": LaunchConfiguration("vesc_2_controller_id")},
                ],
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_dribble_belt_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_dribble_vesc")),
                parameters=[
                    vesc_parameter_file,
                    {"controller_id": LaunchConfiguration("vesc_3_controller_id")},
                ],
            ),
        ]
    )
