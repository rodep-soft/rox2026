import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    mecanum_parameter_file = os.path.join(
        bringup_share,
        "config",
        "mecanum_controller.yaml",
    )
    heading_hold_parameter_file = os.path.join(
        bringup_share,
        "config",
        "heading_hold.yaml",
    )
    odometry_parameter_file = os.path.join(
        bringup_share,
        "config",
        "odometry.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="robot_controller",
                executable="heading_hold_node",
                name="heading_hold_node",
                output="screen",
                parameters=[heading_hold_parameter_file],
            ),
            Node(
                package="robot_controller",
                executable="mecanum_controller_node",
                name="mecanum_controller_node",
                output="screen",
                parameters=[mecanum_parameter_file],
            ),
            Node(
                package="robot_controller",
                executable="mecanum_odometry_node",
                name="mecanum_odometry_node",
                output="screen",
                parameters=[odometry_parameter_file],
            ),
        ]
    )
