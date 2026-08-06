import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    base_frame = LaunchConfiguration("base_frame").perform(context)
    rpm_bottom = float(LaunchConfiguration("rpm_bottom").perform(context))
    rpm_middle = float(LaunchConfiguration("rpm_middle").perform(context))
    rpm_top = float(LaunchConfiguration("rpm_top").perform(context))
    target_distance = float(LaunchConfiguration("target_distance").perform(context))

    bingo_node = Node(
        package="robot_controller",
        executable="bingo_tactical_shooter_node",
        name="bingo_tactical_shooter_node",
        output="screen",
        parameters=[
            {
                "base_frame": base_frame,
                "target_distance": target_distance,
                "rpm_bottom": rpm_bottom,
                "rpm_middle": rpm_middle,
                "rpm_top": rpm_top,
            }
        ],
    )

    return [bingo_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame ID",
            ),
            DeclareLaunchArgument(
                "target_distance",
                default_value="1.5",
                description="Target shooting distance from panels in meters",
            ),
            DeclareLaunchArgument(
                "rpm_bottom",
                default_value="3000.0",
                description="Shooting belt RPM for bottom row panels",
            ),
            DeclareLaunchArgument(
                "rpm_middle",
                default_value="4500.0",
                description="Shooting belt RPM for middle row panels",
            ),
            DeclareLaunchArgument(
                "rpm_top",
                default_value="6000.0",
                description="Shooting belt RPM for top row panels",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
