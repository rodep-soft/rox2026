import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    base_frame = LaunchConfiguration("base_frame").perform(context)
    kp_yaw = float(LaunchConfiguration("kp_yaw").perform(context))
    max_angular_z = float(LaunchConfiguration("max_angular_z").perform(context))
    rpm_bottom = float(LaunchConfiguration("rpm_bottom").perform(context))
    rpm_middle = float(LaunchConfiguration("rpm_middle").perform(context))
    rpm_top = float(LaunchConfiguration("rpm_top").perform(context))
    target_distance = float(LaunchConfiguration("target_distance").perform(context))

    game2_node = Node(
        package="robot_controller",
        executable="game2_auto_node",
        name="game2_auto_node",
        output="screen",
        parameters=[
            {
                "base_frame": base_frame,
                "kp_yaw": kp_yaw,
                "max_angular_z": max_angular_z,
                "target_distance": target_distance,
                "rpm_bottom": rpm_bottom,
                "rpm_middle": rpm_middle,
                "rpm_top": rpm_top,
            }
        ],
    )

    return [game2_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame ID",
            ),
            DeclareLaunchArgument(
                "kp_yaw",
                default_value="0.5",
                description="Low-sensitivity yaw rotation P-gain for Game 2 targeting",
            ),
            DeclareLaunchArgument(
                "max_angular_z",
                default_value="0.35",
                description="Maximum rotation velocity limit in rad/s",
            ),
            DeclareLaunchArgument(
                "target_distance",
                default_value="4.0",
                description="Target shooting distance from Game2 panels in meters",
            ),
            DeclareLaunchArgument(
                "rpm_bottom",
                default_value="3000.0",
                description="Shooting belt RPM for Game2 bottom row panels",
            ),
            DeclareLaunchArgument(
                "rpm_middle",
                default_value="4500.0",
                description="Shooting belt RPM for Game2 middle row panels",
            ),
            DeclareLaunchArgument(
                "rpm_top",
                default_value="6000.0",
                description="Shooting belt RPM for Game2 top row panels",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
