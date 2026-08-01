import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stm32_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "stm32_driver.yaml",
    )
    vesc_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "vesc_driver.yaml",
    )

    edulite05_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "hardware",
                "edulite05.launch.py",
            )
        ),
        launch_arguments={
            "use_mecanum": LaunchConfiguration("use_edulite_mecanum"),
            "use_spring": LaunchConfiguration("use_edulite_spring"),
            "use_dribble_position": LaunchConfiguration(
                "use_edulite_dribble_position"
            ),
        }.items(),
    )

    socketcan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "ros2_socketcan.launch.py",
            )
        ),
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by ros2_socketcan",
            ),
            DeclareLaunchArgument(
                "use_vesc",
                default_value="true",
                description="Launch the VESC (赤ブラシ) driver nodes",
            ),
            DeclareLaunchArgument(
                "use_stm32",
                default_value="true",
                description="Launch the STM32 driver node",
            ),
            DeclareLaunchArgument(
                "use_edulite_mecanum",
                default_value="true",
                description="Launch the four mecanum EduLite driver nodes",
            ),
            DeclareLaunchArgument(
                "use_edulite_spring",
                default_value="true",
                description="Launch the spring EduLite driver node",
            ),
            DeclareLaunchArgument(
                "use_edulite_dribble_position",
                default_value="true",
                description="Launch the dribble-position EduLite driver node",
            ),
            socketcan_launch,
            Node(
                package="hardware_driver",
                executable="stm32_node",
                name="stm32_driver_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_stm32")),
                parameters=[stm32_parameter_file],
            ),
            edulite05_launch,
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_upper_belt_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_vesc")),
                parameters=[vesc_parameter_file],
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_under_belt_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_vesc")),
                parameters=[vesc_parameter_file],
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_dribble_driver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_vesc")),
                parameters=[vesc_parameter_file],
            ),
        ]
    )
