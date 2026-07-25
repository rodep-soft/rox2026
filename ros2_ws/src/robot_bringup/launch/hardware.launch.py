import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stm32_parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "stm32_driver.yaml",
    )

    edulite05_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "edulite05.launch.py",
            )
        )
    )

    socketcan_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_socketcan"),
                "launch",
                "socket_can_bridge.launch.xml",
            )
        ),
        launch_arguments={
            "interface": LaunchConfiguration("can_interface"),
            "enable_can_fd": "false",
            "from_can_bus_topic": "/socketcan_bridge/rx",
            "to_can_bus_topic": "/socketcan_bridge/tx",
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
                "vesc_1_controller_id",
                default_value="51",
                description="CAN controller ID of VESC 1",
            ),
            DeclareLaunchArgument(
                "vesc_2_controller_id",
                default_value="2",
                description="CAN controller ID of VESC 2",
            ),
            DeclareLaunchArgument(
                "vesc_3_controller_id",
                default_value="3",
                description="CAN controller ID of VESC 3",
            ),
            socketcan_launch,
            Node(
                package="hardware_driver",
                executable="stm32_node",
                name="stm32_driver_node",
                output="screen",
                parameters=[stm32_parameter_file],
            ),
            edulite05_launch,
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_driver_1",
                output="screen",
                parameters=[
                    {
                        "controller_id": LaunchConfiguration("vesc_1_controller_id"),
                        "target_rpm_topic": "/vesc_1/target/rpm",
                        "current_rpm_topic": "/vesc_1/current/rpm",
                    }
                ],
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_driver_2",
                output="screen",
                parameters=[
                    {
                        "controller_id": LaunchConfiguration("vesc_2_controller_id"),
                        "target_rpm_topic": "/vesc_2/target/rpm",
                        "current_rpm_topic": "/vesc_2/current/rpm",
                    }
                ],
            ),
            Node(
                package="hardware_driver",
                executable="vesc_node",
                name="vesc_driver_3",
                output="screen",
                parameters=[
                    {
                        "controller_id": LaunchConfiguration("vesc_3_controller_id"),
                        "target_rpm_topic": "/vesc_3/target/rpm",
                        "current_rpm_topic": "/vesc_3/current/rpm",
                    }
                ],
            ),
        ]
    )
