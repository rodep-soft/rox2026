import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "stm32.launch.py",
            )
        )
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
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "vesc.launch.py",
            )
        ),
        launch_arguments={
            "use_vesc": LaunchConfiguration("use_vesc"),
            "use_dribble_vesc": LaunchConfiguration("use_dribble_vesc"),
            "vesc_1_controller_id": LaunchConfiguration("vesc_1_controller_id"),
            "vesc_2_controller_id": LaunchConfiguration("vesc_2_controller_id"),
            "vesc_3_controller_id": LaunchConfiguration("vesc_3_controller_id"),
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
            stm32_launch,
            edulite05_launch,
            vesc_launch,
            socketcan_launch,
        ]
    )
