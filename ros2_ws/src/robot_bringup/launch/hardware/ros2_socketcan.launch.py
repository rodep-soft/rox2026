import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    socketcan_launch_directory = os.path.join(
        get_package_share_directory("ros2_socketcan"),
        "launch",
    )

    socketcan_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                socketcan_launch_directory,
                "socket_can_receiver.launch.py",
            )
        ),
        launch_arguments={
            "interface": LaunchConfiguration("can_interface"),
            "enable_can_fd": "false",
            "interval_sec": "0.005",
            "use_bus_time": "false",
            "from_can_bus_topic": "/socketcan_bridge/rx",
            # "filters": "",
        }.items(),
    )

    socketcan_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                socketcan_launch_directory,
                "socket_can_sender.launch.py",
            )
        ),
        launch_arguments={
            "interface": LaunchConfiguration("can_interface"),
            "enable_can_fd": "false",
            "enable_frame_loopback": "false",
            "timeout_sec": "0.01",
            "to_can_bus_topic": "/socketcan_bridge/tx",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface",
            ),
            socketcan_receiver_launch,
            socketcan_sender_launch,
        ]
    )
