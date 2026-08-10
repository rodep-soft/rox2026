import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


IFF_UP = 0x1


def wait_for_can_interface(context):
    interface_name = LaunchConfiguration("can_interface").perform(context)
    timeout_sec = float(LaunchConfiguration("can_startup_timeout_sec").perform(context))
    interface_path = Path("/sys/class/net") / interface_name
    flags_path = interface_path / "flags"
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() <= deadline:
        if flags_path.exists():
            flags = int(flags_path.read_text(encoding="utf-8").strip(), 16)
            if flags & IFF_UP:
                return create_socketcan_launch_actions(interface_name)
        time.sleep(0.1)

    if not interface_path.exists():
        reason = f"network interface '{interface_name}' does not exist"
    else:
        reason = f"network interface '{interface_name}' is DOWN"
    raise RuntimeError(
        f"SocketCAN startup failed: {reason}. Configure and bring up the CAN interface "
        "before launching the robot."
    )


def create_socketcan_launch_actions(interface_name):
    socketcan_launch_directory = os.path.join(
        get_package_share_directory("ros2_socketcan"),
        "launch",
    )
    receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(socketcan_launch_directory, "socket_can_receiver.launch.py")
        ),
        launch_arguments={
            "interface": interface_name,
            "enable_can_fd": "false",
            "interval_sec": "0.005",
            "use_bus_time": "false",
            "from_can_bus_topic": "/socketcan_bridge/rx",
        }.items(),
    )
    sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(socketcan_launch_directory, "socket_can_sender.launch.py")
        ),
        launch_arguments={
            "interface": interface_name,
            "enable_can_fd": "false",
            "enable_frame_loopback": "false",
            "timeout_sec": "0.05",
            "to_can_bus_topic": "/socketcan_bridge/tx",
        }.items(),
    )
    return [receiver, sender]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface",
            ),
            DeclareLaunchArgument(
                "can_startup_timeout_sec",
                default_value="5.0",
                description="Time to wait for the SocketCAN interface to become UP",
            ),
            OpaqueFunction(function=wait_for_can_interface),
        ]
    )
