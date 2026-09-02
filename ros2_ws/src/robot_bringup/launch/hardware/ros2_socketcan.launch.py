import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
                return create_socketcan_nodes(interface_name)
        time.sleep(0.1)

    if not interface_path.exists():
        reason = f"network interface '{interface_name}' does not exist"
    else:
        reason = f"network interface '{interface_name}' is DOWN"
    raise RuntimeError(
        f"SocketCAN startup failed: {reason}. Configure and bring up the CAN interface "
        "before launching the robot."
    )


def create_socketcan_nodes(interface_name):
    receiver = Node(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        output="screen",
        parameters=[
            {
                "interface": interface_name,
                "enable_can_fd": False,
                "interval_sec": 0.005,
                "filters": "0:0",
                "use_bus_time": False,
                # Let the node perform lifecycle transitions synchronously.
                "auto_configure": True,
                "auto_activate": True,
            }
        ],
        remappings=[("from_can_bus", "/socketcan_bridge/rx")],
    )
    stm32_receiver = Node(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver_stm32",
        output="screen",
        parameters=[
            {
                "interface": interface_name,
                "enable_can_fd": False,
                "interval_sec": 0.005,
                # Match only standard STM32 frames in the kernel. Including
                # CAN_EFF_FLAG in the mask excludes extended frames.
                "filters": (
                    "100:800007FF,310:800007FF,320:800007FF,"
                    "321:800007FF,322:800007FF"
                ),
                "use_bus_time": False,
                "auto_configure": True,
                "auto_activate": True,
            }
        ],
        remappings=[("from_can_bus", "/socketcan_bridge/stm32/rx")],
    )
    edulite_receiver = Node(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver_edulite",
        output="screen",
        parameters=[
            {
                "interface": interface_name,
                "enable_can_fd": False,
                "interval_sec": 0.005,
                # Match EduLite extended feedback, parameter-read response,
                # and fault frame types in the kernel.
                "filters": ("02000000:9F000000,11000000:9F000000," "15000000:9F000000"),
                "use_bus_time": False,
                "auto_configure": True,
                "auto_activate": True,
            }
        ],
        remappings=[("from_can_bus", "/socketcan_bridge/edulite/rx")],
    )
    sender = Node(
        package="ros2_socketcan",
        executable="socket_can_sender_node_exe",
        name="socket_can_sender",
        output="screen",
        parameters=[
            {
                "interface": interface_name,
                "enable_can_fd": False,
                "enable_frame_loopback": False,
                "timeout_sec": 0.05,
                # Avoid launch-service races on slower computers.
                "auto_configure": True,
                "auto_activate": True,
            }
        ],
        remappings=[("to_can_bus", "/socketcan_bridge/tx")],
    )
    return [receiver, stm32_receiver, edulite_receiver, sender]


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
