import subprocess
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
    auto_reset = LaunchConfiguration("auto_reset_can").perform(context).lower() == "true"

    if auto_reset:
        try:
            # TCAN4550 (TI SPI-CAN) の内部PLLとLDOが安定化するよう 0.2s のディレイを設けてリセット
            subprocess.run(["sudo", "ip", "link", "set", interface_name, "down"], capture_output=True, timeout=2.0)
            time.sleep(0.2)
            subprocess.run(["sudo", "ip", "link", "set", interface_name, "type", "can", "bitrate", "1000000", "restart-ms", "100"], capture_output=True, timeout=2.0)
            subprocess.run(["sudo", "ip", "link", "set", interface_name, "txqueuelen", "1000"], capture_output=True, timeout=2.0)
            subprocess.run(["sudo", "ip", "link", "set", interface_name, "up"], capture_output=True, timeout=2.0)
        except Exception:
            pass

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
                "timeout_sec": 0.20,
                # Avoid launch-service races on slower computers.
                "auto_configure": True,
                "auto_activate": True,
            }
        ],
        remappings=[("to_can_bus", "/socketcan_bridge/tx")],
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
            DeclareLaunchArgument(
                "auto_reset_can",
                default_value="true",
                description="Automatically flush and reset CAN interface on launch",
            ),
            OpaqueFunction(function=wait_for_can_interface),
        ]
    )
