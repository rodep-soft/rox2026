import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")

    edulite05_config = os.path.join(
        bringup_share, "config", "edulite05_driver_v2.yaml"
    )
    vesc_config = os.path.join(bringup_share, "config", "vesc_driver.yaml")
    stm32_config = os.path.join(bringup_share, "config", "stm32_driver.yaml")

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface",
    )

    composable_nodes = [
        ComposableNode(
            package="ros2_socketcan",
            plugin="drivers::socketcan::SocketCanReceiverNode",
            name="socket_can_receiver",
            parameters=[
                {
                    "interface": LaunchConfiguration("can_interface"),
                    "enable_can_fd": False,
                    "interval_sec": 0.005,
                    "use_bus_time": False,
                }
            ],
            remappings=[("from_can_bus", "/socketcan_bridge/rx")],
        ),
        ComposableNode(
            package="ros2_socketcan",
            plugin="drivers::socketcan::SocketCanSenderNode",
            name="socket_can_sender",
            parameters=[
                {
                    "interface": LaunchConfiguration("can_interface"),
                    "enable_can_fd": False,
                    "enable_frame_loopback": False,
                    "timeout_sec": 0.05,
                }
            ],
            remappings=[("to_can_bus", "/socketcan_bridge/tx")],
        ),
        ComposableNode(
            package="hardware_driver",
            plugin="stm32_driver::Stm32Node",
            name="stm32_driver_node",
            parameters=[stm32_config],
        ),
    ]

    container = ComposableNodeContainer(
        name="hardware_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    vesc_node = Node(
        package="hardware_driver",
        executable="vesc_node",
        name="vesc_driver",
        output="screen",
        parameters=[vesc_config],
    )
    # EduLite 05 driver is a standalone multi-motor node, not an rclcpp component.
    edulite05_node = Node(
        package="hardware_driver",
        executable="edulite05_node",
        name="edulite05_driver",
        output="screen",
        parameters=[edulite05_config],
    )

    return LaunchDescription([can_interface_arg, container, vesc_node, edulite05_node])
