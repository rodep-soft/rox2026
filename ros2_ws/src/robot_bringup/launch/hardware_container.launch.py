import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")

    edulite05_config = os.path.join(bringup_share, "config", "edulite05_driver.yaml")
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

    edulite_node_names = [
        "edulite05_fl_driver",
        "edulite05_fr_driver",
        "edulite05_rl_driver",
        "edulite05_rr_driver",
        "edulite05_spring_driver",
        "edulite05_dribble_position_driver",
    ]

    for name in edulite_node_names:
        composable_nodes.append(
            ComposableNode(
                package="hardware_driver",
                plugin="Ed05DriverNode",
                name=name,
                parameters=[edulite05_config],
            )
        )

    vesc_node_names = [
        "vesc_upper_belt_driver",
        "vesc_under_belt_driver",
        "vesc_dribble_driver",
    ]

    for name in vesc_node_names:
        composable_nodes.append(
            ComposableNode(
                package="hardware_driver",
                plugin="vesc_driver::Node",
                name=name,
                parameters=[vesc_config],
            )
        )

    container = ComposableNodeContainer(
        name="hardware_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return LaunchDescription([can_interface_arg, container])
