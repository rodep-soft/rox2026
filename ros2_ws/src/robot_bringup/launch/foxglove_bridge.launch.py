from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="WebSocket port for Foxglove Studio",
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "address": "0.0.0.0",
                "send_buffer_limit": 10000000,
                "use_sim_time": False,
            }
        ],
    )

    return LaunchDescription([port_arg, foxglove_node])
