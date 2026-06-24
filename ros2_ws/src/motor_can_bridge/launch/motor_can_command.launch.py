from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NODE_NAMES = [
    "mabuchi_can_command_node",
    "mad_motor_can_command_node",
]


def create_motor_can_command_node(node_name, config_file):
    return Node(
        package="motor_can_bridge",
        executable="motor_can_command_node",
        name=node_name,
        output="screen",
        parameters=[config_file],
    )


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("motor_can_bridge"), "config", "config.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    launch_items = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="motor_can_command_nodeに渡すparameter yaml",
        )
    ]
    launch_items.extend(
        create_motor_can_command_node(node_name, config_file)
        for node_name in NODE_NAMES
    )

    return LaunchDescription(launch_items)
