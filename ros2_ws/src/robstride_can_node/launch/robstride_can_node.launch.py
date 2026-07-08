from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NODE_NAMES = [
    "robstride_position_node",
    "robstride_velocity_node",
]


def create_robstride_can_node(node_name, config_file):
    return Node(
        package="robstride_can_node",
        executable="robstride_can_node",
        name=node_name,
        output="screen",
        parameters=[config_file],
    )


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robstride_can_node"), "config", "config.yaml"]
    )
    config_file = LaunchConfiguration("config_file")

    launch_items = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="robstride_can_nodeに渡すparameter yaml",
        )
    ]
    launch_items.extend(
        create_robstride_can_node(node_name, config_file) for node_name in NODE_NAMES
    )

    return LaunchDescription(launch_items)
