from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MOTOR_CAN_COMMAND_NODE_NAMES = [
    "roller_can_command_node",
    "belt_can_command_node",
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
    config_file = LaunchConfiguration("config_file")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("roller_angle_controller"), "config", "roller_position.yaml"]
    )

    launch_items = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="roller_position.launch.pyで起動する各nodeに渡すparameter yaml",
        ),
        Node(
            package="motor_can_bridge",
            executable="motor_can_packer_node",
            name="motor_can_packer_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="ros2socketcan_bridge",
            executable="ros2socketcan",
            name="ros2socketcan",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="mad_motor",
            executable="belt_controller_node",
            name="belt_controller_node",
            output="screen",
            parameters=[config_file],
        ),
    ]

    launch_items.extend(
        create_motor_can_command_node(node_name, config_file)
        for node_name in MOTOR_CAN_COMMAND_NODE_NAMES
    )

    return LaunchDescription(launch_items)
