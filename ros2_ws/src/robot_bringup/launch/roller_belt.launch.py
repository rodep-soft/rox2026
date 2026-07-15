from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "roller_belt.yaml"]
    )

    roller_controller_node = Node(
        package="roller_controller",
        executable="roller_controller_node",
        name="roller_controller_node",
        output="screen",
        parameters=[config_file],
    )

    belt_controller_node = Node(
        package="belt_controller",
        executable="belt_controller_node",
        name="belt_controller_node",
        output="screen",
        parameters=[config_file],
    )

    roller_belt_can_packer_node = Node(
        package="roller_belt_can_packer",
        executable="roller_belt_can_packer_node",
        name="roller_belt_can_packer_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="belt PWM and STM CAN packer parameter yaml",
            ),
            roller_controller_node,
            belt_controller_node,
            roller_belt_can_packer_node,
        ]
    )
