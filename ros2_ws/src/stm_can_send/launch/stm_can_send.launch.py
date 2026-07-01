from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("stm_can_send"), "config", "config.yaml"]
    )
    config_file = LaunchConfiguration("config_file")
    roller_enabled = LaunchConfiguration("roller_enabled")
    belt_enabled = LaunchConfiguration("belt_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to stm_can_send config file",
            ),
            DeclareLaunchArgument(
                "roller_enabled",
                default_value="true",
                description="Enable roller CAN frame input",
            ),
            DeclareLaunchArgument(
                "belt_enabled",
                default_value="true",
                description="Enable belt CAN frame input",
            ),
            Node(
                package="stm_can_send",
                executable="stm_can_send_node",
                name="stm_can_send_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "roller_enabled": ParameterValue(roller_enabled, value_type=bool),
                        "belt_enabled": ParameterValue(belt_enabled, value_type=bool),
                    },
                ],
            ),
        ]
    )
