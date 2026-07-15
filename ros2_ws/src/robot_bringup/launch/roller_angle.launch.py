from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    roller_config_file = LaunchConfiguration("roller_config_file")
    robstride_config_file = LaunchConfiguration("robstride_config_file")

    default_roller_config_file = PathJoinSubstitution(
        [FindPackageShare("roller_angle_controller"), "config", "roller_config.yaml"]
    )
    default_robstride_config_file = PathJoinSubstitution(
        [FindPackageShare("robstride_can_node"), "config", "config.yaml"]
    )

    roller_position_controller = Node(
        package="roller_angle_controller",
        executable="roller_angle_controller_node",
        name="roller_position_controller",
        output="screen",
        parameters=[roller_config_file],
    )

    robstride_position_node = Node(
        package="robstride_can_node",
        executable="robstride_can_node",
        name="robstride_position_node",
        output="screen",
        parameters=[robstride_config_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "roller_config_file",
                default_value=default_roller_config_file,
                description="roller_angle_controller parameter yaml",
            ),
            DeclareLaunchArgument(
                "robstride_config_file",
                default_value=default_robstride_config_file,
                description="robstride_can_node parameter yaml",
            ),
            roller_position_controller,
            robstride_position_node,
        ]
    )
