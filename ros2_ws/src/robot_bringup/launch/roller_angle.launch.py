from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "roller_angle.yaml"]
    )

    roller_position_controller = Node(
        package="roller_angle_controller",
        executable="roller_angle_controller_node",
        name="roller_position_controller",
        output="screen",
        parameters=[config_file],
    )

    robstride_position_node = Node(
        package="robstride_can_node",
        executable="robstride_can_node",
        name="robstride_position_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="roller_angle_controller and robstride_can_node parameter yaml",
            ),
            roller_position_controller,
            robstride_position_node,
        ]
    )
