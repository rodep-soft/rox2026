import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    default_config = os.path.join(pkg_bringup, "config", "pk_auto_aim.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the PK aim config yaml file",
    )

    test_alignment_only_arg = DeclareLaunchArgument(
        "test_alignment_only",
        default_value="false",
        description="Only test automatic alignment without shooting",
    )

    pk_node = Node(
        package="robot_controller",
        executable="pk_aim",
        name="pk_aim_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"test_alignment_only": LaunchConfiguration("test_alignment_only")},
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            test_alignment_only_arg,
            pk_node,
        ]
    )
