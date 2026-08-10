import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    navigation_share = get_package_share_directory("rox_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    sensor_config = os.path.join(bringup_share, "config", "sensors.yaml")
    ekf_config = os.path.join(bringup_share, "config", "navigate", "ekf.yaml")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(navigation_share, "urdf", "rox2026.urdf.xacro")]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    bringup_share, "config", "navigate", "rox2026.rviz"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time}
                ],
                output="screen",
            ),
            # Wheel odometry is started with the mecanum controller bringup.
            Node(
                package="rox_navigation",
                executable="tag_localization_node",
                parameters=[sensor_config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_odom",
                parameters=[ekf_config, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/local")],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_map",
                parameters=[ekf_config, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odometry/global")],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
