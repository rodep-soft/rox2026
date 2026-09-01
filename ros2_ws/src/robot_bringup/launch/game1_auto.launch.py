import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")

    manual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "manual_robot.launch.py")),
        condition=IfCondition(LaunchConfiguration("enable_manual")),
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
            "enable_foxglove": "false",
            "joy_cmd_vel_topic": "/game1/cmd_vel_raw",
        }.items(),
    )
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "calibrated_apriltag.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_vision")),
        launch_arguments={
            "calibration_file": LaunchConfiguration("calibration_file"),
            "mipi_channel": LaunchConfiguration("mipi_channel"),
            "framerate": LaunchConfiguration("framerate"),
            "mipi_rotation": LaunchConfiguration("mipi_rotation"),
            "roi_width": LaunchConfiguration("roi_width"),
            "roi_height": LaunchConfiguration("roi_height"),
            "tag_size": LaunchConfiguration("tag_size"),
            "allowed_tag_ids": "0,1,10",
            "detector_threads": LaunchConfiguration("detector_threads"),
            "detector_decimate": LaunchConfiguration("detector_decimate"),
            "enable_foxglove": LaunchConfiguration("enable_foxglove"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
        }.items(),
    )

    config_file = LaunchConfiguration("game1_config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_manual",
                default_value="true",
                description="Start hardware, mechanism controllers, EKF and joystick",
            ),
            DeclareLaunchArgument(
                "enable_vision",
                default_value="true",
                description="Start calibrated SC230AI and AprilTag detection",
            ),
            DeclareLaunchArgument(
                "enable_foxglove",
                default_value="false",
                description="Start one Foxglove WebSocket bridge",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove WebSocket port",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by the robot hardware",
            ),
            DeclareLaunchArgument(
                "game1_config_file",
                default_value=os.path.join(
                    bringup_share, "config", "game1_boundary_guard.yaml"
                ),
                description="Boundary guard parameter YAML",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(
                    bringup_share, "config", "camera", "sc230ai_left.yaml"
                ),
                description="SC230AI camera calibration YAML",
            ),
            DeclareLaunchArgument(
                "mipi_channel", default_value="1", description="SC230AI MIPI channel"
            ),
            DeclareLaunchArgument(
                "framerate",
                default_value="10.0",
                description="Camera input rate in frames per second",
            ),
            DeclareLaunchArgument(
                "mipi_rotation",
                default_value="180.0",
                description="Camera image rotation in degrees",
            ),
            DeclareLaunchArgument(
                "roi_width",
                default_value="800",
                description="Centered AprilTag detection ROI width; 0 uses full width",
            ),
            DeclareLaunchArgument(
                "roi_height",
                default_value="1080",
                description="AprilTag detection ROI height; 1080 uses the full image height",
            ),
            DeclareLaunchArgument(
                "tag_size",
                default_value="0.18",
                description="AprilTag edge length in metres",
            ),
            DeclareLaunchArgument(
                "detector_threads",
                default_value="4",
                description="AprilTag detector worker threads",
            ),
            DeclareLaunchArgument(
                "detector_decimate",
                default_value="1.0",
                description="AprilTag input decimation; 1.0 preserves full resolution",
            ),
            manual,
            vision,
            Node(
                package="robot_controller",
                executable="game1_boundary_guard_node",
                name="game1_boundary_guard",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
