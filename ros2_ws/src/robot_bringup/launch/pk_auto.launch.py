import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")

    def include(launch_file, condition, launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            condition=condition,
            launch_arguments=launch_arguments.items(),
        )

    manual_launch = include(
        "manual_robot.launch.py",
        IfCondition(LaunchConfiguration("enable_manual")),
        {
            "can_interface": LaunchConfiguration("can_interface"),
            "enable_foxglove": "false",
        },
    )

    vision_launch = include(
        "calibrated_apriltag.launch.py",
        IfCondition(LaunchConfiguration("enable_vision")),
        {
            "calibration_file": LaunchConfiguration("calibration_file"),
            "mipi_channel": LaunchConfiguration("mipi_channel"),
            "framerate": LaunchConfiguration("framerate"),
            "mipi_rotation": LaunchConfiguration("mipi_rotation"),
            "roi_width": LaunchConfiguration("roi_width"),
            "roi_height": LaunchConfiguration("roi_height"),
            "tag_size": LaunchConfiguration("tag_size"),
            "detector_threads": LaunchConfiguration("detector_threads"),
            "detector_decimate": LaunchConfiguration("detector_decimate"),
            "enable_foxglove": LaunchConfiguration("enable_foxglove"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
        },
    )

    pk_launch = include(
        "controllers/pk_aim.launch.py",
        IfCondition(LaunchConfiguration("enable_pk_auto")),
        {
            "config_file": LaunchConfiguration("pk_config_file"),
            "test_alignment_only": LaunchConfiguration("test_alignment_only"),
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_manual",
                default_value="true",
                description="Start CAN hardware, manual controllers, joystick and EKF",
            ),
            DeclareLaunchArgument(
                "enable_vision",
                default_value="true",
                description="Start calibrated SC230AI and AprilTag detection",
            ),
            DeclareLaunchArgument(
                "enable_pk_auto",
                default_value="true",
                description="Start pk_aim through pk_aim.launch.py",
            ),
            DeclareLaunchArgument(
                "enable_foxglove",
                default_value="false",
                description="Start one Foxglove WebSocket bridge",
            ),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument(
                "pk_config_file",
                default_value=os.path.join(
                    bringup_share, "config", "pk_aim.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(
                    bringup_share, "config", "camera", "sc230ai_left.yaml"
                ),
            ),
            DeclareLaunchArgument("mipi_channel", default_value="1"),
            DeclareLaunchArgument("framerate", default_value="10.0"),
            DeclareLaunchArgument("mipi_rotation", default_value="180.0"),
            DeclareLaunchArgument("roi_width", default_value="800"),
            DeclareLaunchArgument("roi_height", default_value="480"),
            DeclareLaunchArgument("tag_size", default_value="0.18"),
            DeclareLaunchArgument("detector_threads", default_value="4"),
            DeclareLaunchArgument("detector_decimate", default_value="1.0"),
            DeclareLaunchArgument(
                "test_alignment_only",
                default_value="false",
                description="Align only; do not operate shooter mechanisms",
            ),
            manual_launch,
            vision_launch,
            pk_launch,
        ]
    )
