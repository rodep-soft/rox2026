import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
    )

    def include(launch_file, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            **kwargs,
        )

    # hardware (ドライバ類・VESC・CAN等)
    hardware_launch = include("hardware.launch.py")

    # 230AI ステレオビジョン機能（hobot_stereonet, AprilTag & YOLO）
    vision_launch = include(
        "vision_launch.py",
        condition=IfCondition(LaunchConfiguration("enable_vision")),
        launch_arguments={
            "stereonet_version": LaunchConfiguration("stereonet_version"),
            "enable_apriltag": LaunchConfiguration("enable_apriltag"),
            "enable_yolo": LaunchConfiguration("enable_yolo"),
        }.items(),
    )

    # USB Webカメラ (V4L2)
    webcam_launch = include(
        "webcam_launch.py",
        condition=IfCondition(LaunchConfiguration("enable_webcam")),
        launch_arguments={
            "video_device": LaunchConfiguration("video_device"),
            "enable_apriltag": LaunchConfiguration("enable_apriltag"),
        }.items(),
    )

    # 分割された5つの独立コントローラーノード＋入力を一括起動
    launch_files = [
        "controllers/belt_controller.launch.py",
        "controllers/dribbler_controller.launch.py",
        "controllers/arm_position_controller.launch.py",
        "controllers/spring_controller.launch.py",
        "controllers/mecanum_controller.launch.py",
        "input/joy_controller.launch.py",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_vision",
                default_value="false",
                description="Enable 230AI stereo vision launch (hobot_stereonet)",
            ),
            DeclareLaunchArgument(
                "enable_webcam",
                default_value="false",
                description="Enable USB webcam launch (v4l2_camera)",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="V4L2 video device path for webcam (e.g. /dev/video0)",
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="false",
                description="Enable AprilTag detection node",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="false",
                description="Enable BPU-accelerated YOLO ball detection node",
            ),
            DeclareLaunchArgument(
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version for 230AI",
            ),
            hardware_launch,
            vision_launch,
            webcam_launch,
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
