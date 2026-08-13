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

    # hardware (ドライバ類・VESC・CAN・BNO055 IMU等)
    hardware_launch = include(
        "hardware.launch.py",
        launch_arguments={
            "enable_imu": LaunchConfiguration("enable_imu"),
        }.items(),
    )

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

    # Game1 自動シーケンスノード
    game1_shooter_launch = include(
        "game1_shooter.launch.py",
        condition=IfCondition(LaunchConfiguration("enable_game1")),
    )

    # Game2 パネル戦術自動射出ノード
    game2_shooter_launch = include(
        "game2_shooter.launch.py",
        condition=IfCondition(LaunchConfiguration("enable_game2")),
    )

    # BNO055 IMU ドライバノード (libbno055-linux)
    bno055_launch = include(
        "bno055.launch.py",
        condition=IfCondition(LaunchConfiguration("enable_bno055")),
        launch_arguments={
            "imu_uart_port": LaunchConfiguration("imu_uart_port"),
        }.items(),
    )

    # 拡張カルマンフィルタ (EKF) ノード
    ekf_launch = include(
        "ekf.launch.py",
        condition=IfCondition(LaunchConfiguration("enable_ekf")),
    )

    # 分割された5つの独立コントローラーノード＋入力を一括起動
    launch_files = [
        "controllers/belt_controller.launch.py",
        "controllers/dribble_controller.launch.py",
        "controllers/spring_controller.launch.py",
        "controllers/mecanum_controller.launch.py",
        "input/joy_controller.launch.py",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Enable BNO055 IMU & Heading PID Controller",
            ),
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
                "enable_game1",
                default_value="false",
                description="Enable Game1 auto sequence shooter node",
            ),
            DeclareLaunchArgument(
                "enable_game2",
                default_value="false",
                description="Enable Game2 tactical panel shooter node",
            ),
            DeclareLaunchArgument(
                "enable_bno055",
                default_value="true",
                description="Enable BNO055 IMU driver node (libbno055-linux)",
            ),
            DeclareLaunchArgument(
                "imu_uart_port",
                default_value="/dev/ttyUSB0",
                description="UART device path for BNO055 IMU",
            ),
            DeclareLaunchArgument(
                "enable_ekf",
                default_value="true",
                description="Enable Extended Kalman Filter (robot_localization EKF)",
            ),
            DeclareLaunchArgument(
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version for 230AI",
            ),
            DeclareLaunchArgument(
                "enable_foxglove",
                default_value="true",
                description="Enable Foxglove WebSocket Bridge (port 8765)",
            ),
            hardware_launch,
            vision_launch,
            webcam_launch,
            bno055_launch,
            game1_shooter_launch,
            game2_shooter_launch,
            ekf_launch,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "foxglove_bridge.launch.py")),
                condition=IfCondition(LaunchConfiguration("enable_foxglove")),
            ),
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
