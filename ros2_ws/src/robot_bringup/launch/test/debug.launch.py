"""
debug.launch.py  ── デバッグ・テスト用 全機能起動 (auto_node なし)

起動内容:
  - ハードウェア (CAN / VESC / EduLite / STM32)
  - joy_controller (手動操作)
  - Foxglove Bridge
  - 各種コントローラ (belt / dribble / spring / LED)
  - IMU (BNO055 + imu_mux + heading_hold)
  - メカナム車輪制御 + オドメトリ
  - EKF (自己位置推定)
  - 静的 TF (imu_link / stm32_imu_link / camera_link)
  - ステレオカメラ + AprilTag (CSI)
  - USB Webcam
  - apriltag_localizer_node (/apriltag/pose → EKF)

起動しないもの:
  - game1_auto_node
  - game2_auto_node

使い方:
  ros2 launch robot_bringup test/debug.launch.py
  ros2 launch robot_bringup test/debug.launch.py enable_apriltag:=false
  ros2 launch robot_bringup test/debug.launch.py enable_yolo:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    bno055_share = get_package_share_directory("libbno055_linux")
    launch_dir = os.path.join(bringup_share, "launch")

    def include(launch_file, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
        )

    mecanum_parameter_file = os.path.join(
        bringup_share, "config", "mecanum_controller.yaml"
    )
    odometry_parameter_file = os.path.join(bringup_share, "config", "sensors.yaml")
    tag_map_path = os.path.join(bringup_share, "config", "apriltag_tag_map.yaml")

    return LaunchDescription(
        [
            # ── 起動引数 ──────────────────────────────────────────────
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface",
            ),
            DeclareLaunchArgument(
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version",
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="true",
                description="Enable AprilTag detection (CSI camera)",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="false",
                description="Enable YOLO ball detection",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="USB webcam device path",
            ),
            # ── 1. ハードウェア ───────────────────────────────────────
            include(
                "hardware.launch.py",
                list({"can_interface": LaunchConfiguration("can_interface")}.items()),
            ),
            # ── 2. 操作系 & Foxglove Bridge ──────────────────────────
            include("input/joy_controller.launch.py"),
            include("foxglove_bridge.launch.py"),
            # ── 3. アーム・射出・LED コントローラ ────────────────────
            include("controllers/belt_controller.launch.py"),
            include("controllers/dribble_controller.launch.py"),
            include("controllers/spring_controller.launch.py"),
            include("controllers/led_controller.launch.py"),
            # ── 4. IMU ────────────────────────────────────────────────
            Node(
                package="libbno055_linux",
                executable="bno055_publisher_node",
                name="bno055_publisher_node",
                parameters=[os.path.join(bno055_share, "config", "bno055_params.yaml")],
                remappings=[("/imu/data", "/bno055/imu")],
                output="screen",
            ),
            Node(
                package="robot_controller",
                executable="imu_mux_node",
                name="imu_mux_node",
                parameters=[
                    {
                        "primary_imu_topic": "/stm32/imu",
                        "secondary_imu_topic": "/bno055/imu",
                        "output_imu_topic": "/imu/data",
                        "timeout_ms": 100,
                    }
                ],
                output="screen",
            ),
            Node(
                package="libbno055_linux",
                executable="bno055_heading_control_node",
                name="bno055_heading_control_node",
                parameters=[
                    os.path.join(bno055_share, "config", "heading_control_params.yaml")
                ],
                output="screen",
            ),
            # ── 5. メカナム車輪制御 & オドメトリ ─────────────────────
            Node(
                package="robot_controller",
                executable="mecanum_controller_node",
                name="mecanum_controller_node",
                output="screen",
                parameters=[mecanum_parameter_file],
            ),
            Node(
                package="robot_controller",
                executable="mecanum_odometry_node",
                name="mecanum_odometry_node",
                output="screen",
                parameters=[odometry_parameter_file],
            ),
            # ── 6. EKF ───────────────────────────────────────────────
            include("ekf.launch.py"),
            # ── 7. 静的 TF ───────────────────────────────────────────
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "--x",
                    "-0.190",
                    "--y",
                    "-0.020",
                    "--z",
                    "0.225",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "imu_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_stm32_imu_tf",
                arguments=[
                    "--x",
                    "-0.195",
                    "--y",
                    "-0.065",
                    "--z",
                    "0.225",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "-1.218",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "stm32_imu_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_tf",
                arguments=[
                    "--x",
                    "0.265",
                    "--y",
                    "0.035",
                    "--z",
                    "0.193",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "camera_link",
                ],
                output="screen",
            ),
            # ── 8. ステレオカメラ + AprilTag + YOLO (CSI) ────────────
            include(
                "vision_launch.py",
                list(
                    {
                        "stereonet_version": LaunchConfiguration("stereonet_version"),
                        "enable_apriltag": LaunchConfiguration("enable_apriltag"),
                        "enable_yolo": LaunchConfiguration("enable_yolo"),
                        "publish_visual_enabled": "False",
                        "publish_pcd_enabled": "False",
                    }.items()
                ),
            ),
            # ── 9. USB Webcam ─────────────────────────────────────────
            include(
                "webcam_launch.py",
                list(
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "enable_apriltag": "false",
                    }.items()
                ),
            ),
            # ── 10. AprilTag ローカライザ (EKF 自己位置補正) ──────────
            # auto_node なしでもタグ→/apriltag/pose→EKF の動作確認が可能
            Node(
                package="robot_controller",
                executable="apriltag_localizer_node",
                name="apriltag_localizer_node",
                output="screen",
                parameters=[tag_map_path],
            ),
        ]
    )
