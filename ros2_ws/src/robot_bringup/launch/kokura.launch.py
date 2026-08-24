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

    def include(launch_file, launch_arguments=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
            condition=condition,
        )

    # 1. パラメータファイルパス
    mecanum_parameter_file = os.path.join(
        bringup_share, "config", "mecanum_controller.yaml"
    )
    odometry_parameter_file = os.path.join(bringup_share, "config", "odometry.yaml")
    heading_hold_parameter_file = os.path.join(
        bringup_share, "config", "heading_hold.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "side",
                default_value="left",
                description="Field side orientation: 'left' (red) or 'right' (blue) for auto Y/Yaw mirroring",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface for actuators",
            ),
            DeclareLaunchArgument(
                "enable_vision",
                default_value="true",
                description="Enable 230AI MIPI stereo vision launch (mipi_cam / hobot_stereonet / apriltag / yolo)",
            ),
            DeclareLaunchArgument(
                "enable_stereonet",
                default_value="false",
                description="Enable heavy 3D hobot_stereonet depth inference and web visualizer",
            ),
            DeclareLaunchArgument(
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version for 230AI",
            ),
            DeclareLaunchArgument(
                "publish_visual_enabled",
                default_value="False",
                description="Enable publishing visualization image for Web UI",
            ),
            DeclareLaunchArgument(
                "publish_pcd_enabled",
                default_value="False",
                description="Enable publishing PointCloud2",
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="true",
                description="Enable AprilTag detection node on CSI camera",
            ),
            DeclareLaunchArgument(
                "tag_family",
                default_value="16h5",
                description="AprilTag family (16h5)",
            ),
            DeclareLaunchArgument(
                "tag_size",
                default_value="0.18",
                description="AprilTag size in meters",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="false",
                description="Enable BPU-accelerated YOLO ball detection node",
            ),
            DeclareLaunchArgument(
                "game",
                default_value="game1",
                description="Selected game mode: 'game1' (starts webcam & game1) or 'game2' (disables webcam to save USB/BPU bandwidth)",
            ),
            DeclareLaunchArgument(
                "enable_webcam",
                default_value="true",
                description="Enable USB webcam launch (v4l2_camera). Default true for Game 1.",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="V4L2 video device path for webcam",
            ),
            DeclareLaunchArgument(
                "enable_game1",
                default_value="true",
                description="Enable Game 1 auto shooter/passer node",
            ),
            DeclareLaunchArgument(
                "enable_game2",
                default_value="true",
                description="Enable Game 2 auto tactics node",
            ),
            # --- 1. ハードウェア通信 (CAN/VESC/EduLite/STM32) ---
            include(
                "hardware.launch.py",
                list({"can_interface": LaunchConfiguration("can_interface")}.items()),
            ),
            # --- 2. 操作系 & Foxglove Bridge ---
            include("input/joy_controller.launch.py"),
            include("foxglove_bridge.launch.py"),
            # --- 3. アーム・射出・LED 各種コントローラ ---
            include("controllers/belt_controller.launch.py"),
            include("controllers/dribble_controller.launch.py"),
            include("controllers/spring_controller.launch.py"),
            include("controllers/led_controller.launch.py"),
            # --- 4. BNO055 Heading Control Node (Feedforward + 2-DOF) from libbno055_linux ---
            Node(
                package="libbno055_linux",
                executable="bno055_heading_control_node",
                name="bno055_heading_control_node",
                parameters=[heading_hold_parameter_file],
                output="screen",
            ),
            # --- 5. メカナム車輪制御 ＆ オドメトリノード ---
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
            # --- 6. 拡張カルマンフィルタ (EKF 自己位置推定ノード) ＆ AprilTag マップ位置補正ノード ---
            include("ekf.launch.py"),
            Node(
                package="robot_controller",
                executable="apriltag_localizer_node",
                name="apriltag_localizer_node",
                output="screen",
                parameters=[
                    os.path.join(bringup_share, "config", "apriltag_tag_map.yaml"),
                    {"field_side": LaunchConfiguration("side")},
                ],
            ),
            Node(
                package="robot_controller",
                executable="field_visualization_node",
                name="field_visualization_node",
                output="screen",
                parameters=[
                    {"field_side": LaunchConfiguration("side")},
                    {"map_frame": "map"},
                ],
            ),
            # --- 7. Game 1 & Game 2 自律戦術ノード (ボタン即応・デフォルト起動) ---
            Node(
                package="robot_controller",
                executable="game1_auto_node",
                name="game1_auto_node",
                output="screen",
                parameters=[
                    os.path.join(bringup_share, "config", "game1.yaml"),
                    {"field_side": LaunchConfiguration("side")},
                ],
                condition=IfCondition(LaunchConfiguration("enable_game1")),
            ),
            Node(
                package="robot_controller",
                executable="game2_auto_node",
                name="game2_auto_node",
                output="screen",
                parameters=[
                    os.path.join(bringup_share, "config", "game2_controller.yaml"),
                    {"field_side": LaunchConfiguration("side")},
                ],
                condition=IfCondition(LaunchConfiguration("enable_game2")),
            ),
            # --- 8. 230AI MIPI ステレオビジョン & AprilTag / YOLO ---
            include(
                "vision_launch.py",
                launch_arguments=list(
                    {
                        "enable_stereonet": LaunchConfiguration("enable_stereonet"),
                        "stereonet_version": LaunchConfiguration("stereonet_version"),
                        "publish_visual_enabled": LaunchConfiguration(
                            "publish_visual_enabled"
                        ),
                        "publish_pcd_enabled": LaunchConfiguration(
                            "publish_pcd_enabled"
                        ),
                        "enable_apriltag": LaunchConfiguration("enable_apriltag"),
                        "enable_yolo": LaunchConfiguration("enable_yolo"),
                        "tag_family": LaunchConfiguration("tag_family"),
                        "tag_size": LaunchConfiguration("tag_size"),
                        "model_name": LaunchConfiguration("model_name"),
                    }.items()
                ),
                condition=IfCondition(LaunchConfiguration("enable_vision")),
            ),
            # --- 9. USB Webカメラ (V4L2 + AprilTag) ---
            include(
                "webcam_launch.py",
                launch_arguments=list(
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "enable_apriltag": "true",
                    }.items()
                ),
                condition=IfCondition(LaunchConfiguration("enable_webcam")),
            ),
        ]
    )
