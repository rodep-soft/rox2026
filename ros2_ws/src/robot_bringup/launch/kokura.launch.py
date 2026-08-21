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
    bno055_share = get_package_share_directory("libbno055_linux")
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
    odometry_parameter_file = os.path.join(bringup_share, "config", "sensors.yaml")

    return LaunchDescription(
        [
            # --- 起動引数 ---
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
                "model_name",
                default_value="yolov5s",
                description="YOLO model name",
            ),
            DeclareLaunchArgument(
                "enable_webcam",
                default_value="true",
                description="Enable USB webcam launch (v4l2_camera)",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="V4L2 video device path for webcam",
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
            # --- 4. STM32 CAN IMU 中継 (/stm32/imu -> /imu/data) ＆ Heading Controller ---
            Node(
                package="robot_controller",
                executable="imu_mux_node",
                name="imu_mux_node",
                parameters=[
                    {
                        "primary_imu_topic": "/stm32/imu",
                        "secondary_imu_topic": "/stm32/imu",
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
            # --- 6. 拡張カルマンフィルタ (EKF 自己位置推定ノード) ---
            include("ekf.launch.py"),
            # --- 7. 230AI MIPI ステレオビジョン & AprilTag / YOLO ---
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
            # --- 8. USB Webカメラ (V4L2) ---
            include(
                "webcam_launch.py",
                launch_arguments=list(
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "enable_apriltag": "false",
                    }.items()
                ),
                condition=IfCondition(LaunchConfiguration("enable_webcam")),
            ),
            # base_link -> stm32_imu_link 静的 TF (STM32 IMU: 後方-195mm, 右-65mm, 地上高+225mm)
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
                respawn=False,
            ),
        ]
    )
