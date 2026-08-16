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
        kwargs = {
            "launch_description_source": PythonLaunchDescriptionSource(
                os.path.join(launch_dir, launch_file)
            ),
        }
        if launch_arguments:
            kwargs["launch_arguments"] = launch_arguments
        if condition:
            kwargs["condition"] = condition
        return IncludeLaunchDescription(**kwargs)

    # パラメータファイルパス
    mecanum_parameter_file = os.path.join(bringup_share, "config", "mecanum_controller.yaml")
    odometry_parameter_file = os.path.join(bringup_share, "config", "sensors.yaml")

    return LaunchDescription(
        [
            # ── 起動引数 ──────────────────────────────────────────────
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface for actuators",
            ),
            DeclareLaunchArgument(
                "enable_game1",
                default_value="false",
                description="Enable Game1 auto sequence (game1_auto_node + apriltag_localizer_node)",
            ),
            DeclareLaunchArgument(
                "enable_game2",
                default_value="false",
                description="Enable Game2 panel shooter node (game2_auto_node)",
            ),
            DeclareLaunchArgument(
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version",
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="true",
                description="Enable AprilTag detection",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="false",
                description="Enable YOLO ball detection",
            ),

            # ── 1. ハードウェア (CAN / VESC / EduLite / STM32) ──────
            include(
                "hardware.launch.py",
                list({"can_interface": LaunchConfiguration("can_interface")}.items()),
            ),

            # ── 2. 操作系 & Foxglove Bridge ──────────────────────────
            include("input/joy_controller.launch.py"),
            include("foxglove_bridge.launch.py"),

            # ── 3. アーム・射出・LED 各種コントローラ ────────────────
            include("controllers/belt_controller.launch.py"),
            include("controllers/dribble_controller.launch.py"),
            include("controllers/spring_controller.launch.py"),
            include("controllers/led_controller.launch.py"),

            # ── 4. BNO055 IMU + IMU MUX + Heading Controller ─────────
            Node(
                package="libbno055_linux",
                executable="bno055_publisher_node",
                name="bno055_publisher_node",
                parameters=[os.path.join(bno055_share, "config", "bno055_params.yaml")],
                remappings=[("/imu/data", "/bno055/imu")],
                output="screen",
            ),
            # IMU MUX: STM32 CAN 優先、BNO055 バックアップ
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

            # ── 6. EKF 自己位置推定 ───────────────────────────────────
            include("ekf.launch.py"),

            # ── 7. 静的TF (base_link → imu_link / stm32_imu_link / camera_link) ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "--x", "-0.190", "--y", "-0.020", "--z", "0.225",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "imu_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_stm32_imu_tf",
                arguments=[
                    "--x", "-0.195", "--y", "-0.065", "--z", "0.225",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "-1.218",
                    "--frame-id", "base_link", "--child-frame-id", "stm32_imu_link",
                ],
                output="screen",
            ),
            # base_link → camera_link (AprilTag TF lookup に必要)
            # ※ 実測値に合わせること (game2_controller.yaml の camera_offset と同じ値)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_tf",
                arguments=[
                    "--x", "0.265", "--y", "0.035", "--z", "0.193",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "camera_link",
                ],
                output="screen",
            ),

            # ── 8. ステレオカメラ + AprilTag + YOLO (230AI) ───────────
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

            # ── 9. Game1 自動シーケンス (enable_game1:=true で有効) ───
            include(
                "game1.launch.py",
                condition=IfCondition(LaunchConfiguration("enable_game1")),
            ),

            # ── 10. Game2 パネル射出 (enable_game2:=true で有効) ──────
            include(
                "game2.launch.py",
                condition=IfCondition(LaunchConfiguration("enable_game2")),
            ),
        ]
    )
