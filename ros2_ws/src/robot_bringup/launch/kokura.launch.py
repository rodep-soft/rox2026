import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")

    try:
        bno055_share = get_package_share_directory("libbno055_linux")
        bno055_parameter_file = os.path.join(bno055_share, "config", "bno055_params.yaml")
        heading_control_parameter_file = os.path.join(bno055_share, "config", "heading_control_params.yaml")
    except Exception:
        bno055_parameter_file = None
        heading_control_parameter_file = None

    def include(launch_file, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
        )

    # 1. パラメータファイルパス
    mecanum_parameter_file = os.path.join(
        bringup_share, "config", "mecanum_controller.yaml"
    )
    odometry_parameter_file = os.path.join(
        bringup_share, "config", "sensors.yaml"
    )

    heading_hold_params = (
        [heading_control_parameter_file]
        if heading_control_parameter_file and os.path.exists(heading_control_parameter_file)
        else [
            {
                "kp": 4.0,
                "ki": 0.0,
                "kd": 0.05,
                "integral_limit": 0.5,
                "heading_deadband_rad": 0.02,
                "rotation_input_deadband_rad_s": 0.02,
                "max_correction_rad_s": 1.5,
                "control_period_ms": 10,
                "command_timeout_ms": 500,
                "imu_timeout_ms": 250,
                "raw_cmd_vel_topic": "/drive/cmd_vel",
                "corrected_cmd_vel_topic": "/mecanum/cmd_vel_heading",
                "imu_topic": "/imu/data",
            }
        ]
    )

    return LaunchDescription(
        [
            # --- 起動引数 ---
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface for actuators",
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

            # --- 4. 🎯 libbno055-linux: 実績PID完全準拠 高性能 Heading Hold ノード ---
            Node(
                package="libbno055_linux",
                executable="bno055_heading_control_node",
                name="bno055_heading_hold_node",
                output="screen",
                parameters=heading_hold_params,
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

            # --- 6. 🛰️ 拡張カルマンフィルタ (EKF 自己位置推定ノード) ---
            include("ekf.launch.py"),
        ]
    )
