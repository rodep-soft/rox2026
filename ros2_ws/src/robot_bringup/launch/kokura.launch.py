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

    def include(launch_file, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
        )

    # 1. パラメータファイルパス
    mecanum_parameter_file = os.path.join(
        bringup_share, "config", "mecanum_controller.yaml"
    )
    heading_hold_parameter_file = os.path.join(
        bringup_share, "config", "heading_hold.yaml"
    )
    odometry_parameter_file = os.path.join(
        bringup_share, "config", "sensors.yaml"
    )

    return LaunchDescription(
        [
            # --- 起動引数 ---
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface for actuators",
            ),
            DeclareLaunchArgument(
                "imu_i2c_bus",
                default_value="/dev/i2c-0",
                description="I2C bus device path for BNO055 IMU (e.g. /dev/i2c-0, /dev/i2c-5)",
            ),
            DeclareLaunchArgument(
                "imu_i2c_address",
                default_value="40",  # 0x28 (decimal 40)
                description="I2C address for BNO055 IMU (decimal 40 for 0x28, 41 for 0x29)",
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

            # --- 4. 🧭 libbno055-linux: I2C 直結 100Hz IMU ドライバノード ---
            Node(
                package="libbno055_linux",
                executable="bno055_publisher_node",
                name="bno055_imu_node",
                output="screen",
                parameters=[
                    {
                        "device": LaunchConfiguration("imu_i2c_bus"),
                        "address": 40,
                        "frame_id": "imu_link",
                        "publish_rate_hz": 100,
                        "operation_mode": "ndof",  # 9-axis fusion
                    }
                ],
                remappings=[
                    ("imu/data", "/imu/data"),
                ],
            ),
            # 📐 base_link -> imu_link 静的 TF
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=["--x", "0.0", "--y", "0.0", "--z", "0.1",
                           "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                           "--frame-id", "base_link", "--child-frame-id", "imu_link"],
                output="screen",
            ),

            # --- 5. 🎯 libbno055-linux: 実績PID完全準拠 高性能 Heading Hold ノード ---
            Node(
                package="libbno055_linux",
                executable="bno055_heading_control_node",
                name="bno055_heading_hold_node",
                output="screen",
                parameters=[
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
                ],
            ),

            # --- 6. メカナム車輪制御 ＆ オドメトリノード ---
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

            # --- 7. 🛰️ 拡張カルマンフィルタ (EKF 自己位置推定ノード) ---
            include("ekf.launch.py"),
        ]
    )
