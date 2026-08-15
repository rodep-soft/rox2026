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
            # --- 4. libbno055-linux: BNO055 Publisher (/bno055/imu) ＆ Heading Controller (/imu/data購読) ---
            Node(
                package="libbno055_linux",
                executable="bno055_publisher_node",
                name="bno055_publisher_node",
                parameters=[os.path.join(bno055_share, "config", "bno055_params.yaml")],
                remappings=[("/imu/data", "/bno055/imu")],
                output="screen",
            ),
            # --- 4.1 IMU MUX (Dual IMU: STM32 CANを最優先メイン、BNO055をバックアップに設定) ---
            Node(
                package="robot_controller",
                executable="imu_mux_node",
                name="imu_mux_node",
                parameters=[{
                    "primary_imu_topic": "/stm32/imu",
                    "secondary_imu_topic": "/bno055/imu",
                    "output_imu_topic": "/imu/data",
                    "timeout_ms": 100,
                }],
                output="screen",
            ),
            Node(
                package="libbno055_linux",
                executable="bno055_heading_control_node",
                name="bno055_heading_control_node",
                parameters=[os.path.join(bno055_share, "config", "heading_control_params.yaml")],
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
            # base_link -> imu_link 静的 TF (RDK BNO055: 後方-190mm, 右-20mm, 地上高+225mm)
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
                respawn=False,
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
                    "0.0",
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
