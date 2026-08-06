import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # 1. ハードウェアドライバ類 (VESC, CAN, STM32等)
    hardware_launch = include("hardware.launch.py")

    # 2. 正面 CSI ステレオカメラ ＋ AprilTag 検出器ノード
    vision_launch = include(
        "vision_launch.py",
        launch_arguments={
            "stereonet_version": LaunchConfiguration("stereonet_version"),
            "enable_apriltag": "true",
            "enable_yolo": "false",
        }.items(),
    )

    # 3. Game 2 パネル自動戦術射出ノード
    game2_shooter_launch = include(
        "game2_shooter.launch.py",
        launch_arguments={
            "target_distance": LaunchConfiguration("target_distance"),
            "rpm_bottom": LaunchConfiguration("rpm_bottom"),
            "rpm_middle": LaunchConfiguration("rpm_middle"),
            "rpm_top": LaunchConfiguration("rpm_top"),
        }.items(),
    )

    # 4. モータ・機構コントローラーノード ＋ コントローラー入力
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
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version for 230AI",
            ),
            DeclareLaunchArgument(
                "target_distance",
                default_value="1.5",
                description="Target distance for Game 2 shooting in meters",
            ),
            DeclareLaunchArgument(
                "rpm_bottom",
                default_value="3000.0",
                description="Belt RPM for bottom row panels",
            ),
            DeclareLaunchArgument(
                "rpm_middle",
                default_value="4500.0",
                description="Belt RPM for middle row panels",
            ),
            DeclareLaunchArgument(
                "rpm_top",
                default_value="6000.0",
                description="Belt RPM for top row panels",
            ),
            hardware_launch,
            vision_launch,
            game2_shooter_launch,
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
