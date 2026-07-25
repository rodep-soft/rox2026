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

    def include(launch_file, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments=launch_arguments,
        )

    # hardware(driver類)。vesc(赤ブラシ)はrobot.launch.pyでは起動しない。
    hardware_launch = include(
        "hardware.launch.py",
        launch_arguments={
            "can_interface": LaunchConfiguration("can_interface"),
            "use_vesc": "false",
        }.items(),
    )

    # controllerと操作系。topic名や各種パラメータはrobot_bringup/config配下のyamlで管理する。
    # beltとdribbleはbelt_dribble.launch.pyで一緒に起動する。
    launch_files = [
        "belt_dribble.launch.py",
        "mecanum_controller.launch.py",
        "spring_controller.launch.py",
        "dribble_position_controller.launch.py",
        "joy_controller.launch.py",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by ros2_socketcan",
            ),
            hardware_launch,
            *[include(launch_file) for launch_file in launch_files],
        ]
    )
