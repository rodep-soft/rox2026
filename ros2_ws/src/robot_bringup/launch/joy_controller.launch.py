import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "joy_controller.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Joystick device id read by joy_node (/dev/input/js<device_id>)",
            ),
            # 物理コントローラーの入力をsensor_msgs/msg/Joyとして/joyへpublishするドライバ。
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"device_id": LaunchConfiguration("device_id")}],
            ),
            Node(
                package="joy_controller",
                executable="joy_controller_node",
                name="joy_controller",
                output="screen",
                parameters=[parameter_file],
            ),
        ]
    )
