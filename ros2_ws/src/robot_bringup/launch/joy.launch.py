from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 物理コントローラーの入力をsensor_msgs/msg/Joyとして/joyへpublishするドライバ。
    # joy_controller_nodeはこの/joyをsubscribeして各機構の指令へ変換する。
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="Joystick device id read by joy_node (/dev/input/js<device_id>)",
            ),
            DeclareLaunchArgument(
                "deadzone",
                default_value="0.05",
                description="joy_nodeが0とみなす軸のデッドゾーン",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_id": LaunchConfiguration("device_id"),
                        "deadzone": LaunchConfiguration("deadzone"),
                    }
                ],
            ),
        ]
    )
