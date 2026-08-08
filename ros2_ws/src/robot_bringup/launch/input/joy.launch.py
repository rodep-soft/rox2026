from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_device_id = LaunchConfiguration("joy_device_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description="Joystick device index used by joy_node",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{
                    "device_id": ParameterValue(joy_device_id, value_type=int),
                    "autorepeat_rate": 100.0,
                    "coalesce_interval_ms": 10
                }],
            ),
        ]
    )
