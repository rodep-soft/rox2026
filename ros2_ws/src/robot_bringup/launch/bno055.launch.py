from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "imu_uart_port",
            default_value="/dev/ttyUSB0",
            description="UART device path for BNO055 IMU (e.g. /dev/ttyUSB0 or /dev/ttyAMA0)",
        ),
        DeclareLaunchArgument(
            "imu_frame_id",
            default_value="imu_link",
            description="TF frame ID for IMU sensor",
        ),
        Node(
            package="bno055",
            executable="bno055_node",
            name="bno055_imu_node",
            output="screen",
            parameters=[{
                "connection_type": "uart",
                "uart_port": LaunchConfiguration("imu_uart_port"),
                "uart_baudrate": 115200,
                "frame_id": LaunchConfiguration("imu_frame_id"),
                "data_deny_list": [],
            }],
            remappings=[
                ("bno055/imu", "/imu/data"),
            ],
        ),
    ])
