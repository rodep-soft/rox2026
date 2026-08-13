from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_i2c_bus",
                default_value="/dev/i2c-1",
                description="I2C bus device path for BNO055 IMU (e.g. /dev/i2c-1)",
            ),
            DeclareLaunchArgument(
                "imu_i2c_address",
                default_value="0x28",
                description="I2C address for BNO055 IMU (0x28 or 0x29)",
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
                parameters=[
                    {
                        "connection_type": "i2c",
                        "i2c_bus": LaunchConfiguration("imu_i2c_bus"),
                        "i2c_address": LaunchConfiguration("imu_i2c_address"),
                        "frame_id": LaunchConfiguration("imu_frame_id"),
                        "data_deny_list": [],
                    }
                ],
                remappings=[
                    ("bno055/imu", "/imu/data"),
                ],
            ),
        ]
    )
