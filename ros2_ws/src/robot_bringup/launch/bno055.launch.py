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
                default_value="40",  # 0x28 (decimal 40) or 0x29 (decimal 41)
                description="I2C address for BNO055 IMU (decimal 40 for 0x28)",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="imu_link",
                description="TF frame ID for IMU sensor",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="100",
                description="Publish rate in Hz",
            ),
            Node(
                package="libbno055_linux",
                executable="bno055_publisher_node",
                name="bno055_imu_node",
                output="screen",
                parameters=[
                    {
                        "device": LaunchConfiguration("imu_i2c_bus"),
                        "address": 40,
                        "frame_id": LaunchConfiguration("imu_frame_id"),
                        "publish_rate_hz": 100,
                        "operation_mode": "ndof",  # 9-axis fusion
                    }
                ],
                remappings=[
                    ("imu/data", "/imu/data"),
                ],
            ),
        ]
    )
