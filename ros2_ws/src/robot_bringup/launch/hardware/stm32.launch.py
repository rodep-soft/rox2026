import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "stm32_driver.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="hardware_driver",
                executable="stm32_node",
                name="stm32_driver_node",
                output="screen",
                parameters=[
                    parameter_file,
                    {"can_rx_topic": "/socketcan_bridge/stm32/rx"},
                ],
            ),
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
                    "-1.218",
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
