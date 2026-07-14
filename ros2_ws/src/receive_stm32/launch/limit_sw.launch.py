import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="receive_stm32",
                executable="limit_sw_node",
                name="limit_sw_node",
                output="screen",
                parameters=[
                    {"can_receive_topic": "/CAN/can0/receive", "target_can_id": 0x202}
                ],
            )
        ]
    )
