import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters can be overridden via command line arguments
    return LaunchDescription([
        Node(
            package='ps5_controller_bridge',
            executable='bridge_node',
            name='ps5_controller_bridge',
            output='screen',
            parameters=[
                {
                    'wifi_host': os.getenv('WIFI_HOST', '192.168.1.100'),
                    'wifi_port': int(os.getenv('WIFI_PORT', '9999')),
                    'bt_addr': os.getenv('BT_ADDR', '00:11:22:33:44:55'),
                    'bt_port': int(os.getenv('BT_PORT', '1')),
                    'publish_rate': float(os.getenv('PUBLISH_RATE', '50.0')),
                }
            ],
        ),
    ])
