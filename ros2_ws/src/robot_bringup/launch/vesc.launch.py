from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory("robot_bringup"), "config", "vesc_node_param.yaml")
    return LaunchDescription([
        Node(package="hardware_driver", executable="vesc_node", name="vesc_driver_node", output="screen", parameters=[params])
    ])