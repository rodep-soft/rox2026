from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package="robot_controller",
        executable="game1_auto_shooter_node",
        name="game1_auto_shooter_node",
        output="screen",
        parameters=[
            {
                "kp_linear": 1.0,
                "kp_angular": 1.5,
                "max_linear_vel": 1.5,
                "max_angular_vel": 1.0,
                "pos_tolerance": 0.08,
                "yaw_tolerance": 0.05,
            }
        ],
    )

    return LaunchDescription([node])
