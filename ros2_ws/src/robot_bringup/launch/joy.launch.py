from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "joy_conversion.yaml"
    )
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[
            {
                "dev": "/dev/input/js0",
                "autorepeat_rate": 100.0,
            }
        ],
        ),
        Node(
            package="joy_conversion",
            executable="joy_conversion_node",
            name="joy_conversion_node",
            parameters=[parameter_file],
            output="screen",

        ),
    ])