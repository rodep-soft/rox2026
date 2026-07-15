from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    joy_dev = LaunchConfiguration("joy_dev")
    can_interface = LaunchConfiguration("can_interface")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robstride05.yaml"]
    )
    joy_conversion_config_file = PathJoinSubstitution(
        [
            FindPackageShare("joy_conversion"),
            "config",
            "joy_conversion_node.yaml",
        ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[config_file, {"dev": joy_dev}],
    )

    joy_conversion_node = Node(
        package="joy_conversion",
        executable="joy_conversion_node",
        name="joy_conversion_node",
        output="screen",
        parameters=[joy_conversion_config_file],
    )

    roller_position_controller = Node(
        package="roller_angle_controller",
        executable="roller_angle_controller_node",
        name="roller_position_controller",
        output="screen",
        parameters=[config_file],
    )

    robstride_position_node = Node(
        package="robstride_can_node",
        executable="robstride_can_node",
        name="robstride_position_node",
        output="screen",
        parameters=[config_file],
    )

    socketcan_bridge = Node(
        package="ros2socketcan_bridge",
        executable="ros2socketcan",
        name="ros2socketcan",
        output="screen",
        parameters=[config_file, {"CAN_INTERFACE": can_interface}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="robstride05 bringup parameter yaml",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="RDK/gamepad joystick device path",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used for RobStride 05",
            ),
            joy_node,
            joy_conversion_node,
            roller_position_controller,
            robstride_position_node,
            socketcan_bridge,
        ]
    )
