from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    joy_dev = LaunchConfiguration("joy_dev")
    can_interface = LaunchConfiguration("can_interface")

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "mad_motor.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[config_file, {"dev": joy_dev}],
    )

    mad_motor_node = Node(
        package="mad_motor",
        executable="mad_motor_node",
        name="mad_motor_node",
        output="screen",
        parameters=[config_file],
    )

    mad_motor_can_command_node = Node(
        package="motor_can_bridge",
        executable="motor_can_command_node",
        name="mad_motor_can_command_node",
        output="screen",
        parameters=[
            config_file,
            {
                "can_tx_topic": PathJoinSubstitution(
                    ["/CAN", can_interface, "transmit"]
                )
            },
        ],
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
                description="MAD motor bringup parameter yaml",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="RDK/gamepad joystick device path",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used for the MAD motor STM command path",
            ),
            joy_node,
            mad_motor_node,
            mad_motor_can_command_node,
            socketcan_bridge,
        ]
    )
