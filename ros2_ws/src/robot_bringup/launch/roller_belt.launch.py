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
        [FindPackageShare("robot_bringup"), "config", "roller_belt.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[config_file, {"dev": joy_dev}],
    )

    belt_controller_node = Node(
        package="belt_controller",
        executable="belt_controller_node",
        name="belt_controller_node",
        output="screen",
        parameters=[config_file],
    )

    roller_belt_can_packer_node = Node(
        package="roller_belt_can_packer",
        executable="roller_belt_can_packer_node",
        name="roller_belt_can_packer_node",
        output="screen",
        parameters=[
            config_file,
            {
                "can_transmit_topic": PathJoinSubstitution(
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
                description="belt PWM and STM CAN packer parameter yaml",
            ),
            DeclareLaunchArgument(
                "joy_dev",
                default_value="/dev/input/js0",
                description="RDK/gamepad joystick device path",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used for the roller and belt STM command path",
            ),
            joy_node,
            belt_controller_node,
            roller_belt_can_packer_node,
            socketcan_bridge,
        ]
    )
