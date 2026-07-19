from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "spring_param.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="spring_controller",
                executable="spring_joy_sub",
                name="spring_joy_sub",
                parameters=[config_file],
                output="screen",
                prefix=["gdb -ex run --args "],
            ),
            TimerAction(
                period=0.1,
                actions=[
                    Node(
                        package="spring_controller",
                        executable="spring_edulite_controller",
                        name="spring_edulite_controller",
                        parameters=[config_file],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="edulite05_driver",
                        executable="edulite05_single_velocity_node",
                        name="edulite05_single_velocity_node",
                        output="screen",
                        parameters=[config_file],
                    )
                ],
            ),
        ]
    )
