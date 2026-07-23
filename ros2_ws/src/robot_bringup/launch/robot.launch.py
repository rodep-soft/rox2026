import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    bringup_dir = get_package_share_directory("robot_bringup")

    receiver = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        namespace="",
        output="screen",
        parameters=[
            {
                "interface": "can0",
                "enable_can_fd": False,
                "interval_sec": 0.01,
                "filters": "0:0",
                "use_bus_time": False,
            }
        ],
        remappings=[("from_can_bus", "from_can_bus")],
    )

    sender = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_sender_node_exe",
        name="socket_can_sender",
        namespace="",
        output="screen",
        parameters=[
            {
                "interface": "can0",
                "enable_can_fd": False,
                "timeout_sec": 0.01,
            }
        ],
        remappings=[("to_can_bus", "to_can_bus")],
    )

    configure_receiver = RegisterEventHandler(
        OnProcessStart(
            target_action=receiver,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(receiver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_receiver = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=receiver,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(receiver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    start_sender = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=receiver,
            goal_state="active",
            entities=[sender],
        )
    )

    configure_sender = RegisterEventHandler(
        OnProcessStart(
            target_action=sender,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(sender),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_sender = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sender,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(sender),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    start_robot = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sender,
            goal_state="active",
            entities=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "interface.launch.py")
                    ),
                    launch_arguments={"launch_socketcan": "false"}.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "spring.launch.py")
                    )
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "mecanum.launch.py")
                    )
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "roller_belt.launch.py")
                    )
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, "launch", "roller_angle.launch.py")
                    )
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            configure_receiver,
            activate_receiver,
            start_sender,
            configure_sender,
            activate_sender,
            start_robot,
            receiver,
        ]
    )
