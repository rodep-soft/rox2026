from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def lifecycle_handlers(node):
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    return [configure, activate]


def generate_launch_description():
    receiver = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_receiver_node_exe",
        name="socket_can_receiver",
        namespace=TextSubstitution(text=""),
        parameters=[
            {
                "interface": LaunchConfiguration("can_interface"),
                "enable_can_fd": False,
                "interval_sec": 0.1,
                "filters": "0:0",
                "use_bus_time": False,
            }
        ],
        remappings=[("from_can_bus", "/socketcan_bridge/rx")],
        output="screen",
    )
    sender = LifecycleNode(
        package="ros2_socketcan",
        executable="socket_can_sender_node_exe",
        name="socket_can_sender",
        namespace=TextSubstitution(text=""),
        parameters=[
            {
                "interface": LaunchConfiguration("can_interface"),
                "enable_can_fd": False,
                "enable_frame_loopback": False,
                "timeout_sec": 0.01,
            }
        ],
        remappings=[("to_can_bus", "/socketcan_bridge/tx")],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            receiver,
            sender,
            *lifecycle_handlers(receiver),
            *lifecycle_handlers(sender),
        ]
    )
