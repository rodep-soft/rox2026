import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(pkg_bringup, "launch")
    game1_config = os.path.join(pkg_bringup, "config", "game1.yaml")

    side_arg = DeclareLaunchArgument(
        "side", default_value="left", description="Field side: left (A) or right (B)"
    )

    # 1. Foxglove Bridge (ws://localhost:8765)
    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "foxglove_bridge.launch.py"))
    )

    # 2. 3D Field Visualization (CAD Field, Gates, AprilTags, Targets)
    field_viz_node = Node(
        package="robot_controller",
        executable="field_visualization_node",
        name="field_visualization_node",
        output="screen",
        parameters=[{"field_side": LaunchConfiguration("side"), "map_frame": "map"}],
    )

    # 3. Game 1 Trajectory & Ball Simulator
    trajectory_sim_node = Node(
        package="robot_controller",
        executable="trajectory_sim_node",
        name="trajectory_sim_node",
        output="screen",
        parameters=[game1_config, {"field_side": LaunchConfiguration("side")}],
    )

    return LaunchDescription([
        side_arg,
        foxglove_bridge,
        field_viz_node,
        trajectory_sim_node,
    ])
