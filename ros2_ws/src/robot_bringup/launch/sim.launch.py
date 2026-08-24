import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(bringup_share, "launch")

    mecanum_parameter_file = os.path.join(bringup_share, "config", "mecanum_controller.yaml")
    odometry_parameter_file = os.path.join(bringup_share, "config", "odometry.yaml")
    heading_hold_parameter_file = os.path.join(bringup_share, "config", "heading_hold.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="left", description="Field side: left or right"),
        
        # 1. Foxglove Bridge (WebSocket port 8765)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "foxglove_bridge.launch.py"))
        ),

        # Static TF (map -> odom -> base_footprint)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_map_to_odom",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "map", "--child-frame-id", "odom"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_odom_to_base",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "odom", "--child-frame-id", "base_footprint"],
        ),
        
        # 2. 3D Field & Gate Visualizer (Official SDF layout)
        Node(
            package="robot_controller",
            executable="field_visualization_node",
            name="field_visualization_node",
            output="screen",
            parameters=[
                {"field_side": LaunchConfiguration("side")},
                {"map_frame": "map"},
            ],
        ),

        # 3. Odometry & Movement Simulator Node
        Node(
            package="robot_controller",
            executable="mecanum_odometry_node",
            name="mecanum_odometry_node",
            output="screen",
            parameters=[odometry_parameter_file],
        ),

        # 4. Game 1 Auto Sequence Controller
        Node(
            package="robot_controller",
            executable="game1_auto_node",
            name="game1_auto_node",
            output="screen",
            parameters=[
                {"field_side": LaunchConfiguration("side")},
            ],
        ),
    ])
