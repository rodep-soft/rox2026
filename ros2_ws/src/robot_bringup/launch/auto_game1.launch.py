import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_bringup_share = get_package_share_directory("robot_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # robot_bringup パッケージ内の config パスを取得
    default_nav2_params = os.path.join(robot_bringup_share, "config", "auto_game1_nav2.yaml")
    default_auto_game1_params = os.path.join(robot_bringup_share, "config", "auto_game1.yaml")

    # Launch 引数の宣言
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_yaml_file = LaunchConfiguration("map", default="")
    nav2_params_file = LaunchConfiguration("nav2_params_file", default=default_nav2_params)
    auto_game1_params_file = LaunchConfiguration("auto_game1_params_file", default=default_auto_game1_params)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file to load",
    )

    declare_nav2_params_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=default_nav2_params,
        description="Full path to the ROS2 parameters file to use for Nav2 nodes",
    )

    declare_auto_game1_params_cmd = DeclareLaunchArgument(
        "auto_game1_params_file",
        default_value=default_auto_game1_params,
        description="Full path to the ROS2 parameters file to use for auto_game1_node",
    )

    # Nav2 (bringup_launch.py) の組み込み起動
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml_file,
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
    )

    # auto_game1_node の起動設定
    auto_game1_node = Node(
        package="auto_game1",
        executable="auto_game1_node",
        name="auto_game1_node",
        parameters=[auto_game1_params_file],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_nav2_params_cmd,
        declare_auto_game1_params_cmd,
        nav2_launch,
        auto_game1_node,
    ])
