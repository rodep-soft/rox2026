import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory("rox_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    bt_xml = os.path.join(share, "behavior_trees", "navigate_to_pose.xml")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
            "default_nav_to_pose_bt_xml": bt_xml,
        },
        convert_types=True,
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[configured_params],
    )
    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "autostart": autostart, "node_names": ["map_server"]}
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": autostart,
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "arena.yaml")),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share, "config", "nav2.yaml"),
            ),
            GroupAction(
                [
                    SetRemap(src="/cmd_vel", dst="/mecanum/cmd_vel"),
                    map_server,
                    map_lifecycle_manager,
                    navigation,
                ]
            ),
        ]
    )
