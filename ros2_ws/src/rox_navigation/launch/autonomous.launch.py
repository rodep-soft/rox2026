import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(filename, arguments):
    share = get_package_share_directory("rox_navigation")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", filename)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    share = get_package_share_directory("rox_navigation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "arena.yaml")),
            include_launch("localization.launch.py", {"use_sim_time": use_sim_time}),
            include_launch(
                "navigation.launch.py",
                {"use_sim_time": use_sim_time, "map": map_yaml, "autostart": "true"},
            ),
        ]
    )
