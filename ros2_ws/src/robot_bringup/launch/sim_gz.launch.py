# Copyright 2026 Tatsukiyano
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory("robot_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    headless = LaunchConfiguration("headless", default="false")

    display_env = os.environ.get("DISPLAY", ":0")
    set_display_cmd = SetEnvironmentVariable(name="DISPLAY", value=display_env)

    # Qt platform for X11 / WSL
    set_qt_cmd = SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb")

    # Local loopback for gz-transport
    set_gz_ip = SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1")
    set_ign_ip = SetEnvironmentVariable(name="IGN_IP", value="127.0.0.1")

    models_path = os.path.join(pkg_robot_bringup, "models")

    # Set GZ_SIM_RESOURCE_PATH to include our models
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""), ":", models_path],
    )

    world_path = os.path.join(pkg_robot_bringup, "world", "rox2026_field_cad.sdf")

    # Construct gz_args
    gz_args = PythonExpression(
        ["'-r -v 4 ' + '", world_path, "' + (' -s' if '", headless, "' == 'true' else '')"]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")]
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_robot_bringup, "config", "gz_bridge.yaml"),
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_robot_bringup, "launch", "foxglove_bridge.launch.py"))
    )

    return LaunchDescription(
        [
            set_display_cmd,
            set_qt_cmd,
            set_gz_ip,
            set_ign_ip,
            set_gz_resource_path,
            gazebo_sim,
            bridge,
            foxglove_bridge,
        ]
    )
