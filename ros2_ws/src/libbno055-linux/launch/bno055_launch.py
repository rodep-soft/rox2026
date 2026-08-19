import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # Declare launch argument for node type selection
    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='standard',
        description='Type of BNO055 node to launch. Options: [standard, lifecycle]'
    )

    # Resolve default parameters file path
    pkg_share = get_package_share_directory('libbno055_linux')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'bno055_params.yaml'])

    # Declare launch argument for custom parameters file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS 2 parameters YAML configuration file.'
    )

    node_type = LaunchConfiguration('node_type')
    params_file = LaunchConfiguration('params_file')

    # 1. Standard Node Configuration
    standard_node = Node(
        package='libbno055_linux',
        executable='bno055_publisher_node',
        name='bno055_publisher_node',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", node_type, "' == 'standard'"])
        )
    )

    # 3. Lifecycle Managed Node Configuration
    lifecycle_node = LifecycleNode(
        package='libbno055_linux',
        executable='bno055_lifecycle_publisher_node',
        name='bno055_lifecycle_publisher_node',
        namespace='',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", node_type, "' == 'lifecycle'"])
        )
    )

    return LaunchDescription([
        node_type_arg,
        params_file_arg,
        standard_node,
        lifecycle_node
    ])
