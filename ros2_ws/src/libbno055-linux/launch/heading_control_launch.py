import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('libbno055_linux')

    # Resolved default parameter file paths
    default_imu_params = PathJoinSubstitution([pkg_share, 'config', 'bno055_params.yaml'])
    default_control_params = PathJoinSubstitution([pkg_share, 'config', 'heading_control_params.yaml'])

    # Launch arguments
    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='standard',
        description='Node architecture selection. Options: [standard, lifecycle]'
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Whether to launch in a zero-copy Composable Container.'
    )

    imu_params_file_arg = DeclareLaunchArgument(
        'imu_params_file',
        default_value=default_imu_params,
        description='Path to BNO055 IMU Driver YAML configuration file.'
    )

    control_params_file_arg = DeclareLaunchArgument(
        'control_params_file',
        default_value=default_control_params,
        description='Path to Heading Controller YAML configuration file.'
    )

    node_type = LaunchConfiguration('node_type')
    use_composition = LaunchConfiguration('use_composition')
    imu_params_file = LaunchConfiguration('imu_params_file')
    control_params_file = LaunchConfiguration('control_params_file')

    # 1. Zero-Copy Composable Container Mode
    standard_composable_container = ComposableNodeContainer(
        name='bno055_heading_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='libbno055_linux',
                plugin='bno055_ros2::BNO055PublisherNode',
                name='bno055_publisher_node',
                parameters=[imu_params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='libbno055_linux',
                plugin='bno055_ros2::BNO055HeadingControlNode',
                name='bno055_heading_control_node',
                parameters=[control_params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_composition, "' == 'true' and '", node_type, "' == 'standard'"]))
    )

    # 2. Lifecycle Managed Standalone Nodes (Nav2 Enterprise Compatible)
    lifecycle_imu_node = LifecycleNode(
        package='libbno055_linux',
        executable='bno055_lifecycle_publisher_node',
        name='bno055_lifecycle_publisher_node',
        namespace='',
        parameters=[imu_params_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", node_type, "' == 'lifecycle'"]))
    )

    lifecycle_heading_node = LifecycleNode(
        package='libbno055_linux',
        executable='bno055_lifecycle_heading_control_node',
        name='bno055_lifecycle_heading_control_node',
        namespace='',
        parameters=[control_params_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", node_type, "' == 'lifecycle'"]))
    )

    # 3. Standard Standalone Process Mode
    standalone_imu_node = Node(
        package='libbno055_linux',
        executable='bno055_publisher_node',
        name='bno055_publisher_node',
        parameters=[imu_params_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_composition, "' != 'true' and '", node_type, "' == 'standard'"]))
    )

    standalone_heading_node = Node(
        package='libbno055_linux',
        executable='bno055_heading_control_node',
        name='bno055_heading_control_node',
        parameters=[control_params_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_composition, "' != 'true' and '", node_type, "' == 'standard'"]))
    )

    return LaunchDescription([
        node_type_arg,
        use_composition_arg,
        imu_params_file_arg,
        control_params_file_arg,
        standard_composable_container,
        lifecycle_imu_node,
        lifecycle_heading_node,
        standalone_imu_node,
        standalone_heading_node
    ])
