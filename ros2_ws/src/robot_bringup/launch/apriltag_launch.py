import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    node_name = LaunchConfiguration("node_name").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    tag_family = LaunchConfiguration("tag_family").perform(context)
    max_hamming = int(LaunchConfiguration("max_hamming").perform(context))
    tag_size = float(LaunchConfiguration("tag_size").perform(context))
    pkg_name = LaunchConfiguration("pkg_name").perform(context)
    camera_frame_id = LaunchConfiguration("camera_frame_id").perform(context)
    node_params = {
        "image_transport": "raw",
        "family": tag_family,
        "size": tag_size,
        "max_hamming": max_hamming,
        "publish_tf": True,
        "pose_estimation_method": "pnp",
        "decimate": 1.0,
        "blur": 0.0,
        "threads": 4,
        "debug": False,
        "refine_edges": 1,
    }
    if camera_frame_id:
        node_params["camera_frame"] = camera_frame_id

    apriltag_node = Node(
        package=pkg_name,
        executable="apriltag_node",
        name=node_name,
        output="screen",
        remappings=[
            ("image_rect", image_topic),
            ("camera_info", camera_info_topic),
            ("image", image_topic),
        ],
        parameters=[node_params],
    )

    return [apriltag_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_name",
                default_value="apriltag_node",
                description="Node name for AprilTag detector to avoid collisions",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/StereoNetNode/rectify_left_image",
                description="Input camera image topic for AprilTag detection",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/image_combine_raw/left/camera_info",
                description="Input camera info topic for AprilTag detection",
            ),
            DeclareLaunchArgument(
                "tag_family",
                default_value="16h5",
                description="AprilTag family (16h5)",
            ),
            DeclareLaunchArgument(
                "max_hamming",
                default_value="1",
                description="Maximum corrected bit errors (0 rejects hamming=1 false positives)",
            ),
            DeclareLaunchArgument(
                "tag_size",
                default_value="0.18",
                description="AprilTag size in meters (0.18 for 18cm tag)",
            ),
            DeclareLaunchArgument(
                "pkg_name",
                default_value="apriltag_ros",
                description="AprilTag package name (apriltag_ros or hobot_apriltag)",
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="",
                description="Optional override for camera frame ID",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
