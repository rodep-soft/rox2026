import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    tag_family = LaunchConfiguration("tag_family").perform(context)
    tag_size = float(LaunchConfiguration("tag_size").perform(context))
    pkg_name = LaunchConfiguration("pkg_name").perform(context)

    apriltag_node = Node(
        package=pkg_name,
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        remappings=[
            ("image_rect", image_topic),
            ("camera_info", camera_info_topic),
            ("image", image_topic),
        ],
        parameters=[
            {
                "family": tag_family,
                "size": tag_size,
                "max_hamming": 0,
                "publish_tf": True,
            }
        ],
    )

    return [apriltag_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/StereoNetNode/rectify_left_image",
                description="Input camera image topic for AprilTag detection",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/StereoNetNode/rectify_left_camera_info",
                description="Input camera info topic for AprilTag detection",
            ),
            DeclareLaunchArgument(
                "tag_family",
                default_value="tag36h11",
                description="AprilTag family (e.g. tag36h11, tag25h9, tag16h5)",
            ),
            DeclareLaunchArgument(
                "tag_size",
                default_value="0.16",
                description="AprilTag size in meters (e.g. 0.16 for 16cm tag)",
            ),
            DeclareLaunchArgument(
                "pkg_name",
                default_value="apriltag_ros",
                description="AprilTag package name (apriltag_ros or hobot_apriltag)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
