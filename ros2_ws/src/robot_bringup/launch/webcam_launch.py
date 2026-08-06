import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    video_device = LaunchConfiguration("video_device").perform(context)
    image_width = int(LaunchConfiguration("image_width").perform(context))
    image_height = int(LaunchConfiguration("image_height").perform(context))
    camera_frame_id = LaunchConfiguration("camera_frame_id").perform(context)
    pixel_format = LaunchConfiguration("pixel_format").perform(context)
    pkg_name = LaunchConfiguration("pkg_name").perform(context)

    enable_apriltag = LaunchConfiguration("enable_apriltag").perform(context).lower() in [
        "true",
        "1",
    ]
    tag_family = LaunchConfiguration("tag_family").perform(context)
    tag_size = LaunchConfiguration("tag_size").perform(context)

    webcam_node = Node(
        package=pkg_name,
        executable="v4l2_camera_node",
        name="v4l2_webcam_node",
        output="screen",
        parameters=[
            {
                "video_device": video_device,
                "image_size": [image_width, image_height],
                "camera_frame_id": camera_frame_id,
                "pixel_format": pixel_format,
                "output_encoding": "bgr8",
            }
        ],
        remappings=[
            ("image_raw", "/webcam/image_raw"),
            ("camera_info", "/webcam/camera_info"),
        ],
    )

    launch_nodes = [webcam_node]

    if enable_apriltag:
        bringup_share = get_package_share_directory("robot_bringup")
        apriltag_launch_file = os.path.join(bringup_share, "launch", "apriltag_launch.py")
        apriltag_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltag_launch_file),
            launch_arguments={
                "node_name": "apriltag_webcam_node",
                "image_topic": "/webcam/image_raw",
                "camera_info_topic": "/webcam/camera_info",
                "tag_family": tag_family,
                "tag_size": tag_size,
            }.items(),
        )

        launch_nodes.append(apriltag_launch)

    return launch_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="V4L2 video device path (e.g. /dev/video0)",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="640",
                description="Camera image width",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="480",
                description="Camera image height",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="YUYV",
                description="Pixel format (e.g. YUYV, mjpeg, YUV420)",
            ),
            DeclareLaunchArgument(
                "camera_frame_id",
                default_value="webcam_link",
                description="Frame ID for webcam",
            ),
            DeclareLaunchArgument(
                "pkg_name",
                default_value="v4l2_camera",
                description="ROS 2 V4L2 camera package name (default: v4l2_camera)",
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="false",
                description="Enable AprilTag detection on webcam image",
            ),
            DeclareLaunchArgument(
                "tag_family",
                default_value="tag36h11",
                description="AprilTag family (e.g. tag36h11, tag25h9, tag16h5)",
            ),
            DeclareLaunchArgument(
                "tag_size",
                default_value="0.16",
                description="AprilTag size in meters (e.g. 0.16)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
