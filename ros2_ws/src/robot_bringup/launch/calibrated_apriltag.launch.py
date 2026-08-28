import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("robot_bringup")
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    mipi_channel = int(LaunchConfiguration("mipi_channel").perform(context))
    framerate = float(LaunchConfiguration("framerate").perform(context))
    tag_family = LaunchConfiguration("tag_family").perform(context)
    tag_size = LaunchConfiguration("tag_size").perform(context)
    detector_threads = LaunchConfiguration("detector_threads").perform(context)
    detector_decimate = LaunchConfiguration("detector_decimate").perform(context)
    detector_blur = LaunchConfiguration("detector_blur").perform(context)
    detector_refine = LaunchConfiguration("detector_refine").perform(context)
    detector_sharpening = LaunchConfiguration("detector_sharpening").perform(context)
    enable_foxglove = LaunchConfiguration("enable_foxglove").perform(
        context
    ).lower() in ("true", "1")
    foxglove_port = LaunchConfiguration("foxglove_port").perform(context)

    mipi_node = Node(
        package="mipi_cam",
        executable="mipi_cam",
        name="mipi_cam",
        output="screen",
        parameters=[{
            "video_device": "default",
            "channel": mipi_channel,
            "device_mode": "single",
            "framerate": framerate,
            "image_width": 1920,
            "image_height": 1080,
            "out_format": "nv12",
            "io_method": "ros",
            "cal_alpha": 0.0,
            "gdc_enable": True,
            "lpwm_enable": False,
            "frame_id": "default_cam",
            "camera_calibration_file_path": calibration_file,
        }],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # Cache the short-lived post-GDC CameraInfo when available. The measured
    # post-GDC values below keep CameraInfo publishing if that burst is missed.
    mono_node = Node(
        package="robot_bringup",
        executable="nv12_to_mono8_node.py",
        name="nv12_to_mono8_node",
        output="screen",
        parameters=[{
            "input_topic": "/image_raw",
            "output_topic": "/camera/left_mono8",
            "camera_info_topic": "/image_raw/camera_info",
            "output_camera_info_topic": "/camera/camera_info",
            "target_fps": 0.0,
            "use_fallback_camera_info": True,
            "camera_fx": 807.8291351671487,
            "camera_fy": 810.3272057704562,
            "camera_cx": 959.5,
            "camera_cy": 539.5,
        }],
    )

    base_to_camera_link_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_link_tf",
        arguments=[
            "--x", "0.265", "--y", "0.035", "--z", "0.193",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
        output="screen",
    )

    camera_link_to_optical_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_to_optical_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "-1.57079632679", "--pitch", "0.0", "--yaw", "-1.57079632679",
            "--frame-id", "camera_link", "--child-frame-id", "default_cam",
        ],
        output="screen",
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "apriltag_launch.py")
        ),
        launch_arguments={
            "node_name": "apriltag_csi_node",
            "image_topic": "/camera/left_mono8",
            "camera_info_topic": "/camera/camera_info",
            "camera_frame_id": "default_cam",
            "tag_family": tag_family,
            "tag_size": tag_size,
            "max_hamming": "0",
            "detector_threads": detector_threads,
            "detector_decimate": detector_decimate,
            "detector_blur": detector_blur,
            "detector_refine": detector_refine,
            "detector_sharpening": detector_sharpening,
        }.items(),
    )

    launch_nodes = [
        mipi_node,
        mono_node,
        base_to_camera_link_node,
        camera_link_to_optical_node,
        apriltag_launch,
    ]
    if enable_foxglove:
        launch_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "foxglove_bridge.launch.py")
                ),
                launch_arguments={"port": foxglove_port}.items(),
            )
        )

    return launch_nodes


def generate_launch_description():
    default_calibration_file = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config", "camera", "sc230ai_left.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "calibration_file",
            default_value=default_calibration_file,
            description="SC230AI calibration YAML used by the X5 GDC",
        ),
        DeclareLaunchArgument(
            "mipi_channel",
            default_value="1",
            description="Working SC230AI MIPI channel",
        ),
        DeclareLaunchArgument(
            "framerate",
            default_value="5.0",
            description="Camera and AprilTag input rate",
        ),
        DeclareLaunchArgument(
            "tag_family",
            default_value="16h5",
            description="AprilTag family",
        ),
        DeclareLaunchArgument(
            "tag_size",
            default_value="0.18",
            description="AprilTag edge length in metres",
        ),
        DeclareLaunchArgument(
            "detector_threads",
            default_value="4",
            description="AprilTag worker threads",
        ),
        DeclareLaunchArgument(
            "detector_decimate",
            default_value="1.0",
            description="Use full 1920x1080 resolution for distant tags",
        ),
        DeclareLaunchArgument(
            "detector_blur",
            default_value="0.0",
            description="Neutral blur for changing illumination",
        ),
        DeclareLaunchArgument(
            "detector_refine",
            default_value="true",
            description="Refine detected tag corners",
        ),
        DeclareLaunchArgument(
            "detector_sharpening",
            default_value="0.25",
            description="Conservative decoding sharpening for changing illumination",
        ),
        DeclareLaunchArgument(
            "enable_foxglove",
            default_value="false",
            description="Start Foxglove Bridge for detection inspection",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove Bridge WebSocket port",
        ),
        OpaqueFunction(function=launch_setup),
    ])
