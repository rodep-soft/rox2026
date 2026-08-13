import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    image_topic = LaunchConfiguration("image_topic").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    score_threshold = float(LaunchConfiguration("score_threshold").perform(context))
    use_bpu = LaunchConfiguration("use_bpu").perform(context).lower() in ["true", "1"]

    if use_bpu:
        # RDK X5 BPU 加速 YOLO ノード (hobot_dnn_node / dnn_node_sample)
        try:
            dnn_share = get_package_share_directory("dnn_node_sample")
            dnn_launch_file = os.path.join(
                dnn_share, "launch", "dnn_node_sample.launch.py"
            )
            yolo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(dnn_launch_file),
                launch_arguments=list(
                    {
                        "dnn_sample_config_file": f"{model_name}config.json",
                        "image_topic": image_topic,
                        "score_threshold": str(score_threshold),
                    }.items()
                ),
            )
            return [yolo_node]
        except Exception:
            context.get_logger().info(
                "dnn_node_sample not found. Falling back to standard ROS 2 yolo node."
            )

    # 標準 ROS 2 YOLO ノード (yolo_ros / ultralytics)
    yolo_node = Node(
        package="yolo_ros",
        executable="yolo_node",
        name="yolo_ball_detector",
        output="screen",
        parameters=[
            {
                "model": model_name,
                "score_threshold": score_threshold,
            }
        ],
        remappings=[
            ("image_raw", image_topic),
        ],
    )

    return [yolo_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/StereoNetNode/rectify_left_image",
                description="Input image topic for YOLO object detection",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="yolov5s",
                description="YOLO model name (e.g. yolov5s, yolov8n)",
            ),
            DeclareLaunchArgument(
                "score_threshold",
                default_value="0.4",
                description="Confidence score threshold for detection (0.0 ~ 1.0)",
            ),
            DeclareLaunchArgument(
                "use_bpu",
                default_value="true",
                description="Use RDK X5 BPU hardware accelerator for YOLO",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
