import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory("robot_bringup")
    script_path = os.path.join(pkg_bringup, "scripts", "test_yolo_node.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/webcam/image_raw",
            description="Input image topic for YOLOv8 (e.g. /webcam/image_raw or /image_left_raw)",
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="yolov8n.pt",
            description="YOLOv8 model weight (yolov8n.pt, yolov8s.pt)",
        ),
        DeclareLaunchArgument(
            "conf_thresh",
            default_value="0.25",
            description="Confidence threshold (0.0 to 1.0)",
        ),
        Node(
            package="robot_bringup",
            executable="test_yolo_node.py",
            name="yolo_ball_detector",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "model_name": LaunchConfiguration("model_name"),
                "conf_thresh": LaunchConfiguration("conf_thresh"),
            }],
        ),
    ])
