import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    stereonet_version = LaunchConfiguration("stereonet_version").perform(context)
    publish_visual_enabled = LaunchConfiguration("publish_visual_enabled").perform(
        context
    )
    publish_pcd_enabled = LaunchConfiguration("publish_pcd_enabled").perform(context)
    mipi_rotation = LaunchConfiguration("mipi_rotation").perform(context)
    mipi_channel = LaunchConfiguration("mipi_channel").perform(context)
    mipi_channel2 = LaunchConfiguration("mipi_channel2").perform(context)

    enable_apriltag = LaunchConfiguration("enable_apriltag").perform(
        context
    ).lower() in [
        "true",
        "1",
    ]
    tag_family = LaunchConfiguration("tag_family").perform(context)
    tag_size = LaunchConfiguration("tag_size").perform(context)

    enable_yolo = LaunchConfiguration("enable_yolo").perform(context).lower() in [
        "true",
        "1",
    ]
    model_name = LaunchConfiguration("model_name").perform(context)

    # 230AI MIPI ステレオカメラ専用の設定
    use_mipi_cam = "True"
    stereo_image_topic = "/image_combine_raw"
    camera_info_topic = "/image_combine_raw/right/camera_info"
    left_camera_info_topic = "/image_combine_raw/left/camera_info"

    enable_stereonet = LaunchConfiguration("enable_stereonet").perform(
        context
    ).lower() in ["true", "1"]

    launch_nodes = []

    if enable_stereonet:
        try:
            stereonet_share = get_package_share_directory("hobot_stereonet")
            stereonet_launch_file = os.path.join(
                stereonet_share,
                "launch",
                f"stereonet_model_web_visual_{stereonet_version}.launch.py",
            )
            stereonet_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stereonet_launch_file),
                launch_arguments=list(
                    {
                        "use_mipi_cam": use_mipi_cam,
                        "mipi_rotation": mipi_rotation,
                        "mipi_cal_rotation": "0.0",
                        "mipi_channel": mipi_channel,
                        "mipi_channel2": mipi_channel2,
                        "publish_visual_enabled": publish_visual_enabled,
                        "publish_pcd_enabled": publish_pcd_enabled,
                        "publish_rectify_bgr": "True",
                        "stereo_image_topic": stereo_image_topic,
                        "camera_info_topic": camera_info_topic,
                        "left_camera_info_topic": left_camera_info_topic,
                    }.items()
                ),
            )
            launch_nodes.append(stereonet_launch)
        except Exception as e:
            context.get_logger().error(f"Failed to find hobot_stereonet: {e}")
    else:
        # 🚀 超軽量 MIPI カメラ単体起動 (CPU/BPU 負荷ゼロ)
        from launch_ros.actions import Node

        mipi_node = Node(
            package="mipi_cam",
            executable="mipi_cam",
            name="mipi_cam",
            output="screen",
            parameters=[
                {
                    "video_device_name": "default",
                    "channel": int(mipi_channel),
                    "channel2": int(mipi_channel2),
                    "device_mode": "dual",
                    "dual_combine": 1,
                    "framerate": 10.0,
                    "image_width": 1920,
                    "image_height": 1080,
                    "out_format_name": "nv12",
                    "io_method_name": "ros",
                    "cal_alpha": 0.0,
                    "gdc_enable": True,
                    "lpwm_enable": False,
                }
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        )
        launch_nodes.append(mipi_node)

    bringup_share = get_package_share_directory("robot_bringup")

    if enable_apriltag:
        # NV12 (1080p) -> mono8 高速グレースケール変換 ＋ CameraInfo 完全同期配信ノード
        # 📐 base_link -> default_cam (カメラ位置 TF 接続: 前方 +0.265m, 左 +0.035m, 上 +0.193m)
        from launch_ros.actions import Node

        camera_tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_tf",
            arguments=[
                "--x",
                "0.265",
                "--y",
                "0.035",
                "--z",
                "0.193",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "default_cam",
            ],
            output="screen",
        )
        launch_nodes.append(camera_tf_node)

        apriltag_launch_file = os.path.join(
            bringup_share, "launch", "apriltag_launch.py"
        )
        apriltag_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltag_launch_file),
            launch_arguments=list(
                {
                    "node_name": "apriltag_csi_node",
                    "image_topic": "/image_left_raw",
                    "camera_info_topic": "/image_combine_raw/left/camera_info",
                    "camera_frame_id": "default_cam",
                    "tag_family": tag_family,
                    "tag_size": tag_size,
                }.items()
            ),
        )
        launch_nodes.append(apriltag_launch)

    if enable_yolo:
        yolo_launch_file = os.path.join(bringup_share, "launch", "yolo_launch.py")
        yolo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch_file),
            launch_arguments=list(
                {
                    "image_topic": "/StereoNetNode/rectify_left_image",
                    "model_name": model_name,
                }.items()
            ),
        )
        launch_nodes.append(yolo_launch)

    return launch_nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_stereonet",
                default_value="false",
                description="Enable heavy 3D hobot_stereonet depth inference and web visualizer (default false for lightweight AprilTag)",
            ),
            DeclareLaunchArgument(
                "stereonet_version",
                default_value="v2.4_int16",
                description="hobot_stereonet model version for 230AI (e.g. v2.4_int16, v2.5_int16)",
            ),
            DeclareLaunchArgument(
                "publish_visual_enabled",
                default_value="True",
                description="Enable publishing visualization image for Web UI",
            ),
            DeclareLaunchArgument(
                "publish_pcd_enabled",
                default_value="True",
                description="Enable publishing PointCloud2",
            ),
            DeclareLaunchArgument(
                "mipi_rotation",
                default_value="0.0",
                description="MIPI rotation for 230AI camera (default 0.0)",
            ),
            DeclareLaunchArgument(
                "mipi_channel",
                default_value="2",
                description="MIPI channel for left camera (default 2)",
            ),
            DeclareLaunchArgument(
                "mipi_channel2",
                default_value="0",
                description="MIPI channel for right camera (default 0)",
            ),
            DeclareLaunchArgument(
                "enable_apriltag",
                default_value="false",
                description="Enable AprilTag detection node",
            ),
            DeclareLaunchArgument(
                "tag_family",
                default_value="16h5",
                description="AprilTag family (16h5)",
            ),
            DeclareLaunchArgument(
                "tag_size",
                default_value="0.18",
                description="AprilTag size in meters (default 0.18 for 18cm tag)",
            ),
            DeclareLaunchArgument(
                "enable_yolo",
                default_value="false",
                description="Enable YOLO ball detection node",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="yolov5s",
                description="YOLO model name (e.g. yolov5s, yolov8n)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
