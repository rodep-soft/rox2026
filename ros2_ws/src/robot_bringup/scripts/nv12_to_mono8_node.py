#!/usr/bin/env python3
"""
NV12 to mono8 (Grayscale) Zero-Copy Image Converter

230AI MIPI ステレオカメラの生画像 (NV12, 1920x1080) の先頭 Y プレーン（輝度データ）を
そのまま mono8 (Grayscale) 形式として抽出し、フル解像度のまま AprilTag 検出器へ配信する。
色変換計算が不要なため、CPU 負荷はほぼ 0% で 1080p の鮮明な画像が得られる。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class Nv12ToMono8Node(Node):
    def __init__(self):
        super().__init__("nv12_to_mono8_node")

        self.declare_parameter("input_topic", "/image_left_raw")
        self.declare_parameter("output_topic", "/camera/left_mono8")
        self.declare_parameter("target_fps", 10.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.target_fps_ = self.get_parameter("target_fps").value
        self.min_interval_sec_ = 1.0 / self.target_fps_ if self.target_fps_ > 0 else 0.0
        self.last_pub_time_ = 0.0

        self.pub_ = self.create_publisher(Image, output_topic, 10)
        self.camera_info_pub_ = self.create_publisher(
            CameraInfo, "/camera/camera_info", 10
        )

        self.last_camera_info_ = None
        self.info_sub_1_ = self.create_subscription(
            CameraInfo, "/image_combine_raw/left/camera_info", self.camera_info_callback, 10
        )
        self.info_sub_2_ = self.create_subscription(
            CameraInfo, "/image_left_raw/camera_info", self.camera_info_callback, 10
        )
        self.sub_ = self.create_subscription(
            Image, input_topic, self.image_callback, 10
        )

        self.get_logger().info(
            f"NV12 -> mono8 Converter started: {input_topic} -> {output_topic}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.last_camera_info_ = msg

    def image_callback(self, msg: Image):
        # 1. mono8 変換 (NV12 またはその他のフォーマットから Y プレーン抽出)
        y_size = min(len(msg.data), msg.width * msg.height)
        mono_msg = Image()
        mono_msg.header = msg.header
        mono_msg.height = msg.height
        mono_msg.width = msg.width
        mono_msg.encoding = "mono8"
        mono_msg.is_bigendian = msg.is_bigendian
        mono_msg.step = msg.width
        mono_msg.data = msg.data[:y_size]

        # 2. 画像と CameraInfo を全く同じタイムスタンプ・Frame ID で同時配信 (同期率 100%)
        self.pub_.publish(mono_msg)

        info_msg = CameraInfo()
        if self.last_camera_info_ is not None:
            info_msg = self.last_camera_info_
        else:
            # 📷 デフォルト 1080p カメラ内部パラメータ (SC230AI MIPI: fx=800, fy=800, cx=960, cy=540)
            info_msg.width = msg.width
            info_msg.height = msg.height
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.k = [800.0, 0.0, 960.0, 0.0, 800.0, 540.0, 0.0, 0.0, 1.0]
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.p = [800.0, 0.0, 960.0, 0.0, 0.0, 800.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        info_msg.header = msg.header
        self.camera_info_pub_.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nv12ToMono8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
