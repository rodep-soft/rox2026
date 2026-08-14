#!/usr/bin/env python3
"""
NV12 to mono8 (Grayscale) Zero-Copy Image Converter

230AI MIPI ステレオカメラの生画像 (NV12, 1920x1080) の先頭 Y プレーン（輝度データ）を
そのまま mono8 (Grayscale) 形式として抽出し、フル解像度のまま AprilTag 検出器へ配信する。
色変換計算が不要なため、CPU 負荷はほぼ 0% で 1080p の鮮明な画像が得られる。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Nv12ToMono8Node(Node):
    def __init__(self):
        super().__init__("nv12_to_mono8_node")

        self.declare_parameter("input_topic", "/image_left_raw")
        self.declare_parameter("output_topic", "/camera/left_mono8")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.pub_ = self.create_publisher(Image, output_topic, 5)
        self.sub_ = self.create_subscription(Image, input_topic, self.image_callback, 5)

        self.get_logger().info(
            f"NV12 -> mono8 Converter started: {input_topic} -> {output_topic}"
        )

    def image_callback(self, msg: Image):
        # NV12 形式の場合、先頭 width * height バイトがそのまま Y（輝度）データ
        if msg.encoding.lower() in ["nv12", "yuv420"]:
            y_size = msg.width * msg.height
            mono_msg = Image()
            mono_msg.header = msg.header
            mono_msg.height = msg.height
            mono_msg.width = msg.width
            mono_msg.encoding = "mono8"
            mono_msg.is_bigendian = msg.is_bigendian
            mono_msg.step = msg.width
            mono_msg.data = msg.data[:y_size]
            self.pub_.publish(mono_msg)
        elif msg.encoding.lower() in ["mono8", "bgr8", "rgb8"]:
            # すでに標準形式の場合はそのまま転送
            self.pub_.publish(msg)
        else:
            # その他の形式でも Y プレーンとして先頭サイズを安全に抽出
            y_size = min(len(msg.data), msg.width * msg.height)
            mono_msg = Image()
            mono_msg.header = msg.header
            mono_msg.height = msg.height
            mono_msg.width = msg.width
            mono_msg.encoding = "mono8"
            mono_msg.is_bigendian = msg.is_bigendian
            mono_msg.step = msg.width
            mono_msg.data = msg.data[:y_size]
            self.pub_.publish(mono_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nv12ToMono8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
