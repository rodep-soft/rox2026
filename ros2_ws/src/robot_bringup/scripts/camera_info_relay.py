#!/usr/bin/env python3
"""
camera_info リレーノード

image_transport::CameraSubscriber は画像トピック名から camera_info トピックを
自動導出する（例: /StereoNetNode/rectify_left_image → /StereoNetNode/camera_info）。
しかし RDK X5 の StereoNetNode は camera_info を
/StereoNetNode/rectify_left_image/camera_info に配信している。

このリレーノードが実際のトピックから image_transport が期待するトピックへ
メッセージを中継することで同期を成立させる。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoRelay(Node):
    def __init__(self):
        super().__init__("camera_info_relay")

        self.declare_parameter(
            "input_topic", "/StereoNetNode/rectify_left_image/camera_info"
        )
        self.declare_parameter("output_topic", "/StereoNetNode/camera_info")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(CameraInfo, output_topic, 10)
        self.sub1 = self.create_subscription(CameraInfo, input_topic, self._relay_cb, 10)
        self.sub2 = self.create_subscription(
            CameraInfo, "/image_combine_raw/left/camera_info", self._relay_cb, 10
        )
        self.sub3 = self.create_subscription(
            CameraInfo, "/image_left_raw/camera_info", self._relay_cb, 10
        )

        self.get_logger().info(f"Relaying CameraInfo to {output_topic}")

    def _relay_cb(self, msg: CameraInfo):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
