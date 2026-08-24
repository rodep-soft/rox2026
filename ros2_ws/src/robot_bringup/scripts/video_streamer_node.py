#!/usr/bin/env python3
import os
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoStreamerNode(Node):
    def __init__(self):
        super().__init__('video_streamer_node')
        self.video_path = self.declare_parameter(
            'video_path', '/home/tatsv/Documents/image/IMG_1447.mov'
        ).value
        self.topic_name = self.declare_parameter(
            'topic_name', '/webcam/image_raw'
        ).value
        self.fps = self.declare_parameter('fps', 30.0).value

        self.pub_ = self.create_publisher(Image, self.topic_name, 10)
        self.bridge = CvBridge()

        if not os.path.exists(self.video_path):
            self.get_logger().error(f"Video file not found: {self.video_path}")
            return

        self.cap = cv2.VideoCapture(self.video_path)
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"Streaming video [{self.video_path}] -> [{self.topic_name}] @ {self.fps} FPS"
        )

    def timer_callback(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            # ループ再生
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
