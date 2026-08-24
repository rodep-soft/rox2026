#!/usr/bin/env python3
import sys
sys.path.insert(0, '/opt/ros/humble/local/lib/python3.10/dist-packages')
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class VideoTrackingBridgeNode(Node):
    def __init__(self):
        super().__init__('video_tracking_bridge_node')
        self.video_path = '/home/tatsv/Documents/image/IMG_1447.mov'
        self.pub_raw = self.create_publisher(Image, '/webcam/image_raw', 10)
        self.pub_annotated = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(self.video_path)
        self.model = YOLO('/home/tatsv/rox2026/yolo_ball_pipeline/models/molten_ball_best.pt')
        self.timer = self.create_timer(1.0 / 30.0, self.loop)
        self.get_logger().info("🎥 Streaming video tracking to /yolo/annotated_image for Foxglove!")

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                return

        now = self.get_clock().now().to_msg()
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = now
        raw_msg.header.frame_id = 'camera_color_optical_frame'
        self.pub_raw.publish(raw_msg)

        h, w = frame.shape[:2]
        res = self.model(frame, conf=0.18, verbose=False)
        annotated = frame.copy()

        for box in res[0].boxes:
            conf = float(box.conf[0].item())
            xywh = box.xywh[0].tolist()
            bw, bh = float(xywh[2]), float(xywh[3])
            aspect = bw / max(1.0, bh)
            area_ratio = (bw * bh) / float(w * h)

            if (0.70 <= aspect <= 1.40) and area_ratio <= 0.45 and bw >= 15:
                x1 = int(xywh[0] - bw/2)
                y1 = int(xywh[1] - bh/2)
                x2 = int(xywh[0] + bw/2)
                y2 = int(xywh[1] + bh/2)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(annotated, f"ball {conf:.2f}", (x1, max(25, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        ann_msg.header.stamp = now
        ann_msg.header.frame_id = 'camera_color_optical_frame'
        self.pub_annotated.publish(ann_msg)

def main():
    rclpy.init()
    node = VideoTrackingBridgeNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
