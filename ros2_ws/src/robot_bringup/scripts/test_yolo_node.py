#!/usr/bin/env python3
import rclcpp
from rclcpp.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
import cv2
import numpy as np

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class TestYoloNode(Node):
    def __init__(self):
        super().__init__('test_yolo_node')
        self.image_topic = self.declare_parameter('image_topic', '/webcam/image_raw').value
        self.model_name = self.declare_parameter('model_name', 'yolov8n.pt').value
        self.conf_thresh = self.declare_parameter('conf_thresh', 0.25).value

        self.get_logger().info(f"Loading YOLOv8 model: {self.model_name}...")
        if YOLO is not None:
            self.model = YOLO(self.model_name)
            self.get_logger().info("YOLOv8 Model successfully loaded!")
        else:
            self.model = None
            self.get_logger().error("ultralytics package not installed yet. Running dummy/installing mode.")

        self.sub_ = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.pub_detections_ = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.pub_annotated_ = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.bridge = CvBridge() if CvBridge else None

    def image_callback(self, msg: Image):
        if self.model is None:
            return

        # ROS Image -> OpenCV BGR
        if msg.encoding == 'bgr8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        elif msg.encoding == 'rgb8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'mono8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            return

        # 推論
        results = self.model(frame, conf=self.conf_thresh, verbose=False)
        det_array = Detection2DArray()
        det_array.header = msg.header

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0].item())
                xywh = box.xywh[0].tolist()

                # ボール（sports ball / ball）または全物体
                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(xywh[0])
                det.bbox.center.position.y = float(xywh[1])
                det.bbox.size_x = float(xywh[2])
                det.bbox.size_y = float(xywh[3])

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.push_back(det)

                self.get_logger().info(f"[YOLO Detected] {cls_name} ({conf*100:.1f}%) at center=({xywh[0]:.1f}, {xywh[1]:.1f}), size=({xywh[2]:.1f}x{xywh[3]:.1f})")

        self.pub_detections_.publish(det_array)

def main(args=None):
    rclcpp.init(args=args)
    node = TestYoloNode()
    rclcpp.spin(node)
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
