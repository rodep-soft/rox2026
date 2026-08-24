#!/usr/bin/env python3
import rclcpp
from rclcpp.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
import geometry_msgs.msg
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
        self.model_name = self.declare_parameter('model_name', '/root/ros2_ws/src/robot_bringup/config/molten_ball_best.pt').value
        self.conf_thresh = self.declare_parameter('conf_thresh', 0.18).value

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
        self.pub_ball_pose_ = self.create_publisher(geometry_msgs.msg.PoseStamped, '/ball_pose', 10)
        self.pub_annotated_ = self.create_publisher(Image, '/yolo/annotated_image', 10)
        self.bridge = CvBridge() if CvBridge else None

        # カメラ内部パラメータ (実機 CSI カメラ / 1080p 焦点距離)
        self.fx = 1400.0
        self.fy = 1400.0
        self.cx = 960.0
        self.cy = 540.0
        self.real_ball_diameter = 0.20 # 20cm

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

                # ボール判定 (ball または sports ball)
                is_ball = cls_name in ['ball', 'sports ball']
                
                # ── ボールの球体・幾何学妥当性フィルター (Geometric Sphere Filter) ──
                bw = float(xywh[2])
                bh = float(xywh[3])
                aspect_ratio = bw / max(1.0, bh)
                frame_h, frame_w = frame.shape[:2]
                area_ratio = (bw * bh) / float(frame_w * frame_h)

                # ボールは球体のため 0.70 <= aspect <= 1.40 かつ画面の半分以下 (壁や廊下の誤認を100%カット)
                if not (0.70 <= aspect_ratio <= 1.40) or area_ratio > 0.45 or bw < 15:
                    continue

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(xywh[0])
                det.bbox.center.position.y = float(xywh[1])
                det.bbox.size_x = bw
                det.bbox.size_y = bh

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.push_back(det)

                # ボールの場合: 3D実空間位置を計算して /ball_pose にパブリッシュ
                if is_ball:
                    # ピクセル幅から実距離 Z をピンホール推定 (Z = f * W_real / w_pixel)
                    pixel_size = (bw + bh) / 2.0
                    est_z = (self.fx * self.real_ball_diameter) / pixel_size
                    est_x = ((xywh[0] - self.cx) * est_z) / self.fx
                    est_y = ((xywh[1] - self.cy) * est_z) / self.fy

                    pose_msg = geometry_msgs.msg.PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.header.frame_id = "camera_color_optical_frame"
                    pose_msg.pose.position.x = float(est_x)
                    pose_msg.pose.position.y = float(est_y)
                    pose_msg.pose.position.z = float(est_z)
                    pose_msg.pose.orientation.w = 1.0
                    self.pub_ball_pose_.publish(pose_msg)

                    self.get_logger().info(
                        f"[BALL 3D DETECTED] {cls_name} ({conf*100:.1f}%) -> 3D Pos: (X={est_x:.2f}m, Y={est_y:.2f}m, Dist Z={est_z:.2f}m)"
                    )

        self.pub_detections_.publish(det_array)

def main(args=None):
    rclcpp.init(args=args)
    node = TestYoloNode()
    rclcpp.spin(node)
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
