#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
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
        self.conf_thresh = self.declare_parameter('conf_thresh', 0.22).value

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
        self.real_ball_diameter = 0.20  # 20cm

        # 白飛び防止＆コントラスト補正用 LUT (Gamma = 1.4)
        inv_gamma = 1.0 / 1.4
        self.gamma_lut = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

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

        # ── 1. 露出オーバー・白飛び抑制 ＆ コントラスト補正 (CLAHE + Gamma) ──
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv[:, :, 2] = self.clahe.apply(hsv[:, :, 2])
        enhanced = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        enhanced = cv2.LUT(enhanced, self.gamma_lut)

        # ── 2. YOLO推論 ──
        results = self.model(enhanced, conf=self.conf_thresh, verbose=False)
        det_array = Detection2DArray()
        det_array.header = msg.header

        best_ball_pose = None
        min_ball_z = 999.0
        frame_h, frame_w = frame.shape[:2]

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0].item())
                xywh = box.xywh[0].tolist()

                # ボール判定 (ball または sports ball)
                is_ball = cls_name in ['ball', 'sports ball']
                
                # ── 3. 床面ROI ＆ 球体幾何学妥当性フィルター ──
                bw = float(xywh[2])
                bh = float(xywh[3])
                cy = float(xywh[1])
                aspect_ratio = bw / max(1.0, bh)
                area_ratio = (bw * bh) / float(frame_w * frame_h)

                # 画面上部 18% (天井・遠景背景) は除外
                if cy < 0.18 * frame_h:
                    continue

                # ボールは球体のため 0.70 <= aspect <= 1.40 かつ画面の半分以下
                if not (0.70 <= aspect_ratio <= 1.40) or area_ratio > 0.45 or bw < 20:
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
                det_array.detections.append(det)

                # ボールの場合: 3D実空間位置を計算
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

                    # 最も近いボールを追跡対象として選定
                    if est_z < min_ball_z:
                        min_ball_z = est_z
                        best_ball_pose = pose_msg
                        best_ball_info = (xywh, conf, est_z)

        # 最も近いボールの3次元座標を /ball_pose に配信 & 画面に単一ロックオン描画
        if best_ball_pose is not None:
            self.pub_ball_pose_.publish(best_ball_pose)
            xywh, conf, est_z = best_ball_info
            bw, bh = xywh[2], xywh[3]
            x1 = int(xywh[0] - bw / 2.0)
            y1 = int(xywh[1] - bh / 2.0)
            x2 = int(xywh[0] + bw / 2.0)
            y2 = int(xywh[1] + bh / 2.0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(
                frame,
                f"ball {conf:.2f} (Z={est_z:.2f}m)",
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            self.get_logger().info(
                f"[CLOSEST BALL TARGET] 3D Pos: (X={best_ball_pose.pose.position.x:.2f}m, Y={best_ball_pose.pose.position.y:.2f}m, Dist Z={best_ball_pose.pose.position.z:.2f}m)"
            )

        self.pub_detections_.publish(det_array)

        # /yolo/annotated_image 配信 (Foxglove 可視化用)
        ann_msg = Image()
        ann_msg.header = msg.header
        ann_msg.height = msg.height
        ann_msg.width = msg.width
        ann_msg.encoding = 'bgr8'
        ann_msg.is_bigendian = 0
        ann_msg.step = msg.width * 3
        ann_msg.data = frame.tobytes()
        self.pub_annotated_.publish(ann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestYoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
