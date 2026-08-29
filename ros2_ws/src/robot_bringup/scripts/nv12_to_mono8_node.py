#!/usr/bin/env python3
"""
NV12 to mono8 (Grayscale) Zero-Copy Image Converter

230AI MIPI ステレオカメラの生画像 (NV12, 1920x1080) の先頭 Y プレーン（輝度データ）を
そのまま mono8 (Grayscale) 形式として抽出し、フル解像度のまま AprilTag 検出器へ配信する。
色変換計算が不要なため、CPU 負荷はほぼ 0% で 1080p の鮮明な画像が得られる。
"""

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class Nv12ToMono8Node(Node):
    def __init__(self):
        super().__init__("nv12_to_mono8_node")

        self.declare_parameter("input_topic", "/image_left_raw")
        self.declare_parameter("output_topic", "/camera/left_mono8")
        self.declare_parameter("camera_info_topic", "/camera_left_info")
        self.declare_parameter("output_camera_info_topic", "/camera/camera_info")
        self.declare_parameter("target_fps", 5.0)
        self.declare_parameter("use_fallback_camera_info", False)
        self.declare_parameter("camera_fx", 800.0)
        self.declare_parameter("camera_fy", 800.0)
        self.declare_parameter("camera_cx", 960.0)
        self.declare_parameter("camera_cy", 540.0)
        self.declare_parameter("crop_width", 0)
        self.declare_parameter("crop_height", 0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        output_camera_info_topic = self.get_parameter("output_camera_info_topic").value
        self.use_fallback_camera_info_ = self.get_parameter(
            "use_fallback_camera_info"
        ).value
        self.camera_fx_ = float(self.get_parameter("camera_fx").value)
        self.camera_fy_ = float(self.get_parameter("camera_fy").value)
        self.camera_cx_ = float(self.get_parameter("camera_cx").value)
        self.camera_cy_ = float(self.get_parameter("camera_cy").value)
        self.crop_width_ = int(self.get_parameter("crop_width").value)
        self.crop_height_ = int(self.get_parameter("crop_height").value)
        self.target_fps_ = self.get_parameter("target_fps").value
        self.min_interval_sec_ = 1.0 / self.target_fps_ if self.target_fps_ > 0 else 0.0
        self.last_pub_time_ = 0.0

        self.pub_ = self.create_publisher(Image, output_topic, 10)
        self.camera_info_pub_ = self.create_publisher(
            CameraInfo, output_camera_info_topic, 10
        )

        self.last_camera_info_ = None
        self.info_sub_ = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.sub_ = self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"NV12 -> mono8 Converter started: {input_topic} -> {output_topic}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        # X5 GDC output is already rectified. Some mipi_cam versions leave the
        # model name empty even though they publish zero distortion, which can
        # be rejected by consumers using image_geometry.
        if not msg.distortion_model and all(value == 0.0 for value in msg.d):
            msg.distortion_model = "plumb_bob"
        self.last_camera_info_ = msg

    def image_callback(self, msg: Image):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            self.min_interval_sec_ > 0.0
            and now_sec - self.last_pub_time_ < self.min_interval_sec_
        ):
            return

        encoding = msg.encoding.lower()
        if encoding not in ("nv12", "nv12_8", "mono8"):
            self.get_logger().error(
                f"Unsupported input encoding: {msg.encoding}",
                throttle_duration_sec=5.0,
            )
            return

        input_step = msg.step if msg.step >= msg.width else msg.width
        required_y_size = input_step * msg.height
        if len(msg.data) < required_y_size:
            self.get_logger().error(
                f"Image buffer is too short: {len(msg.data)} < {required_y_size}",
                throttle_duration_sec=5.0,
            )
            return

        requested_width = self.crop_width_ if self.crop_width_ > 0 else msg.width
        requested_height = self.crop_height_ if self.crop_height_ > 0 else msg.height
        crop_width = max(1, min(requested_width, msg.width))
        crop_height = max(1, min(requested_height, msg.height))
        crop_x = (msg.width - crop_width) // 2
        crop_y = (msg.height - crop_height) // 2

        if self.last_camera_info_ is None:
            if not self.use_fallback_camera_info_:
                self.get_logger().warn(
                    "Waiting for CameraInfo; image is not forwarded yet",
                    throttle_duration_sec=5.0,
                )
                return

        self.last_pub_time_ = now_sec
        mono_msg = Image()
        mono_msg.header = msg.header
        if not mono_msg.header.frame_id:
            mono_msg.header.frame_id = "default_cam"
        mono_msg.height = crop_height
        mono_msg.width = crop_width
        mono_msg.encoding = "mono8"
        mono_msg.is_bigendian = msg.is_bigendian
        mono_msg.step = crop_width
        if (
            crop_width == msg.width
            and crop_height == msg.height
            and input_step == msg.width
        ):
            mono_msg.data = msg.data[: msg.width * msg.height]
        else:
            y_plane = np.frombuffer(
                msg.data, dtype=np.uint8, count=required_y_size
            ).reshape((msg.height, input_step))
            mono_msg.data = y_plane[
                crop_y : crop_y + crop_height, crop_x : crop_x + crop_width
            ].tobytes()

        if self.last_camera_info_ is not None:
            info_msg = copy.deepcopy(self.last_camera_info_)
        else:
            info_msg = CameraInfo()
            info_msg.width = msg.width
            info_msg.height = msg.height
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.k = [
                self.camera_fx_,
                0.0,
                self.camera_cx_,
                0.0,
                self.camera_fy_,
                self.camera_cy_,
                0.0,
                0.0,
                1.0,
            ]
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.p = [
                self.camera_fx_,
                0.0,
                self.camera_cx_,
                0.0,
                0.0,
                self.camera_fy_,
                self.camera_cy_,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            self.get_logger().warn(
                "Using fallback CameraInfo; calibrate camera for accurate pose",
                throttle_duration_sec=30.0,
            )
        info_msg.header = mono_msg.header
        info_msg.width = crop_width
        info_msg.height = crop_height
        if len(info_msg.k) == 9:
            info_msg.k[2] -= crop_x
            info_msg.k[5] -= crop_y
        if len(info_msg.p) == 12:
            info_msg.p[2] -= crop_x
            info_msg.p[6] -= crop_y
        info_msg.roi.x_offset = 0
        info_msg.roi.y_offset = 0
        info_msg.roi.width = 0
        info_msg.roi.height = 0
        info_msg.roi.do_rectify = False
        self.pub_.publish(mono_msg)
        self.camera_info_pub_.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nv12ToMono8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
