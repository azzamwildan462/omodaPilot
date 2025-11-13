#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class VideoFramePublisher(Node):
    def __init__(self):
        super().__init__('video_frame_publisher')

        # ===== Params =====
        self.declare_parameter('source', '0')                 # '0' (webcam index) atau path file
        self.declare_parameter('topic',  '/camera/image_raw') # topik publish
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('loop', True)                  # loop EOF jika file
        self.declare_parameter('fps_override', 0.0)           # 0.0 = pakai fps dari video
        self.declare_parameter('resize_w', 0)                 # 0 = tidak resize
        self.declare_parameter('resize_h', 0)                 # 0 = tidak resize
        self.declare_parameter('encoding', 'bgr8')            # bgr8/mono8/rgb8

        src_param     = self.get_parameter('source').get_parameter_value().string_value
        topic         = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.loop     = self.get_parameter('loop').get_parameter_value().bool_value
        self.fps_over = float(self.get_parameter('fps_override').value)
        self.out_w    = int(self.get_parameter('resize_w').value)
        self.out_h    = int(self.get_parameter('resize_h').value)
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        # ===== OpenCV capture =====
        # ijinkan "0", "1" sebagai webcam index
        self.cap = self._open_capture(src_param)

        # ambil fps dari sumber
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if math.isnan(fps) or fps <= 0.0:
            fps = 30.0
        if self.fps_over > 0.0:
            fps = self.fps_over
        self.period = 1.0 / max(1e-6, fps)
        self.get_logger().info(f'Video source: "{src_param}" | fps={fps:.2f} | period={self.period:.4f}s')

        # ===== Publisher =====
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(Image, topic, qos)
        self.bridge = CvBridge()

        # ===== Timer =====
        self.timer = self.create_timer(self.period, self._on_timer)

        # Stat
        self.frame_count = 0

    def _open_capture(self, src_param: str) -> cv2.VideoCapture:
        # coba parse int untuk webcam, kalau gagal treat as path
        cap = None
        try:
            idx = int(src_param)
            cap = cv2.VideoCapture(idx)
        except ValueError:
            cap = cv2.VideoCapture(src_param)

        if not cap or not cap.isOpened():
            raise RuntimeError(f'Gagal membuka source: {src_param}')
        return cap

    def _on_timer(self):
        if not self.cap or not self.cap.isOpened():
            self.get_logger().warn('Capture closed; menghentikan timer.')
            self.timer.cancel()
            return

        ok, frame = self.cap.read()
        if not ok:
            # EOF / error
            if self.loop:
                self.get_logger().info('EOF — looping ke awal.')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            else:
                self.get_logger().info('EOF — stop publish.')
                self.timer.cancel()
                return

        # Optional resize
        if self.out_w > 0 and self.out_h > 0:
            frame = cv2.resize(frame, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)

        # Convert encoding if needed (default bgr8)
        img_to_pub = frame
        enc = self.encoding
        if enc == 'rgb8':
            img_to_pub = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif enc == 'mono8':
            img_to_pub = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Build message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        try:
            msg = self.bridge.cv2_to_imgmsg(img_to_pub, encoding=enc)
            msg.header = header
            self.pub.publish(msg)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = VideoFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
