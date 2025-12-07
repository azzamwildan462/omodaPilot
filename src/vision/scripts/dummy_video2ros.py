#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge

import numpy as np

import tf2_ros
from geometry_msgs.msg import TransformStamped


class VideoFramePublisher(Node):
    def __init__(self):
        super().__init__('video_frame_publisher')

        # ===== Params =====
        self.declare_parameter('source', '0')                 # '0' (webcam index) atau path file
        self.declare_parameter('topic',  '/camera/color/image_raw')  # mirip realsense2_node
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        # Depth params (dummy)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('depth_camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('depth_constant_m', 3.0)       # kedalaman konstan (meter)

        self.declare_parameter('frame_id', 'camera_link')     # frame "base" kamera
        self.declare_parameter('loop', True)                  # loop EOF jika file
        self.declare_parameter('fps_override', 0.0)           # 0.0 = pakai fps dari video
        self.declare_parameter('resize_w', 0)                 # 0 = tidak resize
        self.declare_parameter('resize_h', 0)                 # 0 = tidak resize
        self.declare_parameter('encoding', 'bgr8')            # bgr8/mono8/rgb8

        src_param     = self.get_parameter('source').get_parameter_value().string_value
        topic         = self.get_parameter('topic').get_parameter_value().string_value
        caminfo_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.publish_depth = self.get_parameter('publish_depth').get_parameter_value().bool_value
        depth_topic        = self.get_parameter('depth_topic').get_parameter_value().string_value
        depth_caminfo_topic = self.get_parameter('depth_camera_info_topic').get_parameter_value().string_value
        self.depth_const_m = float(self.get_parameter('depth_constant_m').value)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.loop     = self.get_parameter('loop').get_parameter_value().bool_value
        self.fps_over = float(self.get_parameter('fps_override').value)
        self.out_w    = int(self.get_parameter('resize_w').value)
        self.out_h    = int(self.get_parameter('resize_h').value)
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        # ===== TF frames (dummy RealSense-style) =====
        # parent: camera_link, child: camera_color_optical_frame
        self.tf_parent_frame = self.frame_id                    # "camera_link"
        self.tf_child_frame  = 'camera_color_optical_frame'     # fixed name

        # ===== OpenCV capture =====
        self.cap = self._open_capture(src_param)

        # Ambil fps dari sumber
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if math.isnan(fps) or fps <= 0.0:
            fps = 30.0
        if self.fps_over > 0.0:
            fps = self.fps_over
        self.period = 1.0 / max(1e-6, fps)
        self.get_logger().info(f'Video source: "{src_param}" | fps={fps:.2f} | period={self.period:.4f}s')

        # Ambil ukuran frame sumber
        src_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        src_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if src_w <= 0 or src_h <= 0:
            src_w, src_h = 640, 480  # fallback

        # Tentukan ukuran output (setelah resize)
        self.final_w = self.out_w if self.out_w > 0 else src_w
        self.final_h = self.out_h if self.out_h > 0 else src_h
        self.get_logger().info(f'Frame size src=({src_w}x{src_h}) -> out=({self.final_w}x{self.final_h})')

        # ===== Publisher =====
        self.pub          = self.create_publisher(Image, topic, 1)
        self.caminfo_pub  = self.create_publisher(CameraInfo, caminfo_topic, 1)
        self.bridge       = CvBridge()

        # Depth publishers
        if self.publish_depth:
            self.depth_pub     = self.create_publisher(Image, depth_topic, 1)
            self.depth_caminfo_pub = self.create_publisher(CameraInfo, depth_caminfo_topic, 1)
        else:
            self.depth_pub = None
            self.depth_caminfo_pub = None

        # ===== Dummy CameraInfo (D455-ish) =====
        self.camera_info_msg = self._make_dummy_d455_caminfo(self.final_w, self.final_h)
        self.depth_camera_info_msg = self._make_dummy_d455_caminfo(self.final_w, self.final_h)

        # ===== Dummy static depth frame (32FC1) =====
        if self.publish_depth:
            # konstanta dalam meter, float32
            self.depth_frame = np.full(
                (self.final_h, self.final_w),
                float(self.depth_const_m),
                dtype=np.float32
            )
            self.get_logger().info(
                f'Dummy depth: constant {self.depth_const_m:.2f} m, '
                f'shape=({self.final_h}x{self.final_w}), encoding=32FC1'
            )
        else:
            self.depth_frame = None

        # ===== TF Broadcaster =====
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ===== Timer =====
        self.timer = self.create_timer(self.period, self._on_timer)

        # Stat
        self.frame_count = 0

    # ------------------------------------------------------------------
    def _open_capture(self, src_param: str) -> cv2.VideoCapture:
        cap = None
        try:
            idx = int(src_param)
            cap = cv2.VideoCapture(idx)
        except ValueError:
            cap = cv2.VideoCapture(src_param)

        if not cap or not cap.isOpened():
            raise RuntimeError(f'Gagal membuka source: {src_param}')
        return cap

    # ------------------------------------------------------------------
    def _make_dummy_d455_caminfo(self, width: int, height: int) -> CameraInfo:
        caminfo = CameraInfo()
        caminfo.width = width
        caminfo.height = height
        caminfo.distortion_model = 'plumb_bob'

        caminfo.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fov_x_deg = 87.0
        fov_y_deg = 58.0
        fov_x = math.radians(fov_x_deg)
        fov_y = math.radians(fov_y_deg)

        fx = (width / 2.0) / math.tan(fov_x / 2.0)
        fy = (height / 2.0) / math.tan(fov_y / 2.0)
        cx = width / 2.0
        cy = height / 2.0

        caminfo.k[0] = fx
        caminfo.k[1] = 0.0
        caminfo.k[2] = cx
        caminfo.k[3] = 0.0
        caminfo.k[4] = fy
        caminfo.k[5] = cy
        caminfo.k[6] = 0.0
        caminfo.k[7] = 0.0
        caminfo.k[8] = 1.0

        caminfo.r[0] = 1.0
        caminfo.r[1] = 0.0
        caminfo.r[2] = 0.0
        caminfo.r[3] = 0.0
        caminfo.r[4] = 1.0
        caminfo.r[5] = 0.0
        caminfo.r[6] = 0.0
        caminfo.r[7] = 0.0
        caminfo.r[8] = 1.0

        caminfo.p[0]  = fx
        caminfo.p[1]  = 0.0
        caminfo.p[2]  = cx
        caminfo.p[3]  = 0.0
        caminfo.p[4]  = 0.0
        caminfo.p[5]  = fy
        caminfo.p[6]  = cy
        caminfo.p[7]  = 0.0
        caminfo.p[8]  = 0.0
        caminfo.p[9]  = 0.0
        caminfo.p[10] = 1.0
        caminfo.p[11] = 0.0

        self.get_logger().info(
            f'Dummy CameraInfo D455: w={width}, h={height}, '
            f'fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}'
        )

        return caminfo

    # ------------------------------------------------------------------
    def _publish_dummy_tf(self, stamp):
        """
        Broadcast TF:
          parent: self.tf_parent_frame  (default: "camera_link")
          child : self.tf_child_frame   (default: "camera_color_optical_frame")

        Rotasi sesuai konvensi optical frame:
        - X: right
        - Y: down
        - Z: forward
        relatif terhadap camera_link (X forward, Y left, Z up)
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.tf_parent_frame          # "camera_link"
        t.child_frame_id = self.tf_child_frame            # "camera_color_optical_frame"

        # Tidak ada translasi (kamera di origin camera_link)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Rotasi link -> optical:
        # quaternion: x=-0.5, y=0.5, z=-0.5, w=0.5
        t.transform.rotation.x = -0.5
        t.transform.rotation.y =  0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w =  0.5

        self.tf_broadcaster.sendTransform(t)


    # ------------------------------------------------------------------
    def _on_timer(self):
        if not self.cap or not self.cap.isOpened():
            self.get_logger().warn('Capture closed; menghentikan timer.')
            self.timer.cancel()
            return

        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.get_logger().info('EOF — looping ke awal.')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            else:
                self.get_logger().info('EOF — stop publish.')
                self.timer.cancel()
                return

        if self.out_w > 0 and self.out_h > 0:
            frame = cv2.resize(frame, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)

        img_to_pub = frame
        enc = self.encoding
        if enc == 'rgb8':
            img_to_pub = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif enc == 'mono8':
            img_to_pub = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.tf_child_frame   # "camera_color_optical_frame"

        # ===== Publish color Image =====
        try:
            msg = self.bridge.cv2_to_imgmsg(img_to_pub, encoding=enc)
            msg.header = header
            self.pub.publish(msg)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # ===== Publish color CameraInfo =====
        caminfo = self.camera_info_msg
        caminfo.header = header
        self.caminfo_pub.publish(caminfo)

        # ===== Publish dummy DEPTH (32FC1) + CameraInfo =====
        if self.publish_depth and self.depth_pub is not None and self.depth_frame is not None:
            depth_msg = Image()
            depth_msg.header = header
            h, w = self.depth_frame.shape
            depth_msg.height = h
            depth_msg.width  = w
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = 0
            depth_msg.step = w * 4  # float32
            depth_msg.data = self.depth_frame.tobytes()
            self.depth_pub.publish(depth_msg)

            depth_ci = self.depth_camera_info_msg
            depth_ci.header = header
            self.depth_caminfo_pub.publish(depth_ci)

        # ===== Broadcast dummy TF camera_link -> camera_color_optical_frame =====
        self._publish_dummy_tf(header.stamp)

    # ------------------------------------------------------------------
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
