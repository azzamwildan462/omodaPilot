#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
from loguru import logger

import torch
import numpy as np
import cv2

import tf2_ros
from rclpy.time import Time

from models.nydus_network import make_nydus_network
from models.nydus_infer_core import NydusInferCore
from utils.mask2laserscan import Mask2LaserScan

def transform_to_matrix(transform_msg) -> np.ndarray:
    """
    Convert geometry_msgs/Transform (or TransformStamped.transform)
    to 4x4 homogeneous matrix.
    """
    t = transform_msg.translation
    q = transform_msg.rotation

    x, y, z, w = q.x, q.y, q.z, q.w

    # Quaternion -> rotation matrix
    # sumber rumus standard
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array([
        [1.0 - 2.0 * (yy + zz),     2.0 * (xy - wz),         2.0 * (xz + wy)],
        [2.0 * (xy + wz),           1.0 - 2.0 * (xx + zz),   2.0 * (yz - wx)],
        [2.0 * (xz - wy),           2.0 * (yz + wx),         1.0 - 2.0 * (xx + yy)]
    ], dtype=float)

    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = R
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    return T

class NydusMaskNode(Node):
    def __init__(self):
        super().__init__("nydus_mask_node")
        self.lock = Lock()
        self.bridge = CvBridge()
        self.has_init = False

        # -------- Declare params --------
        P = self.declare_parameter
        P("weights", "")
        P("in_width", 640)
        P("in_height", 360)
        P("thr", 0.5)
        P("use_cuda", False)

        # preproc toggles
        P("use_wb", False)
        P("use_clahe", False)
        P("use_adaptive_gamma", False)
        P("tta_gamma", "")           # e.g. "0.9,1.1"

        # post-proc & temporal
        P("touch_bottom", True)
        P("min_area", 1500)
        P("ksize", 7)
        P("alpha_lp", 0.7)
        P("crop_upper", 175)
        P("crop_lower", 100)
        P("reset_on_scene_cut", True)
        P("scene_cut_thresh", 0.35)

        # topics & timer
        P("image_topic", "/camera/image_raw")
        P("cam_info_topic", "/camera/camera_info")
        P("depth_image_topic", "/camera/camera_depth")
        P("camera_frame_id", "camera_link")
        P("base_frame_id", "base_link")
        P("mask_topic", "/road_seg/mask")
        P("publish_period", 0.0)     # 0.0 = run setiap callback timer
        P("publish_overlay", False)     # buat debugging
        P("do_mask2laserscan", False)     # penting
        P("mask2laserscan_px2m_strategy", 0)     # penting
        P("mask2laserscan_scan_strategy", 0)     # penting

        # -------- Get params --------
        g = self.get_parameter
        weights = g("weights").get_parameter_value().string_value
        self.in_w = int(g("in_width").value)
        self.in_h = int(g("in_height").value)
        thr  = float(g("thr").value)
        use_cuda = bool(g("use_cuda").value)

        use_wb = bool(g("use_wb").value)
        use_clahe = bool(g("use_clahe").value)
        use_adg = bool(g("use_adaptive_gamma").value)
        tta_str = g("tta_gamma").get_parameter_value().string_value
        tta = []
        if tta_str.strip():
            try:
                tta = [float(x) for x in tta_str.split(",") if x.strip()]
            except Exception:
                self.get_logger().warn("tta_gamma invalid, ignored")

        touch_bottom = bool(g("touch_bottom").value)
        min_area = int(g("min_area").value)
        ksize = int(g("ksize").value)
        alpha_lp = float(g("alpha_lp").value)
        crop_upper = int(g("crop_upper").value)
        crop_lower = int(g("crop_lower").value)
        reset_on = bool(g("reset_on_scene_cut").value)
        cut_thr = float(g("scene_cut_thresh").value)

        self.image_topic = g("image_topic").get_parameter_value().string_value
        self.cam_info_topic = g("cam_info_topic").get_parameter_value().string_value
        self.depth_image_topic = g("depth_image_topic").get_parameter_value().string_value
        self.camera_frame_id = g("camera_frame_id").get_parameter_value().string_value
        self.base_frame_id   = g("base_frame_id").get_parameter_value().string_value
        self.mask_topic  = g("mask_topic").get_parameter_value().string_value
        period = float(g("publish_period").value)
        self.publish_overlay = bool(g("publish_overlay").value)
        self.do_mask2laserscan = bool(g("do_mask2laserscan").value)
        m2ls_px2m_strategy = int(g("mask2laserscan_px2m_strategy").value)
        m2ls_scan_strategy = int(g("mask2laserscan_scan_strategy").value)

        # -------- Logger --------
        logger.remove()
        logger.add(sys.stdout, colorize=True,
                   format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | "
                          "<cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
                   enqueue=True)

        # -------- Engine (no I/O) --------
        device = "cuda" if (use_cuda and torch.cuda.is_available()) else "cpu"
        self.engine = NydusInferCore(
            model=make_nydus_network(num_classes=1),
            device=device,
            weights=weights,
            in_w=self.in_w, in_h=self.in_h, thr=thr,
            use_wb=use_wb, use_clahe=use_clahe, use_adaptive_gamma=use_adg,
            tta_gamma=tta,
            touch_bottom=touch_bottom, min_area=min_area, ksize=ksize,
            lp_alpha=alpha_lp, crop_upper=crop_upper, crop_lower=crop_lower,
            scene_cut=reset_on, scene_cut_thresh=cut_thr
        )
        if self.do_mask2laserscan:
            self.m2ls = Mask2LaserScan(logger, m2ls_scan_strategy, m2ls_px2m_strategy, base_frame="base_link")

            # TF buffer & listener (listen T base_link <- camera_frame_id)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        logger.info(f"Nydus engine on device: {device}")

        # -------- ROS I/O --------
        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, 1)
        self.sub_cam_info = self.create_subscription(CameraInfo, self.cam_info_topic, self._cb_camera_info, 1)

        if m2ls_px2m_strategy == 0:
            self.sub_depth = self.create_subscription(Image, self.depth_image_topic, self._cb_depth_image, 1)

         # Publishers
        self.pub_mask = self.create_publisher(Image, self.mask_topic, 1)

        if self.publish_overlay:
            self.pub_overlay = self.create_publisher(Image, self.mask_topic + "/overlay", 1)
        
        if self.do_mask2laserscan:
            self.pub_laserscan = self.create_publisher(LaserScan, self.mask_topic + "/laserscan", 1)
        
        # ring buffer of latest frame
        self.last_bgr = None
        self.last_header = None

        # Timer (optional; 0.0 -> run tiap spin)
        self.timer = self.create_timer(period if period > 0.0 else 0.0, self.on_timer)

        self.has_init = True
        logger.info("nydus_mask_node initialized. Waiting for images…")

        self.is_camera_info_set = False # ahhahhahahahahah
        self.has_extrinsic = False  # sudah pernah set T_cam2base? aooasoksaosadoasdasnd
        self.has_extrinsic_updated = False
    
    def _cb_depth_image(self, msg: Image):
        if not self.has_init or not self.is_camera_info_set or not self.do_mask2laserscan:
            return
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth_frame.dtype == np.uint16:
                depth_frame = depth_frame.astype(np.float32) * 0.001  # mm to meter
            elif depth_frame.dtype == np.float32:
                pass  # sudah meter
            else:
                logger.error(f"Depth image has unsupported dtype: {depth_frame.dtype}")
                return
        except Exception as e:
            logger.error(f"cv_bridge error (depth): {e}")
            return

        self.m2ls.px2m_converter.set_depth_frame(depth_frame)
    
    def _cb_camera_info(self, msg: CameraInfo):
        if not self.has_init or not self.do_mask2laserscan:
            return
        if not self.is_camera_info_set:
            # Kalau kamu melakukan resize sebelum infer, idealnya CameraInfo juga di-scale
            # ke (in_w, in_h). Untuk sekarang, langsung pakai apa adanya.
            self.m2ls.px2m_converter.set_camera_info(msg)
            self.is_camera_info_set = True
            logger.info(f"CameraInfo set from topic {self.cam_info_topic}")
        
    def cb_image(self, msg: Image):
        if not self.has_init:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            bgr_resized = cv2.resize(bgr, (self.in_w, self.in_h))

        except Exception as e:
            logger.error(f"cv_bridge error: {e}")
            return
        with self.lock:
            self.last_bgr = bgr_resized
            self.last_header = msg.header

    def on_timer(self):
        if not self.has_init:
            return
        with self.lock:
            if self.last_bgr is None:
                return
            frame = self.last_bgr.copy()
            header = self.last_header

        # Run core (mask only)
        mask_u8, _overlay_bgr, stats = self.engine(frame)   # overlay not used

        # if stats is not None:
        #     t_ms = stats.get("t_ms", 0.0)
        #     logger.info(f"Processed frame | t_ms: {t_ms:.6f}")

        if self.do_mask2laserscan and self.tf_buffer is not None:
            try:
                # Lookup transform base <- camera_frame_id
                # target: base_frame, source: camera_frame
                tf_msg = self.tf_buffer.lookup_transform(
                    self.base_frame_id,
                    self.camera_frame_id,
                    Time()  # latest available
                )
                if self.has_extrinsic_updated is False:
                    T_cam2base = transform_to_matrix(tf_msg.transform)
                    self.m2ls.px2m_converter.set_extrinsic_cam_to_base(T_cam2base)
                    self.has_extrinsic_updated = True
                self.has_extrinsic = True
            except Exception as e:
                if not self.has_extrinsic:
                    logger.warning(f"TF lookup failed (base={self.base_frame_id}, cam={self.camera_frame_id}): {e}")
                # kalau belum ada TF, skip mask2laserscan dulu

        # -------- Optional: mask -> LaserScan --------
        if self.do_mask2laserscan and self.has_extrinsic and self.is_camera_info_set:
            # Mask2LaserScan.__call__(mask, header: Optional[Header])
            laser_scan = self.m2ls(mask_u8, header)
            if laser_scan is not None and self.pub_laserscan is not None:
                self.pub_laserscan.publish(laser_scan)

        # Publish mask as mono8 (0/255)
        mask_img = (mask_u8 * 255).astype(np.uint8)
        out_header = Header()
        out_header.stamp = self.get_clock().now().to_msg()
        out_header.frame_id = header.frame_id if header else "camera"

        msg_mask = self.bridge.cv2_to_imgmsg(mask_img, encoding="mono8")
        msg_mask.header = out_header
        self.pub_mask.publish(msg_mask)

        if self.publish_overlay:
            overlay_bgr = _overlay_bgr if _overlay_bgr is not None else cv2.cvtColor(mask_img, cv2.COLOR_GRAY2BGR)
            msg_overlay = self.bridge.cv2_to_imgmsg(overlay_bgr, encoding="bgr8")
            msg_overlay.header = out_header
            self.pub_overlay.publish(msg_overlay)

def main(args=None):
    rclpy.init(args=args)
    node = NydusMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
