#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from loguru import logger

import torch
import numpy as np

# modulmu (pure-Py)
from models.nydus_network import make_nydus_network
from models.nydus_infer_core import NydusInferCore

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
        P("mask_topic", "/road_seg/mask")
        P("publish_period", 0.0)     # 0.0 = run setiap callback timer
        P("publish_overlay", False)     # buat debugging
        P("do_mask2laserscan", False)     # penting

        # -------- Get params --------
        g = self.get_parameter
        weights = g("weights").get_parameter_value().string_value
        in_w = int(g("in_width").value)
        in_h = int(g("in_height").value)
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
        self.mask_topic  = g("mask_topic").get_parameter_value().string_value
        period = float(g("publish_period").value)
        self.publish_overlay = bool(g("publish_overlay").value)
        self.do_mask2laserscan = bool(g("do_mask2laserscan").value)

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
            in_w=in_w, in_h=in_h, thr=thr,
            use_wb=use_wb, use_clahe=use_clahe, use_adaptive_gamma=use_adg,
            tta_gamma=tta,
            touch_bottom=touch_bottom, min_area=min_area, ksize=ksize,
            lp_alpha=alpha_lp, crop_upper=crop_upper, crop_lower=crop_lower,
            scene_cut=reset_on, scene_cut_thresh=cut_thr
        )
        logger.info(f"Nydus engine on device: {device}")

        # -------- ROS I/O --------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, qos)
        self.pub_mask = self.create_publisher(Image, self.mask_topic, 10)

        if self.publish_overlay:
            self.pub_overlay = self.create_publisher(Image, self.mask_topic + "/overlay", 10)
        
        if self.do_mask2laserscan:
            self.pub_laserscan = self.create_publisher(LaserScan, self.mask_topic + "/laserscan", 10)
        
        # ring buffer of latest frame
        self.last_bgr = None
        self.last_header = None

        # Timer (optional; 0.0 -> run tiap spin)
        self.timer = self.create_timer(period if period > 0.0 else 0.0, self.on_timer)

        self.has_init = True
        logger.info("nydus_mask_node initialized. Waiting for images…")

    def cb_image(self, msg: Image):
        if not self.has_init:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            logger.error(f"cv_bridge error: {e}")
            return
        with self.lock:
            self.last_bgr = bgr
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
