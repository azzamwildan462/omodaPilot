#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import torch
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        # ===== Declare parameters (dengan default) =====
        self.declare_parameter("rgb_topic_sub", "/camera/color/image_raw")
        self.declare_parameter("rgb_topic_pub", "/yolo/annotated_image")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_thresh", 0.4)
        self.declare_parameter("imgsz", 640)

        # Ambil nilai parameter
        self.rgb_topic_sub: str = self.get_parameter("rgb_topic_sub").value
        self.rgb_topic_pub: str = self.get_parameter("rgb_topic_pub").value
        self.model_path: str = self.get_parameter("model_path").value
        self.conf_thresh: float = float(self.get_parameter("conf_thresh").value)
        self.imgsz: int = int(self.get_parameter("imgsz").value)

        self.get_logger().info(
            f"Params:\n"
            f"  rgb_topic_sub = {self.rgb_topic_sub}\n"
            f"  rgb_topic_pub = {self.rgb_topic_pub}\n"
            f"  model_path    = {self.model_path}\n"
            f"  conf_thresh   = {self.conf_thresh}\n"
            f"  imgsz         = {self.imgsz}"
        )

        self.bridge = CvBridge()

        # ===== Load model YOLO =====
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLO model '{self.model_path}' on {self.device}...")
        self.model = YOLO(self.model_path)
        self.model.to(self.device)
        self.get_logger().info("YOLO model loaded.")

        # ===== ROS interface =====
        self.subscription = self.create_subscription(
            Image,
            self.rgb_topic_sub,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            self.rgb_topic_pub,
            10
        )

        self.get_logger().info(
            f"Subscribed to: {self.rgb_topic_sub}, "
            f"publishing annotated image to: {self.rgb_topic_pub}"
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
        
        # Cok hardcode, resize frame ke 640x360
        frame = cv2.resize(frame, (640, 360))

        # ===== YOLO inference =====
        results = self.model.predict(
            source=frame,
            conf=self.conf_thresh,
            device=self.device,
            imgsz=self.imgsz,
            verbose=False
        )

        r = results[0]
        annotated = r.plot()

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.publisher.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error (publish): {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
