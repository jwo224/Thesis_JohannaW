#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


FRONT_CAMERA = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.4.3:1.0-video-index0"
REAR_CAMERA = "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.4.2:1.0-video-index0"


class FrontRearCameraNode(Node):
    def __init__(self):
        super().__init__("front_rear_camera_node")

        self.bridge = CvBridge()

        self.width = 640
        self.height = 480
        self.fps = 10
        self.jpeg_quality = 70

        self.front_cap = self.open_camera(FRONT_CAMERA, "front")
        self.rear_cap = self.open_camera(REAR_CAMERA, "rear")

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        
        self.front_pub = self.create_publisher(
            Image,
            "/front/image_raw",
            image_qos,
        )
        self.rear_pub = self.create_publisher(
            Image,
            "/rear/image_raw",
            image_qos,
        )

        self.front_compressed_pub = self.create_publisher(
            CompressedImage,
            "/front/image_raw/compressed",
            image_qos,
        )
        self.rear_compressed_pub = self.create_publisher(
            CompressedImage,
            "/rear/image_raw/compressed",
            image_qos,
        )

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frames)

        self.get_logger().info("Front/rear camera node started")
        self.get_logger().info(f"Resolution: {self.width}x{self.height}")
        self.get_logger().info(f"FPS: {self.fps}")
        self.get_logger().info(f"JPEG quality: {self.jpeg_quality}")

    def open_camera(self, device, name):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error(f"Could not open {name} camera: {device}")
            return cap

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f"{name} camera opened: {device} "
            f"({actual_width:.0f}x{actual_height:.0f} @ {actual_fps:.0f} FPS)"
        )

        return cap

    def publish_one_camera(self, cap, raw_pub, compressed_pub, frame_id):
        if not cap.isOpened():
            return

        ok, frame = cap.read()

        if not ok or frame is None:
            self.get_logger().warn(f"No frame from {frame_id}")
            return

        stamp = self.get_clock().now().to_msg()

        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = frame_id
        raw_pub.publish(raw_msg)

        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if success:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = frame_id
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            compressed_pub.publish(compressed_msg)

    def publish_frames(self):
        self.publish_one_camera(
            self.front_cap,
            self.front_pub,
            self.front_compressed_pub,
            "front_camera",
        )

        self.publish_one_camera(
            self.rear_cap,
            self.rear_pub,
            self.rear_compressed_pub,
            "rear_camera",
        )

    def destroy_node(self):
        if self.front_cap.isOpened():
            self.front_cap.release()

        if self.rear_cap.isOpened():
            self.rear_cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontRearCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()