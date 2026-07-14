#!/usr/bin/env python3

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('frame_id', 'camera_optical_link')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('camera_fx', 615.0)
        self.declare_parameter('camera_fy', 615.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        self.device = self.get_parameter('device').value
        self.camera_name = self.get_parameter('camera_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image,
            self.get_parameter('image_topic').value,
            qos_profile_sensor_data,
        )
        self.info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            qos_profile_sensor_data,
        )

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open camera device {self.device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self.publish_frame)
        self.last_warn = 0.0
        self.get_logger().info(
            f'{self.camera_name}: publishing {self.device} as {self.width}x{self.height} @ {self.fps:.1f} Hz'
        )

    def make_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def publish_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            now = time.monotonic()
            if now - self.last_warn > 2.0:
                self.get_logger().warn(f'{self.camera_name}: no frame from {self.device}')
                self.last_warn = now
            return

        stamp = self.get_clock().now().to_msg()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        self.image_pub.publish(image_msg)
        self.info_pub.publish(self.make_camera_info(stamp))

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = OpenCVCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
