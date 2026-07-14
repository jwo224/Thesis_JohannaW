#!/usr/bin/env python3

import cv2
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult

from object_detection.msg import Yolov8Inference  # FIXED

bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.image = None

        self.declare_parameter('camera_topic', '/camera_front/image_raw')
        self.camera_topic = self.get_parameter(
            'camera_topic'
        ).get_parameter_value().string_value

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = None
        self.create_camera_subscription(self.camera_topic)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def create_camera_subscription(self, topic):
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.camera_callback,
            self.qos_profile
        )

        self.get_logger().info(f"Camera switched to: {topic}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'camera_topic' and param.type_ == Parameter.Type.STRING:
                self.camera_topic = param.value
                self.create_camera_subscription(self.camera_topic)

        return SetParametersResult(successful=True)

    def camera_callback(self, data):
        self.image = bridge.imgmsg_to_cv2(data, "bgr8")


class Yolo_subscriber(Node):

    def __init__(self, camera_node):
        super().__init__('yolo_subscriber')

        self.camera_node = camera_node

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)

        self.img_pub = self.create_publisher(
            Image,
            "/inference_result_cv2",
            1)

    def yolo_callback(self, data):

        if self.camera_node.image is None:
            return  # no image yet

        img = self.camera_node.image.copy()

        for r in data.yolov8_inference:
            cv2.rectangle(
                img,
                (r.top, r.left),
                (r.bottom, r.right),
                (255, 255, 0),
                2
            )

        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.img_pub.publish(img_msg)


if __name__ == '__main__':
    rclpy.init()

    camera_subscriber = Camera_subscriber()
    yolo_subscriber = Yolo_subscriber(camera_subscriber)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera_subscriber)
    executor.add_node(yolo_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
