#!/usr/bin/env python3

from ultralytics import YOLO
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from object_detection.msg import InferenceResult
from rcl_interfaces.msg import SetParametersResult
from object_detection.msg import Yolov8Inference

bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # -----------------------------
        # Camera topic parameter
        # -----------------------------
        self.declare_parameter('camera_topic', '/camera_front/image_raw')
        self.camera_topic = self.get_parameter(
            'camera_topic'
        ).get_parameter_value().string_value

        # -----------------------------
        # YOLO model   model_path = os.path.expanduser(
        #    '~/Nursing-Home-Robot/src/object_detection/scripts/yolov8n.pt'
        #)
        # -----------------------------
        
        model_path = os.path.expanduser(
            '~/Nursing-Home-Robot/src/object_detection/scripts/trolley.pt'
        )
        self.model = YOLO(model_path)

        self.yolov8_inference = Yolov8Inference()

        # -----------------------------
        # QoS for Gazebo cameras
        # -----------------------------
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -----------------------------
        # Create initial subscription
        # -----------------------------
        self.subscription = None
        self.create_camera_subscription(self.camera_topic)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.yolov8_pub = self.create_publisher(
            Yolov8Inference,
            "/Yolov8_Inference",
            1)

        self.img_pub = self.create_publisher(
            Image,
            "/inference_result",
            1)

        # -----------------------------
        # Dynamic parameter callback
        # -----------------------------
        self.add_on_set_parameters_callback(self.parameter_callback)

    # ------------------------------------------------
    # Create subscription helper
    # ------------------------------------------------
    def create_camera_subscription(self, topic):

        if self.subscription is not None:
            self.destroy_subscription(self.subscription)

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.camera_callback,
            self.qos_profile
        )

        self.get_logger().info(f"Switched to camera: {topic}")

    # ------------------------------------------------
    # Dynamic parameter callback
    # ------------------------------------------------
    def parameter_callback(self, params):

        for param in params:
            if param.name == 'camera_topic' and param.type_ == Parameter.Type.STRING:
                self.camera_topic = param.value
                self.create_camera_subscription(self.camera_topic)

        return SetParametersResult(successful=True)

    # ------------------------------------------------
    # Camera callback
    # ------------------------------------------------
    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            for box in r.boxes:
                inference_result = InferenceResult()
                b = box.xyxy[0].cpu().numpy()
                c = int(box.cls)

                inference_result.class_name = self.model.names[c]
                inference_result.top = int(b[0])
                inference_result.left = int(b[1])
                inference_result.bottom = int(b[2])
                inference_result.right = int(b[3])

                self.yolov8_inference.yolov8_inference.append(inference_result)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
