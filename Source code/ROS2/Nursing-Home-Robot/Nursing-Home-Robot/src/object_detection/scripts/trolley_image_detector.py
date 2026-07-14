#!/usr/bin/env python3

import os
import random

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO

from object_detection.msg import InferenceResult
from object_detection.msg import Yolov8Inference


def tensor_scalar(value):
    if hasattr(value, "item"):
        return value.item()
    if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
        return tensor_scalar(value[0])
    return value


class TrolleyImageDetector(Node):
    def __init__(self):
        super().__init__("trolley_image_detector")

        self.bridge = CvBridge()

        self.model_path = os.path.expanduser(
            "~/Nursing-Home-Robot/src/object_detection/scripts/trolleys.pt"
        )
        self.model = YOLO(self.model_path)

        self.image_folder = os.path.expanduser(
            "~/Nursing-Home-Robot/src/object_detection/test_images/trolley_rotated"
        )
        self.fallback_image_folder = os.path.expanduser(
            "~/Nursing-Home-Robot/src/object_detection/test_images/trolleys"
        )
        self.max_detection_attempts = 12
        self.minimum_confidence = 0.25
        self.last_image_msg = None

        self.command_pub = self.create_publisher(String, "/trolley_command", 10)
        self.yolo_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 10)
        image_qos = QoSProfile(depth=1)
        image_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.image_pub = self.create_publisher(Image, "/inference_result", image_qos)

        self.detect_sub = self.create_subscription(
            Bool,
            "/detect_trolley_request",
            self.detect_callback,
            10,
        )
        self.create_timer(1.0, self.republish_last_image)

        self.get_logger().info("Trolley image detector started.")
        self.get_logger().info(f"Using model: {self.model_path}")
        self.get_logger().info(f"Using image folder: {self.image_folder}")
        self.get_logger().info(f"Fallback image folder: {self.fallback_image_folder}")
        self.get_logger().info("Waiting for /detect_trolley_request...")

    def get_random_image_path(self):
        valid_extensions = [".jpg", ".jpeg", ".png", ".bmp"]

        image_folder = self.image_folder
        if not os.path.isdir(image_folder):
            self.get_logger().warn(
                f"Image folder does not exist: {image_folder}. "
                f"Trying fallback: {self.fallback_image_folder}"
            )
            image_folder = self.fallback_image_folder

        if not os.path.isdir(image_folder):
            self.get_logger().error(f"Image folder does not exist: {image_folder}")
            return None

        image_files = sorted([
            file_name
            for file_name in os.listdir(image_folder)
            if os.path.splitext(file_name.lower())[1] in valid_extensions
        ])

        if not image_files:
            self.get_logger().error(
                f"No images found in folder: {image_folder}"
            )
            return None

        selected_image = random.choice(image_files)
        return os.path.join(image_folder, selected_image)

    def detect_callback(self, msg):
        if not msg.data:
            return

        self.get_logger().info("Detection request received.")

        last_results = None
        last_yolo_msg = None

        for attempt in range(1, self.max_detection_attempts + 1):
            image_path = self.get_random_image_path()
            if image_path is None:
                break

            self.get_logger().info(
                f"Attempt {attempt}/{self.max_detection_attempts}: "
                f"loading image {os.path.basename(image_path)}"
            )

            img = cv2.imread(image_path)
            if img is None:
                self.get_logger().error(f"Could not read image: {image_path}")
                continue

            try:
                results = self.model(img, verbose=False)
            except Exception as exc:
                self.get_logger().error(
                    f"YOLO inference failed for {image_path}: {exc}"
                )
                continue

            if not results:
                self.get_logger().warn("YOLO returned no result objects.")
                continue

            detected_classes = []
            yolo_msg = Yolov8Inference()
            yolo_msg.header.frame_id = "trolley_image"
            yolo_msg.header.stamp = self.get_clock().now().to_msg()

            for result in results:
                for box in result.boxes:
                    class_id = int(tensor_scalar(box.cls))
                    class_name = self.model.names[class_id]
                    confidence = float(tensor_scalar(box.conf))

                    b = box.xyxy[0].cpu().numpy()

                    inference_result = InferenceResult()
                    inference_result.class_name = class_name
                    inference_result.left = int(b[0])
                    inference_result.top = int(b[1])
                    inference_result.right = int(b[2])
                    inference_result.bottom = int(b[3])

                    yolo_msg.yolov8_inference.append(inference_result)
                    detected_classes.append(
                        {
                            "class_name": class_name,
                            "confidence": confidence,
                        }
                    )

                    self.get_logger().info(
                        f"Detected {class_name} with confidence {confidence:.2f}"
                    )

            last_results = results
            last_yolo_msg = yolo_msg
            trolley_command = self.decide_trolley_type(detected_classes)

            if trolley_command is None:
                self.get_logger().warn(
                    "No confident trash/laundry trolley type in this image."
                )
                continue

            self.publish_detection_outputs(results, yolo_msg)

            command_msg = String()
            command_msg.data = trolley_command
            self.command_pub.publish(command_msg)

            self.get_logger().info(f"Published trolley command: {trolley_command}")
            return

        if last_results is not None and last_yolo_msg is not None:
            self.publish_detection_outputs(last_results, last_yolo_msg)

        self.get_logger().warn(
            "No trolley type detected after all random image attempts."
        )
        self.publish_unknown()

    def publish_detection_outputs(self, results, yolo_msg):
        annotated_frame = results[0].plot()
        image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        image_msg.header.frame_id = "trolley_image"
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.last_image_msg = image_msg
        self.image_pub.publish(image_msg)
        self.yolo_pub.publish(yolo_msg)

    def republish_last_image(self):
        if self.last_image_msg is None:
            return
        self.last_image_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(self.last_image_msg)

    def decide_trolley_type(self, detected_classes):
        best_trash_confidence = 0.0
        best_laundry_confidence = 0.0

        for detection in detected_classes:
            class_name = detection["class_name"].lower()
            confidence = detection["confidence"]

            if class_name in ["trash_trolley", "trash", "waste_trolley", "waste"]:
                best_trash_confidence = max(best_trash_confidence, confidence)

            if class_name in ["laundry_trolley", "laundry", "linen_trolley", "linen"]:
                best_laundry_confidence = max(best_laundry_confidence, confidence)

            if class_name in ["empty_trolley", "empty"]:
                self.get_logger().warn(
                    f"Empty trolley detected with confidence {confidence:.2f}; "
                    "simulation mission will treat it as unknown_trolley."
                )

        if (
            best_trash_confidence < self.minimum_confidence
            and best_laundry_confidence < self.minimum_confidence
        ):
            return None

        if best_trash_confidence > best_laundry_confidence:
            return "trash_trolley"

        return "laundry_trolley"

    def publish_unknown(self):
        command_msg = String()
        command_msg.data = "unknown_trolley"
        self.command_pub.publish(command_msg)
        self.get_logger().warn("Published command: unknown_trolley")


def main(args=None):
    rclpy.init(args=args)
    node = TrolleyImageDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Trolley image detector stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
