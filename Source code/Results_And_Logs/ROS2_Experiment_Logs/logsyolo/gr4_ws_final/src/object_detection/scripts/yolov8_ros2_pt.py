#!/usr/bin/env python3

from pathlib import Path
from datetime import datetime
import csv

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from object_detection.msg import InferenceResult
from object_detection.msg import Yolov8Inference


CAMERA_TOPICS = {
    "front": "/camera_front/image_raw",
    "rear": "/camera_rear/image_raw",
    "left": "/camera_left/image_raw",
    "right": "/camera_right/image_raw",
}


class Yolov8CameraDetector(Node):
    def __init__(self):
        super().__init__("yolov8_camera_detector")

        self.bridge = CvBridge()

        self.declare_parameter("camera", "front")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("model", "yolov8s_no_augmentation.pt")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("enabled", True)
        self.declare_parameter("command_topic", "/yolov8_detector_command")
        self.declare_parameter("rotate_180", False)

        self.declare_parameter("save_images", True)
        self.declare_parameter("save_only_when_detected", True)
        self.declare_parameter("save_every_n_frames", 10)
        self.declare_parameter("output_dir", "/home/rocket/gr4_ws/yolo_detection_logs")

        self.camera = self.get_parameter("camera").value
        self.camera_topic = self.resolve_camera_topic()
        self.model_name = self.get_parameter("model").value
        self.confidence = float(self.get_parameter("confidence").value)
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.enabled = bool(self.get_parameter("enabled").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)

        self.save_images = bool(self.get_parameter("save_images").value)
        self.save_only_when_detected = bool(
            self.get_parameter("save_only_when_detected").value
        )
        self.save_every_n_frames = int(self.get_parameter("save_every_n_frames").value)
        self.output_dir = Path(self.get_parameter("output_dir").value)

        self.frame_counter = 0
        self.last_disabled_log_time = 0.0

        self.session_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"{self.camera}_{self.session_time}"
        self.images_dir = self.session_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.session_dir / "detections.csv"
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "wall_time",
                "ros_time_sec",
                "camera",
                "camera_topic",
                "frame_id",
                "image_file",
                "class_name",
                "confidence",
                "left",
                "top",
                "right",
                "bottom",
                "center_x",
                "center_y",
                "width",
                "height",
            ],
        )
        self.csv_writer.writeheader()
        self.csv_file.flush()

        self.model = self.load_model(self.model_name)

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription = None
        self.create_camera_subscription(self.camera_topic)

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            command_qos,
        )

        self.yolov8_pub = self.create_publisher(
            Yolov8Inference,
            "/Yolov8_Inference",
            10,
        )

        self.img_pub = self.create_publisher(
            Image,
            "/inference_result",
            10,
        )

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("YOLOv8 camera detector started.")
        self.get_logger().info(f"Camera selector: {self.camera}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Model: {self.model_name}")
        self.get_logger().info(f"Confidence: {self.confidence}")
        self.get_logger().info(f"Detector enabled: {self.enabled}")
        self.get_logger().info(f"Detector command topic: {self.command_topic}")
        self.get_logger().info(f"Rotate image 180deg before detection: {self.rotate_180}")
        self.get_logger().info(f"Saving images: {self.save_images}")
        self.get_logger().info(f"Save only when detected: {self.save_only_when_detected}")
        self.get_logger().info(f"Save every N frames: {self.save_every_n_frames}")
        self.get_logger().info(f"Output directory: {self.session_dir}")
        self.get_logger().info(f"CSV log: {self.csv_path}")

    def command_callback(self, msg: String):
        command = msg.data.strip().lower().replace(" ", "_")
        if command in ("enable", "enabled", "start", "on", "true", "detect"):
            if not self.enabled:
                self.get_logger().info("YOLO detector enabled by command.")
            self.enabled = True
            return
        if command in ("disable", "disabled", "stop", "off", "false", "idle"):
            if self.enabled:
                self.get_logger().info("YOLO detector disabled by command.")
            self.enabled = False
            return
        self.get_logger().warn(
            f"Unknown YOLO detector command '{msg.data}'. Use enable or disable."
        )

    def resolve_camera_topic(self):
        camera_topic_param = self.get_parameter("camera_topic").value

        if camera_topic_param:
            return camera_topic_param

        camera_param = self.get_parameter("camera").value

        if camera_param in CAMERA_TOPICS:
            return CAMERA_TOPICS[camera_param]

        if camera_param.startswith("/"):
            return camera_param

        self.get_logger().warn(
            f"Unknown camera '{camera_param}', falling back to front camera."
        )
        return CAMERA_TOPICS["front"]

    def load_model(self, model_name):
        package_share = Path(get_package_share_directory("object_detection"))

        possible_paths = [
            package_share / "models" / model_name,
            Path.home() / "gr4_ws" / "src" / "object_detection" / "scripts" / model_name,
            Path(model_name).expanduser(),
        ]

        for path in possible_paths:
            if path.exists():
                self.get_logger().info(f"Loading YOLO model from: {path}")
                return YOLO(str(path))

        raise FileNotFoundError(
            "Could not find YOLO model. Tried:\n"
            + "\n".join(str(path) for path in possible_paths)
        )

    def create_camera_subscription(self, topic):
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.camera_callback,
            self.qos,
        )

        self.get_logger().info(f"Subscribed to camera topic: {topic}")

    def parameter_callback(self, params):
        reload_model = False
        recreate_subscription = False

        for param in params:
            if param.name == "camera" and param.type_ == Parameter.Type.STRING:
                self.camera = param.value
                recreate_subscription = True

            elif param.name == "camera_topic" and param.type_ == Parameter.Type.STRING:
                recreate_subscription = True

            elif param.name == "model" and param.type_ == Parameter.Type.STRING:
                self.model_name = param.value
                reload_model = True

            elif param.name == "confidence":
                self.confidence = float(param.value)

            elif param.name == "publish_annotated_image":
                self.publish_annotated_image = bool(param.value)

            elif param.name == "enabled":
                self.enabled = bool(param.value)

            elif param.name == "rotate_180":
                self.rotate_180 = bool(param.value)

            elif param.name == "save_images":
                self.save_images = bool(param.value)

            elif param.name == "save_only_when_detected":
                self.save_only_when_detected = bool(param.value)

            elif param.name == "save_every_n_frames":
                self.save_every_n_frames = max(1, int(param.value))

        if reload_model:
            self.model = self.load_model(self.model_name)

        if recreate_subscription:
            self.camera_topic = self.resolve_camera_topic()
            self.create_camera_subscription(self.camera_topic)

        return SetParametersResult(successful=True)

    def camera_callback(self, msg):
        self.frame_counter += 1
        if not self.enabled:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_disabled_log_time > 10.0:
                self.last_disabled_log_time = now
                self.get_logger().info(
                    "YOLO detector is disabled; received camera frames but is not running inference."
                )
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Could not convert camera image: {e}")
            return

        if self.rotate_180:
            img = cv2.rotate(img, cv2.ROTATE_180)

        try:
            results = self.model(img, conf=self.confidence, verbose=False)
        except Exception as e:
            self.get_logger().warn(f"YOLO inference failed: {e}")
            return

        inference_msg = Yolov8Inference()
        inference_msg.header.stamp = msg.header.stamp
        inference_msg.header.frame_id = msg.header.frame_id

        detections = []

        for result in results:
            for box in result.boxes:
                b = box.xyxy[0].cpu().numpy()
                class_id = int(box.cls[0])
                conf = float(box.conf[0])

                left = int(b[0])
                top = int(b[1])
                right = int(b[2])
                bottom = int(b[3])

                class_name = self.model.names[class_id]

                inference_result = InferenceResult()
                inference_result.class_name = class_name
                inference_result.left = left
                inference_result.top = top
                inference_result.right = right
                inference_result.bottom = bottom

                inference_msg.yolov8_inference.append(inference_result)

                detections.append({
                    "class_name": class_name,
                    "confidence": conf,
                    "left": left,
                    "top": top,
                    "right": right,
                    "bottom": bottom,
                    "center_x": (left + right) / 2.0,
                    "center_y": (top + bottom) / 2.0,
                    "width": right - left,
                    "height": bottom - top,
                })

        self.yolov8_pub.publish(inference_msg)

        annotated_frame = results[0].plot()
        if (
            len(detections) == 0
            and self.save_only_when_detected
            and self.frame_counter % max(1, self.save_every_n_frames * 10) == 0
        ):
            self.get_logger().info(
                "YOLO processed frames but has no detections. "
                "No image/CSV row is saved because save_only_when_detected=True."
            )

        if self.publish_annotated_image:
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding="bgr8",
            )
            annotated_msg.header.stamp = msg.header.stamp
            annotated_msg.header.frame_id = msg.header.frame_id
            self.img_pub.publish(annotated_msg)

        self.save_detection_outputs(
            msg=msg,
            annotated_frame=annotated_frame,
            detections=detections,
        )

    def save_detection_outputs(self, msg, annotated_frame, detections):
        if not self.save_images:
            return

        if self.save_only_when_detected and len(detections) == 0:
            return

        if self.frame_counter % self.save_every_n_frames != 0:
            return

        wall_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        image_filename = (
            f"{self.camera}_frame_{self.frame_counter:06d}_{wall_time}.jpg"
        )
        image_path = self.images_dir / image_filename

        image_saved = cv2.imwrite(str(image_path), annotated_frame)
        if not image_saved:
            self.get_logger().warn(
                f"Failed to save detection image: {image_path}. "
                "Writing CSV row with an empty image_file field."
            )

        if len(detections) == 0:
            self.csv_writer.writerow({
                "wall_time": wall_time,
                "ros_time_sec": ros_time_sec,
                "camera": self.camera,
                "camera_topic": self.camera_topic,
                "frame_id": msg.header.frame_id,
                "image_file": str(image_path) if image_saved else "",
                "class_name": "",
                "confidence": "",
                "left": "",
                "top": "",
                "right": "",
                "bottom": "",
                "center_x": "",
                "center_y": "",
                "width": "",
                "height": "",
            })
        else:
            for det in detections:
                self.csv_writer.writerow({
                    "wall_time": wall_time,
                    "ros_time_sec": ros_time_sec,
                    "camera": self.camera,
                    "camera_topic": self.camera_topic,
                    "frame_id": msg.header.frame_id,
                    "image_file": str(image_path) if image_saved else "",
                    "class_name": det["class_name"],
                    "confidence": det["confidence"],
                    "left": det["left"],
                    "top": det["top"],
                    "right": det["right"],
                    "bottom": det["bottom"],
                    "center_x": det["center_x"],
                    "center_y": det["center_y"],
                    "width": det["width"],
                    "height": det["height"],
                })

        self.csv_file.flush()

        self.get_logger().info(
            f"Saved detection image: {image_path} "
            f"with {len(detections)} detections"
        )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Yolov8CameraDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
