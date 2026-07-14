#!/usr/bin/env python3

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


CAMERAS = {
    "left": {
        "device": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.4.1:1.0-video-index0",
        "topic": "/camera_left",
        "frame_id": "camera_left_optical_link",
    },
    "rear": {
        "device": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.4.3:1.0-video-index0",
        "topic": "/camera_rear",
        "frame_id": "camera_rear_optical_link",
    },
    "front": {
        "device": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.4.4:1.0-video-index0",
        "topic": "/camera_front",
        "frame_id": "camera_front_optical_link",
    },
    "right": {
        "device": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.4.2:1.0-video-index0",
        "topic": "/camera_right",
        "frame_id": "camera_right_optical_link",
    },
}


CAMERA_PAIRS = {
    "front_rear": ["front", "rear"],
    "left_right": ["left", "right"],
    "front": ["front"],
    "rear": ["rear"],
    "left": ["left"],
    "right": ["right"],
}


class PhysicalFourCameraNode(Node):
    def __init__(self):
        super().__init__("physical_camera_pair_node")

        self.declare_parameter("camera_pair", "front_rear")
        self.declare_parameter("camera_pair_command_topic", "/physical_camera_pair_command")
        self.declare_parameter("camera_pair_status_topic", "/physical_camera_pair_status")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 10)

        self.camera_pair = (
            self.get_parameter("camera_pair").get_parameter_value().string_value
        )
        self.camera_pair_command_topic = (
            self.get_parameter("camera_pair_command_topic").get_parameter_value().string_value
        )
        self.camera_pair_status_topic = (
            self.get_parameter("camera_pair_status_topic").get_parameter_value().string_value
        )
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value

        if self.camera_pair not in CAMERA_PAIRS:
            raise ValueError(
                f"Invalid camera_pair '{self.camera_pair}'. "
                f"Use one of: {list(CAMERA_PAIRS.keys())}"
            )

        self.enabled_camera_names = CAMERA_PAIRS[self.camera_pair]

        self.bridge = CvBridge()

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cameras = {}
        self.status_pub = self.create_publisher(
            String,
            self.camera_pair_status_topic,
            self.status_qos,
        )

        self.get_logger().info(f"Starting camera pair: {self.camera_pair}")
        self.get_logger().info(f"Enabled cameras: {self.enabled_camera_names}")

        self.open_camera_pair(self.camera_pair)
        self.publish_status()
        self.create_subscription(
            String,
            self.camera_pair_command_topic,
            self.camera_pair_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / self.fps, self.publish_all)

    def camera_pair_callback(self, msg: String):
        requested_pair = msg.data.strip()
        if requested_pair not in CAMERA_PAIRS:
            self.get_logger().warn(
                f"Ignoring invalid camera pair command '{requested_pair}'. "
                f"Use one of: {list(CAMERA_PAIRS.keys())}"
            )
            return

        if requested_pair == self.camera_pair:
            self.publish_status()
            return

        self.get_logger().info(f"Switching camera pair: {self.camera_pair} -> {requested_pair}")
        self.close_cameras()
        self.camera_pair = requested_pair
        self.enabled_camera_names = CAMERA_PAIRS[self.camera_pair]
        self.open_camera_pair(self.camera_pair)
        self.publish_status()

    def publish_status(self):
        msg = String()
        msg.data = self.camera_pair
        self.status_pub.publish(msg)

    def open_camera_pair(self, camera_pair: str):
        for name in CAMERA_PAIRS[camera_pair]:
            self.open_camera(name)

    def create_capture(self, name: str):
        cfg = CAMERAS[name]
        cap = cv2.VideoCapture(cfg["device"], cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error(f"Could not open {name}: {cfg['device']}")
            return None

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def open_camera(self, name: str):
        cfg = CAMERAS[name]
        cap = self.create_capture(name)
        if cap is None:
            return

        image_pub = self.create_publisher(
            Image,
            f"{cfg['topic']}/image_raw",
            self.qos,
        )

        info_pub = self.create_publisher(
            CameraInfo,
            f"{cfg['topic']}/camera_info",
            self.qos,
        )

        self.cameras[name] = {
            "cap": cap,
            "image_pub": image_pub,
            "info_pub": info_pub,
            "frame_id": cfg["frame_id"],
            "topic": cfg["topic"],
            "no_frame_count": 0,
        }

        self.get_logger().info(
            f"{name} camera opened: {cfg['device']} -> {cfg['topic']}/image_raw"
        )

    def reopen_camera(self, name: str):
        cam = self.cameras.get(name)
        if cam is None:
            self.open_camera(name)
            return

        old_cap = cam["cap"]
        if old_cap.isOpened():
            old_cap.release()

        new_cap = self.create_capture(name)
        if new_cap is None:
            cam["no_frame_count"] = 0
            return

        cam["cap"] = new_cap
        cam["no_frame_count"] = 0
        self.get_logger().info(f"Reopened {name} camera after missing frames.")

    def make_camera_info(self, frame_id: str, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = self.width
        msg.height = self.height

        horizontal_fov = 1.422
        fx = self.width / (2.0 * math.tan(horizontal_fov / 2.0))
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]

        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        msg.distortion_model = "plumb_bob"

        return msg

    def publish_all(self):
        for name, cam in self.cameras.items():
            ok, frame = cam["cap"].read()

            if not ok or frame is None:
                cam["no_frame_count"] = cam.get("no_frame_count", 0) + 1
                reopen_after = max(3, int(self.fps))
                if cam["no_frame_count"] == 1 or cam["no_frame_count"] % reopen_after == 0:
                    self.get_logger().warn(
                        f"No frame from {name} ({cam['no_frame_count']} consecutive)"
                    )
                if cam["no_frame_count"] >= reopen_after:
                    self.get_logger().warn(f"Reopening {name} camera after missing frames.")
                    self.reopen_camera(name)
                continue
            cam["no_frame_count"] = 0

            stamp = self.get_clock().now().to_msg()

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = cam["frame_id"]

            info_msg = self.make_camera_info(cam["frame_id"], stamp)

            cam["image_pub"].publish(image_msg)
            cam["info_pub"].publish(info_msg)

    def destroy_node(self):
        self.close_cameras()
        super().destroy_node()

    def close_cameras(self):
        for cam in self.cameras.values():
            cap = cam["cap"]
            if cap.isOpened():
                cap.release()
        self.cameras.clear()


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalFourCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
