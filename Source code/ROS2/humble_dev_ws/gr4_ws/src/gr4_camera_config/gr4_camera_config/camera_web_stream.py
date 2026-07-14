#!/usr/bin/env python3

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


JPEG_QUALITY = 80
STREAM_SLEEP = 0.03

app = Flask(__name__)


STREAMS = {
    "front_raw": {
        "topic": "/camera_front/image_raw",
        "encoding": "bgr8",
        "label": "FRONT RAW",
    },
    "front_aruco": {
        "topic": "/front/aruco_image",
        "encoding": "rgb8",
        "label": "FRONT ARUCO",
    },
    "rear_raw": {
        "topic": "/camera_rear/image_raw",
        "encoding": "bgr8",
        "label": "REAR RAW",
    },
    "rear_aruco": {
        "topic": "/rear/aruco_image",
        "encoding": "rgb8",
        "label": "REAR ARUCO",
    },
    "left_raw": {
        "topic": "/camera_left/image_raw",
        "encoding": "bgr8",
        "label": "LEFT RAW",
    },
    "left_aruco": {
        "topic": "/left/aruco_image",
        "encoding": "rgb8",
        "label": "LEFT ARUCO",
    },
    "right_raw": {
        "topic": "/camera_right/image_raw",
        "encoding": "bgr8",
        "label": "RIGHT RAW",
    },
    "right_aruco": {
        "topic": "/right/aruco_image",
        "encoding": "rgb8",
        "label": "RIGHT ARUCO",
    },
}


class CameraWebStreamNode(Node):
    def __init__(self):
        super().__init__("camera_web_stream_node")

        self.bridge = CvBridge()

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.frames = {}
        self.locks = {}

        for key, config in STREAMS.items():
            self.frames[key] = None
            self.locks[key] = threading.Lock()

            self.create_subscription(
                Image,
                config["topic"],
                lambda msg, stream_key=key, encoding=config["encoding"]: self.image_callback(
                    msg,
                    stream_key,
                    encoding,
                ),
                self.qos,
            )

            self.get_logger().info(f"{config['label']}: {config['topic']}")

        self.get_logger().info("Camera web stream node started.")

    def image_callback(self, msg: Image, key: str, encoding: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)

            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            with self.locks[key]:
                self.frames[key] = frame.copy()

        except Exception as e:
            self.get_logger().warn(f"Could not convert {key}: {e}")

    def get_frame(self, key: str):
        with self.locks[key]:
            frame = self.frames[key]

            if frame is None:
                return None

            return frame.copy()


ros_node = None


def make_placeholder(label: str):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    text = f"Waiting for {label}..."
    cv2.putText(
        frame,
        text,
        (40, 240),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return frame


def label_frame(frame, label: str):
    cv2.putText(
        frame,
        label,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        label,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return frame


def get_latest_frame(key: str):
    global ros_node

    label = STREAMS[key]["label"]

    if ros_node is None:
        return make_placeholder(label)

    frame = ros_node.get_frame(key)

    if frame is None:
        return make_placeholder(label)

    return label_frame(frame, label)


def generate_frames(key: str):
    while True:
        frame = get_latest_frame(key)

        success, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
        )

        if not success:
            time.sleep(STREAM_SLEEP)
            continue

        jpg = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )

        time.sleep(STREAM_SLEEP)


@app.route("/")
def index():
    return render_template_string(
        """
        <!doctype html>
        <html>
        <head>
            <title>Robot Camera + ArUco Stream</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    background: #111;
                    color: white;
                    text-align: center;
                    margin: 0;
                    padding: 20px;
                }

                .grid {
                    display: grid;
                    grid-template-columns: 1fr 1fr;
                    gap: 24px;
                    max-width: 1500px;
                    margin: auto;
                }

                .card {
                    background: #1c1c1c;
                    border-radius: 12px;
                    padding: 14px;
                    border: 1px solid #444;
                }

                img {
                    width: 100%;
                    height: auto;
                    max-width: 720px;
                    aspect-ratio: 16 / 9;
                    object-fit: contain;
                    background: black;
                    border: 2px solid #666;
                    border-radius: 10px;
                }

                a {
                    color: #8fd3ff;
                }

                .note {
                    color: #ccc;
                    margin-bottom: 20px;
                }
            </style>
        </head>
        <body>
            <h1>Robot Camera + ArUco Stream</h1>

            <p class="note">
                This page subscribes to ROS image topics only.
                It does not open /dev/video directly.
            </p>

            <div class="grid">
                {% for key, stream in streams.items() %}
                <div class="card">
                    <h2>{{ stream.label }}</h2>
                    <img src="/{{ key }}">
                    <p><a href="/{{ key }}">Open {{ stream.label }} only</a></p>
                    <p style="color:#aaa;">{{ stream.topic }}</p>
                </div>
                {% endfor %}
            </div>
        </body>
        </html>
        """,
        streams=STREAMS,
    )


def make_stream_route(key):
    def route():
        return Response(
            generate_frames(key),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    route.__name__ = f"route_{key}"
    return route


for stream_key in STREAMS.keys():
    app.add_url_rule(
        f"/{stream_key}",
        stream_key,
        make_stream_route(stream_key),
    )


def spin_ros():
    global ros_node
    rclpy.spin(ros_node)


def main():
    global ros_node

    rclpy.init()
    ros_node = CameraWebStreamNode()

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    print("Starting camera + ArUco web stream")
    print("Open this on your laptop browser:")
    print("http://192.168.8.221:8081")
    print()
    print("Direct links:")
    for key in STREAMS.keys():
        print(f"http://192.168.8.221:8081/{key}")

    try:
        app.run(host="0.0.0.0", port=8081, threaded=True)
    finally:
        try:
            ros_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()