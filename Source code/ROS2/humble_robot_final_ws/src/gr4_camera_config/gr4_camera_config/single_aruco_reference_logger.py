#!/usr/bin/env python3

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from aruco_interfaces.msg import ArucoMarkers


class SingleArucoReferenceLogger(Node):
    def __init__(self):
        super().__init__("single_aruco_reference_logger")

        self.declare_parameter("camera", "left")
        self.declare_parameter("topic", "")
        self.declare_parameter("samples", 100)
        self.declare_parameter("output_dir", "/home/rocket/gr4_ws/aruco_reference_logs")

        # For side calibration, ignore IDs 0-15.
        # Set this to 0 if you ever want to record all IDs.
        self.declare_parameter("ignore_ids_below", 16)

        self.camera = self.get_parameter("camera").get_parameter_value().string_value
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.target_samples = self.get_parameter("samples").get_parameter_value().integer_value
        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self.ignore_ids_below = self.get_parameter(
            "ignore_ids_below"
        ).get_parameter_value().integer_value

        if not self.topic:
            self.topic = f"/{self.camera}/aruco_markers"

        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.raw_csv_path = self.output_dir / f"{self.camera}_aruco_reference_raw_{timestamp}.csv"
        self.summary_csv_path = self.output_dir / f"{self.camera}_aruco_reference_summary_{timestamp}.csv"

        self.rows = []
        self.sample_count = 0

        self.sub = self.create_subscription(
            ArucoMarkers,
            self.topic,
            self.marker_callback,
            10,
        )

        self.get_logger().info("Single ArUco reference logger started.")
        self.get_logger().info(f"Camera: {self.camera}")
        self.get_logger().info(f"Topic: {self.topic}")
        self.get_logger().info(f"Target samples: {self.target_samples}")
        self.get_logger().info(f"Ignoring marker IDs below: {self.ignore_ids_below}")
        self.get_logger().info(f"Raw CSV: {self.raw_csv_path}")
        self.get_logger().info(f"Summary CSV: {self.summary_csv_path}")

    def marker_callback(self, msg: ArucoMarkers):
        if self.sample_count >= self.target_samples:
            return

        valid_pairs = []

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)

            if marker_id < self.ignore_ids_below:
                continue

            valid_pairs.append((marker_id, pose))

        if len(valid_pairs) == 0:
            visible_ids = [int(x) for x in msg.marker_ids]
            self.get_logger().warn(
                f"No valid markers in this sample. "
                f"visible_ids={visible_ids}, ignoring IDs below {self.ignore_ids_below}"
            )
            return

        self.sample_count += 1
        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        valid_ids = []

        for marker_id, pose in valid_pairs:
            valid_ids.append(marker_id)

            self.rows.append({
                "camera": self.camera,
                "sample_index": self.sample_count,
                "ros_time_sec": ros_time_sec,
                "frame_id": msg.header.frame_id,
                "marker_id": marker_id,
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w,
            })

        self.get_logger().info(
            f"Collected {self.camera} sample {self.sample_count}/{self.target_samples} "
            f"with valid markers {valid_ids}"
        )

        if self.sample_count >= self.target_samples:
            self.finish()

    def finish(self):
        self.write_raw_csv()
        self.write_summary_csv()

        self.get_logger().info("Finished collecting ArUco reference measurements.")
        self.get_logger().info(f"Saved raw CSV: {self.raw_csv_path}")
        self.get_logger().info(f"Saved summary CSV: {self.summary_csv_path}")

        rclpy.shutdown()

    def write_raw_csv(self):
        fieldnames = [
            "camera",
            "sample_index",
            "ros_time_sec",
            "frame_id",
            "marker_id",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
        ]

        with open(self.raw_csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

    def write_summary_csv(self):
        grouped = {}

        for row in self.rows:
            key = (row["camera"], row["marker_id"], row["frame_id"])
            grouped.setdefault(key, []).append(row)

        summary_rows = []

        for (camera, marker_id, frame_id), rows in grouped.items():
            xs = [r["x"] for r in rows]
            ys = [r["y"] for r in rows]
            zs = [r["z"] for r in rows]
            qxs = [r["qx"] for r in rows]
            qys = [r["qy"] for r in rows]
            qzs = [r["qz"] for r in rows]
            qws = [r["qw"] for r in rows]

            summary_rows.append({
                "camera": camera,
                "marker_id": marker_id,
                "frame_id": frame_id,
                "count": len(rows),

                "mean_x": mean(xs),
                "mean_y": mean(ys),
                "mean_z": mean(zs),

                "std_x": std(xs),
                "std_y": std(ys),
                "std_z": std(zs),

                "mean_qx": mean(qxs),
                "mean_qy": mean(qys),
                "mean_qz": mean(qzs),
                "mean_qw": mean(qws),

                "min_x": min(xs),
                "max_x": max(xs),
                "min_y": min(ys),
                "max_y": max(ys),
                "min_z": min(zs),
                "max_z": max(zs),
            })

        fieldnames = [
            "camera",
            "marker_id",
            "frame_id",
            "count",

            "mean_x",
            "mean_y",
            "mean_z",

            "std_x",
            "std_y",
            "std_z",

            "mean_qx",
            "mean_qy",
            "mean_qz",
            "mean_qw",

            "min_x",
            "max_x",
            "min_y",
            "max_y",
            "min_z",
            "max_z",
        ]

        with open(self.summary_csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(summary_rows)


def mean(values):
    if not values:
        return 0.0
    return sum(values) / len(values)


def std(values):
    if len(values) < 2:
        return 0.0

    m = mean(values)
    variance = sum((v - m) ** 2 for v in values) / (len(values) - 1)
    return math.sqrt(variance)


def main(args=None):
    rclpy.init(args=args)
    node = SingleArucoReferenceLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted. Saving partial results.")
        node.finish()


if __name__ == "__main__":
    main()