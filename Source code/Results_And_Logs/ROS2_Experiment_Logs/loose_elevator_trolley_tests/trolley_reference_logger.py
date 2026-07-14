#!/usr/bin/env python3

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from aruco_interfaces.msg import ArucoMarkers


class TrolleyReferenceLogger(Node):
    def __init__(self):
        super().__init__("trolley_reference_logger")

        self.declare_parameter("samples", 50)
        self.declare_parameter("output_dir", "/home/rocket/gr4_ws/trolley_reference_logs")

        self.target_samples = (
            self.get_parameter("samples").get_parameter_value().integer_value
        )
        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.raw_csv_path = self.output_dir / f"trolley_reference_raw_{timestamp}.csv"
        self.summary_csv_path = self.output_dir / f"trolley_reference_summary_{timestamp}.csv"

        self.rows = []

        self.front_count = 0
        self.rear_count = 0

        self.front_sub = self.create_subscription(
            ArucoMarkers,
            "/front/aruco_markers",
            lambda msg: self.marker_callback(msg, "front"),
            10,
        )

        self.rear_sub = self.create_subscription(
            ArucoMarkers,
            "/rear/aruco_markers",
            lambda msg: self.marker_callback(msg, "rear"),
            10,
        )

        self.get_logger().info("Trolley reference logger started.")
        self.get_logger().info("Place the robot exactly in the desired docking position.")
        self.get_logger().info(f"Collecting {self.target_samples} front and {self.target_samples} rear messages.")
        self.get_logger().info(f"Raw CSV: {self.raw_csv_path}")
        self.get_logger().info(f"Summary CSV: {self.summary_csv_path}")

    def marker_callback(self, msg: ArucoMarkers, camera_name: str):
        if camera_name == "front" and self.front_count >= self.target_samples:
            return

        if camera_name == "rear" and self.rear_count >= self.target_samples:
            return

        if len(msg.marker_ids) == 0:
            return

        if camera_name == "front":
            self.front_count += 1
            sample_index = self.front_count
        else:
            self.rear_count += 1
            sample_index = self.rear_count

        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            row = {
                "camera": camera_name,
                "sample_index": sample_index,
                "ros_time_sec": ros_time_sec,
                "frame_id": msg.header.frame_id,
                "marker_id": int(marker_id),
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "qx": pose.orientation.x,
                "qy": pose.orientation.y,
                "qz": pose.orientation.z,
                "qw": pose.orientation.w,
            }

            self.rows.append(row)

        self.get_logger().info(
            f"Collected {camera_name} sample {sample_index}/{self.target_samples} "
            f"with markers {list(msg.marker_ids)}"
        )

        if self.front_count >= self.target_samples and self.rear_count >= self.target_samples:
            self.finish()

    def finish(self):
        self.write_raw_csv()
        self.write_summary_csv()

        self.get_logger().info("Finished collecting trolley reference measurements.")
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
    node = TrolleyReferenceLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted. Saving partial results.")
        node.finish()


if __name__ == "__main__":
    main()

