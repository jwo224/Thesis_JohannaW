#!/usr/bin/env python3

import csv
import math
import threading
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List

import rclpy
from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401


RAW_FIELDS = [
    "camera",
    "sample_index",
    "ros_time_sec",
    "camera_frame_id",
    "base_frame_id",
    "marker_id",
    "camera_x",
    "camera_y",
    "camera_z",
    "camera_qx",
    "camera_qy",
    "camera_qz",
    "camera_qw",
    "base_x",
    "base_y",
    "base_z",
    "base_qx",
    "base_qy",
    "base_qz",
    "base_qw",
]


SUMMARY_FIELDS = [
    "camera",
    "marker_id",
    "camera_frame_id",
    "base_frame_id",
    "count",
    "mean_camera_x",
    "mean_camera_y",
    "mean_camera_z",
    "std_camera_x",
    "std_camera_y",
    "std_camera_z",
    "mean_base_x",
    "mean_base_y",
    "mean_base_z",
    "std_base_x",
    "std_base_y",
    "std_base_z",
    "min_base_x",
    "max_base_x",
    "min_base_y",
    "max_base_y",
    "min_base_z",
    "max_base_z",
]


class TrolleyReferenceLogger(Node):
    def __init__(self):
        super().__init__("trolley_reference_logger")

        self.declare_parameter("samples", 50)
        self.declare_parameter("output_dir", "/home/rocket/gr4_ws/trolley_reference_logs")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("require_front", False)
        self.declare_parameter("require_rear", False)
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("min_total_rows", 1)
        self.declare_parameter("front_aruco_topic", "/front/aruco_markers")
        self.declare_parameter("rear_aruco_topic", "/rear/aruco_markers")

        self.target_samples = int(self.get_parameter("samples").value)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.require_front = bool(self.get_parameter("require_front").value)
        self.require_rear = bool(self.get_parameter("require_rear").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.min_total_rows = int(self.get_parameter("min_total_rows").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.raw_csv_path = self.output_dir / f"trolley_reference_raw_{timestamp}.csv"
        self.summary_csv_path = self.output_dir / f"trolley_reference_summary_{timestamp}.csv"

        self.rows: List[Dict[str, float]] = []
        self.message_counts = Counter()
        self.empty_message_counts = Counter()
        self.sample_counts = Counter()
        self.marker_counts = Counter()
        self.tf_failures = Counter()
        self.topic_seen = Counter()
        self.finished = False
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            ArucoMarkers,
            str(self.get_parameter("front_aruco_topic").value),
            lambda msg: self.marker_callback(msg, "front"),
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            str(self.get_parameter("rear_aruco_topic").value),
            lambda msg: self.marker_callback(msg, "rear"),
            10,
        )

        self.start_time = self.get_clock().now()
        self.create_timer(0.5, self.timeout_check)

        self.get_logger().info("Trolley reference logger started.")
        self.get_logger().info("Place robot in the desired final docking position.")
        self.get_logger().info(
            f"Target valid messages: samples={self.target_samples}, "
            f"require_front={self.require_front}, require_rear={self.require_rear}, "
            f"timeout_sec={self.timeout_sec:.1f}, min_total_rows={self.min_total_rows}"
        )
        self.get_logger().info(f"Base frame: {self.base_frame}")
        self.get_logger().info(f"Raw CSV: {self.raw_csv_path}")
        self.get_logger().info(f"Summary CSV: {self.summary_csv_path}")

    def marker_callback(self, msg: ArucoMarkers, camera_name: str):
        with self.lock:
            if self.finished:
                return
            if self.sample_counts[camera_name] >= self.target_samples:
                return
            self.topic_seen[camera_name] += 1
            self.message_counts[camera_name] += 1
            if not msg.marker_ids:
                self.empty_message_counts[camera_name] += 1
                return

        valid_rows = []
        now_msg = Time().to_msg()

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)

            pose_cam = PoseStamped()
            pose_cam.header = msg.header
            pose_cam.header.stamp = now_msg
            pose_cam.pose = pose

            try:
                pose_base = self.tf_buffer.transform(
                    pose_cam,
                    self.base_frame,
                    timeout=Duration(seconds=0.20),
                )
            except TransformException as exc:
                with self.lock:
                    self.tf_failures[(camera_name, marker_id)] += 1
                self.get_logger().warn(
                    f"TF failed for {camera_name} marker {marker_id}: "
                    f"{msg.header.frame_id} -> {self.base_frame}: {exc}"
                )
                continue

            ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            valid_rows.append({
                "camera": camera_name,
                "ros_time_sec": ros_time_sec,
                "camera_frame_id": msg.header.frame_id,
                "base_frame_id": self.base_frame,
                "marker_id": marker_id,
                "camera_x": pose.position.x,
                "camera_y": pose.position.y,
                "camera_z": pose.position.z,
                "camera_qx": pose.orientation.x,
                "camera_qy": pose.orientation.y,
                "camera_qz": pose.orientation.z,
                "camera_qw": pose.orientation.w,
                "base_x": pose_base.pose.position.x,
                "base_y": pose_base.pose.position.y,
                "base_z": pose_base.pose.position.z,
                "base_qx": pose_base.pose.orientation.x,
                "base_qy": pose_base.pose.orientation.y,
                "base_qz": pose_base.pose.orientation.z,
                "base_qw": pose_base.pose.orientation.w,
            })

        if not valid_rows:
            return

        with self.lock:
            if self.finished:
                return
            self.sample_counts[camera_name] += 1
            sample_index = self.sample_counts[camera_name]
            for row in valid_rows:
                row["sample_index"] = sample_index
                self.rows.append(row)
                self.marker_counts[(camera_name, row["marker_id"])] += 1

        self.get_logger().info(
            f"Collected {camera_name} sample {sample_index}/{self.target_samples} "
            f"ids={[row['marker_id'] for row in valid_rows]}"
        )

        if self.collection_complete():
            self.finish("sample target reached")

    def collection_complete(self) -> bool:
        with self.lock:
            front_done = self.sample_counts["front"] >= self.target_samples
            rear_done = self.sample_counts["rear"] >= self.target_samples

        if self.require_front and not front_done:
            return False
        if self.require_rear and not rear_done:
            return False
        if not self.require_front and not self.require_rear:
            return front_done or rear_done
        return True

    def timeout_check(self):
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.timeout_sec:
            self.finish(f"timeout after {elapsed:.1f}s")

    def finish(self, reason: str):
        with self.lock:
            if self.finished:
                return
            self.finished = True

        if len(self.rows) >= self.min_total_rows:
            self.write_raw_csv()
            self.write_summary_csv()
            self.get_logger().info(f"Saved partial/complete results: {reason}")
            self.get_logger().info(f"Raw CSV: {self.raw_csv_path}")
            self.get_logger().info(f"Summary CSV: {self.summary_csv_path}")
        else:
            self.get_logger().error(
                f"No usable CSV saved: collected {len(self.rows)} rows, "
                f"min_total_rows={self.min_total_rows}. Reason: {self.no_rows_reason()}"
            )

        self.log_counts()
        if rclpy.ok():
            rclpy.shutdown()

    def no_rows_reason(self) -> str:
        if not self.topic_seen:
            return "no marker topic messages were received"
        if sum(self.message_counts.values()) == sum(self.empty_message_counts.values()):
            return "marker topics were received but contained no markers"
        if self.tf_failures:
            return "markers were received but TF to base_link failed"
        return "markers were received but no valid rows passed collection filters"

    def log_counts(self):
        self.get_logger().info(
            "Message counts: "
            f"front={self.message_counts['front']} rear={self.message_counts['rear']} | "
            f"valid samples front={self.sample_counts['front']} rear={self.sample_counts['rear']}"
        )
        for (camera, marker_id), count in sorted(self.marker_counts.items()):
            self.get_logger().info(f"Marker count {camera} id={marker_id}: {count}")
        for (camera, marker_id), count in sorted(self.tf_failures.items()):
            self.get_logger().warn(f"TF failures {camera} id={marker_id}: {count}")

    def write_raw_csv(self):
        with open(self.raw_csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=RAW_FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)

    def write_summary_csv(self):
        grouped = defaultdict(list)
        for row in self.rows:
            grouped[
                (
                    row["camera"],
                    row["marker_id"],
                    row["camera_frame_id"],
                    row["base_frame_id"],
                )
            ].append(row)

        summary_rows = []
        for (camera, marker_id, camera_frame_id, base_frame_id), rows in grouped.items():
            camera_xs = [r["camera_x"] for r in rows]
            camera_ys = [r["camera_y"] for r in rows]
            camera_zs = [r["camera_z"] for r in rows]
            base_xs = [r["base_x"] for r in rows]
            base_ys = [r["base_y"] for r in rows]
            base_zs = [r["base_z"] for r in rows]

            summary_rows.append({
                "camera": camera,
                "marker_id": marker_id,
                "camera_frame_id": camera_frame_id,
                "base_frame_id": base_frame_id,
                "count": len(rows),
                "mean_camera_x": mean(camera_xs),
                "mean_camera_y": mean(camera_ys),
                "mean_camera_z": mean(camera_zs),
                "std_camera_x": std(camera_xs),
                "std_camera_y": std(camera_ys),
                "std_camera_z": std(camera_zs),
                "mean_base_x": mean(base_xs),
                "mean_base_y": mean(base_ys),
                "mean_base_z": mean(base_zs),
                "std_base_x": std(base_xs),
                "std_base_y": std(base_ys),
                "std_base_z": std(base_zs),
                "min_base_x": min(base_xs),
                "max_base_x": max(base_xs),
                "min_base_y": min(base_ys),
                "max_base_y": max(base_ys),
                "min_base_z": min(base_zs),
                "max_base_z": max(base_zs),
            })

        with open(self.summary_csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_FIELDS)
            writer.writeheader()
            writer.writerows(summary_rows)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def std(values):
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / (len(values) - 1))


def main(args=None):
    rclpy.init(args=args)
    node = TrolleyReferenceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted. Saving partial results if available.")
        node.finish("Ctrl+C")
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
