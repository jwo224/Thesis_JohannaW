#!/usr/bin/env python3

import csv
import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from object_detection.msg import Yolov8Inference
except ImportError:
    Yolov8Inference = None


EVENT_FIELDS = [
    "wall_time",
    "ros_time_sec",
    "run_id",
    "elapsed_sec",
    "event_type",
    "source",
    "label",
    "data",
]

SUMMARY_FIELDS = [
    "wall_time",
    "run_id",
    "outcome",
    "total_sec",
    "time_to_approach_goal_sent",
    "time_to_side_align_complete",
    "time_to_trolley_type_checked",
    "time_to_under_trolley",
    "time_to_fine_first_aruko",
    "time_to_fine_align_complete",
    "time_to_dropoff_goal_sent",
    "time_to_drive_out_done",
    "commands",
    "manual_marks",
    "dropzone_status_events",
    "physical_status_events",
    "yolo_detection_events",
    "last_trolley_type",
    "last_dropzone_status",
    "last_physical_status",
    "events_csv",
]


class MissionRunLogger(Node):
    def __init__(self):
        super().__init__("mission_run_logger")

        self.declare_parameter("output_dir", "~/gr4_ws/mission_run_logs")
        self.declare_parameter("command_topic", "/trolley_command")
        self.declare_parameter("dropzone_status_topic", "/dropzone_mission_status")
        self.declare_parameter("physical_status_topic", "/physical_docking_status")
        self.declare_parameter("logger_command_topic", "/mission_run_logger_command")
        self.declare_parameter("logger_status_topic", "/mission_run_logger_status")
        self.declare_parameter("yolo_topic", "/Yolov8_Inference")
        self.declare_parameter("auto_start_command", "trolley_ready")
        self.declare_parameter("auto_end_on_drive_out_done", True)
        self.declare_parameter("auto_end_on_fine_align_complete", False)

        output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.events_csv_path = output_dir / f"mission_run_events_{timestamp}.csv"
        self.summary_csv_path = output_dir / f"mission_run_summary_{timestamp}.csv"

        self.write_csv_header(self.events_csv_path, EVENT_FIELDS)
        self.write_csv_header(self.summary_csv_path, SUMMARY_FIELDS)

        self.active = False
        self.run_id = ""
        self.start_monotonic = None
        self.start_ros_sec = None
        self.milestones = {}
        self.counts = {}
        self.commands = []
        self.manual_marks = 0
        self.last_dropzone_status = ""
        self.last_physical_status = ""
        self.last_yolo_classes = []
        self.last_trolley_type = ""

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("logger_status_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("logger_command_topic").value),
            self.logger_command_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self.trolley_command_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("dropzone_status_topic").value),
            self.dropzone_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("physical_status_topic").value),
            self.physical_status_callback,
            10,
        )
        if Yolov8Inference is not None:
            self.create_subscription(
                Yolov8Inference,
                str(self.get_parameter("yolo_topic").value),
                self.yolo_callback,
                10,
            )
        else:
            self.get_logger().warn("object_detection messages not available; YOLO events disabled.")

        self.publish_logger_status(
            "mission_run_logger_ready "
            f"events_csv={self.events_csv_path} summary_csv={self.summary_csv_path}"
        )

    def write_csv_header(self, path, fields):
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()

    def now_ros_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def now_wall(self):
        return datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")

    def elapsed_sec(self):
        if self.start_monotonic is None:
            return ""
        return f"{time.monotonic() - self.start_monotonic:.3f}"

    def normalize_command(self, text):
        return str(text).strip().lower().replace(" ", "_").replace("-", "_")

    def start_run(self, reason):
        if self.active:
            self.end_run(f"auto_closed_before_{reason}")

        self.active = True
        self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.start_monotonic = time.monotonic()
        self.start_ros_sec = self.now_ros_sec()
        self.milestones = {}
        self.counts = {
            "command": 0,
            "dropzone_status": 0,
            "physical_status": 0,
            "yolo_detection": 0,
        }
        self.commands = []
        self.manual_marks = 0
        self.last_dropzone_status = ""
        self.last_physical_status = ""
        self.last_yolo_classes = []
        self.last_trolley_type = ""
        self.write_event("run_start", "logger", reason, "")
        self.publish_logger_status(f"run_started id={self.run_id} reason={reason}")

    def end_run(self, outcome):
        if not self.active:
            self.publish_logger_status("no_active_run_to_end")
            return

        self.write_event("run_end", "logger", outcome, "")
        total_sec = time.monotonic() - self.start_monotonic
        row = {
            "wall_time": self.now_wall(),
            "run_id": self.run_id,
            "outcome": outcome,
            "total_sec": f"{total_sec:.3f}",
            "commands": "|".join(self.commands),
            "manual_marks": self.manual_marks,
            "dropzone_status_events": self.counts.get("dropzone_status", 0),
            "physical_status_events": self.counts.get("physical_status", 0),
            "yolo_detection_events": self.counts.get("yolo_detection", 0),
            "last_trolley_type": self.last_trolley_type,
            "last_dropzone_status": self.last_dropzone_status,
            "last_physical_status": self.last_physical_status,
            "events_csv": str(self.events_csv_path),
        }
        for milestone in (
            "approach_goal_sent",
            "side_align_complete",
            "trolley_type_checked",
            "under_trolley",
            "fine_first_aruko",
            "fine_align_complete",
            "dropoff_goal_sent",
            "drive_out_done",
        ):
            row[f"time_to_{milestone}"] = self.milestones.get(milestone, "")

        with open(self.summary_csv_path, "a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
            writer.writerow(row)

        finished_id = self.run_id
        self.active = False
        self.run_id = ""
        self.start_monotonic = None
        self.start_ros_sec = None
        self.publish_logger_status(
            f"run_ended id={finished_id} outcome={outcome} summary_csv={self.summary_csv_path}"
        )

    def write_event(self, event_type, source, label, data):
        if not self.active and event_type not in ("logger_status",):
            return

        if event_type in self.counts:
            self.counts[event_type] += 1

        row = {
            "wall_time": self.now_wall(),
            "ros_time_sec": f"{self.now_ros_sec():.6f}",
            "run_id": self.run_id,
            "elapsed_sec": self.elapsed_sec(),
            "event_type": event_type,
            "source": source,
            "label": label,
            "data": data,
        }
        with open(self.events_csv_path, "a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=EVENT_FIELDS)
            writer.writerow(row)

    def set_milestone(self, name):
        if not self.active or name in self.milestones:
            return
        self.milestones[name] = self.elapsed_sec()

    def logger_command_callback(self, msg):
        text = str(msg.data).strip()
        command = self.normalize_command(text)

        if command == "new_run":
            self.start_run("manual_new_run")
            return
        if command in ("end_run", "stop_run"):
            self.end_run("manual_end")
            return
        if command == "abort_run":
            self.end_run("manual_abort")
            return
        if command == "status":
            active_text = "active" if self.active else "idle"
            self.publish_logger_status(
                f"logger_status state={active_text} run_id={self.run_id or '-'} "
                f"elapsed={self.elapsed_sec() or '-'} events_csv={self.events_csv_path} "
                f"summary_csv={self.summary_csv_path}"
            )
            return
        if command.startswith("mark_") or command.startswith("mark "):
            label = text.split(maxsplit=1)[1] if " " in text else text[len("mark_"):]
            self.add_mark(label)
            return

        self.publish_logger_status(
            "unknown_logger_command. Use new_run, mark <label>, end_run, abort_run, status."
        )

    def add_mark(self, label):
        if not self.active:
            self.start_run("manual_mark_without_active_run")
        self.manual_marks += 1
        self.write_event("manual_mark", "operator", label, "")
        self.publish_logger_status(f"mark_added run_id={self.run_id} label={label}")

    def trolley_command_callback(self, msg):
        command = self.normalize_command(msg.data)
        if command == self.normalize_command(self.get_parameter("auto_start_command").value):
            self.start_run(f"command_{command}")
        elif not self.active:
            return

        self.commands.append(command)
        self.write_event("command", "trolley_command", command, str(msg.data))

    def dropzone_status_callback(self, msg):
        text = str(msg.data).strip()
        if not self.active:
            return
        if text == self.last_dropzone_status:
            return

        self.last_dropzone_status = text
        milestone = self.dropzone_milestone(text)
        if milestone:
            self.set_milestone(milestone)
        trolley_type = self.extract_trolley_type(text)
        if trolley_type:
            self.last_trolley_type = trolley_type

        self.write_event("dropzone_status", "dropzone", milestone or "status", text)
        if (
            text.startswith("drive_out_done")
            and bool(self.get_parameter("auto_end_on_drive_out_done").value)
        ):
            self.end_run("drive_out_done")
        elif (
            text.startswith("fine_align_complete")
            and bool(self.get_parameter("auto_end_on_fine_align_complete").value)
        ):
            self.end_run("fine_align_complete")

    def physical_status_callback(self, msg):
        text = str(msg.data).strip()
        if not self.active:
            return
        if text == self.last_physical_status:
            return

        self.last_physical_status = text
        if text.startswith("fine_debug"):
            return

        milestone = self.physical_milestone(text)
        if milestone:
            self.set_milestone(milestone)
        self.write_event("physical_status", "physical_alignment", milestone or "status", text)

    def yolo_callback(self, msg):
        if not self.active:
            return

        classes = [
            str(result.class_name).strip()
            for result in msg.yolov8_inference
            if str(result.class_name).strip()
        ]
        if not classes or classes == self.last_yolo_classes:
            return

        self.last_yolo_classes = classes
        trolley_type = self.extract_trolley_type(" ".join(classes))
        if trolley_type:
            self.last_trolley_type = trolley_type
        self.write_event(
            "yolo_detection",
            "yolo",
            trolley_type or "unknown",
            json.dumps({"classes": classes}),
        )

    def dropzone_milestone(self, text):
        if text.startswith("approach_goal_sent"):
            return "approach_goal_sent"
        if text.startswith("side_align_complete"):
            return "side_align_complete"
        if text.startswith("trolley_type_checked"):
            return "trolley_type_checked"
        if text.startswith("under_trolley"):
            return "under_trolley"
        if text.startswith("fine_align_first_aruko"):
            return "fine_first_aruko"
        if text.startswith("fine_align_complete"):
            return "fine_align_complete"
        if text.startswith("dropoff_goal_sent"):
            return "dropoff_goal_sent"
        if text.startswith("drive_out_done"):
            return "drive_out_done"
        return ""

    def physical_milestone(self, text):
        if text.startswith("side_align_left_done") or text.startswith("side_align_right_done"):
            return "side_align_complete"
        if text.startswith("drive_straight_under_done"):
            return "under_trolley"
        if text.startswith("fine_first_estimate"):
            return "fine_first_aruko"
        if text.startswith("fine_align_under_done"):
            return "fine_align_complete"
        return ""

    def extract_trolley_type(self, text):
        lowered = text.lower()
        if "laundry_trolley" in lowered:
            return "laundry_trolley"
        if "trash_trolley" in lowered:
            return "trash_trolley"
        if "empty_trolley" in lowered:
            return "empty_trolley"
        return ""

    def publish_logger_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.active:
            node.end_run("logger_shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
