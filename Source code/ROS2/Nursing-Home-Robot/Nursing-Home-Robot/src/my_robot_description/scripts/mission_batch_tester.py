#!/usr/bin/env python3

import csv
import math
import os
import queue
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from gazebo_msgs.srv import GetEntityState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_parallel_angle(angle):
    angle = normalize_angle(angle)
    if angle > math.pi / 2.0:
        angle -= math.pi
    elif angle < -math.pi / 2.0:
        angle += math.pi
    return angle


class MissionBatchTester(Node):
    def __init__(self):
        super().__init__("mission_batch_tester")

        self.declare_parameter("runs", 50)
        self.declare_parameter("mission_step", "step0")
        self.declare_parameter("command_topic", "/trolley_command")
        self.declare_parameter("command_text", "trolley_ready")
        self.declare_parameter("mission_param_overrides", "")
        self.declare_parameter("holonomic_param_overrides", "")
        self.declare_parameter("startup_timeout", 70.0)
        self.declare_parameter("per_run_timeout", 420.0)
        self.declare_parameter("success_hold_time", 1.0)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter("dropoff_x", 5.2)
        self.declare_parameter("dropoff_y", 0.0)
        self.declare_parameter("dropoff_tolerance_x", 0.25)
        self.declare_parameter("dropoff_tolerance_y", 0.20)
        self.declare_parameter("charging_x", 0.0)
        self.declare_parameter("charging_y", 0.0)
        self.declare_parameter("charging_tolerance_x", 0.20)
        self.declare_parameter("charging_tolerance_y", 0.20)
        self.declare_parameter("side_align_target_x", 0.015)
        self.declare_parameter("side_align_target_y", 0.0)
        self.declare_parameter("side_align_target_yaw_deg", 0.0)
        self.declare_parameter("fine_align_target_x", 0.0)
        self.declare_parameter("fine_align_target_y", 0.0)
        self.declare_parameter("fine_align_target_yaw_deg", 0.0)
        # Legacy names kept so older launch commands still work.
        self.declare_parameter("aruco_target_x", 0.0)
        self.declare_parameter("aruco_target_y", 0.0)
        self.declare_parameter("aruco_target_yaw_deg", 0.0)
        self.declare_parameter("result_dir", "/home/group4/Nursing-Home-Robot/mission_batch_results")
        self.declare_parameter("csv_filename", "mission_batch_results.csv")
        self.declare_parameter("log_dir", "/tmp/mission_batch_ros_logs")

        self.runs = int(self.get_parameter("runs").value)
        self.mission_step = self.get_parameter("mission_step").value
        self.command_topic = self.get_parameter("command_topic").value
        self.command_text = self.get_parameter("command_text").value
        self.mission_param_overrides = self.get_parameter("mission_param_overrides").value
        self.holonomic_param_overrides = self.get_parameter("holonomic_param_overrides").value
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.per_run_timeout = float(self.get_parameter("per_run_timeout").value)
        self.success_hold_time = float(self.get_parameter("success_hold_time").value)
        self.poll_period = float(self.get_parameter("poll_period").value)
        self.dropoff_x = float(self.get_parameter("dropoff_x").value)
        self.dropoff_y = float(self.get_parameter("dropoff_y").value)
        self.dropoff_tolerance_x = float(self.get_parameter("dropoff_tolerance_x").value)
        self.dropoff_tolerance_y = float(self.get_parameter("dropoff_tolerance_y").value)
        self.charging_x = float(self.get_parameter("charging_x").value)
        self.charging_y = float(self.get_parameter("charging_y").value)
        self.charging_tolerance_x = float(self.get_parameter("charging_tolerance_x").value)
        self.charging_tolerance_y = float(self.get_parameter("charging_tolerance_y").value)
        self.side_align_target_x = float(self.get_parameter("side_align_target_x").value)
        self.side_align_target_y = float(self.get_parameter("side_align_target_y").value)
        self.side_align_target_yaw = math.radians(
            float(self.get_parameter("side_align_target_yaw_deg").value)
        )
        self.fine_align_target_x = float(self.get_parameter("fine_align_target_x").value)
        self.fine_align_target_y = float(self.get_parameter("fine_align_target_y").value)
        self.fine_align_target_yaw = math.radians(
            float(self.get_parameter("fine_align_target_yaw_deg").value)
        )
        legacy_fine_x = float(self.get_parameter("aruco_target_x").value)
        legacy_fine_y = float(self.get_parameter("aruco_target_y").value)
        legacy_fine_yaw = math.radians(float(self.get_parameter("aruco_target_yaw_deg").value))
        if self.fine_align_target_x == 0.0 and legacy_fine_x != 0.0:
            self.fine_align_target_x = legacy_fine_x
        if self.fine_align_target_y == 0.0 and legacy_fine_y != 0.0:
            self.fine_align_target_y = legacy_fine_y
        if self.fine_align_target_yaw == 0.0 and legacy_fine_yaw != 0.0:
            self.fine_align_target_yaw = legacy_fine_yaw
        self.result_dir = Path(self.get_parameter("result_dir").value)
        self.csv_path = self.result_dir / self.get_parameter("csv_filename").value
        self.log_dir = Path(self.get_parameter("log_dir").value)

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.get_entity_client = self.create_client(GetEntityState, "/get_entity_state")
        self.mission_params_client = self.create_client(
            SetParameters, "/nav2_trolley_mission_controller/set_parameters"
        )
        self.holonomic_params_client = self.create_client(
            SetParameters, "/holonomic_goal_controller/set_parameters"
        )

        self.get_logger().info(
            f"Mission batch tester ready: runs={self.runs}, mission_step={self.mission_step}, csv={self.csv_path}"
        )

    def run_all(self):
        self.result_dir.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "run",
            "success",
            "failure_reason",
            "failure_detail",
            "duration_sec",
            "startup_duration_sec",
            "command_to_success_sec",
            "mission_started_sec",
            "attach_sec",
            "detach_sec",
            "dropoff_detected_sec",
            "charging_detected_sec",
            "last_trolley_x",
            "last_trolley_y",
            "last_trolley_yaw_deg",
            "last_robot_x",
            "last_robot_y",
            "last_robot_yaw_deg",
            "final_trolley_dropoff_error_x",
            "final_trolley_dropoff_error_y",
            "final_trolley_dropoff_error_dist",
            "final_robot_charging_error_x",
            "final_robot_charging_error_y",
            "final_robot_charging_error_dist",
            *self.alignment_fieldnames("side_align"),
            *self.alignment_fieldnames("fine_align"),
            "mission_param_overrides",
            "holonomic_param_overrides",
            "last_log_line",
        ]
        with self.csv_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for run_index in range(1, self.runs + 1):
                row = self.run_once(run_index)
                writer.writerow(row)
                csv_file.flush()
                self.get_logger().info(
                    f"Run {run_index}/{self.runs}: success={row['success']} "
                    f"reason={row['failure_reason']} duration={row['duration_sec']:.1f}s"
                )

        self.get_logger().info(f"Batch complete. Results written to {self.csv_path}")

    def run_once(self, run_index):
        run_start = time.monotonic()
        line_queue = queue.Queue()
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = str(self.log_dir / f"run_{run_index:03d}")
        cmd = [
            "ros2",
            "launch",
            "my_robot_description",
            "sim.launch.py",
            f"mission_step:={self.mission_step}",
        ]

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
            preexec_fn=os.setsid,
        )
        reader = threading.Thread(target=self.read_process_lines, args=(process, line_queue), daemon=True)
        reader.start()

        row = {
            "run": run_index,
            "success": False,
            "failure_reason": "",
            "failure_detail": "",
            "duration_sec": 0.0,
            "startup_duration_sec": "",
            "command_to_success_sec": "",
            "mission_started_sec": "",
            "attach_sec": "",
            "detach_sec": "",
            "dropoff_detected_sec": "",
            "charging_detected_sec": "",
            "last_trolley_x": "",
            "last_trolley_y": "",
            "last_trolley_yaw_deg": "",
            "last_robot_x": "",
            "last_robot_y": "",
            "last_robot_yaw_deg": "",
            "final_trolley_dropoff_error_x": "",
            "final_trolley_dropoff_error_y": "",
            "final_trolley_dropoff_error_dist": "",
            "final_robot_charging_error_x": "",
            "final_robot_charging_error_y": "",
            "final_robot_charging_error_dist": "",
            "mission_param_overrides": self.mission_param_overrides,
            "holonomic_param_overrides": self.holonomic_param_overrides,
            "last_log_line": "",
        }
        row.update(self.empty_alignment_values("side_align"))
        row.update(self.empty_alignment_values("fine_align"))

        ready_seen = False
        command_sent = False
        attach_seen = False
        detach_seen = False
        final_success_since = None
        last_poll = 0.0

        try:
            while time.monotonic() - run_start < self.per_run_timeout:
                now = time.monotonic()
                for line in self.drain_lines(line_queue):
                    row["last_log_line"] = line[-500:]
                    self.update_from_log_line(row, line, run_start)
                    if "Trolley mission controller ready" in line:
                        ready_seen = True
                        row["startup_duration_sec"] = round(now - run_start, 3)

                if not command_sent and ready_seen:
                    if not self.apply_mission_parameters_for_test():
                        row["failure_reason"] = "test_setup_failed"
                        row["failure_detail"] = "Could not apply mission controller test parameters."
                        break
                    if not self.apply_holonomic_parameters_for_test():
                        row["failure_reason"] = "test_setup_failed"
                        row["failure_detail"] = "Could not apply holonomic controller test parameters."
                        break
                    self.publish_command_burst()
                    command_sent = True

                if not ready_seen and now - run_start > self.startup_timeout:
                    row["failure_reason"] = "startup_timeout"
                    row["failure_detail"] = "Mission controller did not become ready."
                    break

                attach_seen = attach_seen or row["attach_sec"] != ""
                detach_seen = detach_seen or row["detach_sec"] != ""

                if command_sent and now - last_poll >= self.poll_period:
                    last_poll = now
                    trolley_pose = self.get_entity_pose("Trolley")
                    robot_pose = self.get_entity_pose("mecanum_bot")
                    if trolley_pose is not None:
                        tx, ty, tyaw = trolley_pose
                        row["last_trolley_x"] = round(tx, 4)
                        row["last_trolley_y"] = round(ty, 4)
                        row["last_trolley_yaw_deg"] = round(math.degrees(tyaw), 3)
                    if robot_pose is not None:
                        rx, ry, ryaw = robot_pose
                        row["last_robot_x"] = round(rx, 4)
                        row["last_robot_y"] = round(ry, 4)
                        row["last_robot_yaw_deg"] = round(math.degrees(ryaw), 3)

                    self.update_final_world_errors(row, trolley_pose, robot_pose)
                    in_dropoff = self.pose_in_box(
                        trolley_pose,
                        self.dropoff_x,
                        self.dropoff_y,
                        self.dropoff_tolerance_x,
                        self.dropoff_tolerance_y,
                    )
                    in_charging = self.pose_in_box(
                        robot_pose,
                        self.charging_x,
                        self.charging_y,
                        self.charging_tolerance_x,
                        self.charging_tolerance_y,
                    )
                    if in_dropoff and (attach_seen or detach_seen):
                        if row["dropoff_detected_sec"] == "":
                            row["dropoff_detected_sec"] = round(now - run_start, 3)
                    if in_charging and detach_seen:
                        if row["charging_detected_sec"] == "":
                            row["charging_detected_sec"] = round(now - run_start, 3)

                    if in_dropoff and in_charging and detach_seen:
                        if final_success_since is None:
                            final_success_since = now
                        elif now - final_success_since >= self.success_hold_time:
                            row["success"] = True
                            row["command_to_success_sec"] = round(now - run_start, 3)
                            break
                    else:
                        final_success_since = None

                if row["failure_reason"]:
                    break

                if process.poll() is not None:
                    row["failure_reason"] = "process_exited"
                    row["failure_detail"] = f"sim.launch.py exited with code {process.returncode}"
                    break

                time.sleep(0.05)

            if not row["success"] and not row["failure_reason"]:
                row["failure_reason"] = "run_timeout"
                row["failure_detail"] = (
                    "Robot did not return to charging while trolley remained in orange drop-off before timeout."
                )
        finally:
            row["duration_sec"] = round(time.monotonic() - run_start, 3)
            self.stop_process(process)

        return row

    def read_process_lines(self, process, line_queue):
        for line in process.stdout:
            line_queue.put(line.rstrip())

    def drain_lines(self, line_queue):
        lines = []
        while True:
            try:
                lines.append(line_queue.get_nowait())
            except queue.Empty:
                return lines

    def publish_command_burst(self):
        msg = String()
        msg.data = self.command_text
        end_time = time.monotonic() + 2.0
        self.get_logger().info(f"Publishing {self.command_text!r} on {self.command_topic}.")
        while time.monotonic() < end_time and rclpy.ok():
            self.command_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.1)

    def apply_mission_parameters_for_test(self):
        if not self.mission_params_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Could not apply mission parameters: mission parameter service not available.")
            return False
        parameters = [
            self.make_parameter("reset_trolley_after_mission", False),
            *self.parse_parameter_overrides(self.mission_param_overrides),
        ]
        return self.set_parameters_with_retries(
            self.mission_params_client,
            parameters,
            "mission parameters",
        )

    def apply_holonomic_parameters_for_test(self):
        parameters = self.parse_parameter_overrides(self.holonomic_param_overrides)
        if not parameters:
            return True
        if not self.holonomic_params_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Could not apply holonomic parameters: parameter service not available.")
            return False
        return self.set_parameters_with_retries(
            self.holonomic_params_client,
            parameters,
            "holonomic parameters",
        )

    def set_parameters_with_retries(self, client, parameters, label):
        for attempt in range(1, 6):
            if self.set_parameters_once(client, parameters, label):
                return True
            self.get_logger().warn(f"Retrying {label} ({attempt}/5).")
            time.sleep(0.5)
        return False

    def set_parameters_once(self, client, parameters, label):
        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"Could not apply {label}: parameter call timed out.")
            return False
        results = future.result().results
        ok = bool(results and all(result.successful for result in results))
        if not ok:
            reasons = [result.reason for result in results if not result.successful]
            self.get_logger().warn(f"Could not apply {label}: {'; '.join(reasons)}")
        return ok

    def parse_parameter_overrides(self, text):
        parameters = []
        if not text:
            return parameters
        for item in text.replace(",", ";").split(";"):
            item = item.strip()
            if not item:
                continue
            if "=" not in item:
                self.get_logger().warn(f"Ignoring malformed mission parameter override: {item}")
                continue
            name, raw_value = item.split("=", 1)
            parameters.append(self.make_parameter(name.strip(), self.parse_parameter_value(raw_value.strip())))
        return parameters

    def parse_parameter_value(self, raw_value):
        lowered = raw_value.lower()
        if lowered in ("true", "false"):
            return lowered == "true"
        try:
            if "." not in raw_value and "e" not in lowered:
                return int(raw_value)
            return float(raw_value)
        except ValueError:
            return raw_value

    def make_parameter(self, name, value):
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        elif isinstance(value, float):
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_STRING
            parameter.value.string_value = str(value)
        return parameter

    def get_entity_pose(self, entity_name):
        if not self.get_entity_client.service_is_ready():
            self.get_entity_client.wait_for_service(timeout_sec=0.05)
            return None
        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = "world"
        future = self.get_entity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
        if not future.done() or future.result() is None or not future.result().success:
            return None
        pose = future.result().state.pose
        return pose.position.x, pose.position.y, quaternion_to_yaw(pose.orientation)

    def pose_in_box(self, pose, target_x, target_y, tolerance_x, tolerance_y):
        if pose is None:
            return False
        x, y, _yaw = pose
        return abs(x - target_x) <= tolerance_x and abs(y - target_y) <= tolerance_y

    def update_final_world_errors(self, row, trolley_pose, robot_pose):
        if trolley_pose is not None:
            tx, ty, _tyaw = trolley_pose
            error_x = tx - self.dropoff_x
            error_y = ty - self.dropoff_y
            row["final_trolley_dropoff_error_x"] = round(error_x, 4)
            row["final_trolley_dropoff_error_y"] = round(error_y, 4)
            row["final_trolley_dropoff_error_dist"] = round(math.hypot(error_x, error_y), 4)

        if robot_pose is not None:
            rx, ry, _ryaw = robot_pose
            error_x = rx - self.charging_x
            error_y = ry - self.charging_y
            row["final_robot_charging_error_x"] = round(error_x, 4)
            row["final_robot_charging_error_y"] = round(error_y, 4)
            row["final_robot_charging_error_dist"] = round(math.hypot(error_x, error_y), 4)

    @staticmethod
    def alignment_fieldnames(stage):
        return [
            f"{stage}_complete_sec",
            f"{stage}_aruco_est_error_x",
            f"{stage}_aruco_est_error_y",
            f"{stage}_aruco_est_yaw_error_deg",
            f"{stage}_aruco_est_marker_count",
            f"{stage}_actual_trolley_base_x",
            f"{stage}_actual_trolley_base_y",
            f"{stage}_actual_trolley_yaw_error_deg",
            f"{stage}_actual_aruco_error_x",
            f"{stage}_actual_aruco_error_y",
            f"{stage}_actual_aruco_error_dist",
            f"{stage}_actual_aruco_yaw_error_deg",
            f"{stage}_aruco_vs_actual_error_x",
            f"{stage}_aruco_vs_actual_error_y",
            f"{stage}_aruco_vs_actual_yaw_error_deg",
        ]

    @classmethod
    def empty_alignment_values(cls, stage):
        return {fieldname: "" for fieldname in cls.alignment_fieldnames(stage)}

    def alignment_target(self, stage):
        if stage == "side_align":
            return (
                self.side_align_target_x,
                self.side_align_target_y,
                self.side_align_target_yaw,
            )
        return (
            self.fine_align_target_x,
            self.fine_align_target_y,
            self.fine_align_target_yaw,
        )

    def update_alignment_actual_errors(self, row, stage, trolley_pose=None, robot_pose=None):
        if trolley_pose is None:
            trolley_pose = self.get_entity_pose("Trolley")
        if robot_pose is None:
            robot_pose = self.get_entity_pose("mecanum_bot")
        if trolley_pose is None or robot_pose is None:
            return

        tx, ty, tyaw = trolley_pose
        rx, ry, ryaw = robot_pose
        trolley_base_x, trolley_base_y = self.world_to_robot_frame(tx, ty, rx, ry, ryaw)
        trolley_yaw_base = normalize_parallel_angle(tyaw - ryaw)
        target_x, target_y, target_yaw = self.alignment_target(stage)
        error_x = trolley_base_x - target_x
        error_y = trolley_base_y - target_y
        yaw_error = normalize_parallel_angle(trolley_yaw_base - target_yaw)

        row[f"{stage}_actual_trolley_base_x"] = round(trolley_base_x, 4)
        row[f"{stage}_actual_trolley_base_y"] = round(trolley_base_y, 4)
        row[f"{stage}_actual_trolley_yaw_error_deg"] = round(math.degrees(trolley_yaw_base), 3)
        row[f"{stage}_actual_aruco_error_x"] = round(error_x, 4)
        row[f"{stage}_actual_aruco_error_y"] = round(error_y, 4)
        row[f"{stage}_actual_aruco_error_dist"] = round(math.hypot(error_x, error_y), 4)
        row[f"{stage}_actual_aruco_yaw_error_deg"] = round(math.degrees(yaw_error), 3)
        self.update_alignment_vs_actual_errors(row, stage)

    def world_to_robot_frame(self, world_x, world_y, robot_x, robot_y, robot_yaw):
        dx = world_x - robot_x
        dy = world_y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    def update_alignment_vs_actual_errors(self, row, stage):
        if (
            row[f"{stage}_aruco_est_error_x"] == ""
            or row[f"{stage}_actual_aruco_error_x"] == ""
        ):
            return
        row[f"{stage}_aruco_vs_actual_error_x"] = round(
            float(row[f"{stage}_aruco_est_error_x"])
            - float(row[f"{stage}_actual_aruco_error_x"]),
            4,
        )
        row[f"{stage}_aruco_vs_actual_error_y"] = round(
            float(row[f"{stage}_aruco_est_error_y"])
            - float(row[f"{stage}_actual_aruco_error_y"]),
            4,
        )
        row[f"{stage}_aruco_vs_actual_yaw_error_deg"] = round(
            float(row[f"{stage}_aruco_est_yaw_error_deg"])
            - float(row[f"{stage}_actual_aruco_yaw_error_deg"]),
            3,
        )

    def update_from_log_line(self, row, line, run_start):
        elapsed = round(time.monotonic() - run_start, 3)
        if "Mission started" in line and row["mission_started_sec"] == "":
            row["mission_started_sec"] = elapsed
        self.update_alignment_estimate_from_log(
            row,
            line,
            elapsed,
            "side_align",
            "side ArUco pre-entry complete:",
        )
        self.update_alignment_estimate_from_log(
            row,
            line,
            elapsed,
            "fine_align",
            "final ArUco center under trolley complete:",
        )
        if "Service 'attach trolley' succeeded" in line and row["attach_sec"] == "":
            row["attach_sec"] = elapsed
        if "Service 'detach trolley' succeeded" in line and row["detach_sec"] == "":
            row["detach_sec"] = elapsed

        if "Direct close approach timed out" in line:
            self.set_failure(row, "direct_close_timeout", line)
        elif "Straight under trolley timed out" in line:
            self.set_failure(row, "straight_under_timeout", line)
        elif "timed out waiting for ArUco markers" in line:
            self.set_failure(row, "aruco_no_markers_timeout", line)
        elif "ArUco" in line and "timed out" in line:
            self.set_failure(row, "aruco_alignment_timeout", line)
        elif "Local target '" in line and "timed out" in line:
            self.set_failure(row, "local_target_timeout", line)
        elif "Holonomic goal '" in line and "timed out" in line:
            self.set_failure(row, "holonomic_goal_timeout", line)
        elif "Service 'attach trolley' failed" in line or "Service 'attach trolley' timed out" in line:
            self.set_failure(row, "attach_failed", line)
        elif "Service 'detach trolley' failed" in line or "Service 'detach trolley' timed out" in line:
            self.set_failure(row, "detach_failed", line)
        elif "No /odom pose received" in line:
            self.set_failure(row, "odom_timeout", line)

    def update_alignment_estimate_from_log(self, row, line, elapsed, stage, completion_text):
        if completion_text not in line:
            return
        match = re.search(
            r"x=([+-]?\d+(?:\.\d+)?), y=([+-]?\d+(?:\.\d+)?), "
            r"yaw=([+-]?\d+(?:\.\d+)?) deg, markers=(\d+)",
            line,
        )
        if match is None:
            return
        row[f"{stage}_complete_sec"] = elapsed
        row[f"{stage}_aruco_est_error_x"] = round(float(match.group(1)), 4)
        row[f"{stage}_aruco_est_error_y"] = round(float(match.group(2)), 4)
        row[f"{stage}_aruco_est_yaw_error_deg"] = round(float(match.group(3)), 3)
        row[f"{stage}_aruco_est_marker_count"] = int(match.group(4))
        self.update_alignment_actual_errors(row, stage)
        self.update_alignment_vs_actual_errors(row, stage)

    def set_failure(self, row, reason, detail):
        if not row["failure_reason"]:
            row["failure_reason"] = reason
            row["failure_detail"] = detail[-500:]

    def stop_process(self, process):
        if process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass


def main():
    rclpy.init()
    node = MissionBatchTester()
    try:
        node.run_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
