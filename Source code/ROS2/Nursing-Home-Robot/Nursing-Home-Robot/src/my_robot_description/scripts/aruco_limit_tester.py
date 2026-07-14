#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path
from typing import Optional, Tuple, Dict, List

import rclpy
from rclpy.node import Node

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

import tf2_ros
import tf2_geometry_msgs


def parse_float_list(text: str) -> List[float]:
    return [float(v.strip()) for v in text.split(",") if v.strip()]


def quaternion_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ArucoLimitTester(Node):
    def __init__(self):
        super().__init__("aruco_limit_tester")

        # Frames and topics
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("front_topic", "/front/aruco_markers")
        self.declare_parameter("rear_topic", "/rear/aruco_markers")

        # Gazebo entities
        self.declare_parameter("robot_entity", "mecanum_bot")
        self.declare_parameter("front_panel_entity", "front_aruco_panel")
        self.declare_parameter("rear_panel_entity", "rear_aruco_panel")

        # Ideal panel locations in base_link
        self.declare_parameter("target_front_x", 0.4875)
        self.declare_parameter("target_rear_x", -0.4875)
        self.declare_parameter("target_y", 0.0)

        # Initial robot pose center
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw_deg", 0.0)

        # Test ranges
        self.declare_parameter(
            "velocity_values",
            "0.02,0.04,0.06,0.08"
        )
        self.declare_parameter(
            "angular_velocity_values",
            "0.08,0.12,0.15,0.20"
        )
        self.declare_parameter(
            "lateral_offsets",
            "0.00,0.02,0.04,0.06,0.08,0.10,0.12,0.14,0.16,0.18,0.20"
        )
        self.declare_parameter(
            "yaw_offsets_deg",
            "0,2,4,6,8,10,12,14,16,18,20"
        )

        # Controller gains
        self.declare_parameter("kx", 0.25)
        self.declare_parameter("ky", 0.50)
        self.declare_parameter("kyaw", 0.90)

        # Direction signs
        self.declare_parameter("x_sign", 1.0)
        self.declare_parameter("y_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)

        # Success criteria
        self.declare_parameter("x_tolerance", 0.005)
        self.declare_parameter("y_tolerance", 0.005)
        self.declare_parameter("yaw_tolerance_deg", 1.0)
        self.declare_parameter("success_hold_time", 0.5)

        # Detection
        self.declare_parameter("min_markers_required", 2)
        self.declare_parameter("detection_wait_timeout", 5.0)

        # Trial timing
        self.declare_parameter("settle_time", 1.0)

        # trial_timeout <= 0.0 means no fixed timeout.
        # Trial then stops only on success, stagnation, or max_trial_time.
        self.declare_parameter("trial_timeout", 0.0)

        # Absolute safety limit. Use 0.0 to disable.
        self.declare_parameter("max_trial_time", 180.0)

        self.declare_parameter("control_period", 0.1)

        # Stagnation handling
        self.declare_parameter("stagnation_timeout", 25.0)
        self.declare_parameter("min_progress", 0.002)

        # Output
        self.declare_parameter(
            "result_dir",
            "/home/group4/Nursing-Home-Robot/aruco_limit_test_results"
        )

        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.front_panel_base: Optional[Tuple[float, float, float, int]] = None
        self.rear_panel_base: Optional[Tuple[float, float, float, int]] = None

        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("front_topic").value,
            self.front_callback,
            10,
        )

        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("rear_topic").value,
            self.rear_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_state_client = self.create_client(GetEntityState, "/get_entity_state")
        self.set_state_client = self.create_client(SetEntityState, "/set_entity_state")

        self.trial_rows = []
        self.summary_rows = []

        self.get_logger().info("ArUco limit tester initialized.")
        self.get_logger().info("Do not run the normal alignment controller at the same time.")
        self.get_logger().info("trial_timeout <= 0 means trials run until success, stagnation, or max_trial_time.")

    def front_callback(self, msg: ArucoMarkers):
        self.front_panel_base = self.average_markers_in_base(msg)

    def rear_callback(self, msg: ArucoMarkers):
        self.rear_panel_base = self.average_markers_in_base(msg)

    def average_markers_in_base(self, msg: ArucoMarkers) -> Optional[Tuple[float, float, float, int]]:
        min_markers_required = self.get_parameter("min_markers_required").value

        if len(msg.poses) < min_markers_required:
            return None

        points = []

        for pose in msg.poses:
            pose_cam = PoseStamped()
            pose_cam.header = msg.header
            pose_cam.pose = pose

            try:
                pose_base = self.tf_buffer.transform(
                    pose_cam,
                    self.base_frame,
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except Exception:
                return None

            points.append((
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z,
            ))

        x = sum(p[0] for p in points) / len(points)
        y = sum(p[1] for p in points) / len(points)
        z = sum(p[2] for p in points) / len(points)

        return x, y, z, len(points)

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def wait_for_services(self):
        self.get_logger().info("Waiting for /get_entity_state and /set_entity_state...")

        while rclpy.ok() and not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_entity_state...")

        while rclpy.ok() and not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /set_entity_state...")

        self.get_logger().info("Gazebo state services are available.")

    def call_get_entity_state(self, name: str):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = "world"

        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done():
            return None

        result = future.result()

        if result is None or not result.success:
            return None

        return result.state.pose

    def call_set_robot_state(self, x: float, y: float, yaw: float) -> bool:
        robot_entity = self.get_parameter("robot_entity").value

        state = EntityState()
        state.name = robot_entity
        state.reference_frame = "world"

        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.02
        state.pose.orientation = quaternion_from_yaw(yaw)

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        req = SetEntityState.Request()
        req.state = state

        future = self.set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done():
            return False

        result = future.result()

        if result is None or not result.success:
            return False

        return True

    def world_to_robot_frame(self, world_x, world_y, robot_x, robot_y, robot_yaw):
        dx = world_x - robot_x
        dy = world_y - robot_y

        c = math.cos(-robot_yaw)
        s = math.sin(-robot_yaw)

        x_robot = c * dx - s * dy
        y_robot = s * dx + c * dy

        return x_robot, y_robot

    def compute_ground_truth_errors(self):
        robot_name = self.get_parameter("robot_entity").value
        front_name = self.get_parameter("front_panel_entity").value
        rear_name = self.get_parameter("rear_panel_entity").value

        robot_pose = self.call_get_entity_state(robot_name)
        front_pose = self.call_get_entity_state(front_name)
        rear_pose = self.call_get_entity_state(rear_name)

        if robot_pose is None or front_pose is None or rear_pose is None:
            return None

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)

        front_x_base, front_y_base = self.world_to_robot_frame(
            front_pose.position.x,
            front_pose.position.y,
            robot_x,
            robot_y,
            robot_yaw,
        )

        rear_x_base, rear_y_base = self.world_to_robot_frame(
            rear_pose.position.x,
            rear_pose.position.y,
            robot_x,
            robot_y,
            robot_yaw,
        )

        target_front_x = self.get_parameter("target_front_x").value
        target_rear_x = self.get_parameter("target_rear_x").value
        target_y = self.get_parameter("target_y").value
        expected_panel_distance = abs(target_front_x - target_rear_x)

        front_x_error = front_x_base - target_front_x
        rear_x_error = rear_x_base - target_rear_x

        true_distance_error_x = 0.5 * (front_x_error + rear_x_error)
        true_lateral_error_y = 0.5 * ((front_y_base - target_y) + (rear_y_base - target_y))
        true_angle_error_rad = math.atan2(front_y_base - rear_y_base, expected_panel_distance)
        true_angle_error_deg = math.degrees(true_angle_error_rad)

        return {
            "true_robot_world_x": robot_x,
            "true_robot_world_y": robot_y,
            "true_robot_world_yaw_deg": math.degrees(robot_yaw),
            "true_front_x": front_x_base,
            "true_front_y": front_y_base,
            "true_rear_x": rear_x_base,
            "true_rear_y": rear_y_base,
            "true_distance_error_x": true_distance_error_x,
            "true_lateral_error_y": true_lateral_error_y,
            "true_angle_error_rad": true_angle_error_rad,
            "true_angle_error_deg": true_angle_error_deg,
        }

    def compute_aruco_errors(self):
        if self.front_panel_base is None or self.rear_panel_base is None:
            return None

        front_x, front_y, front_z, front_n = self.front_panel_base
        rear_x, rear_y, rear_z, rear_n = self.rear_panel_base

        target_front_x = self.get_parameter("target_front_x").value
        target_rear_x = self.get_parameter("target_rear_x").value
        target_y = self.get_parameter("target_y").value
        expected_panel_distance = abs(target_front_x - target_rear_x)

        front_x_error = front_x - target_front_x
        rear_x_error = rear_x - target_rear_x

        distance_error_x = 0.5 * (front_x_error + rear_x_error)
        lateral_error_y = 0.5 * ((front_y - target_y) + (rear_y - target_y))
        angle_error_rad = math.atan2(front_y - rear_y, expected_panel_distance)
        angle_error_deg = math.degrees(angle_error_rad)

        return {
            "aruco_distance_error_x": distance_error_x,
            "aruco_lateral_error_y": lateral_error_y,
            "aruco_angle_error_rad": angle_error_rad,
            "aruco_angle_error_deg": angle_error_deg,
            "aruco_front_x": front_x,
            "aruco_front_y": front_y,
            "aruco_front_z": front_z,
            "aruco_rear_x": rear_x,
            "aruco_rear_y": rear_y,
            "aruco_rear_z": rear_z,
            "front_markers": front_n,
            "rear_markers": rear_n,
        }

    def wait_seconds(self, seconds: float):
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_detections(self, timeout: float) -> bool:
        self.front_panel_base = None
        self.rear_panel_base = None

        end_time = time.time() + timeout

        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.front_panel_base is not None and self.rear_panel_base is not None:
                return True

        return False

    def run_single_trial(
        self,
        trial_id: int,
        test_type: str,
        max_v: float,
        max_wz: float,
        initial_y: float,
        initial_yaw_deg: float,
    ) -> Dict:
        start_x = self.get_parameter("start_x").value
        start_y = self.get_parameter("start_y").value
        start_yaw_deg = self.get_parameter("start_yaw_deg").value

        initial_world_x = start_x
        initial_world_y = start_y + initial_y
        initial_world_yaw = math.radians(start_yaw_deg + initial_yaw_deg)

        self.get_logger().info(
            f"Trial {trial_id}: type={test_type}, max_v={max_v:.3f}, "
            f"max_wz={max_wz:.3f}, initial_y={initial_y:+.3f}, "
            f"initial_yaw={initial_yaw_deg:+.1f} deg"
        )

        self.stop_robot()
        self.wait_seconds(0.2)

        set_ok = self.call_set_robot_state(
            initial_world_x,
            initial_world_y,
            initial_world_yaw,
        )

        if not set_ok:
            self.get_logger().warn("Failed to set robot state.")
            return {
                "trial_id": trial_id,
                "test_type": test_type,
                "max_v": max_v,
                "max_wz": max_wz,
                "initial_y": initial_y,
                "initial_yaw_deg": initial_yaw_deg,
                "success": False,
                "failure_reason": "set_state_failed",
                "alignment_time": float("nan"),
            }

        self.stop_robot()
        self.wait_seconds(self.get_parameter("settle_time").value)

        detected = self.wait_for_detections(
            self.get_parameter("detection_wait_timeout").value
        )

        if not detected:
            self.get_logger().warn("No valid front+rear ArUco detections.")
            return {
                "trial_id": trial_id,
                "test_type": test_type,
                "max_v": max_v,
                "max_wz": max_wz,
                "initial_y": initial_y,
                "initial_yaw_deg": initial_yaw_deg,
                "success": False,
                "failure_reason": "no_detection",
                "alignment_time": float("nan"),
            }

        x_tol = self.get_parameter("x_tolerance").value
        y_tol = self.get_parameter("y_tolerance").value
        yaw_tol = math.radians(self.get_parameter("yaw_tolerance_deg").value)
        success_hold_time = self.get_parameter("success_hold_time").value

        trial_timeout = self.get_parameter("trial_timeout").value
        max_trial_time = self.get_parameter("max_trial_time").value
        stagnation_timeout = self.get_parameter("stagnation_timeout").value
        min_progress = self.get_parameter("min_progress").value
        control_period = self.get_parameter("control_period").value

        kx = self.get_parameter("kx").value
        ky = self.get_parameter("ky").value
        kyaw = self.get_parameter("kyaw").value

        x_sign = self.get_parameter("x_sign").value
        y_sign = self.get_parameter("y_sign").value
        yaw_sign = self.get_parameter("yaw_sign").value

        expected_panel_distance = abs(
            self.get_parameter("target_front_x").value
            - self.get_parameter("target_rear_x").value
        )

        trial_start = time.time()
        last_control_time = 0.0
        success_start = None

        last_aruco = None
        last_true = None
        last_cmd = Twist()

        best_error_score = float("inf")
        last_progress_time = time.time()

        success = False
        failure_reason = ""

        while rclpy.ok():
            now = time.time()
            elapsed = now - trial_start

            if trial_timeout > 0.0 and elapsed > trial_timeout:
                self.stop_robot()
                success = False
                failure_reason = "timeout"
                break

            if max_trial_time > 0.0 and elapsed > max_trial_time:
                self.stop_robot()
                success = False
                failure_reason = "max_trial_time_reached"
                break

            rclpy.spin_once(self, timeout_sec=0.02)

            if now - last_control_time < control_period:
                continue

            last_control_time = now

            aruco = self.compute_aruco_errors()

            if aruco is None:
                self.stop_robot()
                success_start = None
                continue

            true = self.compute_ground_truth_errors()

            distance_error_x = aruco["aruco_distance_error_x"]
            lateral_error_y = aruco["aruco_lateral_error_y"]
            angle_error_rad = aruco["aruco_angle_error_rad"]

            error_score = math.sqrt(
                distance_error_x**2
                + lateral_error_y**2
                + (expected_panel_distance * angle_error_rad) ** 2
            )

            if error_score < best_error_score - min_progress:
                best_error_score = error_score
                last_progress_time = now

            if now - last_progress_time > stagnation_timeout:
                self.stop_robot()
                success = False
                failure_reason = "stagnation"
                break

            cmd = Twist()

            if abs(distance_error_x) > x_tol:
                cmd.linear.x = self.clamp(x_sign * kx * distance_error_x, max_v)

            if abs(lateral_error_y) > y_tol:
                cmd.linear.y = self.clamp(y_sign * ky * lateral_error_y, max_v)

            if abs(angle_error_rad) > yaw_tol:
                cmd.angular.z = self.clamp(yaw_sign * kyaw * angle_error_rad, max_wz)

            aligned = (
                abs(distance_error_x) <= x_tol
                and abs(lateral_error_y) <= y_tol
                and abs(angle_error_rad) <= yaw_tol
            )

            if aligned:
                if success_start is None:
                    success_start = now

                self.stop_robot()

                if now - success_start >= success_hold_time:
                    success = True
                    failure_reason = ""
                    break
            else:
                success_start = None
                self.cmd_pub.publish(cmd)

            last_aruco = aruco
            last_true = true
            last_cmd = cmd

        alignment_time = time.time() - trial_start
        self.stop_robot()
        self.wait_seconds(0.2)

        final_aruco = self.compute_aruco_errors() or last_aruco
        final_true = self.compute_ground_truth_errors() or last_true

        row = {
            "trial_id": trial_id,
            "test_type": test_type,
            "max_v": max_v,
            "max_wz": max_wz,
            "initial_y": initial_y,
            "initial_yaw_deg": initial_yaw_deg,
            "success": success,
            "failure_reason": failure_reason,
            "alignment_time": alignment_time,
            "best_error_score": best_error_score,
            "trial_timeout": trial_timeout,
            "max_trial_time": max_trial_time,
            "stagnation_timeout": stagnation_timeout,
            "min_progress": min_progress,
        }

        if final_aruco is not None:
            row.update(final_aruco)

        if final_true is not None:
            row.update(final_true)

        if final_aruco is not None and final_true is not None:
            row["distance_error_estimation_error"] = (
                final_aruco["aruco_distance_error_x"]
                - final_true["true_distance_error_x"]
            )
            row["lateral_error_estimation_error"] = (
                final_aruco["aruco_lateral_error_y"]
                - final_true["true_lateral_error_y"]
            )
            angle_diff_rad = normalize_angle(
                final_aruco["aruco_angle_error_rad"]
                - final_true["true_angle_error_rad"]
            )
            row["angle_error_estimation_error_rad"] = angle_diff_rad
            row["angle_error_estimation_error_deg"] = math.degrees(angle_diff_rad)

        row["final_cmd_vx"] = last_cmd.linear.x
        row["final_cmd_vy"] = last_cmd.linear.y
        row["final_cmd_wz"] = last_cmd.angular.z

        self.get_logger().info(
            f"Trial {trial_id} result: success={success}, "
            f"time={alignment_time:.2f}s, reason={failure_reason}, "
            f"best_error_score={best_error_score:.4f}"
        )

        return row

    def run_benchmark(self):
        self.wait_for_services()

        velocities = parse_float_list(self.get_parameter("velocity_values").value)
        angular_velocities = parse_float_list(self.get_parameter("angular_velocity_values").value)
        lateral_offsets = parse_float_list(self.get_parameter("lateral_offsets").value)
        yaw_offsets_deg = parse_float_list(self.get_parameter("yaw_offsets_deg").value)

        if len(angular_velocities) != len(velocities):
            self.get_logger().warn(
                "angular_velocity_values length differs from velocity_values. "
                "Using the first angular value for all."
            )
            angular_velocities = [angular_velocities[0] for _ in velocities]

        trial_id = 0

        for velocity_index, max_v in enumerate(velocities):
            max_wz = angular_velocities[velocity_index]

            lateral_successes = []
            yaw_successes = []

            for y_offset in lateral_offsets:
                trial_id += 1

                row = self.run_single_trial(
                    trial_id=trial_id,
                    test_type="lateral_only",
                    max_v=max_v,
                    max_wz=max_wz,
                    initial_y=y_offset,
                    initial_yaw_deg=0.0,
                )

                self.trial_rows.append(row)

                if row.get("success"):
                    lateral_successes.append(row)

            for yaw_offset in yaw_offsets_deg:
                trial_id += 1

                row = self.run_single_trial(
                    trial_id=trial_id,
                    test_type="yaw_only",
                    max_v=max_v,
                    max_wz=max_wz,
                    initial_y=0.0,
                    initial_yaw_deg=yaw_offset,
                )

                self.trial_rows.append(row)

                if row.get("success"):
                    yaw_successes.append(row)

            max_lateral_success = max(
                [abs(row["initial_y"]) for row in lateral_successes],
                default=float("nan"),
            )
            max_yaw_success = max(
                [abs(row["initial_yaw_deg"]) for row in yaw_successes],
                default=float("nan"),
            )

            lateral_times = [row["alignment_time"] for row in lateral_successes]
            yaw_times = [row["alignment_time"] for row in yaw_successes]

            avg_lateral_time = (
                sum(lateral_times) / len(lateral_times)
                if lateral_times else float("nan")
            )
            avg_yaw_time = (
                sum(yaw_times) / len(yaw_times)
                if yaw_times else float("nan")
            )

            self.summary_rows.append({
                "max_v": max_v,
                "max_wz": max_wz,
                "max_successful_lateral_offset_m": max_lateral_success,
                "max_successful_yaw_offset_deg": max_yaw_success,
                "average_lateral_alignment_time_s": avg_lateral_time,
                "average_yaw_alignment_time_s": avg_yaw_time,
                "lateral_success_count": len(lateral_successes),
                "yaw_success_count": len(yaw_successes),
            })

        self.stop_robot()
        self.save_results()

    def save_results(self):
        result_dir = Path(self.get_parameter("result_dir").value)
        result_dir.mkdir(parents=True, exist_ok=True)

        trial_csv = result_dir / "aruco_limit_trials.csv"
        summary_csv = result_dir / "aruco_limit_summary.csv"
        plot_path = result_dir / "aruco_limit_summary_plot.png"

        if self.trial_rows:
            all_keys = []
            for row in self.trial_rows:
                for key in row.keys():
                    if key not in all_keys:
                        all_keys.append(key)

            with open(trial_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=all_keys)
                writer.writeheader()
                writer.writerows(self.trial_rows)

        if self.summary_rows:
            with open(summary_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.summary_rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.summary_rows)

        self.create_summary_plot(plot_path)

        self.get_logger().info(
            "\n"
            "================ LIMIT TEST FINISHED ================\n"
            f"Trial CSV saved to:   {trial_csv}\n"
            f"Summary CSV saved to: {summary_csv}\n"
            f"Plot saved to:        {plot_path}\n"
            "====================================================="
        )

    def create_summary_plot(self, plot_path: Path):
        try:
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(f"Could not create plot: {e}")
            return

        if not self.summary_rows:
            return

        velocities = [row["max_v"] for row in self.summary_rows]
        max_y = [row["max_successful_lateral_offset_m"] for row in self.summary_rows]
        max_yaw = [row["max_successful_yaw_offset_deg"] for row in self.summary_rows]
        t_y = [row["average_lateral_alignment_time_s"] for row in self.summary_rows]
        t_yaw = [row["average_yaw_alignment_time_s"] for row in self.summary_rows]

        plt.figure(figsize=(10, 6))
        plt.plot(velocities, max_y, marker="o", label="Max lateral offset [m]")
        plt.plot(velocities, max_yaw, marker="o", label="Max yaw offset [deg]")
        plt.plot(velocities, t_y, marker="o", label="Avg lateral alignment time [s]")
        plt.plot(velocities, t_yaw, marker="o", label="Avg yaw alignment time [s]")
        plt.xlabel("Maximum linear velocity [m/s]")
        plt.ylabel("Limit / Time")
        plt.title("ArUco Alignment Limits vs Velocity")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLimitTester()

    try:
        node.run_benchmark()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted. Stopping robot.")
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()