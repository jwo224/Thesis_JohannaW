#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path

import rclpy
from aruco_interfaces.msg import ArucoMarkers
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # Registers PoseStamped transforms with tf2.


def parse_float_list(value):
    if isinstance(value, str):
        return [float(item.strip()) for item in value.split(",") if item.strip()]
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return [float(value)]


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


def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ArucoDockingTestRunner(Node):
    def __init__(self):
        super().__init__("aruco_docking_test_runner")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("aruco_topics", "/front/aruco_markers,/rear/aruco_markers,/left/aruco_markers,/right/aruco_markers")
        self.declare_parameter("robot_entity", "mecanum_bot")
        self.declare_parameter("trolley_entity", "Trolley")
        self.declare_parameter("trolley_x", 2.5)
        self.declare_parameter("trolley_y", 0.0)
        self.declare_parameter("trolley_yaw_deg", 0.0)
        self.declare_parameter("robot_start_x", 2.5)
        self.declare_parameter("robot_start_y", 0.85)
        self.declare_parameter("robot_start_yaw_deg", 0.0)
        self.declare_parameter("lateral_offsets", [0.0])
        self.declare_parameter("yaw_offsets_deg", [0.0])
        self.declare_parameter("settle_time", 1.0)
        self.declare_parameter("entity_wait_timeout", 20.0)
        self.declare_parameter("entity_service_timeout", 3.0)
        self.declare_parameter("reset_retries", 5)
        self.declare_parameter("result_dir", "/home/group4/Nursing-Home-Robot/aruco_docking_test_results")
        self.declare_parameter("csv_filename", "aruco_docking_test.csv")

        self.declare_parameter("aruco_timeout", 2.0)
        self.declare_parameter("min_aruco_markers", 3)
        self.declare_parameter("entry_offset_x", 0.015)
        self.declare_parameter("aruco_target_x", 0.0)
        self.declare_parameter("aruco_target_y", 0.0)
        self.declare_parameter("side_align_x_tolerance", 0.020)
        self.declare_parameter("side_align_yaw_tolerance_deg", 2.0)
        self.declare_parameter("skip_side_align_if_already_close", True)
        self.declare_parameter("fine_position_tolerance", 0.010)
        self.declare_parameter("fine_yaw_tolerance_deg", 1.0)
        self.declare_parameter("straight_under_stop_y", 0.16)
        self.declare_parameter("align_hold_time", 0.35)

        self.declare_parameter("aruco_kxy", 0.75)
        self.declare_parameter("aruco_kyaw", 1.6)
        self.declare_parameter("x_control_sign", 1.0)
        self.declare_parameter("y_control_sign", 1.0)
        self.declare_parameter("yaw_control_sign", 1.0)
        self.declare_parameter("fine_min_vxy", 0.012)
        self.declare_parameter("fine_max_vx", 0.035)
        self.declare_parameter("fine_max_vy", 0.035)
        self.declare_parameter("fine_max_wz", 0.10)
        self.declare_parameter("drive_under_max_vx", 0.05)
        self.declare_parameter("drive_under_max_vy", 0.12)
        self.declare_parameter("drive_under_max_wz", 0.16)
        self.declare_parameter("straight_under_speed", 0.16)
        self.declare_parameter("side_align_timeout", 30.0)
        self.declare_parameter("straight_under_timeout", 20.0)
        self.declare_parameter("final_align_timeout", 45.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.robot_entity = self.get_parameter("robot_entity").value
        self.trolley_entity = self.get_parameter("trolley_entity").value
        self.trolley_x = float(self.get_parameter("trolley_x").value)
        self.trolley_y = float(self.get_parameter("trolley_y").value)
        self.trolley_yaw = math.radians(float(self.get_parameter("trolley_yaw_deg").value))
        self.robot_start_x = float(self.get_parameter("robot_start_x").value)
        self.robot_start_y = float(self.get_parameter("robot_start_y").value)
        self.robot_start_yaw = math.radians(float(self.get_parameter("robot_start_yaw_deg").value))
        self.lateral_offsets = parse_float_list(self.get_parameter("lateral_offsets").value)
        self.yaw_offsets = [math.radians(v) for v in parse_float_list(self.get_parameter("yaw_offsets_deg").value)]
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.entity_wait_timeout = float(self.get_parameter("entity_wait_timeout").value)
        self.entity_service_timeout = float(self.get_parameter("entity_service_timeout").value)
        self.reset_retries = int(self.get_parameter("reset_retries").value)
        self.aruco_timeout = float(self.get_parameter("aruco_timeout").value)
        self.min_aruco_markers = int(self.get_parameter("min_aruco_markers").value)
        self.entry_offset_x = float(self.get_parameter("entry_offset_x").value)
        self.aruco_target_x = float(self.get_parameter("aruco_target_x").value)
        self.aruco_target_y = float(self.get_parameter("aruco_target_y").value)
        self.side_align_x_tolerance = float(self.get_parameter("side_align_x_tolerance").value)
        self.side_align_yaw_tolerance = math.radians(float(self.get_parameter("side_align_yaw_tolerance_deg").value))
        self.skip_side_align_if_already_close = bool(self.get_parameter("skip_side_align_if_already_close").value)
        self.fine_position_tolerance = float(self.get_parameter("fine_position_tolerance").value)
        self.fine_yaw_tolerance = math.radians(float(self.get_parameter("fine_yaw_tolerance_deg").value))
        self.straight_under_stop_y = float(self.get_parameter("straight_under_stop_y").value)
        self.align_hold_time = float(self.get_parameter("align_hold_time").value)
        self.aruco_kxy = float(self.get_parameter("aruco_kxy").value)
        self.aruco_kyaw = float(self.get_parameter("aruco_kyaw").value)
        self.x_control_sign = float(self.get_parameter("x_control_sign").value)
        self.y_control_sign = float(self.get_parameter("y_control_sign").value)
        self.yaw_control_sign = float(self.get_parameter("yaw_control_sign").value)
        self.fine_min_vxy = float(self.get_parameter("fine_min_vxy").value)
        self.fine_max_vx = float(self.get_parameter("fine_max_vx").value)
        self.fine_max_vy = float(self.get_parameter("fine_max_vy").value)
        self.fine_max_wz = float(self.get_parameter("fine_max_wz").value)
        self.drive_under_max_vx = float(self.get_parameter("drive_under_max_vx").value)
        self.drive_under_max_vy = float(self.get_parameter("drive_under_max_vy").value)
        self.drive_under_max_wz = float(self.get_parameter("drive_under_max_wz").value)
        self.straight_under_speed = float(self.get_parameter("straight_under_speed").value)
        self.side_align_timeout = float(self.get_parameter("side_align_timeout").value)
        self.straight_under_timeout = float(self.get_parameter("straight_under_timeout").value)
        self.final_align_timeout = float(self.get_parameter("final_align_timeout").value)

        marker_x_positions = [-0.39, -0.26, -0.13, 0.00, 0.13, 0.26, 0.39]
        marker_y_positions = [0.21, 0.07, -0.07, -0.21]
        self.marker_layout = {
            row_index * len(marker_x_positions) + col_index: (x, y)
            for row_index, y in enumerate(marker_y_positions)
            for col_index, x in enumerate(marker_x_positions)
        }
        self.latest_markers = {}
        self.rows = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_state_client = self.create_client(GetEntityState, "/get_entity_state")
        self.set_state_client = self.create_client(SetEntityState, "/set_entity_state")

        for topic in [v.strip() for v in self.get_parameter("aruco_topics").value.split(",") if v.strip()]:
            self.create_subscription(ArucoMarkers, topic, self.aruco_callback, 10)

        self.output_path = Path(self.get_parameter("result_dir").value) / self.get_parameter("csv_filename").value
        self.get_logger().info(f"ArUco docking test runner ready. CSV: {self.output_path}")

    def aruco_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)
            if marker_id not in self.marker_layout:
                continue

            pose_cam = PoseStamped()
            pose_cam.header = msg.header
            pose_cam.header.stamp = Time().to_msg()
            pose_cam.pose = pose

            try:
                pose_base = self.tf_buffer.transform(
                    pose_cam,
                    self.base_frame,
                    timeout=Duration(seconds=0.1),
                )
            except TransformException:
                continue

            self.latest_markers[marker_id] = (
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z,
                quaternion_to_yaw(pose_base.pose.orientation),
                now,
            )

    def wait_for_services(self):
        self.get_logger().info("Waiting for Gazebo entity state services...")
        while rclpy.ok() and not self.get_state_client.wait_for_service(timeout_sec=1.0):
            pass
        while rclpy.ok() and not self.set_state_client.wait_for_service(timeout_sec=1.0):
            pass
        self.wait_for_entities()

    def wait_for_entities(self):
        self.get_logger().info(f"Waiting for Gazebo entities: {self.robot_entity}, {self.trolley_entity}")
        start = time.monotonic()
        while rclpy.ok():
            robot_ready = self.call_get_entity_state(self.robot_entity) is not None
            trolley_ready = self.call_get_entity_state(self.trolley_entity) is not None
            if robot_ready and trolley_ready:
                self.get_logger().info("Gazebo test entities are available.")
                return True
            if time.monotonic() - start > self.entity_wait_timeout:
                self.get_logger().error(
                    f"Timed out waiting for entities. robot_ready={robot_ready}, trolley_ready={trolley_ready}"
                )
                return False
            time.sleep(0.25)

    def call_get_entity_state(self, name):
        request = GetEntityState.Request()
        request.name = name
        request.reference_frame = "world"
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.entity_service_timeout)
        if not future.done() or future.result() is None or not future.result().success:
            return None
        return future.result().state.pose

    def call_set_entity_state(self, name, x, y, z, yaw):
        state = EntityState()
        state.name = name
        state.reference_frame = "world"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation = yaw_to_quaternion(yaw)
        request = SetEntityState.Request()
        request.state = state
        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.entity_service_timeout)
        if not future.done():
            self.get_logger().warn(f"SetEntityState timed out for {name}.")
            return False
        result = future.result()
        if result is None:
            self.get_logger().warn(f"SetEntityState returned no result for {name}.")
            return False
        if not result.success:
            self.get_logger().warn(f"SetEntityState failed for {name}: {result.status_message}")
            return False
        return True

    def reset_trial(self, lateral_offset, yaw_offset):
        self.stop_robot()
        self.latest_markers.clear()
        for attempt in range(1, self.reset_retries + 1):
            if self.call_get_entity_state(self.robot_entity) is None or self.call_get_entity_state(self.trolley_entity) is None:
                self.get_logger().warn(f"Trial reset attempt {attempt}: entities are not ready yet.")
                time.sleep(0.5)
                continue

            ok_trolley = self.call_set_entity_state(
                self.trolley_entity,
                self.trolley_x,
                self.trolley_y,
                0.0,
                self.trolley_yaw,
            )
            ok_robot = self.call_set_entity_state(
                self.robot_entity,
                self.robot_start_x,
                self.robot_start_y + lateral_offset,
                0.02,
                self.robot_start_yaw + yaw_offset,
            )
            if ok_trolley and ok_robot:
                time.sleep(self.settle_time)
                return True
            self.get_logger().warn(
                f"Trial reset attempt {attempt} failed: ok_trolley={ok_trolley}, ok_robot={ok_robot}"
            )
            time.sleep(0.5)
        return False

    def estimate_trolley_from_aruco(self):
        now = self.get_clock().now().nanoseconds / 1e9
        observed = {
            marker_id: point
            for marker_id, point in self.latest_markers.items()
            if now - point[4] <= self.aruco_timeout
        }
        if len(observed) < self.min_aruco_markers:
            return None

        yaw_samples = []
        marker_ids = list(observed.keys())
        for i, marker_a in enumerate(marker_ids):
            for marker_b in marker_ids[i + 1:]:
                local_ax, local_ay = self.marker_layout[marker_a]
                local_bx, local_by = self.marker_layout[marker_b]
                base_ax, base_ay, _, _, _ = observed[marker_a]
                base_bx, base_by, _, _, _ = observed[marker_b]
                local_dx = local_bx - local_ax
                local_dy = local_by - local_ay
                base_dx = base_bx - base_ax
                base_dy = base_by - base_ay
                if math.hypot(local_dx, local_dy) < 0.10:
                    continue
                yaw_samples.append(
                    normalize_angle(math.atan2(base_dy, base_dx) - math.atan2(local_dy, local_dx))
                )

        if yaw_samples:
            trolley_yaw_base = math.atan2(
                sum(math.sin(yaw) for yaw in yaw_samples),
                sum(math.cos(yaw) for yaw in yaw_samples),
            )
        else:
            trolley_yaw_base = next(iter(observed.values()))[3]

        cos_yaw = math.cos(trolley_yaw_base)
        sin_yaw = math.sin(trolley_yaw_base)
        center_x_values = []
        center_y_values = []
        for marker_id, point in observed.items():
            marker_x, marker_y, _, _, _ = point
            local_x, local_y = self.marker_layout[marker_id]
            center_x_values.append(marker_x - (cos_yaw * local_x - sin_yaw * local_y))
            center_y_values.append(marker_y - (sin_yaw * local_x + cos_yaw * local_y))

        return (
            sum(center_x_values) / len(center_x_values),
            sum(center_y_values) / len(center_y_values),
            trolley_yaw_base,
            len(observed),
        )

    def ground_truth_trolley_in_base(self):
        robot_pose = self.call_get_entity_state(self.robot_entity)
        trolley_pose = self.call_get_entity_state(self.trolley_entity)
        if robot_pose is None or trolley_pose is None:
            return None

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = quaternion_to_yaw(robot_pose.orientation)
        dx = trolley_pose.position.x - robot_x
        dy = trolley_pose.position.y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        base_x = cos_yaw * dx + sin_yaw * dy
        base_y = -sin_yaw * dx + cos_yaw * dy
        trolley_yaw = quaternion_to_yaw(trolley_pose.orientation)
        return base_x, base_y, normalize_parallel_angle(trolley_yaw - robot_yaw)

    def min_clamped_cmd(self, error, gain, limit, tolerance):
        if abs(error) <= tolerance:
            return 0.0
        command = clamp(gain * error, limit)
        if abs(command) >= self.fine_min_vxy:
            return command
        return math.copysign(self.fine_min_vxy, command if command != 0.0 else error)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def run_alignment(self, side_only, label, timeout):
        self.get_logger().info(f"Starting phase: {label}")
        start = time.monotonic()
        aligned_since = None
        final_status = "timeout"
        final_estimate = None
        last_log_time = 0.0
        while rclpy.ok():
            estimate = self.estimate_trolley_from_aruco()
            if estimate is None:
                self.stop_robot()
                final_status = "no_markers"
                if time.monotonic() - start > timeout:
                    break
                time.sleep(0.05)
                continue

            center_x, center_y, yaw_error, marker_count = estimate
            yaw_error = normalize_parallel_angle(yaw_error)
            x_error = center_x - (self.entry_offset_x if side_only else self.aruco_target_x)
            y_error = 0.0 if side_only else center_y - self.aruco_target_y

            if side_only:
                aligned = abs(x_error) <= self.side_align_x_tolerance and abs(yaw_error) <= self.side_align_yaw_tolerance
            else:
                aligned = (
                    abs(x_error) <= self.fine_position_tolerance
                    and abs(y_error) <= self.fine_position_tolerance
                    and abs(yaw_error) <= self.fine_yaw_tolerance
                )

            final_estimate = (x_error, y_error, yaw_error, marker_count)
            if aligned:
                self.stop_robot()
                if aligned_since is None:
                    aligned_since = time.monotonic()
                    self.get_logger().info(
                        f"{label}: aligned, holding for {self.align_hold_time:.2f}s "
                        f"(x={x_error:+.3f}, y={y_error:+.3f}, yaw={math.degrees(yaw_error):+.2f} deg)"
                    )
                if time.monotonic() - aligned_since >= self.align_hold_time:
                    final_status = "success"
                    break
                time.sleep(0.05)
                continue

            aligned_since = None
            cmd = Twist()
            cmd.linear.x = self.x_control_sign * self.min_clamped_cmd(
                x_error,
                self.aruco_kxy,
                self.fine_max_vx,
                self.side_align_x_tolerance if side_only else self.fine_position_tolerance,
            )
            if not side_only:
                cmd.linear.y = self.y_control_sign * self.min_clamped_cmd(
                    y_error,
                    self.aruco_kxy,
                    self.fine_max_vy,
                    self.fine_position_tolerance,
                )
            cmd.angular.z = self.yaw_control_sign * clamp(self.aruco_kyaw * yaw_error, self.fine_max_wz)
            self.cmd_pub.publish(cmd)

            if time.monotonic() - last_log_time > 1.0:
                self.get_logger().info(
                    f"{label}: x={x_error:+.3f}, y={y_error:+.3f}, "
                    f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count} | "
                    f"cmd vx={cmd.linear.x:+.3f}, vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
                )
                last_log_time = time.monotonic()

            if time.monotonic() - start > timeout:
                break
            time.sleep(0.1)

        self.stop_robot()
        self.get_logger().info(f"Finished phase: {label} status={final_status}")
        return self.phase_result(label, final_status, start, final_estimate)

    def run_straight_under(self):
        self.get_logger().info("Starting phase: straight_under")
        start = time.monotonic()
        last_direction = -1.0
        final_status = "timeout"
        final_estimate = None
        while rclpy.ok():
            estimate = self.estimate_trolley_from_aruco()
            if estimate is None:
                self.stop_robot()
                final_status = "no_markers"
                if time.monotonic() - start > self.straight_under_timeout:
                    break
                time.sleep(0.05)
                continue

            center_x, center_y, yaw_error, marker_count = estimate
            y_error = center_y - self.aruco_target_y
            yaw_error = normalize_parallel_angle(yaw_error)
            final_estimate = (center_x - self.aruco_target_x, y_error, yaw_error, marker_count)
            if abs(y_error) <= self.straight_under_stop_y:
                self.stop_robot()
                final_status = "success"
                break

            if abs(y_error) > 0.03:
                last_direction = math.copysign(1.0, y_error)

            cmd = Twist()
            cmd.linear.y = self.y_control_sign * last_direction * self.straight_under_speed
            cmd.angular.z = self.yaw_control_sign * clamp(self.aruco_kyaw * yaw_error, self.drive_under_max_wz)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                "straight_under: "
                f"x={center_x - self.aruco_target_x:+.3f}, y={y_error:+.3f}, "
                f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count} | "
                f"cmd vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
            )

            if time.monotonic() - start > self.straight_under_timeout:
                break
            time.sleep(0.1)

        self.stop_robot()
        self.get_logger().info(f"Finished phase: straight_under status={final_status}")
        return self.phase_result("straight_under", final_status, start, final_estimate)

    def phase_result(self, phase, status, start_time, aruco_result):
        gt = self.ground_truth_trolley_in_base()
        if aruco_result is None:
            aruco_x = aruco_y = aruco_yaw = marker_count = None
        else:
            aruco_x, aruco_y, aruco_yaw, marker_count = aruco_result

        if gt is None:
            true_x = true_y = true_yaw = true_x_error = true_y_error = None
        else:
            true_x, true_y, true_yaw = gt
            if phase == "side_pre_entry_align":
                true_x_error = true_x - self.entry_offset_x
                true_y_error = 0.0
            else:
                true_x_error = true_x - self.aruco_target_x
                true_y_error = true_y - self.aruco_target_y

        return {
            "phase": phase,
            "status": status,
            "duration_sec": round(time.monotonic() - start_time, 3),
            "aruco_x_error_m": aruco_x,
            "aruco_y_error_m": aruco_y,
            "aruco_yaw_error_deg": None if aruco_yaw is None else math.degrees(aruco_yaw),
            "marker_count": marker_count,
            "true_center_x_m": true_x,
            "true_center_y_m": true_y,
            "true_yaw_error_deg": None if true_yaw is None else math.degrees(true_yaw),
            "true_x_error_m": true_x_error,
            "true_y_error_m": true_y_error,
            "aruco_minus_true_x_m": None if aruco_x is None or true_x_error is None else aruco_x - true_x_error,
            "aruco_minus_true_y_m": None if aruco_y is None or true_y_error is None else aruco_y - true_y_error,
            "aruco_minus_true_yaw_deg": None
            if aruco_yaw is None or true_yaw is None
            else math.degrees(normalize_parallel_angle(aruco_yaw - true_yaw)),
        }

    def write_csv(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "trial",
            "start_lateral_offset_m",
            "start_yaw_offset_deg",
            "phase",
            "status",
            "duration_sec",
            "aruco_x_error_m",
            "aruco_y_error_m",
            "aruco_yaw_error_deg",
            "marker_count",
            "true_center_x_m",
            "true_center_y_m",
            "true_yaw_error_deg",
            "true_x_error_m",
            "true_y_error_m",
            "aruco_minus_true_x_m",
            "aruco_minus_true_y_m",
            "aruco_minus_true_yaw_deg",
            "side_align_x_tolerance",
            "fine_position_tolerance",
            "fine_yaw_tolerance_deg",
            "straight_under_stop_y",
            "fine_max_vx",
            "fine_max_vy",
            "fine_max_wz",
            "straight_under_speed",
        ]
        with self.output_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f"Wrote {len(self.rows)} rows to {self.output_path}")

    def append_result(self, trial, lateral_offset, yaw_offset, result):
        row = {
            "trial": trial,
            "start_lateral_offset_m": lateral_offset,
            "start_yaw_offset_deg": math.degrees(yaw_offset),
            "side_align_x_tolerance": self.side_align_x_tolerance,
            "fine_position_tolerance": self.fine_position_tolerance,
            "fine_yaw_tolerance_deg": math.degrees(self.fine_yaw_tolerance),
            "straight_under_stop_y": self.straight_under_stop_y,
            "fine_max_vx": self.fine_max_vx,
            "fine_max_vy": self.fine_max_vy,
            "fine_max_wz": self.fine_max_wz,
            "straight_under_speed": self.straight_under_speed,
        }
        row.update(result)
        self.rows.append(row)

    def run_tests(self):
        self.wait_for_services()
        trial = 0
        for lateral_offset in self.lateral_offsets:
            for yaw_offset in self.yaw_offsets:
                trial += 1
                self.get_logger().info(
                    f"Trial {trial}: lateral_offset={lateral_offset:+.3f} m, yaw_offset={math.degrees(yaw_offset):+.1f} deg"
                )
                if not self.reset_trial(lateral_offset, yaw_offset):
                    self.get_logger().error("Failed to reset trial entities.")
                    continue

                if self.skip_side_align_if_already_close and abs(lateral_offset) < 1.0e-4 and abs(yaw_offset) < 1.0e-4:
                    self.get_logger().info("Skipping side_pre_entry_align for exact nominal start pose.")
                    result = self.phase_result("side_pre_entry_align", "skipped_nominal_start", time.monotonic(), None)
                else:
                    result = self.run_alignment(True, "side_pre_entry_align", self.side_align_timeout)
                self.append_result(trial, lateral_offset, yaw_offset, result)
                if result["status"] not in ("success", "skipped_nominal_start"):
                    continue

                result = self.run_straight_under()
                self.append_result(trial, lateral_offset, yaw_offset, result)
                if result["status"] != "success":
                    continue

                result = self.run_alignment(False, "final_center_align", self.final_align_timeout)
                self.append_result(trial, lateral_offset, yaw_offset, result)

        self.write_csv()
        self.get_logger().info("Aruco docking test complete.")


def main():
    rclpy.init()
    node = ArucoDockingTestRunner()
    try:
        node.run_tests()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
