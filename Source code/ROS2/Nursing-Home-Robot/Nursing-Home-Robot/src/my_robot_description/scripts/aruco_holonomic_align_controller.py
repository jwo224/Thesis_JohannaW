#!/usr/bin/env python3

import math
import csv
from pathlib import Path
from typing import Optional, Tuple, Dict, Any

import rclpy
from rclpy.node import Node

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import GetEntityState

import tf2_ros
import tf2_geometry_msgs


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


class ArucoHolonomicAlignController(Node):
    def __init__(self):
        super().__init__("aruco_holonomic_align_controller")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("front_topic", "/front/aruco_markers")
        self.declare_parameter("rear_topic", "/rear/aruco_markers")

        # Expected panel locations when robot is centered between panels.
        self.declare_parameter("target_front_x", 0.4875)
        self.declare_parameter("target_rear_x", -0.4875)
        self.declare_parameter("target_y", 0.0)

        # Ground-truth Gazebo entities.
        self.declare_parameter("use_ground_truth", True)
        self.declare_parameter("robot_entity", "mecanum_bot")
        self.declare_parameter("front_panel_entity", "front_aruco_panel")
        self.declare_parameter("rear_panel_entity", "rear_aruco_panel")
        self.declare_parameter("ground_truth_period", 0.2)

        # Keep 2 as requested.
        self.declare_parameter("min_markers_required", 2)

        # Alignment tolerances.
        self.declare_parameter("x_tolerance", 0.02)
        self.declare_parameter("y_tolerance", 0.02)
        self.declare_parameter("yaw_tolerance_deg", 1.0)

        # Controller gains.
        self.declare_parameter("kx", 0.20)
        self.declare_parameter("ky", 0.35)
        self.declare_parameter("kyaw", 0.60)

        # Speed limits.
        self.declare_parameter("max_vx", 0.04)
        self.declare_parameter("max_vy", 0.04)
        self.declare_parameter("max_wz", 0.15)

        # Correct sign settings from your test.
        self.declare_parameter("x_sign", 1.0)
        self.declare_parameter("y_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)

        # Motion enabled by default.
        self.declare_parameter("enable_motion", True)

        # Output files.
        self.declare_parameter(
            "result_dir",
            "/home/group4/Nursing-Home-Robot/aruco_alignment_results",
        )

        self.base_frame = self.get_parameter("base_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.front_panel_base: Optional[Tuple[float, float, float, int]] = None
        self.rear_panel_base: Optional[Tuple[float, float, float, int]] = None

        self.front_sub = self.create_subscription(
            ArucoMarkers,
            self.get_parameter("front_topic").value,
            self.front_callback,
            10,
        )

        self.rear_sub = self.create_subscription(
            ArucoMarkers,
            self.get_parameter("rear_topic").value,
            self.rear_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.error_pub = self.create_publisher(
            Float64MultiArray,
            "/aruco_holonomic_alignment_error",
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.alignment_done = False
        self.final_report_written = False
        self.start_time = self.get_clock().now()

        self.history = []

        # Gazebo ground-truth service.
        self.gt_client = self.create_client(GetEntityState, "/get_entity_state")
        self.pending_gt_futures: Dict[str, Any] = {}
        self.latest_ground_truth: Optional[Dict[str, float]] = None
        self.last_gt_request_time = -999.0

        self.get_logger().info("ArUco holonomic alignment controller started.")
        self.get_logger().info("Motion enabled by default.")
        self.get_logger().info("Ground-truth logging enabled by default.")
        self.get_logger().info("Controller will stop permanently once alignment is reached.")
        self.get_logger().info("Restart this node to run alignment again.")

    def front_callback(self, msg: ArucoMarkers):
        if not self.alignment_done:
            self.front_panel_base = self.average_markers_in_base(msg)

    def rear_callback(self, msg: ArucoMarkers):
        if not self.alignment_done:
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
            except Exception as e:
                self.get_logger().warn(
                    f"TF failed: {msg.header.frame_id} -> {self.base_frame}: {e}"
                )
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

    def clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def request_ground_truth_if_needed(self, elapsed: float):
        if not self.get_parameter("use_ground_truth").value:
            return

        if not self.gt_client.service_is_ready():
            return

        if self.pending_gt_futures:
            return

        period = self.get_parameter("ground_truth_period").value
        if elapsed - self.last_gt_request_time < period:
            return

        self.last_gt_request_time = elapsed

        robot_entity = self.get_parameter("robot_entity").value
        front_entity = self.get_parameter("front_panel_entity").value
        rear_entity = self.get_parameter("rear_panel_entity").value

        entities = {
            "robot": robot_entity,
            "front": front_entity,
            "rear": rear_entity,
        }

        for key, entity_name in entities.items():
            req = GetEntityState.Request()
            req.name = entity_name
            req.reference_frame = "world"
            self.pending_gt_futures[key] = self.gt_client.call_async(req)

    def update_ground_truth_if_ready(self):
        if not self.pending_gt_futures:
            return

        if not all(future.done() for future in self.pending_gt_futures.values()):
            return

        results = {}

        for key, future in self.pending_gt_futures.items():
            result = future.result()

            if result is None or not result.success:
                self.get_logger().warn(f"Ground truth request failed for: {key}")
                self.pending_gt_futures = {}
                return

            results[key] = result.state.pose

        self.pending_gt_futures = {}

        self.latest_ground_truth = self.compute_ground_truth_errors(
            robot_pose=results["robot"],
            front_pose=results["front"],
            rear_pose=results["rear"],
        )

    def world_to_robot_frame(self, world_x, world_y, robot_x, robot_y, robot_yaw):
        dx = world_x - robot_x
        dy = world_y - robot_y

        c = math.cos(-robot_yaw)
        s = math.sin(-robot_yaw)

        x_robot = c * dx - s * dy
        y_robot = s * dx + c * dy

        return x_robot, y_robot

    def compute_ground_truth_errors(self, robot_pose, front_pose, rear_pose) -> Dict[str, float]:
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

        true_front_x_error = front_x_base - target_front_x
        true_rear_x_error = rear_x_base - target_rear_x

        true_distance_error_x = 0.5 * (true_front_x_error + true_rear_x_error)
        true_lateral_error_y = 0.5 * (
            (front_y_base - target_y) + (rear_y_base - target_y)
        )

        true_angle_error_rad = math.atan2(
            front_y_base - rear_y_base,
            expected_panel_distance,
        )

        true_angle_error_deg = math.degrees(true_angle_error_rad)

        true_midpoint_x = 0.5 * (front_x_base + rear_x_base)
        true_midpoint_y = 0.5 * (front_y_base + rear_y_base)

        true_dx = front_x_base - rear_x_base
        true_dy = front_y_base - rear_y_base
        true_panel_distance = math.sqrt(true_dx**2 + true_dy**2)
        true_panel_distance_error = true_panel_distance - expected_panel_distance

        return {
            "true_robot_world_x": robot_x,
            "true_robot_world_y": robot_y,
            "true_robot_world_yaw_rad": robot_yaw,
            "true_robot_world_yaw_deg": math.degrees(robot_yaw),

            "true_front_x": front_x_base,
            "true_front_y": front_y_base,
            "true_rear_x": rear_x_base,
            "true_rear_y": rear_y_base,

            "true_midpoint_x": true_midpoint_x,
            "true_midpoint_y": true_midpoint_y,

            "true_distance_error_x": true_distance_error_x,
            "true_lateral_error_y": true_lateral_error_y,
            "true_angle_error_rad": true_angle_error_rad,
            "true_angle_error_deg": true_angle_error_deg,

            "true_panel_distance": true_panel_distance,
            "true_panel_distance_error": true_panel_distance_error,
        }

    def nan(self):
        return float("nan")

    def gt_value(self, key: str):
        if self.latest_ground_truth is None:
            return self.nan()
        return self.latest_ground_truth.get(key, self.nan())

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        self.request_ground_truth_if_needed(elapsed)
        self.update_ground_truth_if_ready()

        if self.alignment_done:
            self.stop_robot()

            if not self.final_report_written:
                self.write_final_outputs()
                self.final_report_written = True

            return

        if self.front_panel_base is None or self.rear_panel_base is None:
            self.stop_robot()
            self.get_logger().info("Waiting for both front and rear ArUco panel detections...")
            return

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

        midpoint_x = 0.5 * (front_x + rear_x)
        midpoint_y = 0.5 * (front_y + rear_y)

        measured_dx = front_x - rear_x
        measured_dy = front_y - rear_y
        measured_panel_distance = math.sqrt(measured_dx**2 + measured_dy**2)
        panel_distance_error = measured_panel_distance - expected_panel_distance

        true_distance_error_x = self.gt_value("true_distance_error_x")
        true_lateral_error_y = self.gt_value("true_lateral_error_y")
        true_angle_error_rad = self.gt_value("true_angle_error_rad")
        true_angle_error_deg = self.gt_value("true_angle_error_deg")

        if self.latest_ground_truth is not None:
            distance_error_estimation_error = distance_error_x - true_distance_error_x
            lateral_error_estimation_error = lateral_error_y - true_lateral_error_y
            angle_error_estimation_error_rad = normalize_angle(angle_error_rad - true_angle_error_rad)
            angle_error_estimation_error_deg = math.degrees(angle_error_estimation_error_rad)
        else:
            distance_error_estimation_error = self.nan()
            lateral_error_estimation_error = self.nan()
            angle_error_estimation_error_rad = self.nan()
            angle_error_estimation_error_deg = self.nan()

        x_tol = self.get_parameter("x_tolerance").value
        y_tol = self.get_parameter("y_tolerance").value
        yaw_tol = math.radians(self.get_parameter("yaw_tolerance_deg").value)

        kx = self.get_parameter("kx").value
        ky = self.get_parameter("ky").value
        kyaw = self.get_parameter("kyaw").value

        max_vx = self.get_parameter("max_vx").value
        max_vy = self.get_parameter("max_vy").value
        max_wz = self.get_parameter("max_wz").value

        x_sign = self.get_parameter("x_sign").value
        y_sign = self.get_parameter("y_sign").value
        yaw_sign = self.get_parameter("yaw_sign").value

        cmd = Twist()

        if abs(distance_error_x) > x_tol:
            cmd.linear.x = self.clamp(x_sign * kx * distance_error_x, max_vx)

        if abs(lateral_error_y) > y_tol:
            cmd.linear.y = self.clamp(y_sign * ky * lateral_error_y, max_vy)

        if abs(angle_error_rad) > yaw_tol:
            cmd.angular.z = self.clamp(yaw_sign * kyaw * angle_error_rad, max_wz)

        aligned = (
            abs(distance_error_x) <= x_tol
            and abs(lateral_error_y) <= y_tol
            and abs(angle_error_rad) <= yaw_tol
        )

        self.history.append({
            "time": elapsed,

            # ArUco-estimated alignment.
            "aruco_distance_error_x": distance_error_x,
            "aruco_lateral_error_y": lateral_error_y,
            "aruco_angle_error_rad": angle_error_rad,
            "aruco_angle_error_deg": angle_error_deg,
            "aruco_midpoint_x": midpoint_x,
            "aruco_midpoint_y": midpoint_y,
            "aruco_front_x": front_x,
            "aruco_front_y": front_y,
            "aruco_front_z": front_z,
            "aruco_rear_x": rear_x,
            "aruco_rear_y": rear_y,
            "aruco_rear_z": rear_z,
            "aruco_measured_panel_distance": measured_panel_distance,
            "aruco_panel_distance_error": panel_distance_error,

            # Gazebo true alignment.
            "true_distance_error_x": true_distance_error_x,
            "true_lateral_error_y": true_lateral_error_y,
            "true_angle_error_rad": true_angle_error_rad,
            "true_angle_error_deg": true_angle_error_deg,
            "true_midpoint_x": self.gt_value("true_midpoint_x"),
            "true_midpoint_y": self.gt_value("true_midpoint_y"),
            "true_front_x": self.gt_value("true_front_x"),
            "true_front_y": self.gt_value("true_front_y"),
            "true_rear_x": self.gt_value("true_rear_x"),
            "true_rear_y": self.gt_value("true_rear_y"),
            "true_panel_distance": self.gt_value("true_panel_distance"),
            "true_panel_distance_error": self.gt_value("true_panel_distance_error"),
            "true_robot_world_x": self.gt_value("true_robot_world_x"),
            "true_robot_world_y": self.gt_value("true_robot_world_y"),
            "true_robot_world_yaw_deg": self.gt_value("true_robot_world_yaw_deg"),

            # ArUco estimation error compared with Gazebo truth.
            "distance_error_estimation_error": distance_error_estimation_error,
            "lateral_error_estimation_error": lateral_error_estimation_error,
            "angle_error_estimation_error_rad": angle_error_estimation_error_rad,
            "angle_error_estimation_error_deg": angle_error_estimation_error_deg,

            # Detection/control metadata.
            "front_markers": front_n,
            "rear_markers": rear_n,
            "cmd_vx": cmd.linear.x,
            "cmd_vy": cmd.linear.y,
            "cmd_wz": cmd.angular.z,
        })

        error_msg = Float64MultiArray()
        error_msg.data = [
            distance_error_x,
            lateral_error_y,
            angle_error_rad,
            angle_error_deg,
            midpoint_x,
            midpoint_y,
            measured_panel_distance,
            panel_distance_error,
            front_x,
            front_y,
            rear_x,
            rear_y,
            float(front_n),
            float(rear_n),
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,

            true_distance_error_x,
            true_lateral_error_y,
            true_angle_error_rad,
            true_angle_error_deg,
            distance_error_estimation_error,
            lateral_error_estimation_error,
            angle_error_estimation_error_rad,
            angle_error_estimation_error_deg,
        ]
        self.error_pub.publish(error_msg)

        self.get_logger().info(
            f"ArUco errors: x={distance_error_x:+.3f} m, "
            f"y={lateral_error_y:+.3f} m, "
            f"yaw={angle_error_deg:+.2f} deg | "
            f"True: x={true_distance_error_x:+.3f} m, "
            f"y={true_lateral_error_y:+.3f} m, "
            f"yaw={true_angle_error_deg:+.2f} deg | "
            f"Estimation error: dx={distance_error_estimation_error:+.3f} m, "
            f"dy={lateral_error_estimation_error:+.3f} m, "
            f"dyaw={angle_error_estimation_error_deg:+.2f} deg | "
            f"cmd: vx={cmd.linear.x:+.3f}, "
            f"vy={cmd.linear.y:+.3f}, "
            f"wz={cmd.angular.z:+.3f} | "
            f"markers front/rear={front_n}/{rear_n}"
        )

        if aligned:
            self.get_logger().info("Alignment reached. Stopping robot and saving final results.")
            self.stop_robot()
            self.alignment_done = True
            return

        enable_motion = self.get_parameter("enable_motion").value

        if enable_motion:
            self.cmd_pub.publish(cmd)
        else:
            self.stop_robot()

    def write_final_outputs(self):
        if not self.history:
            self.get_logger().warn("No history available. Nothing to save.")
            return

        result_dir = Path(self.get_parameter("result_dir").value)
        result_dir.mkdir(parents=True, exist_ok=True)

        csv_path = result_dir / "aruco_alignment_history.csv"
        plot_path = result_dir / "aruco_alignment_plot.png"
        summary_path = result_dir / "aruco_alignment_final_summary.txt"

        final = self.history[-1]

        with open(csv_path, "w", newline="") as csvfile:
            fieldnames = list(self.history[0].keys())
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.history)

        with open(summary_path, "w") as f:
            f.write("Final ArUco Holonomic Alignment Result\n")
            f.write("=====================================\n\n")
            f.write(f"Final time: {final['time']:.3f} s\n\n")

            f.write("ArUco-estimated final alignment:\n")
            f.write(f"  Distance error x: {final['aruco_distance_error_x']:+.4f} m\n")
            f.write(f"  Lateral error y: {final['aruco_lateral_error_y']:+.4f} m\n")
            f.write(f"  Angle error: {final['aruco_angle_error_deg']:+.3f} deg\n\n")

            f.write("Gazebo true final alignment:\n")
            f.write(f"  True distance error x: {final['true_distance_error_x']:+.4f} m\n")
            f.write(f"  True lateral error y: {final['true_lateral_error_y']:+.4f} m\n")
            f.write(f"  True angle error: {final['true_angle_error_deg']:+.3f} deg\n\n")

            f.write("ArUco estimation error compared with Gazebo truth:\n")
            f.write(f"  Distance estimation error: {final['distance_error_estimation_error']:+.4f} m\n")
            f.write(f"  Lateral estimation error: {final['lateral_error_estimation_error']:+.4f} m\n")
            f.write(f"  Angle estimation error: {final['angle_error_estimation_error_deg']:+.3f} deg\n\n")

            f.write("Additional data:\n")
            f.write(f"  ArUco panel distance: {final['aruco_measured_panel_distance']:.4f} m\n")
            f.write(f"  True panel distance: {final['true_panel_distance']:.4f} m\n")
            f.write(f"  Front markers: {int(final['front_markers'])}\n")
            f.write(f"  Rear markers: {int(final['rear_markers'])}\n")

        self.create_plot(plot_path)

        self.get_logger().info(
            "\n"
            "================ FINAL ALIGNMENT RESULT ================\n"
            "ArUco-estimated final error:\n"
            f"  x:   {final['aruco_distance_error_x']:+.4f} m\n"
            f"  y:   {final['aruco_lateral_error_y']:+.4f} m\n"
            f"  yaw: {final['aruco_angle_error_deg']:+.3f} deg\n"
            "\n"
            "Gazebo true final error:\n"
            f"  x:   {final['true_distance_error_x']:+.4f} m\n"
            f"  y:   {final['true_lateral_error_y']:+.4f} m\n"
            f"  yaw: {final['true_angle_error_deg']:+.3f} deg\n"
            "\n"
            "ArUco estimation error:\n"
            f"  x:   {final['distance_error_estimation_error']:+.4f} m\n"
            f"  y:   {final['lateral_error_estimation_error']:+.4f} m\n"
            f"  yaw: {final['angle_error_estimation_error_deg']:+.3f} deg\n"
            "\n"
            f"CSV saved to:     {csv_path}\n"
            f"Plot saved to:    {plot_path}\n"
            f"Summary saved to: {summary_path}\n"
            "\n"
            "Controller is now stopped. Restart the node to align again.\n"
            "========================================================"
        )

    def create_plot(self, plot_path: Path):
        try:
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(f"Could not create plot because matplotlib is unavailable: {e}")
            return

        t = [row["time"] for row in self.history]

        aruco_x = [row["aruco_distance_error_x"] for row in self.history]
        true_x = [row["true_distance_error_x"] for row in self.history]
        diff_x = [row["distance_error_estimation_error"] for row in self.history]

        aruco_y = [row["aruco_lateral_error_y"] for row in self.history]
        true_y = [row["true_lateral_error_y"] for row in self.history]
        diff_y = [row["lateral_error_estimation_error"] for row in self.history]

        aruco_yaw = [row["aruco_angle_error_deg"] for row in self.history]
        true_yaw = [row["true_angle_error_deg"] for row in self.history]
        diff_yaw = [row["angle_error_estimation_error_deg"] for row in self.history]

        plt.figure(figsize=(12, 8))

        plt.plot(t, aruco_x, label="ArUco distance error x [m]")
        plt.plot(t, true_x, label="True distance error x [m]")
        plt.plot(t, diff_x, label="ArUco - true x [m]")

        plt.plot(t, aruco_y, label="ArUco lateral error y [m]")
        plt.plot(t, true_y, label="True lateral error y [m]")
        plt.plot(t, diff_y, label="ArUco - true y [m]")

        plt.plot(t, aruco_yaw, label="ArUco angle error [deg]")
        plt.plot(t, true_yaw, label="True angle error [deg]")
        plt.plot(t, diff_yaw, label="ArUco - true angle [deg]")

        plt.xlabel("Time [s]")
        plt.ylabel("Error")
        plt.title("ArUco Alignment Estimate vs Gazebo Ground Truth")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        plt.close()

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoHolonomicAlignController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Stopping robot.")
    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()