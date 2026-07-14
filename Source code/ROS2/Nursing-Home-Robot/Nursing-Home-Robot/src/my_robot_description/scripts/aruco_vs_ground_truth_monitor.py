#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from gazebo_msgs.srv import GetEntityState


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


class ArucoVsGroundTruthMonitor(Node):
    def __init__(self):
        super().__init__("aruco_vs_ground_truth_monitor")

        self.declare_parameter("robot_entity", "mecanum_bot")
        self.declare_parameter("front_panel_entity", "front_aruco_panel")
        self.declare_parameter("rear_panel_entity", "rear_aruco_panel")

        self.declare_parameter("target_front_x", 0.4875)
        self.declare_parameter("target_rear_x", -0.4875)
        self.declare_parameter("target_y", 0.0)

        self.declare_parameter("print_rate_hz", 1.0)

        self.latest_aruco_errors: Optional[Tuple[float, float, float, float]] = None

        self.error_sub = self.create_subscription(
            Float64MultiArray,
            "/aruco_holonomic_alignment_error",
            self.aruco_error_callback,
            10,
        )

        self.gt_error_pub = self.create_publisher(
            Float64MultiArray,
            "/ground_truth_alignment_error",
            10,
        )

        self.comparison_pub = self.create_publisher(
            Float64MultiArray,
            "/aruco_vs_ground_truth_error",
            10,
        )

        self.get_state_client = self.create_client(
            GetEntityState,
            "/get_entity_state",
        )

        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_entity_state service...")

        rate = self.get_parameter("print_rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info("ArUco vs Gazebo ground truth monitor started.")

    def aruco_error_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 4:
            return

        self.latest_aruco_errors = (
            msg.data[0],
            msg.data[1],
            msg.data[2],
            msg.data[3],
        )

    def get_entity_pose(self, name: str):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = "world"

        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if not future.done():
            self.get_logger().warn(f"Timeout getting entity state: {name}")
            return None

        result = future.result()

        if result is None or not result.success:
            self.get_logger().warn(f"Could not get entity state for: {name}")
            return None

        return result.state.pose

    def world_to_robot_frame(self, world_x, world_y, robot_x, robot_y, robot_yaw):
        dx = world_x - robot_x
        dy = world_y - robot_y

        c = math.cos(-robot_yaw)
        s = math.sin(-robot_yaw)

        x_robot = c * dx - s * dy
        y_robot = s * dx + c * dy

        return x_robot, y_robot

    def timer_callback(self):
        if self.latest_aruco_errors is None:
            self.get_logger().info("Waiting for /aruco_holonomic_alignment_error...")
            return

        robot_name = self.get_parameter("robot_entity").value
        front_name = self.get_parameter("front_panel_entity").value
        rear_name = self.get_parameter("rear_panel_entity").value

        robot_pose = self.get_entity_pose(robot_name)
        front_pose = self.get_entity_pose(front_name)
        rear_pose = self.get_entity_pose(rear_name)

        if robot_pose is None or front_pose is None or rear_pose is None:
            return

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

        aruco_distance_error_x, aruco_lateral_error_y, aruco_angle_error_rad, aruco_angle_error_deg = (
            self.latest_aruco_errors
        )

        distance_diff = aruco_distance_error_x - true_distance_error_x
        lateral_diff = aruco_lateral_error_y - true_lateral_error_y
        angle_diff_rad = normalize_angle(aruco_angle_error_rad - true_angle_error_rad)
        angle_diff_deg = math.degrees(angle_diff_rad)

        gt_msg = Float64MultiArray()
        gt_msg.data = [
            true_distance_error_x,
            true_lateral_error_y,
            true_angle_error_rad,
            true_angle_error_deg,
            front_x_base,
            front_y_base,
            rear_x_base,
            rear_y_base,
        ]
        self.gt_error_pub.publish(gt_msg)

        comparison_msg = Float64MultiArray()
        comparison_msg.data = [
            distance_diff,
            lateral_diff,
            angle_diff_rad,
            angle_diff_deg,
            aruco_distance_error_x,
            true_distance_error_x,
            aruco_lateral_error_y,
            true_lateral_error_y,
            aruco_angle_error_deg,
            true_angle_error_deg,
        ]
        self.comparison_pub.publish(comparison_msg)

        self.get_logger().info(
            "\n"
            "================ ArUco vs Ground Truth ================\n"
            f"ArUco distance error x:  {aruco_distance_error_x:+.4f} m\n"
            f"True  distance error x:  {true_distance_error_x:+.4f} m\n"
            f"Difference:              {distance_diff:+.4f} m\n"
            "\n"
            f"ArUco lateral error y:   {aruco_lateral_error_y:+.4f} m\n"
            f"True  lateral error y:   {true_lateral_error_y:+.4f} m\n"
            f"Difference:              {lateral_diff:+.4f} m\n"
            "\n"
            f"ArUco angle error:       {aruco_angle_error_deg:+.3f} deg\n"
            f"True  angle error:       {true_angle_error_deg:+.3f} deg\n"
            f"Difference:              {angle_diff_deg:+.3f} deg\n"
            "\n"
            f"GT front in base:        x={front_x_base:+.3f}, y={front_y_base:+.3f}\n"
            f"GT rear  in base:        x={rear_x_base:+.3f}, y={rear_y_base:+.3f}\n"
            "======================================================="
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoVsGroundTruthMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
