#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

import tf2_ros
import tf2_geometry_msgs


class ArucoPanelPositionMonitor(Node):
    def __init__(self):
        super().__init__("aruco_panel_position_monitor")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("front_topic", "/front/aruco_markers")
        self.declare_parameter("rear_topic", "/rear/aruco_markers")

        self.declare_parameter("target_front_x", 0.4875)
        self.declare_parameter("target_rear_x", -0.4875)
        self.declare_parameter("target_y", 0.0)

        self.declare_parameter("min_markers_required", 2)
        self.declare_parameter("print_rate_hz", 1.0)

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

        self.error_pub = self.create_publisher(
            Float64MultiArray,
            "/aruco_panel_alignment_error",
            10,
        )

        print_rate_hz = self.get_parameter("print_rate_hz").value
        self.timer = self.create_timer(1.0 / print_rate_hz, self.print_status)

        self.get_logger().info("ArUco panel position monitor started.")
        self.get_logger().info("This node only estimates errors. It does not move the robot.")

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

    def print_status(self):
        if self.front_panel_base is None or self.rear_panel_base is None:
            self.get_logger().info(
                "Waiting for both front and rear panel detections..."
            )
            return

        front_x, front_y, front_z, front_n = self.front_panel_base
        rear_x, rear_y, rear_z, rear_n = self.rear_panel_base

        target_front_x = self.get_parameter("target_front_x").value
        target_rear_x = self.get_parameter("target_rear_x").value
        target_y = self.get_parameter("target_y").value

        expected_panel_distance = abs(target_front_x - target_rear_x)

        # Forward/backward distance error:
        # positive means the average panel positions are shifted forward in base_link
        front_x_error = front_x - target_front_x
        rear_x_error = rear_x - target_rear_x
        distance_error_x = 0.5 * (front_x_error + rear_x_error)

        # Sideways/lateral error:
        # positive means the panels are to the left of the robot in base_link
        lateral_error_y = 0.5 * ((front_y - target_y) + (rear_y - target_y))

        # Angle/yaw error:
        # if front and rear y are different, robot is rotated relative to the trolley/panel line
        angle_error_rad = math.atan2(front_y - rear_y, expected_panel_distance)
        angle_error_deg = math.degrees(angle_error_rad)

        # Current measured distance between panels in base_link
        measured_dx = front_x - rear_x
        measured_dy = front_y - rear_y
        measured_panel_distance = math.sqrt(measured_dx**2 + measured_dy**2)

        panel_distance_error = measured_panel_distance - expected_panel_distance

        # Robot center relative to the panel midpoint.
        # In ideal alignment, midpoint should be x=0, y=0 in base_link.
        midpoint_x = 0.5 * (front_x + rear_x)
        midpoint_y = 0.5 * (front_y + rear_y)
        if abs(distance_error_x) < 0.02:
            x_action = "x OK"
        elif distance_error_x > 0:
            x_action = "move backward / reduce +x"
        else:
            x_action = "move forward / increase +x"

        if abs(lateral_error_y) < 0.02:
            y_action = "y OK"
        elif lateral_error_y > 0:
            y_action = "move right / reduce +y"
        else:
            y_action = "move left / increase +y"

        if abs(angle_error_deg) < 2.0:
            yaw_action = "yaw OK"
        elif angle_error_deg > 0:
            yaw_action = "rotate clockwise"
        else:
            yaw_action = "rotate counter-clockwise"

        

        msg = Float64MultiArray()
        msg.data = [
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
        ]
        self.error_pub.publish(msg)

        self.get_logger().info(
            "\n"
            "================ ArUco Panel Alignment ================\n"
            f"Front panel in base_link: x={front_x:+.3f}, y={front_y:+.3f}, z={front_z:+.3f}, markers={front_n}\n"
            f"Rear  panel in base_link: x={rear_x:+.3f}, y={rear_y:+.3f}, z={rear_z:+.3f}, markers={rear_n}\n"
            "\n"
            f"Midpoint in base_link:    x={midpoint_x:+.3f}, y={midpoint_y:+.3f}\n"
            f"Distance error x:         {distance_error_x:+.3f} m\n"
            f"Lateral error y:          {lateral_error_y:+.3f} m\n"
            f"Angle error:              {angle_error_rad:+.3f} rad / {angle_error_deg:+.2f} deg\n"
            f"Measured panel distance:  {measured_panel_distance:.3f} m\n"
            f"Panel distance error:     {panel_distance_error:+.3f} m\n"
            f"Suggested correction:    {x_action}, {y_action}, {yaw_action}\n"
            "======================================================="
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPanelPositionMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

