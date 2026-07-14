#!/usr/bin/env python3

import math
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from aruco_interfaces.msg import ArucoMarkers
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

import tf2_ros
import tf2_geometry_msgs  # Registers PoseStamped transforms with tf2.


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def set_yaw_on_quaternion(q, yaw):
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


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


class TrolleyReadyDockingController(Node):
    def __init__(self):
        super().__init__("trolley_ready_docking_controller")

        self.declare_parameter("command_topic", "/trolley_command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/trolley_docking_status")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("aruco_topic", "/aruco/markers")
        self.declare_parameter("front_aruco_topic", "/front/aruco_markers")
        self.declare_parameter("rear_aruco_topic", "/rear/aruco_markers")
        self.declare_parameter("left_aruco_topic", "/left/aruco_markers")
        self.declare_parameter("right_aruco_topic", "/right/aruco_markers")
        self.declare_parameter("aruco_timeout", 2.0)
        self.declare_parameter("min_aruco_markers", 1)

        self.declare_parameter("robot_entity", "mecanum_bot")
        self.declare_parameter("trolley_entity", "Trolley")
        self.declare_parameter("target_entity", "drop_zone")
        self.declare_parameter("delivery_entity", "orange_drop_zone")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("return_lane_y", 1.25)

        # Final target point relative to the trolley frame. 0,0 means trolley center.
        self.declare_parameter("target_offset_x", 0.0)
        self.declare_parameter("target_offset_y", 0.0)
        self.declare_parameter("entry_offset_x", 0.015)

        # If the trolley's long side runs along local x, enter from local +/- y.
        self.declare_parameter("entrance_axis", "local_y")
        self.declare_parameter("approach_side", "auto")
        self.declare_parameter("approach_clearance", 0.75)
        self.declare_parameter("delivery_exit_clearance", 0.95)
        self.declare_parameter("look_clearance", 0.95)
        self.declare_parameter("look_pause_time", 1.0)
        self.declare_parameter("staging_tolerance", 0.08)

        self.declare_parameter("coarse_position_tolerance", 0.10)
        self.declare_parameter("coarse_yaw_tolerance_deg", 6.0)
        self.declare_parameter("fine_position_tolerance", 0.015)
        self.declare_parameter("fine_yaw_tolerance_deg", 1.0)
        self.declare_parameter("side_align_x_tolerance", 0.025)
        self.declare_parameter("side_align_yaw_tolerance_deg", 2.0)
        self.declare_parameter("success_hold_time", 0.5)

        self.declare_parameter("kx", 0.80)
        self.declare_parameter("ky", 0.80)
        self.declare_parameter("kyaw", 1.40)

        self.declare_parameter("max_vx", 0.25)
        self.declare_parameter("max_vy", 0.25)
        self.declare_parameter("max_wz", 0.60)
        self.declare_parameter("fine_max_vx", 0.04)
        self.declare_parameter("fine_max_vy", 0.04)
        self.declare_parameter("fine_max_wz", 0.12)
        self.declare_parameter("fine_min_vxy", 0.020)
        self.declare_parameter("side_max_wz", 0.60)

        self.declare_parameter("control_period", 0.1)

        self.get_state_client = self.create_client(GetEntityState, "/get_entity_state")
        self.set_state_client = self.create_client(SetEntityState, "/set_entity_state")
        self.attach_client = self.create_client(Trigger, "/attach_trolley")
        self.detach_client = self.create_client(Trigger, "/detach_trolley")
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        marker_x_positions = [-0.39, -0.26, -0.13, 0.00, 0.13, 0.26, 0.39]
        marker_y_positions = [0.21, 0.07, -0.07, -0.21]
        self.marker_layout = {
            row_index * len(marker_x_positions) + col_index: (x, y)
            for row_index, y in enumerate(marker_y_positions)
            for col_index, x in enumerate(marker_x_positions)
        }
        self.latest_markers: Dict[int, Tuple[float, float, float, float, float]] = {}
        self.drive_under_axis = "local_y"
        self.last_aruco_msg_log_time = -999.0
        self.last_ignored_marker_log_time = -999.0
        self.last_tf_warn_time = -999.0
        self.last_waiting_log_time = -999.0

        self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self.command_callback,
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("aruco_topic").value,
            self.aruco_callback,
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("front_aruco_topic").value,
            self.aruco_callback,
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("rear_aruco_topic").value,
            self.aruco_callback,
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("left_aruco_topic").value,
            self.aruco_callback,
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            self.get_parameter("right_aruco_topic").value,
            self.aruco_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            10,
        )

        self.active = False
        self.stage = "idle"
        self.side_sign = 1.0
        self.aligned_since: Optional[float] = None
        self.pending_robot_future = None
        self.pending_target_future = None
        self.pending_attach_future = None
        self.pending_detach_future = None
        self.trolley_attached = False
        self.attach_offset_x = 0.0
        self.attach_offset_y = 0.0
        self.attach_offset_z = 0.0
        self.attach_offset_yaw = 0.0

        self.create_timer(
            self.get_parameter("control_period").value,
            self.control_loop,
        )

        self.get_logger().info(
            "Trolley docking controller ready. Publish 'trolley_ready' or "
            "'trolley ready' on /trolley_command to drive onto the drop zone."
        )

    def command_callback(self, msg):
        command = msg.data.strip().lower().replace(" ", "_")

        if command != "trolley_ready":
            return

        self.active = True
        self.stage = "rotate_parallel"
        self.side_sign = 1.0
        self.aligned_since = None
        self.trolley_attached = False
        self.pending_attach_future = None
        self.pending_detach_future = None
        self.attach_offset_x = 0.0
        self.attach_offset_y = 0.0
        self.attach_offset_z = 0.0
        self.attach_offset_yaw = 0.0
        self.latest_markers.clear()
        self.publish_status("docking_started")
        self.get_logger().info(
            "Trolley ready command received. Moving to right-camera approach point."
        )

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def min_clamped_cmd(self, error: float, gain: float, limit: float, deadband: float, minimum: float) -> float:
        if abs(error) <= deadband:
            return 0.0

        command = self.clamp(gain * error, limit)
        if abs(command) >= minimum:
            return command

        return math.copysign(minimum, command if command != 0.0 else error)

    def aruco_callback(self, msg: ArucoMarkers):
        now = self.get_clock().now().nanoseconds / 1e9
        marker_ids = [int(marker_id) for marker_id in msg.marker_ids]

        if marker_ids and now - self.last_aruco_msg_log_time > 1.0:
            self.get_logger().info(
                f"ArUco message from {msg.header.frame_id}: ids={marker_ids}"
            )
            self.last_aruco_msg_log_time = now

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            if int(marker_id) not in self.marker_layout:
                if now - self.last_ignored_marker_log_time > 1.0:
                    self.get_logger().warn(
                        f"Ignoring ArUco id {int(marker_id)} because it is not in "
                        f"the trolley marker layout. Expected ids: "
                        f"{min(self.marker_layout)}..{max(self.marker_layout)}"
                    )
                    self.last_ignored_marker_log_time = now
                continue

            pose_cam = PoseStamped()
            pose_cam.header = msg.header
            # Camera-to-base is a fixed robot transform. Using latest transform
            # avoids dropping valid detections because of sim/wall-time mismatch.
            pose_cam.header.stamp = Time().to_msg()
            pose_cam.pose = pose

            try:
                pose_base = self.tf_buffer.transform(
                    pose_cam,
                    self.base_frame,
                    timeout=Duration(seconds=0.20),
                )
            except Exception as e:
                if now - self.last_tf_warn_time > 1.0:
                    self.get_logger().warn(
                        f"ArUco TF failed: {msg.header.frame_id} -> "
                        f"{self.base_frame}: {e}"
                    )
                    self.last_tf_warn_time = now
                continue

            self.latest_markers[int(marker_id)] = (
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z,
                yaw_from_quaternion(pose_base.pose.orientation),
                now,
            )

    def estimate_trolley_from_aruco(self):
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = self.get_parameter("aruco_timeout").value

        observed = {
            marker_id: point
            for marker_id, point in self.latest_markers.items()
            if now - point[4] <= timeout and marker_id in self.marker_layout
        }

        if len(observed) < self.get_parameter("min_aruco_markers").value:
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
                    normalize_angle(
                        math.atan2(base_dy, base_dx)
                        - math.atan2(local_dy, local_dx)
                    )
                )

        if yaw_samples:
            sin_sum = sum(math.sin(yaw) for yaw in yaw_samples)
            cos_sum = sum(math.cos(yaw) for yaw in yaw_samples)
            trolley_yaw_base = math.atan2(sin_sum, cos_sum)
        else:
            # One-marker fallback. The marker frame is aligned with the trolley
            # grid, so use its transformed yaw when no marker pair is visible.
            only_marker = next(iter(observed.values()))
            trolley_yaw_base = only_marker[3]

        cos_yaw = math.cos(trolley_yaw_base)
        sin_yaw = math.sin(trolley_yaw_base)

        center_x_values = []
        center_y_values = []

        for marker_id, point in observed.items():
            marker_x, marker_y, _, _, _ = point
            local_x, local_y = self.marker_layout[marker_id]

            center_x_values.append(marker_x - (cos_yaw * local_x - sin_yaw * local_y))
            center_y_values.append(marker_y - (sin_yaw * local_x + cos_yaw * local_y))

        center_x = sum(center_x_values) / len(center_x_values)
        center_y = sum(center_y_values) / len(center_y_values)

        return center_x, center_y, trolley_yaw_base, len(observed)

    def handle_aruco_fine_alignment(self):
        estimate = self.estimate_trolley_from_aruco()

        if estimate is None:
            self.stop_robot()
            self.publish_status("waiting_for_aruco_alignment_markers")
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_waiting_log_time > 1.0:
                self.get_logger().info(
                    "Fine alignment waiting for ArUco markers... "
                    f"accepted marker ids={sorted(self.latest_markers.keys())}"
                )
                self.last_waiting_log_time = now
            self.aligned_since = None
            return

        center_x, center_y, yaw_error, marker_count = estimate
        yaw_error = normalize_parallel_angle(yaw_error)

        target_offset_x = self.get_parameter("target_offset_x").value
        target_offset_y = self.get_parameter("target_offset_y").value
        x_error = center_x - target_offset_x
        y_error = center_y - target_offset_y

        kx = self.get_parameter("kx").value
        ky = self.get_parameter("ky").value
        kyaw = self.get_parameter("kyaw").value

        max_vx = self.get_parameter("fine_max_vx").value
        max_vy = self.get_parameter("fine_max_vy").value
        max_wz = self.get_parameter("fine_max_wz").value

        fine_position_tolerance = self.get_parameter("fine_position_tolerance").value
        fine_yaw_tolerance = math.radians(
            self.get_parameter("fine_yaw_tolerance_deg").value
        )
        fine_min_vxy = self.get_parameter("fine_min_vxy").value

        cmd = Twist()
        cmd.linear.x = self.min_clamped_cmd(
            x_error,
            kx,
            max_vx,
            fine_position_tolerance,
            fine_min_vxy,
        )
        cmd.linear.y = self.min_clamped_cmd(
            y_error,
            ky,
            max_vy,
            fine_position_tolerance,
            fine_min_vxy,
        )
        cmd.angular.z = self.clamp(kyaw * yaw_error, max_wz)

        aligned = (
            abs(x_error) <= fine_position_tolerance
            and abs(y_error) <= fine_position_tolerance
            and abs(yaw_error) <= fine_yaw_tolerance
        )

        now = self.get_clock().now().nanoseconds / 1e9

        if aligned:
            self.stop_robot()
            self.stage = "attach_trolley"
            self.aligned_since = None
            self.latest_markers.clear()
            self.trolley_attached = False
            self.pending_attach_future = None
            self.publish_status("pins_extending_attach_trolley")
            self.get_logger().info(
                "ArUco fine alignment complete within 1 cm. "
                "Extending pins and attaching trolley with Gazebo fixed joint."
            )

            return

        self.aligned_since = None
        self.cmd_pub.publish(cmd)
        self.publish_status("aruco_fine_aligning")
        self.get_logger().info(
            f"ArUco fine align: x={x_error:+.3f} m, "
            f"y={y_error:+.3f} m, "
            f"yaw={math.degrees(yaw_error):+.2f} deg, "
            f"markers={marker_count} | "
            f"cmd vx={cmd.linear.x:+.3f}, vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
        )

    def handle_side_aruco_alignment(self):
        estimate = self.estimate_trolley_from_aruco()

        if estimate is None:
            self.stop_robot()
            self.publish_status("waiting_for_side_aruco_alignment_markers")
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_waiting_log_time > 1.0:
                self.get_logger().info(
                    "Side alignment waiting for ArUco markers... "
                    f"accepted marker ids={sorted(self.latest_markers.keys())}"
                )
                self.last_waiting_log_time = now
            self.aligned_since = None
            return

        center_x, _, yaw_error, marker_count = estimate
        yaw_error = normalize_parallel_angle(yaw_error)
        entry_offset_x = self.get_parameter("entry_offset_x").value
        x_error = center_x - entry_offset_x

        kx = self.get_parameter("kx").value
        kyaw = self.get_parameter("kyaw").value
        max_vx = self.get_parameter("fine_max_vx").value
        max_wz = self.get_parameter("side_max_wz").value

        cmd = Twist()
        cmd.linear.x = self.clamp(kx * x_error, max_vx)
        cmd.linear.y = 0.0
        cmd.angular.z = self.clamp(kyaw * yaw_error, max_wz)

        x_tolerance = self.get_parameter("side_align_x_tolerance").value
        yaw_tolerance = math.radians(
            self.get_parameter("side_align_yaw_tolerance_deg").value
        )
        aligned = abs(x_error) <= x_tolerance and abs(yaw_error) <= yaw_tolerance
        now = self.get_clock().now().nanoseconds / 1e9

        if aligned:
            if self.aligned_since is None:
                self.aligned_since = now

            self.stop_robot()

            if now - self.aligned_since >= self.get_parameter("success_hold_time").value:
                self.stage = "drive_under"
                self.aligned_since = None
                self.latest_markers.clear()
                self.publish_status("side_aruco_alignment_complete")
                self.get_logger().info(
                    "Side ArUco alignment complete. Driving straight under trolley."
                )

            return

        self.aligned_since = None
        self.cmd_pub.publish(cmd)
        self.publish_status("side_aruco_aligning")
        self.get_logger().info(
            f"Side ArUco align before entry: x={center_x:+.3f} m, "
            f"entry_target_x={entry_offset_x:+.3f} m, "
            f"yaw={math.degrees(yaw_error):+.2f} deg, "
            f"markers={marker_count} | "
            f"cmd vx={cmd.linear.x:+.3f}, vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
        )

    def get_entity_pose_async(self, entity_name: str):
        req = GetEntityState.Request()
        req.name = entity_name
        req.reference_frame = "world"
        return self.get_state_client.call_async(req)

    def target_point_to_world(self, target_x, target_y, target_yaw, local_x, local_y):
        world_x = (
            target_x
            + math.cos(target_yaw) * local_x
            - math.sin(target_yaw) * local_y
        )
        world_y = (
            target_y
            + math.sin(target_yaw) * local_x
            + math.cos(target_yaw) * local_y
        )
        return world_x, world_y

    def world_point_to_target(self, world_x, world_y, target_x, target_y, target_yaw):
        dx = world_x - target_x
        dy = world_y - target_y

        cos_yaw = math.cos(-target_yaw)
        sin_yaw = math.sin(-target_yaw)

        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        return local_x, local_y

    def choose_approach_side(self, robot_pose, target_pose):
        approach_side = self.get_parameter("approach_side").value.lower()

        if approach_side in ("+", "plus", "positive", "left"):
            return 1.0

        if approach_side in ("-", "minus", "negative", "right"):
            return -1.0

        target_yaw = yaw_from_quaternion(target_pose.orientation)
        robot_local_x, robot_local_y = self.world_point_to_target(
            robot_pose.position.x,
            robot_pose.position.y,
            target_pose.position.x,
            target_pose.position.y,
            target_yaw,
        )

        entrance_axis = self.get_parameter("entrance_axis").value.lower()
        entrance_position = robot_local_x if "x" in entrance_axis else robot_local_y
        return 1.0 if entrance_position >= 0.0 else -1.0

    def docking_target(self, robot_pose, target_pose):
        zone_x = target_pose.position.x
        zone_y = target_pose.position.y
        zone_yaw = yaw_from_quaternion(target_pose.orientation)

        center_offset_x = self.get_parameter("target_offset_x").value
        center_offset_y = self.get_parameter("target_offset_y").value
        entry_offset_x = self.get_parameter("entry_offset_x").value
        approach_clearance = self.get_parameter("approach_clearance").value
        entrance_axis = self.get_parameter("entrance_axis").value.lower()

        if self.stage == "look_at_trolley":
            approach_clearance = self.get_parameter("look_clearance").value
        else:
            approach_clearance = self.get_parameter("approach_clearance").value

        if self.stage in ("look_at_trolley", "rotate_parallel", "exit_trolley"):
            if "x" in entrance_axis:
                clearance = (
                    self.get_parameter("delivery_exit_clearance").value
                    if self.stage == "exit_trolley"
                    else approach_clearance
                )
                local_x = center_offset_x + self.side_sign * clearance
                local_y = center_offset_y
            else:
                clearance = (
                    self.get_parameter("delivery_exit_clearance").value
                    if self.stage == "exit_trolley"
                    else approach_clearance
                )
                local_x = center_offset_x if self.stage == "exit_trolley" else entry_offset_x
                local_y = center_offset_y + self.side_sign * clearance
        else:
            local_x = center_offset_x
            local_y = center_offset_y

        target_x, target_y = self.target_point_to_world(
            zone_x,
            zone_y,
            zone_yaw,
            local_x,
            local_y,
        )

        if self.stage == "look_at_trolley":
            if "x" in entrance_axis:
                target_yaw = zone_yaw + math.pi if self.side_sign > 0.0 else zone_yaw
            else:
                target_yaw = zone_yaw - self.side_sign * math.pi / 2.0
        else:
            target_yaw = zone_yaw

        return target_x, target_y, normalize_angle(target_yaw)

    def make_cmd_to_target(self, robot_pose, target_x, target_y, target_yaw):
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)

        dx_world = target_x - robot_x
        dy_world = target_y - robot_y

        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)

        # Planar move expects cmd_vel in the robot/base frame, including sideways y.
        dx_robot = cos_yaw * dx_world - sin_yaw * dy_world
        dy_robot = sin_yaw * dx_world + cos_yaw * dy_world
        yaw_error = normalize_angle(target_yaw - robot_yaw)

        kx = self.get_parameter("kx").value
        ky = self.get_parameter("ky").value
        kyaw = self.get_parameter("kyaw").value

        if self.stage == "fine_align":
            max_vx = self.get_parameter("fine_max_vx").value
            max_vy = self.get_parameter("fine_max_vy").value
            max_wz = self.get_parameter("fine_max_wz").value
        else:
            max_vx = self.get_parameter("max_vx").value
            max_vy = self.get_parameter("max_vy").value
            max_wz = self.get_parameter("max_wz").value

        cmd = Twist()
        cmd.linear.x = self.clamp(kx * dx_robot, max_vx)
        cmd.linear.y = self.clamp(ky * dy_robot, max_vy)
        cmd.angular.z = self.clamp(kyaw * yaw_error, max_wz)

        distance_error = math.sqrt(dx_world**2 + dy_world**2)

        return cmd, distance_error, yaw_error, target_x, target_y

    def make_straight_drive_under_cmd(self, robot_pose, target_pose):
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        target_yaw = yaw_from_quaternion(target_pose.orientation)
        entrance_axis = self.get_parameter("entrance_axis").value.lower()

        robot_local_x, robot_local_y = self.world_point_to_target(
            robot_pose.position.x,
            robot_pose.position.y,
            target_pose.position.x,
            target_pose.position.y,
            target_yaw,
        )

        target_offset_x = self.get_parameter("target_offset_x").value
        target_offset_y = self.get_parameter("target_offset_y").value

        if "x" in entrance_axis:
            axis_error = target_offset_x - robot_local_x
            local_vx = self.clamp(
                self.get_parameter("kx").value * axis_error,
                self.get_parameter("max_vx").value,
            )
            local_vy = 0.0
        else:
            axis_error = target_offset_y - robot_local_y
            local_vx = 0.0
            local_vy = self.clamp(
                self.get_parameter("ky").value * axis_error,
                self.get_parameter("max_vy").value,
            )

        world_vx = math.cos(target_yaw) * local_vx - math.sin(target_yaw) * local_vy
        world_vy = math.sin(target_yaw) * local_vx + math.cos(target_yaw) * local_vy

        cos_robot = math.cos(-robot_yaw)
        sin_robot = math.sin(-robot_yaw)

        cmd = Twist()
        cmd.linear.x = cos_robot * world_vx - sin_robot * world_vy
        cmd.linear.y = sin_robot * world_vx + cos_robot * world_vy

        yaw_error = normalize_angle(target_yaw - robot_yaw)
        cmd.angular.z = self.clamp(
            self.get_parameter("kyaw").value * yaw_error,
            self.get_parameter("max_wz").value,
        )

        return cmd, abs(axis_error), yaw_error

    def capture_trolley_attach_offset(self, robot_pose, trolley_pose):
        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        trolley_yaw = yaw_from_quaternion(trolley_pose.orientation)
        dx_world = trolley_pose.position.x - robot_pose.position.x
        dy_world = trolley_pose.position.y - robot_pose.position.y

        cos_robot = math.cos(-robot_yaw)
        sin_robot = math.sin(-robot_yaw)
        self.attach_offset_x = cos_robot * dx_world - sin_robot * dy_world
        self.attach_offset_y = sin_robot * dx_world + cos_robot * dy_world
        self.attach_offset_z = trolley_pose.position.z - robot_pose.position.z
        self.attach_offset_yaw = normalize_angle(trolley_yaw - robot_yaw)

        self.trolley_attached = True
        self.stage = "deliver_trolley"
        self.publish_status("trolley_attached_delivering")
        self.get_logger().info(
            "Trolley attached with preserved offset: "
            f"x={self.attach_offset_x:+.3f}, "
            f"y={self.attach_offset_y:+.3f}, "
            f"z={self.attach_offset_z:+.3f}, "
            f"yaw={math.degrees(self.attach_offset_yaw):+.2f} deg. "
            "Driving to orange drop zone."
        )

    def set_trolley_pose_from_robot(self, robot_pose):
        if not self.set_state_client.service_is_ready():
            self.get_logger().info("Waiting for /set_entity_state...")
            return

        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        cos_robot = math.cos(robot_yaw)
        sin_robot = math.sin(robot_yaw)

        state = EntityState()
        state.name = self.get_parameter("trolley_entity").value
        state.reference_frame = "world"
        state.pose.position.x = (
            robot_pose.position.x
            + cos_robot * self.attach_offset_x
            - sin_robot * self.attach_offset_y
        )
        state.pose.position.y = (
            robot_pose.position.y
            + sin_robot * self.attach_offset_x
            + cos_robot * self.attach_offset_y
        )
        state.pose.position.z = robot_pose.position.z + self.attach_offset_z
        set_yaw_on_quaternion(
            state.pose.orientation,
            normalize_angle(robot_yaw + self.attach_offset_yaw),
        )

        req = SetEntityState.Request()
        req.state = state
        self.set_state_client.call_async(req)

    def handle_attach_trolley(self):
        self.stop_robot()

        if not self.attach_client.service_is_ready():
            self.publish_status("waiting_for_attach_service")
            self.get_logger().info("Waiting for /attach_trolley...")
            return

        if self.pending_attach_future is None:
            self.pending_attach_future = self.attach_client.call_async(Trigger.Request())
            self.publish_status("attaching_trolley")
            self.get_logger().info("Calling /attach_trolley.")
            return

        if not self.pending_attach_future.done():
            return

        result = self.pending_attach_future.result()
        self.pending_attach_future = None

        if result is None or not result.success:
            self.publish_status("attach_trolley_failed")
            message = result.message if result is not None else "service returned no result"
            self.get_logger().warn(f"Failed to attach trolley: {message}")
            return

        self.stage = "deliver_trolley"
        self.trolley_attached = True
        self.publish_status("trolley_attached_delivering")
        self.get_logger().info(
            f"{result.message} Driving to orange drop zone."
        )

    def handle_detach_trolley(self):
        self.stop_robot()

        if not self.detach_client.service_is_ready():
            self.publish_status("waiting_for_detach_service")
            self.get_logger().info("Waiting for /detach_trolley...")
            return

        if self.pending_detach_future is None:
            self.pending_detach_future = self.detach_client.call_async(Trigger.Request())
            self.publish_status("detaching_trolley")
            self.get_logger().info("Calling /detach_trolley.")
            return

        if not self.pending_detach_future.done():
            return

        result = self.pending_detach_future.result()
        self.pending_detach_future = None

        if result is None or not result.success:
            self.publish_status("detach_trolley_failed")
            message = result.message if result is not None else "service returned no result"
            self.get_logger().warn(f"Failed to detach trolley: {message}")
            return

        self.trolley_attached = False
        self.stage = "exit_trolley"
        self.aligned_since = None
        self.publish_status("trolley_detached_exiting_straight")
        self.get_logger().info(
            f"{result.message} Exiting straight out of trolley before returning home."
        )

    def control_loop(self):
        if not self.active:
            return

        if self.stage == "attach_trolley":
            self.handle_attach_trolley()
            return

        if self.stage == "detach_trolley":
            self.handle_detach_trolley()
            return

        if self.stage == "fine_align":
            self.handle_aruco_fine_alignment()
            return

        if self.stage == "side_aruco_align":
            self.handle_side_aruco_alignment()
            return

        if not self.get_state_client.service_is_ready():
            self.stop_robot()
            self.get_logger().info("Waiting for /get_entity_state...")
            return

        robot_entity = self.get_parameter("robot_entity").value
        if self.stage in ("deliver_trolley", "exit_trolley"):
            target_entity = self.get_parameter("delivery_entity").value
        else:
            target_entity = self.get_parameter("target_entity").value

        if self.pending_robot_future is None or self.pending_target_future is None:
            self.pending_robot_future = self.get_entity_pose_async(robot_entity)
            self.pending_target_future = self.get_entity_pose_async(target_entity)
            self.stop_robot()
            return

        if not self.pending_robot_future.done() or not self.pending_target_future.done():
            self.stop_robot()
            return

        robot_result = self.pending_robot_future.result()
        target_result = self.pending_target_future.result()

        self.pending_robot_future = None
        self.pending_target_future = None

        if robot_result is None or not robot_result.success:
            self.stop_robot()
            self.publish_status("robot_entity_missing")
            self.get_logger().warn(f"Could not find Gazebo entity: {robot_entity}")
            return

        if target_result is None or not target_result.success:
            self.stop_robot()
            self.publish_status("target_entity_missing")
            self.get_logger().warn(f"Could not find Gazebo entity: {target_entity}")
            return

        if self.stage == "look_at_trolley" and self.aligned_since is None:
            self.side_sign = self.choose_approach_side(
                robot_result.state.pose,
                target_result.state.pose,
            )

        if self.stage == "drive_under":
            cmd, distance_error, yaw_error = self.make_straight_drive_under_cmd(
                robot_result.state.pose,
                target_result.state.pose,
            )
            target_x = target_result.state.pose.position.x
            target_y = target_result.state.pose.position.y
        elif self.stage == "return_lane":
            target_x = robot_result.state.pose.position.x
            target_y = self.get_parameter("return_lane_y").value
            target_yaw = self.get_parameter("home_yaw").value
            cmd, distance_error, yaw_error, target_x, target_y = self.make_cmd_to_target(
                robot_result.state.pose,
                target_x,
                target_y,
                target_yaw,
            )
        elif self.stage == "return_lane_home_x":
            target_x = self.get_parameter("home_x").value
            target_y = self.get_parameter("return_lane_y").value
            target_yaw = self.get_parameter("home_yaw").value
            cmd, distance_error, yaw_error, target_x, target_y = self.make_cmd_to_target(
                robot_result.state.pose,
                target_x,
                target_y,
                target_yaw,
            )
        elif self.stage == "return_home":
            target_x = self.get_parameter("home_x").value
            target_y = self.get_parameter("home_y").value
            target_yaw = self.get_parameter("home_yaw").value
            cmd, distance_error, yaw_error, target_x, target_y = self.make_cmd_to_target(
                robot_result.state.pose,
                target_x,
                target_y,
                target_yaw,
            )
        else:
            target_x, target_y, target_yaw = self.docking_target(
                robot_result.state.pose,
                target_result.state.pose,
            )

            cmd, distance_error, yaw_error, target_x, target_y = self.make_cmd_to_target(
                robot_result.state.pose,
                target_x,
                target_y,
                target_yaw,
            )

        coarse_position_tolerance = self.get_parameter("coarse_position_tolerance").value
        coarse_yaw_tolerance = math.radians(
            self.get_parameter("coarse_yaw_tolerance_deg").value
        )
        fine_position_tolerance = self.get_parameter("fine_position_tolerance").value
        fine_yaw_tolerance = math.radians(
            self.get_parameter("fine_yaw_tolerance_deg").value
        )
        staging_tolerance = self.get_parameter("staging_tolerance").value

        if self.stage in ("look_at_trolley", "rotate_parallel", "exit_trolley"):
            current_position_tolerance = staging_tolerance
            current_yaw_tolerance = coarse_yaw_tolerance
        elif self.stage == "fine_align":
            current_position_tolerance = fine_position_tolerance
            current_yaw_tolerance = fine_yaw_tolerance
        else:
            current_position_tolerance = coarse_position_tolerance
            current_yaw_tolerance = coarse_yaw_tolerance

        aligned = (
            distance_error <= current_position_tolerance
            and abs(yaw_error) <= current_yaw_tolerance
        )

        now = self.get_clock().now().nanoseconds / 1e9

        if aligned and self.stage == "look_at_trolley":
            self.stop_robot()
            self.publish_status("camera_look_pause")

            if self.aligned_since is None:
                self.aligned_since = now
                self.get_logger().info(
                    "Camera look point reached. Pausing before rotating parallel."
                )

            if now - self.aligned_since >= self.get_parameter("look_pause_time").value:
                self.stage = "rotate_parallel"
                self.aligned_since = None
                self.get_logger().info(
                    "Pause complete. Rotating parallel so side camera sees the trolley."
                )
            return

        if aligned and self.stage == "rotate_parallel":
            self.stage = "side_aruco_align"
            self.aligned_since = None
            self.latest_markers.clear()
            self.stop_robot()
            self.publish_status("side_camera_starting_aruco_alignment")
            self.get_logger().info(
                "Robot is parallel to drop zone. Starting side-camera ArUco centering."
            )
            return

        if aligned and self.stage == "drive_under":
            self.stage = "fine_align"
            self.aligned_since = None
            self.latest_markers.clear()
            self.stop_robot()
            self.publish_status("under_trolley_starting_fine_alignment")
            self.get_logger().info(
                "Robot is under the trolley. Starting fine center and yaw alignment."
            )
            return

        if aligned and self.stage == "deliver_trolley":
            self.stop_robot()
            self.stage = "detach_trolley"
            self.aligned_since = None
            self.pending_detach_future = None
            self.publish_status("orange_zone_reached_detaching_trolley")
            self.get_logger().info(
                "Orange drop zone reached. Detaching trolley."
            )
            return

        if aligned and self.stage == "exit_trolley":
            self.stop_robot()
            self.stage = "return_lane"
            self.aligned_since = None
            self.publish_status("trolley_exited_moving_to_return_lane")
            self.get_logger().info(
                "Robot exited straight out of trolley. Moving to return lane."
            )
            return

        if aligned and self.stage == "return_lane":
            self.stop_robot()
            self.stage = "return_lane_home_x"
            self.aligned_since = None
            self.publish_status("return_lane_reached_following_lane_home")
            self.get_logger().info(
                "Return lane reached. Driving along lane before returning to origin."
            )
            return

        if aligned and self.stage == "return_lane_home_x":
            self.stop_robot()
            self.stage = "return_home"
            self.aligned_since = None
            self.publish_status("return_lane_home_x_reached_returning_home")
            self.get_logger().info(
                "Clear of drop zones. Moving from return lane to origin."
            )
            return

        if aligned and self.stage == "return_home":
            self.stop_robot()
            self.active = False
            self.stage = "idle"
            self.aligned_since = None
            self.publish_status("ready_for_next_trolley")
            self.get_logger().info(
                "Robot returned to origin. Waiting for the next trolley_ready command."
            )
            return

        if aligned:
            if self.aligned_since is None:
                self.aligned_since = now

            self.stop_robot()

            if now - self.aligned_since >= self.get_parameter("success_hold_time").value:
                self.active = False
                self.stage = "idle"
                self.publish_status("docking_complete")
                self.get_logger().info("Robot is centered on the drop zone. Docking complete.")

            return

        self.aligned_since = None
        self.cmd_pub.publish(cmd)
        self.publish_status("docking")

        self.get_logger().info(
            f"{self.stage} to long-side target ({target_x:+.2f}, {target_y:+.2f}) | "
            f"distance={distance_error:.3f} m, "
            f"yaw={math.degrees(yaw_error):+.2f} deg | "
            f"cmd vx={cmd.linear.x:+.3f}, vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrolleyReadyDockingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping trolley docking controller.")
    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
