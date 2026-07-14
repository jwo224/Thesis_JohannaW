#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from aruco_interfaces.msg import ArucoMarkers
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # Registers PoseStamped transforms with tf2.


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
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Nav2TrolleyMissionController(Node):
    def __init__(self):
        super().__init__("nav2_trolley_mission_controller")

        self.declare_parameter("command_topic", "/trolley_command")
        self.declare_parameter("mission_step", "step0")
        self.declare_parameter("goal_topic", "/holonomic_goal")
        self.declare_parameter("cancel_topic", "/holonomic_cancel")
        self.declare_parameter("ignore_radius_topic", "/holonomic_lidar_ignore_radius")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("front_aruco_topic", "/front/aruco_markers")
        self.declare_parameter("rear_aruco_topic", "/rear/aruco_markers")
        self.declare_parameter("left_aruco_topic", "/left/aruco_markers")
        self.declare_parameter("right_aruco_topic", "/right/aruco_markers")
        self.declare_parameter("aruco_timeout", 2.0)
        self.declare_parameter("min_aruco_markers", 1)
        self.declare_parameter("aruco_start_min_markers", 3)
        self.declare_parameter("aruco_start_max_abs_y", 1.10)
        self.declare_parameter("direct_close_speed", 0.10)
        self.declare_parameter("direct_close_timeout", 15.0)
        self.declare_parameter("attached_lidar_ignore_radius", 1.25)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("pickup_x", 2.5)
        self.declare_parameter("pickup_y", 0.0)
        self.declare_parameter("pickup_yaw", 0.0)
        self.declare_parameter("trolley_model_name", "Trolley")
        self.declare_parameter("trolley_reset_z", 0.0)
        self.declare_parameter("reset_trolley_after_mission", True)
        self.declare_parameter("dropoff_x", 5.2)
        self.declare_parameter("dropoff_y", 0.0)
        self.declare_parameter("dropoff_yaw", 0.0)
        self.declare_parameter("approach_clearance", 1.35)
        self.declare_parameter("pickup_aruco_clearance", 1.00)
        self.declare_parameter("exit_clearance", 1.05)
        self.declare_parameter("entry_offset_x", 0.015)
        self.declare_parameter("nav_timeout", 90.0)
        self.declare_parameter("local_position_tolerance", 0.025)
        self.declare_parameter("local_yaw_tolerance_deg", 3.0)
        self.declare_parameter("local_max_vx", 0.12)
        self.declare_parameter("local_max_vy", 0.18)
        self.declare_parameter("local_max_wz", 0.45)
        self.declare_parameter("attached_local_max_vx", 0.14)
        self.declare_parameter("attached_local_max_vy", 0.24)
        self.declare_parameter("attached_local_max_wz", 0.45)
        self.declare_parameter("attached_local_timeout", 45.0)
        self.declare_parameter("attached_exit_position_tolerance", 0.10)
        self.declare_parameter("attached_local_yaw_tolerance_deg", 8.0)
        self.declare_parameter("return_local_max_vx", 0.18)
        self.declare_parameter("return_local_max_vy", 0.18)
        self.declare_parameter("return_local_max_wz", 0.45)
        self.declare_parameter("return_local_timeout", 90.0)
        self.declare_parameter("local_kxy", 0.9)
        self.declare_parameter("local_kyaw", 1.8)
        self.declare_parameter("aruco_target_x", 0.0)
        self.declare_parameter("aruco_target_y", 0.0)
        self.declare_parameter("side_align_x_tolerance", 0.008)
        self.declare_parameter("side_align_yaw_tolerance_deg", 1.0)
        self.declare_parameter("fine_position_tolerance", 0.010)
        self.declare_parameter("attach_position_tolerance", 0.018)
        self.declare_parameter("fine_yaw_tolerance_deg", 1.0)
        self.declare_parameter("fine_max_vx", 0.035)
        self.declare_parameter("fine_max_vy", 0.035)
        self.declare_parameter("fine_max_wz", 0.10)
        self.declare_parameter("drive_under_max_vx", 0.05)
        self.declare_parameter("drive_under_max_vy", 0.12)
        self.declare_parameter("drive_under_max_wz", 0.16)
        self.declare_parameter("drive_under_fast_until_y", 0.20)
        self.declare_parameter("straight_under_speed", 0.16)
        self.declare_parameter("straight_under_stop_y", 0.16)
        self.declare_parameter("straight_under_timeout", 20.0)
        self.declare_parameter("fine_min_vxy", 0.012)
        self.declare_parameter("aruco_kxy", 0.75)
        self.declare_parameter("aruco_kyaw", 1.6)
        self.declare_parameter("aruco_align_timeout", 25.0)

        self.command_topic = self.get_parameter("command_topic").value
        self.mission_step = self.get_parameter("mission_step").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.cancel_topic = self.get_parameter("cancel_topic").value
        self.ignore_radius_topic = self.get_parameter("ignore_radius_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.reload_runtime_parameters()

        self.attach_client = self.create_client(Trigger, "/attach_trolley")
        self.detach_client = self.create_client(Trigger, "/detach_trolley")
        self.set_entity_state_client = self.create_client(SetEntityState, "/set_entity_state")
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self.local_costmap_params_client = self.create_client(
            SetParameters, "/local_costmap/local_costmap/set_parameters"
        )
        self.global_costmap_params_client = self.create_client(
            SetParameters, "/global_costmap/global_costmap/set_parameters"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.cancel_pub = self.create_publisher(Empty, self.cancel_topic, 10)
        self.ignore_radius_pub = self.create_publisher(Float32, self.ignore_radius_topic, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        for topic in (
            self.get_parameter("front_aruco_topic").value,
            self.get_parameter("rear_aruco_topic").value,
            self.get_parameter("left_aruco_topic").value,
            self.get_parameter("right_aruco_topic").value,
        ):
            self.create_subscription(ArucoMarkers, topic, self.aruco_callback, 10)

        self.pose_lock = threading.Lock()
        self.current_pose = None
        self.marker_lock = threading.Lock()
        self.underside_marker_ids = set(range(0, 16))
        self.side_marker_ids = set(range(16, 28))
        self.marker_layout = {
            0: (0.3525, 0.1425),
            1: (0.4475, 0.1425),
            2: (0.3525, 0.0475),
            3: (0.4475, 0.0475),
            4: (0.3525, -0.0475),
            5: (0.4475, -0.0475),
            6: (0.3525, -0.1425),
            7: (0.4475, -0.1425),
            8: (-0.4475, 0.1425),
            9: (-0.3525, 0.1425),
            10: (-0.4475, 0.0475),
            11: (-0.3525, 0.0475),
            12: (-0.4475, -0.0475),
            13: (-0.3525, -0.0475),
            14: (-0.4475, -0.1425),
            15: (-0.3525, -0.1425),
            16: (-0.3750, 0.3065),
            17: (-0.2250, 0.3065),
            18: (-0.0750, 0.3065),
            19: (0.0750, 0.3065),
            20: (0.2250, 0.3065),
            21: (0.3750, 0.3065),
            22: (-0.3750, -0.3065),
            23: (-0.2250, -0.3065),
            24: (-0.0750, -0.3065),
            25: (0.0750, -0.3065),
            26: (0.2250, -0.3065),
            27: (0.3750, -0.3065),
        }
        self.latest_markers = {}
        self.accept_aruco = True
        self.last_aruco_msg_log_time = -999.0
        self.last_tf_warn_time = -999.0
        self.last_waiting_log_time = -999.0
        self.active = False

        self.get_logger().info(
            "Trolley mission controller ready. Publish 'trolley_ready' on "
            f"{self.command_topic} to run charging -> pickup -> dropoff -> home via {self.goal_topic}. "
            f"mission_step={self.mission_step}"
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        with self.pose_lock:
            self.current_pose = (
                pose.position.x,
                pose.position.y,
                quaternion_to_yaw(pose.orientation),
            )

    def aruco_callback(self, msg):
        if not self.accept_aruco:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        marker_ids = [int(marker_id) for marker_id in msg.marker_ids]
        if marker_ids and now - self.last_aruco_msg_log_time > 1.0:
            self.get_logger().info(f"ArUco message from {msg.header.frame_id}: ids={marker_ids}")
            self.last_aruco_msg_log_time = now

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
                    timeout=Duration(seconds=0.20),
                )
            except TransformException as exc:
                if now - self.last_tf_warn_time > 1.0:
                    self.get_logger().warn(
                        f"ArUco TF failed: {msg.header.frame_id} -> {self.base_frame}: {exc}"
                    )
                    self.last_tf_warn_time = now
                continue

            with self.marker_lock:
                self.latest_markers[marker_id] = (
                    pose_base.pose.position.x,
                    pose_base.pose.position.y,
                    pose_base.pose.position.z,
                    quaternion_to_yaw(pose_base.pose.orientation),
                    now,
                )

    def command_callback(self, msg):
        command = msg.data.strip().lower().replace(" ", "_")
        if command != "trolley_ready":
            return
        if self.active:
            self.get_logger().warn("Trolley mission already running; ignoring command.")
            return
        self.active = True
        threading.Thread(target=self.run_mission, daemon=True).start()

    def reload_runtime_parameters(self):
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.home_yaw = float(self.get_parameter("home_yaw").value)
        self.pickup_x = float(self.get_parameter("pickup_x").value)
        self.pickup_y = float(self.get_parameter("pickup_y").value)
        self.pickup_yaw = float(self.get_parameter("pickup_yaw").value)
        self.trolley_model_name = self.get_parameter("trolley_model_name").value
        self.trolley_reset_z = float(self.get_parameter("trolley_reset_z").value)
        self.reset_trolley_after_mission = bool(self.get_parameter("reset_trolley_after_mission").value)
        self.dropoff_x = float(self.get_parameter("dropoff_x").value)
        self.dropoff_y = float(self.get_parameter("dropoff_y").value)
        self.dropoff_yaw = float(self.get_parameter("dropoff_yaw").value)
        self.approach_clearance = float(self.get_parameter("approach_clearance").value)
        self.pickup_aruco_clearance = float(self.get_parameter("pickup_aruco_clearance").value)
        self.exit_clearance = float(self.get_parameter("exit_clearance").value)
        self.entry_offset_x = float(self.get_parameter("entry_offset_x").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)
        self.local_position_tolerance = float(self.get_parameter("local_position_tolerance").value)
        self.local_yaw_tolerance = math.radians(float(self.get_parameter("local_yaw_tolerance_deg").value))
        self.local_max_vx = float(self.get_parameter("local_max_vx").value)
        self.local_max_vy = float(self.get_parameter("local_max_vy").value)
        self.local_max_wz = float(self.get_parameter("local_max_wz").value)
        self.attached_local_max_vx = float(self.get_parameter("attached_local_max_vx").value)
        self.attached_local_max_vy = float(self.get_parameter("attached_local_max_vy").value)
        self.attached_local_max_wz = float(self.get_parameter("attached_local_max_wz").value)
        self.attached_local_timeout = float(self.get_parameter("attached_local_timeout").value)
        self.attached_exit_position_tolerance = float(
            self.get_parameter("attached_exit_position_tolerance").value
        )
        self.attached_local_yaw_tolerance = math.radians(
            float(self.get_parameter("attached_local_yaw_tolerance_deg").value)
        )
        self.return_local_max_vx = float(self.get_parameter("return_local_max_vx").value)
        self.return_local_max_vy = float(self.get_parameter("return_local_max_vy").value)
        self.return_local_max_wz = float(self.get_parameter("return_local_max_wz").value)
        self.return_local_timeout = float(self.get_parameter("return_local_timeout").value)
        self.local_kxy = float(self.get_parameter("local_kxy").value)
        self.local_kyaw = float(self.get_parameter("local_kyaw").value)
        self.aruco_target_x = float(self.get_parameter("aruco_target_x").value)
        self.aruco_target_y = float(self.get_parameter("aruco_target_y").value)
        self.aruco_start_min_markers = int(self.get_parameter("aruco_start_min_markers").value)
        self.aruco_start_max_abs_y = float(self.get_parameter("aruco_start_max_abs_y").value)
        self.direct_close_speed = float(self.get_parameter("direct_close_speed").value)
        self.direct_close_timeout = float(self.get_parameter("direct_close_timeout").value)
        self.attached_lidar_ignore_radius = float(self.get_parameter("attached_lidar_ignore_radius").value)
        self.side_align_x_tolerance = float(self.get_parameter("side_align_x_tolerance").value)
        self.side_align_yaw_tolerance = math.radians(
            float(self.get_parameter("side_align_yaw_tolerance_deg").value)
        )
        self.fine_position_tolerance = float(self.get_parameter("fine_position_tolerance").value)
        self.attach_position_tolerance = float(self.get_parameter("attach_position_tolerance").value)
        self.fine_yaw_tolerance = math.radians(float(self.get_parameter("fine_yaw_tolerance_deg").value))
        self.fine_max_vx = float(self.get_parameter("fine_max_vx").value)
        self.fine_max_vy = float(self.get_parameter("fine_max_vy").value)
        self.fine_max_wz = float(self.get_parameter("fine_max_wz").value)
        self.drive_under_max_vx = float(self.get_parameter("drive_under_max_vx").value)
        self.drive_under_max_vy = float(self.get_parameter("drive_under_max_vy").value)
        self.drive_under_max_wz = float(self.get_parameter("drive_under_max_wz").value)
        self.drive_under_fast_until_y = float(self.get_parameter("drive_under_fast_until_y").value)
        self.straight_under_speed = float(self.get_parameter("straight_under_speed").value)
        self.straight_under_stop_y = float(self.get_parameter("straight_under_stop_y").value)
        self.straight_under_timeout = float(self.get_parameter("straight_under_timeout").value)
        self.fine_min_vxy = float(self.get_parameter("fine_min_vxy").value)
        self.aruco_kxy = float(self.get_parameter("aruco_kxy").value)
        self.aruco_kyaw = float(self.get_parameter("aruco_kyaw").value)
        self.aruco_align_timeout = float(self.get_parameter("aruco_align_timeout").value)

    def run_mission(self):
        try:
            self.reload_runtime_parameters()
            if not self.wait_for_systems():
                return
            self.accept_aruco = True

            pickup_staging_y = self.pickup_y + self.approach_clearance
            pickup_aruco_y = self.pickup_y + self.pickup_aruco_clearance
            dropoff_staging_y = self.dropoff_y + self.approach_clearance
            dropoff_exit_y = self.dropoff_y + self.exit_clearance
            return_lane_y = max(dropoff_staging_y, dropoff_exit_y)
            mission_step = self.mission_step

            self.get_logger().info(f"Mission started at {mission_step}.")

            if mission_step == "step0":
                if not self.run_pickup_approach(pickup_staging_y, pickup_aruco_y):
                    return
                if not self.run_docking_sequence():
                    return
                if not self.run_attached_delivery(pickup_staging_y, dropoff_staging_y):
                    return
            elif mission_step == "step1":
                if not self.run_docking_sequence():
                    return
                if not self.run_attached_delivery(pickup_staging_y, dropoff_staging_y):
                    return
            elif mission_step == "step2":
                if not self.align_with_aruco(side_only=False, label="final ArUco center under trolley"):
                    return
                if not self.attach_trolley():
                    return
                if not self.run_attached_delivery(pickup_staging_y, dropoff_staging_y):
                    return
            elif mission_step == "step3":
                if not self.attach_trolley():
                    return
                if not self.run_attached_delivery(pickup_staging_y, dropoff_staging_y):
                    return
            else:
                self.get_logger().error(
                    f"Unknown mission_step '{mission_step}'. Use step0, step1, step2, or step3."
                )
                return

            if not self.run_dropoff_and_home(return_lane_y):
                return

            self.reset_trolley_after_mission = bool(self.get_parameter("reset_trolley_after_mission").value)
            if self.reset_trolley_after_mission:
                if not self.reset_trolley_to_pickup_zone():
                    return
            else:
                self.get_logger().info("Leaving trolley in orange drop-off zone for batch-test validation.")

            if mission_step != "step0":
                self.mission_step = "step0"
                self.get_logger().info("Test mission step consumed. Next trolley_ready will start from step0.")

            self.get_logger().info("Mission complete. Waiting for next trolley_ready command.")
        finally:
            self.set_lidar_ignore_radius(0.0)
            self.stop_robot()
            self.active = False

    def run_pickup_approach(self, pickup_staging_y, pickup_aruco_y):
        self.get_logger().info("Driving from home to pickup approach.")
        if not self.drive_goal_pose(self.home_x, pickup_staging_y, self.pickup_yaw, "pickup lane entry"):
            return False
        if not self.drive_goal_pose(self.pickup_x, pickup_staging_y, self.pickup_yaw, "pickup approach"):
            return False
        self.cancel_holonomic_goal()
        self.clear_markers()
        self.get_logger().info("Driving closer to trolley with direct ArUco-guided cmd_vel.")
        return self.drive_direct_until_aruco_close(pickup_aruco_y)

    def run_docking_sequence(self):
        self.cancel_holonomic_goal()
        self.get_logger().info("Centering on trolley with side-camera ArUco before entry.")
        if not self.align_with_aruco(side_only=True, label="side ArUco pre-entry"):
            return False

        self.get_logger().info("Driving straight under trolley, ignoring costmap and lidar.")
        if not self.drive_straight_under_trolley():
            return False

        self.get_logger().info("Fine-centering under trolley with ArUco before attaching.")
        if not self.align_with_aruco(side_only=False, label="final ArUco center under trolley"):
            return False

        return self.attach_trolley()

    def attach_trolley(self):
        if not self.call_trigger(self.attach_client, "attach trolley"):
            return False
        self.accept_aruco = False
        self.clear_markers()
        self.get_logger().info("Trolley attached. ArUco processing disabled for the rest of this mission.")
        self.set_nav2_obstacle_layers(False)
        self.clear_nav2_costmaps("after trolley attach")
        return True

    def run_attached_delivery(self, pickup_staging_y, dropoff_staging_y):
        self.set_lidar_ignore_radius(self.attached_lidar_ignore_radius)
        self.clear_nav2_costmaps("before attached delivery")
        self.get_logger().info("Trolley attached. Exiting pickup bay straight into travel lane.")
        if not self.local_drive_to(
            self.pickup_x + self.entry_offset_x,
            pickup_staging_y,
            self.pickup_yaw,
            "pickup exit lane",
            timeout_sec=self.attached_local_timeout,
            max_vx=self.attached_local_max_vx,
            max_vy=self.attached_local_max_vy,
            max_wz=self.attached_local_max_wz,
            position_tolerance=self.attached_exit_position_tolerance,
            yaw_tolerance=self.attached_local_yaw_tolerance,
        ):
            return False

        self.get_logger().info("Driving attached trolley along travel lane to orange drop-off.")
        return self.drive_goal_pose(self.dropoff_x, dropoff_staging_y, self.dropoff_yaw, "drop-off approach")

    def run_dropoff_and_home(self, return_lane_y):
        self.cancel_holonomic_goal()
        self.get_logger().info("Entering orange drop-off straight from the travel lane.")
        if not self.local_drive_to(
            self.dropoff_x,
            self.dropoff_y,
            self.dropoff_yaw,
            "drop-off center",
            timeout_sec=self.attached_local_timeout,
            max_vx=self.attached_local_max_vx,
            max_vy=self.attached_local_max_vy,
            max_wz=self.attached_local_max_wz,
            yaw_tolerance=self.attached_local_yaw_tolerance,
        ):
            return False

        if not self.call_trigger(self.detach_client, "detach trolley"):
            return False

        self.set_lidar_ignore_radius(0.0)
        self.set_nav2_obstacle_layers(True)
        self.clear_nav2_costmaps("after trolley detach")
        self.get_logger().info("Returning to charging via the upper travel lane, avoiding all drop zones.")
        if not self.local_drive_to(
            self.dropoff_x,
            return_lane_y,
            self.dropoff_yaw,
            "orange exit to return lane",
            timeout_sec=self.return_local_timeout,
            max_vx=self.return_local_max_vx,
            max_vy=self.return_local_max_vy,
            max_wz=self.return_local_max_wz,
        ):
            return False

        if not self.local_drive_to(
            self.home_x,
            return_lane_y,
            self.home_yaw,
            "return lane above drop zones",
            timeout_sec=self.return_local_timeout,
            max_vx=self.return_local_max_vx,
            max_vy=self.return_local_max_vy,
            max_wz=self.return_local_max_wz,
        ):
            return False
        return self.local_drive_to(
            self.home_x,
            self.home_y,
            self.home_yaw,
            "charging zone",
            timeout_sec=self.return_local_timeout,
            max_vx=self.return_local_max_vx,
            max_vy=self.return_local_max_vy,
            max_wz=self.return_local_max_wz,
        )

    def wait_for_systems(self):
        self.get_logger().info("Waiting for odom and trolley attach services...")
        if not self.attach_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/attach_trolley service is not available.")
            return False
        if not self.detach_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/detach_trolley service is not available.")
            return False
        if not self.set_entity_state_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/set_entity_state service is not available.")
            return False
        start = time.monotonic()
        while rclpy.ok():
            with self.pose_lock:
                if self.current_pose is not None:
                    return True
            if time.monotonic() - start > 10.0:
                self.get_logger().error("No /odom pose received.")
                return False
            time.sleep(0.05)
        return False

    def drive_goal_pose(self, x, y, yaw, label, stop_on_aruco=False):
        self.get_logger().info(
            f"Holonomic goal '{label}': x={x:+.2f}, y={y:+.2f}, yaw={math.degrees(yaw):+.1f} deg"
        )
        self.goal_pub.publish(self.make_pose(x, y, yaw))
        start = time.monotonic()
        while rclpy.ok():
            with self.pose_lock:
                pose = self.current_pose
            if pose is None:
                time.sleep(0.05)
                continue
            current_x, current_y, current_yaw = pose
            distance = math.hypot(x - current_x, y - current_y)
            yaw_error = abs(normalize_angle(yaw - current_yaw))
            if stop_on_aruco:
                estimate = self.estimate_trolley_from_aruco()
                if (
                    estimate is not None
                    and estimate[3] >= self.aruco_start_min_markers
                    and abs(estimate[1]) <= self.aruco_start_max_abs_y
                ):
                    self.get_logger().info(
                        f"Holonomic goal '{label}' stopped early for ArUco alignment: "
                        f"distance={distance:.3f}, aruco_y={estimate[1]:+.3f}, markers={estimate[3]}"
                    )
                    return True

            if distance <= 0.04 and yaw_error <= math.radians(8.0):
                self.get_logger().info(f"Holonomic goal '{label}' reached.")
                return True

            time.sleep(0.1)
            if time.monotonic() - start > self.nav_timeout:
                self.get_logger().error(
                    f"Holonomic goal '{label}' timed out: distance={distance:.3f}, "
                    f"yaw={math.degrees(yaw_error):.1f} deg"
                )
                return False

    def local_drive_to(
        self,
        target_x,
        target_y,
        target_yaw,
        label,
        timeout_sec=20.0,
        max_vx=None,
        max_vy=None,
        max_wz=None,
        position_tolerance=None,
        yaw_tolerance=None,
    ):
        max_vx = self.local_max_vx if max_vx is None else max_vx
        max_vy = self.local_max_vy if max_vy is None else max_vy
        max_wz = self.local_max_wz if max_wz is None else max_wz
        position_tolerance = self.local_position_tolerance if position_tolerance is None else position_tolerance
        yaw_tolerance = self.local_yaw_tolerance if yaw_tolerance is None else yaw_tolerance
        start = time.monotonic()
        while rclpy.ok():
            with self.pose_lock:
                pose = self.current_pose
            if pose is None:
                time.sleep(0.05)
                continue

            x, y, yaw = pose
            error_x = target_x - x
            error_y = target_y - y
            yaw_error = normalize_angle(target_yaw - yaw)
            distance = math.hypot(error_x, error_y)

            if distance <= position_tolerance and abs(yaw_error) <= yaw_tolerance:
                self.stop_robot()
                self.get_logger().info(f"Local target '{label}' reached.")
                return True

            if time.monotonic() - start > timeout_sec:
                self.stop_robot()
                self.get_logger().error(
                    f"Local target '{label}' timed out: distance={distance:.3f}, "
                    f"yaw={math.degrees(yaw_error):+.2f} deg"
                )
                return False

            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            body_x = cos_yaw * error_x + sin_yaw * error_y
            body_y = -sin_yaw * error_x + cos_yaw * error_y

            cmd = Twist()
            cmd.linear.x = clamp(self.local_kxy * body_x, max_vx)
            cmd.linear.y = clamp(self.local_kxy * body_y, max_vy)
            cmd.angular.z = clamp(self.local_kyaw * yaw_error, max_wz)
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)

        return False

    def drive_direct_until_aruco_close(self, fallback_y):
        start = time.monotonic()
        last_direction = -1.0
        while rclpy.ok():
            estimate = self.estimate_trolley_from_aruco(self.side_marker_ids)
            with self.pose_lock:
                pose = self.current_pose

            if estimate is not None:
                center_x, center_y, yaw_error, marker_count = estimate
                y_error = center_y - self.aruco_target_y
                yaw_error = normalize_parallel_angle(yaw_error)
                if (
                    marker_count >= self.aruco_start_min_markers
                    and abs(y_error) <= self.aruco_start_max_abs_y
                ):
                    self.stop_robot()
                    self.get_logger().info(
                        "Direct close approach complete from ArUco: "
                        f"y={y_error:+.3f}, x={center_x:+.3f}, "
                        f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count}"
                    )
                    return True
                if abs(y_error) > 0.03:
                    last_direction = math.copysign(1.0, y_error)

                cmd = Twist()
                cmd.linear.y = last_direction * self.direct_close_speed
                cmd.angular.z = clamp(self.aruco_kyaw * yaw_error, self.drive_under_max_wz)
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    "Direct close approach: "
                    f"y={y_error:+.3f}, x={center_x:+.3f}, "
                    f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count} | "
                    f"cmd vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
                )
            elif pose is not None:
                _x, y, yaw = pose
                y_error = fallback_y - y
                if abs(y_error) <= 0.05:
                    self.stop_robot()
                    self.get_logger().info("Direct close approach reached fallback odom position.")
                    return True
                cmd = Twist()
                cmd.linear.y = math.copysign(self.direct_close_speed, y_error)
                cmd.angular.z = clamp(self.local_kyaw * normalize_angle(self.pickup_yaw - yaw), self.local_max_wz)
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    "Direct close approach: waiting for ArUco, "
                    f"fallback_y_error={y_error:+.3f} | cmd vy={cmd.linear.y:+.3f}"
                )
            else:
                time.sleep(0.05)
                continue

            if time.monotonic() - start > self.direct_close_timeout:
                self.stop_robot()
                self.get_logger().error("Direct close approach timed out before ArUco was close enough.")
                return False
            time.sleep(0.1)

        return False

    def estimate_trolley_from_aruco(self, allowed_marker_ids=None):
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = float(self.get_parameter("aruco_timeout").value)
        min_markers = int(self.get_parameter("min_aruco_markers").value)
        if allowed_marker_ids is None:
            allowed_marker_ids = set(self.marker_layout)

        with self.marker_lock:
            observed = {
                marker_id: point
                for marker_id, point in self.latest_markers.items()
                if (
                    now - point[4] <= timeout
                    and marker_id in self.marker_layout
                    and marker_id in allowed_marker_ids
                )
            }

        if len(observed) < min_markers:
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
                        math.atan2(base_dy, base_dx) - math.atan2(local_dy, local_dx)
                    )
                )

        if yaw_samples:
            sin_sum = sum(math.sin(yaw) for yaw in yaw_samples)
            cos_sum = sum(math.cos(yaw) for yaw in yaw_samples)
            trolley_yaw_base = math.atan2(sin_sum, cos_sum)
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

        center_x = sum(center_x_values) / len(center_x_values)
        center_y = sum(center_y_values) / len(center_y_values)
        return center_x, center_y, trolley_yaw_base, len(observed)

    def align_with_aruco(self, side_only, label):
        start = time.monotonic()
        aligned_since = None
        allowed_marker_ids = self.side_marker_ids if side_only else self.underside_marker_ids
        while rclpy.ok():
            estimate = self.estimate_trolley_from_aruco(allowed_marker_ids)
            now = self.get_clock().now().nanoseconds / 1e9
            if estimate is None:
                self.stop_robot()
                if now - self.last_waiting_log_time > 1.0:
                    with self.marker_lock:
                        marker_ids = sorted(self.latest_markers.keys())
                    self.get_logger().info(
                        f"{label}: waiting for ArUco markers... accepted marker ids={marker_ids}"
                    )
                    self.last_waiting_log_time = now
                if time.monotonic() - start > self.aruco_align_timeout:
                    self.get_logger().error(f"{label} timed out waiting for ArUco markers.")
                    return False
                time.sleep(0.05)
                continue

            center_x, center_y, yaw_error, marker_count = estimate
            yaw_error = normalize_parallel_angle(yaw_error)
            x_error = center_x - (self.entry_offset_x if side_only else self.aruco_target_x)
            y_error = 0.0 if side_only else center_y - self.aruco_target_y
            position_tolerance = self.fine_position_tolerance
            if not side_only and label.startswith("final ArUco"):
                position_tolerance = self.attach_position_tolerance

            if side_only:
                aligned = (
                    abs(x_error) <= self.side_align_x_tolerance
                    and abs(yaw_error) <= self.side_align_yaw_tolerance
                )
            else:
                aligned = (
                    abs(x_error) <= position_tolerance
                    and abs(y_error) <= position_tolerance
                    and abs(yaw_error) <= self.fine_yaw_tolerance
                )

            if aligned:
                self.stop_robot()
                if aligned_since is None:
                    aligned_since = time.monotonic()
                if time.monotonic() - aligned_since >= 0.35:
                    self.get_logger().info(
                        f"{label} complete: x={x_error:+.3f}, y={y_error:+.3f}, "
                        f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count}"
                    )
                    return True
                time.sleep(0.05)
                continue

            aligned_since = None
            max_vx = self.fine_max_vx
            max_vy = self.fine_max_vy
            max_wz = self.fine_max_wz
            if not side_only and abs(y_error) > self.drive_under_fast_until_y:
                max_vx = self.drive_under_max_vx
                max_vy = self.drive_under_max_vy
                max_wz = self.drive_under_max_wz

            cmd = Twist()
            cmd.linear.x = self.min_clamped_cmd(
                x_error,
                self.aruco_kxy,
                max_vx,
                self.side_align_x_tolerance if side_only else self.fine_position_tolerance,
                self.fine_min_vxy,
            )
            if not side_only:
                cmd.linear.y = self.min_clamped_cmd(
                    y_error,
                    self.aruco_kxy,
                    max_vy,
                    self.fine_position_tolerance,
                    self.fine_min_vxy,
                )
            cmd.angular.z = clamp(self.aruco_kyaw * yaw_error, max_wz)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"{label}: x={x_error:+.3f}, y={y_error:+.3f}, "
                f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count} | "
                f"cmd vx={cmd.linear.x:+.3f}, vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
            )

            if time.monotonic() - start > self.aruco_align_timeout:
                self.stop_robot()
                self.get_logger().error(
                    f"{label} timed out: x={x_error:+.3f}, y={y_error:+.3f}, "
                    f"yaw={math.degrees(yaw_error):+.2f} deg"
                )
                return False
            time.sleep(0.1)

        return False

    def drive_straight_under_trolley(self):
        start = time.monotonic()
        last_direction = -1.0
        while rclpy.ok():
            estimate = self.estimate_trolley_from_aruco(self.side_marker_ids)
            now = self.get_clock().now().nanoseconds / 1e9
            if estimate is None:
                with self.pose_lock:
                    pose = self.current_pose
                if pose is not None:
                    _x, y, yaw = pose
                    y_error = y - self.pickup_y
                    if abs(y_error) <= self.straight_under_stop_y:
                        self.stop_robot()
                        self.get_logger().info(
                            "Straight under trolley complete from odom fallback: "
                            f"y={y_error:+.3f}"
                        )
                        return True

                    if abs(y_error) > 0.03:
                        last_direction = -math.copysign(1.0, y_error)

                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.linear.y = last_direction * self.straight_under_speed
                    cmd.angular.z = clamp(
                        self.local_kyaw * normalize_angle(self.pickup_yaw - yaw),
                        self.drive_under_max_wz,
                    )
                    self.cmd_pub.publish(cmd)
                else:
                    self.stop_robot()

                if now - self.last_waiting_log_time > 1.0:
                    with self.marker_lock:
                        marker_ids = sorted(self.latest_markers.keys())
                    self.get_logger().info(
                        "Straight under trolley: side markers not fresh, using odom fallback... "
                        f"accepted marker ids={marker_ids}"
                    )
                    self.last_waiting_log_time = now
                if time.monotonic() - start > self.straight_under_timeout:
                    self.stop_robot()
                    self.get_logger().error("Straight under trolley timed out before reaching trolley center.")
                    return False
                time.sleep(0.1)
                continue

            center_x, center_y, yaw_error, marker_count = estimate
            y_error = center_y - self.aruco_target_y
            yaw_error = normalize_parallel_angle(yaw_error)
            if abs(y_error) <= self.straight_under_stop_y:
                self.stop_robot()
                self.get_logger().info(
                    "Straight under trolley complete: "
                    f"y={y_error:+.3f}, x={center_x:+.3f}, "
                    f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count}"
                )
                return True

            if abs(y_error) > 0.03:
                last_direction = math.copysign(1.0, y_error)

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = last_direction * self.straight_under_speed
            cmd.angular.z = clamp(self.aruco_kyaw * yaw_error, self.drive_under_max_wz)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                "Straight under trolley: "
                f"y={y_error:+.3f}, x={center_x:+.3f}, "
                f"yaw={math.degrees(yaw_error):+.2f} deg, markers={marker_count} | "
                f"cmd vx={cmd.linear.x:+.3f}, vy={cmd.linear.y:+.3f}, wz={cmd.angular.z:+.3f}"
            )

            if time.monotonic() - start > self.straight_under_timeout:
                self.stop_robot()
                self.get_logger().error(
                    "Straight under trolley timed out: "
                    f"y={y_error:+.3f}, x={center_x:+.3f}, "
                    f"yaw={math.degrees(yaw_error):+.2f} deg"
                )
                return False
            time.sleep(0.1)

        return False

    def min_clamped_cmd(self, error, gain, limit, tolerance, minimum):
        if abs(error) <= tolerance:
            return 0.0
        command = clamp(gain * error, limit)
        if abs(command) >= minimum:
            return command
        return math.copysign(minimum, command if command != 0.0 else error)

    def clear_markers(self):
        with self.marker_lock:
            self.latest_markers.clear()

    def call_trigger(self, client, label):
        self.get_logger().info(f"Calling service: {label}.")
        future = client.call_async(Trigger.Request())
        if not self.wait_for_future(future, 10.0):
            self.get_logger().error(f"Service '{label}' timed out.")
            return False
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"Service '{label}' failed: {message}")
            return False
        self.get_logger().info(f"Service '{label}' succeeded: {response.message}")
        return True

    def reset_trolley_to_pickup_zone(self):
        self.get_logger().info("Resetting trolley back onto the blue pickup zone.")
        state = EntityState()
        state.name = self.trolley_model_name
        state.reference_frame = "world"
        state.pose.position.x = self.pickup_x
        state.pose.position.y = self.pickup_y
        state.pose.position.z = self.trolley_reset_z
        qx, qy, qz, qw = yaw_to_quaternion(self.pickup_yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        request = SetEntityState.Request()
        request.state = state
        future = self.set_entity_state_client.call_async(request)
        if not self.wait_for_future(future, 5.0):
            self.get_logger().error("Timed out resetting trolley onto the blue pickup zone.")
            return False

        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.status_message
            self.get_logger().error(f"Failed to reset trolley onto the blue pickup zone: {message}")
            return False

        self.clear_nav2_costmaps("after trolley reset")
        self.get_logger().info("Trolley reset on blue pickup zone. Ready for the next trolley_ready command.")
        return True

    def clear_nav2_costmaps(self, label):
        for client, name in (
            (self.clear_local_costmap_client, "local_costmap"),
            (self.clear_global_costmap_client, "global_costmap"),
        ):
            if not client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warn(f"Cannot clear {name} {label}: service is not available.")
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            if not self.wait_for_future(future, 1.0):
                self.get_logger().warn(f"Timed out clearing {name} {label}.")
                continue
            self.get_logger().info(f"Cleared {name} {label}.")

    def set_nav2_obstacle_layers(self, enabled):
        for client, costmap_name in (
            (self.local_costmap_params_client, "local_costmap"),
            (self.global_costmap_params_client, "global_costmap"),
        ):
            if not client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warn(f"Cannot update {costmap_name} obstacle layer: service is not available.")
                continue

            request = SetParameters.Request()
            request.parameters = [
                self.make_bool_parameter("obstacle_layer.enabled", enabled),
            ]
            future = client.call_async(request)
            if not self.wait_for_future(future, 1.0):
                self.get_logger().warn(f"Timed out updating {costmap_name} obstacle layer.")
                continue

            results = future.result().results if future.result() is not None else []
            if not results or not results[0].successful:
                reason = "" if not results else results[0].reason
                self.get_logger().warn(f"{costmap_name} obstacle layer update failed: {reason}")
                continue
            state = "enabled" if enabled else "disabled"
            self.get_logger().info(f"{costmap_name} obstacle layer {state}.")

    def make_bool_parameter(self, name, value):
        parameter = Parameter()
        parameter.name = name
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = bool(value)
        return parameter

    def wait_for_future(self, future, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.02)
        return future.done()

    def cancel_holonomic_goal(self):
        self.cancel_pub.publish(Empty())
        self.stop_robot()
        time.sleep(0.25)

    def set_lidar_ignore_radius(self, radius):
        msg = Float32()
        msg.data = float(radius)
        self.ignore_radius_pub.publish(msg)
        self.get_logger().info(f"Set holonomic lidar ignore radius to {msg.data:.2f} m.")

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = Nav2TrolleyMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
