#!/usr/bin/env python3

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Set, Tuple

import rclpy
from aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401


DEFAULT_MARKER_LAYOUT = {
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


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_parallel_angle(angle: float) -> float:
    angle = normalize_angle(angle)
    if angle > math.pi / 2.0:
        angle -= math.pi
    elif angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def trolley_orientation_label(yaw: float) -> str:
    return "flipped_180" if math.cos(yaw) < 0.0 else "normal"


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, value))


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def std(values: List[float]) -> float:
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / (len(values) - 1))


def circular_mean(angles: List[float]) -> float:
    if not angles:
        return 0.0
    return math.atan2(
        sum(math.sin(angle) for angle in angles),
        sum(math.cos(angle) for angle in angles),
    )


def circular_std_about(angles: List[float], center: float) -> float:
    if len(angles) < 2:
        return 0.0
    return std([normalize_angle(angle - center) for angle in angles])


@dataclass
class Observation:
    x: float
    y: float
    z: float
    yaw: float
    stamp_sec: float
    frame_id: str
    camera: str


@dataclass
class Estimate:
    center_x: float
    center_y: float
    yaw: float
    used_ids: Tuple[int, ...]
    cameras: Tuple[str, ...]
    front_count: int
    rear_count: int
    stamp_sec: float


class PhysicalTrolleyAlignmentController(Node):
    def __init__(self):
        super().__init__("physical_trolley_alignment_controller")

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("drive_under_extra_cmd_vel_topic", "")
        self.declare_parameter("command_topic", "/physical_docking_command")
        self.declare_parameter("status_topic", "/physical_docking_status")
        self.declare_parameter("camera_pair_command_topic", "/physical_camera_pair_command")
        self.declare_parameter("camera_pair_republish_sec", 0.5)
        self.declare_parameter("lidar_ignore_radius_topic", "/holonomic_lidar_ignore_radius")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")

        self.declare_parameter("front_aruco_topic", "/front/aruco_markers")
        self.declare_parameter("rear_aruco_topic", "/rear/aruco_markers")
        self.declare_parameter("left_aruco_topic", "/left/aruco_markers")
        self.declare_parameter("right_aruco_topic", "/right/aruco_markers")

        self.declare_parameter("aruco_timeout", 1.0)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("side_align_timeout", 45.0)
        self.declare_parameter("drive_under_timeout", 8.0)
        self.declare_parameter("fine_align_timeout", 45.0)
        self.declare_parameter("fine_debug_period_sec", 1.0)
        self.declare_parameter("camera_switch_settle_sec", 1.5)

        self.declare_parameter("min_side_markers", 2)
        self.declare_parameter("min_under_markers", 2)
        self.declare_parameter("min_front_under_markers", 1)
        self.declare_parameter("min_rear_under_markers", 1)
        self.declare_parameter("under_marker_ids", "")
        self.declare_parameter("side_marker_ids", "")
        self.declare_parameter("side_target_pair_ids", "18,19;24,25")
        self.declare_parameter("fine_preferred_pair_ids", "5,10;3,12")
        self.declare_parameter("fine_require_preferred_pair", True)

        self.declare_parameter("estimate_window_size", 8)
        self.declare_parameter("fine_estimate_window_size", 3)
        self.declare_parameter("estimate_max_std_xy", 0.025)
        self.declare_parameter("estimate_max_std_yaw_deg", 5.0)
        self.declare_parameter("estimate_jump_reject_m", 0.08)
        self.declare_parameter("max_marker_set_change", 2)
        self.declare_parameter("max_reasonable_error_xy", 0.30)
        self.declare_parameter("side_max_reasonable_error_xy", 0.60)

        self.declare_parameter("entry_offset_x", 0.015)
        self.declare_parameter("target_offset_x", 0.1378)
        self.declare_parameter("target_offset_y", 0.0038)
        self.declare_parameter("use_orientation_target_offsets", False)
        self.declare_parameter("target_offset_x_normal", 0.1378)
        self.declare_parameter("target_offset_y_normal", 0.0038)
        self.declare_parameter("target_offset_x_flipped", 0.1378)
        self.declare_parameter("target_offset_y_flipped", 0.0038)
        self.declare_parameter("side_target_offset_x", 0.32)
        self.declare_parameter("side_target_pair_offsets_x", "")
        self.declare_parameter("fine_require_opposite_y_pair", True)
        self.declare_parameter("fine_min_pair_local_y_separation", 0.08)
        self.declare_parameter("fine_use_all_under_markers", False)
        self.declare_parameter("side_require_target_pair_visible", True)
        self.declare_parameter("side_align_x_tolerance", 0.020)
        self.declare_parameter("side_align_yaw_tolerance_deg", 3.0)
        self.declare_parameter("fine_position_tolerance", 0.025)
        self.declare_parameter("fine_position_tolerance_x", -1.0)
        self.declare_parameter("fine_position_tolerance_y", -1.0)
        self.declare_parameter("fine_target_yaw_deg", 0.0)
        self.declare_parameter("fine_target_yaw_deg_normal", 0.0)
        self.declare_parameter("fine_target_yaw_deg_flipped", 0.0)
        self.declare_parameter("fine_yaw_tolerance_deg", 4.0)
        self.declare_parameter("fine_sequential_alignment", True)
        self.declare_parameter("fine_control_mode", "continuous")
        self.declare_parameter("fine_step_settle_sec", 0.35)
        self.declare_parameter("fine_step_min_pulse_sec", 0.12)
        self.declare_parameter("fine_step_max_pulse_sec", 0.35)
        self.declare_parameter("fine_step_error_fraction", 0.45)
        self.declare_parameter("fine_auto_flip_bad_xy_step", True)
        self.declare_parameter("fine_bad_step_error_margin", 0.002)
        self.declare_parameter("success_hold_time", 0.35)

        self.declare_parameter("side_max_vx", 0.0)
        self.declare_parameter("side_min_vx", 0.0)
        self.declare_parameter("side_max_wz", 0.0)
        self.declare_parameter("right_side_vx_sign", 1.0)
        self.declare_parameter("right_side_wz_sign", 1.0)
        self.declare_parameter("fine_max_vx", 0.0)
        self.declare_parameter("fine_max_vy", 0.0)
        self.declare_parameter("fine_max_wz", 0.0)
        self.declare_parameter("fine_min_wz", 0.0)
        self.declare_parameter("fine_min_vxy", 0.0)
        self.declare_parameter("fine_min_vx", -1.0)
        self.declare_parameter("fine_min_vy", -1.0)
        self.declare_parameter("fine_acquire_enable", False)
        self.declare_parameter("fine_acquire_vx", 0.0)
        self.declare_parameter("fine_acquire_vy", 0.0)
        self.declare_parameter("fine_acquire_wz", 0.0)
        self.declare_parameter("fine_acquire_front_seen_vx", 0.012)
        self.declare_parameter("fine_acquire_rear_seen_vx", -0.012)
        self.declare_parameter("fine_acquire_min_markers", 1)
        self.declare_parameter("fine_acquire_timeout", 5.0)
        self.declare_parameter("fine_reacquire_after_pair_loss", True)
        self.declare_parameter("aruco_kxy", 0.50)
        self.declare_parameter("aruco_kyaw", 0.50)
        self.declare_parameter("straight_under_speed", 0.0)
        self.declare_parameter("straight_under_stop_y", 0.16)
        self.declare_parameter("straight_under_duration_sec", 0.0)
        self.declare_parameter("straight_under_distance_m", 0.0)
        self.declare_parameter("straight_under_x_hold_kp", 0.0)
        self.declare_parameter("straight_under_max_vx_correction", 0.0)
        self.declare_parameter("straight_under_vx_bias", 0.0)
        self.declare_parameter("straight_under_wz_bias", 0.0)
        self.declare_parameter("drive_under_lidar_ignore_radius", 0.0)
        self.declare_parameter("drive_under_stop_on_under_markers", True)
        self.declare_parameter("drive_under_marker_stop_hold_sec", 0.0)
        self.declare_parameter("drive_under_marker_stop_min_progress", 0.20)

        self.declare_parameter("side_vx_sign", 1.0)
        self.declare_parameter("side_wz_sign", 1.0)
        self.declare_parameter("under_vx_sign", -1.0)
        self.declare_parameter("under_vy_sign", 1.0)
        self.declare_parameter("under_wz_sign", 1.0)
        self.declare_parameter("straight_under_vy_sign", 1.0)

        # JSON object, e.g. {"0": [0.3525, 0.1425], "1": [0.4475, 0.1425]}.
        self.declare_parameter("marker_layout_json", "")

        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.drive_under_extra_cmd_vel_topic = str(
            self.get_parameter("drive_under_extra_cmd_vel_topic").value
        )
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.camera_pair_command_topic = str(self.get_parameter("camera_pair_command_topic").value)
        self.camera_pair_republish_sec = float(
            self.get_parameter("camera_pair_republish_sec").value
        )
        self.lidar_ignore_radius_topic = str(
            self.get_parameter("lidar_ignore_radius_topic").value
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)

        self.aruco_timeout = float(self.get_parameter("aruco_timeout").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.side_align_timeout = float(self.get_parameter("side_align_timeout").value)
        self.drive_under_timeout = float(self.get_parameter("drive_under_timeout").value)
        self.fine_align_timeout = float(self.get_parameter("fine_align_timeout").value)
        self.fine_debug_period_sec = float(
            self.get_parameter("fine_debug_period_sec").value
        )
        self.camera_switch_settle_sec = float(self.get_parameter("camera_switch_settle_sec").value)

        self.min_side_markers = int(self.get_parameter("min_side_markers").value)
        self.min_under_markers = int(self.get_parameter("min_under_markers").value)
        self.min_front_under_markers = int(self.get_parameter("min_front_under_markers").value)
        self.min_rear_under_markers = int(self.get_parameter("min_rear_under_markers").value)

        self.estimate_window_size = int(self.get_parameter("estimate_window_size").value)
        self.fine_estimate_window_size = int(
            self.get_parameter("fine_estimate_window_size").value
        )
        self.estimate_max_std_xy = float(self.get_parameter("estimate_max_std_xy").value)
        self.estimate_max_std_yaw = math.radians(
            float(self.get_parameter("estimate_max_std_yaw_deg").value)
        )
        self.estimate_jump_reject_m = float(self.get_parameter("estimate_jump_reject_m").value)
        self.max_marker_set_change = int(self.get_parameter("max_marker_set_change").value)
        self.max_reasonable_error_xy = float(self.get_parameter("max_reasonable_error_xy").value)
        self.side_max_reasonable_error_xy = float(
            self.get_parameter("side_max_reasonable_error_xy").value
        )

        self.entry_offset_x = float(self.get_parameter("entry_offset_x").value)
        self.target_offset_x = float(self.get_parameter("target_offset_x").value)
        self.target_offset_y = float(self.get_parameter("target_offset_y").value)
        self.use_orientation_target_offsets = bool(
            self.get_parameter("use_orientation_target_offsets").value
        )
        self.target_offset_x_normal = float(
            self.get_parameter("target_offset_x_normal").value
        )
        self.target_offset_y_normal = float(
            self.get_parameter("target_offset_y_normal").value
        )
        self.target_offset_x_flipped = float(
            self.get_parameter("target_offset_x_flipped").value
        )
        self.target_offset_y_flipped = float(
            self.get_parameter("target_offset_y_flipped").value
        )
        self.side_target_offset_x = float(
            self.get_parameter("side_target_offset_x").value
        )
        self.side_target_pair_offsets_x = self.parse_marker_pair_float_map(
            str(self.get_parameter("side_target_pair_offsets_x").value)
        )
        self.fine_require_opposite_y_pair = bool(
            self.get_parameter("fine_require_opposite_y_pair").value
        )
        self.fine_min_pair_local_y_separation = float(
            self.get_parameter("fine_min_pair_local_y_separation").value
        )
        self.side_require_target_pair_visible = bool(
            self.get_parameter("side_require_target_pair_visible").value
        )
        self.side_align_x_tolerance = float(self.get_parameter("side_align_x_tolerance").value)
        self.side_align_yaw_tolerance = math.radians(
            float(self.get_parameter("side_align_yaw_tolerance_deg").value)
        )
        self.fine_position_tolerance = float(self.get_parameter("fine_position_tolerance").value)
        configured_fine_tolerance_x = float(
            self.get_parameter("fine_position_tolerance_x").value
        )
        configured_fine_tolerance_y = float(
            self.get_parameter("fine_position_tolerance_y").value
        )
        self.fine_position_tolerance_x = (
            self.fine_position_tolerance
            if configured_fine_tolerance_x < 0.0
            else configured_fine_tolerance_x
        )
        self.fine_position_tolerance_y = (
            self.fine_position_tolerance
            if configured_fine_tolerance_y < 0.0
            else configured_fine_tolerance_y
        )
        self.fine_target_yaw = normalize_parallel_angle(
            math.radians(float(self.get_parameter("fine_target_yaw_deg").value))
        )
        self.fine_target_yaw_normal = normalize_parallel_angle(
            math.radians(float(self.get_parameter("fine_target_yaw_deg_normal").value))
        )
        self.fine_target_yaw_flipped = normalize_parallel_angle(
            math.radians(float(self.get_parameter("fine_target_yaw_deg_flipped").value))
        )
        self.fine_yaw_tolerance = math.radians(
            float(self.get_parameter("fine_yaw_tolerance_deg").value)
        )
        self.fine_use_all_under_markers = bool(
            self.get_parameter("fine_use_all_under_markers").value
        )
        self.fine_sequential_alignment = bool(
            self.get_parameter("fine_sequential_alignment").value
        )
        self.fine_control_mode = str(
            self.get_parameter("fine_control_mode").value
        ).strip().lower()
        self.fine_step_settle_sec = float(self.get_parameter("fine_step_settle_sec").value)
        self.fine_step_min_pulse_sec = float(
            self.get_parameter("fine_step_min_pulse_sec").value
        )
        self.fine_step_max_pulse_sec = float(
            self.get_parameter("fine_step_max_pulse_sec").value
        )
        self.fine_step_error_fraction = float(
            self.get_parameter("fine_step_error_fraction").value
        )
        self.fine_auto_flip_bad_xy_step = bool(
            self.get_parameter("fine_auto_flip_bad_xy_step").value
        )
        self.fine_bad_step_error_margin = float(
            self.get_parameter("fine_bad_step_error_margin").value
        )
        self.success_hold_time = float(self.get_parameter("success_hold_time").value)

        self.side_max_vx = float(self.get_parameter("side_max_vx").value)
        self.side_min_vx = float(self.get_parameter("side_min_vx").value)
        self.side_max_wz = float(self.get_parameter("side_max_wz").value)
        self.right_side_vx_sign = float(self.get_parameter("right_side_vx_sign").value)
        self.right_side_wz_sign = float(self.get_parameter("right_side_wz_sign").value)
        self.fine_max_vx = float(self.get_parameter("fine_max_vx").value)
        self.fine_max_vy = float(self.get_parameter("fine_max_vy").value)
        self.fine_max_wz = float(self.get_parameter("fine_max_wz").value)
        self.fine_min_wz = float(self.get_parameter("fine_min_wz").value)
        self.fine_min_vxy = float(self.get_parameter("fine_min_vxy").value)
        configured_min_vx = float(self.get_parameter("fine_min_vx").value)
        configured_min_vy = float(self.get_parameter("fine_min_vy").value)
        self.fine_min_vx = self.fine_min_vxy if configured_min_vx < 0.0 else configured_min_vx
        self.fine_min_vy = self.fine_min_vxy if configured_min_vy < 0.0 else configured_min_vy
        self.fine_acquire_enable = bool(self.get_parameter("fine_acquire_enable").value)
        self.fine_acquire_vx = float(self.get_parameter("fine_acquire_vx").value)
        self.fine_acquire_vy = float(self.get_parameter("fine_acquire_vy").value)
        self.fine_acquire_wz = float(self.get_parameter("fine_acquire_wz").value)
        self.fine_acquire_front_seen_vx = float(
            self.get_parameter("fine_acquire_front_seen_vx").value
        )
        self.fine_acquire_rear_seen_vx = float(
            self.get_parameter("fine_acquire_rear_seen_vx").value
        )
        self.fine_acquire_min_markers = int(self.get_parameter("fine_acquire_min_markers").value)
        self.fine_acquire_timeout = float(self.get_parameter("fine_acquire_timeout").value)
        self.fine_reacquire_after_pair_loss = bool(
            self.get_parameter("fine_reacquire_after_pair_loss").value
        )
        self.aruco_kxy = float(self.get_parameter("aruco_kxy").value)
        self.aruco_kyaw = float(self.get_parameter("aruco_kyaw").value)
        self.straight_under_speed = float(self.get_parameter("straight_under_speed").value)
        self.straight_under_stop_y = float(self.get_parameter("straight_under_stop_y").value)
        self.straight_under_duration_sec = float(
            self.get_parameter("straight_under_duration_sec").value
        )
        self.straight_under_distance_m = float(
            self.get_parameter("straight_under_distance_m").value
        )
        self.straight_under_x_hold_kp = float(
            self.get_parameter("straight_under_x_hold_kp").value
        )
        self.straight_under_max_vx_correction = float(
            self.get_parameter("straight_under_max_vx_correction").value
        )
        self.straight_under_vx_bias = float(
            self.get_parameter("straight_under_vx_bias").value
        )
        self.straight_under_wz_bias = float(
            self.get_parameter("straight_under_wz_bias").value
        )
        self.drive_under_lidar_ignore_radius = float(
            self.get_parameter("drive_under_lidar_ignore_radius").value
        )
        self.drive_under_stop_on_under_markers = bool(
            self.get_parameter("drive_under_stop_on_under_markers").value
        )
        self.drive_under_marker_stop_hold_sec = float(
            self.get_parameter("drive_under_marker_stop_hold_sec").value
        )
        self.drive_under_marker_stop_min_progress = float(
            self.get_parameter("drive_under_marker_stop_min_progress").value
        )

        self.side_vx_sign = float(self.get_parameter("side_vx_sign").value)
        self.side_wz_sign = float(self.get_parameter("side_wz_sign").value)
        self.under_vx_sign = float(self.get_parameter("under_vx_sign").value)
        self.under_vy_sign = float(self.get_parameter("under_vy_sign").value)
        self.under_wz_sign = float(self.get_parameter("under_wz_sign").value)
        self.straight_under_vy_sign = float(self.get_parameter("straight_under_vy_sign").value)

        self.marker_layout = self.load_marker_layout()
        self.underside_marker_ids: Set[int] = self.parse_marker_id_set(
            str(self.get_parameter("under_marker_ids").value),
            set(range(0, 16)),
        )
        self.side_marker_ids: Set[int] = self.parse_marker_id_set(
            str(self.get_parameter("side_marker_ids").value),
            set(range(16, 28)),
        )
        self.side_target_pairs = self.parse_marker_pair_list(
            str(self.get_parameter("side_target_pair_ids").value),
            [(18, 19), (24, 25)],
        )
        self.fine_preferred_pairs = self.parse_marker_pair_list(
            str(self.get_parameter("fine_preferred_pair_ids").value),
            [(5, 10), (3, 12)],
        )
        self.fine_require_preferred_pair = bool(
            self.get_parameter("fine_require_preferred_pair").value
        )

        self.marker_lock = threading.Lock()
        self.latest_markers: Dict[int, Observation] = {}
        self.latest_marker_messages: Dict[str, Tuple[float, Tuple[int, ...], str]] = {}
        self.estimate_window: Deque[Estimate] = deque(maxlen=max(2, self.estimate_window_size))
        self.last_accepted_estimate: Optional[Estimate] = None
        self.last_rejected_reason = ""

        self.mode = "idle"
        self.mode_start_time: Optional[float] = None
        self.sequence_active = False
        self.sequence_wait_until: Optional[float] = None
        self.aligned_since: Optional[float] = None
        self.side_target_pair_seen_fully = False
        self.side_active_target_pair: Optional[Tuple[int, int]] = None
        self.fine_active_marker_pair: Optional[Tuple[int, ...]] = None
        self.fine_had_front_rear_markers = False
        self.fine_first_estimate_status_published = False
        self.fine_sequential_stage = "yaw"
        self.fine_step_state = "measure"
        self.fine_step_until = 0.0
        self.fine_step_cmd = Twist()
        self.fine_vx_direction = 1.0
        self.fine_vy_direction = 1.0
        self.fine_last_pulse_axis: Optional[str] = None
        self.fine_last_pulse_abs_error: Optional[float] = None
        self.acquire_started_at: Optional[float] = None
        self.acquire_best_visible_count = 0
        self.acquire_blocked_until_count: Optional[int] = None
        self.last_wait_log_time = 0.0
        self.last_tf_warn_time = 0.0
        self.last_marker_log_time = 0.0
        self.last_cmd_log_time = 0.0
        self.last_fine_debug_time = 0.0
        self.last_camera_pair_request_time = 0.0
        self.drive_under_start_pose: Optional[Tuple[float, float, float]] = None
        self.drive_under_marker_seen_since: Optional[float] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.drive_under_extra_cmd_pub = None
        if self.drive_under_extra_cmd_vel_topic:
            self.drive_under_extra_cmd_pub = self.create_publisher(
                Twist,
                self.drive_under_extra_cmd_vel_topic,
                10,
            )
        self.lidar_ignore_radius_pub = self.create_publisher(
            Float32,
            self.lidar_ignore_radius_topic,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.camera_pair_pub = self.create_publisher(String, self.camera_pair_command_topic, 10)
        self.create_subscription(String, self.command_topic, self.command_callback, 10)

        self.create_subscription(
            ArucoMarkers,
            str(self.get_parameter("front_aruco_topic").value),
            lambda msg: self.aruco_callback(msg, "front"),
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            str(self.get_parameter("rear_aruco_topic").value),
            lambda msg: self.aruco_callback(msg, "rear"),
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            str(self.get_parameter("left_aruco_topic").value),
            lambda msg: self.aruco_callback(msg, "left"),
            10,
        )
        self.create_subscription(
            ArucoMarkers,
            str(self.get_parameter("right_aruco_topic").value),
            lambda msg: self.aruco_callback(msg, "right"),
            10,
        )

        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.stop_robot()
        self.get_logger().info("Physical trolley marker-layout controller started.")
        self.get_logger().warn(
            "Motion gate: enable_motion=%s. Nonzero /cmd_vel is blocked unless this is true "
            "and the marker-layout estimate is stable." % self.enable_motion
        )
        if self.drive_under_extra_cmd_vel_topic:
            self.get_logger().info(
                "Drive-under also publishes cmd_vel to "
                f"{self.drive_under_extra_cmd_vel_topic} for twist_mux override."
            )
        self.get_logger().warn(
            "Marker layout is a physical calibration assumption. Wrong layout or marker IDs "
            "will produce wrong trolley centers; verify with trolley_reference_logger CSVs."
        )
        self.get_logger().info(
            "Commands: stop, clear_markers, side_align_left, side_align_right, drive_straight_under, "
            "fine_align_under, trolley_ready"
        )
        self.get_logger().info(
            f"Fine under marker whitelist: {sorted(self.underside_marker_ids)}"
        )
        self.get_logger().info(
            "Fine preferred front/rear marker pairs: "
            f"{self.fine_preferred_pairs}; require_preferred={self.fine_require_preferred_pair}"
        )
        self.get_logger().info(
            f"Side marker whitelist: {sorted(self.side_marker_ids)}"
        )
        self.get_logger().info(
            f"Side alignment target marker pairs: {self.side_target_pairs}"
        )
        self.get_logger().info(
            f"Side alignment target x offset: {self.side_target_offset_x:+.4f}m"
        )
        if self.side_target_pair_offsets_x:
            self.get_logger().info(
                "Side alignment pair-specific x offsets: "
                f"{self.side_target_pair_offsets_x}"
            )

    def load_marker_layout(self) -> Dict[int, Tuple[float, float]]:
        layout = dict(DEFAULT_MARKER_LAYOUT)
        layout_json = str(self.get_parameter("marker_layout_json").value).strip()
        if not layout_json:
            return layout

        try:
            parsed = json.loads(layout_json)
            layout = {
                int(marker_id): (float(values[0]), float(values[1]))
                for marker_id, values in parsed.items()
            }
            self.get_logger().info(f"Loaded marker layout override with {len(layout)} markers.")
        except (TypeError, ValueError, json.JSONDecodeError, IndexError) as exc:
            self.get_logger().error(
                f"Invalid marker_layout_json; using built-in physical layout. Error: {exc}"
            )
        return layout

    def parse_marker_id_set(self, text: str, default_ids: Set[int]) -> Set[int]:
        text = text.strip()
        if not text:
            return set(default_ids)

        try:
            marker_ids = {
                int(part.strip())
                for part in text.split(",")
                if part.strip()
            }
        except ValueError as exc:
            self.get_logger().error(
                f"Invalid under_marker_ids='{text}'; using default IDs. Error: {exc}"
            )
            return set(default_ids)

        if not marker_ids:
            self.get_logger().error("under_marker_ids was empty after parsing; using default IDs.")
            return set(default_ids)

        unknown = sorted(marker_id for marker_id in marker_ids if marker_id not in self.marker_layout)
        if unknown:
            self.get_logger().warn(f"under_marker_ids contains IDs without layout entries: {unknown}")

        return marker_ids

    def parse_marker_pair_list(
        self,
        text: str,
        default_pairs: List[Tuple[int, int]],
    ) -> List[Tuple[int, int]]:
        text = text.strip()
        if not text:
            return list(default_pairs)

        pairs = []
        for chunk in text.split(";"):
            parts = [part.strip() for part in chunk.split(",") if part.strip()]
            if not parts:
                continue
            if len(parts) != 2:
                self.get_logger().error(
                    f"Invalid side_target_pair_ids='{text}'; using default pairs."
                )
                return list(default_pairs)
            try:
                pair = (int(parts[0]), int(parts[1]))
            except ValueError:
                self.get_logger().error(
                    f"Invalid side_target_pair_ids='{text}'; using default pairs."
                )
                return list(default_pairs)
            pairs.append(pair)

        if not pairs:
            self.get_logger().error(
                "side_target_pair_ids was empty after parsing; using default pairs."
            )
            return list(default_pairs)

        unknown = sorted(
            marker_id
            for pair in pairs
            for marker_id in pair
            if marker_id not in self.marker_layout
        )
        if unknown:
            self.get_logger().warn(
                f"side_target_pair_ids contains IDs without layout entries: {unknown}"
            )

        return pairs

    def parse_marker_pair_float_map(self, text: str) -> Dict[Tuple[int, int], float]:
        text = text.strip()
        if not text:
            return {}

        offsets = {}
        for chunk in text.split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            if ":" not in chunk:
                self.get_logger().error(
                    f"Invalid side_target_pair_offsets_x='{text}'; expected "
                    "'18,19:0.356;24,25:0.349'. Ignoring pair-specific offsets."
                )
                return {}
            pair_text, offset_text = chunk.split(":", 1)
            parts = [part.strip() for part in pair_text.split(",") if part.strip()]
            if len(parts) != 2:
                self.get_logger().error(
                    f"Invalid side_target_pair_offsets_x='{text}'; expected "
                    "'18,19:0.356;24,25:0.349'. Ignoring pair-specific offsets."
                )
                return {}
            try:
                pair = (int(parts[0]), int(parts[1]))
                offset = float(offset_text.strip())
            except ValueError:
                self.get_logger().error(
                    f"Invalid side_target_pair_offsets_x='{text}'; expected "
                    "'18,19:0.356;24,25:0.349'. Ignoring pair-specific offsets."
                )
                return {}
            offsets[pair] = offset

        return offsets

    def command_callback(self, msg: String):
        command = msg.data.strip().lower().replace(" ", "_")

        if command in ("stop", "cancel", "idle"):
            self.emergency_stop("stopped")
            return

        if command == "clear_markers":
            self.clear_markers()
            self.publish_status("markers_cleared")
            self.get_logger().info("Cleared stored ArUco markers and stability window.")
            return

        if command in ("trolley_ready", "full_docking_sequence", "staged_docking"):
            self.start_staged_docking_sequence()
            return

        if command == "side_align_left":
            self.sequence_active = False
            self.start_mode("side_align_left", "Starting side alignment using side markers 16-27.")
            return

        if command == "side_align_right":
            self.sequence_active = False
            self.start_mode("side_align_right", "Starting right-camera side alignment using side markers 16-27.")
            return

        if command == "drive_straight_under":
            self.sequence_active = False
            self.start_mode(
                "drive_straight_under",
                "Starting straight drive under. Motion still requires enable_motion=True.",
            )
            return

        if command == "fine_align_under":
            self.sequence_active = False
            self.start_front_rear_camera_wait()
            return

        self.get_logger().warn(f"Unknown command: {command}")

    def start_staged_docking_sequence(self):
        self.clear_markers()
        self.sequence_active = True
        self.sequence_wait_until = None
        self.publish_camera_pair("left")
        self.start_mode(
            "side_align_left",
            "Starting staged trolley docking: side_align_left -> drive_straight_under -> "
            "front/rear camera switch -> fine_align_under.",
        )

    def start_fine_align_under(self):
        self.publish_camera_pair("front_rear")
        self.clear_old_markers_only()
        self.reset_estimate_window()
        self.mode = "fine_align_under"
        self.mode_start_time = time.monotonic()
        self.aligned_since = None
        self.acquire_started_at = None
        self.acquire_best_visible_count = 0
        self.acquire_blocked_until_count = None
        self.fine_active_marker_pair = None
        self.fine_had_front_rear_markers = False
        self.fine_first_estimate_status_published = False
        self.fine_sequential_stage = "yaw"
        self.fine_step_state = "measure"
        self.fine_step_until = 0.0
        self.fine_step_cmd = Twist()
        self.fine_vx_direction = 1.0
        self.fine_vy_direction = 1.0
        self.fine_last_pulse_axis = None
        self.fine_last_pulse_abs_error = None
        self.publish_status("fine_align_under_started")
        self.get_logger().info(
            "Starting fine under-trolley estimate/alignment using whitelisted markers."
        )

    def start_front_rear_camera_wait(self):
        self.clear_markers()
        self.publish_camera_pair("front_rear")
        self.mode = "wait_front_rear_camera"
        self.mode_start_time = time.monotonic()
        self.sequence_wait_until = time.monotonic() + self.camera_switch_settle_sec
        self.aligned_since = None
        self.publish_status("front_rear_camera_switch_started")
        self.get_logger().info(
            "Switching to front/rear cameras before final under-trolley alignment."
        )

    def start_mode(self, mode: str, log_text: str):
        if mode == "side_align_left":
            self.publish_camera_pair("left")
        elif mode == "side_align_right":
            self.publish_camera_pair("right")
        elif mode == "drive_straight_under":
            self.publish_camera_pair("front_rear")
        if mode == "drive_straight_under":
            self.set_lidar_ignore_radius(self.drive_under_lidar_ignore_radius)
            self.drive_under_start_pose = self.get_base_pose_in_odom()
            self.drive_under_marker_seen_since = None
            if self.drive_under_start_pose is None:
                self.get_logger().warn(
                    "Could not capture odom start pose for drive_under. "
                    "Distance stop will wait until odom is available."
                )
        self.clear_markers()
        self.mode = mode
        self.mode_start_time = time.monotonic()
        self.aligned_since = None
        if mode in ("side_align_left", "side_align_right"):
            self.side_target_pair_seen_fully = False
            self.side_active_target_pair = None
        self.sequence_wait_until = None
        self.publish_status(f"{mode}_started")
        self.get_logger().info(log_text)

    def aruco_callback(self, msg: ArucoMarkers, camera_name: str):
        now = self.get_clock().now().nanoseconds / 1e9
        marker_ids = [int(marker_id) for marker_id in msg.marker_ids]

        with self.marker_lock:
            self.latest_marker_messages[camera_name] = (
                now,
                tuple(marker_ids),
                msg.header.frame_id,
            )

        if marker_ids and self.mode != "idle" and now - self.last_marker_log_time > 1.0:
            self.last_marker_log_time = now
            self.get_logger().info(
                f"ArUco {camera_name} from {msg.header.frame_id}: ids={marker_ids}"
            )

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
                    self.last_tf_warn_time = now
                    self.get_logger().warn(
                        f"ArUco TF failed: {msg.header.frame_id} -> {self.base_frame}: {exc}"
                    )
                continue

            with self.marker_lock:
                self.latest_markers[marker_id] = Observation(
                    x=pose_base.pose.position.x,
                    y=pose_base.pose.position.y,
                    z=pose_base.pose.position.z,
                    yaw=quaternion_to_yaw(pose_base.pose.orientation),
                    stamp_sec=now,
                    frame_id=msg.header.frame_id,
                    camera=camera_name,
                )

    def control_loop(self):
        if self.mode == "idle":
            return
        if self.mode in ("side_align_left", "side_align_right"):
            self.run_side_align()
        elif self.mode == "drive_straight_under":
            self.run_drive_straight_under()
        elif self.mode == "wait_front_rear_camera":
            self.run_wait_front_rear_camera()
        elif self.mode == "fine_align_under":
            self.run_fine_align_under()

    def run_side_align(self):
        if self.timed_out(self.side_align_timeout):
            self.finish_mode(f"{self.mode}_timeout", success=False)
            return

        camera_filter = "right" if self.mode == "side_align_right" else "left"
        estimate = self.estimate_trolley_from_aruco(
            self.side_marker_ids,
            self.min_side_markers,
            allowed_cameras={camera_filter},
        )
        if estimate is None:
            self.stop_robot()
            self.throttled_info(
                f"{self.mode} waiting for {camera_filter} side markers 16-27. "
                f"visible={self.visible_marker_ids()}"
            )
            return

        side_target = self.side_alignment_target_from_estimate(estimate)
        if side_target is None:
            self.stop_robot()
            self.throttled_info(
                "Side alignment has an estimate, but no valid side target marker pair is configured."
            )
            return

        target_pair, target_x, target_y, local_x, local_y = side_target
        if target_pair != self.side_active_target_pair:
            self.side_active_target_pair = target_pair
            self.side_target_pair_seen_fully = False

        visible_ids = set(estimate.used_ids)
        target_pair_visible = set(target_pair).issubset(visible_ids)
        if target_pair_visible:
            self.side_target_pair_seen_fully = True
        elif self.side_require_target_pair_visible:
            self.stop_robot()
            self.reset_estimate_window()
            state = "lost after being seen" if self.side_target_pair_seen_fully else "not yet visible"
            self.throttled_info(
                "Side alignment target pair is not fully visible; publishing zero cmd_vel. "
                f"state={state} target_pair={list(target_pair)} "
                f"visible={list(estimate.used_ids)}"
            )
            return

        side_target_offset_x = self.side_target_offset_for_pair(target_pair)
        x_error = target_x - side_target_offset_x
        yaw_error = normalize_parallel_angle(estimate.yaw)

        stable, reasons, stats = self.accept_estimate_for_stability(
            estimate,
            min_markers=self.min_side_markers,
            require_front_rear=False,
            x_error=x_error,
            y_error=0.0,
            yaw_error=yaw_error,
            max_reasonable_error_xy=self.side_max_reasonable_error_xy,
        )

        detail = (
            f"target_pair={list(target_pair)} target_x={target_x:+.4f} "
            f"target_y={target_y:+.4f} local_mid=({local_x:+.4f},{local_y:+.4f}) "
            f"side_target_offset_x={side_target_offset_x:+.4f}"
        )
        self.log_estimate("SIDE", estimate, x_error, 0.0, yaw_error, stable, reasons, stats, detail)

        if not stable:
            self.stop_robot()
            return

        yaw_aligned = (
            self.side_max_wz <= 0.0
            or abs(yaw_error) <= self.side_align_yaw_tolerance
        )
        aligned = abs(x_error) <= self.side_align_x_tolerance and yaw_aligned
        if aligned:
            final_summary = (
                f"final camera={camera_filter} ids={list(estimate.used_ids)} "
                f"target_pair={list(target_pair)} "
                f"target_x={target_x:+.4f} target_y={target_y:+.4f} "
                f"side_target_offset_x={side_target_offset_x:+.4f} "
                f"center_x={estimate.center_x:+.4f} center_y={estimate.center_y:+.4f} "
                f"yaw={math.degrees(estimate.yaw):+.2f}deg "
                f"error_x={x_error:+.4f} "
                f"yaw_error={math.degrees(yaw_error):+.2f}deg "
                f"tolerance_x={self.side_align_x_tolerance:.4f} "
                f"tolerance_yaw={math.degrees(self.side_align_yaw_tolerance):.2f}deg"
            )
            self.hold_success_or_finish(f"{self.mode}_done", final_summary)
            return

        self.aligned_since = None
        side_vx_sign = self.right_side_vx_sign if self.mode == "side_align_right" else self.side_vx_sign
        side_wz_sign = self.right_side_wz_sign if self.mode == "side_align_right" else self.side_wz_sign
        cmd = Twist()
        cmd.linear.x = side_vx_sign * self.min_clamped_cmd(
            x_error, self.aruco_kxy, self.side_max_vx, self.side_align_x_tolerance, self.side_min_vx
        )
        cmd.angular.z = side_wz_sign * clamp(self.aruco_kyaw * yaw_error, self.side_max_wz)
        self.publish_motion_or_zero(
            cmd,
            stable,
            (
                f"{self.mode} camera={camera_filter} target_pair={list(target_pair)} error_x={x_error:+.4f} "
                f"yaw_error={math.degrees(yaw_error):+.2f}deg "
                f"side_vx_sign={side_vx_sign:+.1f} "
                f"side_min_vx={self.side_min_vx:+.3f} "
                f"side_wz_sign={side_wz_sign:+.1f}"
            ),
        )

    def run_drive_straight_under(self):
        if self.timed_out(self.drive_under_timeout):
            self.finish_mode("drive_straight_under_timeout", success=False)
            return

        if not self.enable_motion or self.straight_under_speed <= 0.0:
            self.stop_robot()
            self.throttled_info(
                "Straight drive under requested, but motion is disabled or straight_under_speed=0.0."
            )
            return

        if self.straight_under_distance_m > 0.0:
            if self.run_distance_drive_under():
                return

        if self.straight_under_duration_sec > 0.0:
            elapsed = time.monotonic() - self.mode_start_time if self.mode_start_time else 0.0
            if elapsed >= self.straight_under_duration_sec:
                self.finish_mode("drive_straight_under_done", success=True)
                return

            cmd = Twist()
            cmd.linear.x = self.straight_under_vx_bias
            cmd.linear.y = self.straight_under_vy_sign * self.straight_under_speed
            cmd.angular.z = self.straight_under_wz_bias
            self.publish_drive_under_cmd(cmd)
            self.throttled_cmd_info(
                "Timed straight-under drive; publishing cmd_vel "
                f"vx={cmd.linear.x:+.3f} vy={cmd.linear.y:+.3f} "
                f"wz={cmd.angular.z:+.3f} elapsed={elapsed:.1f}/"
                f"{self.straight_under_duration_sec:.1f}s"
            )
            return

        estimate = self.estimate_trolley_from_aruco(self.side_marker_ids, self.min_side_markers)
        if estimate is not None:
            y_error = estimate.center_y - self.target_offset_y
            yaw_error = normalize_parallel_angle(estimate.yaw)
            self.log_estimate("STRAIGHT_UNDER", estimate, 0.0, y_error, yaw_error, True, [], {
                "std_x": 0.0,
                "std_y": 0.0,
                "std_yaw": 0.0,
            })
            if abs(y_error) <= self.straight_under_stop_y:
                self.finish_mode("drive_straight_under_done", success=True)
                return

        cmd = Twist()
        cmd.linear.x = self.straight_under_vx_bias
        cmd.linear.y = self.straight_under_vy_sign * self.straight_under_speed
        cmd.angular.z = self.straight_under_wz_bias
        self.publish_drive_under_cmd(cmd)

    def run_distance_drive_under(self) -> bool:
        if self.drive_under_start_pose is None:
            self.drive_under_start_pose = self.get_base_pose_in_odom()
            if self.drive_under_start_pose is None:
                self.stop_robot()
                self.throttled_info("Distance drive-under waiting for odom -> base_link TF.")
                return True

        current_pose = self.get_base_pose_in_odom()
        if current_pose is None:
            self.stop_robot()
            self.throttled_info("Distance drive-under lost odom -> base_link TF.")
            return True

        start_x, start_y, start_yaw = self.drive_under_start_pose
        current_x, current_y, _ = current_pose
        dx = current_x - start_x
        dy = current_y - start_y
        local_x = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        local_y = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        progress = self.straight_under_vy_sign * local_y

        if self.drive_under_under_marker_stop_reached(progress):
            return True

        if progress >= self.straight_under_distance_m:
            self.finish_mode("drive_straight_under_done", success=True)
            return True

        vx_correction = clamp(
            -self.straight_under_x_hold_kp * local_x,
            self.straight_under_max_vx_correction,
        )
        cmd = Twist()
        cmd.linear.x = self.straight_under_vx_bias + vx_correction
        cmd.linear.y = self.straight_under_vy_sign * self.straight_under_speed
        cmd.angular.z = self.straight_under_wz_bias
        self.publish_drive_under_cmd(cmd)
        self.throttled_cmd_info(
            "Distance straight-under drive; publishing cmd_vel "
            f"vx={cmd.linear.x:+.3f} vy={cmd.linear.y:+.3f} "
            f"wz={cmd.angular.z:+.3f} progress={progress:.3f}/"
            f"{self.straight_under_distance_m:.3f}m local_x={local_x:+.3f}m"
        )
        return True

    def drive_under_under_marker_stop_reached(self, progress: float) -> bool:
        if not self.drive_under_stop_on_under_markers:
            return False
        if progress < self.drive_under_marker_stop_min_progress:
            return False

        estimate = self.estimate_fine_trolley_from_front_rear_pair()
        if estimate is None:
            self.drive_under_marker_seen_since = None
            return False

        now = time.monotonic()
        if self.drive_under_marker_seen_since is None:
            self.drive_under_marker_seen_since = now

        held = now - self.drive_under_marker_seen_since
        target_offset_x, target_offset_y = self.target_offsets_for_estimate(estimate)
        x_error = estimate.center_x - target_offset_x
        y_error = estimate.center_y - target_offset_y
        yaw_error = self.fine_yaw_error(estimate)
        self.publish_first_fine_estimate_status(
            estimate,
            x_error,
            y_error,
            yaw_error,
            target_offset_x,
            target_offset_y,
        )
        self.publish_fine_debug(
            "drive_under_saw_fine_markers",
            estimate=estimate,
            x_error=x_error,
            y_error=y_error,
            yaw_error=yaw_error,
            stable=True,
            reasons=[f"progress={progress:.3f}m", f"hold={held:.2f}s"],
        )

        if held < self.drive_under_marker_stop_hold_sec:
            return False

        self.get_logger().info(
            "Drive-under stopping early because front/rear underside markers are visible "
            f"for fine alignment. progress={progress:.3f}m "
            f"ids={list(estimate.used_ids)} center=({estimate.center_x:+.4f},"
            f"{estimate.center_y:+.4f}) error=({x_error:+.4f},{y_error:+.4f}) "
            f"yaw_error={math.degrees(yaw_error):+.2f}deg"
        )
        self.finish_mode("drive_straight_under_done", success=True)
        return True

    def run_wait_front_rear_camera(self):
        if self.sequence_wait_until is None:
            self.sequence_wait_until = time.monotonic() + self.camera_switch_settle_sec
        self.republish_camera_pair_if_due("front_rear")
        self.stop_robot()
        remaining = self.sequence_wait_until - time.monotonic()
        if remaining > 0.0:
            self.throttled_info(f"Waiting for front/rear cameras to settle: {remaining:.1f}s")
            return
        self.start_fine_align_under()

    def run_fine_align_under(self):
        if self.timed_out(self.fine_align_timeout):
            self.finish_mode("fine_align_under_timeout", success=False)
            return

        estimate = self.estimate_fine_trolley_from_front_rear_pair()
        if estimate is None:
            if (
                self.fine_control_mode == "step"
                and self.fine_had_front_rear_markers
            ):
                if (
                    self.fine_reacquire_after_pair_loss
                    and self.publish_fine_acquire_motion()
                ):
                    return

                self.stop_robot()
                self.aligned_since = None
                self.fine_step_state = "measure"
                self.publish_fine_debug("step_pair_lost_holding_position")
                self.throttled_info(
                    "Fine step alignment has already seen a front/rear marker pair, "
                    "but the pair is not visible now. Holding position and waiting "
                    "instead of driving blind."
                )
                return

            if (
                self.fine_control_mode != "step"
                and self.fine_sequential_alignment
                and self.fine_sequential_stage == "x"
                and self.fine_had_front_rear_markers
            ):
                self.stop_robot()
                self.aligned_since = None
                self.publish_fine_debug("x_stage_pair_lost_holding_position")
                self.throttled_info(
                    "Fine sequential alignment is in X-only stage, but the front/rear "
                    "marker pair is not visible. Holding position instead of running "
                    "marker acquisition motion."
                )
                return

            if self.publish_fine_acquire_motion():
                return

            self.stop_robot()
            self.aligned_since = None
            self.publish_fine_debug("waiting_for_front_rear_pair")
            self.throttled_info(
                "Fine under alignment waiting for enough whitelisted underside markers. "
                f"{self.visible_under_marker_summary()}"
            )
            return

        self.acquire_started_at = None

        target_offset_x, target_offset_y = self.target_offsets_for_estimate(estimate)
        x_error = estimate.center_x - target_offset_x
        y_error = estimate.center_y - target_offset_y
        yaw_error = self.fine_yaw_error(estimate)

        stable, reasons, stats = self.accept_estimate_for_stability(
            estimate,
            min_markers=self.min_under_markers,
            require_front_rear=True,
            x_error=x_error,
            y_error=y_error,
            yaw_error=yaw_error,
            max_reasonable_error_xy=self.max_reasonable_error_xy,
            window_size=self.fine_estimate_window_size,
        )
        self.log_estimate("FINE", estimate, x_error, y_error, yaw_error, stable, reasons, stats)

        if not stable:
            if self.fine_control_mode == "step":
                self.stop_robot()
                self.fine_step_state = "measure"
                self.publish_fine_debug(
                    "step_estimate_not_stable_holding",
                    estimate=estimate,
                    x_error=x_error,
                    y_error=y_error,
                    yaw_error=yaw_error,
                    stable=stable,
                    reasons=reasons,
                )
                self.throttled_cmd_info(
                    "Fine step alignment estimate is not stable/sane; holding still."
                )
                self.aligned_since = None
                return

            if (
                estimate.front_count < self.min_front_under_markers
                or estimate.rear_count < self.min_rear_under_markers
            ) and self.publish_fine_acquire_motion():
                return
            self.stop_robot()
            self.publish_fine_debug(
                "estimate_not_stable_or_sane",
                estimate=estimate,
                x_error=x_error,
                y_error=y_error,
                yaw_error=yaw_error,
                stable=stable,
                reasons=reasons,
            )
            self.throttled_cmd_info("Estimate not stable/sane; publishing zero cmd_vel.")
            self.aligned_since = None
            return

        x_aligned = abs(x_error) <= self.fine_position_tolerance_x
        y_aligned = abs(y_error) <= self.fine_position_tolerance_y
        yaw_aligned = abs(yaw_error) <= self.fine_yaw_tolerance
        aligned = x_aligned and y_aligned and yaw_aligned
        sequential_x_done = (
            self.fine_sequential_alignment
            and self.fine_sequential_stage == "x"
            and x_aligned
        )
        if aligned or sequential_x_done:
            self.throttled_info(
                "Fine under alignment complete; publishing zero cmd_vel. "
                f"error_x={x_error:+.4f} error_y={y_error:+.4f} "
                f"yaw_error={math.degrees(yaw_error):+.2f}deg | "
                f"tolerance_x={self.fine_position_tolerance_x:.4f} "
                f"tolerance_y={self.fine_position_tolerance_y:.4f} "
                f"tolerance_yaw={math.degrees(self.fine_yaw_tolerance):.2f}deg"
            )
            final_summary = (
                f"final ids={list(estimate.used_ids)} "
                f"center_x={estimate.center_x:+.4f} center_y={estimate.center_y:+.4f} "
                f"yaw={math.degrees(estimate.yaw):+.2f}deg "
                f"orientation={trolley_orientation_label(estimate.yaw)} "
                f"error_x={x_error:+.4f} error_y={y_error:+.4f} "
                f"yaw_error={math.degrees(yaw_error):+.2f}deg "
                f"target_x={target_offset_x:+.4f} target_y={target_offset_y:+.4f} "
                f"tolerance_x={self.fine_position_tolerance_x:.4f} "
                f"tolerance_y={self.fine_position_tolerance_y:.4f} "
                f"tolerance_yaw={math.degrees(self.fine_yaw_tolerance):.2f}deg"
            )
            self.hold_success_or_finish("fine_align_under_done", final_summary)
            return

        if self.fine_control_mode == "step":
            self.run_fine_step_alignment(
                estimate,
                x_error,
                y_error,
                yaw_error,
                x_aligned,
                y_aligned,
                yaw_aligned,
            )
            return

        self.aligned_since = None
        cmd = Twist()
        motion_stage = "fine_motion"

        if self.fine_sequential_alignment:
            if self.fine_sequential_stage == "yaw" and yaw_aligned:
                self.fine_sequential_stage = "y"
                self.get_logger().info(
                    "Fine sequential alignment: yaw locked. Continuing with Y-only correction."
                )
            if self.fine_sequential_stage == "y" and y_aligned:
                self.fine_sequential_stage = "x"
                self.get_logger().info(
                    "Fine sequential alignment: Y locked. Continuing with X-only correction."
                )

            if self.fine_sequential_stage == "x":
                motion_stage = "fine_motion_x_stage"
                cmd.linear.x = self.fine_vx_direction * self.under_vx_sign * self.min_clamped_cmd(
                    x_error, self.aruco_kxy, self.fine_max_vx, self.fine_position_tolerance_x, self.fine_min_vx
                )
            elif self.fine_sequential_stage == "y":
                motion_stage = "fine_motion_y_stage"
                cmd.linear.y = self.fine_vy_direction * self.under_vy_sign * self.min_clamped_cmd(
                    y_error, self.aruco_kxy, self.fine_max_vy, self.fine_position_tolerance_y, self.fine_min_vy
                )
            else:
                motion_stage = "fine_motion_yaw_stage"
                cmd.angular.z = self.under_wz_sign * self.fine_yaw_cmd(yaw_error)
        else:
            cmd.linear.x = self.fine_vx_direction * self.under_vx_sign * self.min_clamped_cmd(
                x_error, self.aruco_kxy, self.fine_max_vx, self.fine_position_tolerance_x, self.fine_min_vx
            )
            cmd.linear.y = self.fine_vy_direction * self.under_vy_sign * self.min_clamped_cmd(
                y_error, self.aruco_kxy, self.fine_max_vy, self.fine_position_tolerance_y, self.fine_min_vy
            )
            cmd.angular.z = self.under_wz_sign * self.fine_yaw_cmd(yaw_error)
        self.publish_motion_or_zero(cmd, stable)
        self.publish_fine_debug(
            motion_stage,
            estimate=estimate,
            cmd=cmd,
            x_error=x_error,
            y_error=y_error,
            yaw_error=yaw_error,
            stable=stable,
            reasons=[],
        )

    def run_fine_step_alignment(
        self,
        estimate: Estimate,
        x_error: float,
        y_error: float,
        yaw_error: float,
        x_aligned: bool,
        y_aligned: bool,
        yaw_aligned: bool,
    ):
        now = time.monotonic()

        if self.fine_step_state == "pulse":
            if now < self.fine_step_until:
                self.publish_motion_or_zero(self.fine_step_cmd, True)
                self.publish_fine_debug(
                    f"fine_step_pulse_{self.fine_sequential_stage}",
                    estimate=estimate,
                    cmd=self.fine_step_cmd,
                    x_error=x_error,
                    y_error=y_error,
                    yaw_error=yaw_error,
                    stable=True,
                    reasons=[f"remaining={self.fine_step_until - now:.2f}s"],
                )
                return

            self.stop_robot()
            self.maybe_flip_bad_fine_step_direction(x_error, y_error)
            self.fine_step_state = "settle"
            self.fine_step_until = now + self.fine_step_settle_sec
            self.publish_fine_debug(
                f"fine_step_pulse_done_{self.fine_sequential_stage}",
                estimate=estimate,
                x_error=x_error,
                y_error=y_error,
                yaw_error=yaw_error,
                stable=True,
                reasons=[f"settle={self.fine_step_settle_sec:.2f}s"],
            )
            return

        if self.fine_step_state == "settle":
            self.stop_robot()
            if now < self.fine_step_until:
                self.publish_fine_debug(
                    f"fine_step_settle_{self.fine_sequential_stage}",
                    estimate=estimate,
                    x_error=x_error,
                    y_error=y_error,
                    yaw_error=yaw_error,
                    stable=True,
                    reasons=[f"remaining={self.fine_step_until - now:.2f}s"],
                )
                return
            self.fine_step_state = "measure"

        if self.fine_sequential_stage == "yaw" and yaw_aligned:
            self.fine_sequential_stage = "y"
            self.stop_robot()
            self.fine_step_state = "settle"
            self.fine_step_until = now + self.fine_step_settle_sec
            self.get_logger().info(
                "Fine step alignment: yaw locked. Continuing with Y-only correction."
            )
            return

        if self.fine_sequential_stage == "y" and y_aligned:
            self.fine_sequential_stage = "x"
            self.stop_robot()
            self.fine_step_state = "settle"
            self.fine_step_until = now + self.fine_step_settle_sec
            self.get_logger().info(
                "Fine step alignment: Y locked. Continuing with X-only correction."
            )
            return

        cmd = Twist()
        axis_error = 0.0
        axis_speed = 0.0
        motion_stage = f"fine_step_measure_{self.fine_sequential_stage}"

        if self.fine_sequential_stage == "x":
            cmd.linear.x = self.fine_vx_direction * self.under_vx_sign * self.min_clamped_cmd(
                x_error, self.aruco_kxy, self.fine_max_vx, self.fine_position_tolerance_x, self.fine_min_vx
            )
            axis_error = abs(x_error)
            axis_speed = abs(cmd.linear.x)
            motion_stage = "fine_step_start_x_pulse"
        elif self.fine_sequential_stage == "y":
            cmd.linear.y = self.fine_vy_direction * self.under_vy_sign * self.min_clamped_cmd(
                y_error, self.aruco_kxy, self.fine_max_vy, self.fine_position_tolerance_y, self.fine_min_vy
            )
            axis_error = abs(y_error)
            axis_speed = abs(cmd.linear.y)
            motion_stage = "fine_step_start_y_pulse"
        else:
            cmd.angular.z = self.under_wz_sign * self.fine_yaw_cmd(yaw_error)
            axis_error = abs(yaw_error)
            axis_speed = abs(cmd.angular.z)
            motion_stage = "fine_step_start_yaw_pulse"

        if axis_speed <= 1e-6:
            self.stop_robot()
            self.publish_fine_debug(
                f"fine_step_no_cmd_{self.fine_sequential_stage}",
                estimate=estimate,
                cmd=cmd,
                x_error=x_error,
                y_error=y_error,
                yaw_error=yaw_error,
                stable=True,
                reasons=["axis command is zero"],
            )
            return

        pulse_duration = self.fine_step_error_fraction * axis_error / axis_speed
        pulse_duration = max(
            self.fine_step_min_pulse_sec,
            min(self.fine_step_max_pulse_sec, pulse_duration),
        )

        self.fine_step_cmd = cmd
        self.fine_step_state = "pulse"
        self.fine_step_until = now + pulse_duration
        self.fine_last_pulse_axis = self.fine_sequential_stage
        self.fine_last_pulse_abs_error = axis_error
        self.aligned_since = None
        self.publish_motion_or_zero(cmd, True)
        self.publish_fine_debug(
            motion_stage,
            estimate=estimate,
            cmd=cmd,
            x_error=x_error,
            y_error=y_error,
            yaw_error=yaw_error,
            stable=True,
            reasons=[f"pulse={pulse_duration:.2f}s"],
        )

    def maybe_flip_bad_fine_step_direction(self, x_error: float, y_error: float):
        if (
            not self.fine_auto_flip_bad_xy_step
            or self.fine_last_pulse_axis not in ("x", "y")
            or self.fine_last_pulse_abs_error is None
        ):
            return

        current_error = abs(x_error) if self.fine_last_pulse_axis == "x" else abs(y_error)
        previous_error = self.fine_last_pulse_abs_error
        if current_error <= previous_error + self.fine_bad_step_error_margin:
            return

        if self.fine_last_pulse_axis == "x":
            self.fine_vx_direction *= -1.0
            new_sign = self.fine_vx_direction * self.under_vx_sign
        else:
            self.fine_vy_direction *= -1.0
            new_sign = self.fine_vy_direction * self.under_vy_sign

        self.reset_estimate_window()
        self.fine_step_state = "measure"
        self.get_logger().warn(
            "Fine step %s pulse made error worse; flipping that axis direction. "
            "error %.4f -> %.4f, effective_sign=%+.1f"
            % (
                self.fine_last_pulse_axis,
                previous_error,
                current_error,
                new_sign,
            )
        )

    def estimate_fine_trolley_from_front_rear_pair(self) -> Optional[Estimate]:
        observations = self.visible_under_observations()
        front_ids = sorted(
            marker_id
            for marker_id, observation in observations.items()
            if observation.camera == "front"
        )
        rear_ids = sorted(
            marker_id
            for marker_id, observation in observations.items()
            if observation.camera == "rear"
        )

        if (
            len(front_ids) < self.min_front_under_markers
            or len(rear_ids) < self.min_rear_under_markers
        ):
            return None

        if (
            self.fine_use_all_under_markers
            and not self.fine_require_preferred_pair
            and len(observations) >= self.min_under_markers
        ):
            estimate = self.estimate_trolley_from_aruco(set(observations.keys()), self.min_under_markers)
            if (
                estimate is not None
                and estimate.front_count >= self.min_front_under_markers
                and estimate.rear_count >= self.min_rear_under_markers
            ):
                target_offset_x, target_offset_y = self.target_offsets_for_estimate(estimate)
                x_error = estimate.center_x - target_offset_x
                y_error = estimate.center_y - target_offset_y
                if (
                    abs(x_error) <= self.max_reasonable_error_xy
                    and abs(y_error) <= self.max_reasonable_error_xy
                ):
                    self.fine_had_front_rear_markers = True
                    marker_ids = tuple(sorted(estimate.used_ids))
                    if marker_ids != self.fine_active_marker_pair:
                        self.reset_estimate_window()
                        self.fine_active_marker_pair = marker_ids
                        self.get_logger().info(
                            "Fine alignment active marker set: "
                            f"{list(marker_ids)}"
                        )
                    return estimate

        candidates: List[Tuple[float, Tuple[int, int], Estimate]] = []
        active_pair = set(self.fine_active_marker_pair or ())
        seen_pair_sets = set()

        def candidate_for_pair(pair_set: Set[int]) -> Optional[Tuple[float, Tuple[int, int], Estimate]]:
            pair_key = tuple(sorted(pair_set))
            if len(pair_key) != 2:
                return None
            seen_pair_sets.add(pair_key)
            if self.fine_require_opposite_y_pair and not self.fine_pair_has_opposite_y(pair_key):
                return None

            estimate = self.estimate_trolley_from_aruco(set(pair_key), 2)
            if estimate is None:
                return None
            if (
                estimate.front_count < self.min_front_under_markers
                or estimate.rear_count < self.min_rear_under_markers
            ):
                return None

            target_offset_x, target_offset_y = self.target_offsets_for_estimate(estimate)
            x_error = estimate.center_x - target_offset_x
            y_error = estimate.center_y - target_offset_y
            yaw_error = self.fine_yaw_error(estimate)
            if abs(x_error) > self.max_reasonable_error_xy or abs(y_error) > self.max_reasonable_error_xy:
                return None

            yaw_penalty = 0.002 * abs(math.degrees(yaw_error))
            score = abs(x_error) + abs(y_error) + yaw_penalty
            return score, pair_key, estimate

        for preferred_pair in self.fine_preferred_pairs:
            pair_key = tuple(sorted(preferred_pair))
            if pair_key[0] not in observations or pair_key[1] not in observations:
                continue
            candidate = candidate_for_pair(set(pair_key))
            if candidate is not None:
                candidates.append(candidate)

        if candidates:
            candidates.sort(key=lambda item: item[0])
            estimate = candidates[0][2]
            self.fine_had_front_rear_markers = True
            marker_pair = tuple(sorted(estimate.used_ids))
            if marker_pair != self.fine_active_marker_pair:
                self.reset_estimate_window()
                self.fine_active_marker_pair = marker_pair
                self.get_logger().info(
                    f"Fine alignment active preferred marker pair: {list(marker_pair)}"
                )
            return estimate

        if self.fine_require_preferred_pair:
            return None

        if len(active_pair) == 2:
            active_candidate = candidate_for_pair(active_pair)
            if active_candidate is not None:
                self.fine_had_front_rear_markers = True
                return active_candidate[2]

        for front_id in front_ids:
            for rear_id in rear_ids:
                pair_key = tuple(sorted((front_id, rear_id)))
                if pair_key in seen_pair_sets:
                    continue
                candidate = candidate_for_pair({front_id, rear_id})
                if candidate is not None:
                    candidates.append(candidate)

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        estimate = candidates[0][2]
        self.fine_had_front_rear_markers = True
        marker_pair = tuple(sorted(estimate.used_ids))
        if marker_pair != self.fine_active_marker_pair:
            self.reset_estimate_window()
            self.fine_active_marker_pair = marker_pair
            self.get_logger().info(
                f"Fine alignment active front/rear marker pair: {list(marker_pair)}"
            )
        return estimate

    def fine_yaw_cmd(self, yaw_error: float) -> float:
        return self.min_clamped_cmd(
            yaw_error,
            self.aruco_kyaw,
            self.fine_max_wz,
            self.fine_yaw_tolerance,
            self.fine_min_wz,
        )

    def target_offsets_for_estimate(self, estimate: Estimate) -> Tuple[float, float]:
        if not self.use_orientation_target_offsets:
            return self.target_offset_x, self.target_offset_y
        if math.cos(estimate.yaw) < 0.0:
            return self.target_offset_x_flipped, self.target_offset_y_flipped
        return self.target_offset_x_normal, self.target_offset_y_normal

    def fine_yaw_target_for_estimate(self, estimate: Estimate) -> float:
        if not self.use_orientation_target_offsets:
            return self.fine_target_yaw
        if math.cos(estimate.yaw) < 0.0:
            return self.fine_target_yaw_flipped
        return self.fine_target_yaw_normal

    def fine_yaw_error(self, estimate: Estimate) -> float:
        return normalize_parallel_angle(
            estimate.yaw - self.fine_yaw_target_for_estimate(estimate)
        )

    def side_target_offset_for_pair(self, target_pair: Tuple[int, int]) -> float:
        return (
            self.side_target_pair_offsets_x.get(target_pair)
            or self.side_target_pair_offsets_x.get((target_pair[1], target_pair[0]))
            or self.side_target_offset_x
        )

    def fine_pair_has_opposite_y(self, marker_pair: Tuple[int, ...]) -> bool:
        if len(marker_pair) != 2:
            return False
        first_y = self.marker_layout[marker_pair[0]][1]
        second_y = self.marker_layout[marker_pair[1]][1]
        if first_y * second_y >= 0.0:
            return False
        return abs(first_y - second_y) >= self.fine_min_pair_local_y_separation

    def publish_fine_acquire_motion(self) -> bool:
        observations = self.visible_under_observations()
        visible_count = len(observations)
        front_count = sum(1 for observation in observations.values() if observation.camera == "front")
        rear_count = sum(1 for observation in observations.values() if observation.camera == "rear")
        front_ready = front_count >= self.min_front_under_markers
        rear_ready = rear_count >= self.min_rear_under_markers

        if visible_count <= 0:
            self.acquire_started_at = None
            self.acquire_best_visible_count = 0
            return False

        if not self.enable_motion or not self.fine_acquire_enable:
            self.acquire_started_at = None
            return False

        if visible_count < self.fine_acquire_min_markers:
            self.acquire_started_at = None
            return False

        if (
            self.fine_had_front_rear_markers
            and not (front_ready and rear_ready)
            and not self.fine_reacquire_after_pair_loss
        ):
            self.stop_robot()
            self.acquire_started_at = None
            self.publish_fine_debug("holding_one_side_missing_after_front_rear_seen")
            self.throttled_info(
                "Fine alignment has already seen front and rear markers; holding position "
                "instead of front/rear acquisition motion while one side is missing. "
                f"{self.visible_under_marker_summary()}"
            )
            return True

        cmd = Twist()
        acquire_reason = ""
        if (
            self.fine_had_front_rear_markers
            and self.fine_sequential_stage == "x"
            and abs(self.fine_step_cmd.linear.x) > 0.0
        ):
            cmd.linear.x = -self.fine_step_cmd.linear.x
            acquire_reason = "preferred marker pair lost during X; backing out last X pulse"
        elif front_ready and not rear_ready and abs(self.fine_acquire_front_seen_vx) > 0.0:
            cmd.linear.x = self.fine_acquire_front_seen_vx
            acquire_reason = "front markers visible, rear missing; creeping toward robot front"
        elif rear_ready and not front_ready and abs(self.fine_acquire_rear_seen_vx) > 0.0:
            cmd.linear.x = self.fine_acquire_rear_seen_vx
            acquire_reason = "rear markers visible, front missing; creeping toward robot rear"
        elif any(
            abs(value) > 0.0
            for value in (self.fine_acquire_vx, self.fine_acquire_vy, self.fine_acquire_wz)
        ):
            cmd.linear.x = self.fine_acquire_vx
            cmd.linear.y = self.fine_acquire_vy
            cmd.angular.z = self.fine_acquire_wz
            acquire_reason = "configured marker acquisition motion"
        else:
            self.acquire_started_at = None
            return False

        now = time.monotonic()
        if self.acquire_started_at is None:
            self.acquire_started_at = now
            self.acquire_best_visible_count = visible_count
        elif visible_count > self.acquire_best_visible_count:
            self.acquire_best_visible_count = visible_count

        elapsed = now - self.acquire_started_at
        if elapsed > self.fine_acquire_timeout:
            self.stop_robot()
            self.throttled_info(
                "Fine marker acquisition timeout; publishing zero cmd_vel. "
                f"{self.visible_under_marker_summary()} "
                f"timeout={self.fine_acquire_timeout:.1f}s"
            )
            return True

        self.cmd_pub.publish(cmd)
        self.publish_fine_debug(acquire_reason, cmd=cmd)
        self.throttled_cmd_info(
            "Acquiring whitelisted under markers; publishing cmd_vel "
            f"vx={cmd.linear.x:+.3f} vy={cmd.linear.y:+.3f} wz={cmd.angular.z:+.3f} | "
            f"{acquire_reason} | "
            f"{self.visible_under_marker_summary()}"
        )
        return True

    def estimate_trolley_from_aruco(
        self,
        allowed_marker_ids: Set[int],
        min_markers: int,
        allowed_cameras: Optional[Set[str]] = None,
    ) -> Optional[Estimate]:
        now = self.get_clock().now().nanoseconds / 1e9
        with self.marker_lock:
            observed = {
                marker_id: observation
                for marker_id, observation in self.latest_markers.items()
                if (
                    now - observation.stamp_sec <= self.aruco_timeout
                    and marker_id in self.marker_layout
                    and marker_id in allowed_marker_ids
                    and (allowed_cameras is None or observation.camera in allowed_cameras)
                )
            }

        if len(observed) < min_markers:
            return None

        marker_ids = sorted(observed.keys())
        yaw_samples = []
        for index, marker_a in enumerate(marker_ids):
            for marker_b in marker_ids[index + 1:]:
                local_ax, local_ay = self.marker_layout[marker_a]
                local_bx, local_by = self.marker_layout[marker_b]
                base_ax = observed[marker_a].x
                base_ay = observed[marker_a].y
                base_bx = observed[marker_b].x
                base_by = observed[marker_b].y

                local_dx = local_bx - local_ax
                local_dy = local_by - local_ay
                base_dx = base_bx - base_ax
                base_dy = base_by - base_ay
                if math.hypot(local_dx, local_dy) < 0.10:
                    continue

                yaw_samples.append(
                    normalize_angle(math.atan2(base_dy, base_dx) - math.atan2(local_dy, local_dx))
                )

        trolley_yaw = circular_mean(yaw_samples) if yaw_samples else observed[marker_ids[0]].yaw
        cos_yaw = math.cos(trolley_yaw)
        sin_yaw = math.sin(trolley_yaw)

        center_x_values = []
        center_y_values = []
        cameras = set()
        front_count = 0
        rear_count = 0
        for marker_id, observation in observed.items():
            local_x, local_y = self.marker_layout[marker_id]
            center_x_values.append(observation.x - (cos_yaw * local_x - sin_yaw * local_y))
            center_y_values.append(observation.y - (sin_yaw * local_x + cos_yaw * local_y))
            cameras.add(observation.camera)
            if observation.camera == "front":
                front_count += 1
            elif observation.camera == "rear":
                rear_count += 1

        return Estimate(
            center_x=mean(center_x_values),
            center_y=mean(center_y_values),
            yaw=trolley_yaw,
            used_ids=tuple(marker_ids),
            cameras=tuple(sorted(cameras)),
            front_count=front_count,
            rear_count=rear_count,
            stamp_sec=now,
        )

    def side_alignment_target_from_estimate(
        self,
        estimate: Estimate,
    ) -> Optional[Tuple[Tuple[int, int], float, float, float, float]]:
        valid_pairs = [
            pair
            for pair in self.side_target_pairs
            if pair[0] in self.marker_layout and pair[1] in self.marker_layout
        ]
        if not valid_pairs:
            return None

        used_ids = set(estimate.used_ids)

        def pair_score(pair: Tuple[int, int]) -> Tuple[int, int]:
            visible_in_pair = len(used_ids.intersection(pair))
            nearest_visible = min(
                (abs(pair_id - used_id) for pair_id in pair for used_id in used_ids),
                default=999,
            )
            return visible_in_pair, -nearest_visible

        target_pair = max(valid_pairs, key=pair_score)
        local_ax, local_ay = self.marker_layout[target_pair[0]]
        local_bx, local_by = self.marker_layout[target_pair[1]]
        local_x = (local_ax + local_bx) / 2.0
        local_y = (local_ay + local_by) / 2.0

        now = self.get_clock().now().nanoseconds / 1e9
        with self.marker_lock:
            observation_a = self.latest_markers.get(target_pair[0])
            observation_b = self.latest_markers.get(target_pair[1])

        if (
            observation_a is not None
            and observation_b is not None
            and now - observation_a.stamp_sec <= self.aruco_timeout
            and now - observation_b.stamp_sec <= self.aruco_timeout
            and observation_a.camera in estimate.cameras
            and observation_b.camera in estimate.cameras
        ):
            target_x = (observation_a.x + observation_b.x) / 2.0
            target_y = (observation_a.y + observation_b.y) / 2.0
            return target_pair, target_x, target_y, local_x, local_y

        cos_yaw = math.cos(estimate.yaw)
        sin_yaw = math.sin(estimate.yaw)
        target_x = estimate.center_x + (cos_yaw * local_x - sin_yaw * local_y)
        target_y = estimate.center_y + (sin_yaw * local_x + cos_yaw * local_y)

        return target_pair, target_x, target_y, local_x, local_y

    def accept_estimate_for_stability(
        self,
        estimate: Estimate,
        min_markers: int,
        require_front_rear: bool,
        x_error: float,
        y_error: float,
        yaw_error: float,
        max_reasonable_error_xy: float,
        window_size: Optional[int] = None,
    ) -> Tuple[bool, List[str], Dict[str, float]]:
        reasons = []
        stats = {"std_x": 0.0, "std_y": 0.0, "std_yaw": 0.0}
        target_window_size = max(2, window_size or self.estimate_window_size)

        if len(estimate.used_ids) < min_markers:
            reasons.append(f"need {min_markers} markers, have {len(estimate.used_ids)}")
        if require_front_rear:
            if estimate.front_count < self.min_front_under_markers:
                reasons.append(
                    f"need front markers {self.min_front_under_markers}, have {estimate.front_count}"
                )
            if estimate.rear_count < self.min_rear_under_markers:
                reasons.append(
                    f"need rear markers {self.min_rear_under_markers}, have {estimate.rear_count}"
                )
        if abs(x_error) > max_reasonable_error_xy or abs(y_error) > max_reasonable_error_xy:
            reasons.append(
                f"error outside sane bounds {max_reasonable_error_xy:.3f}m"
            )

        previous_estimate = self.last_accepted_estimate
        if previous_estimate is not None:
            previous_set = set(previous_estimate.used_ids)
            current_set = set(estimate.used_ids)
            set_change = len(previous_set.symmetric_difference(current_set))
            jump = math.hypot(
                estimate.center_x - previous_estimate.center_x,
                estimate.center_y - previous_estimate.center_y,
            )
            reset_for_discontinuity = False

            if set_change > self.max_marker_set_change:
                self.last_rejected_reason = f"marker set changed by {set_change}"
                reasons.append(self.last_rejected_reason)
                reset_for_discontinuity = True

            if jump > self.estimate_jump_reject_m:
                self.last_rejected_reason = f"estimate jumped {jump:.3f}m"
                reasons.append(self.last_rejected_reason)
                reset_for_discontinuity = True

            if reset_for_discontinuity:
                self.reset_estimate_window()

        if reasons:
            return False, reasons, stats

        self.estimate_window.append(estimate)
        self.last_accepted_estimate = estimate

        if len(self.estimate_window) < target_window_size:
            reasons.append(
                f"stability window filling {len(self.estimate_window)}/{target_window_size}"
            )
            return False, reasons, stats

        window_items = list(self.estimate_window)[-target_window_size:]
        xs = [item.center_x for item in window_items]
        ys = [item.center_y for item in window_items]
        yaws = [item.yaw for item in window_items]
        yaw_center = circular_mean(yaws)
        stats = {
            "std_x": std(xs),
            "std_y": std(ys),
            "std_yaw": circular_std_about(yaws, yaw_center),
        }

        if stats["std_x"] > self.estimate_max_std_xy:
            reasons.append(f"std_x {stats['std_x']:.4f} > {self.estimate_max_std_xy:.4f}")
        if stats["std_y"] > self.estimate_max_std_xy:
            reasons.append(f"std_y {stats['std_y']:.4f} > {self.estimate_max_std_xy:.4f}")
        if stats["std_yaw"] > self.estimate_max_std_yaw:
            reasons.append(
                f"std_yaw {math.degrees(stats['std_yaw']):.2f}deg > "
                f"{math.degrees(self.estimate_max_std_yaw):.2f}deg"
            )

        return not reasons, reasons, stats

    def publish_motion_or_zero(self, cmd: Twist, stable: bool, detail: str = ""):
        detail_suffix = f" | {detail}" if detail else ""
        if not self.enable_motion or not stable:
            self.stop_robot()
            self.throttled_cmd_info(
                "Command blocked; publishing zero cmd_vel. "
                f"enable_motion={self.enable_motion} stable={stable}{detail_suffix}"
            )
            return
        self.throttled_cmd_info(
            "Publishing cmd_vel "
            f"vx={cmd.linear.x:+.3f} vy={cmd.linear.y:+.3f} wz={cmd.angular.z:+.3f}"
            f"{detail_suffix}"
        )
        self.cmd_pub.publish(cmd)

    def publish_drive_under_cmd(self, cmd: Twist):
        self.cmd_pub.publish(cmd)
        if self.drive_under_extra_cmd_pub is not None:
            self.drive_under_extra_cmd_pub.publish(cmd)

    def get_base_pose_in_odom(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warn_time > 1.0:
                self.last_tf_warn_time = now
                self.get_logger().warn(
                    f"Drive-under TF failed: {self.odom_frame} -> {self.base_frame}: {exc}"
                )
            return None

        translation = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def set_lidar_ignore_radius(self, radius: float):
        msg = Float32()
        msg.data = max(0.0, float(radius))
        self.lidar_ignore_radius_pub.publish(msg)
        self.get_logger().info(f"Set lidar ignore radius to {msg.data:.2f} m.")

    def hold_success_or_finish(self, status: str, final_summary: Optional[str] = None):
        self.stop_robot()
        if self.aligned_since is None:
            self.aligned_since = time.monotonic()
            return
        if time.monotonic() - self.aligned_since >= self.success_hold_time:
            self.finish_mode(status, success=True, detail=final_summary)

    def log_estimate(
        self,
        label: str,
        estimate: Estimate,
        x_error: float,
        y_error: float,
        yaw_error: float,
        stable: bool,
        reasons: List[str],
        stats: Dict[str, float],
        detail: str = "",
    ):
        now = time.monotonic()
        if now - self.last_wait_log_time < 0.5:
            return
        self.last_wait_log_time = now

        cmd_gate = "enabled" if self.enable_motion else "disabled"
        self.get_logger().info(
            f"{label} estimate | ids={list(estimate.used_ids)} cameras={list(estimate.cameras)} "
            f"front={estimate.front_count} rear={estimate.rear_count} | "
            f"center_x={estimate.center_x:+.4f} center_y={estimate.center_y:+.4f} "
            f"yaw={math.degrees(estimate.yaw):+.2f}deg "
            f"orientation={trolley_orientation_label(estimate.yaw)} | "
            f"error_x={x_error:+.4f} error_y={y_error:+.4f} "
            f"yaw_error={math.degrees(yaw_error):+.2f}deg | "
            f"std_x={stats['std_x']:.4f} std_y={stats['std_y']:.4f} "
            f"std_yaw={math.degrees(stats['std_yaw']):.2f}deg | "
            f"stable={stable} motion={cmd_gate} | "
            f"reasons={'; '.join(reasons) if reasons else 'ok'}"
            f"{' | ' + detail if detail else ''}"
        )

    def publish_fine_debug(
        self,
        reason: str,
        estimate: Optional[Estimate] = None,
        cmd: Optional[Twist] = None,
        x_error: Optional[float] = None,
        y_error: Optional[float] = None,
        yaw_error: Optional[float] = None,
        stable: Optional[bool] = None,
        reasons: Optional[List[str]] = None,
    ):
        now = time.monotonic()
        if now - self.last_fine_debug_time < self.fine_debug_period_sec:
            return
        self.last_fine_debug_time = now

        target_x = self.target_offset_x
        target_y = self.target_offset_y
        estimate_text = "estimate=none"
        if estimate is not None:
            target_x, target_y = self.target_offsets_for_estimate(estimate)
            estimate_text = (
                f"ids={list(estimate.used_ids)} cameras={list(estimate.cameras)} "
                f"front={estimate.front_count}/{self.min_front_under_markers} "
                f"rear={estimate.rear_count}/{self.min_rear_under_markers} "
                f"center=({estimate.center_x:+.4f},{estimate.center_y:+.4f}) "
                f"yaw={math.degrees(estimate.yaw):+.2f}deg "
                f"orientation={trolley_orientation_label(estimate.yaw)}"
            )

        error_text = ""
        if x_error is not None and y_error is not None and yaw_error is not None:
            error_text = (
                f" error=({x_error:+.4f},{y_error:+.4f},"
                f"{math.degrees(yaw_error):+.2f}deg)"
            )

        cmd_text = "cmd=none"
        if cmd is not None:
            cmd_text = (
                f"cmd=({cmd.linear.x:+.3f},{cmd.linear.y:+.3f},"
                f"{cmd.angular.z:+.3f})"
            )

        stable_text = "" if stable is None else f" stable={stable}"
        reason_text = ""
        if reasons is not None:
            reason_text = f" reasons={'; '.join(reasons) if reasons else 'ok'}"

        text = (
            f"fine_debug reason={reason} | {estimate_text}{error_text} | "
            f"target=({target_x:+.4f},{target_y:+.4f}) "
            f"seq_stage={self.fine_sequential_stage} "
            f"active_pair={list(self.fine_active_marker_pair) if self.fine_active_marker_pair else []} "
            f"had_front_rear={self.fine_had_front_rear_markers} | "
            f"{cmd_text}{stable_text}{reason_text} | "
            f"{self.visible_under_marker_summary()} | "
            f"{self.camera_marker_debug_summary()}"
        )
        self.get_logger().info(text)
        self.publish_status(text)

    def publish_first_fine_estimate_status(
        self,
        estimate: Estimate,
        x_error: float,
        y_error: float,
        yaw_error: float,
        target_x: float,
        target_y: float,
    ):
        if self.fine_first_estimate_status_published:
            return

        self.fine_first_estimate_status_published = True
        text = (
            "fine_first_estimate | "
            f"ids={list(estimate.used_ids)} "
            f"center_x={estimate.center_x:+.4f} center_y={estimate.center_y:+.4f} "
            f"yaw={math.degrees(estimate.yaw):+.2f}deg "
            f"orientation={trolley_orientation_label(estimate.yaw)} "
            f"error_x={x_error:+.4f} error_y={y_error:+.4f} "
            f"yaw_error={math.degrees(yaw_error):+.2f}deg "
            f"target_x={target_x:+.4f} target_y={target_y:+.4f}"
        )
        self.get_logger().info(text)
        self.publish_status(text)

    def camera_marker_debug_summary(self) -> str:
        now = self.get_clock().now().nanoseconds / 1e9
        parts = []
        with self.marker_lock:
            marker_messages = dict(self.latest_marker_messages)

        for camera in ("front", "rear", "left", "right"):
            entry = marker_messages.get(camera)
            if entry is None:
                parts.append(f"{camera}=no_msg")
                continue
            stamp_sec, marker_ids, frame_id = entry
            age = now - stamp_sec
            fresh = age <= self.aruco_timeout
            underside_ids = [marker_id for marker_id in marker_ids if marker_id in self.underside_marker_ids]
            side_ids = [marker_id for marker_id in marker_ids if marker_id in self.side_marker_ids]
            state = "fresh" if fresh else "stale"
            parts.append(
                f"{camera}:{state}:{age:.1f}s ids={list(marker_ids)} "
                f"under={underside_ids} side={side_ids} frame={frame_id}"
            )

        return "camera_msgs[" + "; ".join(parts) + "]"

    def min_clamped_cmd(
        self,
        error: float,
        gain: float,
        limit: float,
        tolerance: float,
        minimum: float,
    ) -> float:
        if abs(error) <= tolerance or limit <= 0.0:
            return 0.0
        command = clamp(gain * error, limit)
        if minimum <= 0.0 or abs(command) >= minimum:
            return command
        return math.copysign(min(minimum, limit), command if command != 0.0 else error)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_camera_pair(self, camera_pair: str):
        msg = String()
        msg.data = camera_pair
        self.camera_pair_pub.publish(msg)
        self.last_camera_pair_request_time = time.monotonic()
        self.get_logger().info(f"Requested camera pair: {camera_pair}")

    def republish_camera_pair_if_due(self, camera_pair: str):
        if self.camera_pair_republish_sec <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_camera_pair_request_time >= self.camera_pair_republish_sec:
            self.publish_camera_pair(camera_pair)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        if self.drive_under_extra_cmd_pub is not None:
            self.drive_under_extra_cmd_pub.publish(Twist())

    def emergency_stop(self, status: str):
        self.stop_robot()
        self.set_lidar_ignore_radius(0.0)
        self.mode = "idle"
        self.mode_start_time = None
        self.sequence_active = False
        self.sequence_wait_until = None
        self.aligned_since = None
        self.acquire_started_at = None
        self.acquire_best_visible_count = 0
        self.acquire_blocked_until_count = None
        self.drive_under_start_pose = None
        self.drive_under_marker_seen_since = None
        self.side_target_pair_seen_fully = False
        self.side_active_target_pair = None
        self.fine_active_marker_pair = None
        self.fine_had_front_rear_markers = False
        self.fine_first_estimate_status_published = False
        self.fine_sequential_stage = "yaw"
        self.fine_step_state = "measure"
        self.fine_step_until = 0.0
        self.fine_step_cmd = Twist()
        self.fine_vx_direction = 1.0
        self.fine_vy_direction = 1.0
        self.fine_last_pulse_axis = None
        self.fine_last_pulse_abs_error = None
        self.reset_estimate_window()
        self.publish_status(status)
        self.get_logger().warn("Emergency/stop command received. Published zero Twist and returned to idle.")

    def clear_markers(self):
        with self.marker_lock:
            self.latest_markers.clear()
            self.latest_marker_messages.clear()
        self.reset_estimate_window()

    def reset_estimate_window(self):
        self.estimate_window.clear()
        self.last_accepted_estimate = None

    def clear_old_markers_only(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.marker_lock:
            self.latest_markers = {
                marker_id: observation
                for marker_id, observation in self.latest_markers.items()
                if now - observation.stamp_sec <= self.aruco_timeout
            }
            self.latest_marker_messages = {
                camera: entry
                for camera, entry in self.latest_marker_messages.items()
                if now - entry[0] <= self.aruco_timeout
            }

    def visible_marker_ids(self) -> List[int]:
        now = self.get_clock().now().nanoseconds / 1e9
        with self.marker_lock:
            return sorted(
                marker_id
                for marker_id, observation in self.latest_markers.items()
                if now - observation.stamp_sec <= self.aruco_timeout
            )

    def visible_under_marker_summary(self) -> str:
        observations = self.visible_under_observations()

        visible = sorted(observations.keys())
        front_count = sum(1 for observation in observations.values() if observation.camera == "front")
        rear_count = sum(1 for observation in observations.values() if observation.camera == "rear")
        return (
            f"visible={visible} total={len(visible)}/{self.min_under_markers} "
            f"front={front_count}/{self.min_front_under_markers} "
            f"rear={rear_count}/{self.min_rear_under_markers}"
        )

    def visible_under_marker_count(self) -> int:
        return len(self.visible_under_observations())

    def visible_under_observations(self) -> Dict[int, Observation]:
        now = self.get_clock().now().nanoseconds / 1e9
        with self.marker_lock:
            return {
                marker_id: observation
                for marker_id, observation in self.latest_markers.items()
                if (
                    now - observation.stamp_sec <= self.aruco_timeout
                    and marker_id in self.underside_marker_ids
                )
            }

    def timed_out(self, timeout: float) -> bool:
        return self.mode_start_time is not None and (time.monotonic() - self.mode_start_time) > timeout

    def finish_mode(self, status: str, success: bool, detail: Optional[str] = None):
        self.stop_robot()
        if self.mode == "drive_straight_under" or status.startswith("drive_straight_under"):
            self.set_lidar_ignore_radius(0.0)
            self.drive_under_start_pose = None
            self.drive_under_marker_seen_since = None
        old_mode = self.mode

        if success and self.sequence_active:
            if status == "side_align_left_done":
                self.clear_markers()
                self.start_mode(
                    "drive_straight_under",
                    "Staged docking: side alignment complete. Driving straight under using side markers.",
                )
                return
            if status == "drive_straight_under_done":
                self.clear_markers()
                self.publish_camera_pair("front_rear")
                self.mode = "wait_front_rear_camera"
                self.mode_start_time = time.monotonic()
                self.sequence_wait_until = time.monotonic() + self.camera_switch_settle_sec
                self.aligned_since = None
                self.publish_status("front_rear_camera_switch_started")
                self.get_logger().info(
                    "Staged docking: straight under complete. Switching to front/rear cameras "
                    "before final under-trolley alignment."
                )
                return
            if status == "fine_align_under_done":
                self.sequence_active = False

        self.mode = "idle"
        self.mode_start_time = None
        self.sequence_active = False
        self.sequence_wait_until = None
        self.aligned_since = None
        self.acquire_started_at = None
        self.acquire_best_visible_count = 0
        self.acquire_blocked_until_count = None
        self.fine_active_marker_pair = None
        self.fine_had_front_rear_markers = False
        self.fine_sequential_stage = "yaw"
        self.fine_step_state = "measure"
        self.fine_step_until = 0.0
        self.fine_step_cmd = Twist()
        self.reset_estimate_window()
        status_text = f"{status} | {detail}" if detail else status
        self.publish_status(status_text)
        if success:
            if detail:
                self.get_logger().info(
                    f"Mode finished successfully: {old_mode} -> {status} | {detail}. "
                    "Controller is idle; publishing zero cmd_vel until the next command."
                )
            else:
                self.get_logger().info(
                    f"Mode finished successfully: {old_mode} -> {status}. "
                    "Controller is idle; publishing zero cmd_vel until the next command."
                )
        else:
            self.get_logger().warn(f"Mode stopped: {old_mode} -> {status_text}")

    def throttled_info(self, text: str):
        now = time.monotonic()
        if now - self.last_wait_log_time >= 1.0:
            self.last_wait_log_time = now
            self.get_logger().info(text)

    def throttled_cmd_info(self, text: str):
        now = time.monotonic()
        if now - self.last_cmd_log_time >= 0.5:
            self.last_cmd_log_time = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalTrolleyAlignmentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
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
