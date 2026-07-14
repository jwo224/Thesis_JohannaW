#!/usr/bin/env python3

import os
from pathlib import Path
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from object_detection.msg import Yolov8Inference
except ImportError:
    Yolov8Inference = None


@dataclass
class Zone:
    key: str
    label: str
    x: float
    y: float
    yaw: float
    width: float
    height: float
    color: list


class DropzoneMission(Node):
    def __init__(self):
        super().__init__('dropzone_mission')

        self.declare_parameter('command_topic', '/trolley_command')
        self.declare_parameter('mission_status_topic', '/dropzone_mission_status')
        self.declare_parameter('marker_topic', '/delivery_zones')
        self.declare_parameter('goal_pose_topic', '/goal_pose_raw')
        self.declare_parameter('marker_publish_period', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('trolley_drive_wait_sec', 12.0)
        self.declare_parameter('trolley_stage_distance_m', 1.0)
        self.declare_parameter('trolley_stage_offset_x', 0.0)
        self.declare_parameter('trolley_stage_offset_y', 1.0)
        self.declare_parameter('trolley_stage_camera', 'left')
        self.declare_parameter('trolley_stage_width', 0.45)
        self.declare_parameter('trolley_stage_height', 0.35)
        self.declare_parameter('trolley_stage_color', [1.0, 0.9, 0.0, 0.45])
        self.declare_parameter('alignment_command_topic', '/physical_docking_command')
        self.declare_parameter('alignment_status_topic', '/physical_docking_status')
        self.declare_parameter('side_align_command', 'side_align_left')
        self.declare_parameter('final_drive_command', 'drive_straight_under')
        self.declare_parameter('fine_align_command', 'fine_align_under')
        self.declare_parameter('trolley_detection_topic', '/Yolov8_Inference')
        self.declare_parameter('trolley_detection_command_topic', '/yolov8_detector_command')
        self.declare_parameter('trolley_detection_topic_timeout_sec', 3.0)
        self.declare_parameter('trolley_detection_memory_sec', 30.0)
        self.declare_parameter('trolley_detection_auto_pause_on_first_detection', True)
        self.declare_parameter('allow_image_file_detection_fallback', False)
        self.declare_parameter(
            'detection_model_path',
            '~/gr4_ws/src/object_detection/scripts/trolleys.pt',
        )
        self.declare_parameter(
            'test_image_path',
            '~/gr4_ws/src/object_detection/test_images/current_trolley.jpg',
        )
        self.declare_parameter(
            'test_image_directory',
            '~/gr4_ws/src/object_detection/test_images',
        )
        self.declare_parameter('detection_confidence', 0.25)
        self.declare_parameter('dropzone_exit_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('dropzone_exit_extra_cmd_vel_topic', '/cmd_vel_joy')
        self.declare_parameter('dropzone_exit_vy', -0.07)
        self.declare_parameter('dropzone_exit_vx', 0.0)
        self.declare_parameter('dropzone_exit_wz', 0.0)
        self.declare_parameter('dropzone_exit_duration_sec', 7.0)
        self.declare_parameter('dropzone_exit_rate_hz', 10.0)
        self.declare_parameter('zones.charging.x', 0.0)
        self.declare_parameter('zones.charging.y', 0.0)
        self.declare_parameter('zones.charging.yaw', 0.0)
        self.declare_parameter('zones.charging.width', 0.8)
        self.declare_parameter('zones.charging.height', 0.8)
        self.declare_parameter('zones.charging.color', [0.65, 0.65, 0.65, 0.35])
        self.declare_parameter('zones.trolley_dropzone.x', 0.8)
        self.declare_parameter('zones.trolley_dropzone.y', 0.0)
        self.declare_parameter('zones.trolley_dropzone.yaw', 0.0)
        self.declare_parameter('zones.trolley_dropzone.width', 0.9)
        self.declare_parameter('zones.trolley_dropzone.height', 0.9)
        self.declare_parameter('zones.trolley_dropzone.color', [0.0, 0.8, 0.25, 0.35])
        self.declare_parameter('zones.laundry_dropoff.x', 1.8)
        self.declare_parameter('zones.laundry_dropoff.y', -0.8)
        self.declare_parameter('zones.laundry_dropoff.yaw', -1.57)
        self.declare_parameter('zones.laundry_dropoff.width', 1.0)
        self.declare_parameter('zones.laundry_dropoff.height', 1.0)
        self.declare_parameter('zones.laundry_dropoff.color', [0.0, 0.35, 1.0, 0.35])
        self.declare_parameter('zones.trash_dropoff.x', 1.8)
        self.declare_parameter('zones.trash_dropoff.y', 0.8)
        self.declare_parameter('zones.trash_dropoff.yaw', 1.57)
        self.declare_parameter('zones.trash_dropoff.width', 1.0)
        self.declare_parameter('zones.trash_dropoff.height', 1.0)
        self.declare_parameter('zones.trash_dropoff.color', [1.0, 0.25, 0.0, 0.35])

        self.frame_id = self.get_parameter('frame_id').value
        self.zones = self.load_zones()
        self.command_aliases = self.make_command_aliases()
        self.pending_detection_timer = None
        self.pending_trolley_timer = None
        self.dropzone_alignment_active = False
        self.dropzone_alignment_phase = 'idle'
        self.current_side_align_command = self.get_parameter('side_align_command').value
        self.detected_trolley_type = None
        self.detected_trolley_source = None
        self.latest_detection_classes = []
        self.latest_detection_stamp = None
        self.last_live_trolley_type = None
        self.last_live_detection_classes = []
        self.last_live_detection_stamp = None
        self.trolley_detection_sub = None
        self.trolley_detection_enabled = False
        self.exit_timer = None
        self.exit_started_at = None

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('marker_topic').value,
            marker_qos,
        )

        self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self.command_callback,
            10,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('goal_pose_topic').value,
            10,
        )
        self.mission_status_pub = self.create_publisher(
            String,
            self.get_parameter('mission_status_topic').value,
            10,
        )
        self.exit_cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('dropzone_exit_cmd_vel_topic').value,
            10,
        )
        exit_extra_topic = str(self.get_parameter('dropzone_exit_extra_cmd_vel_topic').value)
        self.exit_extra_cmd_pub = None
        if exit_extra_topic:
            self.exit_extra_cmd_pub = self.create_publisher(Twist, exit_extra_topic, 10)
        self.alignment_command_pub = self.create_publisher(
            String,
            self.get_parameter('alignment_command_topic').value,
            10,
        )
        detection_command_qos = QoSProfile(depth=1)
        detection_command_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trolley_detection_command_pub = self.create_publisher(
            String,
            self.get_parameter('trolley_detection_command_topic').value,
            detection_command_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter('alignment_status_topic').value,
            self.alignment_status_callback,
            10,
        )
        if Yolov8Inference is not None:
            self.trolley_detection_sub = self.create_subscription(
                Yolov8Inference,
                self.get_parameter('trolley_detection_topic').value,
                self.trolley_detection_callback,
                10,
            )
        else:
            self.get_logger().warn(
                'object_detection messages are not available; check_trolley_type will use image-file YOLO only.'
            )

        period = float(self.get_parameter('marker_publish_period').value)
        self.create_timer(period, self.publish_markers)
        self.publish_markers()

        self.get_logger().info(
            'Dropzone mission ready. Commands on /trolley_command: '
            'charging, trolley_dropzone, trolley_ready, side_align, check_trolley_type, '
            'drive_under, fine_align, deliver_trolley, drive_out, laundry_dropoff, trash_dropoff'
        )
        self.publish_mission_status(
            'idle. Send trolley_ready to drive to the trolley approach pose.'
        )

    def load_zones(self):
        labels = {
            'charging': 'CHARGING',
            'trolley_dropzone': 'TROLLEY DROPZONE',
            'laundry_dropoff': 'LAUNDRY DROPOFF',
            'trash_dropoff': 'TRASH DROPOFF',
        }
        zones = {}
        for key, label in labels.items():
            prefix = f'zones.{key}'
            zones[key] = Zone(
                key=key,
                label=label,
                x=float(self.get_parameter(f'{prefix}.x').value),
                y=float(self.get_parameter(f'{prefix}.y').value),
                yaw=float(self.get_parameter(f'{prefix}.yaw').value),
                width=float(self.get_parameter(f'{prefix}.width').value),
                height=float(self.get_parameter(f'{prefix}.height').value),
                color=list(self.get_parameter(f'{prefix}.color').value),
            )
        return zones

    def make_command_aliases(self):
        return {
            'charge': 'charging',
            'charging': 'charging',
            'home': 'charging',
            'dock': 'charging',
            'trolley': 'trolley_dropzone',
            'trolley_drop': 'trolley_dropzone',
            'trolley_dropzone': 'trolley_dropzone',
            'pickup': 'trolley_dropzone',
            'laundry': 'laundry_dropoff',
            'laundry_drop': 'laundry_dropoff',
            'laundry_dropoff': 'laundry_dropoff',
            'laundry_drop_off': 'laundry_dropoff',
            'trash': 'trash_dropoff',
            'trash_drop': 'trash_dropoff',
            'trash_dropoff': 'trash_dropoff',
            'trash_drop_off': 'trash_dropoff',
            'trash_trolley': 'trash_dropoff',
        }

    def command_callback(self, msg):
        command = msg.data.strip().lower().replace(' ', '_').replace('-', '_')
        if command in ('cancel', 'stop'):
            self.cancel_pending_trolley_flow()
            self.publish_alignment_command('stop')
            self.set_trolley_detection_enabled(False, 'stop_command')
            self.publish_mission_status('stopped. Alignment controller was commanded to stop.')
            self.get_logger().warn('Stopped staged dropzone flow and physical alignment controller.')
            self.get_logger().warn('Use RViz cancel or a Nav2 action cancel if a Nav2 goal is still active.')
            return

        if command in ('status', 'mission_status'):
            self.publish_current_stage_status()
            return

        if command in ('trolley_ready', 'approach_trolley', 'approach_dropzone'):
            self.start_trolley_ready_flow()
            return

        if command in ('side_align', 'side_alignment', 'align_side', 'side_align_left', 'side_align_right'):
            self.approve_side_alignment(command)
            return

        if command in ('drive_under', 'drive_straight_under', 'under_trolley'):
            self.approve_drive_under()
            return

        if command in (
            'check_trolley_type',
            'detect_trolley_type',
            'classify_trolley',
            'trolley_type',
        ):
            self.check_trolley_type()
            return

        if command in (
            'fine_align',
            'fine_alignment',
            'fine_align_under',
            'under_trolley_align',
            'restart_fine_align',
            'retry_fine_align',
        ):
            self.approve_fine_alignment()
            return

        if command in (
            'deliver_trolley',
            'deliver_detected',
            'drive_to_detected_dropzone',
            'go_to_detected_dropzone',
        ):
            self.deliver_detected_trolley()
            return

        if command in (
            'drive_out',
            'drive_straight_out',
            'exit_trolley',
            'back_out',
            'leave_trolley',
        ):
            self.start_dropzone_exit()
            return

        if command in ('clear_trolley_type', 'forget_trolley_type'):
            self.clear_detected_trolley_type()
            return

        if command in ('detect_trolley', 'detect_and_deliver'):
            self.start_detect_and_deliver()
            return

        if command == 'laundry_trolley':
            self.remember_manual_trolley_type('laundry_trolley')
            return

        if command == 'trash_trolley':
            self.remember_manual_trolley_type('trash_trolley')
            return

        if command == 'empty_trolley':
            self.handle_empty_trolley_detection('manual_command')
            return

        zone_key = self.command_aliases.get(command)
        if zone_key is None:
            self.get_logger().warn(
                f'Unknown command "{command}". Valid commands: '
                'charging, trolley_dropzone, trolley_ready, side_align, check_trolley_type, '
                'drive_under, fine_align, deliver_trolley, drive_out, laundry_dropoff, '
                'trash_dropoff, status, cancel'
            )
            return

        self.send_goal(self.zones[zone_key])

    def start_trolley_ready_flow(self):
        self.cancel_pending_trolley_flow()
        self.clear_detected_trolley_type(publish=False)

        stage_zone = self.make_staging_zone(self.zones['trolley_dropzone'])
        offset_x = float(self.get_parameter('trolley_stage_offset_x').value)
        offset_y = float(self.get_parameter('trolley_stage_offset_y').value)
        stage_camera = self.get_parameter('trolley_stage_camera').value
        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'approach_goal_sent'

        self.get_logger().info(
            f'Trolley ready: sending staged goal at map offset '
            f'({offset_x:+.2f}, {offset_y:+.2f})m from trolley dropzone. '
            f'Approach yaw={stage_zone.yaw:+.2f}rad so the {stage_camera} camera faces the trolley zone.'
        )
        self.send_goal(stage_zone)
        self.publish_mission_status(
            'approach_goal_sent. Inspect the robot pose, then approve side alignment with: '
            "ros2 topic pub --once /trolley_command std_msgs/msg/String \"{data: 'side_align'}\""
        )

    def approve_side_alignment(self, command):
        if self.dropzone_alignment_phase not in ('approach_goal_sent', 'side_align_complete', 'idle'):
            self.reject_out_of_order_command('side_align')
            return

        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'side_aligning'
        side_command = self.resolve_side_align_command(command)
        self.current_side_align_command = side_command
        self.publish_alignment_command(side_command)
        self.get_logger().info(
            f'Staged dropzone: requested physical side alignment with command "{side_command}". '
            'Waiting for alignment status before accepting drive_under approval.'
        )
        self.publish_mission_status(
            f'side_aligning via {side_command}. Waiting for {side_command}_done.'
        )

    def approve_drive_under(self):
        if self.dropzone_alignment_phase not in ('side_align_complete', 'trolley_type_checked', 'idle'):
            self.reject_out_of_order_command('drive_under')
            return

        drive_command = self.get_parameter('final_drive_command').value
        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'straight_drive'
        self.set_trolley_detection_enabled(False, 'drive_under_started')
        self.publish_alignment_command(drive_command)
        self.get_logger().info(
            f'Staged dropzone: requested straight drive under trolley with command "{drive_command}".'
        )
        self.publish_mission_status(
            f'drive_under_active via {drive_command}. Waiting for {drive_command}_done.'
        )

    def approve_fine_alignment(self):
        if self.dropzone_alignment_phase not in ('straight_drive_complete', 'fine_aligning', 'fine_align_complete', 'idle'):
            self.reject_out_of_order_command('fine_align')
            return

        fine_command = self.get_parameter('fine_align_command').value
        restarting = self.dropzone_alignment_phase == 'fine_aligning'
        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'fine_aligning'
        self.publish_alignment_command(fine_command)
        if restarting:
            self.get_logger().info(
                f'Staged dropzone: restarting final under-trolley ArUco alignment '
                f'with command "{fine_command}".'
            )
        else:
            self.get_logger().info(
                f'Staged dropzone: requested final under-trolley ArUco alignment '
                f'with command "{fine_command}".'
            )
        self.publish_mission_status(
            f'fine_aligning via {fine_command}. Waiting for {fine_command}_done.'
        )

    def trolley_detection_callback(self, msg):
        now = time.monotonic()
        self.latest_detection_classes = [
            str(result.class_name).strip()
            for result in msg.yolov8_inference
            if str(result.class_name).strip()
        ]
        self.latest_detection_stamp = now

        trolley_type = self.most_common_trolley_type(self.latest_detection_classes)
        if trolley_type is None:
            # Empty YOLO frames are normal between detections. Keep the last
            # non-empty live result until trolley_detection_memory_sec expires.
            return

        self.last_live_trolley_type = trolley_type
        self.last_live_detection_classes = list(self.latest_detection_classes)
        self.last_live_detection_stamp = now
        # Detection only records observations. The explicit check_trolley_type
        # command decides the result, stops YOLO, and handles an empty trolley.

    def normalize_trolley_type(self, class_name):
        text = str(class_name).strip().lower()
        if not text:
            return None
        if 'laundry' in text:
            return 'laundry_trolley'
        if 'trash' in text:
            return 'trash_trolley'
        if 'empty' in text:
            return 'empty_trolley'
        return None

    def latest_topic_trolley_type(self):
        if self.last_live_detection_stamp is None:
            return None

        timeout = float(self.get_parameter('trolley_detection_memory_sec').value)
        age = time.monotonic() - self.last_live_detection_stamp
        if age > timeout:
            return None

        return self.last_live_trolley_type

    def most_common_trolley_type(self, class_names):
        counts = {}
        for class_name in class_names:
            trolley_type = self.normalize_trolley_type(class_name)
            if trolley_type is None:
                continue
            counts[trolley_type] = counts.get(trolley_type, 0) + 1

        if not counts:
            return None

        return max(counts.items(), key=lambda item: item[1])[0]

    def check_trolley_type(self):
        if self.dropzone_alignment_phase not in (
            'side_align_complete',
            'trolley_type_checked',
            'straight_drive_complete',
            'fine_align_complete',
            'idle',
        ):
            self.reject_out_of_order_command('check_trolley_type')
            return

        trolley_type = self.latest_topic_trolley_type()
        source = 'live_yolov8_topic'
        if trolley_type is None:
            memory_sec = float(self.get_parameter('trolley_detection_memory_sec').value)
            self.detected_trolley_type = None
            self.detected_trolley_source = None
            self.publish_mission_status(
                'trolley_type_unknown_live_yolo. No current non-empty /Yolov8_Inference '
                f'detection in the last {memory_sec:.1f}s; not using image-file fallback. '
                'YOLO is still enabled; wait a moment and send check_trolley_type again.'
            )
            self.get_logger().warn(
                'No current non-empty live YOLO trolley detection; refusing image-file fallback. '
                f'latest_classes={self.latest_detection_classes} '
                f'last_live={self.last_live_trolley_type} '
                f'last_live_classes={self.last_live_detection_classes}'
            )
            return

        if trolley_type == 'empty_trolley':
            self.set_trolley_detection_enabled(False, 'empty_trolley_checked')
            self.handle_empty_trolley_detection(source)
            return

        if trolley_type not in ('laundry_trolley', 'trash_trolley'):
            self.detected_trolley_type = trolley_type
            self.detected_trolley_source = source
            self.set_trolley_detection_enabled(False, 'unknown_trolley_checked')
            self.publish_mission_status(
                f'trolley_type_unknown: {trolley_type}. Fix detection or manually send laundry_trolley/trash_trolley.'
            )
            self.get_logger().warn(
                f'Trolley type check did not find laundry/trash. result={trolley_type} source={source}'
            )
            return

        self.detected_trolley_type = trolley_type
        self.detected_trolley_source = source
        self.set_trolley_detection_enabled(False, f'{trolley_type}_checked')
        self.dropzone_alignment_active = True
        if self.dropzone_alignment_phase in ('side_align_complete', 'idle'):
            self.dropzone_alignment_phase = 'trolley_type_checked'

        dropzone_key = self.dropzone_for_trolley_type(trolley_type)
        next_command = 'deliver_trolley' if self.dropzone_alignment_phase == 'fine_align_complete' else 'drive_under'
        if self.dropzone_alignment_phase == 'straight_drive_complete':
            next_command = 'fine_align'
        self.publish_mission_status(
            f'trolley_type_checked: {trolley_type} via {source}; remembered target={dropzone_key}. '
            f'Next command: {next_command}.'
        )
        self.get_logger().info(
            f'Remembered trolley type {trolley_type} from {source}; target dropzone={dropzone_key}.'
        )

    def dropzone_for_trolley_type(self, trolley_type):
        if trolley_type == 'laundry_trolley':
            return 'laundry_dropoff'
        if trolley_type == 'trash_trolley':
            return 'trash_dropoff'
        return None

    def handle_empty_trolley_detection(self, source):
        self.detected_trolley_type = 'empty_trolley'
        self.detected_trolley_source = source
        self.set_trolley_detection_enabled(False, 'empty_trolley_detection')
        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'returning_to_charging'
        self.get_logger().warn(
            f'EMPTY TROLLEY DETECTED via {source}. Returning to charging instead of docking/delivering.'
        )
        self.send_goal(self.zones['charging'])
        self.publish_mission_status(
            f'WARNING: empty_trolley_detected via {source}. Sent charging goal; '
            'pickup/delivery cancelled. Waiting for the next trolley_ready command.'
        )

    def deliver_detected_trolley(self):
        dropzone_key = self.dropzone_for_trolley_type(self.detected_trolley_type)
        if dropzone_key is None:
            self.publish_mission_status(
                'cannot_deliver_trolley_type_unknown. Send check_trolley_type, laundry_trolley, or trash_trolley first.'
            )
            self.get_logger().warn('Cannot deliver: no remembered laundry/trash trolley type.')
            return

        if self.dropzone_alignment_phase not in ('fine_align_complete', 'dropoff_goal_sent', 'idle'):
            self.reject_out_of_order_command('deliver_trolley')
            return

        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'dropoff_goal_sent'
        self.send_goal(self.zones[dropzone_key])
        self.publish_mission_status(
            f'dropoff_goal_sent: remembered {self.detected_trolley_type} -> {dropzone_key}. '
            'When the robot arrives, approve trolley release with: '
            "ros2 topic pub --once /trolley_command std_msgs/msg/String \"{data: 'drive_out'}\""
        )

    def remember_manual_trolley_type(self, trolley_type):
        self.detected_trolley_type = trolley_type
        self.detected_trolley_source = 'manual_command'
        self.dropzone_alignment_active = True
        if self.dropzone_alignment_phase in ('side_align_complete', 'idle'):
            self.dropzone_alignment_phase = 'trolley_type_checked'

        dropzone_key = self.dropzone_for_trolley_type(trolley_type)
        next_command = 'deliver_trolley' if self.dropzone_alignment_phase == 'fine_align_complete' else 'drive_under'
        if self.dropzone_alignment_phase == 'straight_drive_complete':
            next_command = 'fine_align'
        self.publish_mission_status(
            f'trolley_type_checked: manually remembered {trolley_type}; target={dropzone_key}. '
            f'Next command: {next_command}.'
        )
        self.get_logger().info(
            f'Manually remembered trolley type {trolley_type}; target dropzone={dropzone_key}.'
        )

    def clear_detected_trolley_type(self, publish=True):
        self.detected_trolley_type = None
        self.detected_trolley_source = None
        self.latest_detection_classes = []
        self.latest_detection_stamp = None
        self.last_live_trolley_type = None
        self.last_live_detection_classes = []
        self.last_live_detection_stamp = None
        if publish:
            self.publish_mission_status('trolley_type_cleared.')

    def set_trolley_detection_enabled(self, enabled, reason):
        if self.trolley_detection_enabled == enabled:
            return

        self.trolley_detection_enabled = enabled
        msg = String()
        msg.data = 'enable' if enabled else 'disable'
        self.trolley_detection_command_pub.publish(msg)
        state = 'enabled' if enabled else 'disabled'
        self.get_logger().info(
            f'YOLO trolley detector {state} via '
            f'{self.get_parameter("trolley_detection_command_topic").value}; '
            f'reason={reason}'
        )

    def start_dropzone_exit(self):
        if self.dropzone_alignment_phase not in ('dropoff_goal_sent', 'idle'):
            self.reject_out_of_order_command('drive_out')
            return

        duration = float(self.get_parameter('dropzone_exit_duration_sec').value)
        rate_hz = float(self.get_parameter('dropzone_exit_rate_hz').value)
        if duration <= 0.0 or rate_hz <= 0.0:
            self.publish_mission_status(
                'drive_out_ignored. dropzone_exit_duration_sec and dropzone_exit_rate_hz must be > 0.'
            )
            self.get_logger().warn('Drive-out ignored because exit duration/rate is invalid.')
            return

        self.stop_dropzone_exit(publish_status=False)
        self.dropzone_alignment_active = True
        self.dropzone_alignment_phase = 'driving_out'
        self.exit_started_at = time.monotonic()
        self.exit_timer = self.create_timer(1.0 / rate_hz, self.run_dropzone_exit)
        self.publish_mission_status(
            f'drive_out_active for {duration:.1f}s. Publishing direct cmd_vel to release trolley.'
        )
        self.get_logger().info(
            f'Starting direct drive-out for {duration:.1f}s: '
            f'vx={float(self.get_parameter("dropzone_exit_vx").value):+.3f} '
            f'vy={float(self.get_parameter("dropzone_exit_vy").value):+.3f} '
            f'wz={float(self.get_parameter("dropzone_exit_wz").value):+.3f}'
        )

    def run_dropzone_exit(self):
        if self.exit_started_at is None:
            self.stop_dropzone_exit()
            return

        duration = float(self.get_parameter('dropzone_exit_duration_sec').value)
        elapsed = time.monotonic() - self.exit_started_at
        if elapsed >= duration:
            self.stop_dropzone_exit()
            released_trolley_type = self.detected_trolley_type or "trolley"
            self.clear_detected_trolley_type(publish=False)
            self.dropzone_alignment_active = False
            self.dropzone_alignment_phase = 'idle'
            self.publish_mission_status(
                f'drive_out_done. Released {released_trolley_type}; cleared trolley type. '
                'Next command: trolley_ready.'
            )
            return

        cmd = Twist()
        cmd.linear.x = float(self.get_parameter('dropzone_exit_vx').value)
        cmd.linear.y = float(self.get_parameter('dropzone_exit_vy').value)
        cmd.angular.z = float(self.get_parameter('dropzone_exit_wz').value)
        self.publish_exit_cmd(cmd)

    def publish_exit_cmd(self, cmd):
        self.exit_cmd_pub.publish(cmd)
        if self.exit_extra_cmd_pub is not None:
            self.exit_extra_cmd_pub.publish(cmd)

    def stop_dropzone_exit(self, publish_status=False):
        if self.exit_timer is not None:
            self.exit_timer.cancel()
            self.exit_timer = None
        self.exit_started_at = None
        self.publish_exit_cmd(Twist())
        if publish_status:
            self.publish_mission_status('drive_out_stopped.')

    def alignment_status_callback(self, msg):
        if not self.dropzone_alignment_active:
            return

        status = msg.data.strip()
        if self.dropzone_alignment_phase == 'side_aligning':
            side_done_status = f'{self.current_side_align_command}_done'
            if status.startswith(side_done_status):
                self.dropzone_alignment_phase = 'side_align_complete'
                self.clear_detected_trolley_type(publish=False)
                self.set_trolley_detection_enabled(True, 'side_align_complete')
                self.get_logger().info(
                    'Staged dropzone: side alignment complete. Waiting for type check or drive_under approval.'
                )
                self.publish_mission_status(
                    f'side_align_complete: {status}. Optional: check trolley type with '
                    "ros2 topic pub --once /trolley_command std_msgs/msg/String \"{data: 'check_trolley_type'}\" "
                    'or approve straight drive with '
                    "ros2 topic pub --once /trolley_command std_msgs/msg/String \"{data: 'drive_under'}\""
                )
                return
            if 'timeout' in status or status.startswith('stopped'):
                self.dropzone_alignment_active = False
                self.dropzone_alignment_phase = 'idle'
                self.set_trolley_detection_enabled(False, 'side_align_stopped')
                self.publish_mission_status(f'side_alignment_stopped: {status}')
                self.get_logger().warn(f'Staged dropzone side alignment stopped: {status}')
                return

        if self.dropzone_alignment_phase == 'straight_drive':
            drive_done_status = f'{self.get_parameter("final_drive_command").value}_done'
            if status.startswith(drive_done_status):
                self.dropzone_alignment_phase = 'straight_drive_complete'
                self.get_logger().info(
                    'Staged dropzone: robot is under the trolley. Waiting for fine_align approval.'
                )
                self.publish_mission_status(
                    'under_trolley. Inspect position, then approve final ArUco alignment with: '
                    "ros2 topic pub --once /trolley_command std_msgs/msg/String \"{data: 'fine_align'}\""
                )
                return
            if 'timeout' in status or status.startswith('stopped'):
                self.dropzone_alignment_active = False
                self.dropzone_alignment_phase = 'idle'
                self.publish_mission_status(f'drive_under_stopped: {status}')
                self.get_logger().warn(f'Staged dropzone final drive stopped: {status}')
                return

        if self.dropzone_alignment_phase == 'fine_aligning':
            fine_done_status = f'{self.get_parameter("fine_align_command").value}_done'
            if status.startswith('fine_first_estimate'):
                self.publish_mission_status(f'fine_align_first_aruko: {status}')
                return
            if status.startswith(fine_done_status):
                self.dropzone_alignment_phase = 'fine_align_complete'
                self.get_logger().info(
                    f'Staged dropzone fine alignment complete: {status}'
                )
                if self.dropzone_for_trolley_type(self.detected_trolley_type) is not None:
                    self.publish_mission_status(
                        f'fine_align_complete: {status}. Remembered {self.detected_trolley_type}; '
                        'approve delivery with '
                        "ros2 topic pub --once /trolley_command std_msgs/msg/String \"{data: 'deliver_trolley'}\""
                    )
                else:
                    self.publish_mission_status(
                        f'fine_align_complete: {status}. Trolley type unknown; send check_trolley_type '
                        'or manually send laundry_trolley/trash_trolley before deliver_trolley.'
                    )
                return
            if 'timeout' in status or status.startswith('stopped'):
                self.dropzone_alignment_active = False
                self.dropzone_alignment_phase = 'idle'
                self.publish_mission_status(f'fine_alignment_stopped: {status}')
                self.get_logger().warn(f'Staged dropzone fine alignment stopped: {status}')

    def publish_alignment_command(self, command):
        msg = String()
        msg.data = command
        self.alignment_command_pub.publish(msg)

    def publish_mission_status(self, text):
        msg = String()
        msg.data = text
        self.mission_status_pub.publish(msg)
        self.get_logger().info(f'Mission status: {text}')

    def publish_current_stage_status(self):
        status_text = f'phase={self.dropzone_alignment_phase}'
        if self.detected_trolley_type:
            status_text += (
                f'; remembered_trolley_type={self.detected_trolley_type}'
                f' source={self.detected_trolley_source}'
            )
        if self.dropzone_alignment_phase == 'approach_goal_sent':
            status_text += '; next command: side_align'
        elif self.dropzone_alignment_phase == 'side_align_complete':
            status_text += '; next command: check_trolley_type or drive_under'
        elif self.dropzone_alignment_phase == 'trolley_type_checked':
            status_text += '; next command: drive_under'
        elif self.dropzone_alignment_phase == 'straight_drive_complete':
            status_text += '; next command: fine_align'
        elif self.dropzone_alignment_phase == 'fine_align_complete':
            status_text += '; next command: deliver_trolley'
        elif self.dropzone_alignment_phase == 'dropoff_goal_sent':
            status_text += '; next command after arrival: drive_out'
        elif self.dropzone_alignment_phase == 'driving_out':
            status_text += '; direct drive_out active'
        elif self.dropzone_alignment_phase == 'returning_to_charging':
            status_text += '; empty trolley detected; charging goal sent'
        elif self.dropzone_alignment_phase == 'idle':
            status_text += '; next command: trolley_ready'
        self.publish_mission_status(status_text)

    def reject_out_of_order_command(self, command):
        status_text = (
            f'ignored {command}; current phase={self.dropzone_alignment_phase}. '
            'Send status to see the next expected command, or stop to cancel.'
        )
        self.publish_mission_status(status_text)
        self.get_logger().warn(status_text)

    def resolve_side_align_command(self, command):
        if command.endswith('_left') or command.startswith('left_'):
            return 'side_align_left'
        if command.endswith('_right') or command.startswith('right_'):
            return 'side_align_right'
        return self.get_parameter('side_align_command').value

    def cancel_pending_trolley_flow(self):
        if self.pending_trolley_timer is not None:
            self.pending_trolley_timer.cancel()
            self.pending_trolley_timer = None
        if self.pending_detection_timer is not None:
            self.pending_detection_timer.cancel()
            self.pending_detection_timer = None
        self.stop_dropzone_exit(publish_status=False)
        self.set_trolley_detection_enabled(False, 'cancel_pending_trolley_flow')
        self.dropzone_alignment_active = False
        self.dropzone_alignment_phase = 'idle'

    def make_staging_zone(self, zone):
        offset_x = float(self.get_parameter('trolley_stage_offset_x').value)
        offset_y = float(self.get_parameter('trolley_stage_offset_y').value)
        stage_x = zone.x + offset_x
        stage_y = zone.y + offset_y
        stage_yaw = self.make_stage_yaw(zone, stage_x, stage_y)
        return Zone(
            key=f'{zone.key}_stage',
            label='TROLLEY APPROACH',
            x=stage_x,
            y=stage_y,
            yaw=stage_yaw,
            width=float(self.get_parameter('trolley_stage_width').value),
            height=float(self.get_parameter('trolley_stage_height').value),
            color=list(self.get_parameter('trolley_stage_color').value),
        )

    def make_stage_yaw(self, zone, stage_x, stage_y):
        camera = str(self.get_parameter('trolley_stage_camera').value).strip().lower()
        dx = zone.x - stage_x
        dy = zone.y - stage_y
        if math.hypot(dx, dy) < 1e-6:
            return zone.yaw

        bearing_to_zone = math.atan2(dy, dx)
        if camera == 'left':
            return self.normalize_angle(bearing_to_zone - math.pi / 2.0)
        if camera == 'right':
            return self.normalize_angle(bearing_to_zone + math.pi / 2.0)
        if camera == 'front':
            return self.normalize_angle(bearing_to_zone)
        if camera == 'rear':
            return self.normalize_angle(bearing_to_zone + math.pi)
        if camera in ('zone', 'dropzone', 'configured'):
            return zone.yaw

        self.get_logger().warn(
            f'Unknown trolley_stage_camera "{camera}". Using trolley_dropzone yaw.'
        )
        return zone.yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def start_detect_and_deliver(self):
        self.cancel_pending_trolley_flow()
        if self.pending_detection_timer is not None:
            self.pending_detection_timer.cancel()

        self.get_logger().info('Detection flow: driving to trolley dropzone before detection.')
        self.send_goal(self.zones['trolley_dropzone'])

        wait_sec = float(self.get_parameter('trolley_drive_wait_sec').value)
        self.get_logger().info(f'Waiting {wait_sec:.1f}s before classifying trolley test image.')
        self.pending_detection_timer = self.create_timer(wait_sec, self.detect_and_deliver_once)

    def detect_and_deliver_once(self):
        if self.pending_detection_timer is not None:
            self.pending_detection_timer.cancel()
            self.pending_detection_timer = None

        trolley_type = self.detect_trolley_type()
        if trolley_type == 'laundry_trolley':
            self.get_logger().info('Detected laundry trolley; going to laundry dropoff.')
            self.send_goal(self.zones['laundry_dropoff'])
        elif trolley_type == 'trash_trolley':
            self.get_logger().info('Detected trash trolley; going to trash dropoff.')
            self.send_goal(self.zones['trash_dropoff'])
        elif trolley_type == 'empty_trolley':
            self.handle_empty_trolley_detection('image_file_yolo')
        else:
            self.get_logger().warn(
                f'Trolley detection returned "{trolley_type}". Returning to charging.'
            )
            self.send_goal(self.zones['charging'])

    def detect_trolley_type(self):
        image_path = self.resolve_test_image_path()
        if image_path is None:
            self.get_logger().error('No test image found for trolley detection.')
            return 'unknown_trolley'

        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.get_logger().warn(
                f'Could not import ultralytics ({exc}); cannot detect trolley type.'
            )
            return 'unknown_trolley'

        model_path = Path(os.path.expanduser(self.get_parameter('detection_model_path').value))
        if not model_path.exists():
            self.get_logger().warn(
                f'Detection model not found at {model_path}; cannot detect trolley type.'
            )
            return 'unknown_trolley'

        try:
            model = YOLO(str(model_path))
            results = model(str(image_path), verbose=False)
        except Exception as exc:
            self.get_logger().warn(
                f'YOLO detection failed for {image_path}: {exc}.'
            )
            return 'unknown_trolley'

        best_name = None
        best_conf = -1.0
        min_conf = float(self.get_parameter('detection_confidence').value)

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = str(model.names[class_id]).lower()
                if confidence > best_conf:
                    best_conf = confidence
                    best_name = class_name

        self.get_logger().info(
            f'Trolley image {image_path} classified by YOLO as {best_name} '
            f'with confidence {best_conf:.2f}.'
        )

        if best_name is None or best_conf < min_conf:
            return 'unknown_trolley'
        if 'laundry' in best_name:
            return 'laundry_trolley'
        if 'trash' in best_name:
            return 'trash_trolley'
        if 'empty' in best_name:
            return 'empty_trolley'
        return 'unknown_trolley'

    def resolve_test_image_path(self):
        configured_path = Path(os.path.expanduser(self.get_parameter('test_image_path').value))
        if configured_path.exists():
            return configured_path

        image_dir = Path(os.path.expanduser(self.get_parameter('test_image_directory').value))
        if not image_dir.exists():
            return None

        image_paths = []
        for pattern in ('*.jpg', '*.jpeg', '*.png'):
            image_paths.extend(image_dir.rglob(pattern))
        if not image_paths:
            return None
        return max(image_paths, key=lambda path: path.stat().st_mtime)

    def send_goal(self, zone):
        pose = self.make_pose(zone)
        self.goal_pub.publish(pose)

        self.get_logger().info(
            f'Published goal to {zone.label} on {self.get_parameter("goal_pose_topic").value}: '
            f'x={zone.x:.2f}, y={zone.y:.2f}, yaw={zone.yaw:.2f}'
        )

    def make_pose(self, zone):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = zone.x
        pose.pose.position.y = zone.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(zone.yaw / 2.0)
        pose.pose.orientation.w = math.cos(zone.yaw / 2.0)
        return pose

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for index, zone in enumerate(self.zones.values()):
            marker_array.markers.append(self.make_zone_marker(zone, index, now))
            marker_array.markers.append(self.make_label_marker(zone, index + 100, now))
            marker_array.markers.append(self.make_arrow_marker(zone, index + 200, now))
        stage_zone = self.make_staging_zone(self.zones['trolley_dropzone'])
        marker_array.markers.append(self.make_zone_marker(stage_zone, 1000, now))
        marker_array.markers.append(self.make_label_marker(stage_zone, 1100, now))
        marker_array.markers.append(self.make_arrow_marker(stage_zone, 1200, now))
        self.marker_pub.publish(marker_array)

    def make_zone_marker(self, zone, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'dropzone_area'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = zone.x
        marker.pose.position.y = zone.y
        marker.pose.position.z = 0.01
        marker.pose.orientation.z = math.sin(zone.yaw / 2.0)
        marker.pose.orientation.w = math.cos(zone.yaw / 2.0)
        marker.scale.x = zone.width
        marker.scale.y = zone.height
        marker.scale.z = 0.02
        marker.color.r = float(zone.color[0])
        marker.color.g = float(zone.color[1])
        marker.color.b = float(zone.color[2])
        marker.color.a = float(zone.color[3])
        return marker

    def make_label_marker(self, zone, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'dropzone_label'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = zone.x
        marker.pose.position.y = zone.y
        marker.pose.position.z = 0.35
        marker.scale.z = 0.25
        marker.text = zone.label
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def make_arrow_marker(self, zone, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'dropzone_heading'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = zone.x
        marker.pose.position.y = zone.y
        marker.pose.position.z = 0.08
        marker.pose.orientation.z = math.sin(zone.yaw / 2.0)
        marker.pose.orientation.w = math.cos(zone.yaw / 2.0)
        marker.scale.x = max(zone.width * 0.45, 0.25)
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = float(zone.color[0])
        marker.color.g = float(zone.color[1])
        marker.color.b = float(zone.color[2])
        marker.color.a = 0.9
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = DropzoneMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
