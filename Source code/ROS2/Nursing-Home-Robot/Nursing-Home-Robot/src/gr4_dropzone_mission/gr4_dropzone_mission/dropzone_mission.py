#!/usr/bin/env python3

import os
from pathlib import Path
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


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
        self.declare_parameter('marker_topic', '/delivery_zones')
        self.declare_parameter('goal_pose_topic', '/goal_pose_raw')
        self.declare_parameter('marker_publish_period', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('trolley_drive_wait_sec', 12.0)
        self.declare_parameter(
            'detection_model_path',
            '~/Nursing-Home-Robot/src/object_detection/scripts/trolleys.pt',
        )
        self.declare_parameter(
            'test_image_path',
            '~/Nursing-Home-Robot/src/object_detection/test_images/current_trolley.jpg',
        )
        self.declare_parameter(
            'test_image_directory',
            '~/Nursing-Home-Robot/src/object_detection/test_images',
        )
        self.declare_parameter('detection_confidence', 0.25)
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

        period = float(self.get_parameter('marker_publish_period').value)
        self.create_timer(period, self.publish_markers)
        self.publish_markers()

        self.get_logger().info(
            'Dropzone mission ready. Commands on /trolley_command: '
            'charging, trolley_dropzone, laundry_dropoff, trash_dropoff'
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
        command = msg.data.strip().lower()
        if command in ('cancel', 'stop'):
            self.get_logger().warn(
                'Cancel is not handled by goal-pose publishing. Use RViz cancel or Nav2 action cancel.'
            )
            return

        if command in ('trolley_ready', 'detect_trolley', 'detect_and_deliver', 'deliver_trolley'):
            self.start_detect_and_deliver()
            return

        if command == 'laundry_trolley':
            self.get_logger().info('Laundry trolley command received; going to laundry dropoff.')
            self.send_goal(self.zones['laundry_dropoff'])
            return

        zone_key = self.command_aliases.get(command)
        if zone_key is None:
            self.get_logger().warn(
                f'Unknown command "{command}". Valid commands: '
                'charging, trolley_dropzone, trolley_ready, laundry_dropoff, trash_dropoff, cancel'
            )
            return

        self.send_goal(self.zones[zone_key])

    def start_detect_and_deliver(self):
        if self.pending_detection_timer is not None:
            self.pending_detection_timer.cancel()

        self.get_logger().info('Trolley ready: driving to trolley dropzone before detection.')
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
