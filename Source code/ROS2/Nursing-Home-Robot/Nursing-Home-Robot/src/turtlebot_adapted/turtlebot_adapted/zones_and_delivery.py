#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray


class Zone:
    def __init__(self, name, x, y, yaw, width, height, color):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.width = width
        self.height = height
        self.color = color


class TrolleyMissionNode(BasicNavigator):
    def __init__(self):
        super().__init__()

        self.latest_command = None

        self.command_subscriber = self.create_subscription(
            String,
            "/trolley_command",
            self.command_callback,
            10,
        )

        self.detect_trolley_pub = self.create_publisher(
            Bool,
            "/detect_trolley_request",
            10,
        )

        self.waiting_zone = Zone(
            name="WAITING_CHARGING_ZONE",
            x=-2.0,
            y=-0.5,
            yaw=0.0,
            width=1.0,
            height=1.0,
            color=(0.7, 0.7, 0.7),
        )

        self.pickup_zone = Zone(
            name="PICKUP_ZONE",
            x=0.5,
            y=0.0,
            yaw=0.0,
            width=1.0,
            height=1.0,
            color=(0.0, 1.0, 0.0),
        )

        self.trash_drop_zone = Zone(
            name="TRASH_DROP_ZONE",
            x=1.8,
            y=0.8,
            yaw=1.57,
            width=1.0,
            height=1.0,
            color=(1.0, 0.3, 0.0),
        )

        self.laundry_drop_zone = Zone(
            name="LAUNDRY_DROP_ZONE",
            x=1.8,
            y=-0.8,
            yaw=-1.57,
            width=1.0,
            height=1.0,
            color=(0.0, 0.4, 1.0),
        )

        self.zones = [
            self.waiting_zone,
            self.pickup_zone,
            self.trash_drop_zone,
            self.laundry_drop_zone,
        ]

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.latest_command = command
        self.get_logger().info(f"Received command: {command}")

    def clear_command(self):
        self.latest_command = None

    def request_trolley_detection(self):
        msg = Bool()
        msg.data = True

        self.get_logger().info("Requesting YOLO trolley detection...")
        self.detect_trolley_pub.publish(msg)

    def wait_for_command(self, allowed_commands):
        self.get_logger().info(
            f"Waiting for command: {', '.join(allowed_commands)}"
        )

        self.clear_command()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.latest_command in allowed_commands:
                command = self.latest_command
                self.clear_command()
                return command

            if self.latest_command is not None:
                self.get_logger().warn(
                    f"Ignoring unknown command here: {self.latest_command}"
                )
                self.clear_command()

        return None

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def make_zone_marker(self, zone, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "delivery_zones"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(zone.x)
        marker.pose.position.y = float(zone.y)
        marker.pose.position.z = 0.01

        qz, qw = self.yaw_to_quaternion(zone.yaw)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = zone.width
        marker.scale.y = zone.height
        marker.scale.z = 0.02

        marker.color.r = zone.color[0]
        marker.color.g = zone.color[1]
        marker.color.b = zone.color[2]
        marker.color.a = 0.35

        return marker

    def make_text_marker(self, zone, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "delivery_zone_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = float(zone.x)
        marker.pose.position.y = float(zone.y)
        marker.pose.position.z = 0.35

        marker.scale.z = 0.25
        marker.text = zone.name

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def publish_zones(self):
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        publisher = self.create_publisher(MarkerArray, "/delivery_zones", qos)

        marker_array = MarkerArray()

        for i, zone in enumerate(self.zones):
            marker_array.markers.append(self.make_zone_marker(zone, i))
            marker_array.markers.append(self.make_text_marker(zone, i + 100))

        for _ in range(10):
            publisher.publish(marker_array)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

    def navigate_to_zone(self, zone):
        self.get_logger().info(f"Navigating to {zone.name}")

        goal_pose = self.make_pose(zone.x, zone.y, zone.yaw)

        self.goToPose(goal_pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()

            if feedback:
                self.get_logger().info(
                    f"Distance remaining to {zone.name}: "
                    f"{feedback.distance_remaining:.2f} m"
                )

            rclpy.spin_once(self, timeout_sec=0.2)

        result = self.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {zone.name}")
            return True

        if result == TaskResult.CANCELED:
            self.get_logger().warn(f"Navigation to {zone.name} was canceled")
            return False

        if result == TaskResult.FAILED:
            self.get_logger().error(f"Navigation to {zone.name} failed")
            return False

        self.get_logger().warn(f"Unknown result while navigating to {zone.name}")
        return False

    def run_mission_loop(self):
        initial_pose = self.make_pose(
            self.waiting_zone.x,
            self.waiting_zone.y,
            self.waiting_zone.yaw,
        )

        self.setInitialPose(initial_pose)

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.waitUntilNav2Active()

        self.publish_zones()

        self.get_logger().info("Robot ready.")
        self.get_logger().info("Starting at waiting / charging zone.")

        self.navigate_to_zone(self.waiting_zone)

        while rclpy.ok():
            self.get_logger().info("Robot is waiting at charging zone.")

            command = self.wait_for_command(["trolley_ready"])

            if command != "trolley_ready":
                continue

            self.get_logger().info("Trolley ready. Going to pickup zone.")
            pickup_success = self.navigate_to_zone(self.pickup_zone)

            if not pickup_success:
                self.get_logger().error(
                    "Could not reach pickup zone. Returning to waiting zone."
                )
                self.navigate_to_zone(self.waiting_zone)
                continue

            self.get_logger().info("Arrived at pickup zone.")
            self.get_logger().info("Requesting object detection for trolley type.")

            self.request_trolley_detection()

            trolley_type = self.wait_for_command(
                ["trash_trolley", "laundry_trolley", "unknown_trolley"]
            )

            if trolley_type == "trash_trolley":
                self.get_logger().info("Trash trolley detected by YOLO.")
                drop_zone = self.trash_drop_zone

            elif trolley_type == "laundry_trolley":
                self.get_logger().info("Laundry trolley detected by YOLO.")
                drop_zone = self.laundry_drop_zone

            else:
                self.get_logger().warn(
                    "YOLO could not identify the trolley type. Returning to waiting zone."
                )
                self.navigate_to_zone(self.waiting_zone)
                continue

            drop_success = self.navigate_to_zone(drop_zone)

            if not drop_success:
                self.get_logger().error(
                    "Could not reach drop zone. Returning to waiting zone."
                )
                self.navigate_to_zone(self.waiting_zone)
                continue

            self.get_logger().info(f"Dropped trolley at {drop_zone.name}")
            time.sleep(2.0)

            self.get_logger().info("Returning to waiting / charging zone.")
            self.navigate_to_zone(self.waiting_zone)


def main(args=None):
    rclpy.init(args=args)

    node = TrolleyMissionNode()

    try:
        node.run_mission_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Mission stopped by user.")
    finally:
        try:
            node.lifecycleShutdown()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
