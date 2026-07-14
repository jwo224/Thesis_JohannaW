#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class HolonomicGoalController(Node):
    def __init__(self):
        super().__init__("holonomic_goal_controller")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cancel_topic", "/holonomic_cancel")
        self.declare_parameter("ignore_radius_topic", "/holonomic_lidar_ignore_radius")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("xy_tolerance", 0.04)
        self.declare_parameter("yaw_tolerance_deg", 8.0)
        self.declare_parameter("max_vx", 0.16)
        self.declare_parameter("max_vy", 0.16)
        self.declare_parameter("max_wz", 0.16)
        self.declare_parameter("k_xy", 0.55)
        self.declare_parameter("k_yaw", 0.8)
        self.declare_parameter("brake_distance", 0.35)
        self.declare_parameter("scan_topic", "/left/scan")
        self.declare_parameter("lidar_x_offset", 0.172)
        self.declare_parameter("avoid_distance", 0.95)
        self.declare_parameter("stop_distance", 0.55)
        self.declare_parameter("obstacle_gain", 0.22)
        self.declare_parameter("obstacle_corridor_width", 0.45)
        self.declare_parameter("lidar_ignore_radius", 0.0)
        self.declare_parameter("attached_ignore_rear", 1.80)
        self.declare_parameter("attached_ignore_front", 0.85)
        self.declare_parameter("attached_ignore_half_width", 0.85)

        self.reload_runtime_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.goal_callback, 10)
        self.create_subscription(Empty, self.get_parameter("cancel_topic").value, self.cancel_callback, 10)
        self.create_subscription(
            Float32,
            self.get_parameter("ignore_radius_topic").value,
            self.ignore_radius_callback,
            10,
        )
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 10)

        self.current_pose = None
        self.goal_pose = None
        self.scan_points = []
        self.timer = self.create_timer(1.0 / float(self.get_parameter("control_rate").value), self.control)

        goal_topic = self.get_parameter("goal_topic").value
        self.get_logger().info(f"Holonomic goal controller ready. Listening on {goal_topic}.")

    def reload_runtime_parameters(self):
        self.xy_tolerance = float(self.get_parameter("xy_tolerance").value)
        self.yaw_tolerance = math.radians(float(self.get_parameter("yaw_tolerance_deg").value))
        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.k_xy = float(self.get_parameter("k_xy").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.brake_distance = float(self.get_parameter("brake_distance").value)
        self.lidar_x_offset = float(self.get_parameter("lidar_x_offset").value)
        self.avoid_distance = float(self.get_parameter("avoid_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.obstacle_gain = float(self.get_parameter("obstacle_gain").value)
        self.obstacle_corridor_width = float(self.get_parameter("obstacle_corridor_width").value)
        self.lidar_ignore_radius = float(self.get_parameter("lidar_ignore_radius").value)
        self.attached_ignore_rear = float(self.get_parameter("attached_ignore_rear").value)
        self.attached_ignore_front = float(self.get_parameter("attached_ignore_front").value)
        self.attached_ignore_half_width = float(self.get_parameter("attached_ignore_half_width").value)

    def parameters_callback(self, parameters):
        for parameter in parameters:
            name = parameter.name
            value = parameter.value
            if name == "xy_tolerance":
                self.xy_tolerance = float(value)
            elif name == "yaw_tolerance_deg":
                self.yaw_tolerance = math.radians(float(value))
            elif name == "max_vx":
                self.max_vx = float(value)
            elif name == "max_vy":
                self.max_vy = float(value)
            elif name == "max_wz":
                self.max_wz = float(value)
            elif name == "k_xy":
                self.k_xy = float(value)
            elif name == "k_yaw":
                self.k_yaw = float(value)
            elif name == "brake_distance":
                self.brake_distance = float(value)
            elif name == "lidar_x_offset":
                self.lidar_x_offset = float(value)
            elif name == "avoid_distance":
                self.avoid_distance = float(value)
            elif name == "stop_distance":
                self.stop_distance = float(value)
            elif name == "obstacle_gain":
                self.obstacle_gain = float(value)
            elif name == "obstacle_corridor_width":
                self.obstacle_corridor_width = float(value)
            elif name == "lidar_ignore_radius":
                self.lidar_ignore_radius = float(value)
            elif name == "attached_ignore_rear":
                self.attached_ignore_rear = float(value)
            elif name == "attached_ignore_front":
                self.attached_ignore_front = float(value)
            elif name == "attached_ignore_half_width":
                self.attached_ignore_half_width = float(value)
        return SetParametersResult(successful=True)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = (
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(pose.orientation),
        )

    def goal_callback(self, msg):
        goal = self.goal_to_odom(msg)
        if goal is None:
            return
        self.goal_pose = goal
        x, y, yaw = goal
        self.get_logger().info(
            f"Accepted holonomic goal in odom: x={x:+.2f}, y={y:+.2f}, yaw={math.degrees(yaw):+.1f} deg"
        )

    def cancel_callback(self, _msg):
        if self.goal_pose is not None:
            self.get_logger().info("Holonomic goal canceled.")
        self.goal_pose = None
        self.stop()

    def ignore_radius_callback(self, msg):
        self.lidar_ignore_radius = max(0.0, float(msg.data))
        self.get_logger().info(f"Holonomic lidar ignore radius set to {self.lidar_ignore_radius:.2f} m.")

    def scan_callback(self, msg):
        points = []
        angle = msg.angle_min
        for reading in msg.ranges:
            if math.isfinite(reading) and msg.range_min <= reading <= min(msg.range_max, self.avoid_distance):
                # Convert from lidar frame to approximate base_link body coordinates.
                obs_x = self.lidar_x_offset + reading * math.cos(angle)
                obs_y = reading * math.sin(angle)
                if not self.should_ignore_scan_point(obs_x, obs_y):
                    points.append((obs_x, obs_y, reading))
            angle += msg.angle_increment
        self.scan_points = points

    def should_ignore_scan_point(self, obs_x, obs_y):
        if self.lidar_ignore_radius <= 0.0:
            return False
        if math.hypot(obs_x, obs_y) < self.lidar_ignore_radius:
            return True
        return (
            -self.attached_ignore_rear <= obs_x <= self.attached_ignore_front
            and abs(obs_y) <= self.attached_ignore_half_width
        )

    def goal_to_odom(self, msg):
        frame = msg.header.frame_id.strip() if msg.header.frame_id else "odom"
        goal_yaw = quaternion_to_yaw(msg.pose.orientation)
        if frame == "odom":
            return (msg.pose.position.x, msg.pose.position.y, goal_yaw)

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Cannot transform goal from {frame} to odom yet: {exc}")
            return None

        tf = transform.transform
        tf_yaw = quaternion_to_yaw(tf.rotation)
        cos_yaw = math.cos(tf_yaw)
        sin_yaw = math.sin(tf_yaw)
        x_in = msg.pose.position.x
        y_in = msg.pose.position.y
        x_out = tf.translation.x + cos_yaw * x_in - sin_yaw * y_in
        y_out = tf.translation.y + sin_yaw * x_in + cos_yaw * y_in
        return (x_out, y_out, normalize_angle(tf_yaw + goal_yaw))

    def control(self):
        if self.current_pose is None or self.goal_pose is None:
            return

        x, y, yaw = self.current_pose
        goal_x, goal_y, goal_yaw = self.goal_pose

        error_x = goal_x - x
        error_y = goal_y - y
        yaw_error = normalize_angle(goal_yaw - yaw)
        distance = math.hypot(error_x, error_y)

        if distance <= self.xy_tolerance and abs(yaw_error) <= self.yaw_tolerance:
            self.stop()
            self.goal_pose = None
            self.get_logger().info("Holonomic goal reached.")
            return

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = cos_yaw * error_x + sin_yaw * error_y
        body_y = -sin_yaw * error_x + cos_yaw * error_y

        speed_scale = min(1.0, max(0.25, distance / self.brake_distance))
        cmd = Twist()
        vx = clamp(self.k_xy * body_x, self.max_vx * speed_scale)
        vy = clamp(self.k_xy * body_y, self.max_vy * speed_scale)
        vx, vy = self.apply_obstacle_avoidance(vx, vy)
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = clamp(self.k_yaw * yaw_error, self.max_wz)
        self.cmd_pub.publish(cmd)

    def apply_obstacle_avoidance(self, vx, vy):
        speed = math.hypot(vx, vy)
        if speed < 1.0e-4 or not self.scan_points:
            return vx, vy

        dir_x = vx / speed
        dir_y = vy / speed
        repulse_x = 0.0
        repulse_y = 0.0
        blocked = False

        for obs_x, obs_y, _reading in self.scan_points:
            obs_dist = math.hypot(obs_x, obs_y)
            if obs_dist < 1.0e-4:
                continue

            along = obs_x * dir_x + obs_y * dir_y
            lateral = abs(-dir_y * obs_x + dir_x * obs_y)
            if along <= 0.0:
                continue

            if along < self.stop_distance and lateral < self.obstacle_corridor_width:
                blocked = True

            if obs_dist < self.avoid_distance:
                strength = self.obstacle_gain * (self.avoid_distance - obs_dist) / self.avoid_distance
                repulse_x -= strength * obs_x / obs_dist
                repulse_y -= strength * obs_y / obs_dist

        if blocked:
            # Remove velocity into the obstacle corridor, but keep lateral escape motion.
            into_obstacle = max(0.0, vx * dir_x + vy * dir_y)
            vx -= into_obstacle * dir_x
            vy -= into_obstacle * dir_y

        vx = clamp(vx + repulse_x, self.max_vx)
        vy = clamp(vy + repulse_y, self.max_vy)
        return vx, vy

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = HolonomicGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
