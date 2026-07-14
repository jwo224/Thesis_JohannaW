#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState


class RobotShellSpawner(Node):
    def __init__(self):
        super().__init__("robot_shell_spawner")

        self.declare_parameter("shell_name", "nursing_home_robot_shell")

        self.declare_parameter("shell_length", 0.65)
        self.declare_parameter("shell_width", 0.50)
        self.declare_parameter("shell_height", 0.45)

        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.25)

        self.declare_parameter("color_r", 0.2)
        self.declare_parameter("color_g", 1.0)
        self.declare_parameter("color_b", 0.2)
        self.declare_parameter("color_a", 0.75)

        self.declare_parameter("update_rate_hz", 20.0)

        self.shell_name = self.get_parameter("shell_name").value

        self.shell_length = float(self.get_parameter("shell_length").value)
        self.shell_width = float(self.get_parameter("shell_width").value)
        self.shell_height = float(self.get_parameter("shell_height").value)

        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)

        self.color_r = float(self.get_parameter("color_r").value)
        self.color_g = float(self.get_parameter("color_g").value)
        self.color_b = float(self.get_parameter("color_b").value)
        self.color_a = float(self.get_parameter("color_a").value)

        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        self.latest_odom = None
        self.spawned = False

        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.set_state_client = self.create_client(SetEntityState, "/set_entity_state")

        self.get_logger().info("Waiting for Gazebo spawn service...")
        self.spawn_client.wait_for_service()
        self.get_logger().info("Gazebo spawn service available.")

        self.get_logger().info("Waiting for Gazebo set entity state service...")
        self.set_state_client.wait_for_service()
        self.get_logger().info("Gazebo set entity state service available.")

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.spawn_shell()

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.update_shell_pose)

    def create_shell_sdf(self):
        return f"""
<sdf version="1.6">
  <model name="{self.shell_name}">
    <static>false</static>
    <link name="shell_link">
      <gravity>false</gravity>

      <visual name="shell_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>{self.shell_length} {self.shell_width} {self.shell_height}</size>
          </box>
        </geometry>
        <material>
          <ambient>{self.color_r} {self.color_g} {self.color_b} {self.color_a}</ambient>
          <diffuse>{self.color_r} {self.color_g} {self.color_b} {self.color_a}</diffuse>
        </material>
      </visual>

      <!-- Very small collision so it does not disturb TurtleBot navigation -->
      <collision name="tiny_collision">
        <pose>0 0 -10 0 0 0</pose>
        <geometry>
          <box>
            <size>0.01 0.01 0.01</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

    def spawn_shell(self):
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = self.offset_z
        pose.orientation.w = 1.0

        request = SpawnEntity.Request()
        request.name = self.shell_name
        request.xml = self.create_shell_sdf()
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = "world"

        self.get_logger().info(f"Spawning visual shell: {self.shell_name}")

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawn result: {future.result().status_message}")
            self.spawned = True
        else:
            self.get_logger().error("Failed to spawn robot shell.")
            self.spawned = False

    def odom_callback(self, msg):
        self.latest_odom = msg

    def update_shell_pose(self):
        if not self.spawned:
            return

        if self.latest_odom is None:
            return

        odom_pose = self.latest_odom.pose.pose

        yaw = self.quaternion_to_yaw(
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w,
        )

        rotated_offset_x = (
            self.offset_x * math.cos(yaw)
            - self.offset_y * math.sin(yaw)
        )
        rotated_offset_y = (
            self.offset_x * math.sin(yaw)
            + self.offset_y * math.cos(yaw)
        )

        state = EntityState()
        state.name = self.shell_name

        state.pose.position.x = float(odom_pose.position.x + rotated_offset_x)
        state.pose.position.y = float(odom_pose.position.y + rotated_offset_y)
        state.pose.position.z = float(self.offset_z)

        state.pose.orientation = odom_pose.orientation

        request = SetEntityState.Request()
        request.state = state

        self.set_state_client.call_async(request)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)

    node = RobotShellSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Robot shell spawner stopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
