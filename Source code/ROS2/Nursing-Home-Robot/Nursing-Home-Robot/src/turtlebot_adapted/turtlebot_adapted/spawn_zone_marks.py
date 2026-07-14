#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


class GazeboZoneSpawner(Node):
    def __init__(self):
        super().__init__("gazebo_zone_spawner")

        self.client = self.create_client(SpawnEntity, "/spawn_entity")

        self.get_logger().info("Waiting for Gazebo /spawn_entity service...")
        self.client.wait_for_service()
        self.get_logger().info("Gazebo spawn service available.")

    def create_zone_sdf(self, name, color):
        r, g, b, a = color

        return f"""
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="zone_link">
      <visual name="zone_visual">
        <pose>0 0 0.006 0 0 0</pose>
        <geometry>
          <box>
            <size>1.0 1.0 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def spawn_zone(self, name, x, y, yaw, color):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)
        pose.orientation.z = qz
        pose.orientation.w = qw

        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.create_zone_sdf(name, color)
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = "world"

        self.get_logger().info(f"Spawning {name} at x={x}, y={y}")

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawn result for {name}: {future.result().status_message}")
        else:
            self.get_logger().error(f"Failed to spawn {name}")

    def spawn_all_zones(self):
        zones = [
            {
                "name": "waiting_charging_zone_mark",
                 "x": -2.0,
                 "y": -0.5,
                 "yaw": 0.0,
                 "color": (0.7, 0.7, 0.7, 0.8),
            },
            {
                "name": "pickup_zone_mark",
                "x": 0.5,
                "y": 0.0,
                "yaw": 0.0,
                "color": (0.0, 1.0, 0.0, 0.8),  # green
            },
            {
                "name": "trash_drop_zone_mark",
                "x": 1.8,
                "y": 0.8,
                "yaw": 1.57,
                "color": (0.0, 0.3, 1.0, 0.8),  # blue
            },
            {
                "name": "laundry_drop_zone_mark",
                "x": 1.8,
                "y": -0.8,
                "yaw": -1.57,
                "color": (1.0, 0.6, 0.0, 0.8),  # orange
            },
        ]

        for zone in zones:
            self.spawn_zone(
                name=zone["name"],
                x=zone["x"],
                y=zone["y"],
                yaw=zone["yaw"],
                color=zone["color"],
            )


def main(args=None):
    rclpy.init(args=args)

    spawner = GazeboZoneSpawner()
    spawner.spawn_all_zones()

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
