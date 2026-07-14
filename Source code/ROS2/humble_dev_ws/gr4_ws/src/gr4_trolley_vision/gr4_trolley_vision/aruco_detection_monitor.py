#!/usr/bin/env python3

import time

import rclpy
from aruco_interfaces.msg import ArucoMarkers
from rclpy.node import Node


class ArucoDetectionMonitor(Node):
    def __init__(self):
        super().__init__('aruco_detection_monitor')
        self.declare_parameter(
            'topics',
            '/front/aruco_markers,/rear/aruco_markers,/left/aruco_markers,/right/aruco_markers',
        )
        self.declare_parameter('print_period', 1.0)

        self.print_period = float(self.get_parameter('print_period').value)
        self.last_print = {}
        for topic in [v.strip() for v in self.get_parameter('topics').value.split(',') if v.strip()]:
            self.create_subscription(
                ArucoMarkers,
                topic,
                lambda msg, topic=topic: self.marker_callback(topic, msg),
                10,
            )
            self.get_logger().info(f'Watching {topic}')

    def marker_callback(self, topic, msg):
        now = time.monotonic()
        if now - self.last_print.get(topic, 0.0) < self.print_period:
            return
        self.last_print[topic] = now
        ids = [int(marker_id) for marker_id in msg.marker_ids]
        self.get_logger().info(
            f'{topic}: ids={ids} count={len(ids)} frame={msg.header.frame_id}'
        )


def main():
    rclpy.init()
    node = ArucoDetectionMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
