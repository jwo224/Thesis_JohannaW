#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuFixer(Node):
    def __init__(self):
        super().__init__('imu_fixer')
        self.sub = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        # measured while standing still
        self.gyro_bias = [0.0656, 0.0, 0.0]

    def callback(self, msg):
        msg.header.frame_id = 'imu_link'

        # remove gyro bias
        msg.angular_velocity.x -= self.gyro_bias[0]
        msg.angular_velocity.y -= self.gyro_bias[1]
        msg.angular_velocity.z -= self.gyro_bias[2]

        # set realistic covariances
        msg.angular_velocity_covariance[0] = 1e-4
        msg.angular_velocity_covariance[4] = 1e-4
        msg.angular_velocity_covariance[8] = 1e-4

        msg.linear_acceleration_covariance[0] = 1e-2
        msg.linear_acceleration_covariance[4] = 1e-2
        msg.linear_acceleration_covariance[8] = 1e-2

        msg.orientation_covariance[0] = 1e-3
        msg.orientation_covariance[4] = 1e-3
        msg.orientation_covariance[8] = 1e-3

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuFixer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

