import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumKinematics(Node):

    def __init__(self):
        super().__init__('mecanum_kinematics')

        self.r = 0.0635
        self.L = 0.2019
        self.W = 0.1475
        self.k = self.L + self.W

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10)

    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        w  = msg.angular.z

        fl = (vx - vy - self.k*w) / self.r
        fr = (vx + vy + self.k*w) / self.r
        rl = (vx + vy - self.k*w) / self.r
        rr = (vx - vy + self.k*w) / self.r

        cmd = Float64MultiArray()
        cmd.data = [fl, fr, rl, rr]
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = MecanumKinematics()
    rclpy.spin(node)
