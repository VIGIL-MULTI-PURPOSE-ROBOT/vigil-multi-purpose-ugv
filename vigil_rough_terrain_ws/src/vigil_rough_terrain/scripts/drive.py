#!/usr/bin/python3
"""Convert cmd_vel into the original eight wheel velocity commands."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class Drive(Node):
    def __init__(self):
        super().__init__('terrain_drive')
        self.target = [0.0, 0.0]
        self.current = [0.0, 0.0]
        self.last = 0.0
        self.pub = self.create_publisher(Float64MultiArray, 'wheel_controller/commands', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_timer(0.02, self.tick)

    def command(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.target = [max(-0.7, min(0.7, msg.linear.x)),
                           max(-0.6, min(0.6, msg.angular.z))]
            self.last = time.monotonic()

    def tick(self):
        target = self.target if time.monotonic() - self.last < 0.6 else [0.0, 0.0]
        for i in range(2):
            self.current[i] += max(-0.01, min(0.01, target[i] - self.current[i]))
        v, w = self.current
        wheels = [(v - side * w * 0.65) / radius for side in [1, -1]
                  for radius in [0.23463, 0.17655, 0.17655, 0.17655]]
        self.pub.publish(Float64MultiArray(data=wheels))

def main():
    rclpy.init()
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Float64MultiArray(data=[0.0] * 8))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
