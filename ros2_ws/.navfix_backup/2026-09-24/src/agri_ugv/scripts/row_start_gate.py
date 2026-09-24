#!/usr/bin/python3
"""START / STOP gate for the existing crop-row mission (dashboard buttons).

It adds no navigation and no motor control. field.launch.py (dashboard:=true) renames
crop_row_driver's output from /cmd_vel_nav to /crop_row/cmd_vel_request; this node sits
between the two:

    crop_row_driver --/crop_row/cmd_vel_request--> row_start_gate --/cmd_vel_nav--> (unchanged:
    velocity_smoother -> collision_monitor -> /cmd_vel -> drive.py -> wheel_controller)

  READY / STOPPED : publishes zero Twist on /cmd_vel_nav (the existing drive.py ramps to rest)
  RUNNING         : forwards every driver command unchanged (same message, same rate)
  COMPLETE        : the driver reported the field sweep complete; zero Twist
  HALTED          : the driver stopped itself with one of its own crop-safety stops; zero Twist

Commands: std_msgs/String on /agri_dashboard/command, data START or STOP.
State:    std_msgs/String JSON on /agri_dashboard/robot_state (latched).
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

READY, RUNNING, STOPPED, COMPLETE, HALTED = 'READY', 'RUNNING', 'STOPPED', 'COMPLETE', 'HALTED'


class RowStartGate(Node):
    def __init__(self):
        super().__init__('row_start_gate')
        self.state = RUNNING if self.declare_parameter('autostart', False).value else READY
        self.hold_rate = float(self.declare_parameter('hold_rate_hz', 20.0).value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.out = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.state_pub = self.create_publisher(String, 'agri_dashboard/robot_state', latched)
        self.create_subscription(Twist, 'crop_row/cmd_vel_request', self.request, 10)
        self.create_subscription(String, 'agri_dashboard/command', self.command, 10)
        self.create_subscription(String, 'exploration/status', self.driver_status, latched)
        self.create_timer(1.0 / self.hold_rate, self.hold)
        self.forwarded = 0
        self.publish_state('launch')

    def publish_state(self, reason):
        self.state_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': reason, 'forwarded': self.forwarded,
            'sim_time': self.get_clock().now().nanoseconds * 1e-9})))
        self.get_logger().info(f'robot state {self.state} ({reason})')

    def command(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == 'START' and self.state in (READY, STOPPED):
            self.state = RUNNING
            self.publish_state('START ROBOT pressed')
        elif cmd == 'STOP' and self.state == RUNNING:
            self.state = STOPPED
            self.out.publish(Twist())              # stop now, not at the next hold tick
            self.publish_state('STOP ROBOT pressed')

    def driver_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get('state') == 'row_complete' and self.state not in (COMPLETE, HALTED):
            # The driver has already stopped itself (mission done, or one of its own crop-safety
            # stops); this only reports it. The driver does not restart after this.
            text = data.get('message', 'row mission finished')
            self.state = COMPLETE if 'complete' in text.lower() else HALTED
            self.publish_state(text)

    def request(self, msg):
        if self.state == RUNNING:
            self.out.publish(msg)                  # unchanged
            self.forwarded += 1

    def hold(self):
        if self.state != RUNNING:
            self.out.publish(Twist())


def main():
    rclpy.init()
    node = RowStartGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.out.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
