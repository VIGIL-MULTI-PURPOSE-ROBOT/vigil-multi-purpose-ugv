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

Command-path watchdog: the row mission reaches the wheels only through the Nav2 velocity_smoother and
collision_monitor. When their lifecycle bringup stalls (seen: navigation_lifecycle stuck at
"Configuring behavior_server") both stay inactive, nothing is published on /cmd_vel and the rover
cannot move although START was pressed. The gate asks both nodes for their lifecycle state
(lifecycle_msgs/GetState) once a second; while RUNNING and the chain has not been active for
chain_timeout seconds, it also publishes the same (unchanged) commands on /cmd_vel for drive.py
(DIRECT path, logged, shown on the dashboard). As soon as both nodes are active the gate stops
publishing on /cmd_vel and the normal Nav2 path is used again. drive.py still applies its own
acceleration and turn-radius limits; the mission keeps its own obstacle avoidance and camera stop.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

try:
    from lifecycle_msgs.srv import GetState
except ImportError:                     # without lifecycle_msgs the watchdog is off
    GetState = None

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
        # --- Nav2 velocity chain watchdog
        self.chain_nodes = list(self.declare_parameter('chain_nodes', ['velocity_smoother', 'collision_monitor']).value)
        self.chain_timeout = float(self.declare_parameter('chain_timeout', 3.0).value)
        self.bypass_enabled = bool(self.declare_parameter('direct_fallback', True).value) and GetState is not None
        self.chain_state = {n: 'unknown' for n in self.chain_nodes}
        self.chain_ok_t = time.monotonic()
        self.direct = False
        self.direct_pub = self.create_publisher(Twist, 'cmd_vel', 10) if self.bypass_enabled else None
        self.chain_clients = {n: self.create_client(GetState, f'/{n}/get_state') for n in self.chain_nodes} if GetState else {}
        self.pending = {}
        self.create_timer(1.0, self.check_chain)
        self.publish_state('launch')

    def publish_state(self, reason):
        self.state_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': reason, 'forwarded': self.forwarded,
            'command_path': 'DIRECT (Nav2 velocity chain inactive)' if self.direct else 'NAV2',
            'chain': self.chain_state,
            'sim_time': self.get_clock().now().nanoseconds * 1e-9})))
        self.get_logger().info(f'robot state {self.state} ({reason})')

    # ------------------------------------------------------------ Nav2 chain watchdog
    def chain_active(self):
        return all(v == 'active' for v in self.chain_state.values())

    def check_chain(self):
        for n, cli in self.chain_clients.items():
            if n in self.pending:
                continue
            if not cli.service_is_ready():
                self.chain_state[n] = 'no service'
                continue
            fut = cli.call_async(GetState.Request())
            self.pending[n] = fut
            fut.add_done_callback(lambda f, n=n: self._on_state(n, f))
        now = time.monotonic()
        if self.chain_active() or not self.bypass_enabled:
            self.chain_ok_t = now
            if self.direct:
                self.direct = False
                self.direct_pub.publish(Twist())
                self.get_logger().info('Nav2 velocity chain active: commands go through velocity_smoother '
                                       '-> collision_monitor again (direct path off)')
                self.publish_state('Nav2 velocity chain active')
        elif self.state == RUNNING and not self.direct and now - self.chain_ok_t >= self.chain_timeout:
            self.direct = True
            self.get_logger().warn(f'Nav2 velocity chain not active {self.chain_state} for '
                                   f'{self.chain_timeout:.0f} s: forwarding the row mission commands straight '
                                   'to /cmd_vel (drive.py) so the rover can move')
            self.publish_state('Nav2 velocity chain inactive: direct command path')

    def _on_state(self, n, fut):
        self.pending.pop(n, None)
        try:
            self.chain_state[n] = fut.result().current_state.label
        except Exception:                               # noqa: BLE001  service went away
            self.chain_state[n] = 'unknown'

    def send(self, msg):
        self.out.publish(msg)
        if self.direct:
            self.direct_pub.publish(msg)

    def command(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == 'START' and self.state in (READY, STOPPED):
            self.state = RUNNING
            self.publish_state('START ROBOT pressed')
        elif cmd == 'STOP' and self.state == RUNNING:
            self.state = STOPPED
            self.send(Twist())              # stop now, not at the next hold tick
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
            self.send(msg)                         # unchanged
            self.forwarded += 1

    def hold(self):
        if self.state != RUNNING:
            self.send(Twist())


def main():
    rclpy.init()
    node = RowStartGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
