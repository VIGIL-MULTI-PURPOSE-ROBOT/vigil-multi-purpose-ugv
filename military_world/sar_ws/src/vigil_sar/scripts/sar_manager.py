#!/usr/bin/python3
"""SAR manager node: runs the search mission on top of the existing navigator.

Subscribes: /sar/command        std_msgs/String  START | STOP  (dashboard SAR switch)
            /navigation/status  std_msgs/String JSON (cliff_navigator: GOAL_REACHED at B, ...)
            /sim/ground_truth   nav_msgs/Odometry
            /sar/humans         std_msgs/String JSON (human_tracker)
Publishes:  /navigation/goal         geometry_msgs/PoseStamped (next search point / building view)
            /navigation/hold         std_msgs/Float64 (scan heading, NaN = drive)
            /navigation/speed_limit  std_msgs/Float64
            /sar/state               std_msgs/String JSON (state, search point k/10, humans, time, tasks)
            /sar/search_points       std_msgs/String JSON
            /sar/events              std_msgs/String JSON {t, text, source}

SAR never starts on its own (sar.enabled is ignored for auto-start on purpose): only the
operator's START. START is accepted in EVERY navigation state - at A, driving to B, near B, at B,
stopped or replanning. SAR pauses A->B and runs the search; STOP hands A->B back
(sar.stop_behavior). The operator's B is remembered separately from SAR's own goals.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import String, Float64  # noqa: E402

from ros_common import nested_params, odom_to_pose  # noqa: E402
from sar_core import SarMission, buildings_from_metadata  # noqa: E402
from sar_paths import find_military_world  # noqa: E402
from terrain_core import Params  # noqa: E402


class SarManager(Node):
    def __init__(self):
        super().__init__('sar_manager', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.cfg = cfg
        sar = dict(cfg['sar'])
        sar.setdefault('detection_range', cfg['thermal_camera'].get('detection_range', 30.0))
        tp = Params.from_nested(cfg)
        buildings = []
        try:
            mw = find_military_world(cfg['world'].get('military_world_dir', ''))
            meta = json.load(open(mw / cfg['world'].get('metadata_file', 'sar_metadata.json')))
            buildings = buildings_from_metadata(meta, sar.get('buildings', 'auto'))
        except (OSError, ValueError, FileNotFoundError) as e:
            self.get_logger().warn(f'no building list ({e}); SAR will search points only')
        m = 3.0
        bounds = (tp.map_origin_x + m, tp.map_origin_y + m, tp.map_origin_x + tp.map_size - m,
                  tp.map_origin_y + tp.map_size - m)
        self.m = SarMission(sar, buildings, thermal_hfov=float(cfg['thermal_camera'].get('horizontal_fov', 1.0472)),
                            robot_radius=tp.circumscribed_radius, bounds=bounds)
        self.pose = None
        self.nav = {}
        self.humans, self.tentative = [], []
        self.sent_seq = 0
        self.b_goal = (float(cfg['navigation'].get('goal_x', 0.0)), float(cfg['navigation'].get('goal_y', 0.0)))
        self.sar_goals = set()          # goals SAR itself sent: these must never replace the operator's B
        self.pending_cmd = None         # a command that arrived before the first robot pose
        self.create_subscription(String, '/sar/command', self.on_cmd, 10)
        self.create_subscription(String, '/navigation/status', self.on_nav, 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/sar/humans', self.on_humans, 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/navigation/goal', 10)
        self.pub_hold = self.create_publisher(Float64, '/navigation/hold', 10)
        self.pub_speed = self.create_publisher(Float64, '/navigation/speed_limit', 10)
        self.pub_state = self.create_publisher(String, '/sar/state', 10)
        self.pub_pts = self.create_publisher(String, '/sar/search_points', 10)
        self.pub_ev = self.create_publisher(String, '/sar/events', 20)
        self.pub_navcmd = self.create_publisher(String, '/navigation/command', 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'sar_manager ready: {len(buildings)} buildings known; SAR OFF until the '
                               f'dashboard switch sends START')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ inputs
    def on_odom(self, m):
        first = self.pose is None
        self.pose = odom_to_pose(m)
        if first and self.pending_cmd is not None:
            cmd, self.pending_cmd = self.pending_cmd, None
            self.on_cmd(String(data=cmd))

    def on_nav(self, m):
        try:
            self.nav = json.loads(m.data)
        except ValueError:
            return
        g = self.nav.get('goal')
        if not self.m.active and g and self._key(g) not in self.sar_goals:
            self.b_goal = tuple(g)                      # B as set on the dashboard (never SAR's goals)
            gc = self.m.goal_cmd
            if self.m.state == 'SAR COMPLETE' and gc is not None and \
                    math.hypot(self.b_goal[0] - gc[0], self.b_goal[1] - gc[1]) > 0.5:
                # operator sent a new B after the mission: release the final-position hold
                self.m.hold = None
                self.m.state = 'SAR OFF'

    def on_humans(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.humans, self.tentative = d.get('humans', []), d.get('tentative', [])

    def on_cmd(self, m):
        cmd = m.data.strip().upper()
        t = self.now()
        if self.pose is None:
            # never drop an operator command: act on it the moment the first pose arrives
            self.pending_cmd = cmd
            self._ev(t, f'SAR command {cmd} queued: waiting for the first robot pose')
            return
        if cmd in ('START', 'ON', 'SAR_ON'):
            ok, msg = self.m.activate(t, self.pose, self.b_goal, self.goal_reached())
            if not ok:
                self._ev(t, msg)
            else:
                self.pub_navcmd.publish(String(data='START'))   # navigator must be running
        elif cmd in ('STOP', 'OFF', 'SAR_OFF', 'ABORT'):
            self.m.abort(t, self.pose, self.b_goal)
        self.flush_events()

    @staticmethod
    def _key(g):
        return (round(float(g[0]), 2), round(float(g[1]), 2))

    def goal_reached(self):
        return self.nav.get('raw_state') == 'GOAL_REACHED'

    # ------------------------------------------------------------ outputs
    def _ev(self, t, text):
        self.get_logger().info(text)
        self.pub_ev.publish(String(data=json.dumps(dict(t=round(t, 1), text=text, source='sar'))))

    def flush_events(self):
        for t, text in self.m.events:
            self._ev(t, text)
        self.m.events.clear()

    def tick(self):
        if self.pose is None:
            return
        t = self.now()
        self.m.tick(t, self.pose, self.goal_reached(), self.humans, self.tentative)
        self.flush_events()
        if self.m.goal_seq != self.sent_seq and self.m.goal_cmd is not None:
            self.sent_seq = self.m.goal_seq
            g = PoseStamped()
            g.header.frame_id = 'world'
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x, g.pose.position.y = map(float, self.m.goal_cmd)
            g.pose.orientation.w = 1.0
            if self._key(self.m.goal_cmd) != self._key(self.b_goal):
                self.sar_goals.add(self._key(self.m.goal_cmd))
            self.pub_goal.publish(g)
        self.pub_hold.publish(Float64(data=float(self.m.hold) if self.m.hold is not None else math.nan))
        self.pub_speed.publish(Float64(data=float(self.m.speed_limit if self.m.active else 0.0)))
        st = self.m.status(t, self.pose)
        st['b_goal'] = list(self.b_goal)
        self.pub_state.publish(String(data=json.dumps(st)))
        self.pub_pts.publish(String(data=json.dumps(dict(points=self.m.search_points,
                                                         tasks=st['tasks']))))


def main():
    rclpy.init()
    node = SarManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
