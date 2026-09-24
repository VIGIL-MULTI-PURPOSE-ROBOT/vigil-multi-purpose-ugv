#!/usr/bin/python3
"""A-to-B navigation with cliff-aware replanning.

This is vigil_rough_terrain/scripts/cliff_navigator.py (that workspace is only read, never changed):
the same planner (planner_core), the same map handling and the same /cmd_vel output that drive.py
turns into eight wheel speeds. The speed governor and command shaper that this copy had grown are
gone. What vigil_sar adds is only what the SAR layer and the dashboard need, marked [SAR]:

  [SAR] /navigation/hold        std_msgs/Float64  heading to turn to and hold while SAR scans; NaN releases
  [SAR] /navigation/speed_limit std_msgs/Float64  SAR's speed cap (<= 0: none)
  [SAR] /navigation/command     std_msgs/String   START | PAUSE from the dashboard
  [SAR] setting B starts the drive at once (navigation.start_on_goal) and is acted on in the
        same callback, so the first command leaves within one planning pass
  [SAR] GOAL_REACHED sends an explicit zero command every tick
  [SAR] costmap rebuilds are throttled to navigation.map_update_period (the map is 170 m wide)
  [DYN] /perception/obstacles (obstacle_tracker.py): people, vehicles and moving obstacles are stamped
        into the planner's class grid with their safety distance and, when moving, along their
        predicted path (obstacle_core.DynamicAvoidance). Collision prediction on the planned path ->
        COLLISION RISK / AVOIDING / CLEAR in /navigation/status, a replan, and a smooth speed cap
        (yield to a crossing person, slow near people). Cliffs are never overwritten. The navigator's
        own driving, planning and escalation are unchanged.

Subscribes: /vision/terrain_classes, /vision/slope, /vision/roughness, /sim/ground_truth,
            /navigation/goal and /goal_pose (geometry_msgs/PoseStamped, e.g. RViz 2D goal)
Publishes:  /cmd_vel, /navigation/path, /navigation/previous_path,
            /navigation/status (JSON string)
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import Twist, PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry, OccupancyGrid, Path  # noqa: E402
from std_msgs.msg import String, Float64  # noqa: E402

from terrain_core import Params, OBSTACLE, CLIFF  # noqa: E402
from planner_core import NavigatorCore, wrap  # noqa: E402
from obstacle_core import ObstacleParams, DynamicAvoidance, tracks_from_msg  # noqa: E402
from ros_common import nested_params, odom_to_pose, grid_to_numpy, path_msg  # noqa: E402


class CliffNavigator(Node):
    def __init__(self):
        super().__init__('cliff_navigator', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        nav_cfg = cfg['navigation']
        self.tp = Params.from_nested(cfg)
        self.nav = NavigatorCore(self.tp, nav_cfg)
        self.pose = None
        self.speed = 0.0
        self.published_version = -1
        self.published_prev = None
        self.slope = self.rough = None
        self.create_subscription(OccupancyGrid, '/vision/slope',
                                 lambda m: setattr(self, 'slope', grid_to_numpy(m).astype(np.float32)), 2)
        self.create_subscription(OccupancyGrid, '/vision/roughness',
                                 lambda m: setattr(self, 'rough', grid_to_numpy(m).astype(np.float32) / 100.0), 2)
        self.create_subscription(OccupancyGrid, '/vision/terrain_classes', self.on_map, 2)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/navigation/goal', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/navigation/path', 2)
        self.pub_prev = self.create_publisher(Path, '/navigation/previous_path', 2)
        self.pub_status = self.create_publisher(String, '/navigation/status', 10)
        # [SAR] hooks
        self.pending_map = None
        self.last_build = -1e9
        self.map_period = float(nav_cfg.get('map_update_period', 0.0))
        self.hold = None
        self.hold_tol = float(nav_cfg.get('hold_heading_tolerance', 0.08))
        self.hold_w = float(nav_cfg.get('hold_max_angular', 0.5))
        self.speed_limit = 0.0
        self.started = bool(nav_cfg.get('autostart', False))
        self.start_on_goal = bool(nav_cfg.get('start_on_goal', True))
        self.start_delay = float(nav_cfg.get('start_delay', 6.0))
        self.create_subscription(Float64, '/navigation/hold', self.on_hold, 10)
        self.create_subscription(Float64, '/navigation/speed_limit',
                                 lambda m: setattr(self, 'speed_limit', float(m.data)), 10)
        self.create_subscription(String, '/navigation/command', self.on_command, 10)
        # [DYN] dynamic obstacles
        self.op = ObstacleParams.from_dict(cfg.get('obstacles', {}))
        self.dyn = DynamicAvoidance(self.op, self.tp, nav_cfg, OBSTACLE, keep_classes=(CLIFF,))
        self.base_cls = None                  # last terrain class grid (before the overlay)
        self.obst_msg, self.obst_t = None, -1e9
        self.obst_dirty = False
        self.had_overlay = False
        self.last_stamp = -1e9
        if self.op.enabled:
            self.create_subscription(String, '/perception/obstacles', self.on_obstacles, 10)
        self.last_logged = None                                # [SAR] state changes go to the log
        # 20 Hz (vigil_rough_terrain: 10 Hz): half the reaction time, for a quicker turn-in
        self.create_timer(float(nav_cfg.get('tick_period', 0.05)), self.tick)
        self.create_timer(1.0, self.republish_paths)
        g = self.nav.goal
        self.get_logger().info(f'cliff_navigator (vigil_rough_terrain): goal B = ({g[0]:.2f}, {g[1]:.2f}); '
                               f'send a new one on /navigation/goal or /goal_pose')

    # ------------------------------------------------------------------ inputs
    def on_map(self, m):
        if self.map_period <= 0.0:                     # vigil_rough_terrain: every map, at once
            self._build(m)
        else:
            self.pending_map = m                       # [SAR] 170 m map: throttled

    def _build(self, m):
        cls = grid_to_numpy(m)
        self.base_cls = np.where(cls < 0, 0, cls).astype(np.uint8)
        self._rebuild()

    def _rebuild(self):
        """[DYN] terrain classes (+ tracked obstacles) -> planner costmap."""
        cls = self.base_cls
        if cls is None:
            return
        ok = lambda a: a if a is not None and a.shape == cls.shape else None  # noqa: E731
        if self.op.enabled and self.pose is not None and self._obstacles_fresh() and self.dyn.active():
            cls = self.dyn.overlay(cls, self.pose, self.nav.goal)
            self.had_overlay = True
        else:
            self.dyn.cells = 0
            self.had_overlay = False
        self.nav.update_map(cls, ok(self.slope), ok(self.rough))

    def on_obstacles(self, m):                                  # [DYN]
        try:
            self.obst_msg = json.loads(m.data)
        except ValueError:
            return
        self.obst_t = self.now()
        self.dyn.set_tracks(tracks_from_msg(self.obst_msg, self.obst_t))
        self.obst_dirty = True

    def _obstacles_fresh(self):
        return self.now() - self.obst_t < max(1.0, 2.0 * self.op.track_timeout)

    def on_odom(self, m):
        self.pose = odom_to_pose(m)
        t = m.twist.twist.linear
        self.speed = float(np.hypot(t.x, t.y))

    def on_goal(self, m):
        t = self.now()
        self.nav.set_goal(m.pose.position.x, m.pose.position.y, t)
        self.get_logger().info(f'new goal B = ({m.pose.position.x:.2f}, {m.pose.position.y:.2f})')
        if self.start_on_goal and not self.started:           # [SAR] setting B = go
            self.started = True
            self.nav.t0 = t - self.start_delay
            self.nav.start = None
            self.nav._event('NAVIGATING', 'goal B set: driving to B', hold=2.0)
        self.last_build = -1e9
        self.tick()                                            # [SAR] act on it now, not next tick

    def on_hold(self, m):                                      # [SAR] scan: turn and hold a heading
        h = float(m.data)
        new = None if not math.isfinite(h) else h
        if self.hold is not None and new is None:
            t = self.now()
            self.nav.move_ref = None                           # scan time is not "no progress"
            self.nav.last_progress_t = t
            self.nav.blocked_since = None
        self.hold = new

    def on_command(self, m):                                   # [SAR] dashboard START / PAUSE
        cmd = m.data.strip().upper()
        t = self.now()
        if cmd == 'START' and not self.started:
            self.started = True
            self.nav.t0 = t - self.start_delay
            self.nav.start = None
            self.nav._event('NAVIGATING', 'START pressed: A -> B mission started', hold=2.0)
        elif cmd in ('PAUSE', 'STOP') and self.started:
            self.started = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ loop
    def tick(self):
        if self.pose is None:
            return
        t = self.now()
        if self.pending_map is not None and t - self.last_build >= self.map_period:
            self.last_build = t
            m, self.pending_map = self.pending_map, None
            self._build(m)
            self.last_stamp = t
        elif self.op.enabled and self.base_cls is not None and t - self.last_stamp >= self.op.stamp_period:
            # [DYN] obstacles moved: re-stamp them (or clear the old stamps) without waiting for terrain
            fresh = self._obstacles_fresh()
            if (self.obst_dirty and fresh and self.dyn.active()) or (self.had_overlay and not (fresh and self.dyn.active())):
                self.last_stamp = t
                self.obst_dirty = False
                if fresh and self.obst_msg is not None:
                    self.dyn.set_tracks(tracks_from_msg(self.obst_msg, t))
                self._rebuild()
        if self.hold is not None:
            v, w = self.hold_tick()
        elif not self.started:
            v, w = 0.0, 0.0
            self.nav.t0 = t
        else:
            v, w = self.nav.tick(t, self.pose)
            if self.speed_limit > 0.0 and v > self.speed_limit:
                v = self.speed_limit
            if self.nav.state in self.nav.TERMINAL:
                v, w = 0.0, 0.0
            elif self.op.enabled and self._obstacles_fresh():
                v, w = self.dyn.apply(t, self.pose, self.nav, v, w, self.speed)   # [DYN] only lowers v
        if self.op.enabled and (not self.started or self.hold is not None or not self._obstacles_fresh()):
            # not driving (or no tracker): still report what is around, without a path
            if self._obstacles_fresh():
                self.dyn.risk = dict(status='CLEAR')
                self.dyn.status = 'CLEAR'
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.cmd.publish(tw)
        st = self.nav.status(self.pose)
        st.update(x=round(self.pose.x, 2), y=round(self.pose.y, 2), heading_deg=round(np.degrees(self.pose.yaw), 1),
                  speed=round(self.speed, 3), tilt=round(self.pose.tilt_deg, 1))
        st['started'] = self.started                            # [SAR] dashboard fields
        st['speed_limit'] = self.speed_limit
        if self.op.enabled:                                     # [DYN] collision status
            st['collision'] = self.dyn.report(self.pose)
            st['collision']['tracker'] = 'OK' if self._obstacles_fresh() else 'NO DATA'
            st['collision_status'] = st['collision']['status'] if self._obstacles_fresh() else 'NO TRACKER DATA'
            if self.dyn.reason and st.get('state') not in ('GOAL_REACHED',):
                st['obstacle_reason'] = self.dyn.reason
        if not self.started and self.hold is None:
            st['state'] = 'READY - SET GOAL B' if self.start_on_goal else 'READY - PRESS START'
            st['reason'] = f'set goal B on the map (current B = {self.nav.goal[0]:.1f}, {self.nav.goal[1]:.1f})'
        if self.hold is not None:
            st['state'] = 'SAR SCAN HOLD'
            st['hold_heading_deg'] = round(math.degrees(self.hold), 1)
        self.pub_status.publish(String(data=json.dumps(st)))
        key = (st.get('state'), st.get('reason'))
        if key != self.last_logged:                            # [SAR] every stop has a logged reason
            self.last_logged = key
            self.get_logger().info(f"state {st.get('state')} at ({self.pose.x:.1f}, {self.pose.y:.1f}) "
                                   f"cmd v={v:.2f} w={w:.2f} speed={self.speed:.2f}: {st.get('reason', '')}"
                                   + (f" (last block: {st['last_block']})" if st.get('last_block') else ''))
        if self.nav.path_version != self.published_version or self.nav.previous_path is not self.published_prev:
            self.republish_paths()

    def hold_tick(self):
        """[SAR] Stand still and turn to the requested heading, only if turning is safe."""
        err = wrap(self.hold - self.pose.yaw)
        if abs(err) <= self.hold_tol:
            return 0.0, 0.0
        pl = self.nav.planner
        if pl.classes is not None and not pl.rotation_clear(self.pose.x, self.pose.y, self.pose.yaw):
            return 0.0, 0.0
        return 0.0, max(-self.hold_w, min(self.hold_w, 1.5 * err))

    def republish_paths(self):
        stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(path_msg(self.nav.path, stamp))
        self.pub_prev.publish(path_msg(self.nav.previous_path, stamp))
        self.published_version = self.nav.path_version
        self.published_prev = self.nav.previous_path


def main():
    rclpy.init()
    node = CliffNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
