#!/usr/bin/python3
"""Whole-field crop-row mission for the supplied cotton field (lawnmower / boustrophedon).

Unchanged from the original driver: the straddle drive (tyres in the two aisles beside the row),
the row-centring control law, the 1.2 m/s cruise speed, the headland U-turn (0.85 m commanded
radius at 0.45 m/s), the cross-track crop-safety limit, the tractor keep-out, the camera-hazard
stop, the topics (cmd_vel_nav, crop_row/path, crop_row/mode, exploration/status).

Extended:
  * rows and row ends come from the field (config/agriculture.yaml + the world model), not from a
    launch number or a fixed 31.5 m: the mission runs row 1 .. last row, whatever that number is;
  * a row is complete only when the robot centre has passed the last plant (+ row_end_tolerance);
  * obstacle avoidance from /agriculture/obstacles (LiDAR + depth tracker, agri_obstacles.py):
      - a MOVING obstacle in the robot's corridor -> YIELD (hold) until it has left, up to yield_timeout
      - a STATIC obstacle inside the crop area    -> DETOUR: move to the neighbouring row line (tyres
        stay in aisles there), pass, move back to the SAME row, continue
      - an obstacle beyond the last plant         -> the row is finished early (no detour needed)
      - the U-turn area blocked                   -> wait if it moves, else a REVERSE U-turn (backing
        round into the next row) so the lawnmower pattern continues
      - BOTH turn areas blocked (e.g. the parked tractor at the east headland) for
        headland_block_timeout -> back straight down the row just driven (row-centred, tyres stay in
        the aisles, the crop there is already covered) up to headland_backoff_max, re-checking both
        turn areas every tick, and turn as soon as one is clear - instead of holding for ever
      - nothing possible yet                      -> hold and retry (the mission is never abandoned)
  * /crop_row/progress (JSON): row, total rows, direction, row ends, progress, coverage, avoidance,
    per-row coverage, rows driven, interrupted rows and uncovered stretches (for agri_supervisor.py).
  * /agriculture/autonomy/command hook (agri_supervisor.py; String):
      RECOVER      after a no-progress detection: straight back-off along the row line (the tyres stay
                   in the aisles), then the same row is resumed. Row phase only; never a teleport.
      SPEED <v>    cap on the commanded speed while autonomy is degraded (0 = none); curvature kept
      HOLD <why> / RESUME   safety hold while continuing would be unsafe (no pose / no obstacle sensing)
      END_ROW      a STATIC obstacle has blocked the row with no free side for block_timeout: the row
                   ends here (the rest is recorded as uncovered, for a revisit) and the lawnmower
                   pattern continues with the headland turn - instead of holding for ever
    Without the supervisor nothing here is active: the mission behaves exactly as before.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry, Path as NavPath  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from agri_mission_core import (Coverage, Detour, FieldGeometry, corridor_hits, find_config,  # noqa: E402
                               load_yaml, package_root, zone_hits)

OB_DEFAULTS = dict(enabled=True, robot_half_width=0.55, robot_half_length=0.85, corridor_margin=0.20,
                   obstacle_avoidance_distance=5.0, stop_distance=1.2, hazard_stop_distance=1.2,
                   yield_timeout=12.0, detour_speed_mps=0.6, detour_transition=4.0, detour_pass_clearance=1.0,
                   allow_outside_rows=True, headland_wait=8.0, retry_period=3.0, predict_horizon=3.0,
                   headland_block_timeout=5.0, headland_backoff_max=3.0, headland_backoff_speed=0.3)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class _CappedPublisher:
    """cmd_vel_nav publisher with the supervisor's speed cap (same curvature: w scales with v)."""

    def __init__(self, pub, owner):
        self.pub, self.owner = pub, owner

    def publish(self, msg):
        cap = self.owner.auto_cap
        v = float(msg.linear.x)
        if cap > 0.0 and abs(v) > cap:
            k = cap / abs(v)
            msg.linear.x = v * k
            msg.angular.z = float(msg.angular.z) * k
        self.pub.publish(msg)


class CropRowDriver(Node):
    def __init__(self):
        super().__init__('crop_row_driver')
        cfg_path = self.declare_parameter('config', find_config(__file__)).value
        cfg = load_yaml(cfg_path) if cfg_path else {}
        cfg.setdefault('field', dict(first_row_y=-13.418, row_spacing=1.22, field_width=26.84,
                                     crop_area_start=-13.9, crop_area_end=14.4, auto_row_detection='dimensions'))
        mission = cfg.setdefault('mission', {})
        # row_count > 0 on the command line still forces a number of rows (0 = from the field)
        forced = int(self.declare_parameter('row_count', 0).value)
        if forced > 0:
            cfg['field'] = dict(cfg['field'], auto_row_detection='fixed', total_rows=forced)
        self.geo = FieldGeometry(cfg, package_root(cfg_path) if cfg_path else None)
        self.ob = dict(OB_DEFAULTS, **cfg.get('obstacles', {}))
        self.speed = self.declare_parameter('cruise_speed_mps', float(mission.get('cruise_speed_mps', 1.20))).value
        self.guard = self.declare_parameter('max_cross_track_m', float(mission.get('max_cross_track_m', 0.36))).value
        self.rows = self.geo.n
        self.row_pitch = self.geo.spacing
        # The physical eight-wheel skid-steer has ~1.395x the requested curvature on soil. Command
        # 0.85 m to obtain the required 0.61 m physical semicircle radius and 1.22 m row shift.
        self.turn_radius = self.declare_parameter('headland_turn_radius_m', float(mission.get('headland_turn_radius_m', .85))).value
        self.turn_speed = self.declare_parameter('headland_turn_speed_mps', float(mission.get('headland_turn_speed_mps', .45))).value
        self.tractor_x = self.declare_parameter('tractor_x_m', 17.60).value
        self.tractor_y = self.declare_parameter('tractor_y_m', 17.40).value
        self.tractor_keepout = self.declare_parameter('tractor_keepout_m', 3.50).value
        self.pose = None
        self.start = None
        self.blocked_until = 0.0
        self.done = False
        self.last_report = ''
        self.phase = 'row'
        self.row = 0
        self.direction = 1
        self.side = 1                   # next row is at +y (+1) or -y (-1)
        self.turn_sign = 0
        self.turn_reverse = False
        self.turn_previous_yaw = None
        self.turn_angle = 0.0
        self.coverage = Coverage(self.geo)
        self.completed_rows = set()
        self.obstacles = []
        self.obstacles_t = -1e9
        self.avoid = None               # {'state', 'since', 'ob'}
        self.detour = None
        self.detours = 0
        self.reverse_turns = 0
        self.wait_since = None
        self.row_end_limit = None       # progress where an obstacle beyond the crop ends this row
        self.headland_backoff = None     # {'x', 'y'} while backing away from a blocked headland
        self.auto_cap = 0.0              # supervisor speed cap (0 = none)
        self.auto_hold = None            # supervisor safety hold reason
        self.recovery = None             # supervisor-requested back-off
        self.recoveries = 0
        self.rows_driven = []            # rows the lawnmower has left (in order)
        self.interrupted = []            # [{row, reason, ...}]
        self.cmd = _CappedPublisher(self.create_publisher(Twist, 'cmd_vel_nav', 10), self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'exploration/status', qos)
        self.path = self.create_publisher(NavPath, 'crop_row/path', qos)
        self.mode = self.create_publisher(String, 'crop_row/mode', qos)
        self.progress_pub = self.create_publisher(String, 'crop_row/progress', qos)
        # Gazebo's world-frame model odometry (simulator pose truth, as before).
        self.create_subscription(Odometry, 'sim/ground_truth', self.odometry, 10)
        # The RGB-D hazard stream excludes green crop foliage while retaining people, equipment,
        # rocks and ditches: the existing last-resort stop.
        self.create_subscription(PointCloud2, 'perception/hazards', self.hazards, qos_profile_sensor_data)
        self.create_subscription(String, 'agriculture/obstacles', self.on_obstacles, 10)
        self.create_subscription(String, 'agriculture/autonomy/command', self.on_autonomy, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0 / float(mission.get('progress_rate_hz', 5.0)), self.publish_progress)
        self.get_logger().info(f'field: {self.geo.summary()}')

    # ------------------------------------------------------------------ inputs
    def odometry(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))

    def hazards(self, msg):
        try:
            p = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            return
        # Crop foliage is excluded upstream. A live stop for a person, rock or equipment in either
        # protected wheel lane. With the obstacle tracker running, avoidance acts first and this
        # stop only covers the last hazard_stop_distance; without it, the original 3.0 m.
        if self.phase in ('reverse_turn', 'headland_turn'):
            return
        reach = 0.65 + (float(self.ob['hazard_stop_distance']) if self.tracker_alive() else 2.35)
        if len(p) and np.any((p[:, 0] > .65) & (p[:, 0] < reach) & (np.abs(p[:, 1]) < .62)):
            self.blocked_until = time.monotonic() + .6

    def on_obstacles(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self.obstacles = [o for o in data.get('obstacles', []) if o.get('confirmed', True)]
        self.obstacles_t = time.monotonic()

    def on_autonomy(self, msg):
        """Supervisor commands (see the module doc)."""
        cmd = msg.data.strip()
        word = cmd.split(' ', 1)[0].upper()
        if word == 'SPEED':
            try:
                self.auto_cap = max(0.0, float(cmd.split()[1]))
            except (IndexError, ValueError):
                pass
        elif word == 'HOLD':
            self.auto_hold = cmd[5:].strip() or 'supervisor hold'
        elif word == 'RESUME':
            self.auto_hold = None
            self.last_report = ''
        elif word == 'END_ROW' and self.phase == 'row' and not self.done and self.pose is not None and not self.detour:
            d = self.direction
            p = self.geo.progress(self.pose[0], d)
            short = self.geo.crop_end_progress(d) - p
            self.row_end_limit = p + 0.1
            self.interrupted.append(dict(row=self.row + 1, reason='row blocked, ended early', shortfall=round(max(0.0, short), 2),
                                         obstacle=((self.avoid or {}).get('ob') or {}).get('cls', 'obstacle')))
            self.avoid = None
            self.report(f'Row {self.row + 1} blocked with no free side: row ended here ({short:.1f} m left for a '
                        f'revisit), continuing with the next row')
        elif word == 'RECOVER' and self.phase == 'row' and not self.done and self.pose is not None:
            self.recovery = dict(since=time.monotonic(), x=self.pose[0], y=self.pose[1])
            self.recoveries += 1
            self.report(f'Row {self.row + 1}: no progress - backing off straight along the row, then retrying')

    def tracker_alive(self):
        return bool(self.ob['enabled']) and time.monotonic() - self.obstacles_t < 2.0

    # ------------------------------------------------------------------ outputs
    def report(self, text, **extra):
        if text == self.last_report:
            return
        self.last_report = text
        record = {'state': 'row_complete' if self.done else 'crop_row', 'message': text,
                  'sim_time': self.get_clock().now().nanoseconds * 1e-9, 'row': self.row + 1,
                  'total_rows': self.rows, **extra}
        self.status.publish(String(data=json.dumps(record)))
        self.get_logger().info(text)

    def publish_path(self):
        msg = NavPath(); msg.header.frame_id = 'odom'; msg.header.stamp = self.get_clock().now().to_msg()
        d, row = self.direction, self.row
        while 0 <= row < self.rows:
            y = float(self.geo.row_y(row))
            for x in np.linspace(self.geo.start_x(d), self.geo.turn_x(d), 22):
                pose = PoseStamped(); pose.header = msg.header; pose.pose.position.x = float(x); pose.pose.position.y = y; pose.pose.orientation.w = 1.0; msg.poses.append(pose)
            d, row = -d, row + self.side
        self.path.publish(msg)

    def publish_progress(self):
        if self.start is None:
            return
        d = self.direction
        p = self.geo.progress(self.pose[0], d) if self.pose else 0.0
        av = None
        if self.avoid:
            ob = self.avoid.get('ob') or {}
            av = dict(state=self.avoid['state'], id=ob.get('id'), cls=ob.get('cls', 'obstacle'),
                      dynamic=bool(ob.get('dynamic')), **self._rel(ob))
        self.progress_pub.publish(String(data=json.dumps(dict(
            phase='done' if self.done else self.phase, row=self.row + 1, total_rows=self.rows,
            direction='EAST' if d > 0 else 'WEST', row_y=round(float(self.geo.row_y(self.row)), 3),
            row_start_x=self.geo.start_x(d), row_end_x=self.geo.turn_x(d),
            crop_start_x=self.geo.x_start, crop_end_x=self.geo.x_end, progress_m=round(p, 2),
            row_length_m=round(self.geo.row_length(d), 2), row_fraction=round(self.coverage.row_fraction(self.row), 3),
            coverage_pct=round(self.coverage.percent(), 2), rows_completed=len(self.completed_rows),
            avoidance=av, detours=self.detours, reverse_turns=self.reverse_turns, done=self.done,
            tracker=self.tracker_alive(), field=self.geo.summary(),
            row_fractions=[round(self.coverage.row_fraction(k), 3) for k in range(self.rows)],
            rows_driven=[k + 1 for k in self.rows_driven], interrupted_rows=self.interrupted[-50:],
            gaps=self.coverage_gaps(), recoveries=self.recoveries,
            autonomy=dict(hold=self.auto_hold, speed_cap=self.auto_cap or None,
                          recovering=self.recovery is not None)))))

    def coverage_gaps(self, min_len=0.5):
        """Uncovered stretches [row, x0, x1] of rows the rover has already driven or completed."""
        out = []
        rows = sorted(set(self.rows_driven) | set(self.completed_rows))
        b = self.coverage.bin
        for k in rows:
            done = self.coverage.done[k]
            i = 0
            while i < len(done):
                if not done[i]:
                    j = i
                    while j < len(done) and not done[j]:
                        j += 1
                    if (j - i) * b >= min_len:
                        out.append([k + 1, round(self.geo.x_start + i * b, 2), round(self.geo.x_start + j * b, 2)])
                    i = j
                else:
                    i += 1
        return out

    def _rel(self, ob):
        if not ob or self.pose is None:
            return {}
        x, y, yaw = self.pose
        dx, dy = ob.get('x', x) - x, ob.get('y', y) - y
        bearing = math.degrees(wrap(math.atan2(dy, dx) - yaw))
        side = 'AHEAD' if abs(bearing) < 20 else ('LEFT' if bearing > 0 else 'RIGHT')
        if abs(bearing) > 120:
            side = 'BEHIND'
        return dict(distance=round(math.hypot(dx, dy), 2), bearing=round(bearing, 1), side=side)

    def set_mode(self, mode):
        self.mode.publish(String(data=mode))

    def stop(self, reason, **extra):
        self.cmd.publish(Twist())
        self.done = True
        self.report(reason, **extra)

    def hold(self, state, ob, text, **extra):
        self.cmd.publish(Twist())
        self.avoid = dict(state=state, ob=ob, since=(self.avoid or {}).get('since', time.monotonic()))
        self.report(text, **extra)

    def tractor_clear(self):
        """Prevent every motion command from entering the tractor keep-out."""
        x, y, _ = self.pose
        distance = math.hypot(x - self.tractor_x, y - self.tractor_y)
        if distance >= self.tractor_keepout:
            return True
        self.cmd.publish(Twist())
        self.report('Tractor safety hold: no-contact keep-out zone active',
                    tractor_distance_m=round(distance, 3),
                    keepout_m=self.tractor_keepout)
        return False

    # ------------------------------------------------------------------ main loop
    def tick(self):
        if self.pose is None or self.done:
            return
        if self.start is None:
            self.start = self.pose
            self.row = self.geo.nearest_row(self.pose[1])
            self.direction = 1 if math.cos(self.pose[2]) >= 0 else -1
            self.side = 1 if self.row < self.rows - 1 else -1
            self.publish_path()
            self.set_mode('row')
            self.report(f'Row {self.row + 1} straddle drive started: tyres are in the two empty crop aisles '
                        f'({self.rows} rows from the {self.geo.source})',
                        row_center_y=float(self.geo.row_y(self.row)), tyre_lane_offsets_m=[-0.5, 0.5],
                        cruise_speed_mps=self.speed)
            return
        if not self.tractor_clear():
            return
        if self.auto_hold:
            self.cmd.publish(Twist())
            self.report(f'Autonomy safety hold: {self.auto_hold}')
            return
        if self.recovery is not None:
            self.back_off()
            return
        if self.phase in ('headland_turn', 'reverse_turn'):
            self.headland_turn()
            return
        if self.phase == 'row_end':
            self.row_end()
            return
        self.follow_row()

    def back_off(self):
        """Supervisor recovery: reverse straight (w = 0, tyres stay in their aisles) up to 0.6 m or
        3 s, only while the lanes behind are clear, then resume the same row."""
        x, y, _ = self.pose
        r = self.recovery
        moved = math.hypot(x - r['x'], y - r['y'])
        behind_blocked = False
        if self.tracker_alive():
            d, hl = self.direction, float(self.ob['robot_half_length'])
            half = self.corridor_half()
            pr = self.geo.progress(x, -d)          # progress measured in the reverse direction
            behind_blocked = bool(corridor_hits(self.obstacles, float(self.geo.row_y(self.row)), -d, self.geo,
                                                pr, pr + hl + 1.2, half))
        if moved >= 0.6 or time.monotonic() - r['since'] >= 3.0 or behind_blocked:
            self.cmd.publish(Twist())
            self.recovery = None
            self.report(f'Row {self.row + 1}: back-off done ({moved:.2f} m) - resuming the row')
            return
        out = Twist()
        out.linear.x = -0.3
        self.cmd.publish(out)

    def corridor_half(self):
        return float(self.ob['robot_half_width']) + float(self.ob['corridor_margin'])

    def follow_row(self):
        x, y, yaw = self.pose
        d = self.direction
        geo = self.geo
        row_y = float(geo.row_y(self.row))
        p = geo.progress(x, d)
        lat, slope = self.detour.offset(p) if self.detour else (0.0, 0.0)
        target_y = row_y + lat
        error_y = y - target_y
        if abs(error_y) > self.guard:
            self.stop('Crop safety stop: tyre lane drift limit reached', cross_track_m=abs(error_y), limit_m=self.guard)
            return
        # coverage: the row is covered where the robot straddles it, or passes beside it on a detour
        self.coverage.mark(self.row, x, detour=self.detour is not None)
        if self.row not in self.completed_rows and p >= geo.crop_end_progress(d):
            self.completed_rows.add(self.row)
            self.report(f'Row {self.row + 1} complete: robot passed the last plant', completed_row=self.row + 1)
        end = geo.row_length(d) if self.row_end_limit is None else min(geo.row_length(d), self.row_end_limit)
        blocked_end = self.row_end_limit is not None and p >= end - .20
        if (p >= end - .20 and self.row in self.completed_rows or blocked_end) and not self.detour:
            if self.row not in self.completed_rows:
                self.completed_rows.add(self.row)
                self.report(f'Row {self.row + 1} complete: stopped before the obstacle at the row end',
                            completed_row=self.row + 1)
            self.phase = 'row_end'
            self.cmd.publish(Twist())
            self.wait_since = None
            return
        speed = self.speed
        if self.detour:
            speed = min(speed, float(self.ob['detour_speed_mps']))
            ph = self.detour.phase(p)
            if ph == 'DONE':
                self.report(f'Back on row {self.row + 1}: obstacle passed, crop-row coverage continues')
                self.detour = None
                self.avoid = None
            else:
                self.avoid = dict(self.avoid or {}, state=ph)
        if self.ob['enabled'] and self.tracker_alive():
            speed = self.plan_obstacles(p, target_y, speed)
            if speed is None:
                return
        if time.monotonic() < self.blocked_until:
            self.cmd.publish(Twist())
            self.report('Protected obstacle stop: obstacle detected in a wheel lane')
            return
        # Pure-pursuit-style row centring (original law). The drive node still limits this command
        # to the rover's 2 m minimum turning radius. A detour adds its heading as feed-forward.
        heading = (0.0 if d > 0 else math.pi) + d * math.atan(slope)
        steering = 1.6 * wrap(heading - yaw) - d * 2.4 * math.atan(error_y / 1.2)
        out = Twist()
        out.linear.x = float(speed * (1.0 - 0.55 * min(abs(error_y) / self.guard, 1.0)))
        out.angular.z = float(np.clip(steering, -self.speed / 2.0, self.speed / 2.0))
        self.cmd.publish(out)

    # ------------------------------------------------------------------ obstacles in the row
    def plan_obstacles(self, p, target_y, speed):
        """Returns the allowed speed, or None when this tick already published a hold."""
        geo, d, ob = self.geo, self.direction, self.ob
        hl, half = float(ob['robot_half_length']), self.corridor_half()
        reach = float(ob['obstacle_avoidance_distance'])
        stop = float(ob['stop_distance'])
        hits = corridor_hits(self.obstacles, target_y, d, geo, p, p + hl + reach, half,
                             horizon=float(ob['predict_horizon']))
        if self.detour:
            # the avoided obstacle is beside us by design; anything else on the detour line -> hold
            aid = ((self.avoid or {}).get('ob') or {}).get('id')
            others = [h for h in hits if h[2].get('id') != aid]
            if others and others[0][0] - (p + hl) < stop:
                self.hold('BLOCKED ON DETOUR', others[0][2], 'Detour line blocked: holding until clear')
                return None
            tracked = [o for o in self.obstacles if o.get('id') == aid]
            if tracked:
                h = corridor_hits(tracked, float(geo.row_y(self.row)), d, geo, -1e9, 1e9, half)
                if h:
                    self.detour.p1 = max(self.detour.p1, h[0][1] + hl + float(ob['detour_pass_clearance']))
            return speed
        if not hits:
            if self.avoid and self.avoid.get('state') in ('YIELDING', 'WAITING'):
                self.report(f'Row {self.row + 1} clear again: continuing')
            if not (self.avoid and self.avoid.get('state') == 'ROW END BLOCKED'):
                self.avoid = None
            return speed
        p_near, p_far, obst = hits[0]
        gap = p_near - (p + hl)                     # robot front to obstacle
        crop_end = geo.crop_end_progress(d)
        now = time.monotonic()
        # an obstacle standing at the end of the row (in the last row_end_zone metres and reaching
        # past the last plant, e.g. a parked tractor in the headland): no detour out of the field;
        # the row ends at the stop distance (any shortfall stays visible in the coverage) and the
        # headland logic turns into the next row (reverse U-turn if needed)
        zone = float(ob.get('row_end_zone', 2.5))
        if p_near - hl - stop >= crop_end or (p_far > crop_end and p_near > crop_end - zone):
            limit = p_near - hl - stop
            if self.row_end_limit is None or abs(limit - self.row_end_limit) > 0.3:
                short = crop_end - limit
                self.interrupted = [i for i in self.interrupted
                                    if not (i['row'] == self.row + 1 and i['reason'] == 'row end blocked')]
                self.interrupted.append(dict(row=self.row + 1, reason='row end blocked', obstacle=obst.get('cls', 'obstacle'),
                                             shortfall=round(max(0.0, short), 2)))
                self.report(f'Row {self.row + 1}: {obst.get("cls", "obstacle")} at the row end'
                            + (f', row ends {short:.1f} m early' if short > 0 else ''), obstacle=obst.get('id'))
            self.row_end_limit = limit
            self.avoid = dict(state='ROW END BLOCKED', ob=obst, since=now)
            if gap < stop:
                self.cmd.publish(Twist())
                return None
            return min(speed, max(0.25, 0.5 * gap))
        if obst.get('dynamic'):
            if gap < stop + 1.5:
                if not self.avoid or self.avoid.get('state') != 'YIELDING':
                    self.avoid = dict(state='YIELDING', ob=obst, since=now)
                    self.report(f'Moving obstacle in row {self.row + 1}: yielding', obstacle=obst.get('id'),
                                **self._rel(obst))
                if now - self.avoid['since'] < float(ob['yield_timeout']):
                    self.cmd.publish(Twist())
                    self.avoid['ob'] = obst
                    return None
                # it keeps standing in the way: go round it like a static obstacle (below)
            else:
                return min(speed, max(0.3, speed * gap / (stop + 4.0)))
        # static obstacle in the crop area: detour via a neighbouring row line
        if gap < float(ob['detour_transition']) + stop + 0.5:
            det = self.plan_detour(p, p_near, p_far, obst)
            if det is not None:
                self.detour = det
                self.detours += 1
                self.interrupted.append(dict(row=self.row + 1, reason='detour', obstacle=obst.get('cls', 'obstacle'),
                                             x=round(float(obst.get('x', 0.0)), 2)))
                self.avoid = dict(state='AVOIDING', ob=obst, since=now)
                self.report(f'Obstacle in row {self.row + 1}: detour via the {"left" if det.A * d > 0 else "right"} '
                            f'row line, then back to row {self.row + 1}', obstacle=obst.get('id'), **self._rel(obst))
                return min(speed, float(ob['detour_speed_mps']))
            if gap < stop + 0.3:
                self.hold('WAITING', obst, f'Row {self.row + 1} blocked and no free side: holding, retrying',
                          obstacle=obst.get('id'))
                return None
        return min(speed, max(0.3, speed * gap / (stop + 4.0)))

    def plan_detour(self, p, p_near, p_far, obst):
        geo, d, ob = self.geo, self.direction, self.ob
        L, hl, half = float(ob['detour_transition']), float(ob['robot_half_length']), self.corridor_half()
        stop = float(ob['stop_distance'])
        p0 = max(p, p_near - hl - stop - L)
        p1 = p_far + hl + float(ob['detour_pass_clearance'])
        row_y = float(geo.row_y(self.row))
        off = float(np.median(np.asarray(obst.get('points') or [[obst['x'], obst['y']]], float).reshape(-1, 2)[:, 1])) - row_y
        # prefer the side away from the obstacle, then the side of the rows still to do
        order = [self.side, -self.side] if abs(off) < 0.1 else [-1 if off > 0 else 1, 1 if off > 0 else -1]
        lo, hi = (-1, self.rows) if ob['allow_outside_rows'] else (0, self.rows - 1)
        if p1 + L > geo.row_length(d):
            return None                     # would end outside the row: handled as a row-end obstacle
        for s in order:
            if not lo <= self.row + s <= hi:
                continue
            y_new = row_y + s * geo.spacing
            hz = float(ob['predict_horizon'])
            on_line = corridor_hits(self.obstacles, y_new, d, geo, p0, p1 + L + hl, half, horizon=hz)
            crossing = corridor_hits(self.obstacles, (row_y + y_new) / 2, d, geo, p0, p0 + L + hl, half, horizon=hz)
            crossing += corridor_hits(self.obstacles, (row_y + y_new) / 2, d, geo, p1, p1 + L + hl, half, horizon=hz)
            if not on_line and not [h for h in crossing if h[2] is not obst]:
                return Detour(p0, p1, s * geo.spacing, L)
        return None

    # ------------------------------------------------------------------ end of a row
    def row_end(self):
        x, y, yaw = self.pose
        d, geo, ob = self.direction, self.geo, self.ob
        if not 0 <= self.row + self.side < self.rows:
            self.stop('Field row sweep complete: rover stopped in the clear headland', rows_completed=self.rows,
                      coverage_pct=round(self.coverage.percent(), 1), distance_m=geo.progress(x, d))
            return
        nxt = self.row + self.side
        y0, y1 = float(geo.row_y(self.row)), float(geo.row_y(nxt))
        # area swept by the rover body turning about a centre 0.61 m (physical radius) to the side:
        # every body point stays within 0.61 + corner distance of that centre
        corner = math.hypot(float(ob['robot_half_length']), float(ob['robot_half_width']))
        sweep = 0.61 + corner + float(ob['corridor_margin'])
        margin = corner - 0.61 + float(ob['corridor_margin'])
        ylo, yhi = min(y0, y1) - margin, max(y0, y1) + margin
        behind = float(ob['robot_half_length']) + float(ob['corridor_margin'])
        hz = float(ob['predict_horizon'])
        alive = self.tracker_alive()
        fwd = zone_hits(self.obstacles, x - d * behind, x + d * sweep, ylo, yhi, horizon=hz) if alive else []
        now = time.monotonic()
        if not fwd:
            self.start_turn(reverse=False)
            return
        moving = any(o.get('dynamic') for o in fwd)
        if self.wait_since is None:
            self.wait_since = now
            self.avoid = dict(state='HEADLAND BLOCKED', ob=fwd[0], since=now)
            self.report(f'Headland after row {self.row + 1} blocked by '
                        f'{"a moving" if moving else "a static"} {fwd[0].get("cls", "obstacle")}', **self._rel(fwd[0]))
        if moving and now - self.wait_since < float(ob['headland_wait']):
            self.cmd.publish(Twist())
            return
        back = zone_hits(self.obstacles, x - d * sweep, x + d * behind, ylo, yhi, horizon=hz)
        if not back:
            self.start_turn(reverse=True)
            return
        # both turn areas blocked: after headland_block_timeout back straight down the row just
        # driven (already covered) until one of the two turn areas is clear
        if now - self.wait_since >= float(ob['headland_block_timeout']) and self.headland_reverse_step():
            return
        self.cmd.publish(Twist())
        self.report(f'Headland after row {self.row + 1}: both turn areas blocked, holding and retrying')

    def headland_reverse_step(self):
        """One tick of the straight, row-centred back-off from a blocked headland. False when it
        cannot (limit reached, lane behind blocked, off the row line)."""
        x, y, yaw = self.pose
        d, ob = self.direction, self.ob
        if self.headland_backoff is None:
            self.headland_backoff = dict(x=x, y=y)
            self.report(f'Headland after row {self.row + 1}: both turn areas blocked - backing down row '
                        f'{self.row + 1} (already covered) to make room for the U-turn')
        hb = self.headland_backoff
        moved = math.hypot(x - hb['x'], y - hb['y'])
        row_y = float(self.geo.row_y(self.row))
        error_y = y - row_y
        behind_blocked = False
        if self.tracker_alive():
            pr = self.geo.progress(x, -d)
            behind_blocked = bool(corridor_hits(self.obstacles, row_y, -d, self.geo, pr,
                                                pr + float(ob['robot_half_length']) + 1.2, self.corridor_half()))
        if moved >= float(ob['headland_backoff_max']) or behind_blocked or abs(error_y) > 0.8 * self.guard:
            return False
        heading = 0.0 if d > 0 else math.pi
        out = Twist()
        out.linear.x = -float(ob['headland_backoff_speed'])
        # reversing: the heading term is unchanged, the cross-track term changes sign
        out.angular.z = 1.6 * wrap(heading - yaw) + d * 2.4 * math.atan(error_y / 1.2)
        self.cmd.publish(out)
        return True

    def start_turn(self, reverse):
        yaw = self.pose[2]
        self.phase = 'reverse_turn' if reverse else 'headland_turn'
        self.turn_reverse = reverse
        self.turn_sign = self.side * self.direction
        self.turn_previous_yaw = yaw
        self.turn_angle = 0.0
        self.avoid = dict(state='REVERSE U-TURN', ob=(self.avoid or {}).get('ob'), since=time.monotonic()) if reverse else None
        self.row_end_limit = None
        self.wait_since = None
        if self.headland_backoff is not None:
            moved = math.hypot(self.pose[0] - self.headland_backoff['x'], self.pose[1] - self.headland_backoff['y'])
            self.report(f'Headland back-off {moved:.1f} m: turn area clear')
            self.headland_backoff = None
        if reverse:
            self.reverse_turns += 1
        self.set_mode('headland')
        self.report(('Reverse U-turn started (headland ahead blocked)' if reverse else 'Clear-headland U-turn started'),
                    completed_row=self.row + 1, next_row=self.row + self.side + 1)

    def headland_turn(self):
        _, y, yaw = self.pose
        self.turn_angle += wrap(yaw-self.turn_previous_yaw)
        self.turn_previous_yaw = yaw
        if abs(self.turn_angle) >= math.pi-.04:
            self.cmd.publish(Twist())
            self.rows_driven.append(self.row)
            self.row += self.side
            self.direction *= -1
            self.phase = 'row'
            self.avoid = None
            expected_y = float(self.geo.row_y(self.row))
            self.set_mode('row')
            if abs(y-expected_y) > self.guard:
                self.stop('Crop safety stop: U-turn did not enter the next protected lane', cross_track_m=abs(y-expected_y), limit_m=self.guard)
                return
            error = abs(y-expected_y)
            self.report(f'U-turn complete: entering next protected crop row (cross-track {error:.3f} m)', row=self.row + 1, cross_track_m=error)
            return
        out = Twist()
        if self.turn_reverse:
            out.linear.x = -self.turn_speed
            out.angular.z = -self.turn_sign * self.turn_speed / self.turn_radius
        else:
            out.linear.x = self.turn_speed
            out.angular.z = self.turn_sign * self.turn_speed / self.turn_radius
        self.cmd.publish(out)


def main():
    rclpy.init(); node = CropRowDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
