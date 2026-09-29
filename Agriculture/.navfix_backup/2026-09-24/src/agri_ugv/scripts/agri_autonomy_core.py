#!/usr/bin/python3
"""Agriculture autonomy supervisor - the reasoning, without ROS (agri_supervisor.py is the node).

It sits AROUND the existing field-coverage mission (crop_row_driver.py) and never replaces it:
the row-wise boustrophedon, row following, detours, yields, reverse U-turns and the START/STOP gate
stay exactly as they are. From the real topics it answers:

  Where am I, how sure am I?   localization confidence from the pose stream, the IMU yaw rate and
                               the wheel encoders (no invented number: each factor is reported)
  Can I still see?             sensor health (RGB, depth, LiDAR, IMU, encoders, pose, obstacle
                               tracker) -> FULL / DEGRADED / LIMITED autonomy / SAFETY HOLD
  What is the field?           rows, spacing, direction, boundary (from the driver's FieldGeometry)
  What is done?                completed / remaining / interrupted rows, uncovered stretches
  Am I making progress?        commanded motion vs actual pose motion -> NO_PROGRESS -> recovery
  What now?                    mission state + one decision (CONTINUE, SLOW_DOWN, AVOID, ...)
  Is the survey complete?      moisture-map %, dry / wet / unmapped share, MOISTURE MAP COMPLETE

Actions it may take (all through crop_row_driver's /agriculture/autonomy/command hook):
  RECOVER       after NO_PROGRESS: controlled straight back-off along the row (tyres stay in the
                aisles), then the driver resumes the same row. Never a teleport.
  SPEED <v>     speed cap while autonomy is degraded (0 = no cap)
  HOLD <why> / RESUME   safety hold only when continuing would be unsafe (no pose, or no obstacle
                sensing at all)
"""
import math
import time
from collections import deque

STATES = ('READY', 'FIELD_ANALYSIS', 'ROW_NAVIGATION', 'OBSTACLE_AVOIDANCE', 'ROW_RECOVERY', 'ROW_REJOIN',
          'MOISTURE_MAPPING', 'REVISIT_REQUIRED', 'COVERAGE_VERIFICATION', 'MISSION_COMPLETE', 'RECOVERY', 'FAULT')
DECISIONS = ('CONTINUE', 'SLOW_DOWN', 'AVOID', 'REPLAN', 'BACKTRACK', 'RECOVER', 'WAIT TEMPORARILY', 'RESUME',
             'ABORT CURRENT SUBTASK', 'COMPLETE MISSION')

DEFAULTS = dict(
    # sensors: expected minimum rate (Hz) and the age (s) after which a stream counts as lost
    sensors=dict(pose=dict(rate=5.0, stale=1.0), imu=dict(rate=20.0, stale=1.0), encoders=dict(rate=10.0, stale=1.0),
                 rgb=dict(rate=2.0, stale=3.0), depth=dict(rate=2.0, stale=3.0), lidar=dict(rate=2.0, stale=3.0),
                 obstacle_tracker=dict(rate=1.0, stale=3.0)),
    # no-progress
    stuck_time=6.0, stuck_distance=0.25, min_command=0.08, recover_limit=3, recover_window=120.0,
    # speed caps (m/s) per autonomy level
    degraded_speed=0.8, limited_speed=0.5,
    # clearance metrics (m, robot footprint to obstacle points)
    collision_clearance=0.05, near_collision_clearance=0.5,
    robot_half_length=0.85, robot_half_width=0.55,
    # coverage verification
    row_complete_fraction=0.98, min_gap=0.5, verify_time=2.0,
    field_analysis_time=3.0,
    block_timeout=20.0,        # s held by a STATIC obstacle with no free side -> end the row (revisit later)
)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class EventLog:
    def __init__(self, n=200):
        self.items = deque(maxlen=n)
        self.last = {}

    def add(self, t, text, kind='info', dedup=5.0):
        if t - self.last.get(text, -1e9) < dedup:
            return
        self.last[text] = t
        self.items.append(dict(t=round(t, 1), wall=time.strftime('%H:%M:%S'), text=text, kind=kind))


class SensorHealth:
    """Per stream: arrival times -> OK / SLOW / LOST / NEVER (rate measured over the last 5 s)."""

    def __init__(self, spec):
        self.spec = spec
        self.times = {k: deque(maxlen=200) for k in spec}

    def beat(self, name, t):
        if name in self.times:
            self.times[name].append(t)

    def state(self, name, t):
        q = self.times[name]
        if not q:
            return 'NEVER', 0.0
        age = t - q[-1]
        recent = [x for x in q if t - x <= 5.0]
        rate = (len(recent) - 1) / max(1e-3, recent[-1] - recent[0]) if len(recent) > 1 else 0.0
        if age > self.spec[name]['stale']:
            return 'LOST', rate
        if rate and rate < 0.5 * self.spec[name]['rate']:
            return 'SLOW', rate
        return 'OK', rate

    def report(self, t):
        return {k: dict(zip(('state', 'rate'), (s, round(r, 1)))) for k in self.spec for s, r in [self.state(k, t)]}


class LocalizationMonitor:
    """Confidence in the pose the mission drives on, from independent measurements:
      fresh    - age of the latest pose (Gazebo odometry of the rover)
      imu      - yaw rate from the IMU vs yaw rate of the pose stream
      wheels   - speed from the wheel encoders vs speed of the pose stream (slip / skid)
      jumps    - pose steps larger than the rover can physically make
    confidence = 100 x fresh x imu x wheels x jumps (each factor 0..1, reported separately)."""

    def __init__(self, max_speed=3.0):
        self.max_speed = max_speed
        self.pose = None            # (t, x, y, yaw)
        self.v_pose = self.w_pose = 0.0
        self.v_wheel = self.w_imu = None
        self.imu_err = 0.0
        self.slip = 0.0
        self.jump_penalty = 0.0
        self.t_pose = -1e9

    def on_pose(self, t, x, y, yaw):
        if self.pose is not None:
            dt = t - self.pose[0]
            if dt > 1e-3:
                d = math.hypot(x - self.pose[1], y - self.pose[2])
                if d > self.max_speed * dt + 0.5:
                    self.jump_penalty = 1.0
                v = d / dt
                w = _wrap(yaw - self.pose[3]) / dt
                self.v_pose += 0.3 * (v - self.v_pose)
                self.w_pose += 0.3 * (w - self.w_pose)
                self.jump_penalty = max(0.0, self.jump_penalty - 0.02)
        self.pose = (t, x, y, yaw)
        self.t_pose = t
        if self.w_imu is not None:
            self.imu_err += 0.1 * (abs(self.w_imu - self.w_pose) - self.imu_err)
        if self.v_wheel is not None:
            s = abs(abs(self.v_wheel) - self.v_pose) / max(0.3, abs(self.v_wheel), self.v_pose)
            self.slip += 0.1 * (s - self.slip)

    def on_imu(self, wz):
        self.w_imu = float(wz)

    def on_wheels(self, v):
        self.v_wheel = float(v)

    def factors(self, t):
        age = t - self.t_pose
        fresh = 1.0 if age < 0.5 else max(0.0, 1.0 - (age - 0.5) / 1.5)
        imu = 1.0 if self.w_imu is None else max(0.3, 1.0 - self.imu_err / 0.6)
        wheels = 1.0 if self.v_wheel is None else max(0.5, 1.0 - 0.8 * self.slip)
        jumps = 1.0 - 0.7 * self.jump_penalty
        return dict(fresh=round(fresh, 2), imu=round(imu, 2), wheels=round(wheels, 2), jumps=round(jumps, 2),
                    imu_available=self.w_imu is not None, encoders_available=self.v_wheel is not None,
                    wheel_slip=round(self.slip, 2))

    def confidence(self, t):
        f = self.factors(t)
        return round(100.0 * f['fresh'] * f['imu'] * f['wheels'] * f['jumps'], 0)


class ProgressMonitor:
    """Commanded motion vs actual motion. NO_PROGRESS = the rover has been told to move (|v| above
    min_command) for stuck_time, but the pose moved less than stuck_distance in that time."""

    def __init__(self, stuck_time, stuck_distance, min_command):
        self.T, self.D, self.vmin = stuck_time, stuck_distance, min_command
        self.cmd_since = None
        self.ref = None
        self.cmd_v = 0.0
        self.events = 0

    def on_cmd(self, t, v):
        self.cmd_v = v
        if abs(v) >= self.vmin:
            if self.cmd_since is None:
                self.cmd_since = t
        else:
            self.cmd_since = None
            self.ref = None

    def check(self, t, pose):
        """pose = (x, y). True once when NO_PROGRESS is detected."""
        if self.cmd_since is None or pose is None:
            return False
        if self.ref is None:
            self.ref = (t, pose)
            return False
        if math.hypot(pose[0] - self.ref[1][0], pose[1] - self.ref[1][1]) > self.D:
            self.ref = (t, pose)
            return False
        if t - self.ref[0] >= self.T and t - self.cmd_since >= self.T:
            self.ref = (t, pose)
            self.events += 1
            return True
        return False

    def reset(self):
        self.cmd_since = None
        self.ref = None


def footprint_clearance(pose, points, half_l, half_w):
    """Distance from the rover's rectangle to the nearest obstacle point (negative = inside)."""
    if pose is None or not points:
        return None
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    best = math.inf
    for px, py in points:
        dx, dy = px - x, py - y
        u, w = c * dx + s * dy, -s * dx + c * dy
        ex, ey = abs(u) - half_l, abs(w) - half_w
        d = math.hypot(max(ex, 0.0), max(ey, 0.0)) if (ex > 0 or ey > 0) else max(ex, ey)
        best = min(best, d)
    return best


class AgriSupervisor:
    AVOID_STATES = ('AVOIDING', 'PASSING OBSTACLE', 'YIELDING', 'WAITING', 'BLOCKED ON DETOUR', 'HEADLAND BLOCKED',
                    'ROW END BLOCKED', 'REVERSE U-TURN')

    def __init__(self, cfg=None):
        c = dict(DEFAULTS)
        for k, v in (cfg or {}).items():
            if k == 'sensors' and isinstance(v, dict):
                c['sensors'] = {n: dict(DEFAULTS['sensors'].get(n, {}), **(s or {})) for n, s in
                                dict(DEFAULTS['sensors'], **v).items()}
            else:
                c[k] = v
        self.c = c
        self.health = SensorHealth(c['sensors'])
        self.loc = LocalizationMonitor()
        self.prog = ProgressMonitor(float(c['stuck_time']), float(c['stuck_distance']), float(c['min_command']))
        self.log = EventLog()
        self.pose = None
        self.progress = {}
        self.gate = None
        self.driver_msg = ''
        self.driver_done = False
        self.obstacles = []
        self.moisture = {}
        self.moisture_now = None
        self.vision = {}
        self.state = 'READY'
        self.decision = 'CONTINUE'
        self.level = 'FULL'
        self.hold = None                 # reason while a safety hold is commanded
        self.speed_cap = 0.0
        self.recovering_until = None
        self.recoveries = deque()
        self.recoveries_total = 0
        self.manual = 0
        self.t_start = None
        self.t_done = None
        self.distance = 0.0
        self.last_xy = None
        self.min_clear = None
        self.collisions = self.near = 0
        self._in_contact = self._in_near = False
        self.prev_avoid = None
        self.prev_row = None
        self.prev_detours = 0
        self.moisture_complete_logged = False
        self.last_decision_change = 0.0
        self.commands = []               # pending commands for the driver
        self.blocked_since = None
        self.decision_override = None

    # ------------------------------------------------------------------ inputs
    def on_pose(self, t, x, y, yaw):
        self.health.beat('pose', t)
        self.loc.on_pose(t, x, y, yaw)
        if self.last_xy is not None and self.running():
            self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.pose = (x, y, yaw)

    def on_imu(self, t, wz):
        self.health.beat('imu', t)
        self.loc.on_imu(wz)

    def on_encoders(self, t, v_wheel):
        self.health.beat('encoders', t)
        self.loc.on_wheels(v_wheel)

    def on_sensor(self, t, name):
        self.health.beat(name, t)

    def on_cmd(self, t, v, w=0.0):
        self.prog.on_cmd(t, v)

    def on_gate(self, t, state):
        if state != self.gate:
            if state == 'RUNNING' and self.t_start is None:
                self.t_start = t
                self.log.add(t, 'Mission started: field coverage', 'mission')
            elif state == 'STOPPED' and self.gate == 'RUNNING':
                self.log.add(t, 'Operator STOP', 'operator')
            elif state == 'RUNNING' and self.gate in ('STOPPED', 'HALTED'):
                self.log.add(t, 'Operator restart', 'operator')
        self.gate = state

    def on_operator(self, t, cmd):
        """Dashboard START/STOP. A STOP during a running mission, or a restart after a fault, is a
        manual intervention (the first START is not)."""
        cmd = cmd.strip().upper()
        if (cmd == 'STOP' and self.gate == 'RUNNING') or (cmd == 'START' and self.state == 'FAULT'):
            self.manual += 1

    def on_driver(self, t, status):
        msg = status.get('message', '')
        if msg and msg != self.driver_msg:
            self.driver_msg = msg
            kind = 'safety' if 'safety' in msg.lower() else 'mission'
            self.log.add(t, msg, kind, dedup=0.0)
        if status.get('state') == 'row_complete':
            self.driver_done = True

    def on_progress(self, t, p):
        self.progress = p
        if self.gate is None and self.t_start is None and not p.get('done'):
            self.t_start = t                   # no START gate (dashboard:=false): the driver starts itself
        row = p.get('row')
        if self.prev_row is not None and row != self.prev_row and not p.get('done'):
            self.log.add(t, f'Row {row} / {p.get("total_rows")} started', 'mission')
        self.prev_row = row
        if int(p.get('detours', 0)) > self.prev_detours:
            self.prev_detours = int(p.get('detours', 0))
            self.log.add(t, f'Row {row} interrupted: detour planned round the obstacle (row identity kept)',
                         'replan', dedup=0.0)

    def on_obstacles(self, t, obs):
        self.health.beat('obstacle_tracker', t)
        self.obstacles = obs or []

    def on_moisture_status(self, t, st):
        self.moisture = st or {}

    def on_moisture(self, t, reading):
        self.moisture_now = reading

    def on_vision(self, t, tracks):
        self.vision = dict(tracks or {}, t=t)

    # ------------------------------------------------------------------ reasoning
    def field(self):
        p = self.progress
        f = p.get('field') or {}
        n = int(f.get('rows', p.get('total_rows', 0)) or 0)
        fr = p.get('row_fractions') or []
        full = float(self.c['row_complete_fraction'])
        done_rows = [k + 1 for k, v in enumerate(fr) if v >= full]
        interrupted = p.get('interrupted_rows') or []
        cur = int(p.get('row', 0) or 0)
        # rows the lawnmower has passed (driven), and those still ahead of it
        passed = set(int(r) for r in p.get('rows_driven', []))
        remaining = [k for k in range(1, n + 1) if k not in done_rows and k not in passed and k != cur]
        gaps = [g for g in (p.get('gaps') or []) if g[2] - g[1] >= float(self.c['min_gap'])]
        bounds = None
        if f:
            y0, sp = f.get('first_row_y'), f.get('row_spacing')
            if y0 is not None and sp is not None and n:
                bounds = [f.get('crop_area_start'), f.get('crop_area_end'), round(y0 - sp / 2, 3),
                          round(y0 + (n - 1) * sp + sp / 2, 3)]
        return dict(rows=n, row_spacing=f.get('row_spacing'), row_direction='EAST-WEST (world x)',
                    boundary=bounds, source=f.get('source'), current_row=cur, completed_rows=done_rows,
                    remaining_rows=remaining, interrupted_rows=interrupted, uncovered=gaps,
                    coverage_pct=p.get('coverage_pct'))

    def perception_confidence(self, t):
        """Share of the perception that is actually working: obstacle sensing (LiDAR, depth, the
        tracker) weighs 60 %, crop/weed vision (RGB + the vision node's timeliness) 40 %."""
        st = {k: self.health.state(k, t)[0] for k in ('rgb', 'depth', 'lidar', 'obstacle_tracker')}
        ok = lambda k: 1.0 if st[k] == 'OK' else (0.6 if st[k] == 'SLOW' else 0.0)  # noqa: E731
        obstacle = 0.35 * ok('lidar') + 0.35 * ok('depth') + 0.30 * ok('obstacle_tracker')
        vision = ok('rgb')
        v = self.vision
        if v:
            age = t - v.get('t', -1e9)
            vision *= 1.0 if age < 3.0 else 0.3
            ms = float(v.get('ms') or 0.0)
            if ms > 500:
                vision *= 0.7
        else:
            vision *= 0.5            # camera alive, vision node not (yet) reporting
        return round(100.0 * (0.6 * obstacle + 0.4 * vision), 0), st

    def autonomy_level(self, t):
        h = {k: self.health.state(k, t)[0] for k in self.health.spec}
        lost = lambda k: h[k] in ('LOST', 'NEVER')  # noqa: E731
        if lost('pose'):
            return 'SAFETY HOLD', 'no pose: the rover cannot follow the rows'
        if lost('lidar') and lost('depth'):
            return 'SAFETY HOLD', 'no LiDAR and no depth camera: people and obstacles cannot be seen'
        reasons = []
        level = 'FULL'
        if lost('obstacle_tracker'):
            level, reasons = 'LIMITED', ['obstacle tracker silent: only the depth emergency stop protects the rover']
        if lost('lidar') or lost('depth'):
            level = 'LIMITED' if level == 'LIMITED' else 'DEGRADED'
            reasons.append(('LiDAR' if lost('lidar') else 'depth camera') + ' lost')
        for k, label in (('rgb', 'RGB camera (crop / weed vision)'), ('imu', 'IMU'), ('encoders', 'wheel encoders')):
            if lost(k):
                level = 'DEGRADED' if level == 'FULL' else level
                reasons.append(label + ' lost')
        return level, '; '.join(reasons)

    def running(self):
        return self.gate == 'RUNNING' or (self.gate is None and self.t_start is not None)

    def _metrics_clearance(self, t):
        pts = []
        for o in self.obstacles:
            pp = o.get('points') or [[o.get('x'), o.get('y')]]
            pts += [tuple(q[:2]) for q in pp if q and q[0] is not None]
        c = footprint_clearance(self.pose, pts, float(self.c['robot_half_length']), float(self.c['robot_half_width']))
        if c is None:
            return
        self.min_clear = c if self.min_clear is None else min(self.min_clear, c)
        contact = c <= float(self.c['collision_clearance'])
        near = c <= float(self.c['near_collision_clearance'])
        if contact and not self._in_contact:
            self.collisions += 1
            self.log.add(t, f'COLLISION: obstacle within {c:.2f} m of the footprint', 'safety', dedup=0.0)
        elif near and not self._in_near and not contact:
            self.near += 1
            self.log.add(t, f'Near miss: obstacle {c:.2f} m from the footprint', 'safety')
        self._in_contact, self._in_near = contact, near

    def step(self, t):
        """Reason once. Returns the status dict; pending driver commands are in self.commands."""
        c = self.c
        self.commands = []
        p = self.progress
        avoid = (p.get('avoidance') or {}).get('state')
        if self.running():
            self._metrics_clearance(t)
        # --- sensor health -> autonomy level (hold only when continuing would be unsafe)
        level, why = self.autonomy_level(t)
        if level != self.level:
            self.log.add(t, f'Autonomy {self.level} -> {level}' + (f': {why}' if why else ''), 'health', dedup=0.0)
        self.level = level
        running = self.running() and not p.get('done')
        if level == 'SAFETY HOLD' and running:
            if self.hold is None:
                self.hold = why
                self.commands.append(f'HOLD {why}')
        elif self.hold is not None:
            self.hold = None
            self.commands.append('RESUME')
            self.log.add(t, 'Sensors back: mission resumed', 'health', dedup=0.0)
        cap = {'DEGRADED': float(c['degraded_speed']), 'LIMITED': float(c['limited_speed'])}.get(level, 0.0)
        if cap != self.speed_cap:
            self.speed_cap = cap
            self.commands.append(f'SPEED {cap:g}')
        # --- no-progress -> controlled recovery (never while holding / yielding on purpose)
        if self.recovering_until is not None and t >= self.recovering_until:
            self.recovering_until = None
            self.log.add(t, f'Recovery finished: resuming row {p.get("row")}', 'recovery', dedup=0.0)
            self.prog.reset()
        if running and self.hold is None and self.recovering_until is None and \
                self.prog.check(t, self.pose[:2] if self.pose else None):
            while self.recoveries and t - self.recoveries[0] > float(c['recover_window']):
                self.recoveries.popleft()
            self.log.add(t, f'NO PROGRESS: commanded {self.prog.cmd_v:.2f} m/s but moved < '
                            f'{c["stuck_distance"]} m in {c["stuck_time"]:.0f} s', 'recovery', dedup=0.0)
            if len(self.recoveries) >= int(c['recover_limit']):
                self.state = 'FAULT'
                self.log.add(t, f'{len(self.recoveries)} recoveries in {c["recover_window"]:.0f} s did not help: '
                                'manual check needed (rover held)', 'safety', dedup=0.0)
                self.commands.append('HOLD repeated no-progress')
                self.hold = 'repeated no-progress'
            else:
                self.recoveries.append(t)
                self.recoveries_total += 1
                self.recovering_until = t + 4.0
                self.commands.append('RECOVER')
                self.log.add(t, 'Recovery: straight back-off along the row, then retry the same row', 'recovery',
                             dedup=0.0)
        # --- blocked for good by a static obstacle: end the row instead of holding for ever
        ob = (p.get('avoidance') or {})
        if running and avoid in ('WAITING', 'BLOCKED ON DETOUR') and not ob.get('dynamic') and self.hold is None:
            if self.blocked_since is None:
                self.blocked_since = t
            elif t - self.blocked_since >= float(c['block_timeout']):
                self.blocked_since = None
                self.commands.append('END_ROW')
                self.log.add(t, f"Row {p.get('row')} blocked by a static {ob.get('cls', 'obstacle')} with no free side "
                                f"for {c['block_timeout']:.0f} s: ending the row here, rest marked for revisit",
                             'replan', dedup=0.0)
                self.decision_override = ('ABORT CURRENT SUBTASK', t + 2.0)
        else:
            self.blocked_since = None
        # --- mission state
        fld = self.field()
        prev = self.state
        if self.driver_done and 'safety stop' in self.driver_msg.lower():
            self.state = 'FAULT'
        elif self.hold == 'repeated no-progress':
            self.state = 'FAULT'
        elif p.get('done') or self.gate == 'COMPLETE':
            if self.t_done is None:
                self.t_done = t
                self.log.add(t, f"Rows finished: verifying coverage ({fld['coverage_pct']}% of the crop rows)",
                             'mission', dedup=0.0)
            if self.moisture and not self.moisture.get('complete') and t - self.t_done < 10.0:
                self.state = 'MOISTURE_MAPPING'          # the map finalises when the rows are done
            elif t - self.t_done < float(c['verify_time']):
                self.state = 'COVERAGE_VERIFICATION'
            elif fld['uncovered'] or [r for r in fld['interrupted_rows'] if (r.get('shortfall') or 0) > c['min_gap']]:
                self.state = 'REVISIT_REQUIRED'
            else:
                self.state = 'MISSION_COMPLETE'
        elif self.gate == 'HALTED':
            self.state = 'FAULT'
        elif not self.running():
            self.state = 'READY'
        elif self.recovering_until is not None:
            self.state = 'ROW_RECOVERY' if p.get('phase') == 'row' else 'RECOVERY'
        elif not p or t - (self.t_start if self.t_start is not None else t) < float(c['field_analysis_time']):
            self.state = 'FIELD_ANALYSIS'
        elif avoid == 'RETURNING TO ROW':
            self.state = 'ROW_REJOIN'
        elif avoid in self.AVOID_STATES:
            self.state = 'OBSTACLE_AVOIDANCE'
        else:
            self.state = 'ROW_NAVIGATION'
        if self.state != prev:
            self.log.add(t, f'State {prev} -> {self.state}', 'state', dedup=0.0)
            if self.state == 'REVISIT_REQUIRED':
                gaps = ', '.join(f'row {g[0]} x {g[1]:.1f}..{g[2]:.1f}' for g in fld['uncovered'][:6])
                short = [r for r in fld['interrupted_rows'] if (r.get('shortfall') or 0) > c['min_gap']]
                self.log.add(t, 'Coverage verification: areas to revisit - ' + (gaps or '') +
                             ('; ' if gaps and short else '') +
                             ', '.join(f"row {r['row']} end ({r['shortfall']:.1f} m, {r.get('obstacle')})" for r in short),
                             'mission', dedup=0.0)
            elif self.state == 'MISSION_COMPLETE':
                self.log.add(t, 'Coverage verified: field complete', 'mission', dedup=0.0)
            if prev in ('OBSTACLE_AVOIDANCE', 'ROW_REJOIN') and self.state == 'ROW_NAVIGATION':
                self.log.add(t, f'Row {p.get("row")} rejoined: coverage continues', 'mission', dedup=0.0)
        if self.moisture.get('complete') and not self.moisture_complete_logged:
            self.moisture_complete_logged = True
            self.log.add(t, 'MOISTURE MAP COMPLETE', 'mission', dedup=0.0)
        # --- decision
        self.t_now = t
        self.decision = self._decide(avoid)
        return self.status(t, fld)

    def _decide(self, avoid):
        s = self.state
        if self.decision_override and self.t_now < self.decision_override[1]:
            return self.decision_override[0]
        if s == 'FAULT':
            return 'ABORT CURRENT SUBTASK'
        if s in ('MISSION_COMPLETE', 'REVISIT_REQUIRED'):
            return 'COMPLETE MISSION'
        if s in ('ROW_RECOVERY', 'RECOVERY'):
            return 'RECOVER'
        if self.hold is not None or s == 'READY':
            return 'WAIT TEMPORARILY'
        if avoid in ('YIELDING', 'WAITING', 'HEADLAND BLOCKED', 'BLOCKED ON DETOUR', 'ROW END BLOCKED'):
            return 'WAIT TEMPORARILY'
        if avoid == 'REVERSE U-TURN':
            return 'BACKTRACK'
        if avoid in ('AVOIDING', 'PASSING OBSTACLE'):
            return 'AVOID'
        if avoid == 'RETURNING TO ROW':
            return 'RESUME'
        if self.prev_avoid and not avoid:
            self.prev_avoid = None
            return 'RESUME'
        self.prev_avoid = avoid
        if self.speed_cap:
            return 'SLOW_DOWN'
        near = [o for o in self.obstacles if float(o.get('distance', 99)) < 5.0]
        if near:
            return 'SLOW_DOWN'
        return 'CONTINUE'

    def status(self, t, fld=None):
        fld = fld or self.field()
        perc, sens = self.perception_confidence(t)
        m = self.moisture
        mcov = m.get('coverage_pct')
        now = self.moisture_now or {}
        obs = sorted(self.obstacles, key=lambda o: float(o.get('distance', 99)))
        near = obs[0] if obs else None
        p = self.progress
        progress_q = ('STUCK' if self.recovering_until is not None else
                      'HOLD' if self.hold else
                      'GOOD' if self.prog.cmd_since is None or self.loc.v_pose > 0.1 else 'SLOW')
        moisture_label = ('MOISTURE MAP COMPLETE' if m.get('complete') else
                          (f'MOISTURE MAP: {mcov:.0f}% COMPLETE' if mcov is not None else 'MOISTURE MAP: no data'))
        mission_time = (t - self.t_start) if self.t_start is not None else 0.0
        return dict(
            domain='agriculture', mission='FIELD COVERAGE', state=self.state, decision=self.decision,
            autonomy_level=self.level, hold=self.hold, speed_cap=self.speed_cap or None,
            navigation=self._nav_label(), current_row=p.get('row'), total_rows=p.get('total_rows'),
            coverage_pct=p.get('coverage_pct'), obstacle=(None if not near else dict(
                cls=near.get('cls'), distance=near.get('distance'), dynamic=bool(near.get('dynamic')),
                speed=near.get('speed'), bearing=near.get('bearing'))),
            localization_confidence=self.loc.confidence(t), localization_factors=self.loc.factors(t),
            perception_confidence=perc, sensors=self.health.report(t), robot_progress=progress_q,
            recovery=('ACTIVE' if self.recovering_until is not None else
                      'EXHAUSTED' if self.state == 'FAULT' else 'READY'),
            manual_interventions=self.manual, field=fld,
            moisture=dict(label=moisture_label, mapping_pct=mcov, complete=bool(m.get('complete')),
                          current=now.get('moisture'), category=now.get('category'),
                          dry_pct=m.get('dry_pct'), wet_pct=m.get('wet_pct'), normal_pct=m.get('normal_pct'),
                          unmapped_pct=None if mcov is None else round(max(0.0, 100.0 - mcov), 1)),
            metrics=dict(
                mission_success=self.state == 'MISSION_COMPLETE', collisions=self.collisions,
                near_collisions=self.near, manual_interventions=self.manual,
                replans=int(p.get('detours', 0)) + int(p.get('reverse_turns', 0)),
                recoveries=self.recoveries_total, no_progress_events=self.prog.events,
                localization_confidence=self.loc.confidence(t),
                min_obstacle_clearance=None if self.min_clear is None else round(self.min_clear, 2),
                distance_m=round(self.distance, 1), mission_time_s=round(mission_time, 0),
                field_coverage_pct=p.get('coverage_pct'), rows_completed=len(fld['completed_rows']),
                rows_revisited=0, moisture_map_pct=mcov,
                unmapped_area_pct=None if mcov is None else round(max(0.0, 100.0 - mcov), 1)),
            events=list(self.log.items)[-60:])

    def _nav_label(self):
        p = self.progress
        ph = p.get('phase')
        av = (p.get('avoidance') or {}).get('state')
        if self.hold:
            return 'SAFETY HOLD'
        if av:
            return av
        return {'row': 'ROW NAVIGATION', 'headland_turn': 'HEADLAND U-TURN', 'reverse_turn': 'REVERSE U-TURN',
                'row_end': 'ROW END', 'recover': 'RECOVERY BACK-OFF', 'done': 'FIELD COMPLETE'}.get(ph, 'READY')
