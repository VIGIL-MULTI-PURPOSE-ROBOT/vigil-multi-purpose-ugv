#!/usr/bin/python3
"""Search-and-rescue mission logic (no ROS imports, unit-testable).

  SAR OFF --(robot reached B)--> SAR READY --(dashboard switch)--> SAR ACTIVE
     -> MOVING TO SEARCH POINT -> SEARCHING (scan headings)          x 10 points
     -> MOVING TO BUILDING -> SCANNING BUILDING (face the wall, sweep)  per nearby building face
     -> (any time) CONFIRMING HUMAN: turn towards an unconfirmed warm candidate
     -> FINAL SCAN -> SAR COMPLETE (robot holds its last safe position)

The mission never drives the wheels itself. It only
  * sends goals to the existing cliff-aware navigator (/navigation/goal), so every
    search leg uses the same A*, footprint safety, cliff avoidance, hill climbing and
    PATH BLOCKED -> REPLAN -> ALTERNATIVE ROUTE escalation as the A -> B mission;
  * asks the navigator to hold a heading while scanning (the navigator only rotates
    when the footprint's rotation circle is clear of cliffs / obstacles);
  * caps the speed (search speed, slower near buildings).
A search point or building view that the navigator cannot reach within
search_timeout is SKIPPED and the mission continues - it never stops on one blocked route.
"""
import math
from dataclasses import dataclass, field


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- geometry
@dataclass
class Building:
    bid: str
    cx: float
    cy: float
    w: float                    # extent along x
    d: float                    # extent along y

    def distance(self, x, y):
        dx = max(abs(x - self.cx) - self.w / 2, 0.0)
        dy = max(abs(y - self.cy) - self.d / 2, 0.0)
        return math.hypot(dx, dy)

    def inside(self, x, y, margin=0.0):
        return abs(x - self.cx) <= self.w / 2 + margin and abs(y - self.cy) <= self.d / 2 + margin

    def push_out(self, x, y, margin):
        """Nearest point on the margin-expanded rectangle boundary (if inside)."""
        if not self.inside(x, y, margin):
            return x, y
        hw, hd = self.w / 2 + margin, self.d / 2 + margin
        cands = [(self.cx - hw - 0.01, y), (self.cx + hw + 0.01, y), (x, self.cy - hd - 0.01), (x, self.cy + hd + 0.01)]
        return min(cands, key=lambda p: math.hypot(p[0] - x, p[1] - y))


def buildings_from_metadata(meta, include='auto'):
    out = []
    want = None if include in ('', 'auto', None) else {s.strip() for s in include.split(',') if s.strip()}
    for b in meta.get('buildings', []):
        if want is not None and b['id'] not in want:
            continue
        out.append(Building(b['id'], float(b['center'][0]), float(b['center'][1]), float(b['w']), float(b['d'])))
    return out


def parse_points(text):
    pts = []
    for chunk in (text or '').replace('\n', ';').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        x, y = (float(v) for v in chunk.split(','))
        pts.append((x, y))
    return pts


@dataclass
class Region:
    cx: float
    cy: float
    width: float
    height: float
    rotation: float = 0.0       # rad

    def to_world(self, u, v):
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return self.cx + c * u - s * v, self.cy + s * u + c * v


def region_from_cfg(sar, goal):
    mode = sar.get('region_mode', 'goal')
    if mode == 'box':
        return Region(float(sar.get('region_center_x', 0.0)), float(sar.get('region_center_y', 0.0)),
                      float(sar.get('region_width', 60.0)), float(sar.get('region_height', 60.0)),
                      math.radians(float(sar.get('region_rotation_deg', 0.0))))
    r = float(sar.get('search_radius', 30.0))
    return Region(float(goal[0]), float(goal[1]), 2 * r, 2 * r, math.radians(float(sar.get('region_rotation_deg', 0.0))))


def generate_search_points(region, n=10, rows=2, spacing=0.0, start=None):
    """Boustrophedon (lawn-mower) pattern: row 1 left->right, row 2 right->left, ...
    Deterministic; the pattern corner nearest `start` is P1 (reproducible, no wandering)."""
    rows = max(1, int(rows))
    cols = int(math.ceil(n / rows))
    if spacing and spacing > 0:
        xs = [(k - (cols - 1) / 2.0) * spacing for k in range(cols)]
    else:
        xs = [-region.width / 2 + (k + 0.5) * region.width / cols for k in range(cols)]
    ys = [-region.height / 2 + (j + 0.5) * region.height / rows for j in range(rows)]
    variants = []
    for flip_x in (False, True):
        for flip_y in (False, True):
            seq = []
            yy = ys[::-1] if flip_y else ys
            for j, y in enumerate(yy):
                row = xs[::-1] if (j % 2 == 1) != flip_x else xs
                seq += [(x, y) for x in row]
            variants.append([region.to_world(u, v) for u, v in seq[:n]])
    if start is None:
        return variants[0]
    return min(variants, key=lambda s: math.hypot(s[0][0] - start[0], s[0][1] - start[1]))


def snap_points(points, buildings, clearance, bounds=None, iterations=6):
    """Move points out of buildings (+clearance) and inside the map bounds.
    Returns [(x, y, moved)]"""
    out = []
    for x0, y0 in points:
        x, y = x0, y0
        for _ in range(iterations):
            moved = False
            for b in buildings:
                if b.inside(x, y, clearance):
                    x, y = b.push_out(x, y, clearance)
                    moved = True
            if bounds is not None:
                xmin, ymin, xmax, ymax = bounds
                nx, ny = min(max(x, xmin), xmax), min(max(y, ymin), ymax)
                moved |= (nx, ny) != (x, y)
                x, y = nx, ny
            if not moved:
                break
        out.append((round(x, 2), round(y, 2), math.hypot(x - x0, y - y0) > 0.01))
    return out


def building_views(b, others, standoff, standoff_min, robot_radius, bounds=None):
    """Observation poses in front of each wall, facing the wall (thermal camera looks
    straight at doors/windows). Falls back to standoff_min in narrow streets (the 7.5 m
    urban canyon); a face with no valid pose is skipped."""
    faces = (('S', 0.0, -1.0, math.pi / 2), ('N', 0.0, 1.0, -math.pi / 2),
             ('W', -1.0, 0.0, 0.0), ('E', 1.0, 0.0, math.pi))
    views = []
    for name, fx, fy, heading in faces:
        for s in (standoff, standoff_min):
            x = b.cx + fx * (b.w / 2 + s)
            y = b.cy + fy * (b.d / 2 + s)
            if any(o.inside(x, y, robot_radius + 0.5) for o in others if o.bid != b.bid):
                continue
            if bounds is not None and not (bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3]):
                continue
            views.append(dict(building=b.bid, face=name, x=round(x, 2), y=round(y, 2),
                              heading=heading, standoff=s))
            break
    return views


# ---------------------------------------------------------------- mission
@dataclass
class Task:
    kind: str                   # point | building | final
    label: str
    x: float
    y: float
    index: int = 0              # search point number (1-based) for kind == point
    heading: float = 0.0        # building: heading that faces the wall
    building: str = ''
    status: str = 'pending'     # pending | active | done | skipped
    phase: str = 'move'         # move | scan
    t_start: float = -1.0
    headings: list = field(default_factory=list)
    h_index: int = 0
    h_t0: float = -1.0
    h_reached: float = -1.0
    confirms: int = 0
    note: str = ''

    def as_dict(self):
        return dict(kind=self.kind, label=self.label, x=self.x, y=self.y, index=self.index,
                    building=self.building, status=self.status, phase=self.phase, note=self.note)


class SarMission:
    STATES = ('SAR OFF', 'SAR READY', 'SAR ACTIVE', 'MOVING TO SEARCH POINT', 'SEARCHING',
              'MOVING TO BUILDING', 'SCANNING BUILDING', 'CONFIRMING HUMAN', 'HUMAN DETECTED',
              'FINAL SCAN', 'SAR COMPLETE')

    def __init__(self, sar, buildings, thermal_hfov=1.0472, robot_radius=1.1, bounds=None):
        self.cfg = dict(sar)
        self.buildings = buildings
        self.hfov = thermal_hfov
        self.robot_radius = robot_radius
        self.bounds = bounds
        self.state = 'SAR OFF'
        self.active = False
        self.tasks = []
        self.current = None
        self.t_activate = None
        self.t_complete = None
        self.events = []                 # (t, text), consumed by the node
        self.log = []                    # full history (t, text)
        self.display_until = -1.0
        self.display_state = None
        self.goal_cmd = None             # (x, y) last goal sent to the navigator
        self.goal_seq = 0
        self.hold = None                 # heading (rad) or None
        self.speed_limit = 0.0
        self.known_humans = set()
        self.confirm = None              # dict(target, t0, heading, resume_phase)
        self.confirm_spots = []          # (x, y, t) candidate spots already looked at
        self.goal_reached = False
        self.search_points = []
        self.reason = ''
        self.nav_goal = None

    # ------------------------------------------------------------- helpers
    def c(self, k, d):
        return self.cfg.get(k, d)

    def _event(self, t, text):
        self.events.append((t, text))
        self.log.append((t, text))

    def _flash(self, t, state, hold=4.0):
        self.display_state, self.display_until = state, t + hold

    def shown_state(self, t):
        return self.display_state if t < self.display_until else self.state

    # ------------------------------------------------------------- planning
    def plan(self, goal, start):
        sar = self.cfg
        if sar.get('region_mode', 'goal') == 'manual':
            raw = parse_points(sar.get('manual_points', ''))
        else:
            region = region_from_cfg(sar, goal)
            raw = generate_search_points(region, int(sar.get('search_points', 10)), int(sar.get('search_rows', 2)),
                                         float(sar.get('search_spacing', 0.0)), start=start)
        clearance = float(sar.get('point_clearance', 2.5)) + self.robot_radius
        pts = snap_points(raw, self.buildings, clearance, self.bounds)
        self.search_points = [dict(index=k + 1, x=x, y=y, moved=m, status='pending') for k, (x, y, m) in enumerate(pts)]
        tasks = []
        assigned = set()
        bdist = float(sar.get('building_search_distance', 25.0))
        for sp in self.search_points:
            tasks.append(Task('point', f"SEARCH POINT {sp['index']}", sp['x'], sp['y'], index=sp['index']))
            near = sorted((b for b in self.buildings if b.bid not in assigned and b.distance(sp['x'], sp['y']) <= bdist),
                          key=lambda b: b.distance(sp['x'], sp['y']))
            for b in near:
                assigned.add(b.bid)
                views = building_views(b, self.buildings, float(sar.get('building_standoff', 7.0)),
                                       float(sar.get('building_standoff_min', 3.5)), self.robot_radius, self.bounds)
                # visit faces starting with the one nearest the search point, then around
                views.sort(key=lambda v: math.hypot(v['x'] - sp['x'], v['y'] - sp['y']))
                if views:
                    first = views[0]
                    rest = sorted(views[1:], key=lambda v: wrap(math.atan2(v['y'] - b.cy, v['x'] - b.cx)
                                                                - math.atan2(first['y'] - b.cy, first['x'] - b.cx)) % (2 * math.pi))
                    views = [first] + rest
                for v in views:
                    tasks.append(Task('building', f"{b.bid} {v['face']} FACE", v['x'], v['y'],
                                      heading=v['heading'], building=b.bid, index=sp['index']))
        if sar.get('final_scan', True) and self.search_points:
            last = self.search_points[-1]
            tasks.append(Task('final', 'FINAL SCAN', last['x'], last['y']))
        self.tasks = tasks
        return tasks

    # ------------------------------------------------------------- commands
    def activate(self, t, pose, nav_goal, goal_reached):
        """Operator START. Accepted in EVERY navigation state - at A, driving A->B, near B, at B,
        stopped, replanning. SAR is a manual mission mode, not a consequence of reaching B, so
        nothing about B or the navigator can refuse it (the old require_goal_reached gate is gone).

        restart_behavior decides what a second START does after a STOP:
          reset  - plan the search points and building views afresh from where the robot is now
          resume - carry on with the tasks that were still pending when SAR was stopped
        Confirmed humans (H1, H2, ...) belong to the tracker and are never cleared either way."""
        if self.active:
            return False, 'SAR already active'
        pending = [tk for tk in self.tasks if tk.status in ('pending', 'active')]
        resume = (self.c('restart_behavior', 'reset') == 'resume' and pending
                  and self.state != 'SAR COMPLETE')
        if resume:
            for tk in self.tasks:
                if tk.status == 'active':
                    tk.status = 'pending'
        else:
            self.plan(nav_goal, (pose.x, pose.y))
        self.b_reached_at_start = bool(goal_reached)
        self.active = True
        self.t_activate = t
        self.t_complete = None
        self.state = 'SAR ACTIVE'
        self.current = None
        self.confirm = None
        n_b = len({tk.building for tk in self.tasks if tk.kind == 'building'})
        how = 'RESUMED' if resume else 'ACTIVATED'
        self._event(t, f'SAR {how}: {len(self.search_points)} search points, {n_b} buildings to scan')
        self._flash(t, 'SAR ACTIVE', 2.0)
        return True, 'SAR ACTIVATED'

    def abort(self, t, pose, b_goal=None):
        """Operator STOP - always accepted while SAR runs.

        stop_behavior:
          resume_b - SAR PAUSED the A->B mission, so give it back: if B had not been reached when
                     SAR started, drive on to B; if it had, stop where the rover is.
          hold     - stop where the rover is.
        The operator's B is never overwritten: it used to be replaced by the robot's position."""
        if not self.active:
            return
        self.active = False
        self.state = 'SAR OFF'
        self.hold = None
        if (self.c('stop_behavior', 'resume_b') == 'resume_b' and b_goal is not None
                and not getattr(self, 'b_reached_at_start', True)):
            self.goal_cmd = (float(b_goal[0]), float(b_goal[1]))
            self._event(t, f'SAR STOPPED by operator - resuming A->B to ({b_goal[0]:.1f}, {b_goal[1]:.1f})')
        else:
            self.goal_cmd = (pose.x, pose.y)       # stop where we are
            self._event(t, 'SAR STOPPED by operator - holding position')
        self.goal_seq += 1

    # ------------------------------------------------------------- tick
    def tick(self, t, pose, goal_reached, humans=(), tentative=()):
        """Returns nothing; read self.goal_cmd / goal_seq, self.hold, self.speed_limit."""
        self.goal_reached = goal_reached
        # new confirmed humans -> HUMAN DETECTED
        for h in humans:
            if h['human_id'] not in self.known_humans:
                self.known_humans.add(h['human_id'])
                if self.active:
                    self._flash(t, 'HUMAN DETECTED', 5.0)
                    if self.confirm is not None:
                        self._end_confirm(t)
        if not self.active:
            if self.state != 'SAR COMPLETE':
                self.state = 'SAR OFF'      # never "READY": SAR is available at all times
            self.hold = None if self.state != 'SAR COMPLETE' else self.hold
            return
        if t - self.t_activate > float(self.c('max_search_time', 2400.0)):
            self._complete(t, pose, 'max_search_time reached')
            return
        # humans first: turn towards an unconfirmed warm candidate before driving on
        if self._maybe_confirm(t, pose, tentative, humans):
            return
        if self.current is None or self.current.status in ('done', 'skipped'):
            self.current = next((tk for tk in self.tasks if tk.status == 'pending'), None)
            if self.current is None:
                self._complete(t, pose, 'all search points and building scans done')
                return
            self._start_task(t, pose, self.current)
        tk = self.current
        if tk.phase == 'move':
            self._move(t, pose, tk)
        else:
            self._scan(t, pose, tk)

    # ------------------------------------------------------------- task handling
    def _start_task(self, t, pose, tk):
        tk.status, tk.phase, tk.t_start = 'active', 'move', t
        self.hold = None
        near = math.hypot(tk.x - pose.x, tk.y - pose.y) < float(self.c('search_point_tolerance', 1.5))
        if tk.kind == 'final' and near:
            tk.phase = 'scan'
            self._prepare_scan(t, pose, tk)
            return
        self._send_goal(tk.x, tk.y)
        self.state = 'MOVING TO BUILDING' if tk.kind == 'building' else 'MOVING TO SEARCH POINT'
        self._event(t, f'{self.state}: {tk.label} ({tk.x:.1f}, {tk.y:.1f})')

    def _send_goal(self, x, y):
        self.goal_cmd = (x, y)
        self.goal_seq += 1

    def _near_building(self, pose):
        d = min((b.distance(pose.x, pose.y) for b in self.buildings), default=1e9)
        return d <= float(self.c('building_standoff', 7.0)) + 3.0

    def _move(self, t, pose, tk):
        slow = self._near_building(pose) or tk.kind == 'building'
        self.speed_limit = float(self.c('building_scan_speed', 0.3)) if slow and self._near_building(pose) \
            else float(self.c('search_speed', 0.45))
        d = math.hypot(tk.x - pose.x, tk.y - pose.y)
        tol = float(self.c('search_point_tolerance', 1.5))
        if tk.kind == 'building':
            tol = max(tol, 1.0)
        if d <= tol:
            tk.phase = 'scan'
            self._prepare_scan(t, pose, tk)
            return
        if t - tk.t_start > float(self.c('search_timeout', 300.0)):
            self._finish(t, tk, 'skipped', f'{tk.label} SKIPPED: not reachable in {self.c("search_timeout", 300.0):.0f} s '
                                           f'(route blocked) - continuing')

    def _prepare_scan(self, t, pose, tk):
        self.speed_limit = 0.0
        if tk.kind == 'building':
            sw = math.radians(float(self.c('building_sweep_deg', 25.0)))
            tk.headings = [tk.heading - sw, tk.heading, tk.heading + sw]
            self.state = 'SCANNING BUILDING'
        else:
            n = max(1, int(self.c('scan_angles', 8)))
            hs = [2 * math.pi * k / n for k in range(n)]
            if tk.kind == 'point' and self.c('skip_seen_sector', True):
                # the thermal camera already looked along the arrival direction while driving in
                hs = [h for h in hs if abs(wrap(h - pose.yaw)) > self.hfov / 2] or hs
            # scan in rotation order starting next to the current heading (no needless full turns)
            hs.sort(key=lambda h: wrap(h - pose.yaw) % (2 * math.pi))
            tk.headings = hs
            self.state = 'FINAL SCAN' if tk.kind == 'final' else 'SEARCHING'
        tk.h_index, tk.h_t0, tk.h_reached = 0, t, -1.0
        self._event(t, f'{self.state}: {tk.label}, {len(tk.headings)} headings')

    def _scan(self, t, pose, tk):
        if tk.h_index >= len(tk.headings):
            what = 'BUILDING SCAN COMPLETE' if tk.kind == 'building' else \
                ('FINAL SCAN COMPLETE' if tk.kind == 'final' else f'SEARCH POINT {tk.index} COMPLETE')
            self._finish(t, tk, 'done', f'{what}' + (f': {tk.label}' if tk.kind == 'building' else ''))
            return
        h = tk.headings[tk.h_index]
        self.hold = h
        dwell = float(self.c('building_scan_dwell', 3.0)) / 1.0 if tk.kind == 'building' else float(self.c('scan_dwell', 1.5))
        err = abs(wrap(h - pose.yaw))
        if tk.h_reached < 0:
            if err < 0.12:
                tk.h_reached = t
            elif t - tk.h_t0 > 12.0:          # rotation blocked (cliff / obstacle around us): skip heading
                tk.note = 'some headings blocked'
                tk.h_index += 1
                tk.h_t0, tk.h_reached = t, -1.0
            return
        if t - tk.h_reached >= dwell:
            tk.h_index += 1
            tk.h_t0, tk.h_reached = t, -1.0

    def _finish(self, t, tk, status, text):
        tk.status = status
        self.hold = None
        if tk.kind == 'point':
            for sp in self.search_points:
                if sp['index'] == tk.index:
                    sp['status'] = status
        self._event(t, text)

    def _complete(self, t, pose, why):
        self.active = False
        self.state = 'SAR COMPLETE'
        self.t_complete = t
        self.hold = pose.yaw                   # stay stationary at the final safe position
        self.speed_limit = 0.0
        self._send_goal(pose.x, pose.y)
        done = sum(1 for sp in self.search_points if sp['status'] == 'done')
        self._event(t, f'SAR COMPLETE ({why}): humans found {len(self.known_humans)}, '
                       f'search points {done}/{len(self.search_points)}')

    # ------------------------------------------------------------- humans first
    def _maybe_confirm(self, t, pose, tentative, humans):
        """Humans first: turn towards an unconfirmed warm candidate and look at it for
        confirm_dwell s (the tracker needs N consecutive frames). Every spot is tried at most
        once per confirm_cooldown and at most max_confirms_per_task times per search leg, so a
        warm object that never confirms can not stall the search."""
        dwell = float(self.c('confirm_dwell', 4.0))
        if self.confirm is not None:
            c = self.confirm
            self.hold = c['heading']
            self.state = 'CONFIRMING HUMAN'
            if c['t_on'] is None:
                if abs(wrap(c['heading'] - pose.yaw)) < 0.12:
                    c['t_on'] = t
                elif t - c['t0'] > 12.0:          # could not turn (rotation blocked)
                    self._end_confirm(t)
                    return False
            elif t - c['t_on'] > dwell:
                self._end_confirm(t)
                return False
            return True
        if dwell <= 0:
            return False
        tk = self.current
        if tk is not None and tk.confirms >= int(self.c('max_confirms_per_task', 3)):
            return False
        rng = float(self.c('detection_range', 30.0))
        rev = float(self.c('revisit_distance', 2.0))
        cool = float(self.c('confirm_cooldown', 120.0))
        for cand in tentative:
            if any(math.hypot(cand['x'] - x, cand['y'] - y) < 3.0 and t - tt < cool for x, y, tt in self.confirm_spots):
                continue
            d = math.hypot(cand['x'] - pose.x, cand['y'] - pose.y)
            if d > rng or any(math.hypot(h['x'] - cand['x'], h['y'] - cand['y']) < rev for h in humans):
                continue
            self.confirm_spots.append((cand['x'], cand['y'], t))
            if tk is not None:
                tk.confirms += 1
            heading = math.atan2(cand['y'] - pose.y, cand['x'] - pose.x)
            self.confirm = dict(track=cand.get('track'), t0=t, t_on=None, heading=heading)
            self.state = 'CONFIRMING HUMAN'
            self.hold = heading
            self._event(t, f'HEAT SIGNATURE at ({cand["x"]:.1f}, {cand["y"]:.1f}), {d:.1f} m - turning to confirm')
            return True
        return False

    def _end_confirm(self, t):
        self.confirm = None
        self.hold = None
        tk = self.current
        if tk is not None and tk.status == 'active':
            if tk.phase == 'move':
                self._send_goal(tk.x, tk.y)       # resume the leg
                self.state = 'MOVING TO BUILDING' if tk.kind == 'building' else 'MOVING TO SEARCH POINT'
            else:
                tk.h_t0, tk.h_reached = t, -1.0   # redo the current heading
                self.state = 'SCANNING BUILDING' if tk.kind == 'building' else 'SEARCHING'

    # ------------------------------------------------------------- status
    def status(self, t, pose=None):
        done = sum(1 for sp in self.search_points if sp['status'] in ('done', 'skipped'))
        cur_pt = None
        if self.current is not None:
            cur_pt = self.current.index
        elapsed = 0.0
        if self.t_activate is not None:
            elapsed = (self.t_complete if self.t_complete is not None else t) - self.t_activate
        return dict(state=self.shown_state(t), raw_state=self.state, active=self.active,
                    search_point=cur_pt, search_points_done=done, search_points_total=len(self.search_points),
                    search_points=self.search_points, humans_found=len(self.known_humans),
                    search_time=round(elapsed, 1),
                    current_task=self.current.as_dict() if self.current is not None else None,
                    tasks=[tk.as_dict() for tk in self.tasks],
                    building_views_done=sum(1 for tk in self.tasks if tk.kind == 'building' and tk.status == 'done'),
                    building_views_total=sum(1 for tk in self.tasks if tk.kind == 'building'),
                    hold=self.hold, speed_limit=self.speed_limit, goal=self.goal_cmd,
                    goal_reached_b=self.goal_reached, reason=self.reason)
