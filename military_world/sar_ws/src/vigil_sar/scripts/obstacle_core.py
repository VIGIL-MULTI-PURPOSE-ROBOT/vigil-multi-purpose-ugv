#!/usr/bin/python3
"""Dynamic-obstacle perception, tracking, prediction and collision assessment (no ROS).

Used by obstacle_tracker.py (the ROS node) and cliff_navigator.py; tested offline by
test/test_dynamic_obstacles.py. All settings come from config/sar_mission.yaml -> obstacles.

Pipeline, every depth frame (existing sensors only: depth + segmentation + ground-truth pose):
  1. extract_points  depth pixels -> world points; the segmentation label says what each one is
                     (the world builder puts HUMAN 10, VEHICLE 20, BUILDING 30, ROCK 40, DEBRIS 50,
                     VEGETATION 60, OBJECT 70 on the models; ground is 1). Without segmentation the
                     points higher than obstacles.min_height above the rover's ground plane are kept.
  2. cluster         0.25 m grid, connected components PER CLASS (a person next to a wall is two
                     objects) -> centroid, radius, height, class.
  3. ObstacleTracker nearest-neighbour association + alpha-beta filter over the frames ->
                     position, velocity, speed, direction; a track is DYNAMIC when its filtered speed
                     stays above dynamic_speed (with hysteresis). Buildings, rocks, debris and
                     vegetation cannot move: their velocity is held at zero (the visible part of a wall
                     changes as the rover drives, which would otherwise look like motion).
  4. predict         constant-velocity positions over collision_prediction_time.
  5. assess          the rover's own future positions along its path vs every predicted obstacle
                     -> CLEAR / COLLISION RISK (+ time to collision, the obstacle, the conflict point).
  6. stamp           what the planner sees: every tracked obstacle as an OBSTACLE disc of
                     radius + its class safety distance, dynamic ones swept along their predicted
                     positions. The planner already inflates OBSTACLE cells by the rover's
                     half-width, so the rover's footprint is respected; replanning, the clearance
                     speed profile (smooth slow-down) and the never-give-up escalation then work on
                     moving obstacles exactly as on walls.
"""
import math
from dataclasses import dataclass

import numpy as np

try:
    import cv2
except ImportError:          # pragma: no cover - cv2 is a package dependency (planner_core)
    cv2 = None

LABELS = {10: 'HUMAN', 20: 'VEHICLE', 30: 'BUILDING', 40: 'ROCK', 50: 'DEBRIS', 60: 'VEGETATION', 70: 'OBJECT'}
GROUND_LABEL = 1
FIXED = ('BUILDING', 'ROCK', 'DEBRIS', 'VEGETATION')     # cannot move: velocity held at 0
MOVABLE = ('HUMAN', 'VEHICLE', 'OBJECT', 'UNKNOWN')


@dataclass
class ObstacleParams:
    # --- the six distances / times of the specification (metres, seconds) ---
    dynamic_obstacle_distance: float = 1.5   # m kept from any MOVING obstacle's edge
    human_safety_distance: float = 2.0       # m kept from a person (moving or not): humans first
    vehicle_safety_distance: float = 1.5     # m kept from a vehicle
    static_obstacle_distance: float = 0.3    # m kept from a static object the overlay handles
    collision_prediction_time: float = 4.0   # s of predicted motion (obstacles AND rover)
    robot_footprint_margin: float = 0.15     # m added around the rover's footprint in the collision test
    # --- perception / tracking ---
    enabled: bool = True
    tracking_range: float = 14.0             # m (depth camera far limit is 15 m)
    min_height: float = 0.25                 # m above the rover's ground plane (no segmentation)
    max_height: float = 2.6                  # m; higher points (canopies, balconies) never block
    cell: float = 0.25                       # m clustering grid
    min_points: int = 5
    pixel_stride: int = 2
    max_cluster_size: float = 6.0            # m; a bigger blob is a building face, split by the grid
    dynamic_speed: float = 0.30              # m/s filtered speed to call a track DYNAMIC
    static_speed: float = 0.15               # m/s ... and below this it is static again
    max_obstacle_speed: float = 4.0          # m/s clamp (a person sprinting is ~3 m/s)
    velocity_max_extent: float = 3.5         # m; bigger objects are never given a velocity
    alpha: float = 0.45                      # alpha-beta position gain
    beta: float = 0.12                       # alpha-beta velocity gain (5 cm noise -> ~0.07 m/s)
    speed_smoothing: float = 0.35            # EMA on the speed used for the dynamic decision
    gate: float = 1.2                        # m association gate (+ max speed x dt)
    confirm_frames: int = 3                  # hits before a track is reported / planned around
    track_timeout: float = 1.5               # s a track survives unseen (predicted meanwhile)
    prediction_step: float = 0.25            # s between predicted positions
    min_robot_speed: float = 0.5             # m/s assumed for the rover in the collision test
    yield_timeout: float = 6.0               # s: after this long yielding to a moving obstacle the
                                             #    prediction sweep is dropped (plan around where it IS)
    stamp_period: float = 0.3                # s between planner overlays (costmap rebuilds)
    goal_keepout: float = 0.6                # m: a stamp never covers the goal closer than this

    @classmethod
    def from_dict(cls, d):
        p = cls()
        for k, v in (d or {}).items():
            if hasattr(p, k) and not isinstance(v, (dict, list)):
                setattr(p, k, type(getattr(p, k))(v))
        return p

    def safety(self, cls, dynamic):
        """Clearance kept from the obstacle's EDGE for this class (humans get the most)."""
        if cls == 'HUMAN':
            return max(self.human_safety_distance, self.dynamic_obstacle_distance if dynamic else 0.0)
        if cls == 'VEHICLE':
            return max(self.vehicle_safety_distance, self.dynamic_obstacle_distance if dynamic else 0.0)
        return self.dynamic_obstacle_distance if dynamic else self.static_obstacle_distance


# ------------------------------------------------------------------ 1. points
def extract_points(pts, valid, seg, pose, p, ground_z=None):
    """World points of obstacles in one depth frame.

    pts (H,W,3) world points (terrain_core.CameraModel.backproject), valid (H,W) bool,
    seg (H,W) label image or None. Returns (xyz (N,3), labels (N,) int)."""
    st = max(1, int(p.pixel_stride))
    P = pts[::st, ::st].reshape(-1, 3)
    V = valid[::st, ::st].reshape(-1)
    L = None
    if seg is not None and seg.shape[:2] == valid.shape[:2]:
        L = seg[::st, ::st].reshape(-1).astype(np.int32)
    P = P[V]
    L = L[V] if L is not None else None
    d = np.hypot(P[:, 0] - pose.x, P[:, 1] - pose.y)
    keep = d <= p.tracking_range
    # height above the rover's ground plane (the plane through base_footprint, tilted with the rover)
    n = pose.R[:, 2]
    gz = pose.z if ground_z is None else ground_z
    h = (P[:, 0] - pose.x) * n[0] + (P[:, 1] - pose.y) * n[1] + (P[:, 2] - gz) * n[2]
    keep &= h <= p.max_height
    if L is not None:
        keep &= L >= 10                      # labelled obstacles only; 0 = sky, 1 = ground
    else:
        keep &= h >= p.min_height
    P = P[keep]
    labels = L[keep] if L is not None else np.zeros(len(P), np.int32)
    return P, labels


# ------------------------------------------------------------------ 2. clusters
def cluster(xyz, labels, p):
    """Connected components on a p.cell grid, separately for each class. Returns a list of dicts
    x, y, radius, height, cls, n."""
    out = []
    if len(xyz) == 0:
        return out
    for lab in np.unique(labels):
        m = labels == lab
        P = xyz[m]
        if len(P) < p.min_points:
            continue
        i = np.floor(P[:, 1] / p.cell).astype(np.int64)
        j = np.floor(P[:, 0] / p.cell).astype(np.int64)
        i0, j0 = i.min(), j.min()
        H, W = int(i.max() - i0 + 3), int(j.max() - j0 + 3)
        if H * W > 4_000_000:                # absurd spread (should not happen inside 14 m)
            continue
        grid = np.zeros((H, W), np.uint8)
        grid[i - i0 + 1, j - j0 + 1] = 1
        if cv2 is not None:
            n, comp = cv2.connectedComponents(grid, connectivity=8)
        else:                                # pragma: no cover
            n, comp = _components(grid)
        cid = comp[i - i0 + 1, j - j0 + 1]
        name = LABELS.get(int(lab), 'UNKNOWN')
        for c in range(1, n):
            q = P[cid == c]
            if len(q) < p.min_points:
                continue
            cx, cy = float(q[:, 0].mean()), float(q[:, 1].mean())
            r = float(np.max(np.hypot(q[:, 0] - cx, q[:, 1] - cy))) + p.cell / 2
            out.append(dict(x=cx, y=cy, radius=max(0.2, r), height=float(q[:, 2].max() - q[:, 2].min()),
                            top=float(q[:, 2].max()), cls=name, n=int(len(q))))
    return out


def _components(grid):                        # pragma: no cover - fallback without OpenCV
    comp = np.zeros(grid.shape, np.int32)
    n = 1
    for a, b in zip(*np.nonzero(grid)):
        if comp[a, b]:
            continue
        stack = [(a, b)]
        comp[a, b] = n
        while stack:
            u, v = stack.pop()
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    x, y = u + du, v + dv
                    if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1] and grid[x, y] and not comp[x, y]:
                        comp[x, y] = n
                        stack.append((x, y))
        n += 1
    return n, comp


# ------------------------------------------------------------------ 3. tracking
class Track:
    __slots__ = ('id', 'cls', 'x', 'y', 'vx', 'vy', 'radius', 'height', 'hits', 'misses', 'first_t', 'last_t',
                 'speed_f', 'dynamic', 'votes')

    def __init__(self, tid, det, t):
        self.id = tid
        self.cls = det['cls']
        self.x, self.y = det['x'], det['y']
        self.vx = self.vy = 0.0
        self.radius = det['radius']
        self.height = det.get('height', 0.0)
        self.hits, self.misses = 1, 0
        self.first_t = self.last_t = t
        self.speed_f = 0.0
        self.dynamic = False
        self.votes = {det['cls']: 1}

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def can_move(self, p):
        return self.cls not in FIXED and 2.0 * self.radius <= p.velocity_max_extent


class ObstacleTracker:
    def __init__(self, p):
        self.p = p
        self.tracks = []
        self.next_id = 1
        self.t = None

    def update(self, t, detections):
        """One frame of cluster detections at time t (seconds). Returns the confirmed tracks."""
        p = self.p
        dt = 0.0 if self.t is None else max(0.0, t - self.t)
        self.t = t
        for tr in self.tracks:                                    # predict
            tr.x += tr.vx * dt
            tr.y += tr.vy * dt
        used = set()
        pairs = []
        for k, tr in enumerate(self.tracks):
            for m, d in enumerate(detections):
                if not _compatible(tr.cls, d['cls']):
                    continue
                dist = math.hypot(d['x'] - tr.x, d['y'] - tr.y)
                gate = p.gate + p.max_obstacle_speed * dt + 0.5 * (tr.radius + d['radius'])
                if dist <= gate:
                    pairs.append((dist, k, m))
        pairs.sort()
        matched_tracks = set()
        for dist, k, m in pairs:
            if k in matched_tracks or m in used:
                continue
            matched_tracks.add(k)
            used.add(m)
            self._correct(self.tracks[k], detections[m], t, dt)
        for k, tr in enumerate(self.tracks):
            if k not in matched_tracks:
                tr.misses += 1
        self.tracks = [tr for tr in self.tracks if t - tr.last_t <= p.track_timeout]
        for m, d in enumerate(detections):
            if m not in used:
                self.tracks.append(Track(self.next_id, d, t))
                self.next_id += 1
        return self.confirmed()

    def _correct(self, tr, d, t, dt):
        p = self.p
        rx, ry = d['x'] - tr.x, d['y'] - tr.y
        tr.x += p.alpha * rx
        tr.y += p.alpha * ry
        if dt > 1e-3 and tr.hits >= 1:
            tr.vx += p.beta * rx / dt
            tr.vy += p.beta * ry / dt
        tr.votes[d['cls']] = tr.votes.get(d['cls'], 0) + 1
        tr.cls = max(tr.votes, key=tr.votes.get)
        tr.radius = 0.7 * tr.radius + 0.3 * d['radius']
        tr.height = max(tr.height, d.get('height', 0.0))
        if not tr.can_move(p):
            tr.vx = tr.vy = 0.0
        s = tr.speed
        if s > p.max_obstacle_speed:
            tr.vx *= p.max_obstacle_speed / s
            tr.vy *= p.max_obstacle_speed / s
        tr.speed_f += p.speed_smoothing * (tr.speed - tr.speed_f)
        if tr.speed_f >= p.dynamic_speed and tr.hits >= p.confirm_frames:
            tr.dynamic = True
        elif tr.speed_f <= p.static_speed:
            tr.dynamic = False
        tr.hits += 1
        tr.misses = 0
        tr.last_t = t

    def confirmed(self):
        return [tr for tr in self.tracks if tr.hits >= self.p.confirm_frames]


def _compatible(a, b):
    return a == b or 'UNKNOWN' in (a, b) or {a, b} <= {'OBJECT', 'DEBRIS'}


class TrackView:
    """A track as received over /perception/obstacles (dict), advanced to the reader's time."""
    __slots__ = ('id', 'cls', 'x', 'y', 'vx', 'vy', 'radius', 'dynamic', 'height')

    def __init__(self, d, dt=0.0):
        self.id, self.cls = d['id'], d['cls']
        self.vx, self.vy = float(d.get('vx', 0.0)), float(d.get('vy', 0.0))
        self.dynamic = bool(d.get('dynamic', False))
        dt = max(0.0, min(1.0, dt)) if self.dynamic else 0.0
        self.x, self.y = float(d['x']) + self.vx * dt, float(d['y']) + self.vy * dt
        self.radius = float(d.get('radius', 0.3))
        self.height = float(d.get('height', 0.0))

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


def tracks_from_msg(msg, now):
    """/perception/obstacles JSON -> [TrackView] predicted to `now` (message latency compensated)."""
    dt = now - float(msg.get('stamp', now))
    return [TrackView(d, dt) for d in msg.get('tracks', [])]


# ------------------------------------------------------------------ 4. prediction
def predict(tr, p, horizon=None):
    """[(t, x, y)] from now to the prediction horizon (constant velocity; static = one point)."""
    horizon = p.collision_prediction_time if horizon is None else horizon
    if not tr.dynamic:
        return [(0.0, tr.x, tr.y)]
    n = max(1, int(math.ceil(horizon / p.prediction_step)))
    return [(k * horizon / n, tr.x + tr.vx * k * horizon / n, tr.y + tr.vy * k * horizon / n) for k in range(n + 1)]


def describe(tr, pose, p, now=None):
    """Track -> JSON-able dict for the dashboard / navigator."""
    dx, dy = tr.x - pose.x, tr.y - pose.y
    dist = math.hypot(dx, dy)
    bearing = math.degrees(_wrap(math.atan2(dy, dx) - pose.yaw))
    side = 'AHEAD' if abs(bearing) <= 30 else ('LEFT' if 30 < bearing <= 150 else
                                               ('RIGHT' if -150 <= bearing < -30 else 'BEHIND'))
    heading = math.degrees(math.atan2(tr.vy, tr.vx)) if tr.speed > 0.05 else None
    # approaching = the obstacle's velocity points towards the rover
    closing = -(dx * tr.vx + dy * tr.vy) / max(dist, 1e-6)
    return dict(id=tr.id, cls=tr.cls, x=round(tr.x, 2), y=round(tr.y, 2), vx=round(tr.vx, 2), vy=round(tr.vy, 2),
                speed=round(tr.speed, 2), heading_deg=None if heading is None else round(heading, 0),
                radius=round(tr.radius, 2), dynamic=bool(tr.dynamic), distance=round(max(0.0, dist - tr.radius), 2),
                centre_distance=round(dist, 2), bearing_deg=round(bearing, 0), direction=side,
                closing_speed=round(closing, 2), safety=round(p.safety(tr.cls, tr.dynamic), 2),
                age=round((now - tr.first_t) if now is not None else 0.0, 1), hits=tr.hits)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------ 5. collision assessment
def robot_future(pose, path, progress, speed, p, horizon=None):
    """[(t, x, y)] of the rover along its path at max(speed, min_robot_speed)."""
    horizon = p.collision_prediction_time if horizon is None else horizon
    v = max(float(speed), p.min_robot_speed)
    n = max(1, int(math.ceil(horizon / p.prediction_step)))
    if path is None or len(path) < 2:
        return [(k * horizon / n, pose.x, pose.y) for k in range(n + 1)]
    pts = [(pose.x, pose.y)] + [tuple(q) for q in np.asarray(path)[min(progress + 1, len(path) - 1):]]
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    out = []
    for k in range(n + 1):
        t = k * horizon / n
        s = v * t
        x, y = pts[-1]
        for (a, b), L in zip(zip(pts, pts[1:]), seg):
            if s <= L:
                f = s / L if L > 1e-9 else 0.0
                x, y = a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
                break
            s -= L
        out.append((t, x, y))
    return out


def assess(pose, path, progress, speed, tracks, p, robot_radius):
    """CLEAR / COLLISION RISK for the rover's planned motion against every predicted obstacle.

    A conflict = at the same future time the rover's centre is closer to the obstacle's centre than
    robot_radius (half-width) + obstacle radius + the class safety distance. (The planner overlay adds
    robot_footprint_margin on top, so a path planned around the overlay is never itself a conflict.) Also a static
    obstacle already inside its safety distance counts (humans first).
    Returns dict(status, ttc, id, cls, x, y, min_clearance)."""
    fut = robot_future(pose, path, progress, speed, p)
    best = dict(status='CLEAR', ttc=None, id=None, cls=None, x=None, y=None, min_clearance=None)
    min_clear = math.inf
    for tr in tracks:
        need = robot_radius + tr.radius + p.safety(tr.cls, tr.dynamic) - 0.05
        now_clear = math.hypot(tr.x - pose.x, tr.y - pose.y) - (robot_radius + tr.radius)
        min_clear = min(min_clear, now_clear)
        for (t, rx, ry) in fut:
            ox, oy = tr.x + tr.vx * t, tr.y + tr.vy * t
            if not tr.dynamic:
                ox, oy = tr.x, tr.y
            if math.hypot(rx - ox, ry - oy) < need:
                # only a risk if the obstacle is moving, or is a person / vehicle (priority classes);
                # a static wall on the path is the planner's business, not a collision prediction
                if tr.dynamic or tr.cls in ('HUMAN', 'VEHICLE'):
                    if best['ttc'] is None or t < best['ttc']:
                        best.update(status='COLLISION RISK', ttc=round(t, 2), id=tr.id, cls=tr.cls,
                                    x=round(ox, 2), y=round(oy, 2))
                break
    best['min_clearance'] = None if min_clear == math.inf else round(min_clear, 2)
    return best


def yield_speed(ttc, cruise, decel=1.5, reaction=0.4):
    """Speed that still stops before a conflict ttc seconds ahead at the current speed (smooth)."""
    if ttc is None:
        return cruise
    d = max(0.0, (ttc - reaction)) * cruise
    return max(0.0, min(cruise, math.sqrt(2.0 * decel * d) * 0.8))


# ------------------------------------------------------------------ 6. planner overlay
def plannable(tracks):
    """Tracks the planner overlay handles: people and vehicles (always - their safety distance is
    bigger than the planner's own margin) and anything moving. Static walls, rocks, debris and props
    are already OBSTACLE cells from terrain_mapper; stamping them again with a margin would close
    gaps the 1.12 m rover fits through (narrow-path handling stays the planner's footprint test)."""
    return [tr for tr in tracks if tr.dynamic or tr.cls in ('HUMAN', 'VEHICLE')]


def stamp(classes, origin_x, origin_y, res, tracks, p, obstacle_class, robot=None, goal=None,
          sweep=True, robot_clear=0.9):
    """Mark every obstacle (+ its safety distance, + its predicted sweep when dynamic) as
    obstacle_class in the class grid, in place. robot=(x, y): the rover's own footprint disc is left
    free (the planner must be able to start); goal=(x, y): never covered closer than goal_keepout.
    sweep: True = dynamic tracks are swept along their prediction, False = never, or a set of track
    ids NOT to sweep (yield timeout). Returns the number of cells set."""
    n_rows, n_cols = classes.shape
    total = 0
    no_sweep = set() if sweep is True else set(sweep or ())
    for tr in plannable(tracks):
        r = tr.radius + p.safety(tr.cls, tr.dynamic) + p.robot_footprint_margin
        centres = [(tr.x, tr.y)]
        if sweep is not False and tr.dynamic and tr.id not in no_sweep:
            centres = [(x, y) for _, x, y in predict(tr, p)]
        xs = [c[0] for c in centres]
        ys = [c[1] for c in centres]
        j0 = max(0, int(math.floor((min(xs) - r - origin_x) / res)))
        j1 = min(n_cols, int(math.ceil((max(xs) + r - origin_x) / res)) + 1)
        i0 = max(0, int(math.floor((min(ys) - r - origin_y) / res)))
        i1 = min(n_rows, int(math.ceil((max(ys) + r - origin_y) / res)) + 1)
        if i1 <= i0 or j1 <= j0:
            continue
        X, Y = np.meshgrid(origin_x + (np.arange(j0, j1) + 0.5) * res, origin_y + (np.arange(i0, i1) + 0.5) * res)
        mask = np.zeros(X.shape, bool)
        # capsule: distance to the polyline of predicted centres
        for (ax, ay), (bx, by) in zip(centres, centres[1:] or centres):
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 < 1e-12:
                d = np.hypot(X - ax, Y - ay)
            else:
                f = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0.0, 1.0)
                d = np.hypot(X - ax - f * dx, Y - ay - f * dy)
            mask |= d <= r
        if robot is not None:
            mask &= np.hypot(X - robot[0], Y - robot[1]) > robot_clear
        if goal is not None:
            mask &= np.hypot(X - goal[0], Y - goal[1]) > p.goal_keepout
        sub = classes[i0:i1, j0:j1]
        total += int(np.count_nonzero(mask & (sub != obstacle_class)))
        sub[mask] = obstacle_class
    return total


# ------------------------------------------------------------------ 7. navigator integration
class DynamicAvoidance:
    """What cliff_navigator adds to the unchanged NavigatorCore (also used by the offline tests).

    overlay()  terrain classes + tracked obstacles -> classes for the planner (stamp). CLIFF / STEEP
               cells are never cleared: a cliff stays a hard constraint whatever moves near it.
    apply()    after NavigatorCore.tick(): collision prediction -> COLLISION RISK / AVOIDING / CLEAR,
               a replan request when the planned motion conflicts with a predicted obstacle, and a
               smooth speed cap (yield to a crossing person, slow near people). It only ever LOWERS
               the speed the navigator asked for: never a turn, never above max speed, no stop-forever
               (after yield_timeout the prediction sweep of that obstacle is dropped and the planner
               routes round where it is; the navigator's own escalation does the rest).
    """

    def __init__(self, p, tp, nav, obstacle_class, keep_classes=()):
        self.p, self.tp, self.nav = p, tp, nav
        self.obstacle_class = obstacle_class
        self.keep_classes = tuple(keep_classes)      # e.g. CLIFF: never relabelled by the overlay
        self.tracks = []
        self.status = 'CLEAR'
        self.risk = dict(status='CLEAR')
        self.yield_since = {}          # track id -> t it started forcing a yield
        self.no_sweep = set()          # track ids whose prediction sweep is dropped (yield timeout)
        self.avoid_until = -1.0
        self.last_replan = -1e9
        self.speed_cap = None
        self.reason = ''
        self.cells = 0
        self.events = []

    def set_tracks(self, tracks):
        self.tracks = list(tracks)
        live = {tr.id for tr in self.tracks}
        self.no_sweep &= live
        self.yield_since = {k: v for k, v in self.yield_since.items() if k in live}

    def active(self):
        return bool(plannable(self.tracks))

    def overlay(self, classes, pose, goal):
        out = classes.copy()
        robot = (pose.x, pose.y) if pose is not None else None
        # the rover's own disc stays free: its footprint corner radius
        self.cells = stamp(out, self.tp.map_origin_x, self.tp.map_origin_y, self.tp.resolution, self.tracks, self.p,
                           self.obstacle_class, robot=robot, goal=goal, sweep=self.no_sweep or True,
                           robot_clear=self.tp.circumscribed_radius - self.tp.safety_margin)
        if self.keep_classes and self.cells:
            keep = np.isin(classes, self.keep_classes)
            out[keep] = classes[keep]                # a cliff stays a cliff (hard, inflated, replanned)
        return out

    def _near_path(self, core, pose, horizon=10.0):
        """Nearest plannable obstacle to the path ahead, as (gap, track): gap = centre distance minus
        (radius + safety + footprint margin + half-width), i.e. 0 = the path touches its overlay."""
        path = core.path
        if path is None or len(path) < 2:
            return None
        pts = np.asarray(path)[core.progress:]
        d = np.r_[0.0, np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))]
        pts = pts[d <= horizon]
        best = None
        for tr in plannable(self.tracks):
            need = tr.radius + self.p.safety(tr.cls, tr.dynamic) + self.p.robot_footprint_margin + self.tp.half_width
            gap = float(np.min(np.hypot(pts[:, 0] - tr.x, pts[:, 1] - tr.y))) - need
            if best is None or gap < best[0]:
                best = (gap, tr)
        return best

    def apply(self, t, pose, core, v, w, speed):
        p, nav = self.p, self.nav
        cruise = float(nav.get('cruise_speed', 3.0))
        self.risk = assess(pose, core.path, core.progress, max(speed, abs(v)), self.tracks, p, self.tp.half_width)
        cap, why = None, ''
        if self.risk['status'] == 'COLLISION RISK':
            tid = self.risk['id']
            self.yield_since.setdefault(tid, t)
            tr = next((x for x in self.tracks if x.id == tid), None)
            if tr is not None and tr.dynamic:
                cap = yield_speed(self.risk['ttc'], cruise, decel=float(nav.get('brake_plan_decel', 1.5)))
                why = (f"yielding to a moving {tr.cls.lower()} (collision predicted in {self.risk['ttc']:.1f} s)"
                       if cap < 0.5 else f"slowing for a moving {tr.cls.lower()} ({self.risk['ttc']:.1f} s)")
            if t - self.yield_since[tid] > p.yield_timeout and tid not in self.no_sweep:
                self.no_sweep.add(tid)                     # plan round where it IS, not where it may go
                self.events.append((t, f'yield timeout: planning round obstacle {tid} as it stands'))
            if t - self.last_replan > float(nav.get('min_replan_interval', 0.5)) and core.replan_reason is None:
                self.last_replan = t
                core.replan_reason = ('dynamic', 'COLLISION RISK')
                core._event('REPLANNING', f"COLLISION RISK: {self.risk['cls'].lower()} predicted on the path in "
                                          f"{self.risk['ttc']:.1f} s - replanning", hold=1.0)
            self.avoid_until = t + 2.0
        else:
            self.yield_since = {}
        # humans first: slow down near people even when they are beside the path
        people = [tr for tr in self.tracks if tr.cls == 'HUMAN']
        if people:
            edge = min(math.hypot(tr.x - pose.x, tr.y - pose.y) - tr.radius - self.tp.half_width for tr in people)
            slow_d = float(nav.get('human_slow_distance', 4.0))
            slow_v = float(nav.get('human_slow_speed', 1.0))
            if edge < slow_d + p.human_safety_distance:
                f = max(0.0, (edge - p.human_safety_distance) / max(slow_d, 1e-3))
                c = slow_v + (cruise - slow_v) * min(1.0, f)
                if cap is None or c < cap:
                    cap, why = c, f'person {max(0.0, edge):.1f} m away - slowing down'
        near = self._near_path(core, pose)
        if near is not None and near[0] < 1.0:
            self.avoid_until = max(self.avoid_until, t + 1.0)
        self.speed_cap = cap
        self.reason = why
        if cap is not None and v > cap:
            v = cap
        if self.risk['status'] == 'COLLISION RISK':
            self.status = 'COLLISION RISK'
        elif t < self.avoid_until:
            self.status = 'AVOIDING'
        else:
            self.status = 'CLEAR'
        return v, w

    def report(self, pose):
        near = sorted(self.tracks, key=lambda tr: math.hypot(tr.x - pose.x, tr.y - pose.y))
        dyn = [tr for tr in self.tracks if tr.dynamic]
        out = dict(status=self.status, risk=self.risk, speed_cap=None if self.speed_cap is None else round(self.speed_cap, 2),
                   reason=self.reason, n_tracks=len(self.tracks), n_dynamic=len(dyn), overlay_cells=self.cells,
                   yield_timeouts=sorted(self.no_sweep))
        if near:
            tr = near[0]
            out['nearest'] = dict(id=tr.id, cls=tr.cls, dynamic=tr.dynamic,
                                  distance=round(max(0.0, math.hypot(tr.x - pose.x, tr.y - pose.y) - tr.radius), 2),
                                  speed=round(tr.speed, 2))
        return out
