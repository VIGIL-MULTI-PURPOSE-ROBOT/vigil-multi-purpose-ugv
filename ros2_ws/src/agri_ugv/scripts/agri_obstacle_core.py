#!/usr/bin/python3
"""Obstacle detection + tracking for the crop-row mission (pure numpy / OpenCV, no ROS).

Input: 3D points in the world frame (the 360 deg LiDAR, plus the depth camera for the near field
in front, where the LiDAR's lowest beam does not reach the ground), and the robot pose.

Detection
  * height above the ground under the robot; lower than obstacle_min_height -> ignored (soil,
    clods, weeds, small rocks the rover drives over)
  * the crop: points inside a crop-row band (|offset| <= tolerance + guard of the row line) and
    lower than crop_max_height are the crop the rover straddles, never obstacles
  * the rover itself is removed
  * the rest is grouped on a world grid (cluster_cell) into clusters
Tracking (not colour, not a single frame)
  * nearest-neighbour association inside track_gate (+ predicted motion)
  * alpha-beta filter on the centroid -> position and velocity
  * dynamic = filtered speed above dynamic_speed for several updates AND a net displacement of at
    least dynamic_min_displacement since the track was (re)anchored
  * a cluster whose extent changes a lot between updates (a large static object - building,
    parked tractor - coming into view piece by piece, so its centroid slides) updates the position
    but not the velocity, and the track is re-anchored: such a centroid shift is not motion
    (before this, a half-seen building at a headland became a "moving building" whose predicted
    path blocked both U-turn areas and held the rover for ever)
  * a track not seen while it should be visible is dropped after track_timeout; static obstacles
    out of sensor range are remembered for memory_timeout
"""
import math

import cv2
import numpy as np

DEFAULTS = dict(obstacle_detection_range=12.0, obstacle_min_height=0.30, crop_max_height=1.00, cluster_cell=0.30,
                min_points=4, track_gate=1.0, track_timeout=3.0, memory_timeout=120.0, dynamic_speed=0.20,
                dynamic_min_displacement=0.5, shape_change=0.25,
                robot_half_width=0.55, robot_half_length=0.85, lidar_blind_radius=2.3)


class ObstacleDetector:
    def __init__(self, ob_cfg, field, row_band):
        self.c = dict(DEFAULTS, **(ob_cfg or {}))
        self.field = field              # agri_mission_core.FieldGeometry
        self.band = float(row_band)     # crop tolerance + guard band

    def filter(self, P, robot):
        """World points (N x 3) -> obstacle points (M x 3) with height above ground."""
        if P is None or not len(P):
            return np.zeros((0, 3))
        c = self.c
        rx, ry, yaw, rz = robot
        P = np.asarray(P, float)
        P = P[np.isfinite(P).all(1)]
        dx, dy = P[:, 0] - rx, P[:, 1] - ry
        dist = np.hypot(dx, dy)
        u = math.cos(yaw) * dx + math.sin(yaw) * dy
        w = -math.sin(yaw) * dx + math.cos(yaw) * dy
        # ground height: the lowest return in each 1.5 m cell (terrain is not perfectly flat),
        # never far from the ground under the robot
        cell = 1.5
        key = np.floor(P[:, 0] / cell).astype(np.int64) * 1000003 + np.floor(P[:, 1] / cell).astype(np.int64)
        _, inv = np.unique(key, return_inverse=True)
        zmin = np.full(inv.max() + 1 if len(inv) else 0, np.inf)
        np.minimum.at(zmin, inv, P[:, 2])
        ground = np.clip(zmin[inv], rz - 0.5, rz + 0.5)
        h = P[:, 2] - ground
        keep = (dist < c['obstacle_detection_range']) & (h > c['obstacle_min_height']) & (h < 4.0)
        keep &= ~((np.abs(u) < c['robot_half_length'] + 0.15) & (np.abs(w) < c['robot_half_width'] + 0.15))
        g = self.field
        k = g.row_index(P[:, 1])
        off = np.abs(P[:, 1] - g.row_y(k))
        in_rows = (P[:, 0] > g.x_start - 0.3) & (P[:, 0] < g.x_end + 0.3) & \
                  (P[:, 1] > g.y0 - g.spacing / 2) & (P[:, 1] < g.row_y(g.n - 1) + g.spacing / 2)
        crop = in_rows & (off <= self.band) & (h < c['crop_max_height'])
        keep &= ~crop
        out = P[keep].copy()
        out[:, 2] = h[keep]
        return out

    def cluster(self, Q):
        """Obstacle points (x, y, h) -> clusters."""
        c = self.c
        if len(Q) < c['min_points']:
            return []
        cell = c['cluster_cell']
        ix = np.floor(Q[:, 0] / cell).astype(int)
        iy = np.floor(Q[:, 1] / cell).astype(int)
        ox, oy = ix.min(), iy.min()
        grid = np.zeros((iy.max() - oy + 3, ix.max() - ox + 3), np.uint8)
        grid[iy - oy + 1, ix - ox + 1] = 1
        n, lab = cv2.connectedComponents(grid, connectivity=8)
        cl = lab[iy - oy + 1, ix - ox + 1]
        out = []
        for i in range(1, n):
            s = cl == i
            if s.sum() < c['min_points']:
                continue
            q = Q[s]
            cx, cy = float(q[:, 0].mean()), float(q[:, 1].mean())
            pts = q[:, :2]
            if len(pts) > 3:
                try:
                    hull = cv2.convexHull(pts.astype(np.float32)).reshape(-1, 2)
                except cv2.error:
                    hull = pts
            else:
                hull = pts
            if len(hull) > 24:
                hull = hull[np.linspace(0, len(hull) - 1, 24).astype(int)]
            out.append(dict(x=cx, y=cy, height=float(np.percentile(q[:, 2], 95)), n=int(s.sum()),
                            size=float(max(np.ptp(q[:, 0]), np.ptp(q[:, 1]), cell)),
                            offsets=(hull - [cx, cy]).tolist()))
        return out

    def detect(self, P, robot):
        return self.cluster(self.filter(P, robot))


class Track:
    def __init__(self, tid, d, t):
        self.id, self.x, self.y = tid, d['x'], d['y']
        self.vx = self.vy = 0.0
        self.t, self.first = t, t
        self.hits, self.moving_hits = 1, 0
        self.ax, self.ay, self.asize = d['x'], d['y'], d['size']   # anchor for the net-displacement test
        self.height, self.size, self.offsets = d['height'], d['size'], d['offsets']
        self.dynamic = False
        self.cls = 'obstacle'
        self.announced = False


class ObstacleTracker:
    def __init__(self, ob_cfg):
        self.c = dict(DEFAULTS, **(ob_cfg or {}))
        self.tracks = []
        self.next_id = 1

    def update(self, clusters, t, robot):
        c = self.c
        a, b = 0.5, 0.25
        free = list(range(len(clusters)))
        for tr in sorted(self.tracks, key=lambda tr: -tr.hits):
            dt = max(t - tr.t, 1e-3)
            px, py = tr.x + tr.vx * dt, tr.y + tr.vy * dt
            best, bd = None, c['track_gate'] + math.hypot(tr.vx, tr.vy) * dt
            for i in free:
                d = math.hypot(clusters[i]['x'] - px, clusters[i]['y'] - py)
                if d < bd:
                    best, bd = i, d
            if best is None:
                continue
            free.remove(best)
            m = clusters[best]
            rx, ry = m['x'] - px, m['y'] - py
            if abs(m['size'] - tr.size) > float(c['shape_change']) * max(tr.size, m['size'], 0.3):
                # extent changed: the centroid moved because more / less of the object is seen
                tr.x, tr.y = m['x'], m['y']
                tr.vx, tr.vy = tr.vx * 0.5, tr.vy * 0.5
                tr.ax, tr.ay, tr.asize = m['x'], m['y'], m['size']
                tr.moving_hits = max(0, tr.moving_hits - 1)
                tr.t, tr.hits = t, tr.hits + 1
                tr.height, tr.size, tr.offsets = m['height'], m['size'], m['offsets']
                tr.dynamic = tr.dynamic and tr.moving_hits >= 3
                continue
            tr.x, tr.y = px + a * rx, py + a * ry
            tr.vx, tr.vy = tr.vx + b * rx / dt, tr.vy + b * ry / dt
            sp = math.hypot(tr.vx, tr.vy)
            if sp > 3.0:                                    # physically implausible: re-seed
                tr.vx, tr.vy = tr.vx * 3.0 / sp, tr.vy * 3.0 / sp
            tr.t, tr.hits = t, tr.hits + 1
            tr.height, tr.size, tr.offsets = m['height'], m['size'], m['offsets']
            tr.moving_hits = tr.moving_hits + 1 if math.hypot(tr.vx, tr.vy) > c['dynamic_speed'] else max(0, tr.moving_hits - 1)
            # a centroid shift explained by the extent growing / shrinking (object revealed piece by
            # piece) is not motion
            moved = math.hypot(tr.x - tr.ax, tr.y - tr.ay) - abs(tr.size - tr.asize) / 2 >= \
                float(c['dynamic_min_displacement'])
            tr.dynamic = tr.moving_hits >= 3 and (moved or tr.dynamic)
            if not tr.dynamic and tr.moving_hits == 0:
                tr.ax, tr.ay, tr.asize = tr.x, tr.y, tr.size   # at rest: re-anchor
        for i in free:
            self.tracks.append(Track(self.next_id, clusters[i], t))
            self.next_id += 1
        rx, ry = robot[0], robot[1]
        keep = []
        for tr in self.tracks:
            age = t - tr.t
            dist = math.hypot(tr.x - rx, tr.y - ry)
            visible = c['lidar_blind_radius'] < dist < c['obstacle_detection_range'] - 1.0
            limit = c['track_timeout'] if (visible or tr.dynamic) else c['memory_timeout']
            if age <= limit:
                if not visible and age > 1.0:
                    tr.vx *= 0.9                              # unseen: stop extrapolating motion
                    tr.vy *= 0.9
                keep.append(tr)
        self.tracks = keep
        return self.tracks

    def export(self, t, robot, labels=()):
        """JSON-ready list for the mission (/agriculture/obstacles)."""
        out = []
        for tr in self.tracks:
            if tr.hits < 2:
                continue
            dt = max(0.0, t - tr.t)
            x, y = tr.x + tr.vx * min(dt, 1.0), tr.y + tr.vy * min(dt, 1.0)
            cls = tr.cls
            for lab in labels:
                if math.hypot(lab['x'] - x, lab['y'] - y) < 1.0:
                    cls = lab['cls']
                    break
            if cls == 'obstacle' and tr.size > 1.5:
                cls = 'large obstacle'
            tr.cls = cls
            dx, dy = x - robot[0], y - robot[1]
            bearing = math.degrees(math.atan2(dy, dx) - robot[2])
            bearing = (bearing + 180) % 360 - 180
            out.append(dict(id=f'OB{tr.id}', x=round(x, 3), y=round(y, 3), vx=round(tr.vx, 3), vy=round(tr.vy, 3),
                            speed=round(math.hypot(tr.vx, tr.vy), 3), dynamic=tr.dynamic, radius=0.05,
                            points=[[round(x + o[0], 3), round(y + o[1], 3)] for o in tr.offsets],
                            height=round(tr.height, 2), size=round(tr.size, 2), cls=cls, confirmed=True,
                            age=round(t - tr.first, 1), last_seen=round(dt, 2), distance=round(math.hypot(dx, dy), 2),
                            bearing=round(bearing, 1)))
        return out
