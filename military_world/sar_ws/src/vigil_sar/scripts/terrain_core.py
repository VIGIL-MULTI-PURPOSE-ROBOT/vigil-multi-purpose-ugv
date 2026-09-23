#!/usr/bin/python3
"""Depth-based terrain + cliff analysis for the VIGIL rover (no ROS imports).

Pipeline (one call to TerrainMapper.process per depth frame):
  1. depth image -> 3-D points in the world frame (camera intrinsics + robot pose)
  2. points -> 2.5-D elevation grid (weighted running mean per cell)
  3. per image column, near -> far edge scan: finds "ground vanishes" events
     (range jump + height loss, or no return at all for a downward ray) and
     marks the edge + the occluded region behind it
  4. map-space analysis: local drop/step, plane-fit slope, roughness
  5. classification: UNKNOWN / SAFE / UNEVEN / CLIMBABLE / POSSIBLE_DROP / STEEP / OBSTACLE / CLIFF

Hills vs cliffs: slopes up to max_safe_slope_deg are normal driving, slopes between that
and max_climb_slope_deg (the physical climbing limit) are CLIMBABLE (costly, never an
obstacle, never inflated). Only drops > max_safe_drop whose hidden face is steeper than
the climbing limit become HIGH CLIFF.

Why the edge scan is needed: a camera 0.65 m above the ground can never see the
bottom face of a drop. A vertical cliff shows up as (a) observed ground that ends,
(b) a gap with no data, (c) ground far below / no return. The ray that grazes the
edge descends at angle theta. Anything hidden beyond the edge descends at least
that steeply. If the lower ground is visible (a drop dz over a gap dh), the hidden
part must also be at least atan(dz/dh) steep somewhere. With theta = the larger:
  theta >= max_climb_slope and drop > max_safe_drop -> CLIFF (lethal, confirmed)
  theta <  max_climb_slope and drop > max_safe_drop -> POSSIBLE_DROP (costly, not
      lethal: from far away a hidden steep-but-legal downslope looks the same;
      the robot confirms or clears it as it gets closer and theta grows).
"""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

UNKNOWN, SAFE, UNEVEN, POSSIBLE_DROP, STEEP, OBSTACLE, CLIFF = range(7)
CLIMBABLE = 7  # appended so existing class ids stay unchanged
CLASS_NAMES = ['UNKNOWN', 'SAFE', 'UNEVEN', 'POSSIBLE DROP', 'STEEP', 'OBSTACLE', 'HIGH CLIFF', 'CLIMBABLE']
LETHAL = (STEEP, OBSTACLE, CLIFF)
# Display severity (class ids are not ordered by danger once CLIMBABLE was appended).
SEVERITY = {UNKNOWN: 0, SAFE: 1, UNEVEN: 2, CLIMBABLE: 3, POSSIBLE_DROP: 4, STEEP: 5, OBSTACLE: 6, CLIFF: 7}
_SEV_LUT = np.zeros(256, np.int16)
for _k, _v in SEVERITY.items():
    _SEV_LUT[_k] = _v
# BGR colours used by the overlay and the dashboard map.
CLASS_BGR = {
    UNKNOWN: (40, 40, 40), SAFE: (80, 200, 60), UNEVEN: (0, 200, 255),
    POSSIBLE_DROP: (0, 140, 255), STEEP: (60, 110, 170), OBSTACLE: (200, 0, 200), CLIMBABLE: (255, 150, 170),
    CLIFF: (30, 30, 255)}
# Occupancy cost (0-100) published on /vision/traversability for RViz / Nav2.
CLASS_COST = {UNKNOWN: -1, SAFE: 0, UNEVEN: 30, CLIMBABLE: 50, POSSIBLE_DROP: 70, STEEP: 100,
              OBSTACLE: 100, CLIFF: 100}


def _get(d, key, default):
    return d.get(key, default) if isinstance(d, dict) else default


@dataclass
class Params:
    """All parameters; see config/vision_nav.yaml for where each comes from."""
    robot_length: float = 1.53
    robot_width: float = 1.12
    robot_height: float = 1.14
    wheelbase: float = 0.82
    ground_clearance: float = 0.3375
    wheel_radius_small: float = 0.1766
    footprint_center_x: float = 0.02
    ground_offset_z: float = -0.052
    max_tilt_deg: float = 38.0
    cam_x: float = 0.4672
    cam_y: float = 0.0
    cam_z: float = 0.5991
    cam_pitch: float = 0.1745
    cam_hfov: float = 1.5184
    cam_width: int = 320
    cam_height: int = 240
    depth_near: float = 0.2
    depth_far: float = 15.0
    map_size: float = 32.0
    map_origin_x: float = -16.0
    map_origin_y: float = -16.0
    resolution: float = 0.10
    max_safe_drop: float = 0.17
    max_safe_step: float = 0.17
    max_climb_step: float = 0.23
    max_climb_slope_deg: float = 33.0
    max_safe_slope_deg: float = 25.0
    uneven_slope_deg: float = 10.0
    uneven_step: float = 0.06
    uneven_roughness: float = 0.03
    safety_margin: float = 0.15
    cliff_detection_distance: float = 6.0
    slope_window: float = 0.9
    min_hits: int = 2
    pixel_stride: int = 2
    use_segmentation: bool = True
    # [SAR] operational zone (world.zone_*); None = the whole map
    zone: tuple = None                 # (x_min, x_max, y_min, y_max)

    @property
    def half_width(self):
        return self.robot_width / 2.0

    @property
    def inscribed_radius(self):
        """Clearance needed sideways when driving straight."""
        return self.robot_width / 2.0 + self.safety_margin

    @property
    def circumscribed_radius(self):
        """Clearance needed to rotate in place (footprint corner sweep)."""
        return math.hypot(self.robot_length / 2.0, self.robot_width / 2.0) + self.safety_margin

    @classmethod
    def from_nested(cls, cfg):
        """Build from the nested dict structure of vision_nav.yaml."""
        r, c, t = _get(cfg, 'robot', {}), _get(cfg, 'camera', {}), _get(cfg, 'terrain', {})
        p = cls()
        mapping = {
            'robot_length': (r, 'length'), 'robot_width': (r, 'width'), 'robot_height': (r, 'height'),
            'wheelbase': (r, 'wheelbase'), 'ground_clearance': (r, 'ground_clearance'),
            'wheel_radius_small': (r, 'wheel_radius_small'),
            'footprint_center_x': (r, 'footprint_center_x'), 'ground_offset_z': (r, 'ground_offset_z'),
            'max_tilt_deg': (r, 'max_tilt_deg'),
            'cam_x': (c, 'x'), 'cam_y': (c, 'y'), 'cam_z': (c, 'z'), 'cam_pitch': (c, 'pitch'),
            'cam_hfov': (c, 'horizontal_fov'), 'cam_width': (c, 'width'), 'cam_height': (c, 'height'),
            'depth_near': (c, 'depth_near'), 'depth_far': (c, 'depth_far'),
        }
        for k in ['map_size', 'map_origin_x', 'map_origin_y', 'resolution', 'max_safe_drop',
                  'max_safe_step', 'max_climb_step', 'max_climb_slope_deg', 'max_safe_slope_deg', 'uneven_slope_deg', 'uneven_step',
                  'uneven_roughness', 'safety_margin', 'cliff_detection_distance', 'slope_window',
                  'min_hits', 'pixel_stride', 'use_segmentation']:
            mapping[k] = (t, k)
        for attr, (section, key) in mapping.items():
            if isinstance(section, dict) and key in section:
                setattr(p, attr, type(getattr(p, attr))(section[key]))
        # [SAR] the operational zone decides the map: origin = zone corner, size = larger side
        w = _get(cfg, 'world', {})
        if isinstance(w, dict) and all(k in w for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max')):
            z = tuple(float(w[k]) for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max'))
            if z[1] > z[0] and z[3] > z[2]:
                p.zone = z
                p.map_origin_x, p.map_origin_y = z[0], z[2]
                p.map_size = max(z[1] - z[0], z[3] - z[2])
        return p

    def in_zone(self, x, y, margin=0.0):
        z = self.zone or (self.map_origin_x, self.map_origin_x + self.map_size,
                          self.map_origin_y, self.map_origin_y + self.map_size)
        return z[0] + margin <= x <= z[1] - margin and z[2] + margin <= y <= z[3] - margin

    def clamp_to_zone(self, x, y, margin=0.0):
        z = self.zone or (self.map_origin_x, self.map_origin_x + self.map_size,
                          self.map_origin_y, self.map_origin_y + self.map_size)
        return (min(max(x, z[0] + margin), z[1] - margin), min(max(y, z[2] + margin), z[3] - margin))

    def sanity_report(self):
        """Human-readable check that thresholds agree with the robot geometry."""
        lines = [
            f'footprint {self.robot_length:.2f} x {self.robot_width:.2f} m, clearance {self.ground_clearance:.3f} m',
            f'max_safe_drop {self.max_safe_drop:.3f} m (smallest wheel radius {self.wheel_radius_small:.3f} m)',
            f'max_safe_step {self.max_safe_step:.3f} m (climbable to {self.max_climb_step:.3f} m), '
            f'max_safe_slope {self.max_safe_slope_deg:.1f} deg (climbable to {self.max_climb_slope_deg:.1f} deg)',
            f'inflation: straight {self.inscribed_radius:.2f} m, turn-in-place {self.circumscribed_radius:.2f} m']
        if self.max_safe_drop > self.ground_clearance or self.max_safe_step > self.ground_clearance:
            lines.append('WARNING: drop/step threshold exceeds ground clearance')
        if self.max_safe_drop > self.wheel_radius_small * 1.05:
            lines.append('WARNING: max_safe_drop larger than the smallest wheel radius')
        return lines


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


@dataclass
class Pose:
    """base_footprint pose in the world frame."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    R: np.ndarray = field(default_factory=lambda: np.eye(3))

    @property
    def yaw(self):
        return math.atan2(self.R[1, 0], self.R[0, 0])

    @property
    def tilt_deg(self):
        """Angle between body z axis and world z (combined roll+pitch)."""
        return math.degrees(math.acos(max(-1.0, min(1.0, self.R[2, 2]))))

    @property
    def roll_pitch_deg(self):
        pitch = math.asin(max(-1.0, min(1.0, -self.R[2, 0])))
        roll = math.atan2(self.R[2, 1], self.R[2, 2])
        return math.degrees(roll), math.degrees(pitch)

    def ground_z(self, p):
        return self.z + p.ground_offset_z


class CameraModel:
    """Pinhole model + fixed extrinsic base_footprint -> camera optical frame."""

    def __init__(self, p, K=None, width=None, height=None):
        self.w = int(width or p.cam_width)
        self.h = int(height or p.cam_height)
        if K is not None and K[0] > 0:
            self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
        else:
            self.fx = self.w / 2.0 / math.tan(p.cam_hfov / 2.0)
            self.fy = self.fx
            self.cx, self.cy = (self.w - 1) / 2.0, (self.h - 1) / 2.0
        # camera_link pitched down by cam_pitch; optical: z fwd, x right, y down.
        R_link = rpy_to_matrix(0.0, p.cam_pitch, 0.0)
        R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        self.R_fp_opt = R_link @ R_link_opt
        self.t_fp_opt = np.array([p.cam_x, p.cam_y, p.cam_z])
        u, v = np.meshgrid(np.arange(self.w, dtype=np.float32), np.arange(self.h, dtype=np.float32))
        self.rays = np.stack([(u - self.cx) / self.fx, (v - self.cy) / self.fy,
                              np.ones_like(u)], axis=-1)  # planar-depth rays (z = 1)

    def world_transform(self, pose):
        R = pose.R @ self.R_fp_opt
        t = pose.R @ self.t_fp_opt + np.array([pose.x, pose.y, pose.z])
        return R, t

    def backproject(self, depth, pose, p):
        """Return world points (H,W,3), valid mask, void mask, ray dirs, origin."""
        R, t = self.world_transform(pose)
        d = depth.astype(np.float32)
        finite = np.isfinite(d)
        valid = finite & (d >= p.depth_near) & (d < p.depth_far * 0.999)
        no_return = (~finite & (d > 0)) | (finite & (d >= p.depth_far * 0.999))
        dd = np.where(valid, d, p.depth_far).astype(np.float32)
        pts_opt = self.rays * dd[..., None]
        pts = pts_opt @ R.T.astype(np.float32) + t.astype(np.float32)
        dirs = self.rays @ R.T.astype(np.float32)
        return pts, valid, no_return, dirs, t

    def project(self, pts_world, pose):
        """World points (N,3) -> pixel (N,2), in-front mask."""
        R, t = self.world_transform(pose)
        pc = (np.asarray(pts_world, dtype=float) - t) @ R
        z = pc[:, 2]
        front = z > 0.05
        zs = np.where(front, z, 1.0)
        return np.stack([self.fx * pc[:, 0] / zs + self.cx, self.fy * pc[:, 1] / zs + self.cy], 1), front


class TerrainMapper:
    """World-frame elevation map + cliff classification."""

    def __init__(self, p):
        self.p = p
        self.n = int(round(p.map_size / p.resolution))
        n = self.n
        self.wsum = np.zeros((n, n), np.float32)
        self.hsum = np.zeros((n, n), np.float32)
        self.hits = np.zeros((n, n), np.int32)
        # Inferred layers from the edge scan (persist until contradicted).
        self.edge_drop = np.zeros((n, n), np.float32)    # drop measured at an edge cell
        self.edge_theta = np.zeros((n, n), np.float32)   # grazing-ray angle (rad)
        self.shadow_drop = np.zeros((n, n), np.float32)  # occluded region behind an edge
        self.shadow_theta = np.zeros((n, n), np.float32)
        self.driven = np.zeros((n, n), bool)
        self.classes = np.zeros((n, n), np.uint8)
        self.height = np.full((n, n), np.nan, np.float32)
        self.slope_deg = np.zeros((n, n), np.float32)
        self.drop = np.zeros((n, n), np.float32)
        self.roughness = np.zeros((n, n), np.float32)
        self.step = np.zeros((n, n), np.float32)
        self.last_frame = None
        self.frames = 0

    # ---------------------------------------------------------------- grid
    def cell(self, x, y):
        i = np.floor((np.asarray(y) - self.p.map_origin_y) / self.p.resolution).astype(int)
        j = np.floor((np.asarray(x) - self.p.map_origin_x) / self.p.resolution).astype(int)
        return i, j

    def inside(self, i, j):
        return (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)

    def cell_center(self, i, j):
        return (self.p.map_origin_x + (np.asarray(j) + 0.5) * self.p.resolution,
                self.p.map_origin_y + (np.asarray(i) + 0.5) * self.p.resolution)

    # ------------------------------------------------------------- process
    def process(self, depth, pose, cam, seg=None, rgb_shape=None):
        p = self.p
        pts, valid, no_return, dirs, origin = cam.backproject(depth, pose, p)
        dist_h = np.hypot(pts[..., 0] - origin[0], pts[..., 1] - origin[1])
        in_range = valid & (dist_h <= p.cliff_detection_distance + 2.0)
        self._integrate(pts, in_range, dist_h)
        down = dirs[..., 2] < -1e-3
        void = no_return & down
        if seg is not None and p.use_segmentation and seg.shape[:2] == depth.shape[:2]:
            # Unlabelled pixels (label 0 = sky / nothing) on a downward ray also
            # mean "no ground there", even if depth reports a far value.
            void |= (seg == 0) & down & ~valid
        self._mark_driven(pose)
        self._edge_scan(pts, valid, void, dirs, origin, pose)
        self.analyze()
        self.frames += 1
        # void rays that would have hit level ground within the detection range
        drop_to_ground = max(1e-3, float(origin[2]) - pose.ground_z(p))
        hor = np.hypot(dirs[..., 0], dirs[..., 1])
        reach = drop_to_ground / np.maximum(-dirs[..., 2], 1e-6) * hor
        void_near = void & (reach <= p.cliff_detection_distance)
        self.last_frame = dict(pts=pts, valid=valid, void=void_near, pose=pose, origin=origin)
        return self.last_frame

    def _integrate(self, pts, mask, dist_h):
        p = self.p
        x, y, z = pts[..., 0][mask], pts[..., 1][mask], pts[..., 2][mask]
        if x.size == 0:
            return
        i, j = self.cell(x, y)
        ok = self.inside(i, j)
        i, j, z, w = i[ok], j[ok], z[ok], 1.0 / (1.0 + dist_h[mask][ok] ** 2)
        flat = i * self.n + j
        size = self.n * self.n
        W = np.bincount(flat, weights=w, minlength=size).reshape(self.n, self.n).astype(np.float32)
        H = np.bincount(flat, weights=w * z, minlength=size).reshape(self.n, self.n).astype(np.float32)
        C = np.bincount(flat, minlength=size).reshape(self.n, self.n).astype(np.int32)
        seen = C > 0
        # Newer observations dominate: cap the accumulated weight.
        cap = 20.0
        scale = np.where(self.wsum > cap, cap / np.maximum(self.wsum, 1e-9), 1.0).astype(np.float32)
        self.wsum = self.wsum * scale + W
        self.hsum = self.hsum * scale + H
        self.hits += C
        # A directly observed cell is no longer an occluded shadow.
        obs = seen & (self.hits >= p.min_hits)
        self.shadow_drop[obs] = 0.0
        self.shadow_theta[obs] = 0.0

    def _mark_driven(self, pose):
        """Cells under the current footprint are physically supported: known SAFE."""
        p = self.p
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        hl, hw = p.robot_length / 2.0, p.robot_width / 2.0
        xs = np.arange(-hl, hl + 1e-6, p.resolution / 2) + p.footprint_center_x
        ys = np.arange(-hw, hw + 1e-6, p.resolution / 2)
        X, Y = np.meshgrid(xs, ys)
        wx = pose.x + c * X - s * Y
        wy = pose.y + s * X + c * Y
        i, j = self.cell(wx.ravel(), wy.ravel())
        ok = self.inside(i, j)
        self.driven[i[ok], j[ok]] = True

    def _edge_scan(self, pts, valid, void, dirs, origin, pose):
        """Column-wise near->far scan for ground that vanishes (see module doc)."""
        p = self.p
        st = max(1, int(p.pixel_stride))
        tan_s = math.tan(math.radians(p.max_climb_slope_deg))
        H, W = valid.shape
        gz0 = pose.ground_z(p)
        # Reference ground point at the front of the footprint (robot stands there).
        fx = pose.x + math.cos(pose.yaw) * (p.footprint_center_x + p.robot_length / 2)
        fy = pose.y + math.sin(pose.yaw) * (p.footprint_center_x + p.robot_length / 2)
        P = pts[::-1][::st, ::st]         # bottom (near) rows first
        V = valid[::-1][::st, ::st]
        VO = void[::-1][::st, ::st]
        D = dirs[::-1][::st, ::st]
        rows, cols = V.shape
        events = []  # (x_ref, y_ref, z_ref, x_land, y_land, drop, theta, is_void)
        clears = []
        maxd = p.cliff_detection_distance
        ox, oy, oz = float(origin[0]), float(origin[1]), float(origin[2])
        Px, Py, Pz = P[..., 0].tolist(), P[..., 1].tolist(), P[..., 2].tolist()
        Vl, VOl = V.tolist(), VO.tolist()
        Dx, Dy, Dz = D[..., 0].tolist(), D[..., 1].tolist(), D[..., 2].tolist()
        for c in range(cols):
            rx, ry, rz = fx, fy, gz0
            void_done = False
            for r in range(rows):
                if Vl[r][c]:
                    x, y, z = Px[r][c], Py[r][c], Pz[r][c]
                    if math.hypot(x - ox, y - oy) > maxd + 1.0:
                        break
                    dh = math.hypot(x - rx, y - ry)
                    dz = rz - z
                    if dz > p.max_safe_drop + dh * 0.5 * tan_s and dh > p.resolution * 1.5:
                        # ground vanished between ref and this point
                        ddx, ddy = x - ox, y - oy
                        theta = math.atan2(oz - z, math.hypot(ddx, ddy))
                        # Two lower bounds on how steep the hidden part is:
                        #  - it lies below the grazing ray (angle theta)
                        #  - it loses dz over dh horizontally, so somewhere the
                        #    slope is at least atan(dz/dh) (mean-value theorem)
                        theta = max(theta, math.atan2(dz, dh))
                        events.append((rx, ry, rz, x, y, dz, theta, False))
                        rx, ry, rz = x, y, z
                    elif z - rz > p.max_climb_step + dh * tan_s:
                        pass  # elevated (rock top / wall): keep the ground reference
                    else:
                        if dh < 2.5 * p.resolution and abs(dz) < p.max_safe_drop:
                            clears.append((rx, ry))
                        rx, ry, rz = x, y, z
                elif VOl[r][c] and not void_done:
                    dz_ray = Dz[r][c]
                    hor = math.hypot(Dx[r][c], Dy[r][c])
                    if hor < 1e-6:
                        continue
                    # where this ray would meet a plane at the reference height
                    drop_needed = oz - rz
                    if drop_needed <= 0:
                        continue
                    t_hit = drop_needed / -dz_ray
                    hx, hy = ox + Dx[r][c] * t_hit, oy + Dy[r][c] * t_hit
                    if math.hypot(hx - ox, hy - oy) > maxd:
                        continue
                    # Nothing within depth_far along this ray: lower bound on the drop.
                    t_far = p.depth_far / max(1e-6, math.sqrt(Dx[r][c] ** 2 + Dy[r][c] ** 2 + dz_ray ** 2))
                    zf = oz + dz_ray * t_far
                    theta = math.atan2(-dz_ray, hor)
                    lx, ly = ox + Dx[r][c] * min(t_far, t_hit * 3), oy + Dy[r][c] * min(t_far, t_hit * 3)
                    events.append((rx, ry, rz, lx, ly, rz - zf, theta, True))
                    void_done = True
        self._apply_events(events, clears)

    def _apply_events(self, events, clears):
        p = self.p
        res = p.resolution
        if clears:
            c = np.array(clears)
            i, j = self.cell(c[:, 0], c[:, 1])
            ok = self.inside(i, j)
            self.edge_drop[i[ok], j[ok]] = 0.0
            self.edge_theta[i[ok], j[ok]] = 0.0
        for rx, ry, rz, lx, ly, drop, theta, _void in events:
            i, j = self.cell(rx, ry)
            if not self.inside(i, j):
                continue
            self.edge_drop[i, j] = max(self.edge_drop[i, j], drop)
            self.edge_theta[i, j] = max(self.edge_theta[i, j], theta)
            L = math.hypot(lx - rx, ly - ry)
            if L < 1e-6:
                continue
            # Shadow: the occluded strip between the edge and the landing point,
            # capped at 2 m (enough for the planner; far data gets refreshed).
            n = int(min(L, 2.0) / (res * 0.5))
            if n < 1:
                continue
            ts = (np.arange(1, n + 1) * res * 0.5) / L
            sx, sy = rx + (lx - rx) * ts, ry + (ly - ry) * ts
            si, sj = self.cell(sx, sy)
            ok = self.inside(si, sj)
            si, sj = si[ok], sj[ok]
            unseen = self.hits[si, sj] < p.min_hits
            si, sj = si[unseen], sj[unseen]
            self.shadow_drop[si, sj] = np.maximum(self.shadow_drop[si, sj], drop)
            self.shadow_theta[si, sj] = np.maximum(self.shadow_theta[si, sj], theta)

    # ------------------------------------------------------------ analysis
    def analyze(self):
        p = self.p
        res = p.resolution
        known = self.hits >= p.min_hits
        h = np.where(known, self.hsum / np.maximum(self.wsum, 1e-9), 0).astype(np.float32)
        self.height = np.where(known, h, np.nan).astype(np.float32)
        k = known.astype(np.float32)
        big, small = np.float32(1e6), np.float32(-1e6)
        kernel = np.ones((3, 3), np.uint8)
        hmin = cv2.erode(np.where(known, h, big).astype(np.float32), kernel)
        hmax = cv2.dilate(np.where(known, h, small).astype(np.float32), kernel)
        allow = res * math.sqrt(2) * math.tan(math.radians(p.max_climb_slope_deg))
        drop = np.where(known & (hmin < big / 2), h - hmin - allow, 0)
        step = np.where(known & (hmax > small / 2), hmax - h - allow, 0)
        self.drop = np.maximum(drop, 0).astype(np.float32)
        self.step = np.maximum(step, 0).astype(np.float32)
        # Normalised-convolution smoothing for slope / neighbourhood mean.
        ks = max(3, int(round(p.slope_window / res)) | 1)
        ksum = cv2.boxFilter(k, -1, (ks, ks), normalize=False)
        hsm = cv2.boxFilter(h * k, -1, (ks, ks), normalize=False) / np.maximum(ksum, 1e-6)
        h2 = cv2.boxFilter(h * h * k, -1, (ks, ks), normalize=False) / np.maximum(ksum, 1e-6)
        gx = cv2.Sobel(hsm, cv2.CV_32F, 1, 0, ksize=3) / (8 * res)
        gy = cv2.Sobel(hsm, cv2.CV_32F, 0, 1, ksize=3) / (8 * res)
        # Roughness = height scatter AROUND the local plane: a smooth hill is not rough.
        # (A plane of gradient g over a w x w window has variance g^2 w^2 / 12.)
        plane_var = (gx * gx + gy * gy) * (ks * res) ** 2 / 12.0
        rough = np.sqrt(np.maximum(h2 - hsm * hsm - plane_var, 0))
        self.roughness = np.where(known, rough, 0).astype(np.float32)
        # Only trust the gradient where the whole 3x3 support is observed.
        support = cv2.boxFilter((ksum > ks).astype(np.float32), -1, (3, 3)) > 0.99
        slope = np.degrees(np.arctan(np.hypot(gx, gy)))
        self.slope_deg = np.where(known & support, slope, 0).astype(np.float32)
        kw = max(5, int(round(1.0 / res)) | 1)
        wsum = cv2.boxFilter(k, -1, (kw, kw), normalize=False)
        wmean = cv2.boxFilter(h * k, -1, (kw, kw), normalize=False) / np.maximum(wsum, 1e-6)
        elevated = known & (h - wmean > p.max_climb_step) & (wsum > kw)
        depressed = known & (wmean - h > p.max_safe_drop) & (wsum > kw)

        cls = np.full(h.shape, UNKNOWN, np.uint8)
        uneven = known & ((self.slope_deg > p.uneven_slope_deg) | (rough > p.uneven_roughness)
                          | (self.step > p.uneven_step) | (self.drop > p.uneven_step))
        cls[known] = SAFE
        cls[uneven] = UNEVEN
        # Hills: above the comfortable limit but within the climbing limit = CLIMBABLE;
        # above the physical climbing limit = STEEP. Steps between the safe and the
        # climbable step height are CLIMBABLE too (rocker-bogie + leading 0.235 m wheel).
        cls[known & (self.slope_deg > p.max_safe_slope_deg)] = CLIMBABLE
        cls[known & (self.step > p.max_safe_step) & (self.step <= p.max_climb_step)] = CLIMBABLE
        cls[known & (self.slope_deg > p.max_climb_slope_deg)] = STEEP
        cls[known & ((self.step > p.max_climb_step) | elevated)] = OBSTACLE
        cls[known & ((self.drop > p.max_safe_drop) & ~elevated)] = CLIFF
        cls[depressed & ((self.step > p.max_climb_step) | (self.drop > p.max_safe_drop))] = CLIFF
        # Inferred from the edge scan.
        # A hidden descent is only a cliff if it is steeper than the robot can climb.
        theta_ok = math.radians(p.max_climb_slope_deg)
        big_drop_edge = self.edge_drop > p.max_safe_drop
        big_drop_shadow = self.shadow_drop > p.max_safe_drop
        possible = (big_drop_shadow & ~known) | (big_drop_edge & (self.edge_theta < theta_ok))
        cls[possible & ((cls <= UNEVEN) | (cls == CLIMBABLE))] = POSSIBLE_DROP
        confirmed_edge = big_drop_edge & (self.edge_theta >= theta_ok)
        confirmed_shadow = big_drop_shadow & (self.shadow_theta >= theta_ok) & ~known
        cls[confirmed_edge | confirmed_shadow] = CLIFF
        # Where the robot has physically stood, unknown/possible become SAFE.
        cls[self.driven & ((cls == UNKNOWN) | (cls == POSSIBLE_DROP))] = SAFE
        self.classes = cls
        return cls

    # -------------------------------------------------------- status ahead
    def status_ahead(self, pose, heading=None):
        """Summarise the corridor the robot is about to drive through."""
        p = self.p
        yaw = pose.yaw if heading is None else heading
        c, s = math.cos(yaw), math.sin(yaw)
        front = p.footprint_center_x + p.robot_length / 2
        hw = p.robot_width / 2 + p.safety_margin
        res = p.resolution
        worst, drop, slope, safe_dist = SAFE, 0.0, 0.0, p.cliff_detection_distance
        blocking = None
        lat = np.arange(-hw, hw + 1e-6, res / 2)
        seen_any = False
        for d in np.arange(0.0, p.cliff_detection_distance, res):
            wx = pose.x + c * (front + d) - s * lat
            wy = pose.y + s * (front + d) + c * lat
            i, j = self.cell(wx, wy)
            ok = self.inside(i, j)
            if not ok.any():
                break
            cl = self.classes[i[ok], j[ok]]
            if (cl != UNKNOWN).any():
                seen_any = True
            drop = max(drop, float(self.edge_drop[i[ok], j[ok]].max()), float(self.drop[i[ok], j[ok]].max()),
                       float(self.shadow_drop[i[ok], j[ok]].max()))
            if d < 3.0:
                slope = max(slope, float(self.slope_deg[i[ok], j[ok]].max()))
            m = int(cl[np.argmax(_SEV_LUT[cl])])
            if m in LETHAL or m == POSSIBLE_DROP:
                if blocking is None:
                    blocking = (d, m)
            if m != UNKNOWN and SEVERITY[m] > SEVERITY[worst]:
                worst = m
        if blocking is not None:
            safe_dist = blocking[0]
        roll, pitch = pose.roll_pitch_deg
        return dict(terrain=CLASS_NAMES[worst] if seen_any else 'UNKNOWN', terrain_class=int(worst),
                    drop=round(drop, 3), slope_ahead=round(slope, 1), tilt=round(pose.tilt_deg, 1),
                    roll=round(roll, 1), pitch=round(pitch, 1), safe_distance=round(safe_dist, 2),
                    blocking=CLASS_NAMES[blocking[1]] if blocking else None)

    def cost_grid(self):
        """int8 occupancy (0..100, -1 unknown) row-major, origin bottom-left."""
        lut = np.zeros(256, np.int8)
        for k, v in CLASS_COST.items():
            lut[k] = v
        return lut[self.classes]
