#!/usr/bin/python3
"""Thermal human detection + tracking core (no ROS imports, unit-testable).

Pipeline per thermal frame (radiometric image, Kelvin per pixel):

  temperature image
    -> human temperature BAND  [temperature_threshold, human_max_temperature]
       (hotter things - engines 335 K, generators, warm-case decoys 314 K - are
       rejected, not accepted: "hot" is not "human")
    -> contrast test against the scene background (median of the frame)
    -> morphology (close small gaps between body parts) + connected components
    -> 3-D range of every blob:
         1. ray-march into the co-mounted depth camera          (best)
         2. intersect the blob's lowest ray with the ground plane (no depth)
         3. size prior (assumed_human_extent)                    (last resort)
    -> metric size / area / aspect-ratio filter (a 0.3 m warm case or a
       6 m warm wall fails even if its temperature is in the band)
    -> candidate with world x, y, z, distance, confidence

Temporal confirmation lives in HumanTracker: a candidate becomes a HUMAN only
after `confirmation_frames` consecutive associated frames, then keeps its id
(H1, H2 ...) forever - re-sightings update the same human, never add a new one.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from terrain_core import rpy_to_matrix


@dataclass
class ThermalParams:
    width: int = 320
    height: int = 240
    hfov: float = 1.0472
    mount_x: float = 0.52
    mount_y: float = 0.0
    camera_height: float = 1.0          # above ground
    pitch: float = 0.0                  # + = down
    ground_offset_z: float = -0.052     # ground relative to base_footprint
    detection_range: float = 30.0
    t_min: float = 296.5
    t_max: float = 311.0
    min_size: float = 0.25
    max_size: float = 2.3
    min_contrast: float = 1.5
    min_pixels: int = 12
    max_aspect: float = 6.0
    min_area: float = 0.04
    max_area: float = 2.0
    ray_step: float = 0.10
    assumed_extent: float = 1.0
    body_reference: float = 305.15
    reliable_range_no_depth: float = 12.0   # ground-plane range trusted only this close
    depth_far: float = 15.0             # depth camera far limit: a pixel AT this value is "no return"
    min_peak: float = 301.0          # a normal human has skin/torso >= 303 K somewhere in the blob
    covered_max_size: float = 1.9    # uniform lukewarm blobs (covered casualty) must be this small
    covered_max_temp: float = 299.5  # lukewarm above this = sun-warmed surface (27 C car body), not a covered person
    covered_max_area: float = 0.9

    @classmethod
    def from_cfg(cls, tc, hd, robot, camera=None):
        p = cls()
        if isinstance(camera, dict) and 'depth_far' in camera:
            p.depth_far = float(camera['depth_far'])
        m = {'width': (tc, 'resolution_width'), 'height': (tc, 'resolution_height'),
             'hfov': (tc, 'horizontal_fov'), 'mount_x': (tc, 'mount_x'), 'mount_y': (tc, 'mount_y'),
             'camera_height': (tc, 'camera_height'), 'pitch': (tc, 'camera_pitch'),
             'detection_range': (tc, 'detection_range'), 't_min': (tc, 'temperature_threshold'),
             't_max': (tc, 'human_max_temperature'), 'min_size': (tc, 'minimum_human_size'),
             'max_size': (hd, 'max_human_size'), 'min_contrast': (hd, 'min_contrast_k'),
             'min_pixels': (hd, 'min_pixels'), 'max_aspect': (hd, 'max_aspect_ratio'),
             'min_area': (hd, 'min_area_m2'), 'max_area': (hd, 'max_area_m2'),
             'ray_step': (hd, 'depth_ray_step'), 'assumed_extent': (hd, 'assumed_human_extent'),
             'reliable_range_no_depth': (hd, 'confirm_max_range_no_depth'),
             'min_peak': (hd, 'min_peak_temperature'), 'covered_max_size': (hd, 'covered_max_size'),
             'covered_max_area': (hd, 'covered_max_area'), 'covered_max_temp': (hd, 'covered_max_temperature'),
             'ground_offset_z': (robot, 'ground_offset_z')}
        for attr, (sec, key) in m.items():
            if isinstance(sec, dict) and key in sec:
                setattr(p, attr, type(getattr(p, attr))(sec[key]))
        return p


class PinholeCamera:
    """Pinhole + fixed extrinsic base_footprint -> optical frame (z fwd, x right, y down)."""

    def __init__(self, width, height, hfov, x, y, z, pitch, K=None):
        self.w, self.h = int(width), int(height)
        if K is not None and K[0] > 0:
            self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]
        else:
            self.fx = self.w / 2.0 / math.tan(hfov / 2.0)
            self.fy = self.fx
            self.cx, self.cy = (self.w - 1) / 2.0, (self.h - 1) / 2.0
        R_link = rpy_to_matrix(0.0, pitch, 0.0)
        R_link_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        self.R_fp_opt = R_link @ R_link_opt
        self.t_fp_opt = np.array([x, y, z], float)

    @property
    def vfov(self):
        return 2.0 * math.atan(self.h / 2.0 / self.fy)

    def world_transform(self, pose):
        R = pose.R @ self.R_fp_opt
        t = pose.R @ self.t_fp_opt + np.array([pose.x, pose.y, pose.z])
        return R, t

    def ray_world(self, u, v, pose):
        R, t = self.world_transform(pose)
        d = R @ np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        return t, d / np.linalg.norm(d)

    def project(self, pts_world, pose):
        R, t = self.world_transform(pose)
        pc = (np.atleast_2d(pts_world) - t) @ R
        z = pc[:, 2]
        zs = np.where(z > 1e-3, z, 1.0)
        return np.stack([self.fx * pc[:, 0] / zs + self.cx, self.fy * pc[:, 1] / zs + self.cy], 1), z


def thermal_camera_from(p, K=None):
    return PinholeCamera(p.width, p.height, p.hfov, p.mount_x, p.mount_y,
                         p.camera_height + p.ground_offset_z, p.pitch, K)


def raw_to_kelvin(raw, encoding, resolution=0.01):
    """Gazebo thermal camera: L16 = temperature / resolution (default 0.01 K);
    L8 = linear over [min_temp, max_temp] (not radiometric enough for SAR)."""
    a = raw.astype(np.float32)
    if encoding in ('mono16', '16uc1', 'l16'):
        return a * float(resolution)
    if encoding in ('32fc1',):
        return a
    raise ValueError(f'thermal encoding {encoding} is not radiometric; use L16 in the sensor')


@dataclass
class Candidate:
    u: float
    v: float
    bbox: tuple                 # x, y, w, h (pixels)
    pixels: int
    mean_temp: float
    peak_temp: float
    distance: float = float('nan')
    range_source: str = ''
    x: float = float('nan')
    y: float = float('nan')
    z: float = float('nan')
    size_w: float = 0.0
    size_h: float = 0.0
    area_m2: float = 0.0
    confidence: float = 0.0
    accepted: bool = False
    reject: str = ''
    reliable: bool = False      # 3-D position good enough to confirm / move a human marker

    def as_dict(self):
        d = {k: (round(v, 3) if isinstance(v, float) and math.isfinite(v) else v)
             for k, v in self.__dict__.items()}
        d['bbox'] = [int(b) for b in self.bbox]
        for k in ('distance', 'x', 'y', 'z'):
            if not (isinstance(d[k], (int, float)) and math.isfinite(d[k])):
                d[k] = None
        return d


class ThermalHumanDetector:
    def __init__(self, p, K=None):
        self.p = p
        self.cam = thermal_camera_from(p, K)

    def set_intrinsics(self, K, width, height):
        self.p.width, self.p.height = int(width), int(height)
        self.cam = thermal_camera_from(self.p, K)

    # ------------------------------------------------------------ 2-D stage
    def segment(self, temp):
        """Two temperature levels, so a person next to a sun-warmed car body never merges into
        one blob with it:
          core      band pixels >= min_peak (skin, clothed torso/limbs: 303-308 K)
          lukewarm  band pixels below min_peak (covered casualty 297.65 K, sun-loaded car
                    body 300 K). A lukewarm blob touching a core belongs to that person; one
                    touching an above-band (engine / equipment) region is part of that machine.
                    The rest is judged with the covered-casualty size limits."""
        p = self.p
        finite = np.isfinite(temp)
        bg = float(np.median(temp[finite])) if finite.any() else 0.0
        band = finite & (temp >= p.t_min) & (temp <= p.t_max) & (temp >= bg + p.min_contrast)
        core = band & (temp >= p.min_peak)
        hot = finite & (temp > p.t_max)
        k3 = np.ones((3, 3), np.uint8)
        out = []
        luke = band & ~core
        for level, mask in (('core', core), ('lukewarm', luke)):
            m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, k3, iterations=2)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
            n, lab, stats, cent = cv2.connectedComponentsWithStats(m, connectivity=8)
            for k in range(1, n):
                comp = lab == k
                sel = comp & mask
                if not sel.any():
                    continue
                reject = ''
                if level == 'lukewarm':
                    ring = cv2.dilate(comp.astype(np.uint8), k3, iterations=2).astype(bool)
                    if (ring & core).any():
                        continue        # clothing / edge pixels of a person: the core blob is the candidate
                    if (ring & hot).any():
                        reject = 'part of a hot object (engine / equipment)'
                vals = temp[sel]
                # size / centroid from the real in-band pixels, not the morphologically grown blob
                # (closing adds ~2 px per side, which would let a 0.15 m warm pipe pass as 0.3 m)
                ys, xs = np.nonzero(sel)
                x, y = int(xs.min()), int(ys.min())
                w, h = int(xs.max()) - x + 1, int(ys.max()) - y + 1
                out.append(Candidate(u=float(xs.mean()), v=float(ys.mean()), bbox=(x, y, w, h),
                                     pixels=int(sel.sum()), mean_temp=float(vals.mean()),
                                     peak_temp=float(vals.max()), reject=reject))
        return out, bg, band

    # ------------------------------------------------------------ 3-D stage
    def _range_from_depth(self, origin, ray, depth, depth_cam, pose):
        if depth is None or depth_cam is None:
            return None
        p = self.p
        d = np.arange(0.3, p.detection_range + 1e-6, p.ray_step)
        pts = origin + np.outer(d, ray)
        uv, zc = depth_cam.project(pts, pose)
        u = np.round(uv[:, 0]).astype(int)
        v = np.round(uv[:, 1]).astype(int)
        H, W = depth.shape[:2]
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H) & (zc > 0.2)
        if not ok.any():
            return None
        D = np.full(d.shape, np.nan, np.float32)
        D[ok] = depth[v[ok], u[ok]]
        # A depth pixel at the camera's far limit is NOT a surface: Gazebo writes the far value (15 m)
        # where nothing is in range. Taken as a surface, it put a person 26 m away at ~14.7 m - a
        # marker on empty ground. Such pixels are "no reading"; the ground-plane range takes over.
        valid = ok & np.isfinite(D) & (D > 0.2) & (D < p.depth_far - 0.3)
        # first sample whose depth-camera z reaches the measured surface
        hit = valid & (zc >= D - max(0.25, 2 * p.ray_step))
        if not hit.any():
            return None
        k = int(np.argmax(hit))
        return float(d[k])

    def _range_from_ground(self, cand, pose):
        """Lowest pixel row of the blob is where the human meets the ground (standing,
        sitting, lying): intersect that ray with the local ground plane."""
        x, y, w, h = cand.bbox
        o, r = self.cam.ray_world(cand.u, y + h - 0.5, pose)
        gz = pose.z + self.p.ground_offset_z
        if r[2] > -1e-3:
            return None
        t = (gz - o[2]) / r[2]
        if t <= 0 or t > self.p.detection_range * 1.5:
            return None
        # distance along the centroid ray with the same horizontal range
        o2, r2 = self.cam.ray_world(cand.u, cand.v, pose)
        hor = math.hypot(r[0] * t, r[1] * t)
        hor2 = math.hypot(r2[0], r2[1]) or 1e-6
        return hor / hor2

    def locate(self, cand, pose, depth=None, depth_cam=None):
        p = self.p
        o, r = self.cam.ray_world(cand.u, cand.v, pose)
        dist = self._range_from_depth(o, r, depth, depth_cam, pose)
        src = 'depth'
        if dist is None:
            dist = self._range_from_ground(cand, pose)
            src = 'ground'
        if dist is None:
            px = max(cand.bbox[2], cand.bbox[3])
            dist = p.assumed_extent * self.cam.fx / max(px, 1)
            src = 'size_prior'
        cand.distance, cand.range_source = float(dist), src
        P = o + r * dist
        cand.x, cand.y, cand.z = float(P[0]), float(P[1]), float(P[2])
        cand.size_w = cand.bbox[2] * dist / self.cam.fx
        cand.size_h = cand.bbox[3] * dist / self.cam.fy
        cand.area_m2 = cand.pixels * dist * dist / (self.cam.fx * self.cam.fy)
        return cand

    def classify(self, cand):
        p = self.p
        big, small = max(cand.size_w, cand.size_h), min(cand.size_w, cand.size_h)
        aspect = big / max(small, 1e-3)
        if cand.reject:
            pass
        elif cand.pixels < p.min_pixels:
            cand.reject = 'too few pixels'
        elif cand.distance > p.detection_range:
            cand.reject = f'beyond detection range ({cand.distance:.1f} m)'
        elif big < p.min_size:
            cand.reject = f'too small ({big:.2f} m)'
        elif big > p.max_size:
            cand.reject = f'too large ({big:.2f} m)'
        elif aspect > p.max_aspect:
            cand.reject = f'aspect {aspect:.1f}'
        elif not (p.min_area <= cand.area_m2 <= p.max_area):
            cand.reject = f'area {cand.area_m2:.2f} m2'
        elif cand.peak_temp < p.min_peak and cand.mean_temp > p.covered_max_temp:
            cand.reject = f'sun-warmed surface {cand.mean_temp - 273.15:.1f}C (between covered and bare human)'
        elif cand.peak_temp < p.min_peak and (big > p.covered_max_size or cand.area_m2 > p.covered_max_area):
            # lukewarm everywhere (no exposed skin / torso) AND bigger than a covered casualty:
            # a sun-loaded panel or car body seen end-on, not a person
            cand.reject = f'warm surface, peak {cand.peak_temp - 273.15:.1f}C'
        cand.accepted = cand.reject == ''
        # depth-ranged, or ground-ranged close by (far away the feet may be hidden by a sill /
        # debris and the ground-plane range is metres off -> such blobs stay CANDIDATES)
        cand.reliable = cand.accepted and (cand.range_source == 'depth' or (
            cand.range_source == 'ground' and cand.distance <= p.reliable_range_no_depth))
        # confidence: temperature closeness to body temp, size plausibility, pixel support, range source
        c_temp = 1.0 - min(1.0, abs(cand.mean_temp - p.body_reference) / max(1.0, p.t_max - p.t_min))
        c_size = 1.0 if 0.5 <= big <= 2.0 else 0.6
        c_px = min(1.0, cand.pixels / (4.0 * p.min_pixels))
        c_rng = {'depth': 1.0, 'ground': 0.8, 'size_prior': 0.5}.get(cand.range_source, 0.5)
        cand.confidence = round(float(0.35 * c_temp + 0.25 * c_size + 0.2 * c_px + 0.2 * c_rng), 3) \
            if cand.accepted else 0.0
        return cand

    def detect(self, temp, pose, depth=None, depth_cam=None):
        cands, bg, band = self.segment(temp)
        for c in cands:
            self.locate(c, pose, depth, depth_cam)
            self.classify(c)
        return cands, bg, band


# ====================================================================== tracking
@dataclass
class Track:
    tid: int
    x: float
    y: float
    z: float
    hits: int = 1
    misses: int = 0
    confirmed: bool = False
    human_id: str = ''
    confidence: float = 0.0
    distance: float = 0.0
    first_t: float = 0.0
    last_t: float = 0.0
    confirmed_t: float = 0.0
    sightings: int = 1
    mean_temp: float = 0.0
    absorbed: bool = False

    def as_dict(self):
        return dict(human_id=self.human_id, track=self.tid, x=round(self.x, 2), y=round(self.y, 2),
                    z=round(self.z, 2), distance=round(self.distance, 2), confidence=round(self.confidence, 3),
                    confirmed=self.confirmed, hits=self.hits, sightings=self.sightings,
                    first_seen=round(self.first_t, 1), detected_at=round(self.confirmed_t, 1),
                    last_seen=round(self.last_t, 1), temperature=round(self.mean_temp, 2))


class HumanTracker:
    """World-frame nearest-neighbour tracker with N-frame confirmation and permanent ids."""

    def __init__(self, confirm_frames=4, miss_frames=3, gate=1.8, merge_distance=2.0, alpha=0.3):
        self.confirm_frames = int(confirm_frames)
        self.miss_frames = int(miss_frames)
        self.gate = float(gate)
        self.merge = float(merge_distance)
        self.alpha = float(alpha)
        self.tracks = []
        self.next_tid = 1
        self.next_hid = 1
        self.events = []            # (t, text) new confirmations, consumed by the node

    @property
    def humans(self):
        return [t for t in self.tracks if t.confirmed]

    @property
    def tentative(self):
        return [t for t in self.tracks if not t.confirmed]

    def update(self, detections, t):
        """detections: list of dicts/objects with x, y, z, confidence, distance, mean_temp (accepted only)."""
        dets = [d if isinstance(d, dict) else d.__dict__ for d in detections]
        used = set()
        # greedy association, closest pairs first
        pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, d in enumerate(dets):
                dist = math.hypot(tr.x - d['x'], tr.y - d['y'])
                gate = max(self.gate, self.merge) if tr.confirmed else self.gate
                if dist <= gate:
                    pairs.append((dist, ti, di))
        pairs.sort()
        matched_t = set()
        for dist, ti, di in pairs:
            if ti in matched_t or di in used:
                continue
            matched_t.add(ti)
            used.add(di)
            tr, d = self.tracks[ti], dets[di]
            reliable = bool(d.get('reliable', True))
            tr.sightings += 1
            tr.misses = 0
            tr.last_t = t
            if tr.confirmed and not reliable:
                continue                    # far / uncertain range: never drags a confirmed marker
            a = self.alpha if tr.confirmed else 0.5
            tr.x += a * (d['x'] - tr.x)
            tr.y += a * (d['y'] - tr.y)
            tr.z += a * (d['z'] - tr.z)
            tr.distance = d.get('distance', tr.distance)
            tr.mean_temp = d.get('mean_temp', tr.mean_temp)
            if tr.confirmed:
                tr.confidence = max(tr.confidence, float(d.get('confidence', 0.0)))
                continue
            tr.confidence = float(d.get('confidence', 0.0))
            if reliable:                    # only well-ranged frames count towards HUMAN DETECTED
                tr.hits += 1
                if tr.hits >= self.confirm_frames:
                    self._confirm(tr, t)
        for ti, tr in enumerate(self.tracks):
            if ti not in matched_t:
                tr.misses += 1
                if not tr.confirmed:
                    tr.hits = 0 if tr.misses > self.miss_frames else tr.hits
        # drop stale tentative tracks (confirmed humans are kept for ever)
        self.tracks = [tr for tr in self.tracks if not tr.absorbed and (tr.confirmed or tr.misses <= self.miss_frames)]
        for di, d in enumerate(dets):
            if di in used:
                continue
            self.tracks.append(Track(tid=self.next_tid, x=d['x'], y=d['y'], z=d['z'],
                                     hits=1 if d.get('reliable', True) else 0,
                                     confidence=float(d.get('confidence', 0.0)),
                                     distance=d.get('distance', 0.0), first_t=t, last_t=t,
                                     mean_temp=d.get('mean_temp', 0.0)))
            self.next_tid += 1
            if self.confirm_frames <= 1 and self.tracks[-1].hits >= 1:
                self._confirm(self.tracks[-1], t)
        self.tracks = [tr for tr in self.tracks if not tr.absorbed]
        return self.humans

    def _confirm(self, tr, t):
        # the same human seen again (e.g. from another search point): reuse its id
        for h in self.humans:
            if h is not tr and math.hypot(h.x - tr.x, h.y - tr.y) <= self.merge:
                h.sightings += tr.hits
                h.last_t = t
                h.misses = 0
                tr.absorbed = True          # merged into h; removed at the end of update()
                return
        tr.confirmed = True
        tr.confirmed_t = t
        tr.human_id = f'H{self.next_hid}'
        self.next_hid += 1
        self.events.append((t, f'HUMAN {tr.human_id} DETECTED at ({tr.x:.1f}, {tr.y:.1f})'))
