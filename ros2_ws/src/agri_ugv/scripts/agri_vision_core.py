#!/usr/bin/python3
"""Agriculture perception core: people, main crop, weeds, unknown vegetation and other objects.

Pure numpy / OpenCV (no ROS) so it can be tested offline. agri_vision.py feeds it the existing
RGB-D camera, the camera pose and the robot pose; it never commands the robot.

Crop protection (the most important rule)
-----------------------------------------
Vegetation pixels are first found by colour (normalised excess green + hue; brightness-independent).
That only says "plant". WHICH plant is decided in 3D, using the known row geometry of the field:

  * every plant pixel is placed in the world with the depth image and the robot pose,
  * its lateral offset from the nearest crop-row line is measured,
  * |offset| <= row_position_tolerance       -> ROW BAND   -> CROP (or UNKNOWN if weak). Never WEED.
  * tolerance < |offset| < tolerance + guard -> GUARD BAND -> UNKNOWN. Never WEED.
  * beyond the guard band (between the rows) -> weed CANDIDATE, which becomes WEED only if ALL hold:
        rooted at the ground (not crop leaves hanging over the aisle), not touching the crop canopy,
        size and height inside the weed limits, not looking like the learned main crop,
        enough depth points, confidence >= weed_confidence_threshold,
        and (in the tracker) seen as a weed in temporal_confirmation_frames frames with no crop vote.
    Any failure -> UNKNOWN. Without depth or pose nothing is ever called a weed.

The tracker (Tracker) adds the last rule: a plant that was ever CROP can never become WEED; a
WEED that later gets a crop vote is downgraded to UNKNOWN.
"""
import math
from copy import deepcopy

import cv2
import numpy as np

CROP, WEED, HUMAN, UNKNOWN, OTHER = 'CROP', 'WEED', 'HUMAN', 'UNKNOWN', 'OTHER_OBJECT'
CLASSES = (CROP, WEED, HUMAN, UNKNOWN, OTHER)
# overlay colours, BGR
COLORS = {CROP: (80, 200, 60), WEED: (40, 40, 255), HUMAN: (255, 60, 255), UNKNOWN: (0, 200, 255),
          OTHER: (255, 170, 40)}

DEFAULTS = {
    'field': dict(first_row_y=-13.418, row_pitch=1.22, row_count=23, row_x_min=-13.9, row_x_max=14.4,
                  field_margin=0.8),
    'camera': dict(x=0.4672, y=0.0, z=0.5991, pitch_down_deg=10.0, min_depth=0.25, max_depth=12.0),
    'performance': dict(process_rate_hz=4.0, overlay_width=640),
    'vegetation': dict(exg_min=0.10, hue_min=25, hue_max=95, saturation_min=40, value_min=18),
    'crop': dict(row_position_tolerance=0.30, guard_band=0.12, plant_spacing=0.30, plant_gap=0.12,
                 min_height=0.05, crop_confidence_threshold=0.60, count_range=6.0, min_points=12),
    'weed': dict(weed_confidence_threshold=0.65, minimum_weed_size=0.04, maximum_weed_size=0.60,
                 max_height=0.45, root_max_height=0.10, min_pixels=6, max_range=7.0,
                 crop_similarity_max=0.75, temporal_confirmation_frames=3),
    'human': dict(min_height=1.3, max_height=2.2, min_width=0.28, max_width=1.30, min_aspect=1.8,
                  max_vegetation_fraction=0.35, min_pixels=25, confidence_threshold=0.55,
                  temporal_confirmation_frames=4, active_seconds=6.0),
    'objects': dict(crop_max_height=1.0, min_height=0.15, max_range=12.0, cell=0.25, min_pixels=15, temporal_confirmation_frames=3),
    'tracking': dict(crop_radius=0.18, weed_radius=0.30, unknown_radius=0.30, human_radius=1.2,
                     object_radius=1.0, max_tracks=30000),
    'dashboard': dict(port=8080, jpeg_quality=80, max_events=200),
}


def load_config(data=None):
    """Defaults, overridden section by section by a dict (the parsed agri_vision.yaml)."""
    cfg = deepcopy(DEFAULTS)
    for sec, vals in (data or {}).items():
        if isinstance(vals, dict):
            cfg.setdefault(sec, {}).update(vals)
    return cfg


# ----------------------------------------------------------------------------------- geometry
class Field:
    """Crop-row lines of the field (rows parallel to world x)."""

    def __init__(self, c):
        self.y0, self.pitch, self.n = float(c['first_row_y']), float(c['row_pitch']), int(c['row_count'])
        self.x0, self.x1, self.margin = float(c['row_x_min']), float(c['row_x_max']), float(c['field_margin'])

    def row_index(self, y):
        return np.clip(np.round((np.asarray(y) - self.y0) / self.pitch), 0, self.n - 1).astype(int)

    def row_y(self, k):
        return self.y0 + np.asarray(k) * self.pitch

    def offset(self, x, y):
        """(row index 0-based, signed lateral offset from that row line)"""
        k = self.row_index(y)
        return k, np.asarray(y) - self.row_y(k)

    def in_field(self, x, y, margin=None):
        m = self.margin if margin is None else margin
        x, y = np.asarray(x), np.asarray(y)
        return ((x >= self.x0 - m) & (x <= self.x1 + m) &
                (y >= self.y0 - self.pitch / 2 - m) & (y <= self.row_y(self.n - 1) + self.pitch / 2 + m))


def rot_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def default_base_to_optical(c):
    """base_footprint -> camera_optical_frame from the config (used when TF is not available)."""
    T = np.eye(4)
    R_link = rot_rpy(0.0, math.radians(c['pitch_down_deg']), 0.0)          # +pitch = nose down
    R_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)          # optical: z fwd, x right, y down
    T[:3, :3] = R_link @ R_opt
    T[:3, 3] = [c['x'], c['y'], c['z']]
    return T


def robot_matrix(x, y, z, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


# ----------------------------------------------------------------------------------- vision
class Frame:
    """One synchronized RGB-D frame. bgr uint8 HxWx3, depth float32 metres (z along the optical axis),
    K 3x3, T_world_cam 4x4 (optical frame -> world), robot = (x, y, yaw), t = seconds."""

    def __init__(self, bgr, depth, K, T_world_cam, robot, t, robot_z=0.0):
        self.bgr, self.depth, self.K, self.T, self.robot, self.t = bgr, depth, np.asarray(K, float), T_world_cam, robot, t
        self.robot_z = robot_z          # base_footprint height (ground under the robot)


def vegetation_mask(bgr, v):
    """Pixel-level plant segmentation (brightness-independent). Not a classifier."""
    f = bgr.astype(np.float32)
    b, g, r = f[..., 0], f[..., 1], f[..., 2]
    s = b + g + r + 1e-3
    exg = (2 * g - r - b) / s
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return ((exg > v['exg_min']) & (h >= v['hue_min']) & (h <= v['hue_max']) &
            (sat >= v['saturation_min']) & (val >= v['value_min'])), exg


def fit_ground(P, rng, iters=40, tol=0.05):
    """RANSAC plane z = a*x + b*y + c through world points P (N x 3). Returns (a, b, c) or None."""
    if len(P) < 30:
        return None
    best, best_n = None, 0
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        A = np.c_[s[:, 0], s[:, 1], np.ones(3)]
        try:
            coef = np.linalg.solve(A, s[:, 2])
        except np.linalg.LinAlgError:
            continue
        if abs(coef[0]) > 0.35 or abs(coef[1]) > 0.35:        # steeper than ~19 deg: not the field floor
            continue
        n = int((np.abs(P[:, 0] * coef[0] + P[:, 1] * coef[1] + coef[2] - P[:, 2]) < tol).sum())
        if n > best_n:
            best, best_n = coef, n
    if best is None or best_n < 20:
        return None
    inl = np.abs(P[:, 0] * best[0] + P[:, 1] * best[1] + best[2] - P[:, 2]) < tol
    A = np.c_[P[inl, 0], P[inl, 1], np.ones(inl.sum())]
    return np.linalg.lstsq(A, P[inl, 2], rcond=None)[0]


def _bbox(us, vs):
    return [int(us.min()), int(vs.min()), int(us.max()) + 1, int(vs.max()) + 1]


class Detector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.field = Field(cfg['field'])
        self.rng = np.random.default_rng(3)
        self.ground = None
        # learned main-crop appearance: chromaticity (r, g) and typical height, from confident crop
        self.crop_chroma = None
        self.crop_height = None
        self.crop_samples = 0

    # ------------------------------------------------------------------ helpers
    def _similar_to_crop(self, chroma, height):
        """0..1 how much a plant looks like the learned main crop (colour and height)."""
        if self.crop_chroma is None or self.crop_samples < 5:
            return 0.0
        d = float(np.linalg.norm(np.asarray(chroma) - self.crop_chroma))
        color = math.exp(-(d / 0.035) ** 2)
        hs = min(1.0, height / max(self.crop_height, 1e-3)) if self.crop_height else 0.0
        hs = 1.0 if hs > 0.7 else hs / 0.7
        return color * hs

    def _learn_crop(self, chroma, height):
        a = 0.1 if self.crop_samples > 20 else 1.0 / (self.crop_samples + 1)
        chroma = np.asarray(chroma, float)
        self.crop_chroma = chroma if self.crop_chroma is None else (1 - a) * self.crop_chroma + a * chroma
        self.crop_height = height if self.crop_height is None else (1 - a) * self.crop_height + a * height
        self.crop_samples += 1

    # ------------------------------------------------------------------ main
    def process(self, fr):
        c = self.cfg
        H, W = fr.depth.shape
        K = fr.K
        vv, uu = np.mgrid[0:H, 0:W]
        d = fr.depth
        valid = np.isfinite(d) & (d > c['camera']['min_depth']) & (d < c['camera']['max_depth'])
        rays = np.stack(((uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(d)), -1)
        Pc = rays * np.where(valid, d, 0)[..., None]
        R, t = fr.T[:3, :3], fr.T[:3, 3]
        Pw = Pc @ R.T + t                                       # H x W x 3 world points
        cam = t
        rng_xy = np.hypot(Pw[..., 0] - cam[0], Pw[..., 1] - cam[1])

        veg, exg = vegetation_mask(fr.bgr, c['vegetation'])

        # ---- ground (soil) height under every pixel
        soil = valid & ~veg & (rng_xy < 8.0) & (Pw[..., 2] < fr.robot_z + 0.35)
        P = Pw[soil]
        if len(P) > 600:
            P = P[self.rng.choice(len(P), 600, replace=False)]
        g = fit_ground(P, self.rng)
        if g is not None:
            self.ground = g
        if self.ground is None:
            self.ground = np.array([0.0, 0.0, fr.robot_z])
        a, b, c0 = self.ground
        hgt = Pw[..., 2] - (a * Pw[..., 0] + b * Pw[..., 1] + c0)

        # ---- row geometry for every valid pixel
        rowk, off = self.field.offset(Pw[..., 0], Pw[..., 1])
        infield = self.field.in_field(Pw[..., 0], Pw[..., 1]) & valid
        tol, guard = c['crop']['row_position_tolerance'], c['crop']['guard_band']
        aoff = np.abs(off)
        band = veg & infield & (aoff <= tol)
        guardm = veg & infield & (aoff > tol) & (aoff < tol + guard)
        inter = veg & infield & (aoff >= tol + guard)
        outside = veg & valid & ~infield

        f = fr.bgr.astype(np.float32)
        ssum = f.sum(-1) + 1e-3
        chroma = np.stack((f[..., 2] / ssum, f[..., 1] / ssum), -1)   # (r, g) chromaticity

        dets = []
        dets += self._crops(band, Pw, hgt, rowk, off, rng_xy, chroma, uu, vv, fr)
        dets += self._weeds(inter, band, guardm, Pw, hgt, off, rng_xy, chroma, uu, vv, fr)
        dets += self._guard(guardm, inter, Pw, hgt, rng_xy, uu, vv, fr)
        dets += self._outside(outside, Pw, hgt, rng_xy, uu, vv, fr)
        # non-green parts of the crop (cotton bolls, stems) on the row lines are crop, not objects
        crop_parts = infield & (aoff < tol + guard) & (hgt < c['objects'].get('crop_max_height', 1.0))
        dets += self._objects(valid & ~veg & ~crop_parts, veg, valid, Pw, hgt, rng_xy, fr, uu, vv)
        masks = dict(band=band, guard=guardm, inter=inter, veg=veg)
        stats = dict(valid_px=int(valid.sum()), veg_px=int(veg.sum()), band_px=int(band.sum()),
                     inter_px=int(inter.sum()), ground=[round(float(x), 4) for x in self.ground],
                     crop_model=None if self.crop_chroma is None else
                     dict(chroma=[round(float(x), 3) for x in self.crop_chroma],
                          height=round(float(self.crop_height), 3), samples=self.crop_samples))
        return dets, masks, stats

    def _rel(self, x, y, fr):
        rx, ry, yaw = fr.robot
        dx, dy = x - rx, y - ry
        return round(math.hypot(dx, dy), 2), round(math.degrees(math.atan2(dy, dx) - yaw + math.pi) % 360 - 180, 1)

    def _det(self, cls, sub, conf, x, y, bbox, fr, **kw):
        dist, bearing = self._rel(x, y, fr)
        d = dict(cls=cls, sub=sub, conf=round(float(conf), 3), x=round(float(x), 3), y=round(float(y), 3),
                 dist=dist, bearing=bearing, bbox=bbox, t=fr.t)
        d.update(kw)
        return d

    # ---- CROP: the row band, split into plants along the row
    def _crops(self, band, Pw, hgt, rowk, off, rng_xy, chroma, uu, vv, fr):
        c = self.cfg['crop']
        out = []
        sel = band & (rng_xy < max(c['count_range'], 1.0))
        if sel.sum() < c['min_points']:
            return out
        X, Y, Hh, K, O = Pw[..., 0][sel], Pw[..., 1][sel], hgt[sel], rowk[sel], off[sel]
        U, V, CH = uu[sel], vv[sel], chroma[sel]
        for k in np.unique(K):
            m = K == k
            order = np.argsort(X[m])
            xs = X[m][order]
            idx = np.flatnonzero(m)[order]
            # split at along-row gaps, then long canopies by the plant spacing
            cuts = np.flatnonzero(np.diff(xs) > c['plant_gap']) + 1
            for seg in np.split(np.arange(len(xs)), cuts):
                if len(seg) < c['min_points']:
                    continue
                length = xs[seg[-1]] - xs[seg[0]]
                n = max(1, int(round(length / c['plant_spacing'])))
                for part in np.array_split(seg, n):
                    if len(part) < c['min_points']:
                        continue
                    ii = idx[part]
                    h95 = float(np.percentile(Hh[ii], 95))
                    align = 1.0 - min(1.0, float(np.median(np.abs(O[ii]))) / c['row_position_tolerance'])
                    tall = min(1.0, h95 / max(c['min_height'], 1e-3))
                    dense = min(1.0, len(ii) / (3.0 * c['min_points']))
                    conf = 0.5 * align + 0.3 * tall + 0.2 * dense
                    cx, cy = float(np.median(X[ii])), float(self.field.row_y(k))
                    ch = CH[ii].mean(0)
                    cls = CROP if (conf >= c['crop_confidence_threshold'] and h95 >= c['min_height']) else UNKNOWN
                    if cls == CROP and conf >= 0.8 and rng_xy[sel][ii].mean() < 5.0:
                        self._learn_crop(ch, h95)
                    out.append(self._det(cls, 'crop' if cls == CROP else 'row-band plant (weak)', conf, cx, cy,
                                         _bbox(U[ii], V[ii]), fr, row=int(k) + 1, height=round(h95, 3),
                                         reason='on crop row line' if cls == CROP else 'on the row line but weak evidence'))
        return out

    # ---- WEED candidates between the rows
    def _weeds(self, inter, band, guardm, Pw, hgt, off, rng_xy, chroma, uu, vv, fr):
        c, cc = self.cfg['weed'], self.cfg['crop']
        out = []
        n, lab, st, _ = cv2.connectedComponentsWithStats(inter.astype(np.uint8), connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] < c['min_pixels']:
                continue
            m = lab == i
            if rng_xy[m].min() > c['max_range']:
                continue
            X, Y, Hh = Pw[..., 0][m], Pw[..., 1][m], hgt[m]
            U, V = uu[m], vv[m]
            h_top, h_low = float(np.percentile(Hh, 95)), float(np.percentile(Hh, 5))
            size = float(max(np.ptp(X), np.ptp(Y), 0.0))
            min_off = float(np.abs(off[m]).min())
            ch = chroma[m].mean(0)
            sim = self._similar_to_crop(ch, h_top)
            touches = self._touches(m, band, Pw)
            reaches_row = self._touches(m, guardm, Pw)
            reasons = []
            if touches:
                reasons.append('touches crop canopy')
            elif reaches_row:
                reasons.append('extends into the guard band next to the crop row')
            if h_low > c['root_max_height']:
                reasons.append('not rooted (leaves above the ground)')
            if size < c['minimum_weed_size']:
                reasons.append('smaller than minimum_weed_size')
            if size > c['maximum_weed_size']:
                reasons.append('larger than maximum_weed_size')
            if h_top > c['max_height']:
                reasons.append('taller than weed max_height')
            if sim > c['crop_similarity_max']:
                reasons.append('looks like the main crop')
            far = min(1.0, (min_off - (cc['row_position_tolerance'] + cc['guard_band'])) / 0.10)
            size_ok = 1.0 if c['minimum_weed_size'] <= size <= c['maximum_weed_size'] else 0.0
            rooted = 1.0 if h_low <= c['root_max_height'] else 0.0
            conf = 0.3 * (0.5 + 0.5 * max(0.0, far)) + 0.25 * size_ok + 0.2 * rooted + 0.25 * (1 - sim)
            if conf < c['weed_confidence_threshold']:
                reasons.append(f'confidence {conf:.2f} < {c["weed_confidence_threshold"]}')
            cls = WEED if not reasons else UNKNOWN
            x, y = float(np.median(X)), float(np.median(Y))
            k, o = self.field.offset(x, y)
            out.append(self._det(cls, 'weed' if cls == WEED else 'plant between rows', conf, x, y, _bbox(U, V), fr,
                                 row_offset=round(float(o), 3), height=round(h_top, 3), size=round(size, 3),
                                 crop_similarity=round(sim, 3),
                                 reason='between rows, rooted, weed-sized, unlike crop' if cls == WEED else '; '.join(reasons)))
        return out

    @staticmethod
    def _touches(m, crop, Pw, gap=0.08):
        """True if the plant component m is physically joined (in 3D, not just next to it in the
        image) to crop-row vegetation: e.g. crop leaves reaching into the aisle."""
        ring = cv2.dilate(m.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool) & crop
        if not ring.any():
            return False
        A = Pw[ring]
        B = Pw[cv2.dilate(ring.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool) & m]
        if not len(B):
            return False
        d = np.sqrt(((A[:, None, :2] - B[None, :, :2]) ** 2).sum(-1) + (A[:, None, 2] - B[None, :, 2]) ** 2)
        return bool(d.min() < gap)

    def _guard(self, guardm, inter, Pw, hgt, rng_xy, uu, vv, fr):
        out = []
        # guard-band pixels that are the edge of a plant rooted between the rows belong to that plant
        inter_near = cv2.dilate(inter.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        n, lab, st, _ = cv2.connectedComponentsWithStats(guardm.astype(np.uint8), connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] < max(8, self.cfg['weed']['min_pixels']):
                continue
            m = lab == i
            if rng_xy[m].min() > self.cfg['weed']['max_range'] or (m & inter_near).any():
                continue
            x, y = float(np.median(Pw[..., 0][m])), float(np.median(Pw[..., 1][m]))
            out.append(self._det(UNKNOWN, 'plant at row edge', 0.5, x, y, _bbox(uu[m], vv[m]), fr,
                                 height=round(float(np.percentile(hgt[m], 95)), 3),
                                 reason='inside the guard band next to the crop row'))
        return out

    def _outside(self, outside, Pw, hgt, rng_xy, uu, vv, fr):
        """Vegetation outside the planted block: trees and bushes (grass is ignored)."""
        out = []
        n, lab, st, _ = cv2.connectedComponentsWithStats(outside.astype(np.uint8), connectivity=8)
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] < 30:
                continue
            m = lab == i
            h = float(np.percentile(hgt[m], 95))
            X, Y = Pw[..., 0][m], Pw[..., 1][m]
            size = float(max(np.ptp(X), np.ptp(Y)))
            if h > 2.0:
                sub = 'tree'
            elif h > 0.5 and size > 0.5:
                sub = 'bush'
            else:
                continue
            out.append(self._det(OTHER, sub, 0.7, float(np.median(X)), float(np.median(Y)),
                                 _bbox(uu[m], vv[m]), fr, height=round(h, 2), size=round(size, 2), reason='vegetation outside the crop rows'))
        return out

    # ---- people and other non-vegetation objects
    def _objects(self, nonveg, veg, valid, Pw, hgt, rng_xy, fr, uu, vv):
        c, hc = self.cfg['objects'], self.cfg['human']
        out = []
        m = nonveg & (hgt > c['min_height']) & (rng_xy < c['max_range'])
        if m.sum() < c['min_pixels']:
            return out
        X, Y = Pw[..., 0][m], Pw[..., 1][m]
        cell = c['cell']
        ix = np.floor(X / cell).astype(int)
        iy = np.floor(Y / cell).astype(int)
        ox, oy = ix.min(), iy.min()
        grid = np.zeros((iy.max() - oy + 3, ix.max() - ox + 3), np.uint8)
        grid[iy - oy + 1, ix - ox + 1] = 1
        n, lab = cv2.connectedComponents(grid, connectivity=8)
        cl = lab[iy - oy + 1, ix - ox + 1]
        H_, U_, V_ = hgt[m], uu[m], vv[m]
        vegP = Pw[veg & valid]
        vegH = hgt[veg & valid]
        Himg = veg.shape[0]
        for i in range(1, n):
            s = cl == i
            if s.sum() < c['min_pixels']:
                continue
            xs, ys, hs = X[s], Y[s], H_[s]
            us, vs = U_[s], V_[s]
            top = float(np.percentile(hs, 97))
            width = float(max(np.ptp(xs), np.ptp(ys), 0.05))
            cx, cy = float(np.median(xs)), float(np.median(ys))
            bbox = _bbox(us, vs)
            cut_top = bbox[1] <= 1                                   # person taller than the view
            box = veg[bbox[1]:bbox[3], bbox[0]:bbox[2]]
            veg_frac = float(box.mean()) if box.size else 0.0
            canopy = bool(len(vegP) and ((np.hypot(vegP[:, 0] - cx, vegP[:, 1] - cy) < 1.5) &
                                         (vegH > max(1.8, top))).sum() > 30)
            aspect = top / width
            hum_reasons = []
            if not (hc['min_height'] <= top <= hc['max_height'] or (cut_top and top >= hc['min_height'] * 0.8)):
                hum_reasons.append('height')
            if not (hc['min_width'] <= width <= hc['max_width']):
                hum_reasons.append('width')
            if aspect < hc['min_aspect'] and not cut_top:
                hum_reasons.append('aspect')
            if veg_frac > hc['max_vegetation_fraction']:
                hum_reasons.append('vegetation')
            if canopy:
                hum_reasons.append('tree canopy above (trunk)')
            if s.sum() < hc['min_pixels']:
                hum_reasons.append('pixels')
            if not hum_reasons:
                conf = (0.35 + 0.25 * min(1.0, aspect / 2.5) + 0.2 * (1 - veg_frac) +
                        0.2 * min(1.0, s.sum() / (4.0 * hc['min_pixels'])))
                if conf >= hc['confidence_threshold']:
                    out.append(self._det(HUMAN, 'person', conf, cx, cy, bbox, fr, height=round(top, 2),
                                         width=round(width, 2), reason='upright, person-sized, not vegetation'))
                    continue
            # other objects (only what the geometry supports)
            if canopy:
                sub = 'tree'
            elif width > 3.0 and top > 2.0:
                sub = 'building'
            elif width > 1.4 and 1.0 < top < 4.0:
                sub = 'vehicle'
            elif width < 0.22 and top > 0.8:
                sub = 'pole' if top > 2.5 else 'fence post'
            elif top < 0.8:
                pix = fr.bgr[vs, us]
                hsv = cv2.cvtColor(pix.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
                sub = 'rock' if np.median(hsv[:, 1]) < 70 else 'equipment'
            else:
                sub = 'obstacle'
            out.append(self._det(OTHER, sub, 0.6, cx, cy, bbox, fr, height=round(top, 2), width=round(width, 2),
                                 reason='non-vegetation ' + sub))
        return out


# ----------------------------------------------------------------------------------- tracking
class Track:
    __slots__ = ('family', 'x', 'y', 'votes', 'streak', 'ever_crop', 'cls', 'id', 'first', 'last', 'conf',
                 'seen', 'sub', 'info', 'announced')

    def __init__(self, family, d):
        self.family, self.x, self.y = family, d['x'], d['y']
        self.votes = {k: 0 for k in CLASSES}
        self.streak, self.ever_crop, self.cls, self.id = 0, False, None, None
        self.first = self.last = d['t']
        self.conf, self.seen, self.sub, self.info, self.announced = 0.0, 0, d['sub'], {}, False


def family_of(cls):
    return {CROP: 'plant', WEED: 'plant', UNKNOWN: 'plant', HUMAN: 'human', OTHER: 'object'}[cls]


class Tracker:
    """World-frame de-duplication + temporal confirmation + the no-CROP->WEED latch."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = []
        self.next_id = {CROP: 1, WEED: 1, HUMAN: 1, UNKNOWN: 1, OTHER: 1}
        self.prefix = {CROP: 'C', WEED: 'W', HUMAN: 'H', UNKNOWN: 'U', OTHER: 'O'}

    def _radius(self, d):
        t = self.cfg['tracking']
        return {CROP: t['crop_radius'], WEED: t['weed_radius'], UNKNOWN: t['unknown_radius'],
                HUMAN: t['human_radius'], OTHER: t['object_radius']}[d['cls']]

    def _match(self, d):
        fam, r = family_of(d['cls']), self._radius(d)
        best, bd = None, r
        for tr in self.tracks:
            if tr.family != fam:
                continue
            dist = math.hypot(tr.x - d['x'], tr.y - d['y'])
            if dist < bd:
                best, bd = tr, dist
        return best

    def update(self, dets, t):
        """Returns a list of events (dicts)."""
        events = []
        need = {WEED: self.cfg['weed']['temporal_confirmation_frames'],
                HUMAN: self.cfg['human']['temporal_confirmation_frames'],
                OTHER: self.cfg['objects']['temporal_confirmation_frames'],
                CROP: 2, UNKNOWN: 3}
        seen_now = set()
        for d in dets:
            tr = self._match(d)
            if tr is None:
                if len(self.tracks) >= self.cfg['tracking']['max_tracks']:
                    continue
                tr = Track(family_of(d['cls']), d)
                self.tracks.append(tr)
            if id(tr) in seen_now and d['cls'] == WEED:
                continue
            seen_now.add(id(tr))
            a = 0.3
            tr.x, tr.y = (1 - a) * tr.x + a * d['x'], (1 - a) * tr.y + a * d['y']
            tr.last, tr.seen = t, tr.seen + 1
            tr.votes[d['cls']] += 1
            tr.conf = max(tr.conf * 0.9, d['conf'])
            tr.info = {k: d[k] for k in ('dist', 'bearing', 'height', 'reason', 'row', 'size') if k in d}
            if d['cls'] == CROP:
                tr.ever_crop = True
            tr.streak = tr.streak + 1 if d['cls'] == WEED else 0
            tr.sub = d['sub']
            old = tr.cls
            tr.cls = self._decide(tr, need)
            if tr.cls is not None and (tr.id is None or not tr.id.startswith(self.prefix[tr.cls])):
                # every class has its own numbering (an UNKNOWN that is confirmed as a weed gets a W id)
                tr.id = f'{self.prefix[tr.cls]}{self.next_id[tr.cls]}'
                self.next_id[tr.cls] += 1
            if tr.cls is not None and old != tr.cls:
                events.append(self._event(tr, old, t))
        return events

    def _decide(self, tr, need):
        v = tr.votes
        if tr.family == 'plant':
            if tr.ever_crop:
                # a plant that was ever seen as CROP can never be a weed
                if v[CROP] >= need[CROP] or tr.cls == CROP:
                    return CROP
                return UNKNOWN if tr.cls in (WEED, UNKNOWN) else None
            if tr.cls == WEED:
                return WEED
            if tr.streak >= need[WEED]:
                return WEED
            if v[UNKNOWN] + v[WEED] >= need[UNKNOWN]:
                return UNKNOWN
            return tr.cls
        if tr.family == 'human':
            return HUMAN if v[HUMAN] >= need[HUMAN] else None
        return OTHER if v[OTHER] >= need[OTHER] else None

    def _event(self, tr, old, t):
        if old == WEED and tr.cls != WEED:
            msg = f'{tr.id} reclassified {tr.cls} (crop protection)'
        elif tr.cls == WEED:
            msg = f'WEED {tr.id} DETECTED'
        elif tr.cls == HUMAN:
            msg = f'HUMAN {tr.id} DETECTED'
        elif tr.cls == OTHER:
            msg = f'{tr.sub.upper()} {tr.id} DETECTED'
        else:
            msg = f'{tr.cls} {tr.id}'
        return dict(t=t, kind=tr.cls, id=tr.id, msg=msg, x=round(tr.x, 2), y=round(tr.y, 2), conf=round(tr.conf, 2),
                    quiet=tr.cls in (CROP, UNKNOWN) and old != WEED)

    def summary(self, t, active_s):
        counts = {k: 0 for k in CLASSES}
        items = []
        for tr in self.tracks:
            if tr.cls is None:
                continue
            counts[tr.cls] += 1
            if tr.cls == CROP:
                continue                     # thousands of plants: counted, not listed
            items.append(dict(id=tr.id, cls=tr.cls, sub=tr.sub, x=round(tr.x, 2), y=round(tr.y, 2),
                              conf=round(tr.conf, 2), first=round(tr.first, 1), last=round(tr.last, 1),
                              active=(t - tr.last) < active_s, **tr.info))
        crops = [(round(tr.x, 2), round(tr.y, 2)) for tr in self.tracks if tr.cls == CROP]
        return counts, items, crops


# ----------------------------------------------------------------------------------- overlay
def draw_overlay(bgr, dets, masks, width, info):
    """Camera image with class tints, boxes, labels, row line, heading and banners."""
    img = bgr.copy()
    tint = img.copy()
    tint[masks['band']] = (0.45 * tint[masks['band']] + 0.55 * np.array(COLORS[CROP])).astype(np.uint8)
    tint[masks['guard']] = (0.5 * tint[masks['guard']] + 0.5 * np.array(COLORS[UNKNOWN])).astype(np.uint8)
    img = cv2.addWeighted(tint, 0.6, img, 0.4, 0)
    s = width / img.shape[1]
    img = cv2.resize(img, (width, int(round(img.shape[0] * s))), interpolation=cv2.INTER_LINEAR)
    h, w = img.shape[:2]
    # row line / heading guide (image centre column projected ground line)
    for pts, col in info.get('row_lines', []):
        pts = [(int(u * s), int(v * s)) for u, v in pts]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(img, a, b, col, 1, cv2.LINE_AA)
    cv2.arrowedLine(img, (w // 2, h - 8), (w // 2, h - 60), (0, 200, 255), 2, cv2.LINE_AA, tipLength=0.3)
    # one CROP label per row (the nearest plant); the other plants get a thin outline only
    nearest = {}
    for d in dets:
        if d['cls'] == CROP and (d.get('row') not in nearest or d['dist'] < nearest[d.get('row')]['dist']):
            nearest[d.get('row')] = d
    labelled = {id(d) for d in nearest.values()}
    for d in sorted(dets, key=lambda d: (d['cls'] == HUMAN, d['cls'] == WEED)):
        if d['cls'] == CROP and not info.get('show_crop_boxes', True):
            continue
        x0, y0, x1, y1 = [int(round(v * s)) for v in d['bbox']]
        col = COLORS[d['cls']]
        th = 3 if d['cls'] in (HUMAN, WEED) else 1
        cv2.rectangle(img, (x0, y0), (x1, y1), col, th, cv2.LINE_AA)
        label = d.get('label') or (d['cls'] if d['cls'] != OTHER else d['sub'].upper())
        if d['cls'] in (HUMAN, WEED, OTHER):
            label += f' {d["dist"]:.1f}m'
        if d['cls'] == CROP and id(d) not in labelled:
            continue
        if d['cls'] == CROP:
            label = f'CROP row {d.get("row", "?")}'
        (tw, tht), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        cv2.rectangle(img, (x0, max(0, y0 - tht - 5)), (x0 + tw + 4, y0), col, -1)
        cv2.putText(img, label, (x0 + 2, y0 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (15, 15, 15), 1, cv2.LINE_AA)
    banner = []
    if info.get('human_active'):
        banner.append(('HUMAN DETECTED', COLORS[HUMAN]))
    if info.get('weed_now'):
        banner.append(('WEED DETECTED', COLORS[WEED]))
    x = 10
    for text, col in banner:
        (tw, tht), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.8, 2)
        cv2.rectangle(img, (x - 6, 6), (x + tw + 6, 16 + tht), col, -1)
        cv2.putText(img, text, (x, 12 + tht), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        x += tw + 24
    status = info.get('status_line')
    if status:
        cv2.rectangle(img, (0, h - 22), (w, h), (20, 20, 20), -1)
        cv2.putText(img, status, (8, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    return img


def project_world_line(T_world_cam, K, pts_world, shape):
    """World polyline -> image pixel polyline (points behind the camera are dropped)."""
    Tinv = np.linalg.inv(T_world_cam)
    P = np.c_[np.asarray(pts_world, float), np.ones(len(pts_world))] @ Tinv.T
    ok = P[:, 2] > 0.3
    P = P[ok]
    if len(P) < 2:
        return []
    u = K[0, 0] * P[:, 0] / P[:, 2] + K[0, 2]
    v = K[1, 1] * P[:, 1] / P[:, 2] + K[1, 2]
    h, w = shape[:2]
    keep = (u > -w) & (u < 2 * w) & (v > -h) & (v < 2 * h)
    return list(zip(u[keep], v[keep]))
