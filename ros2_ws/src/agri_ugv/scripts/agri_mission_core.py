#!/usr/bin/python3
"""Field geometry, row count, coverage and obstacle-corridor helpers for the crop-row mission.

Pure Python / numpy (no ROS) so it can be tested offline.

Row count (config field.auto_row_detection):
  world      - the crop rows are read from the world model (row_XX_seg links of model.sdf)
  dimensions - round(field_width / row_spacing) + 1
  fixed      - field.total_rows
Nothing in the mission depends on a particular number of rows.
"""
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def load_yaml(path):
    import yaml
    try:
        return yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}


def find_config(script_file, name='agriculture.yaml'):
    """<package>/config/<name> next to the script (symlink install) or in the installed share."""
    here = Path(script_file).resolve().parent
    cands = [here.parent / 'config' / name]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory('agri_ugv')) / 'config' / name)
    except Exception:
        pass
    return next((str(c) for c in cands if c.exists()), '')


def package_root(config_path):
    return Path(config_path).resolve().parent.parent if config_path else None


def rows_from_world(model_sdf):
    """[(row_y, x_min, x_max)] from the row_XX_segN links of the farm model, sorted by y."""
    try:
        root = ET.parse(model_sdf).getroot()
    except (OSError, ET.ParseError):
        return []
    model = root.find('model')
    if model is None:
        return []
    base = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
    rows = {}
    for link in model.findall('link'):
        m = re.match(r'row_(\d+)_seg(\d+)$', link.get('name', ''))
        if not m:
            continue
        p = [float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split()]
        rows.setdefault(int(m.group(1)), []).append((base[0] + p[0], base[1] + p[1]))
    out = []
    for k in sorted(rows):
        xs = sorted(x for x, _ in rows[k])
        ys = [y for _, y in rows[k]]
        half = (xs[-1] - xs[0]) / max(len(xs) - 1, 1) / 2 if len(xs) > 1 else 0.0
        out.append((float(np.mean(ys)), xs[0] - half, xs[-1] + half))
    return sorted(out)


class FieldGeometry:
    """Crop rows (parallel to world x), their planted extent and the headland turn points."""

    def __init__(self, cfg, package_dir=None):
        f, m = cfg['field'], cfg.get('mission', {})
        self.spacing = float(f.get('row_spacing', f.get('row_pitch', 1.22)))
        self.y0 = float(f['first_row_y'])
        self.x_start = float(f.get('crop_area_start', f.get('row_x_min', -13.9)))
        self.x_end = float(f.get('crop_area_end', self.x_start + float(f.get('field_length', 28.3))))
        self.margin = float(f.get('field_margin', 0.8))
        self.source = 'fixed'
        mode = str(f.get('auto_row_detection', 'world')).lower()
        n = None
        if mode == 'world' and package_dir is not None:
            rows = rows_from_world(Path(package_dir) / f.get('world_model', 'models/vigil_cotton_farm/model.sdf'))
            if len(rows) >= 1:
                n = len(rows)
                self.y0 = rows[0][0]
                if len(rows) > 1:
                    self.spacing = (rows[-1][0] - rows[0][0]) / (len(rows) - 1)
                self.source = 'world'
        if n is None and mode in ('world', 'dimensions') and f.get('field_width') is not None:
            n = int(round(float(f['field_width']) / self.spacing)) + 1
            self.source = 'dimensions'
        if n is None:
            n = int(f.get('total_rows', f.get('row_count', 1)))
            self.source = 'fixed'
        self.n = max(1, n)
        self.row_end_tolerance = float(m.get('row_end_tolerance', 0.4))
        self.turn_west = self.x_start - float(m.get('headland_start_margin', 0.6))
        self.turn_east = self.x_end + float(m.get('headland_end_margin', 2.6))

    # ---- rows
    def row_y(self, k):
        return self.y0 + np.asarray(k) * self.spacing

    def row_index(self, y):
        return np.clip(np.round((np.asarray(y) - self.y0) / self.spacing), 0, self.n - 1).astype(int)

    def nearest_row(self, y):
        return int(self.row_index(y))

    def width(self):
        return (self.n - 1) * self.spacing

    def bounds(self):
        """Crop field rectangle (x0, x1, y0, y1): planted extent, rows +- half a spacing."""
        return (self.x_start, self.x_end, self.y0 - self.spacing / 2, self.row_y(self.n - 1) + self.spacing / 2)

    # ---- one row in a driving direction (+1 east, -1 west)
    def turn_x(self, direction):
        """Headland turn point at the END of a row driven in `direction`."""
        return self.turn_east if direction > 0 else self.turn_west

    def start_x(self, direction):
        return self.turn_west if direction > 0 else self.turn_east

    def progress(self, x, direction):
        return direction * (x - self.start_x(direction))

    def row_length(self, direction):
        return self.progress(self.turn_x(direction), direction)

    def crop_end_progress(self, direction):
        """Progress at which the robot centre has passed the last plant (+ tolerance)."""
        last = self.x_end if direction > 0 else self.x_start
        return self.progress(last, direction) + self.row_end_tolerance

    def crop_begin_progress(self, direction):
        first = self.x_start if direction > 0 else self.x_end
        return self.progress(first, direction)

    def as_vision_field(self):
        """Keys used by agri_vision_core.Field."""
        return dict(first_row_y=self.y0, row_pitch=self.spacing, row_count=self.n, row_x_min=self.x_start,
                    row_x_max=self.x_end, field_margin=self.margin)

    def summary(self):
        return dict(rows=self.n, source=self.source, first_row_y=round(self.y0, 3), row_spacing=round(self.spacing, 3),
                    crop_area_start=self.x_start, crop_area_end=self.x_end, turn_west=self.turn_west,
                    turn_east=self.turn_east, row_end_tolerance=self.row_end_tolerance)


class Coverage:
    """Which parts of which crop rows the robot has driven (0.25 m bins along every row)."""

    def __init__(self, geo, bin_size=0.25):
        self.geo, self.bin = geo, bin_size
        self.nb = max(1, int(math.ceil((geo.x_end - geo.x_start) / bin_size)))
        self.done = np.zeros((geo.n, self.nb), bool)
        self.detoured = np.zeros((geo.n, self.nb), bool)

    def mark(self, row, x, detour=False, half=0.2):
        if not 0 <= row < self.geo.n:
            return
        i0 = int(math.floor((x - half - self.geo.x_start) / self.bin))
        i1 = int(math.floor((x + half - self.geo.x_start) / self.bin))
        i0, i1 = max(i0, 0), min(i1, self.nb - 1)
        if i0 <= i1:
            self.done[row, i0:i1 + 1] = True
            if detour:
                self.detoured[row, i0:i1 + 1] = True

    def row_fraction(self, row):
        return float(self.done[row].mean()) if 0 <= row < self.geo.n else 0.0

    def percent(self):
        return 100.0 * float(self.done.mean())

    def rows_complete(self):
        return int((self.done.mean(1) >= 0.999).sum())


# ----------------------------------------------------------------------------- obstacle geometry
def obstacle_points(ob, horizon=0.0, steps=4):
    """World points occupied by an obstacle now and (if moving) over the next `horizon` seconds."""
    pts = np.asarray(ob.get('points') or [[ob['x'], ob['y']]], float).reshape(-1, 2)
    r = float(ob.get('radius', 0.0))
    if ob.get('dynamic') and horizon > 0:
        v = np.array([ob.get('vx', 0.0), ob.get('vy', 0.0)])
        pts = np.vstack([pts + v * t for t in np.linspace(0, horizon, steps + 1)])
    return pts, r


def corridor_hits(obstacles, line_y, direction, geo, p_from, p_to, half_width, horizon=0.0):
    """Obstacles whose points lie inside the corridor |y - line_y| < half_width between progress
    p_from and p_to along a row driven in `direction`. Returns [(p_near, p_far, ob)] sorted."""
    out = []
    for ob in obstacles:
        pts, r = obstacle_points(ob, horizon)
        p = geo.progress(pts[:, 0], direction)
        inside = (np.abs(pts[:, 1] - line_y) < half_width + r) & (p + r > p_from) & (p - r < p_to)
        if inside.any():
            out.append((float(p[inside].min() - r), float(p[inside].max() + r), ob))
    return sorted(out, key=lambda h: h[0])


def zone_hits(obstacles, x0, x1, y0, y1, horizon=0.0):
    """Obstacles with any (predicted) point inside the rectangle."""
    xa, xb, ya, yb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
    out = []
    for ob in obstacles:
        pts, r = obstacle_points(ob, horizon)
        if (((pts[:, 0] > xa - r) & (pts[:, 0] < xb + r) & (pts[:, 1] > ya - r) & (pts[:, 1] < yb + r))).any():
            out.append(ob)
    return out


class Detour:
    """Lateral move to the neighbouring row line (offset = side * spacing), pass, move back.
    Offsets are a smooth cosine in progress p (radius > 2 m for a 4 m transition)."""

    def __init__(self, p_start, p_pass_end, amplitude, transition):
        self.p0, self.p1, self.A, self.L = p_start, p_pass_end, amplitude, transition

    def offset(self, p):
        if p <= self.p0:
            return 0.0, 0.0
        if p < self.p0 + self.L:
            s = (p - self.p0) / self.L
            return self.A * (1 - math.cos(math.pi * s)) / 2, self.A * math.pi / (2 * self.L) * math.sin(math.pi * s)
        if p < self.p1:
            return self.A, 0.0
        if p < self.p1 + self.L:
            s = (p - self.p1) / self.L
            return self.A * (1 + math.cos(math.pi * s)) / 2, -self.A * math.pi / (2 * self.L) * math.sin(math.pi * s)
        return 0.0, 0.0

    def phase(self, p):
        if p < self.p0 + self.L:
            return 'AVOIDING'
        if p < self.p1:
            return 'PASSING OBSTACLE'
        if p < self.p1 + self.L:
            return 'RETURNING TO ROW'
        return 'DONE'
