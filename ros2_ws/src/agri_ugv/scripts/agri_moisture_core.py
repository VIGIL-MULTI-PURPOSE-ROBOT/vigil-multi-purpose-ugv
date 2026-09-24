#!/usr/bin/python3
"""Simulated soil moisture: the true field, the probe reading, and the live moisture map.

Pure numpy (no ROS) so it can be tested offline.

MoistureField   the ground truth the simulated probe reads. Smooth, spatially correlated random
                values over the crop field (seeded -> reproducible, the same during one run):
                a coarse random lattice (spacing = correlation_length) interpolated with a smooth
                cubic, plus two broad wet / dry patches, scaled to [min_moisture, max_moisture].
MoistureMap     what the robot has measured: every reading is spread onto a grid with a Gaussian
                weight inside interpolation_radius; cells farther than that from every sample stay
                UNKNOWN (never invented). A cell counts as SURVEYED when a sample lies within
                coverage_radius; the mapping percentage is surveyed field area, not message count.
"""
import math

import numpy as np

DEFAULTS = dict(enabled=True, update_rate=2.0, sensing_radius=0.05, noise=0.8, min_moisture=18.0, max_moisture=78.0,
                seed=42, correlation_length=6.0, probe_x=-0.10, probe_y=0.30, map_resolution=0.25,
                interpolation_radius=1.2, coverage_radius=0.95, complete_threshold=99.0, low_threshold=30.0,
                high_threshold=60.0, event_distance=15.0, event_change=6.0, map_publish_rate=1.0)


def _smoothstep(t):
    return t * t * (3 - 2 * t)


class MoistureField:
    def __init__(self, bounds, cfg):
        c = dict(DEFAULTS, **(cfg or {}))
        self.x0, self.x1, self.y0, self.y1 = bounds
        self.lo, self.hi = float(c['min_moisture']), float(c['max_moisture'])
        self.L = float(c['correlation_length'])
        rng = np.random.default_rng(int(c['seed']))
        pad = 2 * self.L
        self.gx0, self.gy0 = self.x0 - pad, self.y0 - pad
        nx = int(math.ceil((self.x1 - self.x0 + 2 * pad) / self.L)) + 2
        ny = int(math.ceil((self.y1 - self.y0 + 2 * pad) / self.L)) + 2
        self.lattice = rng.normal(0, 1, (ny, nx))
        # two broad regions (one wet, one dry) so the map has clear low / high areas
        self.blobs = [(rng.uniform(self.x0, self.x1), rng.uniform(self.y0, self.y1), s * rng.uniform(1.2, 1.8),
                       rng.uniform(0.25, 0.4) * (self.x1 - self.x0)) for s in (1, -1)]
        # scale the raw field to [lo, hi] using its range over the crop field
        xs, ys = np.meshgrid(np.linspace(self.x0, self.x1, 120), np.linspace(self.y0, self.y1, 120))
        raw = self._raw(xs, ys)
        self.rmin, self.rmax = float(raw.min()), float(raw.max())

    def _raw(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        gx, gy = (x - self.gx0) / self.L, (y - self.gy0) / self.L
        ix, iy = np.clip(np.floor(gx).astype(int), 0, self.lattice.shape[1] - 2), np.clip(np.floor(gy).astype(int), 0, self.lattice.shape[0] - 2)
        fx, fy = _smoothstep(np.clip(gx - ix, 0, 1)), _smoothstep(np.clip(gy - iy, 0, 1))
        L = self.lattice
        v = (L[iy, ix] * (1 - fx) * (1 - fy) + L[iy, ix + 1] * fx * (1 - fy) +
             L[iy + 1, ix] * (1 - fx) * fy + L[iy + 1, ix + 1] * fx * fy)
        for bx, by, amp, r in self.blobs:
            v = v + amp * np.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2 * r * r))
        return v

    def value(self, x, y):
        """True moisture (%) at world (x, y)."""
        r = (self._raw(x, y) - self.rmin) / max(self.rmax - self.rmin, 1e-9)
        return self.lo + (self.hi - self.lo) * np.clip(r, 0, 1)


class MoistureSensor:
    """The probe: true value averaged over sensing_radius plus Gaussian noise (seeded)."""

    def __init__(self, field, cfg):
        c = dict(DEFAULTS, **(cfg or {}))
        self.field, self.noise, self.r = field, float(c['noise']), float(c['sensing_radius'])
        self.rng = np.random.default_rng(int(c['seed']) + 1)

    def read(self, x, y):
        a = np.linspace(0, 2 * math.pi, 7)[:-1]
        true = float(np.mean(np.r_[self.field.value(x, y), self.field.value(x + self.r * np.cos(a), y + self.r * np.sin(a))]))
        return float(np.clip(true + self.rng.normal(0, self.noise), 0, 100)), true


class MoistureMap:
    def __init__(self, bounds, cfg):
        c = dict(DEFAULTS, **(cfg or {}))
        self.x0, self.x1, self.y0, self.y1 = bounds
        self.res = float(c['map_resolution'])
        self.nx = int(math.ceil((self.x1 - self.x0) / self.res))
        self.ny = int(math.ceil((self.y1 - self.y0) / self.res))
        self.R = float(c['interpolation_radius'])
        self.cov_r = float(c['coverage_radius'])
        self.sigma = self.R / 2.0
        self.wsum = np.zeros((self.ny, self.nx))
        self.vsum = np.zeros((self.ny, self.nx))
        self.nearest = np.full((self.ny, self.nx), np.inf)
        self.samples = []
        cx = self.x0 + (np.arange(self.nx) + 0.5) * self.res
        cy = self.y0 + (np.arange(self.ny) + 0.5) * self.res
        self.cx, self.cy = cx, cy
        self.c = c

    def inside(self, x, y, margin=0.0):
        return self.x0 - margin <= x <= self.x1 + margin and self.y0 - margin <= y <= self.y1 + margin

    def add(self, x, y, value, t=0.0):
        if not self.inside(x, y, self.R):
            return False
        self.samples.append((round(x, 3), round(y, 3), round(value, 2), round(t, 2)))
        i0 = max(0, int((x - self.R - self.x0) / self.res))
        i1 = min(self.nx, int((x + self.R - self.x0) / self.res) + 2)
        j0 = max(0, int((y - self.R - self.y0) / self.res))
        j1 = min(self.ny, int((y + self.R - self.y0) / self.res) + 2)
        if i0 >= i1 or j0 >= j1:
            return True
        dx = self.cx[i0:i1][None, :] - x
        dy = self.cy[j0:j1][:, None] - y
        d = np.sqrt(dx * dx + dy * dy)
        w = np.where(d <= self.R, np.exp(-d * d / (2 * self.sigma ** 2)), 0.0)
        self.wsum[j0:j1, i0:i1] += w
        self.vsum[j0:j1, i0:i1] += w * value
        self.nearest[j0:j1, i0:i1] = np.minimum(self.nearest[j0:j1, i0:i1], d)
        return True

    def grid(self):
        """(values %, NaN where unknown), (confidence 0..1: 1 at a sample, 0 at interpolation_radius)"""
        v = np.where(self.wsum > 1e-6, self.vsum / np.maximum(self.wsum, 1e-12), np.nan)
        conf = np.clip(1 - self.nearest / self.R, 0, 1)
        return v, conf

    def coverage(self):
        return 100.0 * float((self.nearest <= self.cov_r).mean())

    def stats(self):
        v, _ = self.grid()
        known = v[np.isfinite(v)]
        lo, hi = float(self.c['low_threshold']), float(self.c['high_threshold'])
        vals = np.array([s[2] for s in self.samples]) if self.samples else np.zeros(0)
        out = dict(samples=len(self.samples), coverage_pct=round(self.coverage(), 2),
                   complete=self.coverage() >= float(self.c['complete_threshold']),
                   mapped_cells=int(known.size), cells=int(v.size))
        if known.size:
            out.update(mean=round(float(known.mean()), 1), min=round(float(known.min()), 1), max=round(float(known.max()), 1),
                       dry_pct=round(100 * float((known < lo).mean()), 1),
                       normal_pct=round(100 * float(((known >= lo) & (known <= hi)).mean()), 1),
                       wet_pct=round(100 * float((known > hi).mean()), 1))
        if vals.size:
            out.update(sample_min=round(float(vals.min()), 1), sample_max=round(float(vals.max()), 1))
        return out

    def category(self, value):
        lo, hi = float(self.c['low_threshold']), float(self.c['high_threshold'])
        return 'DRY' if value < lo else ('WET' if value > hi else 'NORMAL')


class MoistureMapper:
    """Map + event logic (no ROS)."""

    def __init__(self, bounds, cfg):
        self.c = dict(DEFAULTS, **(cfg or {}))
        self.map = MoistureMap(bounds, self.c)
        self.last = None
        self.last_event_pos = None
        self.last_event_value = None
        self.region = None
        self.region_pos = 0.0
        self.complete_announced = False
        self.dist = 0.0
        self.prev = None

    def add(self, r):
        events = []
        if not self.map.add(r['x'], r['y'], r['moisture'], r.get('stamp', 0.0)):
            return events
        self.last = r
        if self.prev is not None:
            self.dist += math.hypot(r['x'] - self.prev[0], r['y'] - self.prev[1])
        self.prev = (r['x'], r['y'])
        v = r['moisture']
        cat = self.map.category(v)
        lo, hi = float(self.c['low_threshold']), float(self.c['high_threshold'])
        # region change: 4 % hysteresis and at least 5 m since the last region event
        hy = 4.0
        if self.region is None:
            self.region = cat
            self.region_pos = self.dist
        elif (cat != self.region and (v < lo - hy or v > hi + hy or lo + hy < v < hi - hy)
              and self.dist - self.region_pos >= 5.0):
            self.region = cat
            self.region_pos = self.dist
            name = {'DRY': 'LOW MOISTURE REGION DETECTED', 'WET': 'HIGH MOISTURE REGION DETECTED',
                    'NORMAL': 'NORMAL MOISTURE REGION'}[cat]
            events.append(dict(kind='MOISTURE', title=name, detail=f"{v:.1f} % at ({r['x']:.1f}, {r['y']:.1f}) m, row {r.get('row', '?')}"))
        if (self.last_event_pos is None or self.dist - self.last_event_pos >= float(self.c['event_distance'])
                or (abs(v - self.last_event_value) >= float(self.c['event_change'])
                    and self.dist - self.last_event_pos >= 6.0)):
            self.last_event_pos, self.last_event_value = self.dist, v
            events.append(dict(kind='MOISTURE', title=f'MOISTURE SAMPLE: {v:.1f} %',
                               detail=f"{cat} at ({r['x']:.1f}, {r['y']:.1f}) m, row {r.get('row', '?')}"))
        if not self.complete_announced and self.map.coverage() >= float(self.c['complete_threshold']):
            self.complete_announced = True
            st = self.map.stats()
            events.append(dict(kind='DONE', title='MOISTURE MAP COMPLETE',
                               detail=f"{st['coverage_pct']} % of the field surveyed, {st['samples']} samples, "
                                      f"{st.get('min', '?')}-{st.get('max', '?')} %"))
        return events

    def finish(self):
        """The row mission has driven every row: the map is final. Whatever is still unsurveyed
        (behind obstacles the robot could not pass) stays UNKNOWN and is reported."""
        if self.complete_announced:
            return []
        self.complete_announced = True
        self.final = True
        st = self.map.stats()
        return [dict(kind='DONE', title='MOISTURE MAP COMPLETE',
                     detail=f"all rows surveyed: {st['coverage_pct']} % of the field area measured "
                            f"({100 - st['coverage_pct']:.1f} % not reachable), {st['samples']} samples, "
                            f"{st.get('min', '?')}-{st.get('max', '?')} %")]

    def status(self):
        st = self.map.stats()
        st['complete'] = bool(st['complete'] or getattr(self, 'final', False))
        st['latest'] = self.last
        s = self.map.samples
        step = max(1, len(s) // 3000)
        st['points'] = [[p[0], p[1], p[2]] for p in s[::step]]
        st['bounds'] = [self.map.x0, self.map.x1, self.map.y0, self.map.y1]
        st['resolution'] = self.map.res
        st['thresholds'] = [float(self.c['low_threshold']), float(self.c['high_threshold'])]
        st['range'] = [float(self.c['min_moisture']), float(self.c['max_moisture'])]
        return st
