"""Offline tests of the simulated soil-moisture sensor and the live moisture map."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
PKG = HERE.parent / 'src/agri_ugv'
sys.path.insert(0, str(PKG / 'scripts'))

from agri_mission_core import FieldGeometry, load_yaml  # noqa: E402
from agri_moisture_core import MoistureField, MoistureMap, MoistureSensor  # noqa: E402
from agri_moisture_core import MoistureMapper  # noqa: E402

CFG = load_yaml(PKG / 'config/agriculture.yaml')
GEO = FieldGeometry(CFG, PKG)
B = GEO.bounds()
M = CFG['moisture']


def grid(field, n=80):
    xs, ys = np.meshgrid(np.linspace(B[0], B[1], n), np.linspace(B[2], B[3], n))
    return field.value(xs, ys)


def test_field_is_reproducible_and_seed_changes_it():
    a, b = grid(MoistureField(B, M)), grid(MoistureField(B, M))
    c = grid(MoistureField(B, dict(M, seed=M['seed'] + 1)))
    assert np.array_equal(a, b)
    assert np.abs(a - c).mean() > 3.0


def test_field_varies_in_range_with_dry_normal_and_wet_regions():
    v = grid(MoistureField(B, M))
    assert v.min() >= M['min_moisture'] - 1e-6 and v.max() <= M['max_moisture'] + 1e-6
    assert v.std() > 6.0                                               # not one constant value
    assert (v < M['low_threshold']).mean() > 0.03
    assert (v > M['high_threshold']).mean() > 0.03
    assert ((v >= M['low_threshold']) & (v <= M['high_threshold'])).mean() > 0.2


def test_field_is_spatially_correlated_not_white_noise():
    f = MoistureField(B, M)
    rng = np.random.default_rng(3)
    p = rng.uniform([B[0], B[2]], [B[1], B[3]], (400, 2))
    near = f.value(p[:, 0] + 0.3, p[:, 1]) - f.value(p[:, 0], p[:, 1])
    far = f.value(rng.uniform(B[0], B[1], 400), rng.uniform(B[2], B[3], 400)) - f.value(p[:, 0], p[:, 1])
    assert np.abs(near).mean() < 0.25 * np.abs(far).mean()


def test_sensor_noise_is_small_and_reading_follows_location():
    f = MoistureField(B, M)
    s = MoistureSensor(f, M)
    xs, ys = np.meshgrid(np.linspace(B[0], B[1], 60), np.linspace(B[2], B[3], 60))
    v = f.value(xs, ys)
    dry = (xs[v == v.min()][0], ys[v == v.min()][0])
    wet = (xs[v == v.max()][0], ys[v == v.max()][0])
    r_dry = [s.read(*dry)[0] for _ in range(50)]
    r_wet = [s.read(*wet)[0] for _ in range(50)]
    assert np.mean(r_dry) < M['low_threshold'] < M['high_threshold'] < np.mean(r_wet)
    err = np.array([s.read(*dry)[0] - s.read(*dry)[1] for _ in range(200)])
    assert 0.3 < err.std() < 2.0                                        # realistic, not unusably noisy


def drive_rows(mapper, sensor, rows, step=0.6):
    """Probe track of the row mission: probe 0.30 m left of the robot, rows alternate direction."""
    for i, k in enumerate(rows):
        d = 1 if i % 2 == 0 else -1
        y = float(GEO.row_y(k)) + 0.30 * d
        xs = np.arange(GEO.x_start, GEO.x_end, step)
        for x in (xs if d > 0 else xs[::-1]):
            v, _ = sensor.read(x, y)
            yield mapper.add(dict(moisture=v, x=float(x), y=y, row=k + 1))


def test_map_accumulates_and_coverage_is_area_not_messages():
    f = MoistureField(B, M)
    s = MoistureSensor(f, M)
    mp = MoistureMapper(B, M)
    covs = []
    for n in (3, 8, GEO.n):
        mp = MoistureMapper(B, M)
        for _ in drive_rows(mp, s, range(n)):
            pass
        covs.append(mp.map.coverage())
    assert covs[0] < covs[1] < covs[2]
    assert covs[0] == pytest.approx(100 * 3 / GEO.n, abs=4)
    assert covs[2] >= M['complete_threshold']
    # more messages on the same rows do not raise the coverage
    mp2 = MoistureMapper(B, M)
    for _ in drive_rows(mp2, s, range(3), step=0.1):
        pass
    assert mp2.map.coverage() == pytest.approx(covs[0], abs=1.5)


def test_interpolated_map_matches_the_true_field_and_unknown_is_not_invented():
    f = MoistureField(B, M)
    s = MoistureSensor(f, M)
    mp = MoistureMapper(B, M)
    for _ in drive_rows(mp, s, range(0, GEO.n, 1)):
        pass
    v, conf = mp.map.grid()
    xs, ys = np.meshgrid(mp.map.cx, mp.map.cy)
    ok = np.isfinite(v)
    assert np.abs(v[ok] - f.value(xs[ok], ys[ok])).mean() < 3.0
    # half the field surveyed: the other half stays unknown
    mp = MoistureMapper(B, M)
    for _ in drive_rows(mp, s, range(0, GEO.n // 2)):
        pass
    v, conf = mp.map.grid()
    top = ys > float(GEO.row_y(GEO.n // 2)) + 2.0
    assert np.isnan(v[top]).all() and (conf[top] == 0).all()


def test_events_are_meaningful_not_every_reading():
    f = MoistureField(B, M)
    s = MoistureSensor(f, M)
    mp = MoistureMapper(B, M)
    events, n = [], 0
    for ev in drive_rows(mp, s, range(GEO.n)):
        n += 1
        events += ev
    titles = [e['title'] for e in events]
    assert n > 1000 and len(events) < n / 5
    assert any('LOW MOISTURE REGION' in t for t in titles) and any('HIGH MOISTURE REGION' in t for t in titles)
    assert titles.count('MOISTURE MAP COMPLETE') == 1
