"""Offline tests of the obstacle detector / tracker (agri_obstacle_core.py).

Synthetic 3D points as the 360 deg LiDAR + depth camera would return them: ground, crop plants on
the row lines, people, a vehicle. Crop plants must never become obstacles; motion is estimated.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
PKG = HERE.parent / 'src/agri_ugv'
sys.path.insert(0, str(PKG / 'scripts'))

from agri_mission_core import FieldGeometry, load_yaml  # noqa: E402
from agri_obstacle_core import ObstacleDetector, ObstacleTracker  # noqa: E402

CFG = load_yaml(PKG / 'config/agriculture.yaml')
GEO = FieldGeometry(CFG, PKG)
BAND = CFG['crop']['row_position_tolerance'] + CFG['crop']['guard_band']
RNG = np.random.default_rng(0)


def ground(cx, cy, r=10.0, n=3000):
    p = RNG.uniform(-r, r, (n, 2)) + [cx, cy]
    return np.c_[p, RNG.normal(0, 0.01, n)]


def crop_rows(x0, x1, rows, h=0.6):
    pts = []
    for k in rows:
        y = GEO.row_y(k)
        n = int((x1 - x0) * 60)
        pts.append(np.c_[RNG.uniform(x0, x1, n), y + RNG.uniform(-0.25, 0.25, n), RNG.uniform(0.05, h, n)])
    return np.vstack(pts)


def cylinder(x, y, r=0.25, h=1.7, n=300):
    a = RNG.uniform(0, 2 * math.pi, n)
    return np.c_[x + r * np.cos(a), y + r * np.sin(a), RNG.uniform(0, h, n)]


def box(x, y, lx, ly, h, n=800):
    return np.c_[RNG.uniform(x - lx / 2, x + lx / 2, n), RNG.uniform(y - ly / 2, y + ly / 2, n), RNG.uniform(0, h, n)]


def det():
    return ObstacleDetector(CFG['obstacles'], GEO, BAND)


ROBOT = (-8.0, float(GEO.row_y(4)), 0.0, 0.0)


def test_crop_rows_and_ground_are_not_obstacles():
    P = np.vstack((ground(ROBOT[0], ROBOT[1]), crop_rows(-13.8, 0, range(2, 8))))
    assert det().detect(P, ROBOT) == []


def test_person_in_the_aisle_and_on_the_row_line_are_obstacles():
    y = ROBOT[1]
    P = np.vstack((ground(ROBOT[0], y), crop_rows(-13.8, 0, range(2, 8)),
                   cylinder(-4.0, y + 0.61), cylinder(-2.0, y)))       # aisle, and standing on the row
    cl = det().detect(P, ROBOT)
    xs = sorted(round(c['x']) for c in cl)
    assert xs == [-4, -2]


def test_small_rocks_are_ignored_large_objects_detected():
    y = ROBOT[1]
    P = np.vstack((ground(ROBOT[0], y), box(-5.0, y + 0.6, 0.3, 0.3, 0.2), box(-3.0, y + 0.6, 3.6, 1.8, 2.5)))
    cl = det().detect(P, ROBOT)
    assert len(cl) == 1 and cl[0]['size'] > 1.5


def test_moving_person_velocity_and_static_vehicle():
    trk, d = ObstacleTracker(CFG['obstacles']), det()
    y = ROBOT[1]
    for i in range(15):
        t = i * 0.2
        P = np.vstack((ground(ROBOT[0], y), cylinder(-4.0, y - 3.0 + 0.8 * t), box(0.0, y + 2.5, 3.6, 1.8, 2.5)))
        trk.update(d.detect(P, ROBOT), t, ROBOT)
    obs = trk.export(15 * 0.2, ROBOT)
    person = min(obs, key=lambda o: abs(o['x'] + 4.0))
    vehicle = min(obs, key=lambda o: abs(o['x']))
    assert person['dynamic'] and person['vy'] == pytest.approx(0.8, abs=0.2) and abs(person['vx']) < 0.2
    assert not vehicle['dynamic'] and vehicle['speed'] < 0.1
    assert len(person['points']) >= 3


def test_tracks_expire_when_the_obstacle_leaves():
    trk, d = ObstacleTracker(CFG['obstacles']), det()
    y = ROBOT[1]
    for i in range(5):
        trk.update(d.detect(np.vstack((ground(ROBOT[0], y), cylinder(-4.0, y + 0.6))), ROBOT), i * 0.2, ROBOT)
    assert trk.export(1.0, ROBOT)
    for i in range(5, 30):
        trk.update(d.detect(ground(ROBOT[0], y), ROBOT), i * 0.2, ROBOT)
    assert trk.export(6.0, ROBOT) == []


def test_the_robot_itself_is_not_an_obstacle():
    x, y = ROBOT[0], ROBOT[1]
    P = np.vstack((ground(x, y), box(x, y, 1.6, 1.0, 1.1)))
    assert det().detect(P, ROBOT) == []
