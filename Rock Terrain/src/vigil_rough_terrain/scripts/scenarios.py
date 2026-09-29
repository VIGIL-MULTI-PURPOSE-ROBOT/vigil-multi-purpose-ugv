#!/usr/bin/python3
"""Cliff-navigation test scenarios (single source of truth).

Used by make_test_worlds.py (Gazebo SDF worlds) and test/closed_loop_sim.py
(offline ray-cast test). Robot starts at A=(0,0) facing +x; goal B is given.
Cliff scenarios put the robot on a 1.5 m high plateau; the plateau edge or a
hole in it is the cliff (1.5 m >> max_safe_drop 0.17 m).
"""
import math
import random

CLIFF_H = 1.5


def plateau(x0, x1, y0, y1, h, holes=()):
    """Axis-aligned plateau (top at h) minus rectangular holes -> list of boxes."""
    xs = sorted({x0, x1, *[max(x0, min(x1, v)) for hx0, hx1, _, _ in holes for v in (hx0, hx1)]})
    ys = sorted({y0, y1, *[max(y0, min(y1, v)) for _, _, hy0, hy1 in holes for v in (hy0, hy1)]})
    boxes = []
    for a, b in zip(xs[:-1], xs[1:]):
        for c, d in zip(ys[:-1], ys[1:]):
            mx, my = (a + b) / 2, (c + d) / 2
            if any(hx0 <= mx <= hx1 and hy0 <= my <= hy1 for hx0, hx1, hy0, hy1 in holes):
                continue
            if b - a > 1e-6 and d - c > 1e-6:
                boxes.append(dict(type='box', pos=(mx, my, h / 2), size=(b - a, d - c, h), pitch=0.0))
    return boxes


def ramp(x0, x1, y0, y1, angle_deg, thickness=0.6):
    """Box whose top face rises from (x0, z=0) to (x1, z=(x1-x0)tan(a))."""
    a = math.radians(angle_deg)
    L = (x1 - x0) / math.cos(a)
    top_mid = ((x0 + x1) / 2, ((x1 - x0) * math.tan(a)) / 2)
    nx, nz = -math.sin(a), math.cos(a)  # top-face normal
    cx, cz = top_mid[0] - nx * thickness / 2, top_mid[1] - nz * thickness / 2
    return [dict(type='box', pos=(cx, (y0 + y1) / 2, cz), size=(L, y1 - y0, thickness), pitch=-a)]


def rocks(n, x0, x1, y0, y1, hmin, hmax, seed=7):
    rnd = random.Random(seed)
    out = []
    for _ in range(n):
        r = rnd.uniform(0.08, 0.2)
        h = rnd.uniform(hmin, min(hmax, 2 * r))
        out.append(dict(type='sphere', pos=(rnd.uniform(x0, x1), rnd.uniform(y0, y1), h - r), radius=r))
    return out


def _cliff(holes=(), y0=-4.0, y1=4.0):
    return plateau(-3.0, 13.0, y0, y1, CLIFF_H, holes)


SCENARIOS = {
    'flat': dict(
        desc='Flat ground: drive straight A->B.',
        objects=[], spawn_z=0.0, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', max_hazard_replans=0, max_abs_y=0.5)),
    'small_rocks': dict(
        desc='30 small rocks (4-12 cm) on the route: must NOT avoid them.',
        objects=rocks(30, 1.5, 9.5, -1.5, 1.5, 0.04, 0.12), spawn_z=0.0, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', max_hazard_replans=0, max_abs_y=0.8)),
    'moderate_slope': dict(
        desc='15 deg ramp up to a 0.80 m plateau (slope limit 25 deg): drive up.',
        objects=ramp(3.0, 6.0, -4.0, 4.0, 15.0) + plateau(6.0, 13.0, -4.0, 4.0, 3.0 * math.tan(math.radians(15))),
        spawn_z=0.0, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', max_hazard_replans=0, max_abs_y=0.8)),
    'small_drop': dict(
        desc='12 cm step down (limit 17 cm): drive off it.',
        objects=plateau(-3.0, 5.0, -4.0, 4.0, 0.12), spawn_z=0.12, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', max_hazard_replans=0, max_abs_y=0.8)),
    'cliff_front': dict(
        desc='1.5 m deep pit directly on the straight line: go around.',
        objects=_cliff(holes=[(3.5, 6.5, -1.6, 1.6)]), spawn_z=CLIFF_H, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', min_hazard_replans=0, min_abs_y=1.5)),
    'cliff_left': dict(
        desc='Plateau edge 1.4 m to the left, parallel to the route: keep driving.',
        objects=_cliff(y0=-4.0, y1=1.4), spawn_z=CLIFF_H, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', max_abs_y=0.8)),
    'cliff_right': dict(
        desc='Right-hand edge cuts into the route (notch): shift left.',
        objects=_cliff(holes=[(4.0, 7.0, -1.4, 0.4)], y0=-1.4, y1=4.0), spawn_z=CLIFF_H, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', min_y=1.0)),
    'narrow_route': dict(
        desc='Chasm across the plateau with one 2.0 m land bridge off-line: find it.',
        objects=_cliff(holes=[(4.0, 6.5, -4.0, 0.9), (4.0, 6.5, 2.9, 4.0)]), spawn_z=CLIFF_H, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', bridge=(0.9, 2.9))),
    'too_narrow': dict(
        desc='Only a 1.2 m bridge (robot 1.12 m + 2x0.15 m margin needs 1.42 m): refuse.',
        objects=_cliff(holes=[(4.0, 6.5, -4.0, 1.4), (4.0, 6.5, 2.6, 4.0)]), spawn_z=CLIFF_H, goal=(10.0, 0.0),
        expect=dict(outcome='SEARCHING'), max_time=120.0),
    'no_route': dict(
        desc='Chasm across the whole plateau: never falls in, never stops, keeps searching (CLIFF_AVOIDANCE).',
        objects=_cliff(holes=[(4.0, 6.5, -4.0, 4.0)]), spawn_z=CLIFF_H, goal=(10.0, 0.0),
        expect=dict(outcome='SEARCHING', state='CLIFF_AVOIDANCE'), max_time=120.0),
    'steep_hill': dict(
        desc='28 deg hill across the whole route (above the 25 deg comfort limit, below the 33 deg '
             'climbing limit): must climb it, slowly, and reach B.',
        objects=ramp(3.0, 6.0, -4.0, 4.0, 28.0) + plateau(6.0, 13.0, -4.0, 4.0, 3.0 * math.tan(math.radians(28))),
        spawn_z=0.0, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', state='CLIMBING', min_tilt=24.0, max_abs_y=1.0)),
    'hill_choice': dict(
        desc='Hill with a 38 deg face on the direct line (beyond the 33 deg limit) and 22 deg flanks: '
             'climb a flank, not the face.',
        objects=(ramp(7.0 - 1.2 / math.tan(math.radians(38)), 7.0, -1.5, 1.5, 38.0)
                 + ramp(7.0 - 1.2 / math.tan(math.radians(22)), 7.0, -5.0, -1.5, 22.0)
                 + ramp(7.0 - 1.2 / math.tan(math.radians(22)), 7.0, 1.5, 5.0, 22.0)
                 + plateau(7.0, 13.0, -5.0, 5.0, 1.2)),
        spawn_z=0.0, goal=(10.0, 0.0),
        expect=dict(outcome='GOAL_REACHED', max_tilt=33.0, avoid_band=(6.3, -1.5, 1.5))),
    'big_rocks': dict(
        desc='15 rocks 15-21 cm high (above the 17 cm comfort step, below the 23 cm climbing step): drive over.',
        objects=rocks(15, 2.0, 9.0, -1.2, 1.2, 0.15, 0.21, seed=11), spawn_z=0.0, goal=(10.0, 0.0),
        # rocks are cost, never obstacles; at 0.6 m/s the 1.3 m look-ahead arcs ~1.5 m around the cluster
        expect=dict(outcome='GOAL_REACHED', max_abs_y=2.0, no_obstacle=True)),
}
