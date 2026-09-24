#!/usr/bin/python3
"""Offline tests: collision geometry, dynamic obstacles, collision prediction, acceleration button,
simulation speed (no ROS, no Gazebo).

The closed-loop scenarios run the REAL code: planner_core.NavigatorCore (unchanged A-to-B navigator),
obstacle_core (tracker, prediction, overlay, DynamicAvoidance) and drive.DriveLaw (the jerk-limited
drive with the acceleration levels). The rover is a kinematic model driven through DriveLaw; people,
vehicles and objects move on straight lines; the "sensor" reports what the depth camera would see
(inside its 87 deg field of view and 14 m range, with 5 cm noise) - the depth -> points -> clusters
step itself is tested separately on rendered depth images (DepthPipelineTests).

    python3 test/test_dynamic_obstacles.py            # all
    python3 test/test_dynamic_obstacles.py -k human
"""
import json
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG / 'scripts'))
from sar_paths import load_config, find_military_world  # noqa: E402
from terrain_core import Params, Pose, CameraModel, rpy_to_matrix, SAFE, OBSTACLE, CLIFF, STEEP, CLIMBABLE  # noqa: E402
from planner_core import NavigatorCore  # noqa: E402
import obstacle_core as oc  # noqa: E402
from drive import DriveLaw  # noqa: E402

CFG = load_config(PKG / 'config/sar_mission.yaml')
PHYS = load_config(PKG / 'config/physics.yaml')['physics']
try:
    MW = find_military_world(os.environ.get('MILITARY_WORLD_DIR', ''))
except FileNotFoundError:
    MW = None
OP = oc.ObstacleParams.from_dict(CFG.get('obstacles', {}))
HFOV = CFG['camera']['horizontal_fov']


def pose_at(x, y, yaw):
    return Pose(x=x, y=y, z=0.0, R=rpy_to_matrix(0.0, 0.0, yaw))


# ============================================================ closed-loop scenario
class Mover:
    """Something in the world: a disc of `radius`, class, straight-line motion with optional
    back-and-forth between two points (a person walking a route)."""

    def __init__(self, cls, x, y, radius, vx=0.0, vy=0.0, until=None, start=0.0, bounce=None):
        self.cls, self.x, self.y, self.radius = cls, float(x), float(y), float(radius)
        self.vx, self.vy, self.until, self.start, self.bounce = vx, vy, until, start, bounce

    def step(self, t, dt):
        if t < self.start or (self.until is not None and t > self.until):
            return
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.bounce:
            (x0, y0), (x1, y1) = self.bounce
            if not (min(x0, x1) - 1e-6 <= self.x <= max(x0, x1) + 1e-6 and
                    min(y0, y1) - 1e-6 <= self.y <= max(y0, y1) + 1e-6):
                self.vx, self.vy = -self.vx, -self.vy
                self.x += 2 * self.vx * dt
                self.y += 2 * self.vy * dt


class Scenario:
    """A small zone (default 60 x 30 m) with walls/cliffs/hills as terrain classes and movers."""

    def __init__(self, size=(60.0, 30.0), origin=(-10.0, -15.0), start=(0.0, 0.0, 0.0), goal=(40.0, 0.0),
                 accel_level=None):
        cfg = json.loads(json.dumps(CFG))
        w = cfg['world']
        w['zone_x_min'], w['zone_y_min'] = origin
        w['zone_x_max'], w['zone_y_max'] = origin[0] + size[0], origin[1] + size[1]
        self.tp = Params.from_nested(cfg)
        self.nav_cfg = dict(cfg['navigation'], goal_x=goal[0], goal_y=goal[1], start_delay=0.0)
        self.nav = NavigatorCore(self.tp, self.nav_cfg)
        n = int(round(self.tp.map_size / self.tp.resolution))
        self.classes = np.full((n, n), SAFE, np.uint8)
        self.slope = np.zeros((n, n), np.float32)
        self.movers = []
        self.x, self.y, self.yaw = start
        self.law = DriveLaw(PHYS['drive'], PHYS['rover'])
        if accel_level is not None:
            self.law.set_level(accel_level)
        self.dyn = oc.DynamicAvoidance(OP, self.tp, self.nav_cfg, OBSTACLE, keep_classes=(CLIFF,))
        self.tracker = oc.ObstacleTracker(OP)
        self.rng = np.random.default_rng(3)
        self.log = []
        self.statuses = set()
        self.min_gap = {}                      # mover index -> min footprint-to-edge gap
        self.min_safety_ratio = {}
        self.hazard_hits = 0

    # --- world editing
    def cells(self, x0, x1, y0, y1):
        r, ox, oy = self.tp.resolution, self.tp.map_origin_x, self.tp.map_origin_y
        return (slice(max(0, int((y0 - oy) / r)), int(math.ceil((y1 - oy) / r))),
                slice(max(0, int((x0 - ox) / r)), int(math.ceil((x1 - ox) / r))))

    def block(self, x0, x1, y0, y1, cls=OBSTACLE):
        self.classes[self.cells(x0, x1, y0, y1)] = cls

    def add(self, m):
        self.movers.append(m)
        return m

    # --- sensing: what the depth camera + segmentation would report this frame
    def sense(self, t):
        dets = []
        for m in self.movers:
            dx, dy = m.x - self.x, m.y - self.y
            d = math.hypot(dx, dy)
            if d > OP.tracking_range or d < 0.3:
                continue
            if abs(math.atan2(dy, dx) - self.yaw + math.pi) % (2 * math.pi) - math.pi > HFOV / 2 + 0.05:
                continue
            nx, ny = self.rng.normal(0.0, 0.05, 2)
            dets.append(dict(x=m.x + nx, y=m.y + ny, radius=m.radius, height=1.7, cls=m.cls, n=40))
        return dets

    def pose(self):
        return pose_at(self.x, self.y, self.yaw)

    def footprint_gap(self, m):
        """Distance from the mover's disc to the rover's 1.53 x 1.12 m footprint (negative = contact)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = m.x - self.x, m.y - self.y
        lx, ly = c * dx + s * dy - self.tp.footprint_center_x, -s * dx + c * dy
        qx = max(abs(lx) - self.tp.robot_length / 2, 0.0)
        qy = max(abs(ly) - self.tp.robot_width / 2, 0.0)
        return math.hypot(qx, qy) - m.radius

    def run(self, t_max=120.0, dt=0.05, sense_period=0.1, stamp_period=0.3):
        nav = self.nav
        nav.update_map(self.classes, self.slope)
        t, last_sense, last_stamp, speed = 0.0, -1.0, -1.0, 0.0
        while t < t_max:
            for m in self.movers:
                m.step(t, dt)
            if t - last_sense >= sense_period - 1e-9:
                last_sense = t
                self.tracker.update(t, self.sense(t))
                self.dyn.set_tracks(self.tracker.confirmed())
            if t - last_stamp >= stamp_period - 1e-9:
                last_stamp = t
                nav.update_map(self.dyn.overlay(self.classes, self.pose(), nav.goal), self.slope)
            pose = self.pose()
            v, w = nav.tick(t, pose)
            if nav.state in nav.TERMINAL:
                v, w = 0.0, 0.0
            v, w = self.dyn.apply(t, pose, nav, v, w, speed)
            self.statuses.add(self.dyn.status)
            # drive: the real DriveLaw ramps (acceleration level, jerk, max speed)
            for _ in range(int(round(dt / 0.01))):
                self.law.tick(v, w, 0.01)
                self.x += self.law.v * math.cos(self.yaw) * 0.01
                self.y += self.law.v * math.sin(self.yaw) * 0.01
                self.yaw += self.law.w * 0.01
            speed = abs(self.law.v)
            for k, m in enumerate(self.movers):
                g = self.footprint_gap(m)
                self.min_gap[k] = min(self.min_gap.get(k, math.inf), g)
            i, j = self.tp_cell(self.x, self.y)
            if 0 <= i < self.classes.shape[0] and 0 <= j < self.classes.shape[1] and self.classes[i, j] in (OBSTACLE, CLIFF):
                self.hazard_hits += 1
            self.log.append((round(t, 2), round(self.x, 2), round(self.y, 2), round(self.law.v, 2), nav.state,
                             self.dyn.status))
            t += dt
            if nav.state == 'GOAL_REACHED' and abs(self.law.v) < 0.02:
                return t
        return None

    def tp_cell(self, x, y):
        r = self.tp.resolution
        return int((y - self.tp.map_origin_y) / r), int((x - self.tp.map_origin_x) / r)


# ============================================================ 1-11, 14, 15: closed loop
class ClosedLoopTests(unittest.TestCase):
    def assertNoContact(self, sc, min_gap=0.05):
        for k, g in sc.min_gap.items():
            self.assertGreater(g, min_gap, f'contact with {sc.movers[k].cls} #{k}: gap {g:.2f} m')

    def assertBaselineHits(self, *movers):
        """Control run: the same scene with the dynamic-obstacle layer switched off must end in
        contact - otherwise the scenario would not test anything."""
        sc = Scenario()
        sc.dyn.set_tracks = lambda tracks: None
        for m in movers:
            sc.add(Mover(m.cls, m.x, m.y, m.radius, m.vx, m.vy, m.until, m.start, m.bounce))
        sc.run(60.0)
        self.assertLess(min(sc.min_gap.values()), 0.0, 'control run without avoidance did not collide')

    def test_01_rover_moves_a_to_b(self):
        sc = Scenario()
        t = sc.run(60.0)
        self.assertIsNotNone(t, 'B not reached')
        self.assertLess(t, 30.0)
        self.assertGreater(max(r[3] for r in sc.log), 2.5, 'rover never reached open-road speed')

    def test_02_standing_human_on_the_route(self):
        self.assertBaselineHits(Mover('HUMAN', 18.0, 0.0, 0.3))
        sc = Scenario()
        sc.add(Mover('HUMAN', 18.0, 0.0, 0.3))
        t = sc.run(80.0)
        self.assertIsNotNone(t)
        self.assertNoContact(sc)
        # humans first: never closer than the human safety distance (minus the footprint rounding)
        self.assertGreater(sc.min_gap[0], OP.human_safety_distance - 0.35)

    def test_03_building_on_the_route(self):
        sc = Scenario()
        sc.block(14.0, 22.0, -4.0, 4.0)            # a 8 x 8 m building on the A-B line
        t = sc.run(90.0)
        self.assertIsNotNone(t)
        self.assertEqual(sc.hazard_hits, 0)

    def test_04_parked_vehicle(self):
        sc = Scenario()
        sc.block(17.8, 22.2, -0.9, 0.9)            # the car body is terrain OBSTACLE ...
        sc.add(Mover('VEHICLE', 20.0, 0.0, 2.3))    # ... and a tracked VEHICLE (safety distance)
        t = sc.run(90.0)
        self.assertIsNotNone(t)
        self.assertEqual(sc.hazard_hits, 0)
        self.assertGreater(sc.min_gap[0], OP.vehicle_safety_distance - 0.4)

    def test_05_static_obstacle(self):
        sc = Scenario()
        sc.block(19.5, 20.5, -1.5, 1.5)             # crate / rock pile on the line
        t = sc.run(80.0)
        self.assertIsNotNone(t)
        self.assertEqual(sc.hazard_hits, 0)

    def test_06_moving_obstacle_crossing(self):
        self.assertBaselineHits(Mover('OBJECT', 16.0, 6.0, 0.4, vy=-0.8))
        sc = Scenario()
        sc.add(Mover('OBJECT', 16.0, 6.0, 0.4, vy=-0.8))     # a trolley rolling across the route
        t = sc.run(90.0)
        self.assertIsNotNone(t)
        self.assertNoContact(sc)
        self.assertTrue({'COLLISION RISK', 'AVOIDING'} & sc.statuses, sc.statuses)

    def test_07_person_walking_across(self):
        self.assertBaselineHits(Mover('HUMAN', 14.0, 7.0, 0.3, vy=-1.2))
        sc = Scenario()
        # timed so that without prediction the rover and the person meet at x = 14
        sc.add(Mover('HUMAN', 14.0, 7.0, 0.3, vy=-1.2))
        t = sc.run(90.0)
        self.assertIsNotNone(t)
        self.assertNoContact(sc, 0.3)
        self.assertTrue({'COLLISION RISK', 'AVOIDING'} & sc.statuses, sc.statuses)

    def test_07b_person_walking_towards_the_rover(self):
        self.assertBaselineHits(Mover('HUMAN', 30.0, 0.3, 0.3, vx=-1.2))
        sc = Scenario()
        sc.add(Mover('HUMAN', 30.0, 0.3, 0.3, vx=-1.2))     # head-on along the road
        t = sc.run(100.0)
        self.assertIsNotNone(t)
        self.assertNoContact(sc, 0.2)

    def test_08_multiple_obstacles(self):
        sc = Scenario()
        sc.block(10.0, 11.0, -2.0, 2.0)
        sc.add(Mover('HUMAN', 18.0, -6.0, 0.3, vy=1.0, bounce=((18.0, -6.0), (18.0, 6.0))))
        sc.add(Mover('HUMAN', 26.0, 1.5, 0.3))
        sc.add(Mover('VEHICLE', 33.0, -3.0, 2.3))
        sc.block(31.0, 35.0, -3.9, -2.1)
        t = sc.run(150.0)
        self.assertIsNotNone(t, 'B not reached with several obstacles')
        self.assertNoContact(sc, 0.2)
        self.assertEqual(sc.hazard_hits, 0)

    def test_09_narrow_passage(self):
        sc = Scenario()
        # wall across the zone with a 1.7 m gap: the 1.12 m rover fits (footprint-aware), and must
        # not be refused by the obstacle overlay (static walls are never stamped again)
        sc.block(15.0, 16.0, -15.0, -0.85)
        sc.block(15.0, 16.0, 0.85, 15.0)
        sc.add(Mover('OBJECT', 15.5, 3.0, 0.3))       # a static crate beside the gap: not stamped
        t = sc.run(120.0)
        self.assertIsNotNone(t, 'rover did not use the 1.7 m gap')
        self.assertEqual(sc.hazard_hits, 0)

    def test_10_cliff_stays_hard(self):
        sc = Scenario()
        sc.block(12.0, 26.0, -15.0, -1.5, CLIFF)       # drop-off beside the road
        sc.add(Mover('HUMAN', 19.0, 1.5, 0.3))         # a person on the other side of the road
        # the overlay never relabels cliff cells
        over = sc.dyn.overlay(sc.classes, sc.pose(), sc.nav.goal)
        self.assertTrue(np.array_equal(over == CLIFF, sc.classes == CLIFF))
        t = sc.run(120.0)
        self.assertIsNotNone(t)
        self.assertEqual(sc.hazard_hits, 0, 'drove onto the cliff to avoid the person')
        self.assertNoContact(sc)

    def test_11_hill_classes_unchanged(self):
        sc = Scenario()
        sc.block(14.0, 24.0, -3.0, 3.0, CLIMBABLE)
        sc.slope[sc.cells(14.0, 24.0, -3.0, 3.0)] = 22.0
        sc.block(26.0, 28.0, 6.0, 9.0, STEEP)
        sc.add(Mover('HUMAN', 27.0, -8.0, 0.3))
        over = sc.dyn.overlay(sc.classes, sc.pose(), sc.nav.goal)
        self.assertTrue(np.array_equal(over == CLIMBABLE, sc.classes == CLIMBABLE))
        t = sc.run(120.0)
        self.assertIsNotNone(t)

    def test_14_15_point_b_stop(self):
        sc = Scenario(goal=(30.0, 5.0))
        sc.add(Mover('HUMAN', 20.0, 12.0, 0.3, vy=-1.0, until=12.0))
        t = sc.run(90.0)
        self.assertIsNotNone(t)
        self.assertLess(math.hypot(sc.x - 30.0, sc.y - 5.0), CFG['navigation']['goal_tolerance'] + 0.05)
        # after B: stays stopped
        x, y = sc.x, sc.y
        for _ in range(40):
            v, w = sc.nav.tick(t, sc.pose())
            self.assertEqual((v, w), (0.0, 0.0))
        self.assertEqual((sc.x, sc.y), (x, y))

    def test_no_stop_forever_when_a_person_blocks_the_gap(self):
        sc = Scenario()
        sc.block(15.0, 16.0, -15.0, -1.2)
        sc.block(15.0, 16.0, 1.2, 15.0)
        sc.block(15.0, 16.0, 6.0, 9.0, SAFE)            # a second, wider way round in the north
        sc.add(Mover('HUMAN', 16.5, 0.0, 0.3))          # stands right in the narrow gap
        t = sc.run(200.0)
        self.assertIsNotNone(t, 'rover waited for ever behind a standing person')
        self.assertNoContact(sc)


# ============================================================ tracker / prediction units
class TrackerTests(unittest.TestCase):
    def test_velocity_and_direction_of_a_walking_person(self):
        tr = oc.ObstacleTracker(OP)
        rng = np.random.default_rng(1)
        for k in range(30):
            t = k * 0.1
            tr.update(t, [dict(x=5.0 + 1.3 * t + rng.normal(0, 0.05), y=2.0 + rng.normal(0, 0.05), radius=0.3,
                               cls='HUMAN', height=1.7)])
        (h,) = tr.confirmed()
        self.assertAlmostEqual(h.vx, 1.3, delta=0.2)
        self.assertAlmostEqual(h.vy, 0.0, delta=0.2)
        self.assertTrue(h.dynamic)
        d = oc.describe(h, pose_at(0, 0, 0), OP, 3.0)
        self.assertEqual(d['cls'], 'HUMAN')
        self.assertAlmostEqual(d['heading_deg'], 0.0, delta=12.0)

    def test_static_person_is_not_dynamic(self):
        tr = oc.ObstacleTracker(OP)
        rng = np.random.default_rng(2)
        for k in range(40):
            tr.update(k * 0.1, [dict(x=5.0 + rng.normal(0, 0.06), y=rng.normal(0, 0.06), radius=0.3, cls='HUMAN')])
        (h,) = tr.confirmed()
        self.assertFalse(h.dynamic)

    def test_walls_never_move(self):
        tr = oc.ObstacleTracker(OP)
        for k in range(30):     # the visible part of a wall slides as the rover drives past
            tr.update(k * 0.1, [dict(x=8.0 + 0.2 * k, y=3.0, radius=3.0, cls='BUILDING')])
        (w,) = tr.confirmed()
        self.assertEqual((w.vx, w.vy, w.dynamic), (0.0, 0.0, False))

    def test_two_people_keep_their_ids(self):
        tr = oc.ObstacleTracker(OP)
        for k in range(30):
            t = k * 0.1
            tr.update(t, [dict(x=5 + t, y=1.0, radius=0.3, cls='HUMAN'), dict(x=5 + t, y=-1.0, radius=0.3, cls='HUMAN')])
        a, b = sorted(tr.confirmed(), key=lambda q: q.y)
        self.assertEqual({a.id, b.id}, {1, 2})
        self.assertLess(a.y, 0.0)

    def test_track_expires(self):
        tr = oc.ObstacleTracker(OP)
        for k in range(5):
            tr.update(k * 0.1, [dict(x=5, y=0, radius=0.3, cls='HUMAN')])
        tr.update(0.5 + OP.track_timeout + 0.1, [])
        self.assertEqual(tr.confirmed(), [])

    def test_collision_prediction(self):
        tr = oc.TrackView(dict(id=1, cls='HUMAN', x=6.0, y=4.0, vx=0.0, vy=-1.0, dynamic=True, radius=0.3))
        path = np.array([[x, 0.0] for x in np.arange(0.0, 20.0, 0.2)])
        r = oc.assess(pose_at(0, 0, 0), path, 0, 1.5, [tr], OP, 0.56)
        self.assertEqual(r['status'], 'COLLISION RISK')
        self.assertLess(r['ttc'], OP.collision_prediction_time)
        away = oc.TrackView(dict(id=2, cls='HUMAN', x=6.0, y=6.0, vx=0.0, vy=1.0, dynamic=True, radius=0.3))
        self.assertEqual(oc.assess(pose_at(0, 0, 0), path, 0, 1.5, [away], OP, 0.56)['status'], 'CLEAR')

    def test_class_safety_distances(self):
        self.assertGreaterEqual(OP.safety('HUMAN', False), OP.safety('VEHICLE', False))
        self.assertGreaterEqual(OP.safety('HUMAN', True), OP.dynamic_obstacle_distance)
        self.assertEqual(OP.safety('ROCK', False), OP.static_obstacle_distance)
        for k in ('dynamic_obstacle_distance', 'human_safety_distance', 'vehicle_safety_distance',
                  'static_obstacle_distance', 'collision_prediction_time', 'robot_footprint_margin'):
            self.assertIn(k, CFG['obstacles'], f'{k} not configurable in sar_mission.yaml')

    def test_overlay_sweeps_the_predicted_motion(self):
        sc = Scenario()
        tv = oc.TrackView(dict(id=1, cls='HUMAN', x=20.0, y=6.0, vx=0.0, vy=-1.0, dynamic=True, radius=0.3))
        sc.dyn.set_tracks([tv])
        over = sc.dyn.overlay(sc.classes, sc.pose(), sc.nav.goal)
        i, j = sc.tp_cell(20.0, 3.0)                      # 3 s ahead of the person
        self.assertEqual(over[i, j], OBSTACLE)
        i, j = sc.tp_cell(sc.x, sc.y)                     # never on the rover itself
        self.assertEqual(over[i, j], SAFE)
        i, j = sc.tp_cell(*sc.nav.goal)
        self.assertEqual(over[i, j], SAFE)


# ============================================================ depth -> points -> clusters
class DepthPipelineTests(unittest.TestCase):
    """Render the real 320 x 240 depth + label images of boxes with the rover camera model and run
    extract_points -> cluster -> tracker, as obstacle_tracker.py does."""

    def render(self, pose, boxes):
        tp = Params.from_nested(CFG)
        cam = CameraModel(tp)
        R, t = cam.world_transform(pose)
        d = cam.rays @ R.T
        depth = np.full(d.shape[:2], np.inf, np.float32)
        label = np.zeros(d.shape[:2], np.uint8)
        # ground plane z = pose.ground_z
        gz = pose.ground_z(tp)
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (gz - t[2]) / d[..., 2]
        hit = (s > 0) & np.isfinite(s)
        depth[hit] = s[hit]
        label[hit] = 1
        for (x0, x1, y0, y1, z0, z1, lab) in boxes:
            with np.errstate(divide='ignore', invalid='ignore'):
                lo = (np.array([x0, y0, z0]) - t) / d
                hi = (np.array([x1, y1, z1]) - t) / d
            tn = np.nanmax(np.minimum(lo, hi), axis=-1)
            tf = np.nanmin(np.maximum(lo, hi), axis=-1)
            ok = (tf >= tn) & (tn > 0) & (tn < depth)
            depth[ok] = tn[ok]
            label[ok] = lab
        depth[depth > tp.depth_far] = np.inf
        return tp, cam, depth, label

    def frame(self, pose, boxes, seg=True):
        tp, cam, depth, label = self.render(pose, boxes)
        pts, valid, _, _, _ = cam.backproject(depth, pose, tp)
        xyz, labs = oc.extract_points(pts, valid, label if seg else None, pose, OP, ground_z=pose.ground_z(tp))
        return oc.cluster(xyz, labs, OP)

    def test_person_and_wall_are_separate_classes(self):
        pose = pose_at(0, 0, 0)
        pose.z = 0.052
        dets = self.frame(pose, [(6.0, 6.4, -0.2, 0.2, 0.0, 1.75, 10), (7.0, 7.4, -4.0, -0.5, 0.0, 3.0, 30)])
        cls = sorted(d['cls'] for d in dets)
        self.assertEqual(cls, ['BUILDING', 'HUMAN'])
        h = next(d for d in dets if d['cls'] == 'HUMAN')
        self.assertAlmostEqual(h['x'], 6.05, delta=0.25)
        self.assertAlmostEqual(h['y'], 0.0, delta=0.2)

    def test_without_segmentation_height_decides(self):
        pose = pose_at(0, 0, 0)
        pose.z = 0.052
        dets = self.frame(pose, [(6.0, 6.4, -0.2, 0.2, 0.0, 1.75, 10)], seg=False)
        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0]['cls'], 'UNKNOWN')

    def test_walking_person_velocity_from_depth(self):
        tr = oc.ObstacleTracker(OP)
        for k in range(20):
            t = k * 0.1
            pose = pose_at(0.5 * t, 0, 0)            # the rover drives too
            pose.z = 0.052
            y = 3.0 - 1.0 * t
            dets = self.frame(pose, [(8.0, 8.4, y - 0.2, y + 0.2, 0.0, 1.75, 10)])
            tr.update(t, dets)
        (h,) = [q for q in tr.confirmed() if q.cls == 'HUMAN']
        self.assertTrue(h.dynamic)
        self.assertAlmostEqual(h.vy, -1.0, delta=0.25)
        self.assertAlmostEqual(h.vx, 0.0, delta=0.25)


# ============================================================ 2-4: collision geometry in the world
@unittest.skipIf(MW is None, 'military_world not found')
class CollisionGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tempfile
        from build_sar_world import build
        cls.tmp = tempfile.mkdtemp()
        cls.rep = build(json.loads(json.dumps(CFG)), MW, os.path.join(cls.tmp, 'w.sdf'))
        cls.world = ET.parse(os.path.join(cls.tmp, 'w.sdf')).getroot().find('world')

    def masks(self, model):
        return [c.findtext('surface/contact/collide_bitmask') for c in model.iter('collision')]

    def test_walking_people_are_force_driven_rigid_bodies_that_hit_the_rover(self):
        """No teleporting: controller 2 (force + balance torque) on dynamic bodies under gravity, and the
        collide bitmasks let people touch the rover (0xffff) and the ground (0x01), not the static world."""
        walkers = [m for m in self.world.findall('model') if m.find("plugin[@name='sar::WaypointSystem']") is not None]
        self.assertGreater(len(walkers), 10)
        for m in walkers:
            plug = m.find("plugin[@name='sar::WaypointSystem']")
            self.assertEqual(plug.findtext('controller_version'), '2', m.get('name'))
            self.assertEqual(m.findtext('static'), 'false')
            if plug.findtext('carried') != 'true':
                self.assertEqual(m.findtext('link/gravity'), 'true')
                self.assertLessEqual(float(plug.findtext('cruise_speed')), CFG['world']['human_max_speed'])
            masks = set(self.masks(m))
            self.assertEqual(masks, {'0x05'}, m.get('name'))
            self.assertTrue(int('0x05', 16) & 0xffff and int('0x05', 16) & 0x01 and not int('0x05', 16) & 0x02)
        ground = self.world.find("model[@name='SAR_Terrain_00_00']")
        self.assertEqual(set(self.masks(ground)), {'0x01'})
        # the physics engine stays the rover-validated one
        self.assertEqual(self.world.findtext('physics/dart/collision_detector'), 'ode')

    def test_every_person_has_a_primitive_body_that_follows_the_body_parts(self):
        src = ET.parse(MW / CFG['world']['source_sdf']).getroot().find('world')
        people = [m for m in self.world.findall('model') if 'Person' in m.get('name')]
        self.assertEqual(len(people), 37)
        for m in people:
            cols = list(m.iter('collision'))
            self.assertEqual(len(cols), 7, m.get('name'))
            kinds = sorted(c.find('geometry')[0].tag for c in cols)
            self.assertEqual(kinds, ['cylinder'] * 6 + ['sphere'], m.get('name'))
            self.assertEqual(sum(c.get('name').endswith('_head') for c in cols), 1)
            self.assertEqual(sum(c.get('name').endswith('_leg') for c in cols), 2)
            self.assertEqual(sum(c.get('name').endswith('_arm') for c in cols), 2)
            orig = src.find(f"model[@name='{m.get('name')}']")
            if orig is None:
                continue                     # people added inside buildings: copies of StaticPerson_009
            for c, o in zip(cols, orig.iter('collision')):
                self.assertEqual(c.findtext('pose'), o.findtext('pose'), 'primitive keeps the body part pose')
                sx, sy, sz = [float(v) for v in o.findtext('geometry/box/size').split()]
                g = c.find('geometry')[0]
                r = float(g.findtext('radius'))
                self.assertGreaterEqual(r + 1e-3, 0.5 * max(sx, sy), 'covers the body part cross-section')
                if g.tag == 'cylinder':
                    self.assertAlmostEqual(float(g.findtext('length')), sz, places=3)

    def test_buildings_vehicles_rocks_debris_collide_in_the_zone(self):
        """Every wall, vehicle part, rock and debris shape the rover can reach (bottom below
        world.collision_max_bottom, inside the zone) still collides; upper floors / roofs may not."""
        from build_sar_world import obstacle_category, pose_of
        w = CFG['world']
        src = ET.parse(MW / w['source_sdf']).getroot().find('world')
        gen = {m.get('name'): m for m in self.world.findall('model')}
        checked = {}
        for m in src.findall('model'):
            cat = obstacle_category(m)
            if cat not in ('BUILDING', 'VEHICLE', 'ROCK', 'DEBRIS'):
                continue
            mp = pose_of(m)
            for link in m.findall('link'):
                lp = pose_of(link)
                for c in link.findall('collision'):
                    cp = pose_of(c)
                    size = c.findtext('geometry/box/size')
                    h = float(size.split()[2]) if size else float(c.findtext('geometry/cylinder/length') or 0.0)
                    x, y = mp[0] + lp[0] + cp[0], mp[1] + lp[1] + cp[1]
                    bottom = lp[2] + cp[2] - h / 2
                    if not (w['zone_x_min'] < x < w['zone_x_max'] and w['zone_y_min'] < y < w['zone_y_max']):
                        continue
                    if bottom > w['collision_max_bottom'] or c.find('geometry/box') is None and \
                            c.find('geometry/cylinder') is None:
                        continue
                    g = gen[m.get('name')]
                    have = [q.get('name') for q in g.iter('collision')]
                    self.assertIn(c.get('name'), have, f"{m.get('name')} lost a reachable collision")
                    for q in g.iter('collision'):  # no walker contact tests, still hits the rover (0xffff)
                        self.assertIn(q.findtext('surface/contact/collide_bitmask'), ('0x02', None))
                    checked[cat] = checked.get(cat, 0) + 1
        for cat in ('BUILDING', 'VEHICLE', 'ROCK', 'DEBRIS'):
            self.assertGreater(checked.get(cat, 0), 3, (cat, checked))

    def test_obstacles_are_labelled_for_segmentation(self):
        labs = self.rep['obstacle_labels']
        for cat in ('HUMAN', 'VEHICLE', 'BUILDING', 'ROCK', 'DEBRIS'):
            self.assertGreater(labs.get(cat, 0), 0, cat)
        m = self.world.find("model[@name='SAR_DynamicPerson_021']")
        self.assertEqual(m.findtext("plugin[@name='gz::sim::systems::Label']/label"), '10')

    def test_walking_speed_is_realistic_and_unscaled(self):
        tops = self.rep['walker_speed']['top_speeds']
        for name, v in tops.items():
            self.assertLessEqual(v, CFG['world']['human_max_speed'] + 1e-6, name)
        self.assertEqual(self.rep['walker_speed']['scale'], 1.0)

    def test_simulation_speed_target(self):
        phys = self.world.find('physics')
        self.assertEqual(float(phys.findtext('real_time_factor')), float(CFG['world']['simulation_speed']))
        # execution speed only: step, gravity unchanged
        self.assertEqual(float(phys.findtext('max_step_size')), 0.001)
        self.assertEqual(self.world.findtext('gravity'), '0 0 -9.81')


# ============================================================ robot collision model
class RobotCollisionTests(unittest.TestCase):
    """The rover's collision body covers its CAD body (base_link), wheels are cylinders, and the added
    proxies are collision-only (no mass) and never lower than the chassis box."""

    @classmethod
    def setUpClass(cls):
        try:
            import xacro
            from xacro import substitution_args
        except ImportError:
            raise unittest.SkipTest('xacro not installed')
        orig = substitution_args._find

        def _find(resolved, a, args, context):
            return str(PKG) if args and args[0] == 'vigil_sar' else orig(resolved, a, args, context)
        substitution_args._find = _find
        try:
            doc = xacro.process_file(str(PKG / 'urdf/agri_ugv.urdf.xacro'),
                                     mappings={'simulation': 'true', 'vision_sensors': 'true', 'thermal_camera': 'true'})
        finally:
            substitution_args._find = orig
        cls.urdf = ET.fromstring(doc.toxml())

    def boxes(self, link):
        out = []
        for c in self.urdf.find(f"link[@name='{link}']").findall('collision'):
            b = c.find('geometry/box')
            if b is None:
                continue
            o = c.find('origin')
            xyz = [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]
            size = [float(v) for v in b.get('size').split()]
            out.append((np.array(xyz) - np.array(size) / 2, np.array(xyz) + np.array(size) / 2))
        return out

    def test_body_mesh_is_covered(self):
        stl = PKG / 'meshes/cad_body.stl'
        if not stl.exists():
            self.skipTest('meshes/cad_body.stl not present')
        b = stl.read_bytes()
        n = int.from_bytes(b[80:84], 'little')
        v = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v']
        v = v.reshape(-1, 3)[::7]
        inside = np.zeros(len(v), bool)
        for lo, hi in self.boxes('base_link'):
            inside |= np.all((v >= lo - 0.02) & (v <= hi + 0.02), axis=1)
        self.assertGreater(inside.mean(), 0.97, f'only {inside.mean():.1%} of the CAD body is inside a collision box')

    def test_proxies_add_no_mass_and_stay_above_the_chassis_bottom(self):
        base = self.urdf.find("link[@name='base_link']")
        self.assertEqual(base.find('inertial/mass').get('value'), '26.4')
        boxes = self.boxes('base_link')
        chassis_bottom = boxes[0][0][2]
        for lo, _ in boxes[1:]:
            self.assertGreater(lo[2], chassis_bottom + 0.09)

    def test_wheels_and_sensors_collide(self):
        for w in ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4'):
            self.assertIsNotNone(self.urdf.find(f"link[@name='{w}_wheel']/collision/geometry/cylinder"), w)
        for link in ('lidar_link', 'thermal_mast', 'thermal_link'):
            self.assertIsNotNone(self.urdf.find(f"link[@name='{link}']/collision"), link)
        # no collide_bitmask on the rover: it keeps 0xffff and collides with people (0x05)
        self.assertNotIn('collide_bitmask', ET.tostring(self.urdf).decode())


# ============================================================ Gazebo people scenarios (test_motion.sh)
@unittest.skipIf(MW is None, 'military_world not found')
class PeopleScenarioWorldTests(unittest.TestCase):
    def test_people_scenarios_are_generated_with_physical_people(self):
        import tempfile
        import scenario_worlds as sw
        out = Path(tempfile.mkdtemp())
        for name in ('human_block', 'human_standing', 'human_crossing'):
            sc = sw.SCENARIOS[name]
            (out / f'{name}.sdf').write_text(sw.world_sdf(name, sc))
            w = ET.parse(out / f'{name}.sdf').getroot().find('world')
            meta = sw.describe(name, sc)
            for q in meta['people']:
                m = w.find(f"model[@name='{q['name']}']")
                self.assertIsNotNone(m)
                self.assertEqual(len(list(m.iter('collision'))), 7)
                self.assertIsNotNone(m.find("plugin[@name='gz::sim::systems::OdometryPublisher']"))
                self.assertEqual(m.findtext("plugin[@name='gz::sim::systems::Label']/label"), '10')
                walker = m.find("plugin[@name='sar::WaypointSystem']")
                if q['kind'] == 'walker':
                    self.assertEqual(walker.findtext('controller_version'), '2')
                    self.assertEqual(m.findtext('static'), 'false')
                else:
                    self.assertIsNone(walker)
                    self.assertEqual(m.findtext('static'), 'true')
        self.assertEqual(sw.describe('human_block', sw.SCENARIOS['human_block'])['drive'], 'probe')


# ============================================================ 16: acceleration button
class AccelerationTests(unittest.TestCase):
    def test_levels_raise_acceleration_smoothly_and_never_the_top_speed(self):
        d = PHYS['drive']
        times = {}
        for lvl in range(len(d['accel_levels'])):
            law = DriveLaw(d, PHYS['rover'])
            law.set_level(lvl)
            prev_a, t, max_jerk = 0.0, 0.0, 0.0
            while law.v < 2.99 and t < 20:
                law.tick(3.0, 0.0, 0.01)
                max_jerk = max(max_jerk, abs(law.a - prev_a) / 0.01)
                prev_a = law.a
                t += 0.01
            times[lvl] = t
            self.assertLessEqual(max_jerk, d['max_jerk'] + 1e-6)
            self.assertLessEqual(law.v, d['max_linear'] + 1e-9)
            self.assertLessEqual(max(abs(law.a), 0), d['accel_levels'][lvl] + 1e-6)
            law.tick(10.0, 0.0, 0.01)                 # asking for more never exceeds max_linear
            self.assertLessEqual(law.v, d['max_linear'] + 1e-9)
        self.assertLess(times[len(times) - 1], times[0] - 0.5, 'higher level is not quicker')

    def test_level_is_capped_below_the_wheelie_limit_on_a_climb(self):
        d = PHYS['drive']
        law = DriveLaw(d, PHYS['rover'])
        law.set_level(len(d['accel_levels']) - 1)
        law.set_pitch(math.radians(33.0))              # nose up on the steepest climb
        self.assertLessEqual(law.acc, max(d['max_accel'], 1.99 * d['stability_margin']) + 1e-6)
        self.assertLess(law.acc, law.levels[-1])
        self.assertGreaterEqual(law.acc, d['max_accel'] - 1e-9)   # never below the validated base value
        law.set_pitch(0.0)
        self.assertAlmostEqual(law.acc, d['accel_levels'][-1])

    def test_button_cycles_up_and_back(self):
        d = PHYS['drive']
        law = DriveLaw(d, PHYS['rover'])
        names = []
        for _ in range(len(d['accel_levels']) + 1):
            names.append(law.level_name)
            law.step_level(+1)
        self.assertEqual(names[0], 'NORMAL')
        self.assertEqual(names[-1], names[-2], 'the + button stops at the highest level')
        law.reset_level()
        self.assertEqual(law.level_name, 'NORMAL')
        self.assertEqual(law.acc, d['max_accel'])


# ============================================================ 17: simulation speed
class SimulationSpeedTests(unittest.TestCase):
    def test_rtf_meter(self):
        from sim_speed import RtfMeter
        m = RtfMeter(window=5.0)
        for k in range(60):          # 4x: 0.4 s of simulation per 0.1 s of wall clock
            m.add(k * 0.1, k * 0.4)
        self.assertAlmostEqual(m.rtf(), 4.0, places=2)
        m = RtfMeter(window=5.0)
        for k in range(60):
            m.add(k * 0.1, k * 0.1 * 0.37)
        self.assertAlmostEqual(m.rtf(), 0.37, places=2)

    def test_set_physics_request(self):
        from sim_speed import set_physics_command
        cmd = set_physics_command('military_world', 4.0, 0.001)
        self.assertIn('/world/military_world/set_physics', cmd)
        self.assertIn('real_time_factor: 4', ' '.join(cmd))
        self.assertIn('max_step_size: 0.001', ' '.join(cmd))


if __name__ == '__main__':
    unittest.main(verbosity=2)
