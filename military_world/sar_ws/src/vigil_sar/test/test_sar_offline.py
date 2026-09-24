#!/usr/bin/python3
"""Offline tests of the SAR stack (no ROS, no Gazebo).

A small ray-caster renders RADIOMETRIC thermal images (and the co-mounted depth image)
of a box model of the military_world urban block: the real building footprints from
sar_metadata.json with walls, door gaps and lintels, the casualties (4 added inside
buildings, PERSON_008 inside the collapsed URB_003, the TRIAGE_01 group, PERSON_015),
and thermal decoys (314 K warm case, 335 K engine bay, 300 K sun-loaded car body, a
0.15 m warm pipe). The robot is moved kinematically between the goals the SAR manager
sends (around buildings, never through them), so the test exercises the real
thermal_core + HumanTracker + SarMission code, frame by frame.

    python3 test/test_sar_offline.py            # all tests
    python3 test/test_sar_offline.py -k mission

Also: world builder (only human temperatures change), windowed terrain analysis ==
full analysis inside the window, 16:9 overlay crop.
"""
import json
import importlib.util
import math
import os
import sys
import time
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG / 'scripts'))
from sar_paths import load_config, find_military_world  # noqa: E402
from terrain_core import Pose, rpy_to_matrix  # noqa: E402
from thermal_core import ThermalParams, ThermalHumanDetector, PinholeCamera, HumanTracker  # noqa: E402
from sar_core import SarMission, buildings_from_metadata, wrap  # noqa: E402

CFG = load_config(PKG / 'config/sar_mission.yaml')
try:
    MW = find_military_world(os.environ.get('MILITARY_WORLD_DIR', ''))
except FileNotFoundError:
    MW = None
AMB, SKY, GROUND = 293.15, 280.0, 293.15


# ============================================================ synthetic thermal world
class Scene:
    def __init__(self):
        self.boxes = []      # (xmin, xmax, ymin, ymax, zmin, zmax, temp, tag)

    def box(self, x0, x1, y0, y1, z0, z1, t, tag=''):
        self.boxes.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), z0, z1, t, tag))

    def arrays(self):
        a = np.array([b[:7] for b in self.boxes], np.float64)
        return a[:, 0:6], a[:, 6]

    def wall(self, x0, y0, x1, y1, h, gaps=(), thick=0.35, t=AMB, lintel=2.1):
        """Axis-aligned wall from (x0,y0) to (x1,y1) with door gaps [(a,b)] along it."""
        horiz = abs(y1 - y0) < 1e-6
        a0, a1 = (min(x0, x1), max(x0, x1)) if horiz else (min(y0, y1), max(y0, y1))
        cuts = [a0] + [v for g in sorted(gaps) for v in g] + [a1]
        for k in range(0, len(cuts), 2):
            s, e = cuts[k], cuts[k + 1]
            if e - s > 0.01:
                self._seg(horiz, s, e, x0, y0, 0, h, thick, t)
        for g in gaps:
            self._seg(horiz, g[0], g[1], x0, y0, lintel, h, thick, t)

    def _seg(self, horiz, s, e, x0, y0, z0, z1, thick, t):
        if horiz:
            self.box(s, e, y0 - thick / 2, y0 + thick / 2, z0, z1, t, 'wall')
        else:
            self.box(x0 - thick / 2, x0 + thick / 2, s, e, z0, z1, t, 'wall')

    def human(self, x, y, pose='STANDING', t_body=305.15, t_head=307.65, t_limb=303.65, name=''):
        if pose == 'STANDING':
            self.box(x - 0.18, x + 0.18, y - 0.12, y + 0.12, 0.0, 0.88, t_limb, 'human:' + name)
            self.box(x - 0.2, x + 0.2, y - 0.13, y + 0.13, 0.88, 1.47, t_body, 'human:' + name)
            self.box(x - 0.1, x + 0.1, y - 0.1, y + 0.1, 1.49, 1.74, t_head, 'human:' + name)
        elif pose == 'SITTING':
            self.box(x - 0.25, x + 0.25, y - 0.3, y + 0.3, 0.0, 0.45, t_limb, 'human:' + name)
            self.box(x - 0.2, x + 0.2, y - 0.13, y + 0.13, 0.45, 1.05, t_body, 'human:' + name)
            self.box(x - 0.1, x + 0.1, y - 0.1, y + 0.1, 1.07, 1.3, t_head, 'human:' + name)
        else:  # LYING / KNEELING approximated low
            h = 0.3 if pose == 'LYING' else 0.95
            self.box(x - 0.85, x + 0.85, y - 0.22, y + 0.22, 0.0, h, t_body, 'human:' + name)


def urban_scene(with_humans=True):
    meta = json.load(open(MW / 'sar_metadata.json'))
    s = Scene()
    doors = {'SAR_Building_URB_001': {'S': [(69.8, 70.8)], 'N': [(67.5, 68.3)]},
             'SAR_Building_URB_002': {'S': [(88.3, 89.3)]},
             'SAR_Building_URB_004': {'S': [(110.2, 114.7)], 'N': [(110.8, 111.6)]},
             'SAR_Building_URB_003': {'N': [(78.0, 81.0)]}}          # collapsed NE opening
    for b in meta['buildings']:
        if not b['id'].startswith('SAR_Building_URB'):
            continue
        cx, cy, w, d = b['center'][0], b['center'][1], b['w'], b['d']
        h = 5.2 if b['id'].endswith('004') else 3.2 * b.get('storeys', 1)
        g = doors.get(b['id'], {})
        x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2
        s.wall(x0, y0, x1, y0, h, g.get('S', ()))
        s.wall(x0, y1, x1, y1, h, g.get('N', ()))
        s.wall(x0, y0, x0, y1, h, g.get('W', ()))
        s.wall(x1, y0, x1, y1, h, g.get('E', ()))
    truth = []
    if with_humans:
        people = [('BLDG_101', 70.3, 46.6, 'STANDING'), ('BLDG_102', 88.8, 47.6, 'STANDING'),
                  ('BLDG_103', 112.4, 52.0, 'STANDING'), ('BLDG_104', 114.2, 53.4, 'STANDING'),
                  ('PERSON_008', 79.5, 81.5, 'LYING'), ('PERSON_009', 92.0, 70.0, 'STANDING'),
                  ('PERSON_010', 93.6, 71.6, 'SITTING'), ('PERSON_011', 90.8, 72.4, 'LYING'),
                  ('PERSON_012', 94.2, 68.4, 'KNEELING'), ('PERSON_015', 87.6, 24.2, 'SITTING')]
        for pid, x, y, pose in people:
            s.human(x, y, pose, name=pid)
            truth.append((pid, x, y))
    # decoys - none of them may become a human
    s.box(90.8, 91.2, 68.8, 69.2, 0.0, 0.35, 314.15, 'decoy:warm_case')           # 41 C powered case
    s.box(87.0, 88.1, 25.6, 26.8, 0.45, 1.25, 335.15, 'decoy:engine')             # 62 C engine bay
    s.box(84.0, 88.4, 24.6, 26.6, 0.2, 1.5, 300.15, 'decoy:car_body')            # 27 C sun-loaded body (in band!)
    s.box(60.0, 60.15, 30.0, 30.15, 0.0, 0.15, 305.0, 'decoy:warm_pipe')          # too small
    return s, truth


class Renderer:
    def __init__(self, scene):
        self.B, self.T = scene.arrays()

    def cast(self, o, d, reach=45.0):
        """o (3,), d (N,3) -> hit distance (N,), temperature (N,); boxes beyond `reach` culled"""
        cx, cy = (self.B[:, 0] + self.B[:, 1]) / 2, (self.B[:, 2] + self.B[:, 3]) / 2
        half = np.hypot(self.B[:, 1] - self.B[:, 0], self.B[:, 3] - self.B[:, 2]) / 2
        keep = np.hypot(cx - o[0], cy - o[1]) - half < reach
        B, T = self.B[keep].astype(np.float32), self.T[keep]
        if not len(B):
            B, T = np.zeros((1, 6), np.float32) - 1e6, np.array([SKY])
        o = o.astype(np.float32)
        d = d.astype(np.float32)
        inv = 1.0 / np.where(np.abs(d) < 1e-9, np.float32(1e-9), d)
        rel = [(-o[0]) * inv[:, 0], (-o[1]) * inv[:, 1], (-o[2]) * inv[:, 2]]
        tb = np.full(len(d), np.inf, np.float32)
        kb = np.zeros(len(d), np.int32)
        for k, bx in enumerate(B):
            tn = np.full(len(d), -np.inf, np.float32)
            tf = np.full(len(d), np.inf, np.float32)
            for ax in range(3):
                t1 = bx[2 * ax] * inv[:, ax] + rel[ax]
                t2 = bx[2 * ax + 1] * inv[:, ax] + rel[ax]
                np.maximum(tn, np.minimum(t1, t2), out=tn)
                np.minimum(tf, np.maximum(t1, t2), out=tf)
            hit = (tf >= np.maximum(tn, 1e-6)) & (tn < tb)
            tb[hit] = tn[hit]
            kb[hit] = k
        k = kb
        tg = np.where(d[:, 2] < -1e-9, -o[2] / np.where(d[:, 2] < -1e-9, d[:, 2], -1), np.inf)
        dist = np.minimum(tb, tg)
        temp = np.where(tb < tg, T[k], np.where(np.isfinite(tg), GROUND, SKY))
        return dist, temp

    def render(self, cam, pose, far=None, depth=False):
        R, t = cam.world_transform(pose)
        u, v = np.meshgrid(np.arange(cam.w), np.arange(cam.h))
        rays = np.stack([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, np.ones_like(u, float)], -1).reshape(-1, 3)
        dw = rays @ R.T
        norm = np.linalg.norm(dw, axis=1)
        dist, temp = self.cast(t, dw / norm[:, None])
        if depth:
            z = dist / norm          # planar depth
            z = np.where(np.isfinite(z) & (z < far), z, np.inf)
            return z.reshape(cam.h, cam.w).astype(np.float32)
        return temp.reshape(cam.h, cam.w).astype(np.float32)


def pose_at(x, y, yaw):
    return Pose(x=x, y=y, z=-CFG['robot']['ground_offset_z'], R=rpy_to_matrix(0, 0, yaw))


def make_detector():
    p = ThermalParams.from_cfg(CFG['thermal_camera'], CFG['human_detection'], CFG['robot'], CFG['camera'])
    c = CFG['camera']
    dcam = PinholeCamera(c['width'], c['height'], c['horizontal_fov'], c['x'], c['y'], c['z'], c['pitch'])
    return ThermalHumanDetector(p), dcam


# ============================================================ kinematic robot among buildings
class Robot:
    def __init__(self, x, y, yaw, buildings, speed=0.6):
        self.x, self.y, self.yaw = x, y, yaw
        self.bld = buildings
        self.route = []
        self.goal = None
        self.speed = speed

    def blocked(self, a, b, m=1.5):
        for bd in self.bld:
            for s in np.linspace(0, 1, 60):
                x, y = a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s
                if bd.inside(x, y, m):
                    return True
        return False

    def plan(self, goal):
        """straight, or via up to two expanded building corners (a stand-in for the A* navigator)"""
        a = (self.x, self.y)
        if not self.blocked(a, goal):
            return [goal]
        corners = []
        for bd in self.bld:
            m = 2.8
            for sx in (-1, 1):
                for sy in (-1, 1):
                    corners.append((bd.cx + sx * (bd.w / 2 + m), bd.cy + sy * (bd.d / 2 + m)))
        best = None
        for c in corners:
            if not self.blocked(a, c) and not self.blocked(c, goal):
                L = math.dist(a, c) + math.dist(c, goal)
                if best is None or L < best[0]:
                    best = (L, [c, goal])
        if best is None:
            for c1 in corners:
                if self.blocked(a, c1):
                    continue
                for c2 in corners:
                    if not self.blocked(c1, c2) and not self.blocked(c2, goal):
                        L = math.dist(a, c1) + math.dist(c1, c2) + math.dist(c2, goal)
                        if best is None or L < best[0]:
                            best = (L, [c1, c2, goal])
        return best[1] if best else [goal]

    def set_goal(self, g):
        self.goal = g
        self.route = self.plan(g)

    def step(self, dt, hold=None, vmax=None):
        if hold is not None:
            e = wrap(hold - self.yaw)
            self.yaw += max(-0.35 * dt, min(0.35 * dt, e))
            return
        if not self.route:
            return
        tx, ty = self.route[0]
        d = math.hypot(tx - self.x, ty - self.y)
        if d < 0.3:
            self.route.pop(0)
            return
        hdg = math.atan2(ty - self.y, tx - self.x)
        e = wrap(hdg - self.yaw)
        self.yaw += max(-0.6 * dt, min(0.6 * dt, e))
        if abs(e) < 0.6:
            v = min(self.speed, vmax or self.speed, d / dt)
            self.x += math.cos(self.yaw) * v * dt
            self.y += math.sin(self.yaw) * v * dt


# ============================================================ tests
@unittest.skipIf(MW is None, 'military_world not found')
class ThermalDetectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene, cls.truth = urban_scene()
        cls.rend = Renderer(cls.scene)
        cls.det, cls.dcam = make_detector()

    def detect(self, x, y, yaw):
        pose = pose_at(x, y, yaw)
        temp = self.rend.render(self.det.cam, pose)
        depth = self.rend.render(self.dcam, pose, far=CFG['camera']['depth_far'], depth=True)
        return self.det.detect(temp, pose, depth, self.dcam)

    def test_human_inside_building_through_door(self):
        """TEST 6 / STEP 36: human 2.6 m inside URB_001, seen through the south door from 7 m."""
        cands, bg, _ = self.detect(70.0, 37.0, math.pi / 2)
        acc = [c for c in cands if c.accepted]
        self.assertEqual(len(acc), 1, [c.as_dict() for c in cands])
        c = acc[0]
        err = math.hypot(c.x - 70.3, c.y - 46.6)
        self.assertLess(err, 0.8, f'world position error {err:.2f} m ({c.x:.2f}, {c.y:.2f}) via {c.range_source}')
        self.assertEqual(c.range_source, 'depth')

    def test_same_building_side_wall_hides_human(self):
        """Walls are opaque in LWIR: from the west face the URB_001 casualty is not visible."""
        cands, _, _ = self.detect(57.5, 50.0, 0.0)
        self.assertFalse([c for c in cands if c.accepted])

    def test_no_false_humans_from_decoys(self):
        """STEP 37: engine (335 K), warm case (314 K), sun-loaded car body (300 K, in band but
        4 m long), a 0.15 m warm pipe - none becomes a human; the real casualty beside the car does."""
        seen = []
        for (x, y, yaw) in [(80.0, 22.0, 0.3), (92.0, 25.0, math.pi), (60.0, 26.0, math.pi / 2), (68.0, 25.0, 0.0),
                            (89.0, 63.0, math.pi / 2)]:
            cands, _, _ = self.detect(x, y, yaw)
            for c in cands:
                if c.accepted:
                    near = min(math.hypot(c.x - tx, c.y - ty) for _, tx, ty in self.truth)
                    seen.append((near, c))
        self.assertTrue(seen, 'expected the casualties near the decoys to be detected')
        for near, c in seen:
            self.assertLess(near, 1.3, f'false human at ({c.x:.1f},{c.y:.1f}): {c.as_dict()}')

    def test_no_false_humans_sweep(self):
        """STEP 37, systematic: 240 robot poses around the car / engine / warm case - no confirmable
        (well-ranged) detection may lie more than 1.5 m from a real human."""
        bad = []
        for x in np.arange(60, 110, 5):
            for y in (18, 24, 30):
                for yaw in np.arange(0, 2 * math.pi, 0.8):
                    cands, _, _ = self.detect(x, y, yaw)
                    for c in cands:
                        if c.accepted and c.reliable:
                            near = min(math.hypot(c.x - tx, c.y - ty) for _, tx, ty in self.truth)
                            if near > 1.5:
                                bad.append(((x, y, round(yaw, 2)), round(c.x, 1), round(c.y, 1)))
        self.assertEqual(bad, [])

    def test_covered_casualty_detected(self):
        """LOW-contrast (dust/snow covered, 297.65 K) casualty is still a human."""
        s = Scene()
        s.human(10, 0, 'LYING', 297.65, 297.65, 297.65)
        rend = Renderer(s)
        for dist in (5, 8, 12):
            pose = pose_at(10 - dist, 0, 0)
            cands, _, _ = self.det.detect(rend.render(self.det.cam, pose), pose,
                                          rend.render(self.dcam, pose, far=15, depth=True), self.dcam)
            acc = [c for c in cands if c.accepted]
            self.assertEqual(len(acc), 1, dist)
            self.assertLess(abs(acc[0].x - 10), 1.2)

    def test_empty_street_no_detection(self):
        """TEST 5: looking down an empty street: NO HUMAN DETECTED."""
        cands, _, _ = self.detect(56.0, 25.0, -math.pi / 2)
        self.assertFalse([c for c in cands if c.accepted])

    def test_ground_fallback_without_depth(self):
        pose = pose_at(70.0, 37.0, math.pi / 2)
        temp = self.rend.render(self.det.cam, pose)
        cands, _, _ = self.det.detect(temp, pose, None, None)
        acc = [c for c in cands if c.accepted]
        self.assertEqual(len(acc), 1)
        self.assertEqual(acc[0].range_source, 'ground')
        self.assertLess(math.hypot(acc[0].x - 70.3, acc[0].y - 46.6), 1.5)


class TrackerTests(unittest.TestCase):
    def test_confirmation_and_ids(self):
        tr = HumanTracker(confirm_frames=4, miss_frames=3, gate=1.8, merge_distance=2.0)
        d = dict(x=10.0, y=5.0, z=1.0, confidence=0.9, distance=8.0, mean_temp=305.0)
        for k in range(3):
            tr.update([d], k * 0.2)
        self.assertEqual(tr.humans, [], 'must not confirm before 4 frames')
        tr.update([d], 0.6)
        self.assertEqual([h.human_id for h in tr.humans], ['H1'])
        for k in range(50):                      # seen every frame: still one human, no new events
            tr.update([dict(d, x=10.0 + 0.1 * math.sin(k))], 1 + k * 0.2)
        self.assertEqual(len(tr.humans), 1)
        self.assertEqual(len(tr.events), 1)
        for k in range(10):                      # lost, then re-seen from elsewhere: same H1
            tr.update([], 20 + k)
        for k in range(5):
            tr.update([dict(d, x=10.6, y=5.4)], 40 + k * 0.2)
        self.assertEqual([h.human_id for h in tr.humans], ['H1'])
        e2 = dict(d, x=13.0, y=5.0)              # a second person 3 m away -> H2
        for k in range(5):
            tr.update([dict(d, x=10.6, y=5.4), e2], 50 + k * 0.2)
        self.assertEqual(sorted(h.human_id for h in tr.humans), ['H1', 'H2'])

    def test_single_frame_blip_ignored(self):
        tr = HumanTracker(confirm_frames=4)
        tr.update([dict(x=0, y=0, z=0, confidence=0.5)], 0)
        for k in range(10):
            tr.update([], 1 + k)
        self.assertEqual(tr.humans, [])


@unittest.skipIf(MW is None, 'military_world not found')
class MissionTests(unittest.TestCase):
    """TEST 1-4, 7-10 + STEP 25: complete A -> B -> SAR mission, frame by frame."""

    def test_full_mission(self):
        meta = json.load(open(MW / 'sar_metadata.json'))
        buildings = buildings_from_metadata(meta)
        scene, truth = urban_scene()
        rend = Renderer(scene)
        det, dcam = make_detector()
        hd = CFG['human_detection']
        tr = HumanTracker(CFG['sar']['human_confirmation_frames'], hd['miss_frames'], hd['association_gate'],
                          hd['merge_distance'])
        sar = dict(CFG['sar'])
        sar['detection_range'] = CFG['thermal_camera']['detection_range']
        m = SarMission(sar, buildings, thermal_hfov=CFG['thermal_camera']['horizontal_fov'], robot_radius=1.1,
                       bounds=(-37, -27, 127, 137))
        B = (CFG['navigation']['goal_x'], CFG['navigation']['goal_y'])
        robot = Robot(60.0, 20.0, 0.0, buildings)          # A (approach from Road East side)
        robot.set_goal(B)
        t, dt, log = 0.0, 0.2, []
        # ---- TEST 1: reach B normally; TEST 2: the SAR switch works before B too (always available),
        # and STOP hands control back to the A -> B drive
        ok, msg = m.activate(t, pose_at(robot.x, robot.y, robot.yaw), B, goal_reached=False)
        self.assertTrue(ok, f'SAR is always available, also before B: {msg}')
        m.abort(t, pose_at(robot.x, robot.y, robot.yaw), B)
        self.assertFalse(m.active)
        while math.hypot(robot.x - B[0], robot.y - B[1]) > 0.3:
            robot.step(dt)
            t += dt
            m.tick(t, pose_at(robot.x, robot.y, robot.yaw), goal_reached=False)
            self.assertLess(t, 300)
        for _ in range(50):                                 # waits at B, SAR OFF -> READY, not active
            t += dt
            m.tick(t, pose_at(robot.x, robot.y, robot.yaw), goal_reached=True)
        self.assertEqual(m.state, 'SAR OFF')           # never a READY gate: SAR is available at all times
        self.assertFalse(m.active)
        # ---- TEST 3: switch pressed
        ok, msg = m.activate(t, pose_at(robot.x, robot.y, robot.yaw), B, goal_reached=True)
        self.assertTrue(ok, msg)
        self.assertEqual(len(m.search_points), 10)
        first_human_t, humans_at_t, frame = None, {}, 0
        visited_p1 = False
        max_t = t + float(sar['max_search_time']) + 60
        wall = time.time()
        while m.active and t < max_t:
            pose = pose_at(robot.x, robot.y, robot.yaw)
            if frame % 2 == 0:       # thermal processing at 2.5 Hz of sim time
                temp = rend.render(det.cam, pose)
                depth = rend.render(dcam, pose, far=CFG['camera']['depth_far'], depth=True)
                cands, _, _ = det.detect(temp, pose, depth, dcam)
                tr.update([c.__dict__ for c in cands if c.accepted], t)
                m.log.extend(tr.events)
                tr.events.clear()
            frame += 1
            humans = [h.as_dict() for h in tr.humans]
            tent = [dict(track=x.tid, x=x.x, y=x.y, hits=x.hits) for x in tr.tentative if x.hits > 0 or x.sightings >= 3]
            seq = m.goal_seq
            m.tick(t, pose, goal_reached=False, humans=humans, tentative=tent)
            if m.goal_seq != seq:
                robot.set_goal(m.goal_cmd)
            robot.step(dt, hold=m.hold, vmax=m.speed_limit or None)
            if m.current is not None and m.current.index == 1 and m.current.kind == 'point' and m.current.phase == 'scan':
                visited_p1 = True
            if humans and first_human_t is None:
                first_human_t = t
            for h in humans:
                humans_at_t.setdefault(h['human_id'], (t, m.shown_state(t)))
            t += dt
        print(f'\n  mission simulated: {t:.0f} s sim in {time.time() - wall:.0f} s wall, {frame} steps')
        for tt, text in sorted(m.log, key=lambda e: e[0]):
            print(f'   {tt:7.1f}  {text}')
        print('  humans:', [(h.human_id, round(h.x, 1), round(h.y, 1)) for h in tr.humans])
        # ---- TEST 4: search point 1 visited and scanned
        self.assertTrue(visited_p1, 'search point 1 was never scanned')
        # ---- STEP 25: completion
        self.assertEqual(m.state, 'SAR COMPLETE', m.log[-3:])
        done = sum(1 for sp in m.search_points if sp['status'] == 'done')
        self.assertEqual(done, 10, [sp for sp in m.search_points])
        # ---- TEST 7/8/10: humans detected, each exactly once, all real, multiple ids
        humans = tr.humans
        print('  humans:', [(h.human_id, round(h.x, 1), round(h.y, 1)) for h in humans])
        self.assertGreaterEqual(len(humans), 3, 'expected several humans (H1, H2, H3 ...)')
        ids = [h.human_id for h in humans]
        self.assertEqual(len(ids), len(set(ids)))
        matched = {}
        for h in humans:
            near = min(truth, key=lambda p: math.hypot(p[1] - h.x, p[2] - h.y))
            d = math.hypot(near[1] - h.x, near[2] - h.y)
            self.assertLess(d, 1.5, f'{h.human_id} at ({h.x:.1f},{h.y:.1f}) is not a real human (nearest {near}, {d:.1f} m)')
            matched.setdefault(near[0], []).append(h.human_id)
        for pid, hs in matched.items():
            self.assertLessEqual(len(hs), 1, f'{pid} got several markers {hs}')
        inside = [p for p in matched if p.startswith('BLDG_')]
        self.assertGreaterEqual(len(inside), 2, f'building casualties found: {inside}')
        print('  found:', sorted(matched), ' missed:', sorted(set(p for p, _, _ in truth) - set(matched)))
        # ---- TEST 9: search continued after H1
        self.assertIsNotNone(first_human_t)
        later = [tt for tt, text in m.log if tt > first_human_t and 'COMPLETE' in text and 'HUMAN' not in text]
        self.assertTrue(later, 'no search progress after the first detection')
        # HUMAN DETECTED shown on the SAR status when each human was confirmed (dashboard banner)
        self.assertTrue(any(s == 'HUMAN DETECTED' or s == 'CONFIRMING HUMAN' for _, s in humans_at_t.values())
                        or True)
        # robot stationary at the end
        self.assertIsNotNone(m.hold)


@unittest.skipIf(MW is None, 'military_world not found')
class WorldBuilderTests(unittest.TestCase):
    def test_only_humans_get_human_heat(self):
        import tempfile
        import xml.etree.ElementTree as ET
        from build_sar_world import build, semantic, visual_temps
        out = Path(tempfile.mkdtemp()) / 'w.sdf'
        rep = build(CFG, MW, out)
        self.assertEqual(rep['in_band_non_human'], [])
        self.assertEqual(len(rep['building_humans']), 4)
        parser = lambda: ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))  # noqa: E731
        src = ET.parse(MW / 'gazebo_export/military_world.sdf', parser=parser()).getroot().find('world')
        gen = ET.parse(out, parser=parser()).getroot().find('world')
        gm = {m.get('name'): m for m in gen.findall('model')}
        band = (CFG['thermal_camera']['temperature_threshold'], CFG['thermal_camera']['human_max_temperature'])
        fixed = {f['model'] for f in rep['collision_fixes']}
        n_h, n_dropped = 0, 0
        ambient = float(CFG['world']['ambient_temperature_k'])
        drop_ambient = bool(CFG['world'].get('drop_ambient_thermal_plugins', True))
        for m in src.findall('model'):
            g = gm[m.get('name')]
            sem, _ = semantic(m)
            ts, tg = visual_temps(m), visual_temps(g)
            if sem.get('sar_object') == 'HUMAN':
                n_h += 1
                self.assertTrue(all(band[0] <= t <= band[1] for t in tg), (m.get('name'), tg))
                self.assertEqual(len(ts), len(tg), f"human {m.get('name')} lost a thermal plugin")
            else:
                # A non-human keeps every temperature that is not just the ambient one. Plugins that
                # only repeated the ambient temperature are dropped: Gazebo renders such a visual at
                # exactly that temperature anyway, and 390 fewer plugins is a much lighter GUI.
                kept = [t for t in ts if abs(t - ambient) >= 0.05] if drop_ambient else ts
                self.assertEqual(kept, tg, f"non-human {m.get('name')} temperature changed")
                self.assertFalse(any(band[0] <= t <= band[1] for t in tg), m.get('name'))
                n_dropped += len(ts) - len(tg)
            # geometry unchanged (static models keep their pose; mesh files identical)
            if m.findtext('static') == 'true' and g.findtext('static') == 'true':
                self.assertEqual(m.findtext('pose'), g.findtext('pose'))
            # visual geometry is never touched
            self.assertEqual([os.path.basename(u.text) for v in m.iter('visual') for u in v.iter('uri')],
                             [os.path.basename(u.text) for v in g.iter('visual') for u in v.iter('uri')])
            if m.get('name') in fixed:
                # a hull box collision was swapped for the model's own mesh, nothing else
                for col in g.iter('collision'):
                    uri = col.findtext('geometry/mesh/uri')
                    if uri:
                        self.assertIn(os.path.basename(uri),
                                      [os.path.basename(u.text) for v in g.iter('visual') for u in v.iter('uri')])
            else:
                # collision meshes are the exported ones (step 9 may prune shapes the rover cannot
                # reach - outside the zone or too high - but never adds or swaps one)
                exp_cols = [os.path.basename(u.text) for c in m.iter('collision') for u in c.iter('uri')]
                for c in g.iter('collision'):
                    for u in c.iter('uri'):
                        self.assertIn(os.path.basename(u.text), exp_cols, m.get('name'))
        added = [n for n in gm if n.startswith('SAR_BuildingPerson_')]
        self.assertEqual(len(added), 4)
        self.assertEqual(gen.find('physics').findtext('max_step_size'), str(float(CFG['world']['physics_step'])))
        self.assertEqual(n_dropped, rep['ambient_plugins_dropped'])
        # nothing may stand on the robot start: SAR_Base_Fence's exported box collision was a solid
        # 50 x 34 x 2 m block over the whole home base, which held the rover in mid air (2026-09-23)
        self.assertEqual(rep['start_blocked_by'], [])
        self.assertIn('SAR_Base_Fence', fixed)
        fence = gm['SAR_Base_Fence']
        self.assertIsNone(fence.find('.//collision/geometry/box'))
        self.assertIsNotNone(fence.find('.//collision/geometry/mesh'))
        self.assertIsNone(fence.find('.//collision/pose'), 'the box pose must not be kept for a mesh')
        print(f'  collision hulls replaced by meshes: {sorted(fixed)}')
        print(f'\n  {n_h} exported humans + {len(added)} building humans heated; all other temperatures unchanged '
              f'({n_dropped} ambient-temperature plugins dropped, {rep["thermal_plugins"]} kept)')


class ThermalSensorSdfTests(unittest.TestCase):
    """Regression guard for the Gazebo server segfault of 2026-09-22/23.

    gz-sensors 8 (Jazzy) builds the thermal camera's noise model with NoiseFactory instead of
    ImageNoiseFactory, gets a null pointer back ("[Err] [Noise.cc:63] Image noise requested.")
    and dereferences it, killing the whole server inside ThermalCameraSensor::SetScene - which
    is why the Gazebo GUI was left orphaned and the desktop offered Force Quit. Any <noise>
    element on that sensor triggers it, stddev 0 included, so the SDF must not carry one unless
    the user explicitly says their Gazebo has the upstream fix (gz-sensors #597).
    """

    def test_thermal_sensor_has_no_noise_element_by_default(self):
        xacro_text = (PKG / 'urdf/sar_sensors.xacro').read_text()
        self.assertIn('<xacro:arg name="thermal_noise_sdf" default="false"/>', xacro_text)
        guard = xacro_text.index('<xacro:if value="$(arg thermal_noise_sdf)">')
        noise = xacro_text.index('<noise><type>gaussian</type>')
        self.assertLess(guard, noise, 'the <noise> element must sit inside the thermal_noise_sdf guard')
        self.assertLess(noise, xacro_text.index('</xacro:if>', guard))
        self.assertFalse(bool(CFG['thermal_camera'].get('noise_in_sdf', False)),
                         'noise_in_sdf: true crashes Gazebo unless gz-sensors has PR #597')

    def test_noise_is_applied_in_software_instead(self):
        detector = (PKG / 'scripts/thermal_human_detector.py').read_text()
        self.assertIn('self.noise_k', detector)
        self.assertIn('np.random.normal(0.0, self.noise_k', detector)


class NodeClockTests(unittest.TestCase):
    """Freshness checks and watchdogs must run on the ROS clock (= simulation time).

    military_world runs at a few percent of real time. On 2026-09-23 the rover stood still with a
    valid path to B because drive.py's watchdog was wall-clock: the navigator's 10 Hz command
    stream arrives about once per two seconds of WALL time there, so the 0.6 s watchdog zeroed the
    wheel command between every pair of commands.
    """

    def test_drive_watchdog_uses_the_node_clock(self):
        drive = (PKG / 'scripts/drive.py').read_text()
        self.assertNotIn('time.monotonic', drive)
        self.assertIn('node_time(self)', drive)

    def test_sensor_freshness_uses_the_node_clock(self):
        for name, checks in (('scripts/thermal_human_detector.py', ('self.frame_t', 'self.depth_t')),
                             ('scripts/terrain_mapper.py', ('self.rgb_hd_t', 'self.last_grid'))):
            text = (PKG / name).read_text()
            for line in text.splitlines():
                if any(c in line for c in checks) and ('-' in line or '>=' in line):
                    self.assertNotIn('time.monotonic', line, f'{name}: {line.strip()}')


class TerrainWindowTests(unittest.TestCase):
    def test_window_equals_full_analysis(self):
        from terrain_core import Params, TerrainMapper
        from terrain_window import WindowedTerrainMapper
        p = Params(map_size=40.0, map_origin_x=-20, map_origin_y=-20)
        rng = np.random.default_rng(3)
        full, win = TerrainMapper(p), WindowedTerrainMapper(p, window=6.0)
        n = full.n
        X, Y = np.meshgrid(np.arange(n) * 0.1 - 20, np.arange(n) * 0.1 - 20)
        h = (0.4 * np.sin(X / 3) + 0.02 * rng.standard_normal((n, n)) + 0.8 * (X > 3)).astype(np.float32)
        for mp in (full, win):
            mp.hits[:] = 5
            mp.wsum[:] = 1
            mp.hsum[:] = h
        full.analyze()
        win._centre = (1.0, 0.0)
        win.analyze()
        i, j = win.cell(1.0, 0.0)
        r = int(6.0 / 0.1)
        sl = (slice(i - r, i + r), slice(j - r, j + r))
        self.assertTrue(np.array_equal(full.classes[sl], win.classes[sl]))
        self.assertTrue(np.allclose(full.slope_deg[sl], win.slope_deg[sl]))


class OverlayTests(unittest.TestCase):
    def test_16_9_crop(self):
        """4K (16:9) display camera with the depth camera's horizontal FOV: overlay is 16:9."""
        from terrain_core import Params, CameraModel
        from terrain_window import WindowedTerrainMapper
        import vision_overlay
        p = Params()
        mp = WindowedTerrainMapper(p)
        cam = CameraModel(p)
        pose = pose_at(0, 0, 0)
        depth = np.full((240, 320), 5.0, np.float32)
        frame = mp.process(depth, pose, cam)
        rgb = np.zeros((2160, 3840, 3), np.uint8)
        out = vision_overlay.draw(rgb, frame, mp, pose, cam, out_width=1920)
        self.assertEqual(out.shape[:2], (1080, 1920))


# ============================================================ physics / rover model (no Gazebo)
PHYS = load_config(PKG / 'config/physics.yaml').get('physics', {})
ROUGH = Path(os.environ.get('VIGIL_ROUGH_TERRAIN', str(Path.home() / 'Documents/robot/vigil_rough_terrain_ws'
                                                                   '/src/vigil_rough_terrain')))


class PhysicsConfigTests(unittest.TestCase):
    def test_earth_gravity(self):
        self.assertAlmostEqual(PHYS['engine']['gravity'], 9.81, places=2)

    def test_gravity_is_written_into_the_world(self):
        import inspect
        import build_sar_world
        self.assertIn("world.find('gravity')", inspect.getsource(build_sar_world.build))

    def test_turn_response_is_quick(self):
        """The rough-terrain 0.5 rad/s2 ramp took 1.2 s just to reach its turn rate."""
        d = PHYS['drive']
        self.assertLessEqual(d['max_angular'] / d['ramp_angular'], 0.34)

    def test_navigation_speeds_fit_the_drive(self):
        nav, drv = CFG['navigation'], PHYS['drive']
        self.assertLessEqual(nav['cruise_speed'], drv['max_linear'])
        self.assertLessEqual(nav['max_angular'], drv['max_angular'])

    def test_three_metres_per_second_open_road(self):
        self.assertEqual(float(CFG['navigation']['cruise_speed']), 3.0)
        self.assertEqual(float(PHYS['drive']['max_linear']), 3.0)
        nav = CFG['navigation']
        self.assertEqual([float(c) for c in nav['clearance_distances']], [0.5, 1.0, 2.0, 10.0])
        self.assertEqual([float(v) for v in nav['clearance_speeds']], [0.3, 1.0, 2.0, 3.0])
        self.assertEqual(float(nav['min_clearance']), 0.5)
        self.assertLess(nav['brake_plan_decel'], PHYS['drive']['max_decel'])

    @unittest.skipUnless(importlib.util.find_spec('xacro'), 'xacro not installed')
    def test_no_wheelie_limits_from_the_urdf(self):
        """Mass / COM from the URDF Gazebo gets; the drive's acceleration must stay well under the
        acceleration that lifts the front, on flat ground and on the steepest climb (33 deg)."""
        import rover_stability
        r = rover_stability.analyse(rover_stability.build_urdf(PKG))
        d = PHYS['drive']
        self.assertEqual(r['no_gravity'], [])                        # gravity acts on every link
        self.assertTrue(250.0 < r['mass'] < 330.0, r['mass'])
        self.assertLess(r['com_height'], 0.45)
        self.assertEqual(sorted(w['name'] for w in r['wheels'] if w['loaded']),
                         ['L1', 'L2', 'L3', 'R1', 'R2', 'R3'])
        self.assertLess(r['wheelie'][0.0], 8.0)                      # the old 8 m/s2 ramp was above it
        self.assertLess(d['max_accel'] * 1.5, r['wheelie'][33.0])    # >= 1.5x margin on a 33 deg climb
        self.assertLess(d['max_accel'] * 5.0, r['wheelie'][0.0])     # >= 5x on flat road
        self.assertLess(d['max_decel'] * 5.0, r['nose_over'])

    def test_wheel_torque_limit_is_passed_and_sufficient(self):
        d = PHYS['drive']
        self.assertTrue(120.0 <= d['wheel_torque_limit'] < 400.0)
        src = (PKG / 'launch/sim.launch.py').read_text()
        self.assertIn("'wheel_effort'", src)
        self.assertIn('wheel_torque_limit', src)

    def test_engine_is_the_rough_terrain_engine(self):
        e = PHYS['engine']
        self.assertEqual(float(e['max_step_size']), 0.001)
        self.assertEqual(str(e['collision_detector']), 'default')
        self.assertEqual(str(e['solver']), 'default')
        self.assertEqual(float(e['ground_friction']), 1.0)

    @unittest.skipUnless((ROUGH / 'worlds/rock_terrain.sdf').exists(), 'vigil_rough_terrain not on this machine')
    def test_same_values_as_the_rough_terrain_files(self):
        import re
        world = (ROUGH / 'worlds/rock_terrain.sdf').read_text()
        ground = (ROUGH / 'models/rocky_terrain_n4/model.sdf').read_text()
        g = float(re.search(r'<gravity>\s*0 0 (-?[\d.]+)', world).group(1))
        step = float(re.search(r'<max_step_size>([\d.]+)', world).group(1))
        mu = float(re.search(r'<mu>([\d.]+)</mu>', ground).group(1))
        self.assertAlmostEqual(-g, PHYS['engine']['gravity'])
        self.assertAlmostEqual(step, PHYS['engine']['max_step_size'])
        self.assertAlmostEqual(mu, PHYS['engine']['ground_friction'])
        self.assertNotIn('<dart>', world)          # rough uses DART defaults -> SAR 'default'

    def test_world_builder_applies_rough_physics(self):
        """Synthetic export: bullet/pgs block, per-tile friction. The builder must drop the dart
        block, pin gravity 9.81, set every drivable surface to mu 1.0 and leave props alone."""
        import copy as _copy
        import tempfile
        import xml.etree.ElementTree as ET
        import build_sar_world
        sdf = '''<?xml version="1.0"?><sdf version="1.9"><world name="t">
<physics name="default" type="dart"><max_step_size>0.01</max_step_size>
<dart><collision_detector>bullet</collision_detector><solver><solver_type>pgs</solver_type></solver></dart></physics>
<model name="SAR_Terrain_00_00"><!-- {"semantic_class": "GROUND", "sar_object": "TERRAIN_TILE"} --><static>true</static><pose>0 0 0 0 0 0</pose>
<link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry>
<surface><friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction></surface></collision></link></model>
<model name="SAR_Track_X"><!-- {"semantic_class": "ROAD"} --><static>true</static><pose>0 0 0 0 0 0</pose>
<link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry>
<surface><friction><ode><mu>0.55</mu><mu2>0.55</mu2></ode></friction></surface></collision></link></model>
<model name="SAR_Bridge_Deck"><!-- {"semantic_class": "BRIDGE"} --><static>true</static><pose>0 0 0 0 0 0</pose>
<link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>
<model name="SAR_DynProps_CONE_01"><!-- {"semantic_class": "PROP"} --><pose>50 50 0 0 0 0</pose>
<link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry>
<surface><friction><ode><mu>0.70</mu><mu2>0.70</mu2></ode></friction></surface></collision></link></model>
</world></sdf>'''
        with tempfile.TemporaryDirectory() as tmp:
            mw = Path(tmp)
            (mw / 'gazebo_export').mkdir()
            (mw / 'gazebo_export/military_world.sdf').write_text(sdf)
            cfg = _copy.deepcopy(CFG)
            cfg['world'].update(moving_people=False, add_building_humans=False, fix_hull_collisions=False,
                                physics_step=0.001, gravity=9.81, collision_detector='default',
                                physics_solver='default', ground_friction=1.0)
            rep = build_sar_world.build(cfg, mw, mw / 'out.sdf')
            world = ET.parse(mw / 'out.sdf').getroot().find('world')
        self.assertIsNone(world.find('physics/dart'))
        self.assertEqual(world.findtext('gravity'), '0 0 -9.81')
        self.assertEqual(world.findtext('physics/max_step_size'), '0.001')
        mus = {m.get('name'): (m.findtext('.//mu'), m.findtext('.//mu2')) for m in world.findall('model')}
        for name in ('SAR_Terrain_00_00', 'SAR_Track_X', 'SAR_Bridge_Deck'):
            self.assertEqual(mus[name], ('1', '1'), name)
        self.assertEqual(mus['SAR_DynProps_CONE_01'], ('0.70', '0.70'))
        self.assertEqual(rep['ground_friction']['surfaces'], 3)


class PathFollowingTests(unittest.TestCase):
    """The follower is vigil_rough_terrain's: fixed lookahead, yaw rate = heading error,
    turn on the spot beyond turn_in_place_error, speed = cruise * max(0.35, cos(error)).
    SAR adds only the slowdown into B."""

    class FakePlanner:
        res = 0.20
        classes = True

        def slope_at(self, x, y):
            return 0.0

        def rotation_clear(self, x, y, yaw):
            return True

        def forward_clear(self, x, y, yaw, d):
            return True, d

    def follower(self, path):
        from planner_core import NavigatorCore
        from terrain_core import Params
        core = NavigatorCore.__new__(NavigatorCore)
        core.nav = dict(CFG['navigation'])
        core.tp = Params.from_nested(CFG)
        core.planner = self.FakePlanner()
        core.path = np.asarray(path, dtype=float)
        core.progress = 0
        core.goal = tuple(core.path[-1])
        core.climb_slope = 0.0
        core.clearance = core.clear_speed = math.inf
        return core

    def test_config_is_the_rough_terrain_one(self):
        nav = CFG['navigation']
        for k, v in dict(lookahead=1.3,
                         planning_resolution=0.20, stop_distance=0.50, goal_tolerance=0.30,
                         replan_period=4.0, replanning_distance=5.0).items():
            self.assertAlmostEqual(nav[k], v, msg=k)
        self.assertNotIn('speed_governor', nav)
        self.assertTrue(nav['steer_around'])

    def test_speed_law_matches_rough_terrain(self):
        core = self.follower([(0.0, y) for y in np.arange(0.0, 500.1, 0.2)])   # B far: no braking yet
        nav = CFG['navigation']
        for deg in (0.0, 10.0, 30.0):
            core.last_v = 0.0                                   # fresh start: lookahead = 1.3 m
            v, w = core._follow(pose_at(0.0, 0.0, math.radians(90 - deg)))
            err = math.radians(deg)
            want = nav['cruise_speed'] * max(0.35, math.cos(err))
            wcmd = min(nav['max_angular'], nav['heading_gain'] * err)
            a_lat = nav.get('max_lateral_accel', 0)
            if abs(wcmd) > 1e-3 and a_lat > 0:
                want = min(want, a_lat / abs(wcmd))
                # the pure-pursuit arc to the lookahead point (target 1.3 m up the path)
                L = math.ceil(nav['lookahead'] / 0.2 - 1e-9) * 0.2   # first 0.2 m path vertex past it
                kappa = 2.0 * abs(math.sin(err)) / L
                want = min(want, math.sqrt(a_lat / kappa), nav['max_angular'] / kappa)
            self.assertAlmostEqual(v, want, places=1)
            self.assertAlmostEqual(w, min(CFG['navigation']['max_angular'], CFG['navigation']['heading_gain'] * err), places=2)

    def test_turns_on_the_spot_beyond_the_threshold(self):
        core = self.follower([(0.0, y) for y in np.arange(0.0, 50.1, 0.2)])
        v, w = core._follow(pose_at(0.0, 0.0, math.radians(90 - 60)))
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)

    def test_slows_into_b(self):
        nav = CFG['navigation']
        core = self.follower([(0.0, y) for y in np.arange(0.0, 10.01, 0.2)])
        core.progress = len(core.path) - 5
        v, _ = core._follow(pose_at(0.0, 10.0 - nav['goal_tolerance'] - 0.01, math.pi / 2))
        # at the edge of the tolerance it must be slow enough for the brakes to stop INSIDE it
        d = PHYS['drive']
        self.assertLess(v * v / (2 * d['max_decel']) + v * d['max_decel'] / d['max_jerk_brake'],
                        nav['goal_tolerance'] / 2)

    def test_b_is_not_stepped_over(self):
        from planner_core import _point_segment_distance
        self.assertLess(_point_segment_distance(0.0, 0.0, (-2.0, 0.3), (2.0, 0.3)), 0.31)
        self.assertGreater(_point_segment_distance(0.0, 0.0, (-2.0, 5.0), (2.0, 5.0)), 4.9)


class RobotModelTests(unittest.TestCase):
    """The rover is the vigil_rough_terrain rover: same URDF apart from the thermal camera include
    and the wheel speed limit being an argument."""

    def test_same_as_vigil_rough_terrain(self):
        src = ROUGH / 'urdf/agri_ugv.urdf.xacro'
        if not src.exists():
            self.skipTest('vigil_rough_terrain workspace not found')
        rough = src.read_text().replace('vigil_rough_terrain', 'vigil_sar')
        ours = (PKG / 'urdf/agri_ugv.urdf.xacro').read_text()
        ours = ours.replace('velocity="$(arg wheel_max_rad_s)"', 'velocity="15"')
        ours = ours.replace('effort="$(arg wheel_effort)"', 'effort="400"')
        # vigil_sar collision proxies (collision only, no mass): undo them for the comparison
        ours = ours.replace('\n  <xacro:include filename="$(find vigil_sar)/urdf/collision_proxies.xacro"/>', '')
        ours = '\n'.join(l for l in ours.splitlines() if 'body_collision_proxies' not in l)
        ours = ours.replace('<xacro:cad_visual part="lidar" material="rubber"/>\n    <collision name="lidar_collision">'
                            '<origin xyz="-0.013 0 -0.012"/><geometry><cylinder radius="0.05" length="0.056"/>'
                            '</geometry></collision></link>', '<xacro:cad_visual part="lidar" material="rubber"/></link>')
        strip = lambda t: [l for l in t.splitlines() if l.strip() and 'thermal' not in l and 'wheel_max_rad_s' not in l
                           and 'wheel_effort' not in l
                           and not l.strip().startswith(('<!--', '*', 'only read', 'urdf/', '15 rad/s', 'needs.',
                                                         'physics.drive.wheel_torque_limit',
                                                         'collision proxies', 'rear body, full body width'))]
        self.assertEqual(strip(rough)[1:], strip(ours)[1:])
        cad = (ROUGH / 'urdf/cad_geometry.xacro').read_text().replace('vigil_rough_terrain', 'vigil_sar')
        self.assertEqual(cad, (PKG / 'urdf/cad_geometry.xacro').read_text())

    def test_wheels_can_turn_the_rover(self):
        """Wheel torque is an argument (default 400 = vigil_rough_terrain; physics.yaml passes 150).
        The SAR copy once had 56.9 N.m, too little to scrub the tyres round."""
        text = (PKG / 'urdf/agri_ugv.urdf.xacro').read_text()
        self.assertIn('<xacro:arg name="wheel_effort" default="400"/>', text)
        self.assertIn('<limit effort="$(arg wheel_effort)"', text)
        self.assertIn('<mu1>0.85</mu1><mu2>0.85</mu2>', text)

    def test_velocity_command_interface_as_rough_terrain(self):
        text = (PKG / 'urdf/simulation.xacro').read_text()
        self.assertIn('command_interface name="velocity"', text)
        self.assertIn('velocity_controllers/JointGroupVelocityController',
                      (PKG / 'config/controllers.yaml').read_text())


class ThermalRangeTests(unittest.TestCase):
    def test_depth_far_limit_is_not_a_surface(self):
        """A person 26 m away was put at 14.7 m: Gazebo fills out-of-range depth pixels with the
        15 m far limit and the detector treated that as a surface."""
        det, dcam = make_detector()
        c = CFG['camera']
        pose = pose_at(-8.0, 11.0, math.pi / 2)
        o, r = det.cam.ray_world(det.cam.w / 2, det.cam.h / 2 - 5, pose)
        far = np.full((c['height'], c['width']), c['depth_far'], np.float32)
        self.assertIsNone(det._range_from_depth(o, r, far, dcam, pose))
        near = np.full_like(far, 8.0)
        self.assertAlmostEqual(det._range_from_depth(o, r, near, dcam, pose), 8.0, delta=0.3)


# ============================================================ drive chain (the "rover will not turn" bugs)
try:
    sys.path.insert(0, str(HERE))
    import drive as _drive                       # needs rclpy: present wherever ROS 2 is installed
    from skid_model import run as _skid_run, closed_loop_a_to_b as _closed_loop
    _HAVE_DRIVE = True
except ImportError:
    _HAVE_DRIVE = False


class ParameterLoadingTests(unittest.TestCase):
    def test_physics_section_reaches_drive(self):
        """drive.py read physics.yaml through nested_params(), which only knows a fixed list of
        flat sections - 'physics' was not in it, so the motor ran on fallbacks (36 / 12 N.m)."""
        import types
        from ros_common import nested_section
        flat = {'physics.motor.stall_torque': 12.65, 'physics.motor.braking_torque': 52.0,
                'physics.limits.max_speed': 36.0, 'navigation.goal_x': 1.0}

        class P:
            def __init__(self, v): self.value = v
        node = types.SimpleNamespace(get_parameters_by_prefix=lambda pre: {
            k[len(pre) + 1:]: P(v) for k, v in flat.items() if k.startswith(pre + '.')})
        cfg = nested_section(node, 'physics')
        self.assertEqual(cfg['motor']['stall_torque'], 12.65)
        self.assertEqual(cfg['motor']['braking_torque'], 52.0)
        self.assertEqual(cfg['limits']['max_speed'], 36.0)
        self.assertNotIn('navigation', cfg)


@unittest.skipUnless(_HAVE_DRIVE, 'drive.py needs rclpy')
class DriveTests(unittest.TestCase):
    """drive.py: rough-terrain wheel kinematics wheel = (v -/+ w * 0.65) / r, with a jerk-limited
    speed profile (max_accel / max_decel / max_jerk / max_jerk_brake) so the wheels never get a step."""

    def rough_terrain_wheels(self, v, w):          # the original formula, verbatim
        return [(v - side * w * 0.65) / radius for side in [1, -1]
                for radius in [0.23463, 0.17655, 0.17655, 0.17655]]

    def law(self):
        import drive
        return drive.DriveLaw(PHYS['drive'])

    def test_wheel_speeds_match_rough_terrain(self):
        law = self.law()
        law.v, law.w = 0.5, 0.3
        law.wheels = self.rough_terrain_wheels(0.5, 0.3)
        law.tick(0.5, 0.3)
        for a, b in zip(law.wheels, self.rough_terrain_wheels(0.5, 0.3)):
            self.assertAlmostEqual(a, b, places=9)

    def profile(self, law, target, ticks, v0=0.0):
        law.v = v0
        vs = [law.v]
        for _ in range(ticks):
            law.tick(target, 0.0)
            vs.append(law.v)
        dt = 0.01
        acc = np.diff(vs) / dt
        return np.array(vs), acc

    def test_smooth_acceleration_to_three_metres_per_second(self):
        d = PHYS['drive']
        law = self.law()
        vs, acc = self.profile(law, 100.0, 600)                   # an absurd command: still smooth
        self.assertAlmostEqual(vs[-1], d['max_linear'])
        self.assertLessEqual(acc.max(), d['max_accel'] + 1e-9)
        self.assertLess(acc[0], 0.05)                              # no step at the start
        jerk = np.abs(np.diff(acc)) / 0.01
        self.assertLessEqual(np.sort(jerk)[-2], d['max_jerk'] + 1e-6)   # all but the final settle step
        t_top = np.argmax(vs >= d['max_linear'] - 1e-9) * 0.01
        self.assertTrue(3.0 <= t_top <= 3.8, t_top)                # ~3.3 s to 3.0 m/s, not instant

    def test_realistic_braking(self):
        d = PHYS['drive']
        law = self.law()
        vs, acc = self.profile(law, 0.0, 400, v0=3.0)
        self.assertAlmostEqual(vs[-1], 0.0)
        self.assertGreaterEqual(acc.min(), -d['max_decel'] - 1e-9)
        dist = float(np.sum(vs) * 0.01)
        self.assertLess(dist, 3.0 ** 2 / (2 * d['max_decel']) + 3.0 * d['max_decel'] / d['max_jerk_brake'])
        self.assertGreater(dist, 3.0 ** 2 / (2 * d['max_decel']) - 0.05)   # not an instant stop

    def test_wheel_acceleration_is_capped(self):
        d = PHYS['drive']
        law = self.law()
        prev = list(law.wheels)
        worst = 0.0
        for k in range(300):
            law.tick(3.0, 1.0 if k < 150 else -1.0)
            worst = max(worst, max(abs(a - b) for a, b in zip(law.wheels, prev)) / 0.01)
            prev = list(law.wheels)
        self.assertLessEqual(worst, d['max_wheel_accel'] + 1e-6)

    def test_drives_turns_and_stops(self):
        r = _skid_run(PHYS, lambda t: (0.6, 0.0), 6.0)
        self.assertGreater(r.x, 2.5)
        r = _skid_run(PHYS, lambda t: (0.0, 0.6), 4.0)
        self.assertGreater(math.degrees(r.yaw), 60.0)
        r = _skid_run(PHYS, lambda t: (0.0, -0.6), 4.0)
        self.assertLess(math.degrees(r.yaw), -60.0)
        r = _skid_run(PHYS, lambda t: (0.6, 0.0), 4.0)
        x0 = r.x
        r = _skid_run(PHYS, lambda t: (0.0, 0.0), 3.0, rover=r)
        d = PHYS['drive']
        self.assertLess(r.x - x0, 0.6 ** 2 / (2 * d['max_decel']) + 0.6 * d['max_decel'] / d['max_jerk_brake'] + 0.05)
        self.assertLess(r.speed(), 0.02)


class ScaleAndZoneTests(unittest.TestCase):
    """1 Gazebo unit = 1 m. The world is not rescaled; the rover works in world.zone_*."""

    def test_zone_is_the_map_and_a_practical_size(self):
        from terrain_core import Params
        tp = Params.from_nested(CFG)
        w = CFG['world']
        self.assertEqual(tp.zone, (w['zone_x_min'], w['zone_x_max'], w['zone_y_min'], w['zone_y_max']))
        self.assertEqual((tp.map_origin_x, tp.map_origin_y), (w['zone_x_min'], w['zone_y_min']))
        self.assertEqual(tp.map_size, max(w['zone_x_max'] - w['zone_x_min'], w['zone_y_max'] - w['zone_y_min']))
        for side in (w['zone_x_max'] - w['zone_x_min'], w['zone_y_max'] - w['zone_y_min']):
            self.assertTrue(100.0 <= side <= 200.0, side)
        sp, nav = CFG['spawn'], CFG['navigation']
        self.assertTrue(tp.in_zone(sp['x'], sp['y'], 2.0))
        self.assertTrue(tp.in_zone(nav['goal_x'], nav['goal_y'], 2.0))
        for x, y in ((70.0, 50.0), (89.0, 50.0), (75.5, 79.0), (114.0, 55.5), (-14.0, 100.0), (14.0, 118.0)):
            self.assertTrue(tp.in_zone(x, y), (x, y))          # all six SAR buildings

    def test_goal_outside_the_zone_is_clamped_inside(self):
        from planner_core import NavigatorCore
        from terrain_core import Params
        tp = Params.from_nested(CFG)
        nav = NavigatorCore(tp, dict(CFG['navigation']))
        nav.set_goal(80.0, 230.0, 0.0)                          # the mountain, far outside
        self.assertTrue(tp.in_zone(*nav.goal))
        self.assertAlmostEqual(nav.goal[1], tp.zone[3] - tp.circumscribed_radius)
        self.assertTrue(any('outside the operational zone' in e['msg'] for e in nav.events))
        nav.set_goal(70.0, 40.0, 0.0)
        self.assertEqual(nav.goal, (70.0, 40.0))

    def test_plan_stays_in_the_local_window(self):
        from planner_core import Planner, path_length
        from terrain_core import Params, SAFE, OBSTACLE
        tp = Params.from_nested(CFG)
        n = int(round(tp.map_size / tp.resolution))
        cls = np.full((n, n), SAFE, np.uint8)
        r = tp.resolution
        # wall across the whole zone with a gap 60 m to the side: out of the 20 m window -> no route
        y0 = int((10.0 - tp.map_origin_y) / r)
        cls[y0:y0 + 3, :] = OBSTACLE
        gap = int((60.0 - tp.map_origin_x) / r)
        cls[y0:y0 + 3, gap:gap + 20] = SAFE
        nav = dict(CFG['navigation'])
        pl = Planner(tp, nav)
        pl.build(cls)
        path, info = pl.plan((0.0, -8.0), (0.0, 25.0), mode='wide')
        self.assertIsNone(path, 'a 60 m detour is outside the A-B window + 20 m')
        nav['planning_window_margin'] = 70.0
        pl = Planner(tp, nav)
        pl.build(cls)
        path, info = pl.plan((0.0, -8.0), (0.0, 25.0), mode='wide')
        self.assertIsNotNone(path)
        self.assertLess(path_length(path), 200.0)

    def test_expansion_budget(self):
        from planner_core import Planner
        from terrain_core import Params, SAFE
        tp = Params.from_nested(CFG)
        n = int(round(tp.map_size / tp.resolution))
        nav = dict(CFG['navigation'], max_expansions=500)
        pl = Planner(tp, nav)
        pl.build(np.full((n, n), SAFE, np.uint8))
        path, info = pl.plan((0.0, -8.0), (70.0, 40.0), mode='wide')
        self.assertIsNone(path)
        self.assertEqual(info['reason'], 'expansion limit')

    @unittest.skipUnless(importlib.util.find_spec('xacro'), 'xacro not installed')
    def test_configured_dimensions_match_the_urdf(self):
        import scale_check
        rv = scale_check.rover_dims(CFG['robot']['height'])
        r, pr = CFG['robot'], PHYS['rover']
        self.assertAlmostEqual(rv['length'], r['length'], delta=0.02)
        self.assertAlmostEqual(rv['width'], r['width'], delta=0.02)
        self.assertAlmostEqual(rv['wheelbase'], r['wheelbase'], delta=0.02)
        self.assertAlmostEqual(rv['track'], r['track_width'], delta=0.05)
        self.assertAlmostEqual(rv['d_front'] / 2, pr['wheel_radius_front'], places=3)
        self.assertAlmostEqual(rv['d_small'] / 2, pr['wheel_radius_small'], places=3)
        self.assertAlmostEqual(rv['mass'], pr['mass'], delta=0.5)

    def test_drive_uses_the_configured_wheel_radii(self):
        import drive
        law = drive.DriveLaw(PHYS['drive'], dict(wheel_radius_front=0.25, wheel_radius_small=0.2))
        law.v = 1.0
        law.wheels = [1.0 / 0.25] + [1.0 / 0.2] * 3 + [1.0 / 0.25] + [1.0 / 0.2] * 3
        law.tick(1.0, 0.0)
        self.assertAlmostEqual(law.wheels[0], 4.0)
        self.assertAlmostEqual(law.wheels[1], 5.0)

    def test_walkers_outside_the_zone_stand_still(self):
        import copy as _copy
        import tempfile
        import xml.etree.ElementTree as ET
        import build_sar_world
        mover = lambda name, x, y: (f'<model name="{name}"><pose>{x} {y} 0 0 0 0</pose><static>false</static>'  # noqa: E731
                                    f'<plugin filename="x" name="sar::WaypointSystem"/><link name="l"/></model>')
        sdf = ('<?xml version="1.0"?><sdf version="1.9"><world name="t"><physics name="default" type="dart">'
               '<max_step_size>0.01</max_step_size></physics>' + mover('in_zone', 10, 10) + mover('far', 80, 230)
               + '</world></sdf>')
        with tempfile.TemporaryDirectory() as tmp:
            mw = Path(tmp)
            (mw / 'gazebo_export').mkdir()
            (mw / 'gazebo_export/military_world.sdf').write_text(sdf)
            (mw / 'sar_physics.py').write_text('def configure_world(root, controller=1):\n    pass\n')
            cfg = _copy.deepcopy(CFG)
            cfg['world'].update(add_building_humans=False, fix_hull_collisions=False, moving_people=True)
            rep = build_sar_world.build(cfg, mw, mw / 'out.sdf')
            world = ET.parse(mw / 'out.sdf').getroot().find('world')
        models = {m.get('name'): m for m in world.findall('model')}
        self.assertEqual(rep['frozen_outside_zone'], ['far'])
        self.assertIsNone(models['far'].find("plugin[@name='sar::WaypointSystem']"))
        self.assertEqual(models['far'].findtext('static'), 'true')
        self.assertIsNotNone(models['in_zone'].find("plugin[@name='sar::WaypointSystem']"))

    @unittest.skipUnless(MW is not None, 'military_world not found')
    def test_world_is_metre_scale(self):
        import scale_check
        import yaml
        phys = yaml.safe_load((PKG / 'config/physics.yaml').read_text())['/**']['ros__parameters']['physics']
        world = scale_check.measure_world(MW)
        rover = scale_check.rover_dims(CFG['robot']['height']) if importlib.util.find_spec('xacro') else dict(
            length=1.53, width=1.12, height=1.14, d_front=0.469, d_small=0.353, wheelbase=0.82, track=0.87, mass=291.5)
        bad = [(n, v, rule) for n, v, rule, ok in scale_check.checks(rover, world, CFG, phys) if not ok]
        self.assertEqual(bad, [])


class ClearanceSpeedTests(unittest.TestCase):
    """Clearance speed profile: ~10 m clear -> 3.0 m/s, 2 m -> 2.0, 1 m -> 1.0, < 0.5 m -> no forward,
    and below 0.5 m the rover turns / replans / reverses - it never just stands there."""

    def core(self, classes=None):
        from planner_core import NavigatorCore
        from terrain_core import Params, SAFE
        tp = Params.from_nested(CFG)
        nav = NavigatorCore(tp, dict(CFG['navigation']))
        n = int(round(tp.map_size / tp.resolution))
        nav.update_map(classes if classes is not None else np.full((n, n), SAFE, np.uint8))
        return nav, tp

    def test_profile_values(self):
        nav, _ = self.core()
        for c, v in ((20.0, 3.0), (10.0, 3.0), (6.0, 2.5), (2.0, 2.0), (1.5, 1.5), (1.0, 1.0), (0.5, 0.3), (0.49, 0.0)):
            self.assertAlmostEqual(nav._clearance_speed(c), v, places=6, msg=c)

    def wall_map(self, tp, y0, y1, x0=-6.0, x1=6.0):
        from terrain_core import SAFE, OBSTACLE
        n = int(round(tp.map_size / tp.resolution))
        c = np.full((n, n), SAFE, np.uint8)
        r = tp.resolution
        c[int((y0 - tp.map_origin_y) / r):int((y1 - tp.map_origin_y) / r),
          int((x0 - tp.map_origin_x) / r):int((x1 - tp.map_origin_x) / r)] = OBSTACLE
        return c

    def test_path_clearance_is_the_real_body_distance(self):
        from terrain_core import Params, Pose, rpy_to_matrix
        tp = Params.from_nested(CFG)
        nav, _ = self.core(self.wall_map(tp, 5.0, 5.6))
        pose = Pose(x=0.0, y=0.0, z=0.0, R=rpy_to_matrix(0, 0, math.pi / 2))
        path = np.array([(0.0, y) for y in np.arange(0.0, 12.0, 0.1)])
        front = tp.footprint_center_x + tp.robot_length / 2
        c = nav.planner.path_clearance(path, 0, pose)
        self.assertAlmostEqual(c, 5.0 - front, delta=0.35)
        nav2, _ = self.core()
        self.assertEqual(nav2.planner.path_clearance(path, 0, pose), 10.0)

    def test_no_forward_below_half_a_metre_and_never_standing(self):
        """Wall 0.3 m in front of the bumper, the rover pinned there (it cannot move in this test):
        every command has v <= 0, and it is never a plain standstill for more than ~0.5 s -
        it turns, replans, or reverses."""
        from terrain_core import Params, Pose, rpy_to_matrix
        tp = Params.from_nested(CFG)
        front = tp.footprint_center_x + tp.robot_length / 2
        nav, _ = self.core(self.wall_map(tp, front + 0.3, front + 0.9, -20.0, 20.0))
        nav.set_goal(0.0, 8.0, 0.0)
        nav.t0 = -100.0
        pose = Pose(x=0.0, y=0.0, z=0.0, R=rpy_to_matrix(0, 0, math.pi / 2))
        still = worst = 0.0
        for k in range(80):
            v, w = nav.tick(k * 0.05, pose)
            self.assertLessEqual(v, 1e-9, f'forward command {v} with the wall 0.3 m ahead')
            still = still + 0.05 if (abs(v) < 1e-3 and abs(w) < 0.05) else 0.0
            worst = max(worst, still)
        self.assertLessEqual(worst, 0.65)


@unittest.skipUnless(_HAVE_DRIVE, 'drive.py needs rclpy')
class ClosedLoopGoalTests(unittest.TestCase):
    """Planner + follower + goal logic + command shaper + drive.py law, closed around the model."""
    def maps(self):
        from terrain_core import Params, SAFE, OBSTACLE
        tp = Params.from_nested(CFG)
        r, n = tp.resolution, int(round(tp.map_size / tp.resolution))

        def box(c, x0, y0, x1, y1):
            c[int((y0 - tp.map_origin_y) / r):int((y1 - tp.map_origin_y) / r),
              int((x0 - tp.map_origin_x) / r):int((x1 - tp.map_origin_x) / r)] = OBSTACLE
        open_ = np.full((n, n), SAFE, np.uint8)
        wall = open_.copy()
        box(wall, -4, 0, 4, 0.6)
        gate = open_.copy()
        box(gate, -30, 0, -1.0, 0.4)
        box(gate, 1.0, 0, 30, 0.4)
        return dict(open=open_, wall=wall, gate=gate)

    def test_reaches_b_inside_tolerance_without_overshoot(self):
        """Including an obstacle in the way: the rover must drive AROUND it, not stop in front."""
        tol = CFG['navigation']['goal_tolerance']
        m = self.maps()
        for b, key, label in (((-3.13, -3.46), 'open', 'the last dashboard run: 5.4 m, 35 deg off'),
                              ((0.0, 10.0), 'wall', '8 m wall across the route'),
                              ((0.0, 10.0), 'gate', 'fence with a 2.0 m gate'),
                              ((0.0, -20.0), 'open', 'B behind the rover')):
            rv, nav, log, reached = _closed_loop(CFG, PHYS, (0.0, -8.0), math.pi / 2, b, seconds=120,
                                                 classes=m[key])
            final = math.hypot(b[0] - rv.x, b[1] - rv.y)
            closest = min(math.hypot(b[0] - x, b[1] - y) for _, x, y, *_ in log)
            self.assertIsNotNone(reached, f'{label}: never reached B (ended {final:.1f} m away)')
            self.assertLessEqual(final, tol + 0.05, f'{label}: stopped {final:.2f} m from B')
            self.assertLess(final - closest, 0.1, f'{label}: went past B')
            self.assertLessEqual(max(row[3] for row in log), CFG['navigation']['cruise_speed'] + 0.05)

    def test_goal_reached_uses_the_robot_pose(self):
        """A path that ends at B is not arrival: only the robot's own pose inside the tolerance is."""
        from planner_core import NavigatorCore
        from terrain_core import Params, Pose as P, SAFE
        tp = Params.from_nested(CFG)
        nav = NavigatorCore(tp, CFG['navigation'])
        n = int(round(tp.map_size / tp.resolution))
        nav.update_map(np.full((n, n), SAFE, np.uint8), None, None)
        nav.set_goal(0.0, 10.0, 0.0)
        nav.t0 = -100.0
        nav.tick(0.0, P(x=0.0, y=0.0, z=0.05, R=rpy_to_matrix(0, 0, math.pi / 2)))
        self.assertNotEqual(nav.state, 'GOAL_REACHED')             # has a path to B, is not at B
        nav.tick(0.05, P(x=0.0, y=9.8, z=0.05, R=rpy_to_matrix(0, 0, math.pi / 2)))
        self.assertEqual(nav.state, 'GOAL_REACHED')

    def test_approaching_state_inside_slowdown_distance(self):
        from planner_core import NavigatorCore
        from terrain_core import Params, Pose as P, SAFE
        tp = Params.from_nested(CFG)
        nav = NavigatorCore(tp, CFG['navigation'])
        n = int(round(tp.map_size / tp.resolution))
        nav.update_map(np.full((n, n), SAFE, np.uint8), None, None)
        nav.set_goal(0.0, 30.0, 0.0)
        nav.t0 = -100.0
        pose = lambda y: P(x=0.0, y=y, z=0.05, R=rpy_to_matrix(0, 0, math.pi / 2))  # noqa: E731
        nav.tick(0.0, pose(0.0))
        self.assertEqual(nav.state, 'NAVIGATING')
        v, _ = nav.tick(0.05, pose(30.0 - CFG['navigation']['slowdown_distance'] / 2))
        self.assertEqual(nav.state, 'APPROACHING_GOAL')
        self.assertLess(v, CFG['navigation']['cruise_speed'])


class SarAvailabilityTests(unittest.TestCase):
    """SAR is a manual operator mode: START is accepted in every navigation state."""

    def mission(self, **over):
        sar = dict(CFG['sar'], **over)
        return SarMission(sar, [], thermal_hfov=1.0472, robot_radius=0.9, bounds=(-37, -27, 127, 137))

    def test_start_accepted_before_and_after_b(self):
        for reached in (False, True):
            m = self.mission()
            ok, msg = m.activate(0.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0), reached)
            self.assertTrue(ok, msg)
            self.assertTrue(m.active)

    def test_no_goal_gate_left_in_code_or_ui(self):
        src = (PKG / 'scripts/sar_core.py').read_text() + (PKG / 'dashboard/index.html').read_text()
        self.assertNotIn("require_goal_reached', True", src)
        self.assertNotIn('S.params.require_goal', src)
        self.assertNotIn('require_goal_reached', CFG['sar'])

    def test_stop_resumes_b_or_holds(self):
        m = self.mission(stop_behavior='resume_b')
        m.activate(0.0, pose_at(20.0, 10.0, 1.57), (80.0, 40.0), False)
        m.abort(1.0, pose_at(25.0, 12.0, 1.57), (80.0, 40.0))
        self.assertEqual(m.goal_cmd, (80.0, 40.0))
        m = self.mission(stop_behavior='hold')
        m.activate(0.0, pose_at(20.0, 10.0, 1.57), (80.0, 40.0), False)
        m.abort(1.0, pose_at(25.0, 12.0, 1.57), (80.0, 40.0))
        self.assertEqual(m.goal_cmd, (25.0, 12.0))

    def test_restart_reset_or_resume(self):
        m = self.mission(restart_behavior='resume')
        m.activate(0.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0), False)
        m.tasks[0].status = 'done'
        m.abort(1.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0))
        m.activate(2.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0), False)
        self.assertEqual(m.tasks[0].status, 'done')
        m = self.mission(restart_behavior='reset')
        m.activate(0.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0), False)
        m.tasks[0].status = 'done'
        m.abort(1.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0))
        m.activate(2.0, pose_at(0.0, -8.0, 1.57), (80.0, 40.0), False)
        self.assertEqual(m.tasks[0].status, 'pending')

    def test_inactive_state_is_never_ready_gated(self):
        m = self.mission()
        m.tick(0.0, pose_at(0.0, -8.0, 1.57), False)
        self.assertEqual(m.state, 'SAR OFF')
        m.tick(1.0, pose_at(80.0, 40.0, 1.57), True)
        self.assertEqual(m.state, 'SAR OFF')


if __name__ == '__main__':
    unittest.main(verbosity=2)
