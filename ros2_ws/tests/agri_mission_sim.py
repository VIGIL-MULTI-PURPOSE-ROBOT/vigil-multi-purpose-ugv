"""Kinematic simulation of the agriculture rover for offline mission tests.

The real crop_row_driver.py node runs unmodified on ROS stand-ins (ros_stubs.py). Its /cmd_vel_nav
goes through a copy of drive.py's limits (2 m minimum radius on rows, 0.85 m commanded in the
headland, 2 m/s^2 ramp, 0.6 s watchdog) and the skid-steer's 1.395x tight-turn curvature on soil.
Obstacles are reported to the driver as a perfect tracker would (position, velocity, dynamic flag).
"""
import importlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent / 'src/agri_ugv'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PKG / 'scripts'))
import ros_stubs  # noqa: E402

CONFIG = str(PKG / 'config/agriculture.yaml')
HALF_L, HALF_W = 0.85, 0.55


class Obstacle:
    def __init__(self, oid, path, radius=0.3, height=1.7, cls='obstacle'):
        self.id, self.path, self.r, self.h, self.cls = oid, path, radius, height, cls

    def at(self, t):
        return np.asarray(self.path(t), float)


def static(x, y):
    return lambda t: (x, y)


def linear(x0, y0, vx, vy, t0=0.0, t1=1e9):
    return lambda t: (x0 + vx * (min(max(t, t0), t1) - t0), y0 + vy * (min(max(t, t0), t1) - t0))


class Sim:
    def __init__(self, rows=None, obstacles=(), start=None, params=None, tracker=True, sensor_range=12.0):
        self.reg = ros_stubs.install()
        self.reg.params.update({'config': CONFIG})
        if rows:
            self.reg.params['row_count'] = rows
        self.reg.params.update(params or {})
        for m in ('crop_row_driver', 'agri_mission_core'):
            sys.modules.pop(m, None)
        self.mod = importlib.import_module('crop_row_driver')
        self.t = 0.0
        self.mod.time = SimpleNamespace(monotonic=lambda: self.t)
        self.node = self.mod.CropRowDriver()
        g = self.node.geo
        self.x, self.y, self.yaw = start if start else (g.turn_west, float(g.row_y(0)), 0.0)
        self.v = self.w = 0.0
        self.tv = self.tw = 0.0
        self.last_cmd = -1.0
        self.mode = 'row'
        self.obstacles = list(obstacles)
        self.tracker = tracker
        self.range = sensor_range
        self.min_clear = 1e9
        self.trace = []
        self.cmds = self.reg.pubs['cmd_vel_nav']
        self.reg.subs.setdefault('crop_row/mode', [])
        self.reg.pubs['crop_row/mode'].publish = self._mode

    def _mode(self, m):
        self.mode = m.data

    def odom(self):
        q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(self.yaw / 2), w=math.cos(self.yaw / 2))
        self.reg.deliver('sim/ground_truth', SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=self.x, y=self.y, z=0.0), orientation=q)),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(
                x=self.v * math.cos(self.yaw), y=self.v * math.sin(self.yaw), z=0.0)))))

    def obstacle_msg(self):
        obs = []
        for o in self.obstacles:
            p = o.at(self.t)
            if math.hypot(p[0] - self.x, p[1] - self.y) > self.range:
                continue
            v = (o.at(self.t + 0.1) - o.at(self.t - 0.1)) / 0.2
            ang = np.linspace(0, 2 * math.pi, 9)[:-1]
            pts = np.c_[p[0] + o.r * np.cos(ang), p[1] + o.r * np.sin(ang)]
            sp = float(np.hypot(*v))
            obs.append(dict(id=o.id, x=float(p[0]), y=float(p[1]), vx=float(v[0]), vy=float(v[1]), speed=sp,
                            dynamic=sp > 0.2, radius=0.0, points=pts.round(3).tolist(), height=o.h, cls=o.cls,
                            confirmed=True, distance=round(math.hypot(p[0] - self.x, p[1] - self.y), 2),
                            bearing=round(math.degrees(math.atan2(p[1] - self.y, p[0] - self.x) - self.yaw), 1)))
        self.reg.deliver('agriculture/obstacles', SimpleNamespace(data=json.dumps(dict(obstacles=obs))))

    def clearance(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        best = 1e9
        for o in self.obstacles:
            p = o.at(self.t)
            dx, dy = p[0] - self.x, p[1] - self.y
            u, w = c * dx + s * dy, -s * dx + c * dy
            ex, ey = max(abs(u) - HALF_L, 0), max(abs(w) - HALF_W, 0)
            best = min(best, math.hypot(ex, ey) - o.r)
        return best

    def step(self, dt=0.05):
        self.reg.t = self.t
        self.odom()
        if self.tracker and int(round(self.t / dt)) % 4 == 0:
            self.obstacle_msg()
        n0 = len(self.cmds.msgs)
        self.node.tick()
        if len(self.cmds.msgs) > n0:
            m = self.cmds.msgs[-1]
            self.tv, self.tw, self.last_cmd = m.linear.x, m.angular.z, self.t
        # drive.py: watchdog, ramp, radius clamp
        tv, tw = (self.tv, self.tw) if self.t - self.last_cmd < .6 else (0.0, 0.0)
        R = .85 if self.mode == 'headland' else 2.0
        tw = float(np.clip(tw, -abs(tv) / R, abs(tv) / R))
        self.v += float(np.clip(tv - self.v, -.1, .1))
        self.w += float(np.clip(tw - self.w, -.1, .1))
        w = float(np.clip(self.w, -abs(self.v) / R, abs(self.v) / R))
        if self.mode == 'headland':
            w *= 1.395
        self.yaw += w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.t += dt
        self.min_clear = min(self.min_clear, self.clearance())
        if int(round(self.t / dt)) % 10 == 0:
            self.trace.append((round(self.t, 1), round(self.x, 2), round(self.y, 2), self.node.phase, self.node.row))

    def run(self, t_max=3000.0):
        while self.t < t_max and not self.node.done:
            self.step()
        return self

    def messages(self):
        return [json.loads(m.data)['message'] for m in self.reg.pubs['exploration/status'].msgs]
