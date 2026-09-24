"""Tiny ray-cast renderer of a crop field for the offline perception tests (no Gazebo).

It produces what the robot's RGB-D camera would give: a 320 x 240 BGR image, a depth image
(metres along the optical axis), the intrinsics, and the camera pose - with ground-truth labels.
Plants are ellipsoids with leaf-like colour noise, people are clothed cylinders with a head.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/agri_ugv/scripts'))
from agri_vision_core import Frame, default_base_to_optical, load_config, robot_matrix, rot_rpy  # noqa: E402

W, H = 320, 240
FX = (W / 2) / math.tan(1.5184 / 2)
K = np.array([[FX, 0, (W - 1) / 2], [0, FX, (H - 1) / 2], [0, 0, 1]])
CROP_BGR = (40, 125, 48)
WEED_BGR = (45, 175, 125)
SOIL_BGR = (70, 105, 135)


class Scene:
    def __init__(self, seed=0):
        self.objs = []          # (kind, label, params)
        self.rng = np.random.default_rng(seed)
        self.shadow = None      # (x0, x1) world x band in shadow
        self.light = 1.0

    def plant(self, x, y, h, rx, ry, bgr, label):
        self.objs.append(('ell', label, dict(c=np.array([x, y, h / 2]), r=np.array([rx, ry, h / 2]), bgr=bgr)))

    def crop_row(self, row_y, x0, x1, spacing=0.30, h=0.42, jitter=0.04, dense=False, bgr=CROP_BGR):
        x = x0
        while x <= x1:
            r = 0.19 if dense else 0.13
            self.plant(x, row_y + self.rng.uniform(-jitter, jitter), h * self.rng.uniform(0.85, 1.1),
                       r, r * 1.1, bgr, 'CROP')
            x += spacing

    def weed(self, x, y, size=0.12, h=0.12, bgr=WEED_BGR):
        self.plant(x, y, h, size / 2, size / 2, bgr, 'WEED')

    def human(self, x, y, h=1.72):
        self.objs.append(('cyl', 'HUMAN', dict(c=np.array([x, y]), r=0.21, z0=0.0, z1=h - 0.24, bgr=(125, 85, 60))))
        self.objs.append(('ell', 'HUMAN', dict(c=np.array([x, y, h - 0.12]), r=np.array([0.11, 0.11, 0.12]), bgr=(95, 140, 200))))

    def render(self, robot_x, robot_y, yaw):
        c = load_config()
        T = robot_matrix(robot_x, robot_y, 0.0, rot_rpy(0, 0, yaw)) @ default_base_to_optical(c['camera'])
        vv, uu = np.mgrid[0:H, 0:W]
        dc = np.stack(((uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones((H, W))), -1)
        dw = dc @ T[:3, :3].T
        o = T[:3, 3]
        tbest = np.full((H, W), np.inf)
        img = np.zeros((H, W, 3), np.float32)
        label = np.full((H, W), '', object)
        # ground z = 0
        with np.errstate(divide='ignore', invalid='ignore'):
            tg = np.where(dw[..., 2] < -1e-6, -o[2] / dw[..., 2], np.inf)
        hit = tg < tbest
        tbest[hit] = tg[hit]
        img[hit] = SOIL_BGR
        label[hit] = 'SOIL'
        for kind, lab, p in self.objs:
            if kind == 'ell':
                oc = (o - p['c']) / p['r']
                dd = dw / p['r']
                a = (dd * dd).sum(-1)
                b = 2 * (dd * oc).sum(-1)
                cc = (oc * oc).sum() - 1
                disc = b * b - 4 * a * cc
                with np.errstate(invalid='ignore'):
                    t = np.where(disc > 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
            else:
                ox, oy = o[0] - p['c'][0], o[1] - p['c'][1]
                a = dw[..., 0] ** 2 + dw[..., 1] ** 2
                b = 2 * (dw[..., 0] * ox + dw[..., 1] * oy)
                cc = ox * ox + oy * oy - p['r'] ** 2
                disc = b * b - 4 * a * cc
                with np.errstate(invalid='ignore', divide='ignore'):
                    t = np.where(disc > 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
                z = o[2] + t * dw[..., 2]
                t = np.where((z >= p['z0']) & (z <= p['z1']), t, np.inf)
            t = np.where(t > 0, t, np.inf)
            hit = t < tbest
            tbest[hit] = t[hit]
            img[hit] = p['bgr']
            label[hit] = lab
        P = o + dw * tbest[..., None]
        # leaf / soil texture, lighting and shadows
        noise = self.rng.normal(0, 1, (H, W, 1)).astype(np.float32)
        img = img * (1 + 0.12 * noise)
        shade = np.full((H, W), self.light, np.float32)
        if self.shadow is not None:
            inside = (P[..., 0] > self.shadow[0]) & (P[..., 0] < self.shadow[1])
            shade[inside] *= 0.42
        img = np.clip(img * shade[..., None], 0, 255).astype(np.uint8)
        depth = np.where(np.isfinite(tbest) & (tbest < 15), tbest, np.inf).astype(np.float32)
        img[~np.isfinite(tbest)] = (200, 170, 140)          # sky
        return Frame(img, depth, K, T, (robot_x, robot_y, yaw), 0.0, robot_z=0.0), label, P
