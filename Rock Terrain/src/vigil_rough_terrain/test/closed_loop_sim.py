#!/usr/bin/python3
"""Offline closed-loop test of the vision + navigation cores (no ROS, no Gazebo).

Renders synthetic depth images by ray-casting a height field built from the same
scenario geometry as the Gazebo test worlds, feeds them through TerrainMapper and
NavigatorCore, integrates the commanded motion and sets pitch/roll/z from the
six ground-bearing wheel contact points. It checks each scenario's expected
outcome, that the robot body never ends up over a drop, that the footprint keeps MIN_CLEARANCE
from the true drop edges, and that unreachable goals end in a terminal safe stop (zero velocity).

Drive model (default): like scripts/drive.py in Gazebo, the wheels do not follow a command
instantly. A command takes effect one control tick (0.1 s) later and the actual speed then
slews towards it at drive_linear_accel (m/s^2) / drive_angular_accel (rad/s^2), 0.01 per
20 ms tick in drive.py = 0.5. So the robot COASTS after a stop command (0.36 m from 0.6 m/s),
which the Gazebo regression showed matters near cliff edges. --ideal-drive restores the old
idealised model (the robot moves exactly as commanded, stops instantly) for comparison only.

    python3 test/closed_loop_sim.py                 # all scenarios
    python3 test/closed_loop_sim.py cliff_front     # one scenario
    python3 test/closed_loop_sim.py --terrain       # the real rock-terrain mesh
    python3 test/closed_loop_sim.py --png out_dir   # save final map images
"""
import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG / 'scripts'))
from terrain_core import Params, TerrainMapper, CameraModel, Pose, rpy_to_matrix, CLASS_NAMES, CLIFF  # noqa: E402
from planner_core import NavigatorCore, path_length  # noqa: E402
from scenarios import SCENARIOS, CLIFF_H  # noqa: E402

# Minimum distance (m) between the robot footprint rectangle and the TRUE drop edge of a cliff
# scenario. Stricter than body_over_drop; the original planner reached 0.10 m offline.
MIN_CLEARANCE = 0.10


def drop_clearance(sc, tp, res=0.05):
    """Distance from the footprint rectangle (tp.robot_length x tp.robot_width, centred at
    footprint_center_x) to the nearest edge of the plateau top, from the exact scenario geometry.
    0 = part of the footprint is over the drop."""
    from scipy.ndimage import distance_transform_edt
    x0, y0 = -6.0, -8.0
    nx, ny = int(24 / res), int(16 / res)
    X, Y = np.meshgrid(x0 + (np.arange(nx) + .5) * res, y0 + (np.arange(ny) + .5) * res, indexing='ij')
    top = np.zeros_like(X, bool)
    for o in sc['objects']:
        if o.get('type') != 'box' or abs(o.get('pitch', 0)) > 1e-6:
            continue
        (cx, cy, cz), (sx, sy, sz) = o['pos'], o['size']
        if cz + sz / 2 >= CLIFF_H - 0.1:
            top |= (abs(X - cx) <= sx / 2) & (abs(Y - cy) <= sy / 2)
    d = distance_transform_edt(top) * res
    L, W, cxo, n = tp.robot_length, tp.robot_width, tp.footprint_center_x, 12
    u = np.linspace(-1, 1, n)
    loc = np.concatenate([np.stack([cxo + L / 2 * u, np.full(n, W / 2)], 1),
                          np.stack([cxo + L / 2 * u, np.full(n, -W / 2)], 1),
                          np.stack([np.full(n, cxo + L / 2), W / 2 * u], 1),
                          np.stack([np.full(n, cxo - L / 2), W / 2 * u], 1)])

    def clearance(x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = x + c * loc[:, 0] - s * loc[:, 1], y + s * loc[:, 0] + c * loc[:, 1]
        i = np.floor((px - x0) / res).astype(int)
        j = np.floor((py - y0) / res).astype(int)
        ok = (i >= 0) & (i < d.shape[0]) & (j >= 0) & (j < d.shape[1])
        v = np.zeros(len(px))
        v[ok] = d[i[ok], j[ok]]
        return float(v.min())
    return clearance

HRES = 0.02
# Ground-bearing wheel contact points in base_footprint (wheels 1-3 per side).
WHEELS = [(0.5505, 0.466), (0.0941, 0.436), (-0.2729, 0.436),
          (0.5505, -0.466), (0.0941, -0.436), (-0.2729, -0.436)]


def load_cfg():
    raw = yaml.safe_load((PKG / 'config/vision_nav.yaml').read_text())
    return raw['/**']['ros__parameters']


class HeightField:
    def __init__(self, size=32.0, origin=-16.0, res=HRES):
        self.res, self.origin = res, origin
        self.n = int(size / res)
        self.h = np.zeros((self.n, self.n), np.float32)

    def xy(self):
        c = self.origin + (np.arange(self.n) + 0.5) * self.res
        return np.meshgrid(c, c)  # X[i,j]=x_j, Y[i,j]=y_i

    def add(self, obj):
        X, Y = self.xy()
        if obj['type'] == 'sphere':
            cx, cy, cz = obj['pos']
            r = obj['radius']
            d2 = (X - cx) ** 2 + (Y - cy) ** 2
            top = np.where(d2 < r * r, cz + np.sqrt(np.maximum(r * r - d2, 0)), -1e9)
            self.h = np.maximum(self.h, top)
            return
        cx, cy, cz = obj['pos']
        sx, sy, sz = obj['size']
        p = obj.get('pitch', 0.0)
        iny = np.abs(Y - cy) <= sy / 2
        if abs(p) < 1e-9:
            top = np.where(iny & (np.abs(X - cx) <= sx / 2), cz + sz / 2, -1e9)
        else:
            # rotation about y by p: x = cx + u cos p + w sin p; z = cz - u sin p + w cos p
            a, c = sx / 2, sz / 2
            dx = X - cx
            best = np.full(X.shape, -1e9)
            cp, sp = math.cos(p), math.sin(p)
            for w in (-c, c):
                u = (dx - w * sp) / cp
                ok = np.abs(u) <= a
                best = np.where(ok, np.maximum(best, cz - u * sp + w * cp), best)
            for u in (-a, a):
                w = (dx - u * cp) / sp
                ok = np.abs(w) <= c
                best = np.where(ok, np.maximum(best, cz - u * sp + w * cp), best)
            top = np.where(iny, best, -1e9)
        self.h = np.maximum(self.h, top.astype(np.float32))

    @classmethod
    def from_stl(cls, path, scale=(1.8, 1.8, 1.44)):
        hf = cls()
        hf.h[:] = np.nan
        data = Path(path).read_bytes()
        n = struct.unpack('<I', data[80:84])[0]
        tri = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', 'u2')]),
                            count=n, offset=84)['v'].astype(np.float64) * np.array(scale)
        import cv2
        # rasterise triangles (max z) with barycentric sampling
        for t in tri:
            (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = t
            j0 = int((min(x0, x1, x2) - hf.origin) / hf.res); j1 = int((max(x0, x1, x2) - hf.origin) / hf.res) + 1
            i0 = int((min(y0, y1, y2) - hf.origin) / hf.res); i1 = int((max(y0, y1, y2) - hf.origin) / hf.res) + 1
            j0, i0 = max(j0, 0), max(i0, 0)
            j1, i1 = min(j1, hf.n - 1), min(i1, hf.n - 1)
            if j1 < j0 or i1 < i0:
                continue
            xs = hf.origin + (np.arange(j0, j1 + 1) + 0.5) * hf.res
            ys = hf.origin + (np.arange(i0, i1 + 1) + 0.5) * hf.res
            X, Y = np.meshgrid(xs, ys)
            det = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(det) < 1e-12:
                continue
            l0 = ((y1 - y2) * (X - x2) + (x2 - x1) * (Y - y2)) / det
            l1 = ((y2 - y0) * (X - x2) + (x0 - x2) * (Y - y2)) / det
            l2 = 1 - l0 - l1
            inside = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
            Z = l0 * z0 + l1 * z1 + l2 * z2
            sub = hf.h[i0:i1 + 1, j0:j1 + 1]
            hf.h[i0:i1 + 1, j0:j1 + 1] = np.where(inside, np.fmax(sub, Z), sub)
        # outside the 18 x 18 m mesh there is nothing: a sheer drop
        hf.h = np.where(np.isnan(hf.h), -50.0, hf.h).astype(np.float32)
        return hf

    def sample(self, x, y):
        j = np.clip(((np.asarray(x) - self.origin) / self.res).astype(int), 0, self.n - 1)
        i = np.clip(((np.asarray(y) - self.origin) / self.res).astype(int), 0, self.n - 1)
        out = self.h[i, j]
        outside = (np.asarray(x) < self.origin) | (np.asarray(x) > -self.origin) | \
                  (np.asarray(y) < self.origin) | (np.asarray(y) > -self.origin)
        return np.where(outside, -50.0, out)

    def render_depth(self, cam, pose, p):
        R, t = cam.world_transform(pose)
        dirs = (cam.rays.reshape(-1, 3) @ R.T).astype(np.float32)  # planar-depth param
        depth = np.full(dirs.shape[0], np.inf, np.float32)
        active = np.ones(dirs.shape[0], bool)
        tt, prev = p.depth_near, p.depth_near
        while tt < p.depth_far and active.any():
            idx = np.nonzero(active)[0]
            pts = t + dirs[idx] * tt
            hit = pts[:, 2] <= self.sample(pts[:, 0], pts[:, 1])
            if hit.any():
                hi = idx[hit]
                lo_t = np.full(hi.size, prev, np.float32)
                hi_t = np.full(hi.size, tt, np.float32)
                for _ in range(6):
                    mid = (lo_t + hi_t) / 2
                    q = t + dirs[hi] * mid[:, None]
                    below = q[:, 2] <= self.sample(q[:, 0], q[:, 1])
                    hi_t = np.where(below, mid, hi_t)
                    lo_t = np.where(below, lo_t, mid)
                depth[hi] = hi_t
                active[hi] = False
            prev = tt
            tt += max(0.02, 0.012 * tt)
        return depth.reshape(cam.h, cam.w)

    def wheel_pose(self, x, y, yaw, p):
        c, s = math.cos(yaw), math.sin(yaw)
        U = np.array([w[0] for w in WHEELS]); V = np.array([w[1] for w in WHEELS])
        zs = []
        for du in (-0.08, 0.0, 0.08):  # a wheel rides on the highest point under it
            zs.append(self.sample(x + c * (U + du) - s * V, y + s * (U + du) + c * V))
        Z = np.max(np.stack(zs), axis=0)
        A = np.stack([np.ones_like(U), U, V], 1)
        a, b, cc = np.linalg.lstsq(A, Z, rcond=None)[0]
        pitch, roll = -math.atan(b), math.atan(cc)
        pose = Pose(x=x, y=y, z=float(a) - p.ground_offset_z, R=rpy_to_matrix(roll, pitch, yaw))
        resid = float(np.max(np.abs(A @ [a, b, cc] - Z)))
        return pose, resid

    def body_over_drop(self, pose, p, thresh=0.35):
        """True if part of the (un-inflated) footprint is over terrain far below the
        plane through the wheels (i.e. hanging over a drop; a hill tilts the plane)."""
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        xs = np.arange(-p.robot_length / 2, p.robot_length / 2 + 1e-6, 0.05) + p.footprint_center_x
        ys = np.arange(-p.robot_width / 2, p.robot_width / 2 + 1e-6, 0.05)
        X, Y = np.meshgrid(xs, ys)
        z = self.sample(pose.x + c * X - s * Y, pose.y + s * X + c * Y)
        plane = pose.ground_z(p) + pose.R[2, 0] * X + pose.R[2, 1] * Y
        return bool((z < plane - thresh).mean() > 0.05)

def slew(cur, target, step):
    return cur + max(-step, min(step, target - cur))


def run(name, sc, cfg, max_time=None, dt=0.1, cam_period=0.5, speed=None, verbose=True,
        hf=None, start=(0.0, 0.0, 0.0), save_png=None, ideal_drive=False):
    tp = Params.from_nested(cfg)
    max_time = max_time or sc.get('max_time', 160.0)
    nav = dict(cfg['navigation'])
    if speed is not None:
        nav['cruise_speed'] = speed
    nav['start_delay'] = 0.5
    nav['goal_x'], nav['goal_y'] = sc['goal']
    if hf is None:
        hf = HeightField()
        for o in sc['objects']:
            hf.add(o)
    # render at half resolution for speed (intrinsics scale accordingly)
    cam = CameraModel(tp, width=160, height=120)
    mapper = TerrainMapper(tp)
    navc = NavigatorCore(tp, nav)
    x, y, yaw = start
    t, next_cam = 0.0, 0.0
    traj, max_tilt, over_drop, states = [], 0.0, 0, []
    stopped, stop_streak, max_streak, active = 0, 0, 0, 0
    terminal_t, moved_after_terminal, min_clear = None, 0.0, math.inf
    # drive model state: command waiting for the next control tick, actual wheel-level speed
    pending, act_v, act_w = (0.0, 0.0), 0.0, 0.0
    a_lin = nav.get('drive_linear_accel', 0.5) * dt
    a_ang = nav.get('drive_angular_accel', 0.5) * dt
    clear_fn = drop_clearance(sc, tp) if sc.get('spawn_z', 0.0) >= CLIFF_H - 1e-6 else None
    t_wall = time.time()
    while t < max_time:
        pose, resid = hf.wheel_pose(x, y, yaw, tp)
        if t >= next_cam:
            depth = hf.render_depth(cam, pose, tp)
            mapper.process(depth, pose, cam)
            navc.update_map(mapper.classes, mapper.slope_deg, mapper.roughness, t=t)
            next_cam += cam_period
        v, w = navc.tick(t, pose)
        st = navc.status(pose)
        if not states or states[-1] != st['state']:
            states.append(st['state'])
        max_tilt = max(max_tilt, pose.tilt_deg)
        if hf.body_over_drop(pose, tp):
            over_drop += 1
        traj.append((round(t, 2), x, y))
        if clear_fn is not None and navc.state != 'WAITING':
            min_clear = min(min_clear, clear_fn(x, y, yaw))
        if navc.state == 'GOAL_REACHED':
            break
        if navc.state in navc.TERMINAL:          # terminal failure: must hold zero velocity
            if terminal_t is None:
                terminal_t = t
            moved_after_terminal += abs(v) * dt + abs(w) * dt
            if t - terminal_t >= 5.0:
                break
        elif navc.state != 'WAITING':
            active += 1
            # idle = the robot is not moving. With the drive model a zero command is followed by
            # braking (still moving), so this checks the ACTUAL motion; in the ideal-drive mode the
            # two are identical (the original rule).
            mv, mw = (v, w) if ideal_drive else (act_v, act_w)
            if abs(mv) < 1e-6 and abs(mw) < 1e-6:
                stopped += 1
                stop_streak += 1
                max_streak = max(max_streak, stop_streak)
            else:
                stop_streak = 0
        if ideal_drive:
            act_v, act_w = v, w
        else:  # one tick of latency, then acceleration-limited (drive.py)
            act_v, act_w = slew(act_v, pending[0], a_lin), slew(act_w, pending[1], a_ang)
            pending = (v, w)
        x += act_v * math.cos(yaw) * dt
        y += act_v * math.sin(yaw) * dt
        yaw += act_w * dt
        t += dt
    ys = np.array([p[2] for p in traj]); xs = np.array([p[1] for p in traj])
    res = dict(scenario=name, outcome=navc.state, sim_time=round(t, 1), wall=round(time.time() - t_wall, 1),
               hazard_replans=navc.hazard_replans, replans=navc.replans, max_tilt=round(max_tilt, 1),
               body_over_drop_steps=over_drop, max_abs_y=round(float(np.abs(ys).max()), 2),
               min_y=round(float(ys.min()), 2), max_y=round(float(ys.max()), 2),
               final=(round(x, 2), round(y, 2)), states=states,
               cliff_cells=int((mapper.classes == CLIFF).sum()),
               stopped_fraction=round(stopped / max(1, active), 3), longest_stop_s=round(max_streak * dt, 1),
               failure_reason=navc.failure_reason, terminal_t=terminal_t,
               moved_after_terminal=round(moved_after_terminal, 4),
               min_clearance=None if clear_fn is None else round(min_clear, 3))
    exp = sc.get('expect', {})
    if exp.get('outcome') == 'SAFE_STOP':  # B unreachable: a terminal safe stop, never GOAL_REACHED
        ok = (res['outcome'] in navc.FAILURES and over_drop == 0 and res['moved_after_terminal'] == 0.0)
    else:
        ok = res['outcome'] == exp.get('outcome', res['outcome']) and over_drop == 0
    if 'max_hazard_replans' in exp:
        ok &= res['hazard_replans'] <= exp['max_hazard_replans']
    if 'max_abs_y' in exp:
        ok &= res['max_abs_y'] <= exp['max_abs_y']
    if 'min_abs_y' in exp:
        ok &= res['max_abs_y'] >= exp['min_abs_y']
    if 'min_y' in exp:
        ok &= res['max_y'] >= exp['min_y']
    # Before a terminal state the motors must not sit idle (short 0.1 s gaps between manoeuvres are
    # fine). Goal scenarios keep the original strict 1.0 s rule. In the unreachable (SAFE_STOP)
    # scenarios a safety stop may hold for blocked_timeout, and a recovery that finds neither observed
    # ground to reverse onto nor a clear turn adds one more blocked_timeout before escalating; a
    # longer idle streak would mean the navigator froze instead of escalating to a terminal state.
    idle_limit = 1.0
    if exp.get('outcome') == 'SAFE_STOP':
        idle_limit = 2 * nav.get('blocked_timeout', 4.0) + 1.0
    res['idle_limit_s'] = idle_limit
    ok &= res['longest_stop_s'] <= idle_limit
    # footprint clearance to the true drop edges (scenario geometry, cliff scenarios only)
    if res['min_clearance'] is not None:
        ok &= res['min_clearance'] >= MIN_CLEARANCE
    if 'min_tilt' in exp:
        ok &= res['max_tilt'] >= exp['min_tilt']
    if 'max_tilt' in exp:
        ok &= res['max_tilt'] <= exp['max_tilt']
    if 'state' in exp:
        ok &= exp['state'] in states
    if 'avoid_band' in exp:  # (x, y_min, y_max): must NOT cross this band (e.g. the too-steep face)
        bx, y0, y1 = exp['avoid_band']
        at = np.abs(xs - bx) < 0.3
        ok &= not bool(np.any((ys[at] > y0) & (ys[at] < y1)))
    if exp.get('no_obstacle'):  # rocks must be cost (UNEVEN/CLIMBABLE), never OBSTACLE
        from terrain_core import OBSTACLE
        res['obstacle_cells'] = int((mapper.classes == OBSTACLE).sum())
        ok &= res['obstacle_cells'] == 0
    if 'bridge' in exp:
        at = np.abs(xs - 5.25) < 0.3
        ok &= bool(at.any()) and bool(np.all((ys[at] > exp['bridge'][0]) & (ys[at] < exp['bridge'][1])))
    res['pass'] = bool(ok)
    if save_png:
        save_map_png(Path(save_png) / f'{name}.png', mapper, navc, traj, tp)
    if verbose:
        print(json.dumps(res))
    return res


def save_map_png(path, mapper, navc, traj, tp):
    import cv2
    from terrain_core import CLASS_BGR
    lut = np.zeros((256, 3), np.uint8)
    for k, v in CLASS_BGR.items():
        lut[k] = v
    img = lut[mapper.classes][::-1].copy()
    img = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return (int((x - tp.map_origin_x) / tp.resolution * 3), int((tp.map_size - (y - tp.map_origin_y)) / tp.resolution * 3))
    for pth, col in ((navc.previous_path, (160, 160, 160)), (navc.path, (255, 255, 0))):
        if pth is not None:
            cv2.polylines(img, [np.array([px(*q) for q in pth], np.int32)], False, col, 2)
    cv2.polylines(img, [np.array([px(q[1], q[2]) for q in traj], np.int32)], False, (255, 255, 255), 1)
    cv2.circle(img, px(*navc.goal), 6, (255, 0, 255), -1)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='*')
    ap.add_argument('--terrain', action='store_true', help='run on the real rock terrain (A->B of the comparison)')
    ap.add_argument('--png', default=None)
    ap.add_argument('--ideal-drive', action='store_true',
                    help='old idealised drive (instant response); for comparison only, not a pass criterion')
    a = ap.parse_args()
    cfg = load_cfg()
    results = []
    if a.terrain:
        hf = HeightField.from_stl(PKG / 'models/rocky_terrain_n4/meshes/terrain_collision.stl')
        sc = dict(goal=(cfg['navigation']['goal_x'], cfg['navigation']['goal_y']), objects=[],
                  expect=dict(outcome='GOAL_REACHED'))
        results.append(run('rock_terrain', sc, cfg, hf=hf, start=(-5.4, -3.6, 0.0), max_time=200, save_png=a.png,
                           ideal_drive=a.ideal_drive))
    else:
        for name in (a.names or SCENARIOS):
            results.append(run(name, SCENARIOS[name], cfg, save_png=a.png, ideal_drive=a.ideal_drive))
    print('\nSUMMARY')
    for r in results:
        print(f"  {'PASS' if r['pass'] else 'FAIL'}  {r['scenario']:<15} {r['outcome']:<22} t={r['sim_time']:<6} "
              f"replans={r['replans']} hazard_replans={r['hazard_replans']} y=[{r['min_y']},{r['max_y']}] "
              f"tilt={r['max_tilt']} over_drop={r['body_over_drop_steps']} clearance={r['min_clearance']}")
    sys.exit(0 if all(r['pass'] for r in results) else 1)


if __name__ == '__main__':
    main()
