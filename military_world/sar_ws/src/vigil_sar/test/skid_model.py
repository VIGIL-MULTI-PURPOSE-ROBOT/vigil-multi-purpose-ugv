"""Planar rigid-body model of the 8-wheel skid-steer rover, for testing the REAL drive.py control
law without Gazebo.

It is a MODEL, not the simulator: rigid chassis, equal static wheel loads, Coulomb tyre friction
with separate longitudinal (mu) and lateral (mu_lat) limits combined in a friction ellipse, wheel
spin inertia, and joint (rolling-resistance) friction - the same parameters as physics.yaml and the
CAD geometry. What it can answer: does a given motor controller produce enough torque to make
this rover yaw, drive and stop the way the navigator expects? It cannot stand in for Gazebo's
contact solver, terrain or suspension; the Gazebo test (test_motion.sh) does that.
"""
import math

# CAD wheel centres (base_link): rocker origin x -0.5697, y +-0.30; wheel offsets from cad_geometry
_WX = [1.135327881, 0.681537231, 0.313886093, 0.0]
_WY = [0.166163162, 0.135723442, 0.135723442, 0.135723442]
_R = [0.23463, 0.17655, 0.17655, 0.17655]


class SkidRover:
    """The vigil_rough_terrain rover as a planar rigid body: CAD wheel positions, mu equal both ways,
    the rear pair (wheel 4) lifted so six wheels carry the load, as in that model."""

    def __init__(self, phys, yaw=0.0):
        rv = phys.get('rover', {})
        self.m = float(rv.get('mass', 291.2))
        self.g = float(phys['engine']['gravity'])
        self.mu = self.mu_lat = float(rv.get('wheel_friction', 0.85))
        lifted = bool(rv.get('rear_pair_lifted', True))
        n_ground = 6 if lifted else 8
        xs = [-0.5697 + x for x in _WX]
        ground = [k for k in range(4) if not (lifted and k == 3)]
        xc = sum(xs[k] for k in ground) / len(ground)
        self.wheels = []
        for side, sy in (('L', 1.0), ('R', -1.0)):
            for k in range(4):
                r = _R[k]
                self.wheels.append(dict(name=f'{side}{k + 1}', x=xs[k] - xc, y=sy * (0.30 + _WY[k]),
                                        r=r, J=0.5 * (35.2 if k == 0 else 15.3) * r * r, w=0.0, tf=0.2,
                                        N=0.0 if (lifted and k == 3) else self.m * self.g / n_ground))
        self.Izz = self.m * (1.53 ** 2 + 1.12 ** 2) / 12.0
        self.x = self.y = 0.0
        self.yaw = yaw
        self.vx = self.vy = self.r = 0.0                # body-frame velocities

    def omega(self):
        return {w['name']: w['w'] for w in self.wheels}

    def speed(self):
        return math.hypot(self.vx, self.vy)

    def step(self, torques, dt, eps=0.02, wheel_speeds=None):
        """torques: motor torque per wheel; or wheel_speeds: velocity-controlled joints (as with
        velocity_controllers in Gazebo) - the wheel turns at exactly that speed and the chassis
        still only moves through the tyre friction forces."""
        if wheel_speeds is not None:
            for wh, ws in zip(self.wheels, wheel_speeds):
                wh['w'] = ws
            torques = [None] * 8
        Fx_sum = Fy_sum = Mz = 0.0
        for wh, tau in zip(self.wheels, torques):
            vxi = self.vx - self.r * wh['y']
            vyi = self.vy + self.r * wh['x']
            s = vxi - wh['w'] * wh['r']                 # contact-point slip, rolling direction
            N = wh['N']
            if N <= 0.0:
                continue
            fx = -self.mu * N * max(-1.0, min(1.0, s / eps))
            fy = -self.mu_lat * N * max(-1.0, min(1.0, vyi / eps))
            # friction ellipse: the tyre cannot give full grip both ways at once
            k = math.hypot(fx / (self.mu * N), fy / (self.mu_lat * N))
            if k > 1.0:
                fx, fy = fx / k, fy / k
            # wheel spin: motor torque, reaction of the ground force, joint (rolling) friction
            if tau is not None:
                tf = wh['tf'] * max(-1.0, min(1.0, wh['w'] / 0.05))
                wh['w'] += (tau - wh['r'] * fx - tf) / wh['J'] * dt
            Fx_sum += fx
            Fy_sum += fy
            Mz += wh['x'] * fy - wh['y'] * fx
        self.vx += (Fx_sum / self.m + self.r * self.vy) * dt
        self.vy += (Fy_sum / self.m - self.r * self.vx) * dt
        self.r += Mz / self.Izz * dt
        c, s_ = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * self.vx - s_ * self.vy) * dt
        self.y += (s_ * self.vx + c * self.vy) * dt
        self.yaw += self.r * dt


class DriveLaw:
    """drive.py's own DriveLaw (jerk-limited speed, yaw ramp, wheel-acceleration cap) at 100 Hz."""

    def __init__(self, phys, *_ignored, **_kw):
        import drive                                     # the real module
        self.law = drive.DriveLaw(phys.get('drive', {}), phys.get('rover', {}))
        self.dt = drive.TICK
        self.speeds = [0.0] * 8

    @property
    def current(self):
        return [self.law.v, self.law.w]

    def tick(self, v, w, omega=None, dt=None, fresh=True):
        self.speeds = self.law.tick(v if fresh else 0.0, w if fresh else 0.0, dt or self.dt)
        return self.speeds


def run(phys, command, seconds, mode='pi', override=None, rover=None, dt=0.0005):
    """Hold command(t)->(v, w) for `seconds`; returns the rover."""
    rv = rover or SkidRover(phys)
    law = DriveLaw(phys, mode, override)
    steps_per_tick = int(round(law.dt / dt))
    t, n = 0.0, 0
    while t < seconds:
        if n % steps_per_tick == 0:
            v, w = command(t)
            law.tick(v, w)
        rv.step(None, dt, wheel_speeds=law.speeds)
        t += dt
        n += 1
    return rv


def closed_loop_a_to_b(cfg, phys, start, yaw, goal, seconds=60.0, classes=None, mode='pi', override=None,
                       reveal=None):
    """The real NavigatorCore (A* + follower + goal logic), as cliff_navigator runs it, driving the real
    drive.py control law, closed around the rover model. The navigator only ever sees the model's
    pose - as it only ever sees Gazebo's ground truth in the simulator."""
    import numpy as np
    from planner_core import NavigatorCore
    from terrain_core import Params, Pose, rpy_to_matrix, SAFE
    tp = Params.from_nested(cfg)
    nav_cfg = cfg['navigation']
    nav = NavigatorCore(tp, nav_cfg)
    n = int(round(tp.map_size / tp.resolution))
    truth = classes if classes is not None else np.full((n, n), SAFE, np.uint8)
    # reveal=(range_m, fov_rad): the map fills in only where the depth camera would see, the way
    # terrain_mapper builds it in Gazebo - obstacles appear when the rover gets near them
    from terrain_core import UNKNOWN
    known = np.full((n, n), UNKNOWN, np.uint8) if reveal else truth
    if reveal:
        ii, jj = np.mgrid[0:n, 0:n]
        cx = tp.map_origin_x + (jj + 0.5) * tp.resolution
        cy = tp.map_origin_y + (ii + 0.5) * tp.resolution
    nav.update_map(known, None, None)
    nav.set_goal(goal[0], goal[1], 0.0)
    nav.t0 = -100.0
    shaper = None
    rv = SkidRover(phys, yaw)
    # the model's origin is the wheel-set centre; place it at A
    rv.x, rv.y = start
    law = DriveLaw(phys, mode, override)
    dt, t, k = 0.0005, 0.0, 0
    v = w = 0.0
    log = []
    reached_t = None
    while t < seconds:
        if reveal and k % int(round(reveal[2] / dt if len(reveal) > 2 else 1000)) == 0:   # map update (default 2 Hz)
            dx, dy = cx - rv.x, cy - rv.y
            rng = np.hypot(dx, dy)
            ang = np.abs(np.arctan2(np.sin(np.arctan2(dy, dx) - rv.yaw), np.cos(np.arctan2(dy, dx) - rv.yaw)))
            seen = (rng < reveal[0]) & ((ang < reveal[1] / 2) | (rng < 1.5))
            known[seen] = truth[seen]
            nav.update_map(known, None, None)
        if k % 100 == 0:                                  # navigator: 20 Hz
            pose = Pose(x=rv.x, y=rv.y, z=0.05, R=rpy_to_matrix(0, 0, rv.yaw))
            v, w = nav.tick(t, pose)
            if nav.state in nav.TERMINAL:
                v, w = 0.0, 0.0
                if reached_t is None:
                    reached_t = t
            log.append((t, rv.x, rv.y, rv.speed(), nav.state, v, w, nav.clearance, rv.yaw))
        if k % 20 == 0:                                   # drive.py: 100 Hz
            law.tick(v, w)
        rv.step(None, dt, wheel_speeds=law.speeds)
        t += dt
        k += 1
        if reached_t is not None and t - reached_t > 3.0:
            break
    return rv, nav, log, reached_t
