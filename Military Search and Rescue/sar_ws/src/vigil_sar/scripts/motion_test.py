#!/usr/bin/python3
"""Gazebo acceptance tests for the SAR UGV - judged ONLY on physical evidence.

Every verdict comes from what the physics engine reports, never from the dashboard or the plan:
  /sim/ground_truth   gz OdometryPublisher: the model's true world pose and velocity
  /joint_states       the wheel joints' measured rotation (joint_state_broadcaster)
  /drive/status       what drive.py commanded (per-wheel target speed and torque)
  /navigation/status  what the navigator believes (state, goal) - checked AGAINST ground truth
  /sar/state          whether the SAR manager accepted a switch command

  --phase motion   (flat world, sim.launch.py only, NO navigator: this node owns /cmd_vel)
      TEST 1 forward   TEST 2 turn left   TEST 3 turn right   TEST 4 stop
      + reverse, arc left, arc right, and wheel rotation directions for each
      + GRAVITY / rest checks (IMU 9.81, level, six loaded wheels touching), and the WHEELIE tests:
      full-speed step from rest (smooth to exactly 3.0 m/s, front wheels stay down), speed cap,
      braking from 3 m/s (realistic deceleration, no nose-over)
  --phase scenario (scenario_worlds.py world + sim.launch.py + navigation.launch.py)
      flat_road / moderate_slope / steep_hill / obstacle / cliff_front: reach B, top speed, acceleration,
      front-wheel contact, tilt, no standing stops, clearance speed profile, never in the pit / the wall
      human_block: the test drives INTO a standing person - the contact must stop the rover (no pass-through)
      human_standing / human_crossing: the navigator must keep its distance from a standing / walking
      person (their true pose from a gz OdometryPublisher on the person) and reach B
  --phase mission  (military_world, full sar_mission.launch.py)
      TEST 7 SAR at A   TEST 5 reach B   TEST 8 SAR while driving   TEST 9 SAR near B
      TEST 6 no overshoot   TEST 10 SAR after B   TEST 11 SAR again after STOP
      --humans MIN  also runs SAR for up to MIN simulated minutes and waits for H1

Run through test_motion.sh, which starts and stops the simulation around each phase.
"""
import argparse
import json
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
try:
    from sensor_msgs.msg import Imu
except ImportError:                                     # pragma: no cover
    Imu = None
try:
    from ros_gz_interfaces.msg import Contacts          # /suspension/contacts/<wheel> (bridge.yaml)
except ImportError:                                     # pragma: no cover
    Contacts = None

WHEELS = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def roll_pitch(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class Monitor:
    """Samples the physical state on every spin: speed, acceleration (from ground-truth speed over
    0.2 s), nose-up pitch (+ = front up), tilt, height, and how long the front pair (L1 + R1) /
    rear loaded pair (L3 + R3) have BOTH been off the ground (contact sensors)."""

    def __init__(self, p):
        self.p = p
        self.hist = []                     # (t, speed)
        self.max_speed = 0.0
        self.max_acc = self.max_dec = 0.0
        self.max_nose_up = self.max_nose_down = 0.0
        self.max_tilt = 0.0
        self.min_z = math.inf
        self.front_off = self.rear_off = 0.0
        self.max_front_off = self.max_rear_off = 0.0
        self.t_prev = None
        self.pitch0 = p.rp[1]

    def sample(self):
        p = self.p
        if p.gt is None:
            return
        t, sp = p.gt[0], p.gt[4]
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        self.hist.append((t, sp))
        while self.hist and t - self.hist[0][0] > 0.2:
            t0, s0 = self.hist.pop(0)
            if t - t0 > 0.15:
                a = (sp - s0) / (t - t0)
                self.max_acc = max(self.max_acc, a)
                self.max_dec = max(self.max_dec, -a)
        self.max_speed = max(self.max_speed, sp)
        nose_up = -math.degrees(p.rp[1] - self.pitch0)
        self.max_nose_up = max(self.max_nose_up, nose_up)
        self.max_nose_down = max(self.max_nose_down, -nose_up)
        self.max_tilt = max(self.max_tilt, math.degrees(max(abs(p.rp[0]), abs(p.rp[1]))))
        self.min_z = min(self.min_z, p.z)
        if p.contact_msgs:
            self.front_off = self.front_off + dt if not (p.touching('L1') or p.touching('R1')) else 0.0
            self.rear_off = self.rear_off + dt if not (p.touching('L3') or p.touching('R3')) else 0.0
            self.max_front_off = max(self.max_front_off, self.front_off)
            self.max_rear_off = max(self.max_rear_off, self.rear_off)


class Probe(Node):
    def __init__(self, own_cmd_vel):
        super().__init__('vigil_motion_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt = None                 # (t, x, y, yaw, speed)
        self.wheel = {}
        self.js_count = 0
        self.nav = {}
        self.sar = {}
        self.humans = []
        self.drive = {}
        self.rp = (0.0, 0.0)           # roll, pitch (rad) from ground truth
        self.z = 0.0
        self.imu_z = None              # IMU specific force, z (m/s2): +9.81 at rest under Earth gravity
        self.contact_t = {w: -1e9 for w in WHEELS}
        self.contact_msgs = 0
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_gt, qos_profile_sensor_data)
        if Imu is not None:
            self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        if Contacts is not None:
            for w in WHEELS:
                self.create_subscription(Contacts, f'/suspension/contacts/{w}',
                                         lambda m, w=w: self.on_contact(w, m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.create_subscription(String, '/navigation/status', lambda m: self._json(m, 'nav'), 10)
        self.create_subscription(String, '/sar/state', lambda m: self._json(m, 'sar'), 10)
        self.create_subscription(String, '/drive/status', lambda m: self._json(m, 'drive'), 10)
        self.create_subscription(String, '/sar/humans', self.on_humans, 10)
        self.people = {}               # name -> (x, y) true pose (scenario people)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10) if own_cmd_vel else None
        self.goal = self.create_publisher(PoseStamped, '/navigation/goal', 10)
        self.sar_cmd = self.create_publisher(String, '/sar/command', 10)

    # ------------------------------------------------------------------ inputs
    def on_gt(self, m):
        p, v = m.pose.pose, m.twist.twist.linear
        self.gt = (self.now(), p.position.x, p.position.y, yaw_of(p.orientation), math.hypot(v.x, v.y))
        self.rp = roll_pitch(p.orientation)
        self.z = p.position.z

    def on_imu(self, m):
        self.imu_z = float(m.linear_acceleration.z)

    def watch_person(self, name, topic):
        self.create_subscription(Odometry, topic, lambda m, n=name: self.people.__setitem__(
            n, (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), qos_profile_sensor_data)

    def on_contact(self, wheel, m):
        self.contact_msgs += 1
        if len(m.contacts) > 0:
            self.contact_t[wheel] = self.now()

    def touching(self, wheel, age=0.15):
        return self.now() - self.contact_t[wheel] < age

    def on_js(self, m):
        self.js_count += 1
        for n, vel in zip(m.name, m.velocity):
            k = n.replace('_joint', '')
            if k in WHEELS:
                self.wheel[k] = float(vel)

    def _json(self, m, attr):
        try:
            setattr(self, attr, json.loads(m.data))
        except ValueError:
            pass

    def on_humans(self, m):
        try:
            self.humans = json.loads(m.data).get('humans', [])
        except ValueError:
            pass

    # ------------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, sim_seconds, cmd=None, until=None, wall_limit=None):
        """Spin until `sim_seconds` of SIMULATION time pass (or `until()` is true), publishing
        `cmd` at 20 Hz of sim time. The world may run far below real time, so the wall-clock
        guard is generous; hitting it is reported, not hidden."""
        t0, wall0, last_pub = self.now(), time.time(), -1e9
        wall_limit = wall_limit or (sim_seconds * 60.0 + 60.0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            t = self.now()
            if cmd is not None and self.cmd is not None and t - last_pub >= 0.05:
                tw = Twist()
                tw.linear.x, tw.angular.z = float(cmd[0]), float(cmd[1])
                self.cmd.publish(tw)
                last_pub = t
            if until is not None and until():
                return True
            if t - t0 >= sim_seconds:
                return until is None
            if time.time() - wall0 > wall_limit:
                return False

    def wait_for(self, what, pred, wall=180.0):
        wall0 = time.time()
        while rclpy.ok() and time.time() - wall0 < wall:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        print(f'  ... gave up waiting for {what} after {wall:.0f} s', flush=True)
        return False

    def send_goal(self, x, y):
        g = PoseStamped()
        g.header.frame_id = 'world'
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        g.pose.orientation.w = 1.0
        self.goal.publish(g)

    def press_sar(self, cmd):
        self.sar_cmd.publish(String(data=cmd))

    def wheel_signs(self):
        """(mean left wheel speed, mean right wheel speed) in rad/s, from the joints themselves."""
        L = [self.wheel.get(w, 0.0) for w in WHEELS[:4]]
        R = [self.wheel.get(w, 0.0) for w in WHEELS[4:]]
        return sum(L) / 4.0, sum(R) / 4.0


class Report:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail):
        self.rows.append((name, ok, detail))
        print(f"{'PASS' if ok else 'FAIL'}  {name:34s} {detail}", flush=True)

    def summary(self):
        n = sum(1 for _, ok, _ in self.rows if ok)
        print(f'\n{n}/{len(self.rows)} checks passed', flush=True)
        return n == len(self.rows)


# ====================================================================== phase: motion
def phase_motion(p, rep):
    if not p.wait_for('/sim/ground_truth', lambda: p.gt is not None):
        rep.add('ground truth available', False, 'no /sim/ground_truth - the bridge or the model is missing')
        return
    if not p.wait_for('/joint_states', lambda: p.js_count > 5):
        rep.add('wheel joint states', False, 'no /joint_states - joint_state_broadcaster is not running')
        return
    p.spin_sim(2.0, cmd=(0.0, 0.0))                      # settle on the springs
    d = p.drive
    if d:
        ok = d.get('mode') == 'velocity'
        rep.add('drive.py = rough-terrain drive', ok,
                f"mode {d.get('mode')}, max {d.get('max_linear')} m/s / {d.get('max_angular')} rad/s")
    else:
        rep.add('drive.py status', False, 'no /drive/status - is the new drive.py running? (rebuild)')

    # ---- gravity and the rover at rest (Earth gravity acting on the whole model, wheels loaded)
    z0 = p.z
    p.spin_sim(2.0, cmd=(0.0, 0.0))
    if p.imu_z is not None:
        rep.add('GRAVITY  IMU at rest', abs(p.imu_z - 9.81) < 0.4,
                f'specific force z {p.imu_z:.2f} m/s2 (Earth gravity 9.81 on the resting rover)')
    else:
        rep.add('GRAVITY  IMU at rest', False, 'no /imu/data')
    roll, pitch = (math.degrees(a) for a in p.rp)
    rep.add('        rests level and settled', abs(roll) < 3.0 and abs(pitch) < 3.0 and abs(p.z - z0) < 0.01,
            f'roll {roll:+.1f} deg, pitch {pitch:+.1f} deg, height change over 2 s {1000 * (p.z - z0):+.1f} mm')
    if p.contact_msgs:
        down = [w for w in WHEELS if p.touching(w)]
        want = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
        rep.add('        loaded wheels on the ground', all(w in down for w in want),
                f'touching: {" ".join(down) or "none"} (want {" ".join(want)}; L4/R4 are lifted in the CAD)')
    else:
        rep.add('        loaded wheels on the ground', False, 'no /suspension/contacts/* messages (bridge?)')

    def run(name, cmd, secs):
        _, x0, y0, yaw0, _ = p.gt
        signs = []
        turned = [0.0, yaw0]                       # yaw accumulated sample by sample: a turn of
                                                   # more than 180 deg must not wrap to negative

        def sample():
            turned[0] += wrap(p.gt[3] - turned[1])
            turned[1] = p.gt[3]
            if p.gt[0] - t_start > secs * 0.4:
                signs.append(p.wheel_signs())
            return False
        t_start = p.now()
        ok = p.spin_sim(secs, cmd=cmd, until=sample)
        sample()
        _, x1, y1, yaw1, sp = p.gt
        fwd = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        lat = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
        dyaw = math.degrees(turned[0])
        wl = sum(s[0] for s in signs) / max(1, len(signs))
        wr = sum(s[1] for s in signs) / max(1, len(signs))
        return dict(fwd=fwd, lat=lat, dyaw=dyaw, speed=sp, wl=wl, wr=wr, completed=ok)

    r = run('forward', (0.6, 0.0), 8.0)
    rep.add('TEST 1  forward 0.6 m/s x 8 s', r['fwd'] > 3.0 and abs(r['lat']) < 1.0 and r['wl'] > 0 and r['wr'] > 0,
            f"moved {r['fwd']:.2f} m ahead, {r['lat']:+.2f} m sideways; wheels L {r['wl']:+.1f} R {r['wr']:+.1f} rad/s")
    _, x0, y0, _, v0 = p.gt
    p.spin_sim(4.0, cmd=(0.0, 0.0))
    _, x1, y1, _, v1 = p.gt
    stop_d = math.hypot(x1 - x0, y1 - y0)
    rep.add('TEST 4  stop', v1 < 0.05, f"from {v0:.2f} m/s: stopped in {stop_d:.2f} m, residual {v1:.3f} m/s")

    r = run('reverse', (-0.5, 0.0), 4.0)
    rep.add('        reverse 0.5 m/s x 4 s', r['fwd'] < -1.0 and r['wl'] < 0 and r['wr'] < 0,
            f"moved {r['fwd']:.2f} m; wheels L {r['wl']:+.1f} R {r['wr']:+.1f} rad/s (both must be < 0)")
    p.spin_sim(3.0, cmd=(0.0, 0.0))

    r = run('left', (0.0, 0.6), 5.0)
    rep.add('TEST 2  turn LEFT in place', r['dyaw'] > 45.0 and r['wl'] < 0 < r['wr'],
            f"yaw {r['dyaw']:+.0f} deg (want > +45), drifted {math.hypot(r['fwd'], r['lat']):.2f} m; "
            f"wheels L {r['wl']:+.1f} R {r['wr']:+.1f} (L<0<R)")
    p.spin_sim(2.0, cmd=(0.0, 0.0))
    r = run('right', (0.0, -0.6), 5.0)
    rep.add('TEST 3  turn RIGHT in place', r['dyaw'] < -45.0 and r['wr'] < 0 < r['wl'],
            f"yaw {r['dyaw']:+.0f} deg (want < -45); wheels L {r['wl']:+.1f} R {r['wr']:+.1f} (R<0<L)")
    p.spin_sim(2.0, cmd=(0.0, 0.0))
    r = run('arc left', (0.6, 0.4), 6.0)
    rep.add('        arc left while driving', r['dyaw'] > 30.0 and r['wr'] > r['wl'] > 0,
            f"yaw {r['dyaw']:+.0f} deg over {math.hypot(r['fwd'], r['lat']):.1f} m (want > +30); "
            f"wheels L {r['wl']:+.1f} < R {r['wr']:+.1f}")
    p.spin_sim(3.0, cmd=(0.0, 0.0))
    r = run('arc right', (0.6, -0.4), 6.0)
    rep.add('        arc right while driving', r['dyaw'] < -30.0 and r['wl'] > r['wr'] > 0,
            f"yaw {r['dyaw']:+.0f} deg (want < -30); wheels L {r['wl']:+.1f} > R {r['wr']:+.1f}")
    p.spin_sim(3.0, cmd=(0.0, 0.0))

    # ---- WHEELIE: the worst command there is - full speed at once from standing still
    d = p.drive or {}
    top = float(d.get('max_linear', 3.0))
    mon = Monitor(p)
    t0 = p.now()
    reached = [None]

    def watch():
        mon.sample()
        if reached[0] is None and p.gt[4] >= top - 0.1:
            reached[0] = p.now() - t0
        return False
    p.spin_sim(7.0, cmd=(top, 0.0), until=watch)
    rep.add('WHEELIE  step to full speed', mon.max_nose_up < 3.0 and mon.max_front_off < 0.1,
            f'front pitched up at most {mon.max_nose_up:.1f} deg (want < 3); front pair L1+R1 off the ground '
            f'at most {mon.max_front_off:.2f} s (want < 0.1)')
    rep.add('        smooth acceleration', mon.max_acc <= float(d.get('max_accel', 1.0)) + 0.3,
            f"max {mon.max_acc:.2f} m/s2 (drive max_accel {d.get('max_accel')}); "
            f"{'%.1f s' % reached[0] if reached[0] else 'never'} to {top - 0.1:.1f} m/s (not instant)")
    rep.add('        open-road speed = 3.0 m/s', abs(mon.max_speed - 3.0) <= 0.1 and (reached[0] or 0) > 2.5,
            f'top speed {mon.max_speed:.2f} m/s (want 3.0 +- 0.1)')
    mon2 = Monitor(p)
    p.spin_sim(2.0, cmd=(8.0, 0.0), until=lambda: (mon2.sample(), False)[1])
    rep.add('        speed cap (8 m/s commanded)', mon2.max_speed <= 3.1, f'top speed {mon2.max_speed:.2f} m/s')
    _, x0, y0, _, v0 = p.gt
    mon3 = Monitor(p)
    p.spin_sim(4.0, cmd=(0.0, 0.0), until=lambda: (mon3.sample(), False)[1])
    _, x1, y1, _, v1 = p.gt
    dist = math.hypot(x1 - x0, y1 - y0)
    rep.add('BRAKING  from 3 m/s', v1 < 0.05 and mon3.max_dec <= float(d.get('max_decel', 2.0)) + 0.5
            and dist > 1.5 and mon3.max_nose_down < 4.0 and mon3.max_rear_off < 0.1,
            f'from {v0:.2f} m/s: stopped in {dist:.2f} m, max deceleration {mon3.max_dec:.2f} m/s2 '
            f"(drive max_decel {d.get('max_decel')}), nose down {mon3.max_nose_down:.1f} deg, rear pair off "
            f'{mon3.max_rear_off:.2f} s')


# ====================================================================== phase: mission
def sar_press(p, rep, name, want_active=True):
    p.press_sar('START' if want_active else 'STOP')
    ok = p.spin_sim(5.0, until=lambda: bool(p.sar.get('active')) == want_active, wall_limit=240)
    state = p.sar.get('state')
    rep.add(name, ok, f"SAR state -> {state!r} (nav: {p.nav.get('raw_state')})")
    return ok


def dist_b(p, b):
    return math.hypot(b[0] - p.gt[1], b[1] - p.gt[2])


def phase_mission(p, rep, goal, humans_minutes):
    if not p.wait_for('/sim/ground_truth', lambda: p.gt is not None, wall=300):
        rep.add('ground truth available', False, 'no /sim/ground_truth')
        return
    if not p.wait_for('/navigation/status', lambda: bool(p.nav), wall=300):
        rep.add('navigator running', False, 'no /navigation/status')
        return
    if not p.wait_for('/sar/state', lambda: bool(p.sar), wall=120):
        rep.add('SAR manager running', False, 'no /sar/state')
        return
    p.spin_sim(3.0)
    a = (p.gt[1], p.gt[2])
    tol = float(p.nav.get('goal_tolerance', 1.0))
    print(f'  A = ({a[0]:.2f}, {a[1]:.2f}) from ground truth; B = {goal}; goal tolerance {tol} m', flush=True)

    # TEST 7 - SAR at A, before any goal
    sar_press(p, rep, 'TEST 7  SAR at A', True)
    sar_press(p, rep, '        STOP SAR at A', False)
    p.spin_sim(2.0)

    # TEST 5 / 8 / 9 / 6 - drive to B, pressing SAR on the way and near B
    t_goal = p.now()
    p.send_goal(*goal)
    trail, reached_at, reached_d = [], None, None
    pressed_moving = pressed_near = False
    start_d = dist_b(p, goal)

    def watch():
        nonlocal reached_at, reached_d, pressed_moving, pressed_near
        if p.gt is None:
            return False
        trail.append((p.gt[1], p.gt[2]))
        d = dist_b(p, goal)
        st = p.nav.get('raw_state')
        if not pressed_moving and st == 'NAVIGATING' and p.gt[4] > 0.3 and d < 0.8 * start_d:
            pressed_moving = True
            sar_press(p, rep, 'TEST 8  SAR while driving A->B', True)
            sar_press(p, rep, '        STOP SAR (A->B resumes)', False)
        if not pressed_near and st == 'APPROACHING_GOAL' and d > tol:
            pressed_near = True
            sar_press(p, rep, 'TEST 9  SAR near B', True)
            sar_press(p, rep, '        STOP SAR (A->B resumes)', False)
        if st == 'GOAL_REACHED' and reached_at is None and p.sar.get('active') is False:
            reached_at, reached_d = p.now(), d
            return True
        return False
    budget = max(240.0, 8.0 * start_d)                     # >= 8 s per metre of A-B distance
    p.spin_sim(budget, until=watch, wall_limit=budget * 30)
    if reached_at is None:
        rep.add('TEST 5  reach B', False,
                f"no GOAL_REACHED in {budget:.0f} sim s; robot {dist_b(p, goal):.2f} m from B, nav {p.nav.get('raw_state')}")
        return
    rep.add('TEST 5  reach B', reached_d <= tol + 0.05,
            f"GOAL_REACHED after {reached_at - t_goal:.1f} sim s with the REAL robot {reached_d:.2f} m from B "
            f"(tolerance {tol})")
    if not pressed_moving:
        rep.add('TEST 8  SAR while driving A->B', False, 'the drive was too short to press it in NAVIGATING')
    if not pressed_near:
        rep.add('TEST 9  SAR near B', False, 'never saw APPROACHING_GOAL outside the tolerance')
    closest = min(math.hypot(goal[0] - x, goal[1] - y) for x, y in trail)
    p.spin_sim(4.0)                                         # does it stay there?
    final = dist_b(p, goal)
    rep.add('TEST 6  no overshoot, stays at B', final <= tol and final - closest < 0.5 and p.gt[4] < 0.1,
            f"closest {closest:.2f} m, 4 s later {final:.2f} m, speed {p.gt[4]:.3f} m/s")

    # TEST 10 / 11
    sar_press(p, rep, 'TEST 10 SAR after B', True)
    p.spin_sim(3.0)
    sar_press(p, rep, '        STOP SAR after B', False)
    p.spin_sim(2.0)
    sar_press(p, rep, 'TEST 11 SAR again after STOP', True)
    if humans_minutes > 0:
        print(f'  searching for up to {humans_minutes} simulated minutes for a thermal human ...', flush=True)
        ok = p.spin_sim(humans_minutes * 60.0, until=lambda: len(p.humans) > 0, wall_limit=humans_minutes * 3600)
        if ok and p.humans:
            h = p.humans[0]
            rep.add('HUMAN detected (thermal)', True,
                    f"{h.get('human_id')} at X {h.get('x'):.1f} Y {h.get('y'):.1f}, {h.get('temperature', 0) - 273.15:.1f} C")
        else:
            rep.add('HUMAN detected (thermal)', False, f'none confirmed within {humans_minutes} sim minutes')
    sar_press(p, rep, '        STOP SAR', False)


# ====================================================================== phase: scenario
def _box_distance(x, y, box):
    x0, x1, y0, y1 = box
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def _profile(nav, c):
    ds, vs = nav.get('clearance_distances'), nav.get('clearance_speeds')
    if not ds:
        return math.inf
    if c < ds[0]:
        return 0.0
    for (c0, v0), (c1, v1) in zip(zip(ds, vs), list(zip(ds, vs))[1:]):
        if c <= c1:
            return v0 + (v1 - v0) * (c - c0) / (c1 - c0)
    return vs[-1]


def footprint_gap(p, px, py, radius):
    """Distance (m) from the rover's 1.53 x 1.12 m body rectangle (true pose) to a person's body circle;
    negative = they overlap."""
    _, x, y, yaw, _ = p.gt
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    u, w = c * dx + s * dy - 0.02, -s * dx + c * dy
    ex, ey = abs(u) - 1.53 / 2, abs(w) - 1.12 / 2
    inside = ex < 0 and ey < 0
    d = max(ex, ey) if inside else math.hypot(max(ex, 0.0), max(ey, 0.0))
    return d - radius


def start_people_bridge(sc):
    """Bridge the scenario people's gz odometry (not in bridge.yaml) for the duration of the test."""
    args = [f"{q['odometry']}@nav_msgs/msg/Odometry[gz.msgs.Odometry" for q in sc.get('people', [])]
    if not args:
        return None
    try:
        return subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *args,
                                 '--ros-args', '-p', 'use_sim_time:=true'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        return None


def person_positions(p, sc):
    out = []
    for q in sc.get('people', []):
        pos = p.people.get(q['name'])
        if pos is None and q['kind'] == 'static':
            pos = (q['at'][0], q['at'][1], 0.0)
        if pos is not None:
            out.append((q['name'], pos, q['radius']))
    return out


def phase_push(p, rep, sc):
    """human_block: drive straight at a standing person (0.8 m/s commanded the whole time). With real
    collision geometry the rover stops at the contact; it must never overlap the person's body."""
    name = sc['name']
    if not p.wait_for('/sim/ground_truth', lambda: p.gt is not None, wall=300):
        rep.add(f'{name}: ground truth', False, 'no /sim/ground_truth')
        return
    p.spin_sim(3.0)
    q = sc['people'][0]
    min_gap, speeds, start = [math.inf], [], (p.gt[1], p.gt[2])
    moved = [0.0]

    def watch():
        for _, pos, r in person_positions(p, sc):
            min_gap[0] = min(min_gap[0], footprint_gap(p, pos[0], pos[1], r))
            moved[0] = max(moved[0], math.hypot(pos[0] - q['at'][0], pos[1] - q['at'][1]))
        speeds.append((p.gt[0], p.gt[4]))
        return False
    p.spin_sim(float(sc['max_time']), cmd=(0.8, 0.0), until=watch, wall_limit=float(sc['max_time']) * 40 + 120)
    p.spin_sim(1.0, cmd=(0.0, 0.0))
    t_end = speeds[-1][0] if speeds else 0.0
    tail = [v for t, v in speeds if t_end - t <= 4.0]
    v_tail = sum(tail) / max(1, len(tail))
    travelled = math.hypot(p.gt[1] - start[0], p.gt[2] - start[1])
    reach = math.hypot(q['at'][0] - start[0], q['at'][1] - start[1])
    rep.add(f'{name}: stopped by the person', v_tail < 0.15,
            f'mean speed over the last 4 s {v_tail:.2f} m/s while 0.8 m/s was still commanded; travelled '
            f'{travelled:.2f} m towards a person {reach:.1f} m ahead')
    rep.add(f'{name}: never passes through', min_gap[0] > -0.08,
            f'closest rover-body-to-person-body distance {min_gap[0]:+.2f} m (negative = overlap; contact '
            f'solver penetration is a few cm)')
    rep.add(f'{name}: person is a real obstacle', moved[0] < 0.1,
            f'the standing person moved {moved[0]:.2f} m (static casualty: must not be pushed away)')


def phase_scenario(p, rep, sc, nav_cfg):
    name = sc['name']
    print(f"  scenario {name}: {sc['desc']}", flush=True)
    if not p.wait_for('/sim/ground_truth', lambda: p.gt is not None, wall=300):
        rep.add(f'{name}: ground truth', False, 'no /sim/ground_truth')
        return
    if not p.wait_for('/navigation/status', lambda: bool(p.nav), wall=300):
        rep.add(f'{name}: navigator running', False, 'no /navigation/status')
        return
    p.spin_sim(3.0)
    goal = tuple(sc['goal'])
    tol = float(p.nav.get('goal_tolerance', 0.3))
    mon = Monitor(p)
    states, excess = [], [0.0, None]
    still = [0.0, 0.0, None]                          # current, worst, state during the worst
    yaw_prev = [p.gt[3], p.gt[0]]
    reached = [None, None]
    person_gap = [math.inf, None]
    near_obstacle = [math.inf]
    in_pit = [False]
    p.send_goal(*goal)
    t_goal = p.now()

    def watch():
        mon.sample()
        t, x, y, yaw, sp = p.gt
        st = p.nav.get('raw_state') or p.nav.get('state')
        if not states or states[-1] != st:
            states.append(st)
        dt = max(1e-6, t - yaw_prev[1])
        yaw_rate = abs(wrap(yaw - yaw_prev[0])) / dt
        yaw_prev[:] = [yaw, t]
        if st not in ('GOAL_REACHED', 'WAITING') and t - t_goal > 3.0 and sp < 0.05 and yaw_rate < 0.05:
            still[0] += dt
            if still[0] > still[1]:
                still[1], still[2] = still[0], st
        else:
            still[0] = 0.0
        c = p.nav.get('clearance')
        if c is not None and sp > 0.05:
            e = sp - _profile(nav_cfg, float(c))
            if e > excess[0]:
                excess[:] = [e, (round(float(c), 2), round(sp, 2))]
        for pname, pos, r in person_positions(p, sc):
            g = footprint_gap(p, pos[0], pos[1], r)
            if g < person_gap[0]:
                person_gap[:] = [g, pname]
        if 'obstacle_box' in sc:
            near_obstacle[0] = min(near_obstacle[0], _box_distance(x, y, sc['obstacle_box']))
        if 'pit_box' in sc and _box_distance(x, y, sc['pit_box']) == 0.0:
            in_pit[0] = True
        if st == 'GOAL_REACHED' and reached[0] is None:
            reached[:] = [t - t_goal, math.hypot(goal[0] - x, goal[1] - y)]
            return True
        return False
    p.spin_sim(float(sc['max_time']), until=watch, wall_limit=float(sc['max_time']) * 40 + 120)
    p.spin_sim(3.0, until=lambda: (mon.sample(), False)[1])
    final = math.hypot(goal[0] - p.gt[1], goal[1] - p.gt[2])
    path = ' > '.join(s for s in states if s)[:160]
    rep.add(f'{name}: reach B', reached[0] is not None and reached[1] <= tol + 0.05 and final <= tol + 0.1,
            (f'GOAL_REACHED after {reached[0]:.1f} sim s, real robot {reached[1]:.2f} m from B, {final:.2f} m 3 s later'
             if reached[0] is not None else f'not reached in {sc["max_time"]:.0f} sim s: {final:.2f} m from B')
            + f'  [{path}]')
    rep.add(f'{name}: no wheelie', mon.max_front_off < 0.5,
            f'front pair L1+R1 both off the ground at most {mon.max_front_off:.2f} s (want < 0.5: crests only); '
            f"max tilt {mon.max_tilt:.1f} deg on a {sc['slope_deg']:.0f} deg scenario")
    top_ok = mon.max_speed <= 3.1 and (sc['expect'].get('top_speed') is None or mon.max_speed >= 2.9)
    rep.add(f'{name}: speed / acceleration', top_ok and mon.max_acc <= 1.5,
            f'top {mon.max_speed:.2f} m/s (max 3.0' + (', want 3.0 here' if sc['expect'].get('top_speed') else '')
            + f'), max acceleration {mon.max_acc:.2f} m/s2, max braking {mon.max_dec:.2f} m/s2')
    rep.add(f'{name}: never a standing stop', still[1] < 3.0,
            f'longest time standing still (not turning) before B: {still[1]:.1f} s' + (f' in {still[2]}' if still[2] else ''))
    rep.add(f'{name}: clearance speed profile', excess[0] <= 0.7,
            f'speed above the clearance target by at most {excess[0]:.2f} m/s (<= 0.7: braking at 2 m/s2 '
            f'after a hazard first appears on the map)'
            + (f' (clearance {excess[1][0]} m at {excess[1][1]} m/s - braking lag)' if excess[1] else ''))
    if 'obstacle_box' in sc:
        rep.add(f'{name}: steered round the wall', near_obstacle[0] >= 0.56,
                f'robot centre came within {near_obstacle[0]:.2f} m of the wall (half width 0.56 m)')
    if sc.get('people'):
        want = float(sc['expect'].get('min_gap', 0.3))
        seen = any(q['name'] in p.people or q['kind'] == 'static' for q in sc['people'])
        rep.add(f'{name}: never touched the person', seen and person_gap[0] > 0.02,
                (f'closest rover body to {person_gap[1]}: {person_gap[0]:.2f} m' if seen else
                 'no person pose received (odometry bridge)'))
        rep.add(f'{name}: kept a safe distance', seen and person_gap[0] >= want,
                f'closest {person_gap[0]:.2f} m, want >= {want:.1f} m (human safety distance 2.0 m in the planner)')
        if p.nav.get('collision'):
            rep.add(f'{name}: dynamic-obstacle layer active', True,
                    f"collision status at the end: {p.nav.get('collision_status')}, tracker "
                    f"{p.nav['collision'].get('tracker')}")
    if 'pit_box' in sc:
        zmin = sc['expect'].get('min_z', -math.inf)
        rep.add(f'{name}: never fell / entered the pit', not in_pit[0] and mon.min_z >= zmin,
                f"lowest body height {mon.min_z:.2f} m (plateau top {sc['goal_z']:.2f} m); centre over the pit: {in_pit[0]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--phase', choices=('motion', 'mission', 'scenario'), required=True)
    ap.add_argument('--scenario', default='')
    ap.add_argument('--meta', default='', help='scenarios.json from scenario_worlds.py')
    ap.add_argument('--config', default='', help='sar_mission.yaml (for the clearance profile)')
    ap.add_argument('--goal', nargs=2, type=float, default=(-3.0, -2.0))
    ap.add_argument('--humans', type=float, default=0.0, help='sim minutes of SAR to wait for H1 (0 = skip)')
    a, _ = ap.parse_known_args()
    rclpy.init()
    sc = json.load(open(a.meta))[a.scenario] if a.phase == 'scenario' else {}
    p = Probe(own_cmd_vel=a.phase == 'motion' or sc.get('drive') == 'probe')
    rep = Report()
    bridge = start_people_bridge(sc) if sc else None
    for q in sc.get('people', []):
        p.watch_person(q['name'], q['odometry'])
    try:
        if a.phase == 'motion':
            phase_motion(p, rep)
        elif a.phase == 'scenario' and sc.get('drive') == 'probe':
            phase_push(p, rep, sc)
        elif a.phase == 'scenario':
            nav_cfg = {}
            try:
                import yaml
                nav_cfg = yaml.safe_load(open(a.config))['/**']['ros__parameters']['navigation']
            except Exception as e:                      # the profile check is then skipped
                print(f'  (no clearance profile: {e})', flush=True)
            phase_scenario(p, rep, sc, nav_cfg)
        else:
            phase_mission(p, rep, tuple(a.goal), a.humans)
    except KeyboardInterrupt:
        pass
    if bridge is not None:
        bridge.terminate()
    ok = rep.summary()
    p.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
