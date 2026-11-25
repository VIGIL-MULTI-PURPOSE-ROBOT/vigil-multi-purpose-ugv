#!/usr/bin/env python3
"""VIGIL Rock Terrain regression monitor v2 (read-only observer; publishes nothing).

Watches one vision_nav.launch.py scenario run and writes into --out:
  timeline.jsonl   state transitions, cliff detections, terminal events, periodic samples
  result.json      measured metrics + outcome category (see metrics.classify)
  trajectory.csv   ground-truth samples (sim time) every 0.25 s

Measured (never estimated): sim and wall time, real-time factor from /clock, path length from
time-ordered ground-truth samples (metrics.PathLength), max roll and max pitch separately, total and
hazard replans, recoveries, explore attempts, longest time without >= 0.1 m progress towards B,
minimum footprint clearance to the TRUE drop edges (cliff scenarios), sensor rates, final navigation
state / failure reason, and - after a terminal failure - whether the robot actually stopped.

Ends at GOAL_REACHED (+3 s), a terminal failure (+5 s to verify the stop), FELL / rollover, the
sim-time limit, the wall-time limit, or a startup failure (no /clock, no ground truth, no status).

Collision with the environment is NOT instrumented (no body contact sensor); the clearance metric is
geometric (ground-truth pose vs. known scenario geometry) and is no substitute for one.
"""
import argparse
import importlib.util
import json
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import (PathLength, Progress, RateMeter, real_time_factor, quat_roll_pitch, classify,  # noqa: E402
                     verdict, TERMINAL_FAILURES, MIN_CLEARANCE)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class DropMap:
    """Distance (m) from a point on the plateau top to the nearest drop edge (scenario geometry)."""

    def __init__(self, objects, cliff_h, res=0.05):
        from scipy.ndimage import distance_transform_edt
        self.res, self.x0, self.y0 = res, -6.0, -8.0
        nx, ny = int(24.0 / res), int(16.0 / res)
        X, Y = np.meshgrid(self.x0 + (np.arange(nx) + .5) * res, self.y0 + (np.arange(ny) + .5) * res, indexing='ij')
        top = np.zeros_like(X, bool)
        for o in objects:
            if o.get('type') != 'box' or abs(o.get('pitch', 0.0)) > 1e-6:
                continue
            (cx, cy, cz), (sx, sy, sz) = o['pos'], o['size']
            if cz + sz / 2 >= cliff_h - 0.1:
                top |= (abs(X - cx) <= sx / 2) & (abs(Y - cy) <= sy / 2)
        self.dist = distance_transform_edt(top) * res

    def clearance(self, pts):
        i = np.floor((pts[:, 0] - self.x0) / self.res).astype(int)
        j = np.floor((pts[:, 1] - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.dist.shape[0]) & (j >= 0) & (j < self.dist.shape[1])
        d = np.zeros(len(pts))
        d[ok] = self.dist[i[ok], j[ok]]
        return float(d.min())


def footprint_points(x, y, yaw, L, W, cx, n=12):
    u = np.linspace(-1, 1, n)
    loc = np.concatenate([np.stack([cx + L / 2 * u, np.full(n, W / 2)], 1), np.stack([cx + L / 2 * u, np.full(n, -W / 2)], 1),
                          np.stack([np.full(n, cx + L / 2), W / 2 * u], 1), np.stack([np.full(n, cx - L / 2), W / 2 * u], 1)])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * loc[:, 0] - s * loc[:, 1], y + s * loc[:, 0] + c * loc[:, 1]], 1)


class Monitor(Node):
    def __init__(self, a):
        super().__init__('vigil_regression_monitor')
        self.a, self.out = a, Path(a.out)
        self.tl = open(self.out / 'timeline.jsonl', 'w')
        self.wall0 = time.monotonic()
        cfg = yaml.safe_load(Path(a.config).read_text())['/**']['ros__parameters']
        r = cfg['robot']
        self.L, self.W, self.CX = r['length'], r['width'], r['footprint_center_x']
        sc = load_module(a.scenarios, 'scenarios')
        self.spec = sc.SCENARIOS[a.scenario]
        self.goal = tuple(self.spec['goal'])
        self.cliff_h = sc.CLIFF_H
        self.cliff = self.spec['spawn_z'] >= sc.CLIFF_H - 1e-6
        self.dropmap = DropMap(self.spec['objects'], sc.CLIFF_H) if self.cliff else None
        # state
        self.sim = self.sim0 = None
        self.clock_first = self.clock_last = None      # (sim, wall)
        self.first = {}
        self.status = None
        self.states, self.raw_states = [], []
        self.goal_t = self.terminal_t = self.terminal_state = None
        self.end = None
        self.path = PathLength()
        self.progress = Progress(self.goal, step=0.1)
        self.rates = {k: RateMeter() for k in ('depth', 'cmd', 'status', 'terrain', 'gt', 'clock')}
        self.max_roll = self.max_pitch = self.max_tilt = 0.0
        self.min_clear, self.min_clear_at = None, None
        self.fell = None
        self.spawn_z = self.z_min = None
        self.last_cmd_nonzero_t = None
        self.speed_after_terminal = 0.0
        self.cliff_t = None
        self.max_cliff_cells = 0
        self.traj, self.last_traj_t = [], -1e9
        self.recovery_entries = 0
        self.errors = []
        s = qos_profile_sensor_data
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(String, '/navigation/status', self.on_status, 10)
        self.create_subscription(String, '/vision/cliff', self.on_cliff, 10)
        self.create_subscription(String, '/vision/terrain', lambda m: self.tick_rate('terrain'), 10)
        self.create_subscription(Odometry, a.truth_topic, self.on_gt, s)  # /eval/ground_truth when truth is isolated
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Image, '/camera/depth_image', lambda m: self.tick_rate('depth'), s)
        self.create_timer(0.5, self.check)
        self.log(event='monitor_start', scenario=a.scenario, expect=self.spec.get('expect'), goal=self.goal)

    # ------------------------------------------------------------------ helpers
    def wall(self):
        return time.monotonic() - self.wall0

    def tsim(self):
        return None if self.sim is None else self.sim - self.sim0

    def log(self, **kv):
        kv.setdefault('wall', round(self.wall(), 2))
        kv.setdefault('sim', None if self.tsim() is None else round(self.tsim(), 2))
        if not self.tl.closed:
            self.tl.write(json.dumps(kv) + '\n')
            self.tl.flush()

    def tick_rate(self, k):
        if self.sim is not None:              # rates are measured once the simulation clock runs
            self.rates[k].add(self.tsim(), self.wall())
        if k not in self.first:
            self.first[k] = round(self.wall(), 2)
            self.log(event='first_message', topic=k)

    # ------------------------------------------------------------------ callbacks
    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.sim0 is None:
            self.sim0 = t
            self.clock_first = (t, self.wall())
        self.sim = t
        self.clock_last = (t, self.wall())
        self.tick_rate('clock')

    def on_status(self, m):
        self.tick_rate('status')
        if 'status' not in self.first or not (self.out / 'nav_seen').exists():
            (self.out / 'nav_seen').write_text(str(round(self.wall(), 2)))
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self.status = st
        disp, raw = st.get('state'), st.get('raw_state')
        if not self.states or self.states[-1]['state'] != disp:
            self.states.append(dict(state=disp, sim=None if self.tsim() is None else round(self.tsim(), 2)))
            self.log(event='state', state=disp, raw_state=raw, reason=st.get('reason'), replans=st.get('replans'),
                     hazard_replans=st.get('hazard_replans'), recoveries=st.get('recoveries'))
        if not self.raw_states or self.raw_states[-1] != raw:
            if raw == 'RECOVERY':
                self.recovery_entries += 1
            self.raw_states.append(raw)
        if raw == 'GOAL_REACHED' and self.goal_t is None:
            self.goal_t = self.tsim()
        if raw in TERMINAL_FAILURES and self.terminal_t is None:
            self.terminal_t, self.terminal_state = self.tsim(), raw
            self.log(event='terminal_failure', state=raw, reason=st.get('failure_reason'))

    def on_cliff(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            return
        self.max_cliff_cells = max(self.max_cliff_cells, int(c.get('cliff_cells') or 0))
        if c.get('detected') and self.cliff_t is None:
            self.cliff_t = self.tsim()
            self.log(event='cliff_detected', drop=c.get('drop'), distance=c.get('distance'))

    def on_gt(self, m):
        self.tick_rate('gt')
        if self.sim is None:
            return
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = stamp - self.sim0 if stamp > 0 else self.tsim()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        roll, pitch = quat_roll_pitch(q.x, q.y, q.z, q.w)
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(math.radians(roll)) * math.cos(math.radians(pitch))))))
        if self.spawn_z is None:
            self.spawn_z = p.z
        self.z_min = p.z if self.z_min is None else min(self.z_min, p.z)
        raw = (self.status or {}).get('raw_state')
        active = raw not in (None, 'WAITING')
        self.path.add(t, p.x, p.y)
        if active:
            self.progress.add(t, p.x, p.y)
            self.max_roll = max(self.max_roll, abs(roll))
            self.max_pitch = max(self.max_pitch, abs(pitch))
            self.max_tilt = max(self.max_tilt, tilt)
            if self.cliff:
                c = self.dropmap.clearance(footprint_points(p.x, p.y, yaw, self.L, self.W, self.CX))
                if self.min_clear is None or c < self.min_clear:
                    self.min_clear, self.min_clear_at = c, [round(p.x, 2), round(p.y, 2), round(t, 2)]
                if p.z < self.cliff_h - 0.5 and self.fell is None:
                    self.fell = dict(sim=round(t, 2), x=round(p.x, 2), y=round(p.y, 2), z=round(p.z, 2))
                    self.log(event='FELL', **self.fell)
            if tilt > 70.0 and self.fell is None:
                self.fell = dict(sim=round(t, 2), rollover_tilt=round(tilt, 1))
                self.log(event='ROLLOVER', **self.fell)
        if self.terminal_t is not None and t > self.terminal_t + 2.0:      # allow drive.py ramp-down
            v = m.twist.twist.linear
            self.speed_after_terminal = max(self.speed_after_terminal, math.hypot(v.x, v.y))
        if t - self.last_traj_t >= 0.25:
            self.traj.append((round(t, 3), round(p.x, 4), round(p.y, 4), round(p.z, 4), round(roll, 2), round(pitch, 2)))
            self.last_traj_t = t

    def on_cmd(self, m):
        self.tick_rate('cmd')
        if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3:
            self.last_cmd_nonzero_t = self.tsim()

    # ------------------------------------------------------------------ end conditions
    def check(self):
        if self.end:
            return
        w, ts = self.wall(), self.tsim()
        if self.sim is None and w > self.a.startup_timeout:
            return self.finish('NO_SIM_CLOCK')
        if 'gt' not in self.first and w > self.a.startup_timeout:
            return self.finish('NO_GROUND_TRUTH')
        if self.status is None and w > self.a.startup_timeout:
            return self.finish('NO_NAV_STATUS')
        if self.fell:
            return self.finish('FELL_OR_ROLLOVER')
        if self.goal_t is not None and ts - self.goal_t >= 3.0:
            return self.finish('GOAL_REACHED')
        if self.terminal_t is not None and ts - self.terminal_t >= 5.0:
            return self.finish('TERMINAL')
        if ts is not None and ts >= self.a.sim_limit:
            return self.finish('SIM_TIME_LIMIT')
        if w >= self.a.wall_limit:
            return self.finish('WALL_TIME_LIMIT')
        if int(w * 2) % 20 == 0 and self.status:
            st = self.status
            self.log(event='sample', state=st.get('state'), x=st.get('x'), y=st.get('y'),
                     hazard_replans=st.get('hazard_replans'), no_progress_for=self.progress.current(ts or 0))

    def finish(self, why):
        self.end = why
        st = self.status or {}
        exp = (self.spec.get('expect') or {}).get('outcome')
        stopped_ok = None
        if self.terminal_t is not None:
            late_cmd = self.last_cmd_nonzero_t is not None and self.last_cmd_nonzero_t > self.terminal_t + 0.25
            stopped_ok = (not late_cmd) and self.speed_after_terminal <= 0.05
        cat = classify(why, st.get('raw_state'), self.fell, stopped_ok, exp)
        cf, cl = self.clock_first or (None, None), self.clock_last or (None, None)
        recov = st.get('recoveries')
        res = dict(
            scenario=self.a.scenario, trial=self.a.trial, expected=exp, end_condition=why, category=cat,
            verdict=verdict(cat, exp, self.min_clear),
            verdict_note=(f'FAIL: footprint came within {self.min_clear:.3f} m of the true drop edge '
                          f'(required >= {MIN_CLEARANCE:.2f} m)'
                          if self.min_clear is not None and self.min_clear < MIN_CLEARANCE - 1e-9 else None),
            final_state=st.get('state'), raw_state=st.get('raw_state'), failure_reason=st.get('failure_reason'),
            goal_reached=self.goal_t is not None, goal_time_sim_s=None if self.goal_t is None else round(self.goal_t, 2),
            terminal_failure=self.terminal_state,
            terminal_time_sim_s=None if self.terminal_t is None else round(self.terminal_t, 2),
            stopped_after_terminal=stopped_ok, max_speed_after_terminal=round(self.speed_after_terminal, 3),
            sim_elapsed_s=None if self.tsim() is None else round(self.tsim(), 2),
            wall_elapsed_s=round(self.wall(), 2),
            real_time_factor=real_time_factor(cf[0], cl[0], cf[1], cl[1]),
            path_length_m=round(self.path.length, 2), path_samples=self.path.samples,
            path_duplicates_ignored=self.path.duplicates, path_glitches_ignored=self.path.glitches,
            replans=st.get('replans'), hazard_replans=st.get('hazard_replans'),
            recoveries=recov if recov is not None else self.recovery_entries,
            recoveries_source='navigator status' if recov is not None else 'RECOVERY state entries (baseline code)',
            explore_attempts=st.get('explore_attempts'), shield_vetoes=st.get('shield_vetoes'),
            body_overlap_ticks=st.get('body_overlap_ticks'),
            longest_no_progress_s=round(self.progress.longest, 2),
            min_footprint_clearance_m=None if self.min_clear is None else round(self.min_clear, 3),
            min_clearance_at=self.min_clear_at,
            max_roll_deg=round(self.max_roll, 2), max_pitch_deg=round(self.max_pitch, 2),
            max_tilt_deg=round(self.max_tilt, 2), fell_or_rollover=self.fell,
            ground_truth_z=dict(spawn=self.spawn_z, min=self.z_min),
            cliff_first_detected_sim_s=None if self.cliff_t is None else round(self.cliff_t, 2),
            max_cliff_cells=self.max_cliff_cells,
            collision_instrumentation='none (geometric clearance only)',
            rates_hz_sim={k: r.hz('sim') for k, r in self.rates.items()},
            rates_hz_wall={k: r.hz('wall') for k, r in self.rates.items()},
            message_counts={k: r.n for k, r in self.rates.items()},
            first_message_wall_s=self.first, state_transitions=self.states, last_status=st)
        (self.out / 'result.json').write_text(json.dumps(res, indent=2))
        with open(self.out / 'trajectory.csv', 'w') as f:
            f.write('sim_s,x,y,z,roll_deg,pitch_deg\n')
            for row in self.traj:
                f.write(','.join(str(v) for v in row) + '\n')
        self.log(event='monitor_end', end_condition=why, category=cat)
        self.tl.close()
        raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--trial', type=int, default=1)
    ap.add_argument('--out', required=True)
    ap.add_argument('--scenarios', required=True)
    ap.add_argument('--config', required=True)
    ap.add_argument('--sim-limit', type=float, default=150.0)
    ap.add_argument('--wall-limit', type=float, default=1200.0)
    ap.add_argument('--startup-timeout', type=float, default=300.0)
    ap.add_argument('--truth-topic', default='/sim/ground_truth',
                    help='ground-truth topic (evaluation only); /eval/ground_truth with truth isolation')
    a = ap.parse_args()
    rclpy.init()
    node = None
    try:
        node = Monitor(a)
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        if node and not node.end:
            try:
                node.finish('INTERRUPTED')
            except SystemExit:
                pass
    except Exception:  # noqa: BLE001 - a monitor crash must be reported, never hidden
        Path(a.out, 'monitor_error.txt').write_text(traceback.format_exc())
        if node and not node.end:
            try:
                node.finish('MONITOR_ERROR')
            except SystemExit:
                pass
            except Exception:
                pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
