#!/usr/bin/env python3
"""VIGIL isolated evaluator: the only node meant to read simulator truth in GPS-denied runs.

Run beside a launch (same ROS_DOMAIN_ID), stop with SIGINT/SIGTERM; it then writes:
  <out>/eval_trajectory.csv   t, ground truth x y z yaw, estimate x y z yaw (empty when unavailable)
  <out>/guard.json            GPS-denied guard verdict for the whole run (+ every distinct finding)
  <out>/eval_report.json      localization metrics (loc_metrics.evaluate) + guard verdict + settings

  python3 evaluator.py --out DIR --mode isolation|baseline [--truth-topic /eval/ground_truth]
                       [--est-frame map --base-frame base_footprint] [--est-topic TOPIC]

The estimate is read from TF (est-frame -> base-frame) at each ground-truth stamp, or from an
Odometry topic (--est-topic). Milestone 1 has no estimator in Rock Terrain: the report then shows
availability 0 and no error metrics - by design, nothing is filled in.
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

import guard_core  # noqa: E402
import loc_metrics  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Evaluator(Node):
    def __init__(self, a):
        super().__init__('vigil_evaluator', parameter_overrides=[])
        self.a = a
        self.tf = Buffer(cache_time=Duration(seconds=120.0))
        self.listener = TransformListener(self.tf, self)
        self.pending, self.samples, self.gt_count, self.last_t = [], [], 0, -1e18
        self.est_latest = []          # (t, x, y, z, yaw) when --est-topic is used
        self.history = []
        self.create_subscription(Odometry, a.truth_topic, self.on_truth, qos_profile_sensor_data)
        if a.est_topic:
            self.create_subscription(Odometry, a.est_topic, self.on_est, qos_profile_sensor_data)
        self.create_timer(1.0, self.check_graph)
        self.create_timer(0.2, self.associate)
        self.t0 = time.monotonic()

    # ------------------------------------------------------------------ data
    def on_truth(self, m):
        self.gt_count += 1
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.last_t < self.a.sample_dt:      # also drops repeated / out-of-order stamps
            return
        self.last_t = t
        self.pending.append((t, (p.x, p.y, p.z, yaw_of(q)), m.header.stamp))

    def on_est(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.est_latest.append((t, p.x, p.y, p.z, yaw_of(q)))
        if len(self.est_latest) > 4000:
            del self.est_latest[:-2000]

    def estimate_at(self, t, stamp):
        if self.a.est_topic:
            best = min(self.est_latest, key=lambda e: abs(e[0] - t), default=None)
            if best is None or abs(best[0] - t) > self.a.max_dt:
                return None
            return best[1:]
        try:
            tr = self.tf.lookup_transform(self.a.est_frame, self.a.base_frame, Time.from_msg(stamp))
        except Exception:  # noqa: BLE001 - any TF failure = estimate unavailable at this time
            return None
        v, q = tr.transform.translation, tr.transform.rotation
        return (v.x, v.y, v.z, yaw_of(q))

    def associate(self, final=False):
        now = self.get_clock().now().nanoseconds * 1e-9
        keep = []
        for t, gt, stamp in self.pending:
            if not final and now - t < self.a.tf_wait:
                keep.append((t, gt, stamp))
                continue
            self.samples.append(dict(t=t, gt=gt, est=self.estimate_at(t, stamp)))
        self.pending = keep

    # ------------------------------------------------------------------ guard
    def check_graph(self):
        graph = {}
        for name, _types in self.get_topic_names_and_types():
            if not (name in guard_core.TRUTH_SOURCE or name in guard_core.EVAL_TOPICS or name == '/tf'
                    or name.startswith(guard_core.ORACLE_PREFIX) or name.startswith(guard_core.CONTACT_PREFIX)):
                continue
            fq = lambda i: (i.node_namespace.rstrip('/') + '/' + i.node_name)  # noqa: E731
            graph[name] = dict(subs=[fq(i) for i in self.get_subscriptions_info_by_topic(name)],
                               pubs=[fq(i) for i in self.get_publishers_info_by_topic(name)])
        for t in guard_core.TRUTH_SOURCE:       # topics with subscribers only still show up here
            graph.setdefault(t, dict(subs=[], pubs=[]))
        j = guard_core.judge(self.a.mode, graph, allowed=guard_core.DEFAULT_ALLOWED + tuple(self.a.allow))
        j['wall_s'] = round(time.monotonic() - self.t0, 1)
        self.history.append(j)

    # ------------------------------------------------------------------ output
    def write(self):
        self.associate(final=True)
        os.makedirs(self.a.out, exist_ok=True)
        with open(os.path.join(self.a.out, 'eval_trajectory.csv'), 'w', newline='') as f:
            w = csv.writer(f, lineterminator='\n')
            w.writerow(['t', 'gt_x', 'gt_y', 'gt_z', 'gt_yaw', 'est_x', 'est_y', 'est_z', 'est_yaw'])
            for s in self.samples:
                e = s['est'] or ('', '', '', '')
                w.writerow([f"{s['t']:.3f}"] + [f'{v:.4f}' for v in s['gt']] +
                           [(f'{v:.4f}' if v != '' else '') for v in e])
        guard = guard_core.merge(self.history)
        with open(os.path.join(self.a.out, 'guard.json'), 'w') as f:
            json.dump(guard, f, indent=1)
        metrics = loc_metrics.evaluate(self.samples)
        report = dict(mode=self.a.mode, truth_topic=self.a.truth_topic, truth_messages=self.gt_count,
                      estimate_source=(self.a.est_topic or f'tf {self.a.est_frame}->{self.a.base_frame}'),
                      guard_verdict=guard.get('verdict'), metrics=metrics)
        with open(os.path.join(self.a.out, 'eval_report.json'), 'w') as f:
            json.dump(report, f, indent=1)
        print(json.dumps(dict(guard=guard.get('verdict'), truth_messages=self.gt_count,
                              samples=metrics['samples'], availability=metrics['availability'],
                              gt_max_displacement_m=metrics['gt_max_displacement_m'])), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--mode', choices=['isolation', 'baseline'], required=True)
    ap.add_argument('--truth-topic', default=None, help='default: /eval/ground_truth (isolation), /sim/ground_truth (baseline)')
    ap.add_argument('--est-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--est-topic', default='')
    ap.add_argument('--sample-dt', type=float, default=0.1, help='s between evaluated samples')
    ap.add_argument('--tf-wait', type=float, default=0.5, help='s to wait for TF before looking up')
    ap.add_argument('--max-dt', type=float, default=0.15, help='s, est-topic association window')
    ap.add_argument('--allow', action='append', default=[], help='extra allowed node name pattern')
    a = ap.parse_args()
    if a.truth_topic is None:
        a.truth_topic = '/eval/ground_truth' if a.mode == 'isolation' else '/sim/ground_truth'
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    n = Evaluator(a)
    stop = {'now': False}

    def handler(*_):
        stop['now'] = True
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    try:
        while rclpy.ok() and not stop['now']:
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.write()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
