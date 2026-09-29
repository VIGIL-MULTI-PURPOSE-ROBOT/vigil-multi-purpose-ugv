#!/usr/bin/python3
"""Gazebo check of the dynamic-obstacle layer, acceleration and simulation speed (observe only).

Run it in a second terminal while the mission runs:

    cd ~/Documents/military_world/sar_ws && source install/setup.bash
    export ROS_DOMAIN_ID=<same as the mission> GZ_PARTITION=vigil_sar
    ros2 run vigil_sar dynamic_check.py            # Ctrl+C to finish

Every 10 s (and at the end, into sar_ws/diagnosis/dynamic_check.json) it reports:
  * tracked obstacles by class, how many moving, the closest approach to a PERSON and to a vehicle
    (edge distance, from the tracker) against the configured safety distances
  * how long the collision status was CLEAR / COLLISION RISK / AVOIDING
  * navigator states (so a stop-forever would show as a long WAITING / RECOVERY streak)
  * the acceleration level and the peak commanded acceleration
  * the MEASURED simulation speed (sim seconds per wall second) against the target
Nothing is published; the mission is not touched.
"""
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from sim_speed import RtfMeter  # noqa: E402


class Check(Node):
    def __init__(self):
        from rclpy.parameter import Parameter
        # simulation clock: the measured speed is sim seconds per wall second
        super().__init__('dynamic_check', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.out = Path(self.declare_parameter('output', os.path.join(os.getcwd(), 'diagnosis/dynamic_check.json')).value)
        self.out.parent.mkdir(parents=True, exist_ok=True)
        self.t_status = {}
        self.t_state = {}
        self.last = None
        self.closest = {}
        self.max_tracks = {}
        self.max_dynamic = 0
        self.params = {}
        self.accel = dict(levels_seen=set(), peak=0.0)
        self.rtf = RtfMeter(window=10.0)
        self.rtf_log = []
        self.create_subscription(String, '/perception/obstacles', self.on_obst, 10)
        self.create_subscription(String, '/navigation/status', self.on_nav, 10)
        self.create_subscription(String, '/drive/status', self.on_drive, 10)
        self.create_timer(1.0, self.sample_rtf)
        self.wall0 = time.time()
        self.next_report = time.time() + 10.0
        self.get_logger().info(f'dynamic_check: writing {self.out}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_obst(self, m):
        d = json.loads(m.data)
        self.params = d.get('params', self.params)
        self.max_dynamic = max(self.max_dynamic, d.get('dynamic', 0))
        for k, v in d.get('counts', {}).items():
            self.max_tracks[k] = max(self.max_tracks.get(k, 0), v)
        for tr in d.get('tracks', []):
            c = tr['cls']
            if tr['distance'] < self.closest.get(c, (math.inf,))[0]:
                self.closest[c] = (tr['distance'], tr['dynamic'], round(d.get('stamp', 0.0), 1))

    def on_nav(self, m):
        d = json.loads(m.data)
        t = time.time()
        if self.last is not None:
            dt = t - self.last[0]
            self.t_status[self.last[1]] = self.t_status.get(self.last[1], 0.0) + dt
            self.t_state[self.last[2]] = self.t_state.get(self.last[2], 0.0) + dt
        self.last = (t, d.get('collision_status', 'n/a'), d.get('state', 'n/a'))
        if time.time() > self.next_report:
            self.next_report = time.time() + 10.0
            self.report()

    def on_drive(self, m):
        d = json.loads(m.data)
        self.accel['levels_seen'].add(d.get('accel_level_name', '?'))
        self.accel['peak'] = max(self.accel['peak'], abs(float(d.get('accel', 0.0) or 0.0)))
        self.accel['max_linear'] = d.get('max_linear')
        self.accel['current'] = d.get('accel_level_name')

    def sample_rtf(self):
        self.rtf.add(time.time(), self.now())
        r = self.rtf.rtf()
        if r is not None:
            self.rtf_log.append(r)

    def summary(self):
        p = self.params
        safety = dict(HUMAN=p.get('human_safety_distance'), VEHICLE=p.get('vehicle_safety_distance'))
        close = {c: dict(edge_distance=round(v[0], 2), moving=v[1], at_sim_time=v[2], safety=safety.get(c))
                 for c, v in self.closest.items()}
        rt = sorted(self.rtf_log)
        return dict(wall_seconds=round(time.time() - self.wall0, 0),
                    collision_status_seconds={k: round(v, 1) for k, v in self.t_status.items()},
                    navigator_state_seconds={k: round(v, 1) for k, v in sorted(self.t_state.items(), key=lambda kv: -kv[1])},
                    closest_approach=close, max_tracked=self.max_tracks, max_moving=self.max_dynamic,
                    acceleration=dict(self.accel, levels_seen=sorted(self.accel['levels_seen'])),
                    measured_rtf=dict(median=round(rt[len(rt) // 2], 2) if rt else None,
                                      min=round(rt[0], 2) if rt else None, max=round(rt[-1], 2) if rt else None),
                    params=p)

    def report(self):
        s = self.summary()
        self.out.write_text(json.dumps(s, indent=2))
        h = s['closest_approach'].get('HUMAN')
        self.get_logger().info(
            f"status {s['collision_status_seconds']} | closest person "
            f"{(str(h['edge_distance']) + ' m (safety ' + str(h['safety']) + ')') if h else '-'} | "
            f"tracked max {s['max_tracked']} moving {s['max_moving']} | accel {s['acceleration'].get('current')} "
            f"peak {s['acceleration']['peak']:.2f} m/s2 | sim speed {s['measured_rtf']['median']}x")


def main():
    rclpy.init()
    node = Check()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
