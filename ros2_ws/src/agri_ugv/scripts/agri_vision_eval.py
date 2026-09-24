#!/usr/bin/python3
"""Gazebo check of the agriculture perception against the world's real contents (observe only).

Run it in a second terminal while the simulation runs (after pressing START ROBOT):

    cd ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=91
    ros2 run agri_ugv agri_vision_eval.py            # Ctrl+C to finish

Ground truth comes from the world files (never used by the perception itself):
  * people  : worker_s_* in vigil_cotton_farm/model.sdf and worker_d_* in worlds/field.world
  * crop    : the row lines of the field (config/agriculture.yaml field:, = model.sdf row_xx_seg)
It reports, every 10 s and at the end (reports/agri_vision/eval.json):
  CROP->WEED violations (a WEED inside a crop row band) - must be 0
  people found / missed / false people, confirmed weeds with their distance from the row line,
  the row sequence driven (from the existing driver's own status) - must be 1, 2, 3, ... in order,
  and overlay snapshots in reports/agri_vision/frames/ for inspection.
"""
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from agri_vision import agri_config, default_config_path, image_to_numpy, load_yaml  # noqa: E402
from agri_vision_core import Field, load_config  # noqa: E402


def _pose(e):
    t = e.findtext('pose')
    return [float(v) for v in t.split()] if t else [0.0] * 6


def _compose(a, b):
    c, s = math.cos(a[5]), math.sin(a[5])
    return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2], 0, 0, a[5] + b[5]]


def known_people(share):
    """World positions of every person model in the supplied world files."""
    people = []
    model = share / 'models/vigil_cotton_farm/model.sdf'
    try:
        m = ET.parse(model).getroot().find('model')
        base = _pose(m)
        for link in m.findall('link'):
            lp = _compose(base, _pose(link))
            for v in link.findall('visual'):
                uri = v.findtext('.//uri') or ''
                if '/human/' in uri:
                    p = _compose(lp, _pose(v))
                    people.append((v.get('name'), p[0], p[1]))
    except (OSError, ET.ParseError):
        pass
    try:
        w = ET.parse(share / 'worlds/field.world').getroot().find('world')
        for m in w.findall('model'):
            if '/human/' in ''.join(u.text or '' for u in m.iter('uri')):
                p = _pose(m)
                people.append((m.get('name'), p[0], p[1]))
    except (OSError, ET.ParseError):
        pass
    return people


class Eval(Node):
    def __init__(self):
        super().__init__('agri_vision_eval')
        from ament_index_python.packages import get_package_share_directory
        self.share = Path(get_package_share_directory('agri_ugv'))
        self.cfg = agri_config(self.declare_parameter('config', default_config_path()).value)
        self.out = Path(self.declare_parameter('output', os.path.join(os.getcwd(), 'reports/agri_vision')).value)
        (self.out / 'frames').mkdir(parents=True, exist_ok=True)
        self.field = Field(self.cfg['field'])
        self.people = known_people(self.share)
        self.tol = self.cfg['crop']['row_position_tolerance']
        self.frame_violations = []
        self.frames = 0
        self.tracks = {}
        self.rows_seen = [1]
        self.driver_msgs = []
        self.last_snap = 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'agri_vision/detections', self.on_dets, 10)
        self.create_subscription(String, 'agri_vision/tracks', lambda m: setattr(self, 'tracks', json.loads(m.data)), latched)
        self.create_subscription(String, 'exploration/status', self.on_driver, latched)
        self.create_subscription(Image, 'agri_vision/overlay', self.on_overlay, 2)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{len(self.people)} people in the world files; writing {self.out}')

    def on_dets(self, m):
        d = json.loads(m.data)
        self.frames += 1
        for det in d['detections']:
            if det['cls'] == 'WEED':
                _, off = self.field.offset(det['x'], det['y'])
                if abs(off) <= self.tol and self.field.in_field(det['x'], det['y']):
                    self.frame_violations.append(dict(t=d['t'], x=det['x'], y=det['y'], offset=float(off)))

    def on_driver(self, m):
        data = json.loads(m.data)
        self.driver_msgs.append(data.get('message', ''))
        if 'U-turn complete' in data.get('message', '') and data.get('row'):
            self.rows_seen.append(int(data['row']))

    def on_overlay(self, m):
        if time.time() - self.last_snap > 10.0:
            self.last_snap = time.time()
            cv2.imwrite(str(self.out / 'frames' / f'overlay_{int(self.last_snap)}.jpg'), image_to_numpy(m))

    def report(self):
        items = self.tracks.get('items', [])
        humans = [i for i in items if i['cls'] == 'HUMAN']
        weeds = [i for i in items if i['cls'] == 'WEED']
        matched, false_h = set(), []
        for h in humans:
            near = [p for p in self.people if math.hypot(p[1] - h['x'], p[2] - h['y']) < 1.2]
            if near:
                matched.add(near[0][0])
            else:
                false_h.append(h['id'])
        weed_rows = []
        for w in weeds:
            k, off = self.field.offset(w['x'], w['y'])
            weed_rows.append(dict(id=w['id'], x=w['x'], y=w['y'], row_offset=round(float(off), 3),
                                 in_row_band=bool(abs(off) <= self.tol and self.field.in_field(w['x'], w['y']))))
        seq_ok = self.rows_seen == list(range(1, len(self.rows_seen) + 1))
        rep = dict(
            frames=self.frames, counts=self.tracks.get('counts', {}),
            crop_to_weed_violations_frames=len(self.frame_violations),
            crop_to_weed_violations_confirmed=sum(w['in_row_band'] for w in weed_rows),
            people_in_world=len(self.people), people_found=sorted(matched), false_people=false_h,
            weeds=weed_rows, row_sequence=self.rows_seen, row_sequence_in_order=seq_ok,
            driver_last=self.driver_msgs[-1] if self.driver_msgs else None,
            vision_ms=self.tracks.get('ms'), crop_model=self.tracks.get('crop_model'),
            first_violations=self.frame_violations[:10])
        (self.out / 'eval.json').write_text(json.dumps(rep, indent=2))
        ok = rep['crop_to_weed_violations_confirmed'] == 0 and rep['crop_to_weed_violations_frames'] == 0 and seq_ok
        self.get_logger().info(
            f"frames {self.frames} | counts {rep['counts']} | CROP->WEED {rep['crop_to_weed_violations_frames']} "
            f"| people found {len(matched)}/{len(self.people)} false {len(false_h)} | rows {self.rows_seen} "
            f"| {'OK' if ok else 'CHECK eval.json'}")


def main():
    rclpy.init()
    node = Eval()
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
