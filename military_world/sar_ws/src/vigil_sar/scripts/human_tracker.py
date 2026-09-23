#!/usr/bin/python3
"""Human tracking node: per-frame thermal candidates -> confirmed humans H1, H2, ...

Subscribes: /vision/thermal_human  std_msgs/String JSON (thermal_human_detector)
Publishes:  /sar/humans            std_msgs/String JSON {humans: [...], tentative: [...]}
            /sar/human_markers     visualization_msgs/MarkerArray (RViz, frame 'world')
            /sar/events            std_msgs/String JSON {t, text, source}  one per NEW human only

A candidate is declared HUMAN DETECTED after human_detection.confirmation_frames
consecutive associated frames. Once confirmed, its id never changes and re-sightings
(also from later search points, within merge_distance) update the same human.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from ros_common import nested_params  # noqa: E402
from thermal_core import HumanTracker  # noqa: E402


class HumanTrackerNode(Node):
    def __init__(self):
        super().__init__('human_tracker', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        hd, sar = cfg['human_detection'], cfg['sar']
        frames = int(sar.get('human_confirmation_frames', hd.get('confirmation_frames', 4)))
        self.tr = HumanTracker(confirm_frames=frames, miss_frames=int(hd.get('miss_frames', 3)),
                               gate=float(hd.get('association_gate', 1.8)),
                               merge_distance=max(float(hd.get("merge_distance", 1.5)),
                                                  float(sar.get('revisit_distance', 1.5))),
                               alpha=float(hd.get('position_smoothing', 0.3)))
        self.create_subscription(String, '/vision/thermal_human', self.on_det, 10)
        self.pub = self.create_publisher(String, '/sar/humans', 10)
        self.pub_m = self.create_publisher(MarkerArray, '/sar/human_markers', 2)
        self.pub_ev = self.create_publisher(String, '/sar/events', 20)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'human_tracker: HUMAN DETECTED after {frames} consecutive frames')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_det(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        dets = [c for c in d.get('candidates', []) if c.get('accepted') and c.get('x') is not None]
        self.tr.update(dets, self.now())
        for t, text in self.tr.events:
            self.get_logger().info(text)
            self.pub_ev.publish(String(data=json.dumps(dict(t=round(t, 1), text=text, source='thermal'))))
        self.tr.events.clear()
        self.publish()

    def publish(self):
        humans = [h.as_dict() for h in self.tr.humans]
        tent = [dict(track=t.tid, x=round(t.x, 2), y=round(t.y, 2), hits=t.hits, confidence=round(t.confidence, 3))
                for t in self.tr.tentative if t.hits > 0 or t.sightings >= 3]
        self.pub.publish(String(data=json.dumps(dict(t=round(self.now(), 1), humans=humans, tentative=tent,
                                                     count=len(humans)))))
        ma = MarkerArray()
        for k, h in enumerate(self.tr.humans):
            for j, kind in enumerate(('sphere', 'text')):
                mk = Marker()
                mk.header.frame_id = 'world'
                mk.header.stamp = self.get_clock().now().to_msg()
                mk.ns = 'sar_humans'
                mk.id = 2 * k + j
                mk.pose.position.x, mk.pose.position.y = h.x, h.y
                mk.pose.position.z = h.z + (1.2 if kind == 'text' else 0.0)
                mk.pose.orientation.w = 1.0
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.15, 0.15, 1.0
                if kind == 'sphere':
                    mk.type = Marker.SPHERE
                    mk.scale.x = mk.scale.y = mk.scale.z = 0.8
                else:
                    mk.type = Marker.TEXT_VIEW_FACING
                    mk.scale.z = 0.8
                    mk.text = f'{h.human_id} ({h.x:.1f}, {h.y:.1f})'
                ma.markers.append(mk)
        self.pub_m.publish(ma)


def main():
    rclpy.init()
    node = HumanTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
