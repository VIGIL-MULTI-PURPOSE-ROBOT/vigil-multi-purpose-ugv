#!/usr/bin/python3
"""Drive via cmd_vel; record real physics, contact, TF and sensor telemetry.
No pose, reset, animation or suspension command calls are used.
"""
import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data as qos
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState, Imu, Image, PointCloud2
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation

class Traverse(Node):
    def __init__(self, args):
        super().__init__('suspension_traversal_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.counts = Counter()
        self.joints, self.contacts = {}, {}
        self.contact_stamps = {}
        self.depth, self.pose, self.start = 0., None, None
        self.samples, self.frames = [], set()
        self.waypoint, self.reason = 0, 'duration'
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos)
        self.create_subscription(Odometry, '/sim/ground_truth', self.pose_cb, qos)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, qos)
        for typ, topic in [(Imu, '/imu/data'), (Image, '/camera/image'), (Image, '/camera/depth_image'), (PointCloud2, '/lidar/points')]:
            self.create_subscription(typ, topic, lambda m, t=topic: self.counts.update([t]), qos)
        for side in 'LR':
            for i in range(1, 5):
                name = f'{side}{i}'
                self.create_subscription(Contacts, f'/suspension/contacts/{name}', lambda m, k=name: self.contact_cb(k, m), qos)
    def joint_cb(self, msg):
        self.counts['joints'] += 1
        self.joints = dict(zip(msg.name, msg.position))
    def pose_cb(self, msg):
        self.counts['odom'] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    def tf_cb(self, msg):
        self.counts['tf'] += 1
        self.frames.update(t.child_frame_id for t in msg.transforms)
    def contact_cb(self, name, msg):
        self.counts['contacts/' + name] += 1
        self.contacts[name] = bool(msg.contacts)
        self.contact_stamps[name] = self.get_clock().now().nanoseconds * 1e-9
        for c in msg.contacts:
            if c.depths:
                self.depth = max(self.depth, max(c.depths))
    def step(self):
        if self.pose is None or not self.joints:
            return True
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = now
        elapsed = now - self.start
        rot = Rotation.from_quat(self.pose[3:])
        roll, pitch, yaw = rot.as_euler('xyz')
        corners = np.array([[x, y, .5872-.1264-.3523/2] for x in [-.0753-1.0737/2, -.0753+1.0737/2] for y in [-.25, .25]])
        bottom = (rot.apply(corners) + self.pose[:3])[:, 2].min()
        self.samples.append(dict(t=elapsed, pose=self.pose, rpy=[roll, pitch, yaw], joints=self.joints.copy(), contacts={k: v and now-self.contact_stamps[k] < .06 for k,v in self.contacts.items()}, bottom_world_z=float(bottom)))
        cmd = Twist()
        active = elapsed < self.args.duration
        if self.args.drive and elapsed > 3 and active:
            route = [[-4.896, -3.024], [-4.32, -2.16], [-2.88, -2.16], [-1.44, -2.16], [0, -2.16], [0, -0.72], [-1.44, -0.72], [-2.88, -0.72], [-4.32, -0.72]]  # x0.72: 18 x 18 m terrain
            dx, dy = route[self.waypoint][0]-self.pose[0], route[self.waypoint][1]-self.pose[1]
            if math.hypot(dx, dy) < .3:
                self.waypoint += 1
                if self.waypoint == len(route):
                    self.reason, active = 'route_complete', False
            heading = math.atan2(math.sin(math.atan2(dy, dx)-yaw), math.cos(math.atan2(dy, dx)-yaw))
            cmd.linear.x = self.args.speed if abs(heading) < .65 else 0.
            cmd.angular.z = max(-.45, min(.45, heading))
            if max(abs(roll), abs(pitch)) > math.radians(38):
                self.reason, active = 'tilt_stop', False
        self.pub.publish(cmd if active else Twist())
        return active
    def finish(self):
        self.pub.publish(Twist())
        summary = dict(stop_reason=self.reason, waypoints_reached=self.waypoint, counts=dict(self.counts), tf_frames=sorted(self.frames), max_contact_depth_m=self.depth, samples=len(self.samples))
        if self.samples:
            a = self.samples
            summary.update(start_pose=a[0]['pose'], end_pose=a[-1]['pose'], max_abs_roll_pitch_deg=np.rad2deg(np.abs([s['rpy'][:2] for s in a]).max(0)).tolist(), final_bottom_world_z=a[-1]['bottom_world_z'], joint_ranges={k: [min(s['joints'].get(k, 0) for s in a), max(s['joints'].get(k, 0) for s in a)] for k in self.joints if 'suspension' in k or 'rocker' in k}, contact_fraction={k: sum(s['contacts'].get(k, False) for s in a)/len(a) for k in self.contacts})
        Path(self.args.output).write_text(json.dumps({'summary': summary, 'samples': self.samples}, indent=2))
        print(json.dumps(summary, indent=2), flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--speed', type=float, default=.25)
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--output', default='/tmp/rover-traversal.json')
    args = parser.parse_args()
    rclpy.init()
    node = Traverse(args)
    deadline, next_step = time.monotonic() + max(90, args.duration * 8), 0.
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now >= next_step:
                next_step = now + .05
                if not node.step():
                    break
        else:
            node.reason = 'wall_timeout'
    except KeyboardInterrupt:
        node.reason = 'interrupted'
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
