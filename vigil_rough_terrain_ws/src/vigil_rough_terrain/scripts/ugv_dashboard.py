#!/usr/bin/python3
"""UGV navigation dashboard: a small web server inside a ROS 2 node.

Open http://localhost:8080 (port from vision_nav.yaml). No extra Python packages:
the HTTP server is the standard library, images are encoded with OpenCV.

  /              dashboard page (dashboard/index.html)
  /camera.jpg    latest /vision/overlay frame (the page polls this ~10 Hz)
  /camera.mjpg   same as an MJPEG stream (for VLC / other viewers)
  /map.png       terrain class map (world frame, 1 px = 1 cell, north up)
  /state.json    robot / terrain / navigation state + paths
  POST /goal     {"x":..,"y":..} -> publishes /navigation/goal (click on the map)
"""
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry, OccupancyGrid, Path  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from terrain_core import CLASS_BGR, CLASS_NAMES, Params  # noqa: E402
from ros_common import nested_params, image_to_numpy, odom_to_pose, grid_to_numpy, path_from_msg  # noqa: E402

LUT = np.zeros((256, 3), np.uint8)
for _k, _v in CLASS_BGR.items():
    LUT[_k] = _v
LUT[0] = (34, 30, 28)  # unknown = dashboard background tone


def find_page():
    here = FsPath(os.path.realpath(__file__)).parent
    cands = [here.parent / 'dashboard/index.html']
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(FsPath(get_package_share_directory('vigil_rough_terrain')) / 'dashboard/index.html')
    except Exception:
        pass
    for c in cands:
        if c.exists():
            return c
    raise FileNotFoundError('dashboard/index.html not found')


class Dashboard(Node):
    def __init__(self):
        super().__init__('ugv_dashboard', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.p = Params.from_nested(cfg)
        self.port = int(cfg['dashboard'].get('port', 8080))
        self.quality = int(cfg['dashboard'].get('jpeg_quality', 80))
        self.lock = threading.Lock()
        self.jpeg = self._placeholder()
        self.frame_id = 0
        self.map_png = None
        self.map_meta = None
        self.pose = None
        self.speed = 0.0
        self.path = self.prev_path = None
        self.nav = {}
        self.terrain = {}
        self.trail = []
        self.start = None
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/vision/overlay', self.on_overlay, 2)
        self.create_subscription(Image, '/camera/image', self.on_raw, s)
        self.create_subscription(OccupancyGrid, '/vision/terrain_classes', self.on_map, 2)
        self.create_subscription(Path, '/navigation/path', lambda m: self._set('path', path_from_msg(m)), 2)
        self.create_subscription(Path, '/navigation/previous_path', lambda m: self._set('prev_path', path_from_msg(m)), 2)
        self.create_subscription(String, '/navigation/status', lambda m: self._set('nav', json.loads(m.data)), 10)
        self.create_subscription(String, '/vision/terrain', lambda m: self._set('terrain', json.loads(m.data)), 10)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, s)
        self.goal_pub = self.create_publisher(PoseStamped, '/navigation/goal', 10)
        self.last_overlay = 0.0
        self.last_map_encode = 0.0
        self.page = find_page()
        self.server = ThreadingHTTPServer(('0.0.0.0', self.port), self._handler())
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'UGV dashboard: open http://localhost:{self.port}')

    # ------------------------------------------------------------- ROS side
    def _set(self, k, v):
        with self.lock:
            setattr(self, k, v)

    def _placeholder(self):
        img = np.full((480, 640, 3), 28, np.uint8)
        cv2.putText(img, 'waiting for /vision/overlay ...', (150, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (150, 150, 150), 1, cv2.LINE_AA)
        return cv2.imencode('.jpg', img)[1].tobytes()

    def on_overlay(self, m):
        img = image_to_numpy(m)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.lock:
                self.jpeg = buf.tobytes()
                self.frame_id += 1
            self.last_overlay = time.time()

    def on_raw(self, m):
        # Until the vision node publishes, show the raw camera so the page is never blank.
        if time.time() - self.last_overlay < 2.0:
            return
        img = cv2.resize(image_to_numpy(m), (640, 480))
        cv2.putText(img, 'RAW CAMERA (vision node not running)', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 200, 255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.lock:
                self.jpeg = buf.tobytes()
                self.frame_id += 1

    def on_map(self, m):
        now = time.time()
        if now - self.last_map_encode < 0.5:
            return
        self.last_map_encode = now
        cls = np.clip(grid_to_numpy(m), 0, 255).astype(np.uint8)
        img = LUT[cls][::-1]  # row 0 = north (max y) for the browser
        ok, buf = cv2.imencode('.png', img)
        if ok:
            meta = dict(resolution=m.info.resolution, width=m.info.width, height=m.info.height,
                        origin_x=m.info.origin.position.x, origin_y=m.info.origin.position.y)
            with self.lock:
                self.map_png, self.map_meta = buf.tobytes(), meta

    def on_odom(self, m):
        pose = odom_to_pose(m)
        t = m.twist.twist.linear
        with self.lock:
            self.pose = pose
            self.speed = math.hypot(t.x, t.y)
            if self.start is None:
                self.start = (pose.x, pose.y)
            if not self.trail or math.hypot(pose.x - self.trail[-1][0], pose.y - self.trail[-1][1]) > 0.1:
                self.trail.append((round(pose.x, 2), round(pose.y, 2)))
                self.trail = self.trail[-3000:]

    def state(self):
        with self.lock:
            pose, nav, ter = self.pose, dict(self.nav), dict(self.terrain)
            d = dict(nav=nav, terrain=ter, map=self.map_meta, frame=self.frame_id,
                     path=None if self.path is None else np.round(self.path[::2], 2).tolist(),
                     previous_path=None if self.prev_path is None else np.round(self.prev_path[::2], 2).tolist(),
                     trail=self.trail[-1500:], start=nav.get('start') or self.start, goal=nav.get('goal'),
                     class_names=CLASS_NAMES,
                     class_colors=['#%02x%02x%02x' % tuple(int(c) for c in LUT[k][::-1])
                                   for k in range(len(CLASS_NAMES))],
                     params=dict(length=self.p.robot_length, width=self.p.robot_width,
                                 footprint_center_x=self.p.footprint_center_x,
                                 max_safe_drop=self.p.max_safe_drop, max_safe_slope=self.p.max_safe_slope_deg,
                                 max_climb_slope=self.p.max_climb_slope_deg,
                                 detection=self.p.cliff_detection_distance,
                                 inflation=round(self.p.inscribed_radius, 2)))
            if pose is not None:
                roll, pitch = pose.roll_pitch_deg
                d['robot'] = dict(x=round(pose.x, 2), y=round(pose.y, 2), z=round(pose.z, 2),
                                  heading=round(math.degrees(pose.yaw), 1), yaw=pose.yaw,
                                  roll=round(roll, 1), pitch=round(pitch, 1), speed=round(self.speed, 3))
        return d

    # ------------------------------------------------------------ HTTP side
    def _handler(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self._send(200, 'text/html; charset=utf-8', node.page.read_bytes())
                elif self.path.startswith('/state.json'):
                    self._send(200, 'application/json', json.dumps(node.state()).encode())
                elif self.path.startswith('/map.png'):
                    with node.lock:
                        png = node.map_png
                    if png is None:
                        self._send(404, 'text/plain', b'no map yet')
                    else:
                        self._send(200, 'image/png', png)
                elif self.path.startswith('/camera.jpg'):
                    with node.lock:
                        jpg = node.jpeg
                    self._send(200, 'image/jpeg', jpg)
                elif self.path.startswith('/camera.mjpg'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    last = -1
                    try:
                        while True:
                            with node.lock:
                                fid, jpg = node.frame_id, node.jpeg
                            if fid != last:
                                last = fid
                                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                                 + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                            time.sleep(0.05)
                    except (BrokenPipeError, ConnectionResetError):
                        return
                else:
                    self._send(404, 'text/plain', b'not found')

            def do_POST(self):
                if self.path.startswith('/goal'):
                    n = int(self.headers.get('Content-Length', 0))
                    try:
                        g = json.loads(self.rfile.read(n))
                        msg = PoseStamped()
                        msg.header.frame_id = 'world'
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.pose.position.x, msg.pose.position.y = float(g['x']), float(g['y'])
                        msg.pose.orientation.w = 1.0
                        node.goal_pub.publish(msg)
                        self._send(200, 'application/json', b'{"ok":true}')
                    except (ValueError, KeyError):
                        self._send(400, 'application/json', b'{"ok":false}')
                else:
                    self._send(404, 'text/plain', b'not found')
        return H


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
