#!/usr/bin/python3
"""UGV SAR control station: a small web server inside a ROS 2 node.

Extended from vigil_rough_terrain/scripts/ugv_dashboard.py (same design: standard-library
HTTP server, OpenCV JPEG/PNG encoding, one HTML page). Open http://localhost:8080.

  /                dashboard page (dashboard/index.html)
  /camera.jpg      RGB overlay (/vision/overlay: 4K camera scaled to rgb_camera.overlay_width,
                   path / terrain / cliff / heading drawn by terrain_mapper)
  /camera.mjpg     same as MJPEG stream
  /camera4k.jpg    one full-resolution frame of /camera/hd/image (encoded on request only)
  /thermal.jpg     thermal panel (/vision/thermal_overlay: boxes, HUMAN DETECTED banner)
  /map.png         terrain class map (world frame, north up)
  /state.json      robot / terrain / navigation / SAR / humans / events
  POST /goal       {"x":..,"y":..}          -> /navigation/goal   (SET GOAL B)
  POST /nav        {"cmd":"START"|"PAUSE"}  -> /navigation/command
  POST /sar        {"cmd":"START"|"STOP"}   -> /sar/command       (SAR switch)
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
        cands.append(FsPath(get_package_share_directory('vigil_sar')) / 'dashboard/index.html')
    except Exception:
        pass
    for c in cands:
        if c.exists():
            return c
    raise FileNotFoundError('dashboard/index.html not found')


def load_buildings(cfg):
    try:
        from sar_paths import find_military_world
        mw = find_military_world(cfg['world'].get('military_world_dir', ''))
        meta = json.load(open(mw / cfg['world'].get('metadata_file', 'sar_metadata.json')))
        return [dict(id=b['id'], x=b['center'][0], y=b['center'][1], w=b['w'], d=b['d'], label=b.get('label', ''))
                for b in meta.get('buildings', [])]
    except Exception:
        return []


def placeholder(text, w=640, h=480):
    img = np.full((h, w, 3), 28, np.uint8)
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.putText(img, text, ((w - tw) // 2, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1, cv2.LINE_AA)
    return cv2.imencode('.jpg', img)[1].tobytes()


class Dashboard(Node):
    def __init__(self):
        super().__init__('ugv_dashboard', automatically_declare_parameters_from_overrides=True)
        cfg = nested_params(self)
        self.cfg = cfg
        self.p = Params.from_nested(cfg)
        self.port = int(cfg['dashboard'].get('port', 8080))
        self.quality = int(cfg['dashboard'].get('jpeg_quality', 80))
        self.map_period = float(cfg['dashboard'].get('map_png_period', 1.0))
        self.lock = threading.Lock()
        self.jpeg = placeholder('waiting for /vision/overlay ...')
        self.thermal_jpeg = placeholder('waiting for /vision/thermal_overlay ...', 640, 480)
        self.frame_id = 0
        self.thermal_id = 0
        self.hd_msg = None
        self.map_png = None
        self.map_meta = None
        self.pose = None
        self.speed = 0.0
        self.path = self.prev_path = None
        self.nav = {}
        self.terrain = {}
        self.cliff = {}
        self.sar = {}
        self.humans, self.tentative = [], []
        self.thermal = {}
        self.events = []            # [{wall, t, text, source}]
        self.nav_events_seen = set()
        self.trail = []
        self.start = None
        self.buildings = load_buildings(cfg)
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/vision/overlay', self.on_overlay, 2)
        self.create_subscription(Image, '/vision/thermal_overlay', self.on_thermal_img, 2)
        self.create_subscription(Image, '/camera/image', self.on_raw, s)
        self.create_subscription(Image, '/camera/hd/image', lambda m: self._set('hd_msg', m), s)
        self.create_subscription(OccupancyGrid, '/vision/terrain_classes', self.on_map, 2)
        self.create_subscription(Path, '/navigation/path', lambda m: self._set('path', path_from_msg(m)), 2)
        self.create_subscription(Path, '/navigation/previous_path', lambda m: self._set('prev_path', path_from_msg(m)), 2)
        self.create_subscription(String, '/navigation/status', self.on_nav, 10)
        self.create_subscription(String, '/vision/terrain', lambda m: self._set('terrain', json.loads(m.data)), 10)
        self.create_subscription(String, '/vision/cliff', lambda m: self._set('cliff', json.loads(m.data)), 10)
        self.create_subscription(String, '/vision/thermal_human', self.on_thermal, 10)
        self.create_subscription(String, '/sar/state', lambda m: self._set('sar', json.loads(m.data)), 10)
        self.create_subscription(String, '/sar/humans', self.on_humans, 10)
        self.create_subscription(String, '/sar/events', self.on_event, 50)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, s)
        self.goal_pub = self.create_publisher(PoseStamped, '/navigation/goal', 10)
        self.nav_cmd_pub = self.create_publisher(String, '/navigation/command', 10)
        self.sar_cmd_pub = self.create_publisher(String, '/sar/command', 10)
        self.last_overlay = 0.0
        self.last_map_encode = 0.0
        self.rtf, self.rtf_mark = None, None
        self.page = find_page()
        self.server = ThreadingHTTPServer(('0.0.0.0', self.port), self._handler())
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'UGV SAR dashboard: open http://localhost:{self.port}')

    # ------------------------------------------------------------- ROS side
    def _set(self, k, v):
        with self.lock:
            setattr(self, k, v)

    def _add_event(self, t, text, source):
        with self.lock:
            self.events.append(dict(wall=time.strftime('%H:%M:%S'), t=t, text=text, source=source))
            self.events = self.events[-300:]

    def on_event(self, m):
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        self._add_event(e.get('t'), e.get('text', ''), e.get('source', ''))

    def on_nav(self, m):
        d = json.loads(m.data)
        # navigation events (REPLANNING, CLIFF AHEAD, GOAL_REACHED ...) into the same log, once each
        for e in reversed(d.get('events', [])):
            key = (e.get('t'), e.get('msg'))
            if key not in self.nav_events_seen:
                self.nav_events_seen.add(key)
                self._add_event(e.get('t'), f"{e.get('state')}: {e.get('msg')}", 'nav')
        with self.lock:
            self.nav = d

    def on_humans(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.humans, self.tentative = d.get('humans', []), d.get('tentative', [])

    def on_thermal(self, m):
        d = json.loads(m.data)
        acc = [c for c in d.get('candidates', []) if c.get('accepted')]
        with self.lock:
            self.thermal = dict(human_detected=d.get('human_detected', False), count=d.get('count', 0),
                                background_k=d.get('background_k'), stamp=d.get('stamp'), wall=time.time(),
                                detections=[dict(x=c['x'], y=c['y'], distance=c['distance'], confidence=c['confidence'],
                                                 temp=c['mean_temp']) for c in acc])

    def on_overlay(self, m):
        img = image_to_numpy(m)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.lock:
                self.jpeg = buf.tobytes()
                self.frame_id += 1
            self.last_overlay = time.time()

    def on_thermal_img(self, m):
        img = image_to_numpy(m)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.lock:
                self.thermal_jpeg = buf.tobytes()
                self.thermal_id += 1

    def on_raw(self, m):
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
        if now - self.last_map_encode < self.map_period:
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

    def real_time_factor(self):
        """sim seconds per wall second, over the last few seconds. military_world with 1 ms
        physics, 20 walking people and five cameras runs far below real time; everything on the
        dashboard then happens at that fraction of the speed, so it is worth showing."""
        sim, wall = self.get_clock().now().nanoseconds * 1e-9, time.time()
        if self.rtf_mark is None or sim < self.rtf_mark[0]:
            self.rtf_mark = (sim, wall)
            return None
        d_sim, d_wall = sim - self.rtf_mark[0], wall - self.rtf_mark[1]
        if d_wall < 3.0:
            return self.rtf
        self.rtf = round(d_sim / d_wall, 3) if d_wall > 0 else None
        self.rtf_mark = (sim, wall)
        return self.rtf

    def on_odom(self, m):
        pose = odom_to_pose(m)
        t = m.twist.twist.linear
        with self.lock:
            self.pose = pose
            self.speed = math.hypot(t.x, t.y)
            if self.start is None:
                self.start = (pose.x, pose.y)
            if not self.trail or math.hypot(pose.x - self.trail[-1][0], pose.y - self.trail[-1][1]) > 0.2:
                self.trail.append((round(pose.x, 2), round(pose.y, 2)))
                self.trail = self.trail[-6000:]

    def state(self):
        with self.lock:
            pose, nav, ter = self.pose, dict(self.nav), dict(self.terrain)
            sar = dict(self.sar)
            tasks = sar.pop('tasks', [])
            cfg = self.cfg
            rtf = self.real_time_factor()
            d = dict(rtf=rtf, nav=nav, terrain=ter, cliff=dict(self.cliff), map=self.map_meta, frame=self.frame_id,
                     thermal_frame=self.thermal_id,
                     path=None if self.path is None else np.round(self.path[::3], 2).tolist(),
                     previous_path=None if self.prev_path is None else np.round(self.prev_path[::3], 2).tolist(),
                     trail=self.trail[-3000:], start=nav.get('start') or self.start,
                     # B is the OPERATOR's B at all times - the SAR manager remembers it apart
                     # from its own search goals, so the marker never jumps to a search point
                     # or to the spot where SAR was stopped.
                     goal=sar.get('b_goal') or nav.get('goal'),
                     nav_goal=nav.get('goal'),
                     sar=sar, tasks=tasks, humans=list(self.humans), tentative=list(self.tentative),
                     thermal=dict(self.thermal, age=round(time.time() - self.thermal.get('wall', 0), 1))
                     if self.thermal else {},
                     events=self.events[-150:], buildings=self.buildings,
                     class_names=CLASS_NAMES,
                     class_colors=['#%02x%02x%02x' % tuple(int(c) for c in LUT[k][::-1])
                                   for k in range(len(CLASS_NAMES))],
                     params=dict(length=self.p.robot_length, width=self.p.robot_width,
                                 footprint_center_x=self.p.footprint_center_x,
                                 max_safe_drop=self.p.max_safe_drop, max_safe_slope=self.p.max_safe_slope_deg,
                                 max_climb_slope=self.p.max_climb_slope_deg,
                                 detection=self.p.cliff_detection_distance,
                                 inflation=round(self.p.inscribed_radius, 2),
                                 rgb=f"{cfg['rgb_camera'].get('width')}x{cfg['rgb_camera'].get('height')}",
                                 thermal=f"{cfg['thermal_camera'].get('resolution_width')}x"
                                         f"{cfg['thermal_camera'].get('resolution_height')}",
                                 thermal_band=[cfg['thermal_camera'].get('temperature_threshold'),
                                               cfg['thermal_camera'].get('human_max_temperature')],
                                 goal_tolerance=cfg['navigation'].get('goal_tolerance', 0.3)))
            if pose is not None:
                roll, pitch = pose.roll_pitch_deg
                d['robot'] = dict(x=round(pose.x, 2), y=round(pose.y, 2), z=round(pose.z, 2),
                                  heading=round(math.degrees(pose.yaw), 1), yaw=pose.yaw,
                                  roll=round(roll, 1), pitch=round(pitch, 1), speed=round(self.speed, 3))
                # distance from the REAL robot pose (Gazebo ground truth) to the operator's B
                if d['goal']:
                    d['dist_to_b'] = round(math.hypot(d['goal'][0] - pose.x, d['goal'][1] - pose.y), 2)
        return d

    def snapshot_4k(self):
        with self.lock:
            m = self.hd_msg
        if m is None:
            return placeholder('no /camera/hd/image yet')
        img = image_to_numpy(m)
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()

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
                elif self.path.startswith('/camera4k.jpg'):
                    self._send(200, 'image/jpeg', node.snapshot_4k())
                elif self.path.startswith('/camera.jpg'):
                    with node.lock:
                        jpg = node.jpeg
                    self._send(200, 'image/jpeg', jpg)
                elif self.path.startswith('/thermal.jpg'):
                    with node.lock:
                        jpg = node.thermal_jpeg
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

            def _body(self):
                n = int(self.headers.get('Content-Length', 0))
                return json.loads(self.rfile.read(n) or b'{}')

            def do_POST(self):
                try:
                    g = self._body()
                    if self.path.startswith('/goal'):
                        msg = PoseStamped()
                        msg.header.frame_id = 'world'
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.pose.position.x, msg.pose.position.y = float(g['x']), float(g['y'])
                        msg.pose.orientation.w = 1.0
                        node.goal_pub.publish(msg)
                        node._add_event(None, f"SET GOAL B = ({float(g['x']):.1f}, {float(g['y']):.1f})", 'operator')
                    elif self.path.startswith('/nav'):
                        cmd = str(g.get('cmd', '')).upper()
                        node.nav_cmd_pub.publish(String(data=cmd))
                        node._add_event(None, f'NAVIGATION {cmd}', 'operator')
                    elif self.path.startswith('/sar'):
                        cmd = str(g.get('cmd', '')).upper()
                        node.sar_cmd_pub.publish(String(data=cmd))
                        node._add_event(None, f'SAR SWITCH -> {cmd}', 'operator')
                    else:
                        self._send(404, 'text/plain', b'not found')
                        return
                    self._send(200, 'application/json', b'{"ok":true}')
                except (ValueError, KeyError, TypeError):
                    self._send(400, 'application/json', b'{"ok":false}')
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
