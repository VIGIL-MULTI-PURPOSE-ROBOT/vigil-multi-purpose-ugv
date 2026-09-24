#!/usr/bin/python3
"""Agriculture dashboard: a small web server inside a ROS 2 node (same design as the rock-terrain
dashboard). Open http://localhost:8080.

  /              dashboard/index.html
  /camera.jpg    latest /agri_vision/overlay frame (raw /camera/image until the vision node runs)
  /camera.mjpg   the same as an MJPEG stream
  /state.json    robot, row mission, detections, map data, event log
  POST /command  {"cmd": "START"} or {"cmd": "STOP"} -> /agri_dashboard/command (row_start_gate.py)

Everything shown comes from the running system: robot pose from /sim/ground_truth, row state from
the existing crop_row_driver topics (/crop_row/path, /crop_row/mode, /exploration/status),
detections from agri_vision.py. The page never moves the robot marker by itself.
"""
import json
import math
import re
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from agri_vision import agri_config, default_config_path, image_to_numpy, load_yaml  # noqa: E402
from agri_vision_core import Field, load_config  # noqa: E402


def find_page():
    here = Path(os.path.realpath(__file__)).parent
    cands = [here.parent / 'dashboard/index.html']
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory('agri_ugv')) / 'dashboard/index.html')
    except Exception:
        pass
    for c in cands:
        if c.exists():
            return c
    raise FileNotFoundError('dashboard/index.html not found')


class RowTracker:
    """Row number / direction / state for the dashboard, from the existing driver's own topics."""

    def __init__(self, pitch, total):
        self.pitch, self.total = pitch, total
        self.y0 = None
        self.rows = []          # [(y, x_start, x_end)] from /crop_row/path, in driving order
        self.mode = None        # 'row' / 'headland'
        self.current = None
        self.completed = 0
        self.last_msg = ''
        self.hold = None        # latest safety hold message from the driver

    def on_path(self, poses):
        if not poses:
            return
        per = 22                                                   # crop_row_driver.publish_path
        self.rows = [(poses[i][1], poses[i][0], poses[i + per - 1][0]) for i in range(0, len(poses) - per + 1, per)]
        self.total = len(self.rows) or self.total
        self.y0 = poses[0][1]

    def row_from_pose(self, y):
        if self.y0 is None:
            return None
        return int(min(max(round((y - self.y0) / self.pitch), 0), max(self.total - 1, 0))) + 1

    def on_status(self, data):
        """Returns dashboard events for driver messages."""
        msg = data.get('message', '')
        ev = []
        if msg == self.last_msg:
            return ev
        self.last_msg = msg
        m = re.match(r'Row (\d+) complete', msg)
        if m:
            self.completed = max(self.completed, int(m.group(1)))
            ev.append(('ROW', f'ROW {m.group(1)} COMPLETE', msg.split(': ', 1)[-1]))
        elif 'U-turn started' in msg:
            ev.append(('ROW', f'TURNING TO ROW {data.get("next_row", "?")}',
                       'reverse U-turn (headland ahead blocked)' if 'Reverse' in msg else 'headland U-turn'))
        elif 'U-turn complete' in msg:
            ev.append(('ROW', f'ROW {data.get("row", "?")} STARTED', msg))
            self.hold = None
        elif 'straddle drive started' in msg:
            ev.append(('ROW', f'ROW {data.get("row", 1)} READY', msg))
        elif 'sweep complete' in msg:
            self.completed = self.total
            ev.append(('DONE', 'FIELD COMPLETE', msg))
        elif 'detour' in msg:
            ev.append(('OBSTACLE', 'AVOIDING OBSTACLE', msg))
        elif 'Back on row' in msg:
            ev.append(('OBSTACLE', 'RETURNED TO ROW', msg))
        elif 'yielding' in msg:
            ev.append(('OBSTACLE', 'DYNAMIC OBSTACLE DETECTED', msg))
        elif 'clear again' in msg:
            ev.append(('OBSTACLE', 'ROW CLEAR', msg))
        elif 'Headland' in msg and 'blocked' in msg:
            ev.append(('OBSTACLE', 'HEADLAND BLOCKED', msg))
        elif 'at the row end' in msg:
            ev.append(('OBSTACLE', 'ROW END BLOCKED', msg))
        elif 'no free side' in msg or 'Detour line blocked' in msg:
            ev.append(('OBSTACLE', 'WAITING FOR A CLEAR WAY', msg))
        elif 'safety' in msg.lower() or 'obstacle stop' in msg.lower():
            self.hold = msg
            ev.append(('SAFETY', 'SAFETY HOLD', msg))
        return ev


class Dashboard(Node):
    def __init__(self):
        super().__init__('agri_dashboard')
        path = self.declare_parameter('config', default_config_path()).value
        row_count = int(self.declare_parameter('row_count', 23).value)
        self.cfg = agri_config(path)
        self.field = Field(self.cfg['field'])
        d = self.cfg['dashboard']
        self.port, self.quality, self.max_events = int(d['port']), int(d['jpeg_quality']), int(d['max_events'])
        self.lock = threading.Lock()
        self.jpeg = self._placeholder()
        self.frame_id = 0
        self.last_overlay = 0.0
        self.pose = None
        self.speed = 0.0
        self.cmd_speed = 0.0
        self.trail = []
        self.robot_state = None
        self.tracks = {}
        self.events = []
        self.rows = RowTracker(self.field.pitch, row_count)
        s = qos_profile_sensor_data
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, 'agri_vision/overlay', self.on_overlay, 2)
        self.create_subscription(Image, 'camera/image', self.on_raw, s)
        self.create_subscription(Odometry, 'sim/ground_truth', self.on_odom, s)
        self.create_subscription(NavPath, 'crop_row/path', self.on_path, latched)
        self.create_subscription(String, 'crop_row/mode', self.on_mode, latched)
        self.create_subscription(String, 'exploration/status', self.on_driver, latched)
        self.create_subscription(String, 'agri_dashboard/robot_state', self.on_robot_state, latched)
        self.create_subscription(String, 'agri_vision/tracks', self.on_tracks, latched)
        self.create_subscription(String, 'agri_vision/events', self.on_vision_event, 50)
        self.create_subscription(Twist, 'drive/limited', self.on_cmd, 10)
        # whole-field mission, obstacles, moisture
        self.progress = {}
        self.obstacles = []
        self.moisture = None
        self.moist_status = {}
        self.moist_png = None
        self.moist_version = 0
        self.moist_grid = None
        self.moist_conf = None
        self.create_subscription(String, 'crop_row/progress', self.on_progress, latched)
        self.create_subscription(String, 'agriculture/obstacles', self.on_obstacles, 5)
        self.create_subscription(String, 'agriculture/obstacle_events', self.on_obstacle_event, 20)
        self.create_subscription(String, 'agriculture/moisture', self.on_moisture, 20)
        self.create_subscription(OccupancyGrid, 'agriculture/moisture_map', self.on_moist_map, latched)
        self.create_subscription(OccupancyGrid, 'agriculture/moisture_map/confidence', self.on_moist_conf, latched)
        self.create_subscription(String, 'agriculture/moisture_map/status', self.on_moist_status, latched)
        self.create_subscription(String, 'agriculture/moisture_events', self.on_moist_event, 50)
        self.cmd_pub = self.create_publisher(String, 'agri_dashboard/command', 10)
        self.page = find_page()
        self.server = ThreadingHTTPServer(('0.0.0.0', self.port), self._handler())
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Agriculture dashboard: open http://localhost:{self.port}')

    # ------------------------------------------------------------------ ROS side
    def event(self, kind, title, detail='', **kw):
        e = dict(clock=time.strftime('%H:%M:%S'), t=round(self.sim_t(), 1), kind=kind, title=title, detail=detail, **kw)
        with self.lock:
            self.events.insert(0, e)
            del self.events[self.max_events:]

    def sim_t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _placeholder(self):
        img = np.full((480, 640, 3), 28, np.uint8)
        cv2.putText(img, 'waiting for the camera ...', (190, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1, cv2.LINE_AA)
        return cv2.imencode('.jpg', img)[1].tobytes()

    def _store(self, img):
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.lock:
                self.jpeg = buf.tobytes()
                self.frame_id += 1

    def on_overlay(self, m):
        self._store(image_to_numpy(m))
        self.last_overlay = time.time()

    def on_raw(self, m):
        if time.time() - self.last_overlay < 2.0:
            return
        img = cv2.resize(image_to_numpy(m), (640, 480))
        cv2.putText(img, 'RAW CAMERA (vision node not running)', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 200, 255), 1, cv2.LINE_AA)
        self._store(img)

    def on_odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose = (p.x, p.y, yaw)
            self.speed = math.hypot(v.x, v.y)
            if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) > 0.15:
                self.trail.append((round(p.x, 2), round(p.y, 2)))
                self.trail = self.trail[-4000:]

    def on_cmd(self, m):
        self.cmd_speed = m.linear.x

    def on_path(self, m):
        with self.lock:
            self.rows.on_path([(p.pose.position.x, p.pose.position.y) for p in m.poses])

    def on_mode(self, m):
        with self.lock:
            old, self.rows.mode = self.rows.mode, m.data

    def on_driver(self, m):
        try:
            data = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            evs = self.rows.on_status(data)
        for kind, title, detail in evs:
            self.event(kind, title, detail)

    def on_robot_state(self, m):
        try:
            data = json.loads(m.data)
        except ValueError:
            return
        old = self.robot_state['state'] if self.robot_state else None
        with self.lock:
            self.robot_state = data
        st = data.get('state')
        if st != old:
            title = {'RUNNING': 'ROBOT STARTED', 'STOPPED': 'ROBOT STOPPED', 'READY': 'ROBOT READY',
                     'COMPLETE': 'MISSION COMPLETE', 'HALTED': 'ROBOT HALTED'}.get(st, st)
            self.event('ROBOT', title, data.get('reason', ''))

    def on_tracks(self, m):
        try:
            data = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.tracks = data

    def on_vision_event(self, m):
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        if e.get('quiet'):
            return
        detail = f"at ({e['x']:.1f}, {e['y']:.1f}) m, confidence {e['conf']:.2f}"
        self.event(e['kind'], e['msg'], detail, id=e.get('id'))

    def _json(self, m):
        try:
            return json.loads(m.data)
        except ValueError:
            return None

    def on_progress(self, m):
        d = self._json(m)
        if d:
            with self.lock:
                self.progress = d
                if d.get('total_rows'):
                    self.rows.total = d['total_rows']

    def on_obstacles(self, m):
        d = self._json(m)
        if d:
            with self.lock:
                self.obstacles = d.get('obstacles', [])

    def on_obstacle_event(self, m):
        o = self._json(m)
        if not o:
            return
        x0, x1, y0, y1 = self.field_bounds()
        # only obstacles in / next to the crop field are worth a log line (not every fence post)
        if not (x0 - 3 < o['x'] < x1 + 3 and y0 - 3 < o['y'] < y1 + 3):
            return
        title = ('DYNAMIC OBSTACLE DETECTED' if o.get('dynamic') else 'OBSTACLE DETECTED') + f" {o['id']}"
        self.event('OBSTACLE', title, f"{o.get('cls', 'obstacle')} at ({o['x']:.1f}, {o['y']:.1f}) m, "
                                      f"{o.get('distance', 0):.1f} m, {o.get('speed', 0):.2f} m/s")

    def field_bounds(self):
        f = self.field
        return (f.x0, f.x1, float(f.row_y(0)) - f.pitch / 2, float(f.row_y(f.n - 1)) + f.pitch / 2)

    def on_moisture(self, m):
        d = self._json(m)
        if d:
            with self.lock:
                self.moisture = d

    def on_moist_map(self, m):
        with self.lock:
            self.moist_grid = m
        self._render_moisture()

    def on_moist_conf(self, m):
        with self.lock:
            self.moist_conf = m
        self._render_moisture()

    def _render_moisture(self):
        with self.lock:
            g, c, st = self.moist_grid, self.moist_conf, dict(self.moist_status)
        if g is None:
            return
        h, w = g.info.height, g.info.width
        v = np.array(g.data, np.int16).reshape(h, w).astype(float)
        conf = np.array(c.data, np.int16).reshape(h, w) / 100.0 if (c is not None and c.info.width == w) else np.ones((h, w))
        lo, hi = st.get('range', [18.0, 78.0])
        t = np.clip((v - lo) / max(hi - lo, 1e-6), 0, 1)
        # dry (brown) -> normal (pale) -> wet (blue), continuous
        stops = np.array([[140, 81, 10], [216, 179, 101], [245, 245, 220], [90, 180, 172], [1, 102, 94], [8, 69, 148]], float)
        pos = np.linspace(0, 1, len(stops))
        rgb = np.stack([np.interp(t, pos, stops[:, k]) for k in range(3)], -1)
        alpha = np.where(v < 0, 0, 90 + 165 * np.clip(conf, 0, 1))
        img = np.dstack((rgb[..., ::-1], alpha)).astype(np.uint8)[::-1]      # BGRA, row 0 = north
        ok, buf = cv2.imencode('.png', img)
        if ok:
            with self.lock:
                self.moist_png = buf.tobytes()
                self.moist_version += 1
                self.moist_meta = dict(origin_x=g.info.origin.position.x, origin_y=g.info.origin.position.y,
                                       resolution=g.info.resolution, width=w, height=h)

    def on_moist_status(self, m):
        d = self._json(m)
        if d:
            with self.lock:
                self.moist_status = d

    def on_moist_event(self, m):
        e = self._json(m)
        if e:
            self.event(e.get('kind', 'MOISTURE'), e['title'], e.get('detail', ''))

    # ------------------------------------------------------------------ state for the page
    def state(self):
        with self.lock:
            r = self.rows
            pose = self.pose
            rs = dict(self.robot_state or {})
            tracks = dict(self.tracks)
            d = dict(frame=self.frame_id, trail=self.trail[-2500:], events=self.events[:60])
        f = self.field
        d['field'] = dict(rows=[round(float(f.row_y(k)), 3) for k in range(f.n)], x_min=f.x0, x_max=f.x1)
        d['mission_rows'] = [[round(y, 3), round(a, 2), round(b, 2)] for y, a, b in r.rows]
        state = rs.get('state') or ('NO START GATE' if r.mode else 'CONNECTING')
        row = r.row_from_pose(pose[1]) if pose else None
        if r.mode == 'headland':
            rstate, direction = 'TURNING TO NEXT ROW', 'TURNING'
        elif state in ('READY',):
            rstate, direction = 'READY AT ROW START', 'FORWARD'
        elif state in ('STOPPED', 'HALTED'):
            rstate, direction = 'STOPPED', '—'
        elif state == 'COMPLETE':
            rstate, direction = 'FIELD COMPLETE', '—'
        elif r.hold and self.speed < 0.05:
            rstate, direction = 'SAFETY HOLD', 'FORWARD'
        else:
            rstate, direction = 'FOLLOWING ROW', 'FORWARD'
        heading = None
        if pose:
            heading = 'EAST (+x)' if math.cos(pose[2]) > 0.5 else 'WEST (-x)' if math.cos(pose[2]) < -0.5 else 'TURNING'
        d['robot_state'] = dict(state=state, reason=rs.get('reason', ''))
        d['row'] = dict(current=row, total=r.total, completed=r.completed, state=rstate, direction=direction,
                        heading=heading, mode=r.mode, hold=r.hold)
        if pose:
            d['robot'] = dict(x=round(pose[0], 2), y=round(pose[1], 2), yaw=round(pose[2], 4),
                              heading=round(math.degrees(pose[2]), 1), speed=round(self.speed, 2),
                              cmd_speed=round(self.cmd_speed, 2))
        d['counts'] = tracks.get('counts', {})
        d['items'] = tracks.get('items', [])
        d['crops'] = tracks.get('crops', [])
        d['vision'] = dict(ms=tracks.get('ms'), frames=tracks.get('frames'), crop_model=tracks.get('crop_model'))
        d['humans_active'] = [i for i in d['items'] if i['cls'] == 'HUMAN' and i.get('active')]
        with self.lock:
            prog, obs, mo = dict(self.progress), list(self.obstacles), self.moisture
            ms = {k: v for k, v in self.moist_status.items() if k != 'points'}
            pts = self.moist_status.get('points', [])
            d['moisture_map'] = dict(version=self.moist_version, meta=getattr(self, 'moist_meta', None))
        if prog:
            d['row'].update(current=prog.get('row', row), total=prog.get('total_rows', r.total),
                            completed=prog.get('rows_completed', r.completed))
            av = prog.get('avoidance')
            if av and prog.get('phase') == 'row':
                d['row']['state'] = {'AVOIDING': 'AVOIDING OBSTACLE', 'PASSING OBSTACLE': 'PASSING OBSTACLE',
                                     'RETURNING TO ROW': 'RETURNING TO ROW', 'YIELDING': 'YIELDING TO MOVING OBSTACLE',
                                     'WAITING': 'WAITING FOR A CLEAR WAY', 'BLOCKED ON DETOUR': 'WAITING FOR A CLEAR WAY',
                                     'ROW END BLOCKED': 'ROW END BLOCKED'}.get(av['state'], d['row']['state'])
            if prog.get('phase') == 'reverse_turn':
                d['row']['state'], d['row']['direction'] = 'REVERSE U-TURN TO NEXT ROW', 'TURNING'
            if prog.get('phase') == 'row_end' and av:
                d['row']['state'] = 'HEADLAND BLOCKED'
            if prog.get('direction') and d['row']['direction'] == 'FORWARD':
                d['row']['heading'] = 'EAST (+x)' if prog['direction'] == 'EAST' else 'WEST (-x)'
        d['mission'] = prog
        near = [o for o in obs if o.get('distance', 99) < 8.0]
        fb = self.field_bounds()
        d['obstacles'] = [dict(id=o['id'], x=o['x'], y=o['y'], vx=o['vx'], vy=o['vy'], dynamic=o['dynamic'],
                               cls=o.get('cls'), points=o.get('points', [])[:24], distance=o.get('distance'),
                               bearing=o.get('bearing')) for o in obs
                          if fb[0] - 4 < o['x'] < fb[1] + 4 and fb[2] - 4 < o['y'] < fb[3] + 4]
        d['obstacle_now'] = min(near, key=lambda o: o['distance']) if near else None
        d['moisture'] = dict(latest=mo, status=ms, points=pts[-4000:])
        field_done = bool(prog.get('done')) or rs.get('state') == 'COMPLETE'
        d['mission_complete'] = field_done and bool(ms.get('complete'))
        return d

    # ------------------------------------------------------------------ HTTP side
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
                elif self.path.startswith('/moisture.png'):
                    with node.lock:
                        png = node.moist_png
                    if png is None:
                        self._send(404, 'text/plain', b'no moisture map yet')
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
                if not self.path.startswith('/command'):
                    self._send(404, 'text/plain', b'not found')
                    return
                n = int(self.headers.get('Content-Length', 0))
                try:
                    cmd = str(json.loads(self.rfile.read(n))['cmd']).upper()
                except (ValueError, KeyError, TypeError):
                    self._send(400, 'application/json', b'{"ok":false}')
                    return
                if cmd not in ('START', 'STOP'):
                    self._send(400, 'application/json', b'{"ok":false}')
                    return
                node.cmd_pub.publish(String(data=cmd))
                node.event('ROBOT', f'{cmd} ROBOT pressed', 'sent to the row mission start/stop gate')
                self._send(200, 'application/json', b'{"ok":true}')
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
