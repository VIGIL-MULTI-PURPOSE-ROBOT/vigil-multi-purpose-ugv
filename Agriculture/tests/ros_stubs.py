"""Minimal stand-ins for rclpy and ROS message modules so the agriculture nodes can be driven
offline by the tests (no ROS installation needed). install() replaces them in sys.modules."""
import sys
import types
from types import SimpleNamespace


class Msg(SimpleNamespace):
    pass


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)
        if len(self.msgs) > 2000:
            del self.msgs[:1000]


class _Now:
    def __init__(self, t):
        self.nanoseconds = int(t * 1e9)

    def to_msg(self):
        return Msg(sec=int(self.nanoseconds // 10**9), nanosec=int(self.nanoseconds % 10**9))


class Registry:
    def __init__(self):
        self.pubs, self.subs, self.timers, self.t = {}, {}, [], 0.0
        self.params = {}
        self.log = []

    def deliver(self, topic, msg):
        for cb in self.subs.get(topic.lstrip('/'), []):
            cb(msg)


REG = Registry()


class Node:
    def __init__(self, name, **kw):
        self._name = name

    def declare_parameter(self, name, default=None):
        return _Param(REG.params.get(name, default))

    def create_publisher(self, t, topic, qos):
        p = _Pub(topic)
        REG.pubs[topic.lstrip('/')] = p
        return p

    def create_subscription(self, t, topic, cb, qos):
        REG.subs.setdefault(topic.lstrip('/'), []).append(cb)

    def create_timer(self, period, cb):
        REG.timers.append((period, cb))

    def get_logger(self):
        return SimpleNamespace(info=lambda *a, **k: REG.log.append(a[0]), warn=lambda *a, **k: REG.log.append(a[0]))

    def get_clock(self):
        return SimpleNamespace(now=lambda: _Now(REG.t))

    def destroy_node(self):
        pass


def _mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


def _msgcls(name, **defaults):
    def init(self, **kw):
        d = {k: (v() if callable(v) else v) for k, v in defaults.items()}
        d.update(kw)
        Msg.__init__(self, **d)
    return type(name, (Msg,), {'__init__': init})


def install():
    global REG
    REG = Registry()
    vec = lambda: Msg(x=0.0, y=0.0, z=0.0)  # noqa: E731
    rclpy = _mod('rclpy', init=lambda *a, **k: None, ok=lambda: True, shutdown=lambda: None, spin=lambda n: None)
    rclpy.node = _mod('rclpy.node', Node=Node)
    rclpy.qos = _mod('rclpy.qos', QoSProfile=lambda **k: None, DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
                     qos_profile_sensor_data=None)
    rclpy.time = _mod('rclpy.time', Time=lambda *a, **k: None)
    _mod('geometry_msgs')
    _mod('geometry_msgs.msg', Twist=_msgcls('Twist', linear=vec, angular=vec), PoseStamped=_msgcls('PoseStamped', header=Msg, pose=lambda: Msg(position=Msg(x=0.0, y=0.0, z=0.0), orientation=Msg(x=0.0, y=0.0, z=0.0, w=1.0))))
    _mod('nav_msgs')
    _mod('nav_msgs.msg', Odometry=_msgcls('Odometry'), Path=_msgcls('Path', poses=list, header=lambda: Msg(frame_id='', stamp=None)),
         OccupancyGrid=_msgcls('OccupancyGrid', header=lambda: Msg(frame_id='', stamp=None), info=lambda: Msg(resolution=0.0, width=0, height=0, origin=Msg(position=Msg(x=0.0, y=0.0, z=0.0), orientation=Msg(x=0.0, y=0.0, z=0.0, w=1.0))), data=list))
    _mod('sensor_msgs')
    _mod('sensor_msgs.msg', PointCloud2=_msgcls('PointCloud2'), Image=_msgcls('Image'), CameraInfo=_msgcls('CameraInfo'))
    _mod('std_msgs')
    _mod('std_msgs.msg', String=_msgcls('String', data=''), Header=_msgcls('Header'))
    pc2 = _mod('sensor_msgs_py.point_cloud2', read_points_numpy=lambda msg, **k: msg.points)
    _mod('sensor_msgs_py', point_cloud2=pc2)
    _mod('message_filters', Subscriber=lambda *a, **k: None,
         ApproximateTimeSynchronizer=lambda *a, **k: SimpleNamespace(registerCallback=lambda cb: None))
    _mod('tf2_ros', Buffer=lambda: SimpleNamespace(lookup_transform=lambda *a: (_ for _ in ()).throw(RuntimeError())),
         TransformListener=lambda *a: None)
    return REG
