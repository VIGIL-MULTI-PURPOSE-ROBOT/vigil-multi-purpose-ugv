#!/usr/bin/python3
"""cmd_vel -> the eight wheel VELOCITY commands (velocity_controllers/JointGroupVelocityController).

Wheel kinematics as vigil_rough_terrain: skid steer, wheel = (v -/+ w * skid) / r with each wheel's own
CAD radius. What limits the motion (config/physics.yaml -> physics.drive):

  max_linear      3.0 m/s    open-road top speed
  max_accel       1.0 m/s2   speeding up        } jerk-limited S-curve: the acceleration itself
  max_decel       2.0 m/s2   braking            } changes by at most max_jerk per second, so the
  max_jerk        3.0 m/s3   (speeding up)      } wheels never get a torque step
  max_jerk_brake  8.0 m/s3   (braking: builds 2.0 m/s2 in 0.25 s, so stops at B and hazards stay short)
  max_angular     1.0 rad/s, ramp_angular 3.0 rad/s2
  max_wheel_accel 25 rad/s2  last-resort cap on any single wheel's speed change

Why: the previous 8 m/s2 ramp was above what this rover can take before the front lifts - 7.7 m/s2 on
flat ground, 1.9 m/s2 on a 33 deg climb (centre of mass 0.33 m high, 0.31 m ahead of the rearmost
loaded wheel; see scripts/rover_stability.py). The velocity controller then made each 0.16 m/s step
with up to the full 400 N.m per wheel. The 2026-09-23 17:18 run pitched back 39 deg on the flat apron.
Nothing here pushes the rover down or moves it: only the wheel speed commands are shaped.

Also: the command watchdog runs on SIMULATION time (the big world runs below real time), and
/drive/status (JSON, 5 Hz) reports the command, the shaped speed/acceleration and every wheel.
"""
import json
import math
import os
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
except ImportError:            # DriveLaw alone (offline tests without ROS 2)
    rclpy = None
    Node = object

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from ros_common import nested_section, node_time  # noqa: E402

COMMAND_TIMEOUT = 0.6          # s of SIMULATION time without cmd_vel -> brake to a stop
WHEELS = ('L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4')
RADIUS = {'1': 0.23463, '2': 0.17655, '3': 0.17655, '4': 0.17655}   # CAD defaults; physics.rover overrides
SIDE = (1, 1, 1, 1, -1, -1, -1, -1)
TICK = 0.01                    # s: 100 Hz = the controller_manager update rate


class DriveLaw:
    """The motion limits, without ROS (the node below and test/skid_model.py both use this).

    Linear speed follows a jerk-limited profile: the acceleration moves towards
    sign(dv) * min(a_max, sqrt(jerk * |dv|)) by at most jerk * dt per tick, so it rises smoothly,
    and tapers off as the speed arrives (no overshoot). a_max is max_accel while |v| grows and
    max_decel while it shrinks. Yaw rate is ramped. Each wheel's commanded speed may then change by at
    most max_wheel_accel * dt."""

    def __init__(self, d, rover=None):
        rover = rover or {}
        front = float(rover.get('wheel_radius_front', RADIUS['1']))
        small = float(rover.get('wheel_radius_small', RADIUS['2']))
        self.radius = {'1': front, '2': small, '3': small, '4': small}
        self.max_v = float(d.get('max_linear', 3.0))
        self.max_w = float(d.get('max_angular', 1.0))
        self.acc = float(d.get('max_accel', 1.0))
        self.dec = float(d.get('max_decel', 2.0))
        self.jerk = float(d.get('max_jerk', 3.0))
        self.jerk_brake = float(d.get('max_jerk_brake', self.jerk))
        self.ramp_w = float(d.get('ramp_angular', 3.0))
        self.wheel_acc = float(d.get('max_wheel_accel', 25.0))
        self.skid = float(d.get('skid_factor', 0.65))
        self.v = self.a = self.w = 0.0
        self.wheels = [0.0] * 8
        # ACCELERATION + (dashboard): a few speeding-up limits, the first = max_accel (validated).
        # Only the acceleration limit changes: max_linear, braking, jerk and the wheel cap stay, and
        # the jerk limit makes a level change itself smooth (the acceleration never steps).
        self.base_acc = self.acc
        levels = d.get('accel_levels') or [self.acc]
        self.levels = [float(v) for v in levels]
        if abs(self.levels[0] - self.acc) > 1e-9:
            self.levels = [self.acc] + [v for v in self.levels if v > self.acc]
        names = list(d.get('accel_level_names') or [])
        self.level_names = [names[k] if k < len(names) else f'{v:g} m/s2' for k, v in enumerate(self.levels)]
        self.level = 0
        # never above stability_margin x the wheelie limit at the current pitch (rover_stability.py
        # model: flat 7.7 m/s2, 33 deg climb 1.9 m/s2), and never below the validated max_accel
        self.stab_margin = float(d.get('stability_margin', 0.5))
        self.com_h = float(d.get('com_height', 0.33))
        self.com_back = float(d.get('com_ahead_of_rear_contact', 0.31))
        self.spin_factor = float(d.get('wheelie_spin_factor', 0.835))
        self.pitch = 0.0
        self.acc_cap = math.inf

    # ------------------------------------------------------------ acceleration levels
    @property
    def level_name(self):
        return self.level_names[self.level]

    def wheelie_limit(self, pitch):
        """Front-lift acceleration (m/s2) on a climb of `pitch` rad (nose up > 0)."""
        g = 9.81
        up = max(0.0, pitch)
        return max(0.0, g * (self.com_back * math.cos(up) - self.com_h * math.sin(up)) / self.com_h * self.spin_factor)

    def _apply_level(self):
        want = self.levels[self.level]
        self.acc_cap = max(self.base_acc, self.stab_margin * self.wheelie_limit(self.pitch))
        self.acc = min(want, self.acc_cap)

    def set_level(self, k):
        self.level = max(0, min(len(self.levels) - 1, int(k)))
        self._apply_level()

    def step_level(self, d=1):
        self.set_level(self.level + d)

    def reset_level(self):
        self.set_level(0)

    def set_pitch(self, pitch):
        self.pitch = float(pitch)
        self._apply_level()

    def clamp_cmd(self, v, w):
        return max(-self.max_v, min(self.max_v, v)), max(-self.max_w, min(self.max_w, w))

    def tick(self, v_t, w_t, dt=TICK):
        v_t, w_t = self.clamp_cmd(v_t, w_t)
        dv = v_t - self.v
        speeding_up = abs(v_t) > abs(self.v) and v_t * self.v >= 0.0
        a_max, jerk = (self.acc, self.jerk) if speeding_up else (self.dec, self.jerk_brake)
        # taper curve a = sqrt(jerk * dv): half the jerk budget, so the discrete taper can follow it
        # and the acceleration is ~0 when the speed arrives (no residual step at the end)
        a_goal = math.copysign(min(a_max, math.sqrt(jerk * abs(dv))), dv) if dv else 0.0
        self.a += max(-jerk * dt, min(jerk * dt, a_goal - self.a))
        self.a = max(-max(self.acc, self.dec), min(max(self.acc, self.dec), self.a))
        nv = self.v + self.a * dt
        if (v_t - nv) * dv <= 0.0:          # arrived (or would pass it): hold the target
            nv, self.a = v_t, 0.0
        self.v = nv
        self.w += max(-self.ramp_w * dt, min(self.ramp_w * dt, w_t - self.w))
        cap = self.wheel_acc * dt
        want = [(self.v - side * self.w * self.skid) / self.radius[n[1]] for n, side in zip(WHEELS, SIDE)]
        self.wheels = [old + max(-cap, min(cap, new - old)) for old, new in zip(self.wheels, want)]
        return self.wheels


class Drive(Node):
    def __init__(self):
        super().__init__('terrain_drive', automatically_declare_parameters_from_overrides=True)
        phys = nested_section(self, 'physics')
        d = phys.get('drive', {})
        self.law = DriveLaw(d, phys.get('rover', {}))
        self.max_v, self.max_w = self.law.max_v, self.law.max_w
        self.target = [0.0, 0.0]
        self.current = [0.0, 0.0]
        self.last = -1e9
        self.n_cmd = 0
        self.omega = {w: 0.0 for w in WHEELS}
        self.n_js = 0
        self.wheels = [0.0] * 8
        self.t_prev = None
        self.pub = self.create_publisher(Float64MultiArray, 'wheel_controller/commands', 10)
        self.pub_status = self.create_publisher(String, 'drive/status', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        # ACCELERATION + button (dashboard): UP | DOWN | NORMAL | <level index>
        self.create_subscription(String, 'drive/accel_command', self.on_accel, 10)
        # pitch -> the level is capped below the wheelie limit on climbs
        self.create_subscription(Odometry, 'sim/ground_truth', self.on_odom, 10)
        self.create_timer(TICK, self.tick)
        self.create_timer(0.2, self.report)
        self.get_logger().info(
            f'drive: wheel VELOCITY control, max {self.max_v:.2f} m/s / {self.max_w:.2f} rad/s, accel '
            f'{self.law.acc:.2f} m/s2, brake {self.law.dec:.2f} m/s2, jerk {self.law.jerk:.2f} / brake '
            f'{self.law.jerk_brake:.2f} m/s3, yaw ramp '
            f'{self.law.ramp_w:.2f} rad/s2, wheel accel cap {self.law.wheel_acc:.0f} rad/s2, skid {self.law.skid:.2f} m')

    def command(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.target = list(self.law.clamp_cmd(msg.linear.x, msg.angular.z))
            self.last = node_time(self)
            self.n_cmd += 1

    def on_accel(self, msg):
        c = msg.data.strip().upper()
        before = self.law.level_name
        if c in ('UP', '+', 'ACCEL+'):
            self.law.step_level(+1)
        elif c in ('DOWN', '-'):
            self.law.step_level(-1)
        elif c in ('NORMAL', 'RESET', '0'):
            self.law.reset_level()
        elif c.isdigit():
            self.law.set_level(int(c))
        self.get_logger().info(f'acceleration {before} -> {self.law.level_name}: speeding-up limit '
                               f'{self.law.levels[self.law.level]:.2f} m/s2 (now {self.law.acc:.2f}, stability cap '
                               f'{self.law.acc_cap:.2f}); top speed {self.law.max_v:.2f} m/s unchanged')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        # nose-up pitch of the body (x axis) = -asin(R[2,0])
        r20 = 2.0 * (q.x * q.z - q.w * q.y)
        self.law.set_pitch(-math.asin(max(-1.0, min(1.0, r20))))

    def on_joints(self, msg):
        self.n_js += 1
        for name, vel in zip(msg.name, msg.velocity):
            key = name.replace('_joint', '')
            if key in self.omega:
                self.omega[key] = float(vel)

    def tick(self):
        now = node_time(self)
        dt = TICK if self.t_prev is None else max(0.0, min(0.05, now - self.t_prev))
        self.t_prev = now
        if dt <= 0.0:
            return                                   # sim time paused: nothing to integrate
        fresh = now - self.last < COMMAND_TIMEOUT
        target = self.target if fresh else [0.0, 0.0]
        self.wheels = self.law.tick(target[0], target[1], dt)
        self.current = [self.law.v, self.law.w]
        self.pub.publish(Float64MultiArray(data=self.wheels))

    def report(self):
        now = node_time(self)
        d = dict(mode='velocity', cmd_v=round(self.target[0], 3), cmd_w=round(self.target[1], 3),
                 ramped_v=round(self.current[0], 3), ramped_w=round(self.current[1], 3),
                 accel=round(self.law.a, 3), max_accel=round(self.law.acc, 3), max_decel=self.law.dec,
                 max_jerk=self.law.jerk, accel_level=self.law.level, accel_level_name=self.law.level_name,
                 accel_levels=self.law.levels, accel_level_names=self.law.level_names,
                 accel_request=self.law.levels[self.law.level], accel_stability_cap=round(self.law.acc_cap, 2),
                 pitch_deg=round(math.degrees(self.law.pitch), 1),
                 cmd_fresh=now - self.last < COMMAND_TIMEOUT,
                 cmd_age=round(now - self.last, 2) if self.last > 0 else None,
                 cmd_msgs=self.n_cmd, joint_state_msgs=self.n_js, max_linear=self.max_v, max_angular=self.max_w,
                 wheels={n: (round(t, 2), round(self.omega[n], 2)) for n, t in zip(WHEELS, self.wheels)})
        self.pub_status.publish(String(data=json.dumps(d)))


def main():
    rclpy.init()
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Float64MultiArray(data=[0.0] * 8))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
