#!/usr/bin/env python3
"""Self-test of the GPS-denied guard against a live ROS 2 graph (no Gazebo needed).

Runs evaluator.py three times on a private ROS_DOMAIN_ID while this script plays the rest of the
graph, and checks the guard's verdict each time:

  A  a rogue node reads /eval/ground_truth                 -> VIOLATION naming /rogue_navigator
  B  only the evaluator reads /eval/ground_truth           -> ISOLATED
  C  a node republishes truth on /sim/ground_truth          -> VIOLATION ("has publisher")

  source /opt/ros/jazzy/setup.bash && python3 tools/vigil_eval/guard_selftest.py
Exit code 0 only if all three cases give the expected verdict.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = os.environ.get('VIGIL_SELFTEST_DOMAIN', '88')
HERE = os.path.dirname(os.path.realpath(__file__))


def run_case(name, rogue_sub, republish, seconds=8.0):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    out = tempfile.mkdtemp(prefix=f'vigil_guard_{name}_')
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN)
    ev = subprocess.Popen([sys.executable, os.path.join(HERE, 'evaluator.py'), '--out', out, '--mode', 'isolation'],
                          env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    rclpy.init()
    bridge = Node('fake_truth_bridge')
    pub = bridge.create_publisher(Odometry, '/eval/ground_truth', 10)
    clock = bridge.create_publisher(Clock, '/clock', 10)   # the evaluator runs on simulation time
    nodes = [bridge]
    if rogue_sub:
        rogue = Node('rogue_navigator')
        rogue.create_subscription(Odometry, '/eval/ground_truth', lambda m: None, 10)
        nodes.append(rogue)
    if republish:
        rep = Node('truth_republisher')
        rpub = rep.create_publisher(Odometry, '/sim/ground_truth', 10)
        nodes.append(rep)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        el = time.monotonic() - t0
        c = Clock()
        c.clock.sec, c.clock.nanosec = int(el), int((el % 1) * 1e9)
        clock.publish(c)
        m = Odometry()
        m.header.frame_id = 'world'
        m.header.stamp = c.clock
        pub.publish(m)
        if republish:
            rpub.publish(m)
        ex.spin_once(timeout_sec=0.05)
    ev.send_signal(signal.SIGINT)
    try:
        log = ev.communicate(timeout=20)[0]
    except subprocess.TimeoutExpired:
        ev.kill()
        log = ev.communicate()[0]
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()
    try:
        with open(os.path.join(out, 'guard.json')) as f:
            g = json.load(f)
    except OSError:
        g = dict(verdict='NO_FILE', log=log[-500:])
    return g, out


def main():
    cases = [('A_rogue_reader', True, False, 'VIOLATION', '/rogue_navigator'),
             ('B_clean', False, False, 'ISOLATED', None),
             ('C_republished_truth', False, True, 'VIOLATION', 'has publisher')]
    ok = True
    for name, rogue, rep, want, needle in cases:
        g, out = run_case(name, rogue, rep)
        text = json.dumps(g)
        good = g.get('verdict') == want and (needle is None or needle in text)
        ok &= good
        print(f"{'PASS' if good else 'FAIL'}  {name:22s} verdict={g.get('verdict')} (expected {want})  "
              f"checks={g.get('checks')}  out={out}")
        for k in ('violations', 'warnings'):
            for line in g.get(k, []):
                print(f'        {k[:-1]}: {line}')
    print('GUARD SELF-TEST', 'PASSED' if ok else 'FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
