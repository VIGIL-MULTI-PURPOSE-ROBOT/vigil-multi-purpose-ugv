"""GPS-denied guard: decide from a snapshot of the ROS graph whether simulator truth can reach the
robot (no ROS dependency; the ROS node in evaluator.py feeds it).

Simulator-truth channels (Phase 1 audit, section 3):
  /sim/ground_truth   true base pose (gz OdometryPublisher)
  /sim/link_poses     true pose of every link (gz PosePublisher)
In truth-isolation mode the bridge publishes them ONLY as /eval/ground_truth and /eval/link_poses.

  oracle topics       /camera/segmentation/*   Gazebo's per-pixel identity of the hit object
  contact topics      /suspension/contacts/*   contact state with WORLD-frame contact positions

Verdicts
  ISOLATED                 truth-isolation mode and no violation (warnings may exist)
  VIOLATION                truth-isolation mode and at least one violation
  GROUND_TRUTH_ASSISTED    baseline mode: lists the nodes that use simulator truth
"""
import fnmatch

TRUTH_SOURCE = ('/sim/ground_truth', '/sim/link_poses')
EVAL_TOPICS = ('/eval/ground_truth', '/eval/link_poses')
ORACLE_PREFIX = '/camera/segmentation/'
CONTACT_PREFIX = '/suspension/contacts/'

# Nodes allowed to read /eval/* : the evaluator, the regression monitor (evaluation only) and the
# bag recorder. '_ros2cli_*' are short-lived nodes of `ros2 topic echo/hz` typed by a person.
DEFAULT_ALLOWED = ('/vigil_evaluator', '/vigil_regression_monitor', '/rosbag2_recorder', '/_ros2cli_*')
# TF publishers expected in Rock Terrain milestone 1 (anything else is reported as a warning).
DEFAULT_TF_PUBLISHERS = ('/robot_state_publisher',)


def _allowed(node, patterns):
    return any(fnmatch.fnmatch(node, p) for p in patterns)


def judge(mode, graph, allowed=DEFAULT_ALLOWED, tf_publishers=DEFAULT_TF_PUBLISHERS):
    """mode: 'isolation' or 'baseline'. graph: {topic: {'subs': [node,...], 'pubs': [node,...]}}.

    Returns dict(verdict, violations[], warnings[], truth_users[], dormant[])."""
    v, w, users, dormant = [], [], [], []

    def subs(t):
        return sorted(set(graph.get(t, {}).get('subs', [])))

    def pubs(t):
        return sorted(set(graph.get(t, {}).get('pubs', [])))

    for t in TRUTH_SOURCE:
        s, p = subs(t), pubs(t)
        if p and s:
            users += [f'{n} <- {t}' for n in s if not _allowed(n, allowed)]
        if mode == 'isolation':
            if p:
                v.append(f'{t} has publisher(s) {p}: simulator truth is on an operational topic')
            for n in s:
                if not p and not _allowed(n, allowed):
                    dormant.append(f'{n} subscribes to {t} (no publisher: receives nothing)')
    for t in EVAL_TOPICS:
        bad = [n for n in subs(t) if not _allowed(n, allowed)]
        if bad:
            (v if mode == 'isolation' else w).append(f'{t} read by non-evaluation node(s) {bad}')
            users += [f'{n} <- {t}' for n in bad]
    for t in sorted(graph):
        if t.startswith(ORACLE_PREFIX) and pubs(t):
            readers = [n for n in subs(t) if not _allowed(n, allowed)]
            msg = f'{t} is published (Gazebo segmentation oracle); readers: {readers or "none"}'
            if mode == 'isolation':
                v.append(msg)
            else:
                users += [f'{n} <- {t}' for n in readers]
        if t.startswith(CONTACT_PREFIX) and pubs(t):
            readers = [n for n in subs(t) if not _allowed(n, allowed)]
            if readers:
                msg = f'{t} (world-frame contact positions) read by {readers}'
                (v if mode == 'isolation' else w).append(msg)
    tfp = [n for n in pubs('/tf') if not _allowed(n, tuple(tf_publishers) + tuple(allowed))]
    if tfp:
        w.append(f'/tf published by nodes outside the expected set: {tfp}')
    if mode == 'isolation':
        verdict = 'VIOLATION' if v else 'ISOLATED'
    else:
        verdict = 'GROUND_TRUTH_ASSISTED' if users else 'NO_TRUTH_USER_SEEN'
    return dict(verdict=verdict, violations=v, warnings=w, truth_users=sorted(set(users)),
                dormant=sorted(set(dormant)))


def merge(history):
    """Combine per-second judgements into one run verdict (any violation anywhere = VIOLATION)."""
    if not history:
        return dict(verdict='NO_DATA', checks=0)
    out = dict(checks=len(history), violations=[], warnings=[], truth_users=[], dormant=[])
    for h in history:
        for k in ('violations', 'warnings', 'truth_users', 'dormant'):
            out[k] = sorted(set(out[k]) | set(h[k]))
    verdicts = {h['verdict'] for h in history}
    for vv in ('VIOLATION', 'GROUND_TRUTH_ASSISTED', 'ISOLATED', 'NO_TRUTH_USER_SEEN'):
        if vv in verdicts:
            out['verdict'] = vv
            break
    return out
