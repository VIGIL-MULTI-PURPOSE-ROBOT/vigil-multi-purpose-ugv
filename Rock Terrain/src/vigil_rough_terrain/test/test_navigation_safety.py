#!/usr/bin/python3
"""Unit tests for the navigation safety / bounded-failure behaviour (no ROS, no Gazebo).

    python3 -m pytest -q test/test_navigation_safety.py

They drive planner_core.NavigatorCore directly with synthetic terrain-class maps, so they test the
real decision logic: replanning hysteresis, path stability, recovery limits, unreachable-goal
detection, stale terrain / pose handling, safe stop (zero velocity), new-goal restart, the motion
shield, unknown-terrain handling when reversing, and the edge-scan reference fix. Sensor physics is
NOT simulated here (see closed_loop_sim.py for the closed-loop scenarios).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / 'scripts'))
sys.path.insert(0, str(PKG / 'test'))

from terrain_core import Params, Pose, rpy_to_matrix, SAFE, CLIFF, UNKNOWN, POSSIBLE_DROP  # noqa: E402
from planner_core import NavigatorCore  # noqa: E402


def cfg():
    return yaml.safe_load((PKG / 'config/vision_nav.yaml').read_text())['/**']['ros__parameters']


def make(goal=(8.0, 0.0), **nav_over):
    c = cfg()
    tp = Params.from_nested(c)
    nav = dict(c['navigation'])
    nav.update(start_delay=0.0, goal_x=goal[0], goal_y=goal[1])
    nav.update(nav_over)
    return tp, NavigatorCore(tp, nav)


def grid(tp, fill=SAFE):
    n = int(round(tp.map_size / tp.resolution))
    return np.full((n, n), fill, np.uint8)


def cells(tp, x0, x1, y0, y1):
    """Index slices of the world rectangle [x0,x1] x [y0,y1] (rows = y)."""
    r = tp.resolution
    return (slice(int((y0 - tp.map_origin_y) / r), int((y1 - tp.map_origin_y) / r)),
            slice(int((x0 - tp.map_origin_x) / r), int((x1 - tp.map_origin_x) / r)))


def pose(x=0.0, y=0.0, yaw=0.0):
    return Pose(x=x, y=y, z=0.0, R=rpy_to_matrix(0.0, 0.0, yaw))


def drive(nav, cls, t0, t1, p=None, dt=0.1, map_every=0.2, move=True):
    """Tick the navigator; integrate unicycle motion; publish `cls` every map_every s."""
    p = p or pose()
    x, y, yaw = p.x, p.y, p.yaw
    t, next_map, cmds = t0, t0, []
    while t < t1 - 1e-9:
        if map_every and t >= next_map - 1e-9:
            nav.update_map(cls, t=t)
            next_map += map_every
        v, w = nav.tick(t, pose(x, y, yaw), t)
        cmds.append((t, v, w))
        if move:
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
        t += dt
    return cmds, pose(x, y, yaw)


# ------------------------------------------------------------------ hysteresis / stability
def test_no_replanning_on_unchanged_map_even_if_blocked_reason_repeats():
    """Baseline thrash: a blocked-ahead reason set every tick replanned every tick (180 of 210 plans
    < 0.15 s apart). Now a hazard replan needs new map data or min_replan_interval."""
    tp, nav = make()
    cls = grid(tp)
    nav.update_map(cls, t=0.0)
    plans = []
    orig = nav._plan

    def spy(t, p):
        plans.append(t)
        return orig(t, p)
    nav._plan = spy
    for k in range(30):                      # 3 s, map NOT updated
        nav.replan_reason = ('hazard', 'PATH BLOCKED')
        nav.tick(k * 0.1, pose(), k * 0.1)
    # first plan + at most one per min_replan_interval (1.0 s) on the unchanged map
    assert len(plans) <= 1 + int(3.0 / nav.nav['min_replan_interval']) + 1


def test_replan_happens_when_new_map_arrives():
    tp, nav = make()
    cls = grid(tp)
    nav.update_map(cls, t=0.0)
    nav.tick(0.0, pose(), 0.0)
    n0 = nav.last_plan_t
    nav.replan_reason = ('hazard', 'CLIFF AHEAD')
    nav.update_map(cls, t=0.1)               # new terrain data
    nav.tick(0.1, pose(), 0.1)
    assert nav.last_plan_t > n0


def test_path_is_stable_on_a_static_map():
    tp, nav = make(goal=(8.0, 0.0))
    cls = grid(tp)
    drive(nav, cls, 0.0, 0.2, move=False)
    v0 = nav.path_version
    drive(nav, cls, 0.2, 10.0, move=False)    # 10 s on the same map, robot not moving
    assert nav.path_version == v0, 'path must not be replaced without a better or safer route'
    assert nav.hazard_replans == 0


# ------------------------------------------------------------------ bounded failure
def test_recovery_limit_reached_is_terminal_and_stops():
    tp, nav = make(max_recoveries=2)
    cls = grid(tp)
    nav.update_map(cls, t=0.0)
    nav.tick(0.0, pose(), 0.0)
    for k in range(3):
        nav._start_recovery(1.0 + k, pose(), 'test')
        nav.recovery = None
    assert nav.state == 'RECOVERY_LIMIT_REACHED'
    assert 'limit 2' in nav.failure_reason
    for k in range(20):
        assert nav.tick(5.0 + k * 0.1, pose(), 5.0 + k * 0.1) == (0.0, 0.0)
    assert nav.status()['terminal'] is True


def enclosed_goal_map(tp, goal=(8.0, 0.0), r_in=1.6, r_out=2.4):
    cls = grid(tp)
    n = cls.shape[0]
    c = tp.map_origin_x + (np.arange(n) + 0.5) * tp.resolution
    X, Y = np.meshgrid(c, c)
    d = np.hypot(X - goal[0], Y - goal[1])
    cls[(d >= r_in) & (d <= r_out)] = CLIFF     # a closed ring of cliff around B
    return cls


def test_unreachable_goal_needs_confirmation_then_stops():
    tp, nav = make(goal=(8.0, 0.0))
    cls = enclosed_goal_map(tp)
    cmds, _ = drive(nav, cls, 0.0, 3.0, move=False)
    assert nav.state not in nav.TERMINAL, 'one failed planning attempt must never be terminal'
    cmds, _ = drive(nav, cls, 3.0, 30.0, move=False)
    assert nav.state == 'GOAL_UNREACHABLE'
    assert 'through known or unexplored terrain' in nav.failure_reason
    # confirmation used time AND several new maps (never a single attempt)
    assert nav.nav['unreachable_confirm_maps'] >= 2
    after, _ = drive(nav, cls, 30.0, 35.0, move=False)     # terminal: zero velocity from now on
    assert all(v == 0.0 and w == 0.0 for _, v, w in after)
    assert nav.state == 'GOAL_UNREACHABLE'


def test_temporary_blockage_is_not_declared_unreachable():
    """A ring that opens again before the confirmation time must not end the mission."""
    tp, nav = make(goal=(8.0, 0.0))
    closed = enclosed_goal_map(tp)
    drive(nav, closed, 0.0, nav.nav['unreachable_confirm_time'] * 0.5, move=False)
    drive(nav, grid(tp), nav.nav['unreachable_confirm_time'] * 0.5, 20.0, move=False)
    assert nav.state not in nav.FAILURES
    assert nav.closed_since is None


def test_no_progress_budget_ends_in_goal_unreachable():
    tp, nav = make(goal=(8.0, 0.0), no_progress_failure_time=10.0)
    cls = grid(tp)
    cmds, _ = drive(nav, cls, 0.0, 15.0, move=False)  # robot never moves: no progress
    assert nav.state == 'GOAL_UNREACHABLE'
    assert 'no meaningful progress' in nav.failure_reason


# ------------------------------------------------------------------ stale inputs
def test_stale_terrain_holds_then_sensor_timeout():
    tp, nav = make()
    cls = grid(tp)
    drive(nav, cls, 0.0, 1.0)
    cmds, p = drive(nav, cls, 1.0, 2.5, map_every=0)        # map stops
    late = [c for c in cmds if c[0] > 1.0 + nav.nav['map_stale_time'] + 0.15]
    assert late and all(v == 0.0 and w == 0.0 for _, v, w in late)
    assert nav.state == 'HOLD'
    drive(nav, cls, 2.5, 7.0, p=p, map_every=0)
    assert nav.state == 'SENSOR_TIMEOUT'


def test_stale_terrain_recovers_before_timeout():
    tp, nav = make()
    cls = grid(tp)
    _, p = drive(nav, cls, 0.0, 1.0)
    _, p = drive(nav, cls, 1.0, 3.0, p=p, map_every=0)      # 2 s gap: HOLD, not terminal
    assert nav.state == 'HOLD'
    cmds, _ = drive(nav, cls, 3.0, 5.0, p=p)                  # map resumes
    assert nav.state not in nav.TERMINAL
    assert any(v > 0.05 for _, v, _ in cmds)


def test_stale_pose_holds_then_localization_lost():
    tp, nav = make()
    cls = grid(tp)
    drive(nav, cls, 0.0, 1.0)
    for k in range(10, 60):                                  # pose stamp frozen at t = 1.0
        t = k * 0.1
        if k % 2 == 0:
            nav.update_map(cls, t=t)
        v, w = nav.tick(t, pose(), 1.0)
        if t > 1.0 + nav.nav['pose_stale_time'] + 0.15:
            assert (v, w) == (0.0, 0.0)
    assert nav.state == 'LOCALIZATION_LOST'


def test_no_pose_at_all_waits_then_localization_lost():
    tp, nav = make(startup_timeout=5.0)
    for k in range(80):
        assert nav.tick(k * 0.1, None, None) == (0.0, 0.0)
    assert nav.state == 'LOCALIZATION_LOST'


def test_nonfinite_pose_is_never_used():
    tp, nav = make()
    cls = grid(tp)
    nav.update_map(cls, t=0.0)
    bad = Pose(x=float('nan'), y=0.0, z=0.0, R=np.eye(3))
    for k in range(5):
        assert nav.tick(k * 0.1, bad, k * 0.1) == (0.0, 0.0)


# ------------------------------------------------------------------ safe stop / new goal
def test_terminal_failure_never_reports_goal_reached_and_new_goal_restarts():
    tp, nav = make(goal=(8.0, 0.0), no_progress_failure_time=5.0)
    cls = grid(tp)
    drive(nav, cls, 0.0, 8.0, move=False)
    assert nav.state == 'GOAL_UNREACHABLE'
    assert nav.status()['state'] != 'GOAL_REACHED'
    nav.set_goal(4.0, 0.0, t=8.0)
    assert nav.state not in nav.TERMINAL
    assert nav.recoveries == 0 and nav.failure_reason is None
    cmds, p = drive(nav, cls, 8.0, 30.0)
    assert any(v > 0.05 for _, v, _ in cmds), 'navigation must restart after a new goal'
    assert nav.state == 'GOAL_REACHED' and math.hypot(p.x - 4.0, p.y) < 0.5


def test_goal_reached_only_by_distance():
    tp, nav = make(goal=(3.0, 0.0))
    cls = grid(tp)
    _, p = drive(nav, cls, 0.0, 20.0)
    assert nav.state == 'GOAL_REACHED'
    assert math.hypot(p.x - 3.0, p.y) <= nav.nav['goal_tolerance'] + 1e-6


# ------------------------------------------------------------------ cliff safety
def test_motion_shield_blocks_turn_into_cliff():
    """A turn in place whose corner would sweep into a cliff must not be commanded (the original
    _follow turned anyway when rotation was not clear). The robot is already turning at 0.6 rad/s:
    since the drive model was added the shield predicts the ACTUAL rotation, and a robot starting
    from rest only rotates ~0.2 rad before it can stop again (tested separately below)."""
    tp, nav = make()
    cls = grid(tp)
    # cliff 0.25 m beside the robot's left side
    cls[cells(tp, -2.0, 2.0, 0.56 + 0.25, 3.0)] = CLIFF
    nav.update_map(cls, t=0.0)
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    nav.vel_est, nav.cmd = (0.0, 0.6), (0.0, 0.6)
    assert not nav._motion_clear(pose(), 0.0, 0.6, 0.10)
    v, w = nav._shield(0.0, pose(), 0.0, 0.6)
    assert (v, w) == (0.0, 0.0)


def test_reversing_into_unobserved_terrain_is_blocked():
    tp, nav = make()
    cls = grid(tp, UNKNOWN)
    cls[cells(tp, -0.9, 3.0, -1.0, 1.0)] = SAFE              # observed ahead/under, unknown behind
    nav.update_map(cls, t=0.0)
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    assert not nav.planner.rear_clear(0.0, 0.0, 0.0, 0.3)
    cls[cells(tp, -3.0, 3.0, -1.0, 1.0)] = SAFE              # now observed behind too
    nav.update_map(cls, t=0.1)
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    assert nav.planner.rear_clear(0.0, 0.0, 0.0, 0.3)


def test_heading_search_sweeps_the_whole_footprint():
    """The CLIFF_AVOIDANCE heading search used to check only the robot centre line plus the end
    pose without margin; every accepted segment must now pass the swept-footprint check."""
    tp, nav = make(goal=(8.0, 2.0))
    cls = grid(tp)
    cls[cells(tp, 2.0, 4.0, 0.8, 1.6)] = CLIFF               # pit corner towards the goal heading
    nav.update_map(cls, t=0.0)
    p = pose(0.6, 0.0, 0.0)
    nav.planner.set_body_cells(p.x, p.y, p.yaw)
    seg, info = nav._best_heading(p)
    assert seg is not None
    near = nav._near(p)
    assert nav.planner.check_path(seg, 0, horizon=3.0, skip=0.0, **near) is None


def test_body_overlap_cells_do_not_freeze_the_robot():
    tp, nav = make()
    cls = grid(tp)
    cls[cells(tp, 0.5, 0.6, 0.0, 0.1)] = CLIFF               # spurious cell under the front of the body
    nav.update_map(cls, t=0.0)
    assert nav.planner.start_clearance(0.0, 0.0, 0.0) == -1.0   # body overlaps the spurious cell
    assert nav.planner.set_body_cells(0.0, 0.0, 0.0) > 0
    # the cell already under the body is not a reason to freeze: straight ahead / turning in place
    # only touch that same cell, so they are allowed
    assert nav._motion_clear(pose(), 0.3, 0.0, 0.0)
    assert nav._motion_clear(pose(), 0.0, 0.6, 0.0)
    # a motion that brings NEW hazard cells under the footprint stays forbidden
    cls2 = cls.copy()
    cls2[cells(tp, 0.9, 1.3, -0.4, 0.4)] = CLIFF
    nav.update_map(cls2, t=0.1)
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    assert not nav._motion_clear(pose(), 0.6, 0.0, 0.0)


def test_edge_scan_does_not_mark_the_robot_front_as_cliff():
    """Regression for the false HIGH CLIFF cells at the robot's own front (observed 55x in
    cliff_front, 165x in narrow_route offline): an event whose reference is still the robot front
    must not set edge_drop on that cell."""
    from terrain_core import TerrainMapper
    c = cfg()
    tp = Params.from_nested(c)
    m = TerrainMapper(tp)
    fx, fy = 0.8, 0.0
    m._apply_events([(fx, fy, 0.0, 3.0, 0.0, 1.5, 1.2, False, True)], [])
    i, j = m.cell(fx, fy)
    assert m.edge_drop[i, j] == 0.0
    m._apply_events([(fx, fy, 0.0, 3.0, 0.0, 1.5, 1.2, False, False)], [])
    assert m.edge_drop[i, j] == pytest.approx(1.5)



# ------------------------------------------------------------------ stopping distance (Gazebo findings)
def test_shield_accounts_for_stopping_distance():
    """Gazebo regression: the robot coasted ~0.15 m after a stop command (drive.py ramps the wheels
    at 0.5 m/s^2) and overhung the plateau edge. The same forward command must be judged by where
    the robot can actually stop: fine from rest, rejected at full speed."""
    tp, nav = make()
    cls = grid(tp)
    front = tp.footprint_center_x + tp.robot_length / 2
    cls[cells(tp, front + 0.7, front + 3.0, -2.0, 2.0)] = CLIFF      # drop 0.7 m ahead of the front
    nav.update_map(cls, t=0.0)
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    nav.vel_est, nav.cmd = (0.0, 0.0), (0.0, 0.0)
    assert nav._motion_clear(pose(), 0.6, 0.0, 0.10)                 # from rest: stops well short
    nav.vel_est, nav.cmd = (0.6, 0.0), (0.6, 0.0)
    assert not nav._motion_clear(pose(), 0.6, 0.0, 0.10)             # at 0.6 m/s: ~0.8 m to stop
    v, w = nav._shield(0.0, pose(), 0.6, 0.0)
    assert v < 0.6                                                   # slowed or stopped, never full speed


def test_velocity_estimate_follows_drive_ramp():
    tp, nav = make()
    nav.cmd, nav.last_tick_t = (0.6, 0.0), 0.0
    nav.vel_est = nav._ramp(nav.vel_est, nav.cmd, 0.5)
    assert nav.vel_est[0] == pytest.approx(0.25)                     # 0.5 m/s^2 for 0.5 s
    nav.vel_est = nav._ramp(nav.vel_est, (0.0, 0.0), 0.2)
    assert nav.vel_est[0] == pytest.approx(0.15)


def test_speed_is_capped_before_unobserved_terrain():
    """Do not outrun the map: unknown terrain right ahead -> creep speed; known-safe ahead -> cruise."""
    tp, nav = make()
    front = tp.footprint_center_x + tp.robot_length / 2
    cls = grid(tp)
    nav.update_map(cls, t=0.0)
    assert nav._seen_speed_cap(pose()) == pytest.approx(nav.nav['cruise_speed'])
    cls2 = grid(tp)
    cls2[cells(tp, front + 0.05, front + 5.0, -3.0, 3.0)] = UNKNOWN   # unseen inside the margin
    nav.update_map(cls2, t=0.1)
    cap = nav._seen_speed_cap(pose())
    assert cap == pytest.approx(nav.nav['creep_speed'])
    cls3 = grid(tp)
    cls3[cells(tp, front + 0.6, front + 5.0, -3.0, 3.0)] = UNKNOWN   # 0.6 m known ahead
    nav.update_map(cls3, t=0.2)
    cap3 = nav._seen_speed_cap(pose())
    a = nav.nav['drive_linear_accel']
    lat = nav.nav['command_latency'] + nav.nav['perception_latency']
    d_stop = cap3 * lat + cap3 ** 2 / (2 * a)
    assert nav.nav['creep_speed'] < cap3 < nav.nav['cruise_speed']
    assert d_stop <= 0.6 - nav.nav['shield_margin'] + 0.051         # stops before the unknown cells


def test_path_following_applies_the_seen_terrain_cap():
    tp, nav = make()
    front = tp.footprint_center_x + tp.robot_length / 2
    cls = grid(tp)
    cls[cells(tp, front + 0.05, front + 5.0, -3.0, 3.0)] = UNKNOWN
    nav.update_map(cls, t=0.0)
    nav.path = np.array([[x, 0.0] for x in np.arange(0.0, 6.0, 0.2)])
    nav.progress = 0
    v, w = nav._follow(pose())
    assert 0.0 < v <= nav.nav['creep_speed'] + 1e-9
    cls[cls == UNKNOWN] = SAFE
    nav.update_map(cls, t=0.1)
    v2, _ = nav._follow(pose())
    assert v2 > nav.nav['creep_speed']


def test_unconfirmed_drop_blocks_motion_onto_it():
    """POSSIBLE_DROP cells (the mapper could not confirm or clear the drop, e.g. when driving
    alongside an edge) must not come under the footprint + margin."""
    tp, nav = make()
    front = tp.footprint_center_x + tp.robot_length / 2
    cls = grid(tp)
    cls[cells(tp, front + 0.3, front + 2.0, -2.0, 2.0)] = POSSIBLE_DROP
    nav.update_map(cls, t=0.0)
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    nav.vel_est, nav.cmd = (0.3, 0.0), (0.3, 0.0)
    assert not nav._motion_clear(pose(), 0.3, 0.0, 0.10)
    cls[cls == POSSIBLE_DROP] = SAFE
    nav.update_map(cls, t=0.1)
    assert nav._motion_clear(pose(), 0.3, 0.0, 0.10)


def test_proven_ground_clears_phantom_cliff_and_unknown_track():
    """Gazebo too_narrow t2 / no_route t3: a HIGH CLIFF cell UNDER the robot (impossible while it
    stands level on it) made the unseen ground around it 'risky' and the spawn area behind was never
    observed, so no recovery could move. Ground the robot stood on is now proven."""
    tp, nav = make()
    cls = grid(tp, UNKNOWN)
    cls[cells(tp, 1.0, 6.0, -3.0, 3.0)] = SAFE                       # only seen ahead of the spawn
    cls[cells(tp, 1.9, 2.0, 0.0, 0.1)] = CLIFF                       # phantom cell
    nav.update_map(cls, t=0.0)
    p0 = pose(0.0, 0.0, 0.0)
    nav.mark_proven(p0)                                              # spawn pose
    p1 = pose(1.7, 0.0, 0.0)
    assert not nav.planner.rear_clear(p1.x, p1.y, p1.yaw, 0.3)        # gap between spawn and 1.0 m unseen
    nav.mark_proven(p1)                                              # drove forward onto the phantom
    i, j = int((0.05 - tp.map_origin_y) / tp.resolution), int((1.95 - tp.map_origin_x) / tp.resolution)
    assert nav.planner.classes[i, j] == SAFE                         # phantom cleared for navigation
    assert nav.planner.raw[0][i, j] == CLIFF                         # the mapper's own map is untouched
    # reversing back along its own track (incl. the never-observed spawn area) is now possible
    for x in np.arange(0.0, 1.71, 0.1):
        nav.mark_proven(pose(float(x), 0.0, 0.0))
    assert nav.planner.rear_clear(1.7, 0.0, 0.0, 0.3)


def test_proven_ground_never_clears_a_real_edge_beside_the_robot():
    tp, nav = make()
    cls = grid(tp)
    side = tp.robot_width / 2
    cls[cells(tp, -2.0, 2.0, side - 0.1, 3.0)] = CLIFF               # edge cells under the outer 0.1 m
    nav.update_map(cls, t=0.0)
    nav.mark_proven(pose())
    nav.update_map(cls, t=0.1)                                       # next map applies proven ground
    i, j = int((side - 0.05 - tp.map_origin_y) / tp.resolution), int((0.0 - tp.map_origin_x) / tp.resolution)
    assert nav.planner.proven_seen[i, j]                             # the robot did touch this cell ...
    assert nav.planner.classes[i, j] == CLIFF                        # ... but not deep inside: stays a cliff


def test_tilted_robot_proves_nothing():
    tp, nav = make()
    cls = grid(tp, UNKNOWN)
    nav.update_map(cls, t=0.0)
    tilted = Pose(x=0.0, y=0.0, z=0.0, R=rpy_to_matrix(0.0, math.radians(20.0), 0.0))
    assert not nav.mark_proven(tilted)
    assert nav.planner.proven_seen is None or not nav.planner.proven_seen.any()


def test_slow_reverse_cannot_creep_towards_an_edge_inside_the_band():
    """Gazebo 25 Nov 20:54, too_narrow t1: a recovery reversed slowly at a slight angle to an edge
    and reached 0.000 m. rear_clear (observed ground, no margin) allowed it, and the 0.05 m margin
    levels let it creep closer step by step. Inside the safety band the body may not get closer."""
    tp, nav = make()
    cls = grid(tp)
    cls[cells(tp, -4.0, 4.0, 0.85, 3.0)] = CLIFF                     # edge along the robot's left side
    nav.update_map(cls, t=0.0)
    yaw = math.radians(-14.0)                                        # reversing swings the rear left
    nav.planner.set_body_cells(0.0, 0.0, yaw)
    nav.vel_est, nav.cmd = (-0.12, 0.0), (-0.12, 0.0)
    sc = nav.planner.start_clearance(0.0, 0.0, yaw, drops=True)
    assert 0.0 <= sc < nav.nav['shield_margin']                      # inside the band
    assert nav.planner.rear_clear(0.0, 0.0, yaw, 0.3)                # the old reverse check allowed it
    assert not nav._motion_clear(pose(0.0, 0.0, yaw), -0.12, 0.0, max(0.0, min(0.10, sc)))
    # the recovery therefore does not reverse here
    nav.recovery = dict(phase='reverse', t0=0.0, x0=0.0, y0=0.0, yaw0=yaw, turn=1.0, why='TEST', flipped=False)
    v, w = nav._recover(0.0, pose(0.0, 0.0, yaw))
    assert v >= 0.0
    # reversing parallel to the same edge (not getting closer) stays allowed
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    sc0 = nav.planner.start_clearance(0.0, 0.0, 0.0, drops=True)
    assert nav._motion_clear(pose(), -0.12, 0.0, max(0.0, min(0.10, sc0)))


def test_recoveries_that_cannot_move_end_quickly():
    """Offline too_narrow / no_route on Ubuntu 24.04: six recoveries that could not move were used
    up in 16 s of standing still. Two in a row now end in RECOVERY_LIMIT_REACHED at once."""
    tp, nav = make()
    cls = grid(tp, UNKNOWN)                                          # nothing observed: cannot reverse
    cls[cells(tp, -1.0, 1.0, -0.7, 0.7)] = SAFE
    cls[cells(tp, 0.9, 3.0, -3.0, 3.0)] = CLIFF                      # drop ahead and around
    cls[cells(tp, -3.0, 3.0, 0.75, 3.0)] = CLIFF
    cls[cells(tp, -3.0, 3.0, -3.0, -0.75)] = CLIFF
    nav.update_map(cls, t=0.0)
    p = pose()
    nav.planner.set_body_cells(0.0, 0.0, 0.0)
    t = 0.0
    for _ in range(3):
        nav._start_recovery(t, p, 'TEST - boxed in')
        while nav.recovery is not None:
            nav._recover(t, p)
            t += 0.1
        if nav.state == 'RECOVERY_LIMIT_REACHED':
            break
    assert nav.state == 'RECOVERY_LIMIT_REACHED'
    assert 'could not move' in nav.failure_reason
    assert nav.recoveries < nav.nav['max_recoveries']
    assert nav.tick(t, p) == (0.0, 0.0)

# ------------------------------------------------------------------ measurement helpers
def test_true_geometry_clearance_helper():
    from closed_loop_sim import drop_clearance
    from scenarios import SCENARIOS
    tp = Params.from_nested(cfg())
    clr = drop_clearance(SCENARIOS['cliff_front'], tp)
    assert clr(0.0, 0.0, 0.0) == pytest.approx(3.0 - (0.746 - 0.0), abs=0.08)   # rear edge at x = -3
    assert clr(2.8, 0.0, 0.0) == 0.0                                            # front over the pit
    nr = drop_clearance(SCENARIOS['narrow_route'], tp)
    assert nr(5.0, 1.9, 0.0) == pytest.approx(0.44, abs=0.06)                  # centred on the bridge
