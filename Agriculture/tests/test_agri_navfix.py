"""Offline tests of the 'robot not moving / not doing the rows' fix (no ROS, no Gazebo).

    cd ros2_ws && python3 -m pytest -q tests/test_agri_navfix.py
"""
import re
import sys
from pathlib import Path

import pytest

from agri_mission_sim import PKG, Obstacle, Sim, static

sys.path.insert(0, str(PKG / 'scripts'))
from agri_autonomy_core import AgriSupervisor  # noqa: E402
from agri_obstacle_core import ObstacleTracker  # noqa: E402


def completed_rows(sim):
    return sorted({int(m.group(1)) for m in (re.match(r'Row (\d+) complete', t) for t in sim.messages()) if m})


def test_both_headland_turn_areas_blocked_backs_off_and_continues():
    s = Sim(rows=5)
    y4 = float(s.node.geo.row_y(3))
    s.obstacles = [Obstacle('tractor', static(16.9, y4), 0.8, 2.5, 'vehicle'),
                   Obstacle('crate', static(13.6, y4 - 0.2), 0.2, 1.0, 'equipment')]
    s.run(t_max=1500)
    msgs = s.messages()
    assert any('both turn areas blocked - backing down row 3' in m for m in msgs)
    assert s.node.done and 'sweep complete' in msgs[-1]
    assert completed_rows(s) == [1, 2, 3, 4, 5]
    assert s.min_clear > 0.0


def test_full_field_all_rows_with_the_east_headland_tractor():
    s = Sim()                                          # 23 rows from the world
    y4 = float(s.node.geo.row_y(3))
    s.obstacles = [Obstacle('tractor', static(17.9, y4 + 0.15), 1.2, 2.5, 'vehicle')]
    s.run(t_max=6000)
    assert s.node.rows == 23 and s.node.done and 'sweep complete' in s.messages()[-1]
    assert completed_rows(s) == list(range(1, 24))
    assert s.min_clear > 0.0


def test_half_seen_static_object_is_not_a_moving_obstacle():
    tr = ObstacleTracker({})
    t = 0.0
    for k in range(30):                                # a 4 m building coming into view piece by piece
        size = min(4.0, 0.6 + 0.15 * k)
        cx = 16.0 + size / 2
        clusters = [dict(x=cx, y=-10.0, height=3.0, n=50, size=size, offsets=[[0, 0]])]
        tracks = tr.update(clusters, t, (14.0, -11.0, 0.0))
        t += 0.2
    assert tracks and not any(x.dynamic for x in tracks)


def test_walking_person_still_moving():
    tr = ObstacleTracker({})
    t = 0.0
    for k in range(20):
        tracks = tr.update([dict(x=5.0, y=-10 + 0.8 * t, height=1.7, n=20, size=0.5, offsets=[[0, 0]])],
                           t, (0.0, -11.0, 0.0))
        t += 0.2
    assert tracks[0].dynamic


def _sup_running():
    s = AgriSupervisor({})
    s.on_gate(0.0, 'RUNNING')
    s.on_progress(0.0, dict(row=1, total_rows=23, phase='row'))
    return s


def test_no_progress_fault_hold_does_not_loop_and_is_retried():
    s = _sup_running()
    t, cmds = 0.0, []
    while t < 120.0:
        for n in ('pose', 'imu', 'encoders', 'rgb', 'depth', 'lidar', 'obstacle_tracker'):
            s.health.beat(n, t)
        s.on_pose(t, -14.5, -13.4, 0.0)                # commanded but not moving
        s.on_cmd(t, 1.0)
        s.step(t)
        cmds += [(round(t, 1), c) for c in s.commands]
        t += 0.25
    holds = [c for c in cmds if c[1].startswith('HOLD')]
    resumes = [c for c in cmds if c[1] == 'RESUME']
    assert len(holds) >= 1
    # no HOLD/RESUME ping-pong: each resume comes >= fault_retry after its hold
    for (th, _), (tr_, _) in zip(holds, resumes):
        assert tr_ - th >= 29.0


def test_gate_has_direct_path_when_nav2_chain_inactive():
    src = (PKG / 'scripts/row_start_gate.py').read_text()
    assert "create_publisher(Twist, 'cmd_vel', 10)" in src and "GetState" in src
    assert "velocity_smoother" in src and "collision_monitor" in src


def test_supervisor_measures_progress_on_the_drive_input():
    src = (PKG / 'scripts/agri_supervisor.py').read_text()
    assert "create_subscription(Twist, 'cmd_vel', lambda m: self.sup.on_cmd" in src


# ------------------------------------------------------------------ full field + dynamic obstacles
import math  # noqa: E402

import numpy as np  # noqa: E402


def walking_worker(t):
    """The field's moving worker (environment.py: worker_d_02 at x = 10, y = -14 + 3 sin(0.2 t)),
    walking across rows 1-3."""
    return np.array([10.0, -14.0 + 3.0 * math.sin(0.2 * t)])


def test_full_field_with_walking_worker_and_tractor_covers_every_row():
    s = Sim()
    y4 = float(s.node.geo.row_y(3))
    s.obstacles = [Obstacle('worker_d_02', walking_worker, 0.3, 1.7, 'human'),
                   Obstacle('tractor', static(17.9, y4 + 0.15), 1.2, 2.5, 'vehicle')]
    s.run(t_max=8000)
    msgs = s.messages()
    assert s.node.rows == 23 and s.node.done and 'sweep complete' in msgs[-1], msgs[-5:]
    assert completed_rows(s) == list(range(1, 24))
    assert any('Moving obstacle in row' in m for m in msgs)          # the walking worker was detected
    assert s.min_clear > 0.0                                          # and never touched
    assert s.node.coverage.percent() > 95.0


def test_pushed_off_the_row_rejoins_the_same_row_and_continues():
    s = Sim(rows=5)
    pushed = False
    while s.t < 2000 and not s.node.done:
        s.step()
        if not pushed and s.node.row == 2 and s.node.phase == 'row' and abs(s.x) < 1.0:
            s.y += 0.55                                               # a worker shoves the rover sideways
            pushed = True
    msgs = s.messages()
    assert pushed
    assert any('off the row line' in m and 'row 3' in m for m in msgs)
    assert any('Back on the row 3 line' in m for m in msgs)
    assert not any('Crop safety stop' in m for m in msgs)
    assert s.node.done and completed_rows(s) == [1, 2, 3, 4, 5]


def test_obstacle_that_just_stopped_walking_is_still_treated_as_moving():
    import json
    from types import SimpleNamespace
    s = Sim(rows=3)
    node = s.node
    ob = dict(id='OB7', x=0.0, y=-13.4, vx=0.0, vy=0.6, dynamic=True, points=[[0.0, -13.4]], cls='human')
    node.on_obstacles(SimpleNamespace(data=json.dumps(dict(obstacles=[ob]))))
    s.t += 2.0                                                        # it pauses (turning round)
    node.on_obstacles(SimpleNamespace(data=json.dumps(dict(obstacles=[dict(ob, vy=0.0, dynamic=False)]))))
    assert node.obstacles[0]['dynamic']
    s.t += 7.0                                                        # standing still for good
    node.on_obstacles(SimpleNamespace(data=json.dumps(dict(obstacles=[dict(ob, vy=0.0, dynamic=False)]))))
    assert not node.obstacles[0]['dynamic']
