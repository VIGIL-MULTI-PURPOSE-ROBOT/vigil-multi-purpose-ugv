"""Offline tests of the whole-field row mission and its obstacle avoidance (no ROS, no Gazebo).

    cd ros2_ws && python3 -m pytest -q tests/test_agri_mission.py

The real crop_row_driver.py runs on ROS stand-ins with a kinematic rover (tests/agri_mission_sim.py).
"""
import math
import re
from pathlib import Path

import pytest

from agri_mission_sim import PKG, Obstacle, Sim, linear, static


def completed_rows(sim):
    return [int(m.group(1)) for m in (re.match(r'Row (\d+) complete', t) for t in sim.messages()) if m]


def row_y(sim, k):
    return float(sim.node.geo.row_y(k - 1))


# ------------------------------------------------------------------ field coverage
@pytest.mark.parametrize('rows', [3, 5, 10])
def test_small_fields_run_to_the_last_row(rows):
    s = Sim(rows=rows).run()
    assert s.node.done and 'sweep complete' in s.messages()[-1]
    assert completed_rows(s) == list(range(1, rows + 1))
    assert s.node.coverage.percent() == pytest.approx(100.0)


def test_full_world_rows_come_from_the_world_model():
    s = Sim().run()
    assert s.node.geo.source == 'world' and s.node.rows == 23
    assert completed_rows(s) == list(range(1, 24))
    assert s.node.coverage.percent() == pytest.approx(100.0)
    # lawnmower: every U-turn enters the next row at the planned cross-track
    assert sum('U-turn complete' in m for m in s.messages()) == 22


def test_row_is_complete_only_at_the_real_row_end():
    s = Sim(rows=3)
    geo = s.node.geo
    while not s.node.completed_rows:
        s.step()
    assert s.x >= geo.x_end + geo.row_end_tolerance - 0.1          # passed the last plant (+ tolerance)


def test_no_hard_coded_row_numbers_in_the_mission_code():
    for f in ('crop_row_driver.py', 'agri_mission_core.py'):
        src = (PKG / 'scripts' / f).read_text()
        assert not re.search(r'row\w*\s*==\s*\d', src), f
        assert not re.search(r'rows\s*=\s*\d', src), f


# ------------------------------------------------------------------ obstacles
def test_static_obstacle_detour_returns_to_the_same_row():
    ob = Obstacle('rock', static(0.0, row_y(Sim(rows=3), 2) + 0.2), 0.35, 0.6, 'rock')
    s = Sim(rows=3, obstacles=[ob]).run()
    msgs = s.messages()
    assert any('Obstacle in row 2: detour' in m for m in msgs)
    assert any('Back on row 2' in m for m in msgs)
    assert completed_rows(s) == [1, 2, 3] and s.min_clear > 0.1
    assert s.node.coverage.percent() == pytest.approx(100.0)


def test_human_crossing_the_row_is_yielded_to():
    y1 = row_y(Sim(rows=3), 1)
    # walks across row 1 at 0.6 m/s, reaching the row as the robot gets there
    walker = Obstacle('person', linear(-3.0, y1 - 4.0, 0.0, 0.6, t0=0.0, t1=16.0), 0.3, 1.7, 'person')
    s = Sim(rows=3, obstacles=[walker]).run()
    msgs = s.messages()
    assert any('Moving obstacle in row 1: yielding' in m for m in msgs)
    assert not any('detour' in m for m in msgs)
    assert completed_rows(s) == [1, 2, 3] and s.min_clear > 0.1


def test_moving_vehicle_in_the_headland_is_waited_for():
    g = Sim(rows=3).node.geo
    ymid = (row_y(Sim(rows=3), 1) + row_y(Sim(rows=3), 2)) / 2
    # a vehicle drives north through the east headland when the robot ends row 1
    veh = Obstacle('vehicle', linear(g.turn_east + 0.8, ymid - 9.0, 0.0, 0.8, t0=0.0, t1=60.0), 0.9, 2.0, 'vehicle')
    s = Sim(rows=3, obstacles=[veh]).run()
    assert completed_rows(s) == [1, 2, 3] and s.min_clear > 0.05


def test_obstacle_entering_from_the_side_and_staying():
    y2 = row_y(Sim(rows=3), 2)
    # steps into row 2 from the aisle and stays: yield first, then treated as static and passed
    ob = Obstacle('person', linear(2.0, y2 + 2.0, 0.0, -0.5, t0=40.0, t1=44.0), 0.3, 1.7, 'person')
    s = Sim(rows=3, obstacles=[ob]).run()
    assert completed_rows(s) == [1, 2, 3] and s.min_clear > 0.05
    assert s.node.coverage.percent() == pytest.approx(100.0)


def test_obstacle_leaving_the_row():
    y1 = row_y(Sim(rows=3), 1)
    # stands on row 1 until t=20 s, then walks away sideways
    ob = Obstacle('person', linear(-4.0, y1, 0.0, 0.7, t0=20.0, t1=30.0), 0.3, 1.7, 'person')
    s = Sim(rows=3, obstacles=[ob]).run()
    assert completed_rows(s) == [1, 2, 3] and s.min_clear > 0.05
    assert s.node.coverage.percent() == pytest.approx(100.0)


def test_blocked_headland_uses_a_reverse_u_turn_and_keeps_the_pattern():
    sim = Sim(rows=4)
    g = sim.node.geo
    y1, y2 = row_y(sim, 1), row_y(sim, 2)
    # a parked machine in the east headland beside row 2: not in row 1's way, but in the area
    # the forward U-turn from row 1 to row 2 would sweep
    ob = Obstacle('machine', static(g.turn_east + 1.9, y2 + 0.3), 0.4, 1.5, 'equipment')
    s = Sim(rows=4, obstacles=[ob]).run()
    assert s.node.reverse_turns >= 1
    assert any('Reverse U-turn started' in m for m in s.messages())
    assert completed_rows(s) == [1, 2, 3, 4] and s.min_clear > 0.05


def test_full_world_with_its_parked_tractor_and_workers():
    """The objects in the supplied world that stopped the old mission after row 3."""
    obs = [Obstacle('tractor_a', static(16.7, -9.6), 0.9, 2.5, 'vehicle'),
           Obstacle('tractor_b', static(17.9, -9.6), 0.9, 2.5, 'vehicle'),
           Obstacle('tractor_c', static(19.1, -9.6), 0.9, 2.5, 'vehicle'),
           Obstacle('worker_s_03', static(16.9, -7.4), 0.3, 1.7, 'person'),
           Obstacle('worker_s_02', static(-14.6, 8.05), 0.3, 1.7, 'person'),
           Obstacle('worker_s_01', static(5.2, 4.27), 0.3, 1.7, 'person'),
           Obstacle('worker_s_04', static(-8.05, -5.52), 0.3, 1.7, 'person')]
    s = Sim(obstacles=obs).run(4000)
    assert s.node.done and completed_rows(s) == list(range(1, 24))
    assert s.min_clear > 0.05
    assert s.node.coverage.percent() > 98.5          # only the row ends behind the tractor / worker are short


def test_without_the_tracker_the_original_behaviour_remains():
    s = Sim(rows=3, tracker=False).run()
    assert completed_rows(s) == [1, 2, 3]
