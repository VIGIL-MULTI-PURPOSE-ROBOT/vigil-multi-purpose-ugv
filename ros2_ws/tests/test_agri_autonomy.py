"""Offline tests of the agriculture autonomy supervisor (A1 - A10), closed loop (no ROS, no Gazebo).

    cd ros2_ws && python3 -m pytest -q tests/test_agri_autonomy.py

The REAL crop_row_driver.py runs on the kinematic rover of agri_mission_sim.py; the REAL
agri_autonomy_core.AgriSupervisor reads the same data the node gets in Gazebo (pose, commanded
velocity, wheel speed, IMU yaw rate, the driver's /crop_row/progress and status, the obstacle tracker,
the moisture map from agri_moisture_core) and its commands go back through the driver's
/agriculture/autonomy/command hook. Sensor heartbeats can be switched off to test degradation.
"""
import json
import math

import numpy as np
import pytest

from agri_mission_sim import Obstacle, Sim, linear, static
from agri_autonomy_core import AgriSupervisor
from agri_mission_core import load_yaml
from agri_moisture_core import MoistureField, MoistureMapper, MoistureSensor
from agri_mission_sim import CONFIG


class AutoSim(Sim):
    """Sim + supervisor + moisture survey. `mud` = (x0, x1, row): forward motion is impossible there
    (wheels spin) until the rover has reversed once - a stuck spot a back-off frees."""

    def __init__(self, *a, mud=None, dead=(), moisture=True, **k):
        super().__init__(*a, **k)
        cfg = load_yaml(CONFIG)
        self.sup = AgriSupervisor(dict(cfg.get('autonomy', {})))
        self.dead = set(dead)
        self.mud = mud
        self.freed = False
        self.states, self.decisions, self.commands = [], [], []
        self.reg.subs.setdefault('agriculture/autonomy/command', [])
        self.sup.on_gate(0.0, 'RUNNING')
        self.moist = None
        if moisture:
            mc = dict(cfg['moisture'])
            b = self.node.geo.bounds()
            self.moist = MoistureMapper(b, mc)
            self.sensor = MoistureSensor(MoistureField(b, mc), mc)

    def step(self, dt=0.05):
        x0, y0 = self.x, self.y
        super().step(dt)
        if self.mud and not self.freed:
            mx0, mx1, row = self.mud
            if mx0 <= x0 <= mx1 and abs(y0 - float(self.node.geo.row_y(row - 1))) < 0.5:
                if self.v > 0:                      # wheels turn, the rover does not move forward
                    self.x, self.y = x0, y0
                elif self.v < -0.05:
                    self.freed = True
        t = self.t
        s = self.sup
        s.on_pose(t, self.x, self.y, self.yaw)
        s.on_cmd(t, self.tv if t - self.last_cmd < .6 else 0.0)
        if 'imu' not in self.dead:
            s.on_imu(t, self.w)
        if 'encoders' not in self.dead:
            s.on_encoders(t, self.tv if t - self.last_cmd < .6 else 0.0)
        if int(round(t / dt)) % 2 == 0:
            for name in ('rgb', 'depth', 'lidar'):
                if name not in self.dead:
                    s.on_sensor(t, name)
        if int(round(t / dt)) % 4 == 0:
            self.node.publish_progress()
            if self.reg.pubs['crop_row/progress'].msgs:
                s.on_progress(t, json.loads(self.reg.pubs['crop_row/progress'].msgs[-1].data))
            if self.tracker and 'obstacle_tracker' not in self.dead:
                s.on_obstacles(t, self._obs())
            st = self.reg.pubs['exploration/status'].msgs
            if st:
                s.on_driver(t, json.loads(st[-1].data))
            if self.moist is not None and self.node.done and not getattr(self, 'moist_final', False):
                self.moist_final = True
                self.moist.finish()                  # agri_moisture_map.py does this when the rows are done
                s.on_moisture_status(t, self.moist.status())
            if self.moist is not None and self.node.geo.bounds()[0] <= self.x <= self.node.geo.bounds()[1]:
                val = self.sensor.read(self.x - 0.1, self.y + 0.3)[0]
                self.moist.add(dict(x=self.x - 0.1, y=self.y + 0.3, moisture=val, stamp=t))
                s.on_moisture(t, dict(moisture=val, category=self.moist.map.category(val)))
                s.on_moisture_status(t, self.moist.status())
            s.on_vision(t, dict(ms=40.0))
            st = s.step(t)
            if not self.states or self.states[-1] != st['state']:
                self.states.append(st['state'])
            if not self.decisions or self.decisions[-1] != st['decision']:
                self.decisions.append(st['decision'])
            for c in s.commands:
                self.commands.append((round(t, 1), c))
                self.reg.deliver('agriculture/autonomy/command', type('M', (), {'data': c})())

    def _obs(self):
        obs = []
        for o in self.obstacles:
            p = o.at(self.t)
            if math.hypot(p[0] - self.x, p[1] - self.y) > self.range:
                continue
            ang = np.linspace(0, 2 * math.pi, 9)[:-1]
            pts = np.c_[p[0] + o.r * np.cos(ang), p[1] + o.r * np.sin(ang)]
            v = (o.at(self.t + 0.1) - o.at(self.t - 0.1)) / 0.2
            obs.append(dict(id=o.id, x=float(p[0]), y=float(p[1]), vx=float(v[0]), vy=float(v[1]), cls=o.cls,
                            dynamic=bool(np.hypot(*v) > 0.2),
                            speed=float(np.hypot(*v)), points=pts.tolist(),
                            distance=round(math.hypot(p[0] - self.x, p[1] - self.y), 2)))
        return obs

    def finish(self, extra=6.0):
        """Run the mission, then let the supervisor verify coverage."""
        self.run()
        t_end = self.t + extra
        while self.t < t_end:
            self.step()
        return self

    def status(self):
        return self.sup.status(self.t)


def row_y(k, rows=3):
    return float(Sim(rows=rows).node.geo.row_y(k - 1))


# A1 ------------------------------------------------------------------ complete field coverage
def test_a1_complete_field_coverage_and_mission_complete():
    s = AutoSim(rows=4).finish()
    st = s.status()
    assert st['state'] == 'MISSION_COMPLETE' and st['decision'] == 'COMPLETE MISSION'
    assert st['field']['rows'] == 4 and st['field']['completed_rows'] == [1, 2, 3, 4]
    assert st['field']['remaining_rows'] == [] and st['field']['uncovered'] == []
    assert st['metrics']['field_coverage_pct'] == pytest.approx(100.0)
    assert st['metrics']['collisions'] == 0 and st['metrics']['manual_interventions'] == 0
    assert 'FIELD_ANALYSIS' in s.states and 'ROW_NAVIGATION' in s.states and 'COVERAGE_VERIFICATION' in s.states
    assert st['localization_confidence'] >= 90 and st['autonomy_level'] == 'FULL'


def test_a1_field_understanding_comes_from_the_world():
    s = AutoSim()
    for _ in range(80):
        s.step()
    f = s.status()['field']
    assert f['rows'] == 23 and f['source'] == 'world'
    assert f['row_spacing'] == pytest.approx(1.22, abs=0.01)
    assert f['boundary'][0] == pytest.approx(-13.9) and f['boundary'][1] == pytest.approx(14.4)
    assert len(f['remaining_rows']) == 22                      # everything but the current row


# A2 / A6 / A7 --------------------------------------------------------- static obstacle, interrupted row, rejoin
def test_a2_a6_a7_static_obstacle_interrupts_the_row_which_is_rejoined():
    ob = Obstacle('rock', static(0.0, row_y(2) + 0.2), 0.35, 0.6, 'rock')
    s = AutoSim(rows=3, obstacles=[ob]).finish()
    st = s.status()
    assert 'OBSTACLE_AVOIDANCE' in s.states and 'ROW_REJOIN' in s.states
    i = s.states.index('OBSTACLE_AVOIDANCE')
    assert 'ROW_NAVIGATION' in s.states[i:]                    # back to row navigation afterwards
    assert any(r['row'] == 2 and r['reason'] == 'detour' for r in st['field']['interrupted_rows'])
    assert 'AVOID' in s.decisions and 'RESUME' in s.decisions
    assert st['field']['completed_rows'] == [1, 2, 3] and st['state'] == 'MISSION_COMPLETE'
    assert st['metrics']['collisions'] == 0 and s.min_clear > 0.1
    assert any('interrupted' in e['text'] for e in st['events']) and any('rejoined' in e['text'] for e in st['events'])


# A3 ------------------------------------------------------------------ moving human
def test_a3_moving_human_is_waited_for_then_the_row_continues():
    walker = Obstacle('person', linear(-3.0, row_y(1) - 4.0, 0.0, 0.6, t0=0.0, t1=16.0), 0.3, 1.7, 'person')
    s = AutoSim(rows=3, obstacles=[walker]).finish()
    st = s.status()
    assert 'WAIT TEMPORARILY' in s.decisions
    assert st['state'] == 'MISSION_COMPLETE' and st['metrics']['collisions'] == 0 and s.min_clear > 0.1


# A4 ------------------------------------------------------------------ moving vehicle
def test_a4_moving_vehicle_in_the_headland():
    g = Sim(rows=3).node.geo
    car = Obstacle('car', linear(g.turn_east + 1.0, g.row_y(0) - 6.0, 0.0, 0.8, t0=0.0, t1=30.0), 1.2, 1.6, 'vehicle')
    s = AutoSim(rows=3, obstacles=[car]).finish()
    st = s.status()
    assert st['state'] == 'MISSION_COMPLETE' and st['metrics']['collisions'] == 0


# A5 ------------------------------------------------------------------ animal / object
def test_a5_animal_wandering_into_the_row():
    dog = Obstacle('dog', linear(4.0, row_y(1) + 3.0, 0.0, -0.4, t0=5.0, t1=14.0), 0.35, 0.6, 'animal')
    s = AutoSim(rows=3, obstacles=[dog]).finish()
    st = s.status()
    assert st['state'] == 'MISSION_COMPLETE' and st['metrics']['collisions'] == 0 and s.min_clear > 0.05


# A8 ------------------------------------------------------------------ stuck -> recovery -> resume
def test_a8_no_progress_triggers_a_controlled_back_off_and_the_row_resumes():
    s = AutoSim(rows=2, mud=(-6.0, -5.0, 1))
    s.finish()
    st = s.status()
    assert any(c == 'RECOVER' for _, c in s.commands), s.commands
    assert st['metrics']['no_progress_events'] >= 1 and st['metrics']['recoveries'] >= 1
    assert 'ROW_RECOVERY' in s.states and 'RECOVER' in s.decisions
    assert s.freed                                           # it really reversed (no teleport)
    assert st['state'] == 'MISSION_COMPLETE' and st['field']['completed_rows'] == [1, 2]
    assert any('NO PROGRESS' in e['text'] for e in st['events'])


def test_a8_repeated_failure_is_a_fault_not_an_endless_loop():
    s = AutoSim(rows=2)
    s.freed = True
    s.mud = None
    # permanent stuck spot: forward AND backward motion impossible
    orig = AutoSim.step

    def stuck_step(self, dt=0.05):
        x, y = self.x, self.y
        orig(self, dt)
        if self.t > 5.0:
            self.x, self.y = x, y
    s.step = stuck_step.__get__(s)
    while s.t < 120.0 and s.sup.state != 'FAULT':
        s.step()
    assert s.sup.state == 'FAULT' and s.sup.recoveries_total == int(s.sup.c['recover_limit'])
    assert s.node.auto_hold == 'repeated no-progress'


# A9 ------------------------------------------------------------------ uncovered region detection
def test_a9_blocked_row_is_ended_and_reported_for_revisit_instead_of_holding_for_ever():
    """A crate between rows 1 and 2 near the row end: no detour side is free and it is not a row-end
    obstacle, so the existing driver held for ever. The supervisor ends the row after block_timeout,
    the pattern continues, and the skipped stretch is reported for a revisit."""
    g = Sim(rows=3).node.geo
    crate = Obstacle('crate', static(g.x_end - 0.8, row_y(2)), 0.6, 1.0, 'object')
    s = AutoSim(rows=3, obstacles=[crate]).finish()
    st = s.status()
    assert s.node.done and st['state'] == 'REVISIT_REQUIRED', s.states
    assert ('END_ROW' in [c for _, c in s.commands]) and 'ABORT CURRENT SUBTASK' in s.decisions
    short = [r for r in st['field']['interrupted_rows'] if (r.get('shortfall') or 0) > 0.5]
    assert short and short[0]['row'] == 1
    assert any(g_[0] == 1 for g_ in st['field']['uncovered'])
    assert len(s.node.completed_rows) == 3                  # the lawnmower went on through every row
    assert any('revisit' in e['text'] for e in st['events'])
    assert st['metrics']['collisions'] == 0


# A10 ----------------------------------------------------------------- moisture mapping completion
def test_a10_moisture_map_progress_and_completion():
    s = AutoSim(rows=23)
    seen = []
    while not s.node.done and s.t < 3000:
        s.step()
        m = s.sup.status(s.t)['moisture']
        if m['mapping_pct'] is not None and (not seen or m['mapping_pct'] - seen[-1] >= 10):
            seen.append(m['mapping_pct'])
    s.moist.finish()
    s.sup.on_moisture_status(s.t, s.moist.status())
    for _ in range(40):
        s.step()
    st = s.status()
    assert len(seen) >= 5 and seen == sorted(seen)            # progress rises as the rover drives
    assert st['moisture']['complete'] and st['moisture']['label'] == 'MOISTURE MAP COMPLETE'
    assert st['moisture']['current'] is not None and st['moisture']['unmapped_pct'] < 5
    assert any(e['text'] == 'MOISTURE MAP COMPLETE' for e in st['events'])


# sensor health ------------------------------------------------------- degradation, hold, resume
def test_sensor_loss_degrades_limits_and_holds_only_when_unsafe():
    s = AutoSim(rows=2, dead={'rgb'})
    for _ in range(200):
        s.step()
    assert s.sup.level == 'DEGRADED' and s.node.auto_cap == pytest.approx(0.8)
    s.dead |= {'obstacle_tracker'}
    s.tracker = False
    for _ in range(120):
        s.step()
    assert s.sup.level == 'LIMITED' and s.node.auto_cap == pytest.approx(0.5)
    x = s.x
    s.dead |= {'lidar', 'depth'}
    for _ in range(120):
        s.step()
    assert s.sup.level == 'SAFETY HOLD' and s.node.auto_hold
    hold_x = s.x
    for _ in range(60):
        s.step()
    assert abs(s.x - hold_x) < 0.05                           # really held
    s.dead = set()
    s.tracker = True
    for _ in range(120):
        s.step()
    assert s.sup.level == 'FULL' and s.node.auto_hold is None and s.node.auto_cap == 0.0
    assert s.x > x                                            # moving again
    s.finish()
    assert s.status()['state'] == 'MISSION_COMPLETE'


def test_localization_confidence_is_derived_not_invented():
    s = AutoSim(rows=2)
    for _ in range(200):
        s.step()
    good = s.sup.loc.confidence(s.t)
    f = s.sup.loc.factors(s.t)
    assert good >= 90 and f['imu_available'] and f['encoders_available']
    # wheel slip: encoders say 1.2 m/s, the pose does not move
    for k in range(60):
        t = s.t + 0.05 * (k + 1)
        s.sup.on_pose(t, s.x, s.y, s.yaw)
        s.sup.on_encoders(t, 1.2)
        s.sup.on_imu(t, 0.0)
    assert s.sup.loc.factors(t)['wheel_slip'] > 0.5 and s.sup.loc.confidence(t) < good
    # pose stream stops -> confidence falls towards 0
    assert s.sup.loc.confidence(t + 3.0) == 0
