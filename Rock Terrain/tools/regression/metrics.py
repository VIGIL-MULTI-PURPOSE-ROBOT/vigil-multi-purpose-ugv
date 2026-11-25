#!/usr/bin/env python3
"""Pure-Python measurement helpers for the Gazebo regression monitor (no ROS imports, unit-tested
in test_metrics.py).

Why this exists: the 25 Nov 2026 baseline monitor added the distance from the last 0.5 s trajectory
sample on EVERY ground-truth message (~100 Hz), so one 0.5 s step was counted up to ~50 times
(narrow_route reported 440.88 m). PathLength integrates consecutive, time-ordered samples instead.
"""
import math


class PathLength:
    """Distance travelled from timestamped (t, x, y) samples.

    * samples with a timestamp <= the previous accepted one are DUPLICATES / out of order: ignored
    * a step longer than max_speed * dt + jump_tol is a GLITCH (teleport/reset): not added, counted
    * sub-centimetre jitter is not accumulated: distance is added when the position has moved at
      least min_step from the last anchor (anchor method), so a robot standing still adds ~0 m
    """

    def __init__(self, max_speed=2.0, jump_tol=0.05, min_step=0.01):
        self.max_speed, self.jump_tol, self.min_step = max_speed, jump_tol, min_step
        self.length = 0.0
        self.last_t = None
        self.anchor = None
        self.duplicates = 0
        self.glitches = 0
        self.samples = 0

    def add(self, t, x, y):
        if not (math.isfinite(t) and math.isfinite(x) and math.isfinite(y)):
            self.glitches += 1
            return
        if self.last_t is not None and t <= self.last_t:
            self.duplicates += 1
            return
        if self.anchor is None:
            self.anchor, self.anchor_t, self.last_t = (x, y), t, t
            self.samples += 1
            return
        dt = t - self.last_t
        self.last_t = t
        self.samples += 1
        d = math.hypot(x - self.anchor[0], y - self.anchor[1])
        if d > self.max_speed * max(dt, t - self.anchor_t) + self.jump_tol:
            self.glitches += 1
            self.anchor, self.anchor_t = (x, y), t   # re-anchor after a jump, do not count it
            return
        if d >= self.min_step:
            self.length += d
            self.anchor, self.anchor_t = (x, y), t


class Progress:
    """Longest interval without a net improvement of `step` m in the best distance to the goal."""

    def __init__(self, goal, step=0.1):
        self.goal, self.step = goal, step
        self.best = None
        self.best_t = None
        self.t0 = None
        self.longest = 0.0

    def add(self, t, x, y):
        d = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if self.best is None:
            self.best, self.best_t, self.t0 = d, t, t
            return
        if d < self.best - self.step:
            self.best, self.best_t = d, t
        self.longest = max(self.longest, t - self.best_t)

    def current(self, t):
        return None if self.best_t is None else t - self.best_t


class RateMeter:
    """Message rate over the whole run in sim time and wall time (first..last message)."""

    def __init__(self):
        self.n = 0
        self.sim0 = self.sim1 = self.wall0 = self.wall1 = None

    def add(self, sim, wall):
        self.n += 1
        if self.n == 1:
            self.sim0, self.wall0 = sim, wall
        self.sim1, self.wall1 = sim, wall

    def hz(self, which='sim'):
        if self.n < 3:
            return None
        a, b = (self.sim0, self.sim1) if which == 'sim' else (self.wall0, self.wall1)
        if a is None or b is None or b - a <= 0:
            return None
        return round((self.n - 1) / (b - a), 2)


def real_time_factor(sim0, sim1, wall0, wall1):
    """Simulated seconds per wall second between two /clock observations."""
    if None in (sim0, sim1, wall0, wall1) or wall1 - wall0 <= 0:
        return None
    return round((sim1 - sim0) / (wall1 - wall0), 3)


def quat_roll_pitch(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


TERMINAL_FAILURES = ('GOAL_UNREACHABLE', 'RECOVERY_LIMIT_REACHED', 'SENSOR_TIMEOUT', 'LOCALIZATION_LOST')


def classify(end, raw_state, fell, stopped_ok, expected):
    """Outcome category for the report (independent of PASS/FAIL against expectations).
    end: monitor end condition; raw_state: navigator raw_state; fell: dict or None;
    stopped_ok: True/False/None (robot still after a terminal failure)."""
    if end in ('NO_SIM_CLOCK', 'NO_GROUND_TRUTH'):
        return 'SIMULATION_FAILURE'
    if end == 'NO_NAV_STATUS':
        return 'NAVIGATION_FAILURE (navigator never published status)'
    if end in ('MONITOR_ERROR', 'INTERRUPTED'):
        return 'MONITOR_FAILURE'
    if fell:
        return 'NAVIGATION_FAILURE (fell / rolled over)'
    if raw_state == 'GOAL_REACHED':
        return 'GOAL_REACHED'
    if raw_state in TERMINAL_FAILURES:
        return 'SAFE_STOP' if stopped_ok else 'NAVIGATION_FAILURE (terminal state but robot kept moving)'
    if end == 'SIM_TIME_LIMIT':
        return 'TIMEOUT (no terminal state)'
    if end == 'WALL_TIME_LIMIT':
        return 'TIMEOUT (wall-clock limit; simulation too slow?)'
    return f'UNCLASSIFIED ({end}, {raw_state})'


# Minimum footprint clearance (m) to the TRUE drop edge for a PASS in the cliff scenarios. Same value
# as closed_loop_sim.MIN_CLEARANCE. The monitor measures it on a 0.05 m grid: 0.000 means part of
# the footprint rectangle (outer tyre faces) was beyond the plateau edge, 0.050 means < 0.10 m.
MIN_CLEARANCE = 0.10


def verdict(category, expected_outcome, min_clearance=None, min_required=MIN_CLEARANCE):
    """PASS / FAIL against the scenario expectation. A timeout is never a PASS. Reaching the right
    outcome is not enough: in the cliff scenarios the footprint must also have kept min_required
    from the true drop edge for the whole run (min_clearance None = not a cliff scenario)."""
    if min_clearance is not None and min_clearance < min_required - 1e-9:
        return 'FAIL'
    if expected_outcome == 'GOAL_REACHED':
        return 'PASS' if category == 'GOAL_REACHED' else 'FAIL'
    if expected_outcome in ('SAFE_STOP', 'SEARCHING'):
        return 'PASS' if category == 'SAFE_STOP' else 'FAIL'
    return 'FAIL'
