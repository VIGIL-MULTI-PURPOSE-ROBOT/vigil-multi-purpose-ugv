#!/usr/bin/env python3
"""Unit tests for the regression-monitor measurement helpers.  python3 -m pytest -q test_metrics.py"""
import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics import PathLength, Progress, RateMeter, real_time_factor, quat_roll_pitch, classify, verdict  # noqa: E402


def test_straight_line_length_at_high_message_rate():
    p = PathLength()
    for k in range(1001):                    # 10 m at 0.5 m/s, sampled at 50 Hz
        t = k * 0.02
        p.add(t, 0.5 * t, 0.0)
    assert p.length == pytest.approx(10.0, abs=0.02)


def test_baseline_overcount_bug_is_gone():
    """Old monitor: distance from the last 0.5 s sample added on every 100 Hz message."""
    p = PathLength()
    for k in range(10001):                   # 100 s, 0.1 m/s, 100 Hz -> 10 m
        t = k * 0.01
        p.add(t, 0.1 * t, 0.0)
    assert p.length == pytest.approx(10.0, abs=0.05)


def test_duplicates_and_out_of_order_samples_are_ignored():
    p = PathLength()
    for k in range(101):
        t = k * 0.1
        p.add(t, t, 0.0)
        p.add(t, t, 0.0)                     # duplicate stamp
        p.add(t - 0.05, t - 5.0, 0.0)        # out of order
    assert p.length == pytest.approx(10.0, abs=0.02)
    assert p.duplicates == 202


def test_stationary_jitter_does_not_accumulate():
    p = PathLength()
    rnd = random.Random(1)
    for k in range(5000):
        p.add(k * 0.01, rnd.gauss(0, 0.002), rnd.gauss(0, 0.002))
    assert p.length < 0.2


def test_teleport_is_not_counted():
    p = PathLength()
    for k in range(51):
        p.add(k * 0.1, k * 0.05, 0.0)        # 2.5 m
    p.add(5.1, 100.0, 0.0)                    # reset/teleport
    for k in range(1, 21):
        p.add(5.1 + k * 0.1, 100.0 + k * 0.05, 0.0)  # +1 m
    assert p.length == pytest.approx(3.5, abs=0.05)
    assert p.glitches == 1


def test_curved_path():
    p = PathLength()
    for k in range(3601):                     # quarter circle r = 2 m
        a = (math.pi / 2) * k / 3600
        p.add(k * 0.01, 2 * math.cos(a), 2 * math.sin(a))
    assert p.length == pytest.approx(math.pi, abs=0.03)


def test_progress_longest_gap():
    g = Progress((10.0, 0.0), step=0.1)
    for k in range(101):
        g.add(k * 0.1, min(k * 0.1, 3.0), 0.0)   # moves to x = 3 by t = 3, then stands for 7 s
    assert g.longest == pytest.approx(7.0, abs=0.15)


def test_rate_and_rtf():
    r = RateMeter()
    for k in range(101):
        r.add(sim=k * 0.1, wall=k * 0.2)
    assert r.hz('sim') == pytest.approx(10.0)
    assert r.hz('wall') == pytest.approx(5.0)
    assert real_time_factor(0.0, 50.0, 0.0, 100.0) == 0.5
    assert real_time_factor(0.0, 1.0, 5.0, 5.0) is None


def test_roll_pitch_are_separate():
    s = math.sin(math.radians(10) / 2)
    roll, pitch = quat_roll_pitch(s, 0.0, 0.0, math.cos(math.radians(10) / 2))
    assert roll == pytest.approx(10.0, abs=1e-6) and abs(pitch) < 1e-6
    roll, pitch = quat_roll_pitch(0.0, s, 0.0, math.cos(math.radians(10) / 2))
    assert pitch == pytest.approx(10.0, abs=1e-6) and abs(roll) < 1e-6


def test_classification_and_verdict():
    assert classify('GOAL_REACHED', 'GOAL_REACHED', None, None, 'GOAL_REACHED') == 'GOAL_REACHED'
    assert classify('TERMINAL', 'GOAL_UNREACHABLE', None, True, 'SAFE_STOP') == 'SAFE_STOP'
    assert classify('TERMINAL', 'GOAL_UNREACHABLE', None, False, 'SAFE_STOP').startswith('NAVIGATION_FAILURE')
    assert classify('SIM_TIME_LIMIT', 'CLIFF_AVOIDANCE', None, None, 'SAFE_STOP').startswith('TIMEOUT')
    assert classify('NO_SIM_CLOCK', None, None, None, 'GOAL_REACHED') == 'SIMULATION_FAILURE'
    assert classify('MONITOR_ERROR', None, None, None, 'GOAL_REACHED') == 'MONITOR_FAILURE'
    assert verdict('TIMEOUT (no terminal state)', 'SAFE_STOP') == 'FAIL'     # a timeout is never PASS
    assert verdict('SAFE_STOP', 'GOAL_REACHED') == 'FAIL'
    assert verdict('GOAL_REACHED', 'GOAL_REACHED') == 'PASS'


def test_clearance_is_a_pass_criterion():
    """Gazebo 25 Nov 18:51: cliff_front t2 reached B with 0.000 m clearance and was reported PASS."""
    assert verdict('GOAL_REACHED', 'GOAL_REACHED', 0.0) == 'FAIL'
    assert verdict('GOAL_REACHED', 'GOAL_REACHED', 0.05) == 'FAIL'
    assert verdict('GOAL_REACHED', 'GOAL_REACHED', 0.10) == 'PASS'
    assert verdict('SAFE_STOP', 'SAFE_STOP', 0.0) == 'FAIL'
    assert verdict('SAFE_STOP', 'SAFE_STOP', 1.4) == 'PASS'
    assert verdict('GOAL_REACHED', 'GOAL_REACHED', None) == 'PASS'      # not a cliff scenario
