"""Offline tests for the VIGIL evaluation tools (no ROS):  python3 -m pytest -q tools/vigil_eval"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import guard_core  # noqa: E402
import loc_metrics as lm  # noqa: E402
import truth_scan  # noqa: E402


def straight(n=101, length=10.0):
    return [(length * i / (n - 1), 0.0, 0.0, 0.0) for i in range(n)]


def samples(gt, est, dt=0.1):
    return [dict(t=i * dt, gt=g, est=e) for i, (g, e) in enumerate(zip(gt, est))]


# ------------------------------------------------------------------ metrics
def test_perfect_estimate_is_zero_error():
    gt = straight()
    r = lm.evaluate(samples(gt, gt))
    assert r['availability'] == 1.0
    assert r['ate_start']['rmse'] == pytest.approx(0.0, abs=1e-9)
    assert r['rpe']['1m']['max'] == pytest.approx(0.0, abs=1e-9)
    assert r['yaw_err_deg']['max'] == pytest.approx(0.0, abs=1e-9)
    assert r['gt_path_length_m'] == pytest.approx(10.0)


def test_constant_offset_in_another_frame_is_removed_by_start_alignment():
    # the estimate lives in a frame rotated 90 deg and shifted (e.g. SLAM map started elsewhere)
    gt = straight()
    est = [(5.0 - 0.0 * x, 3.0 + x, 0.0, math.pi / 2) for (x, _, _, _) in gt]
    r = lm.evaluate(samples(gt, est))
    assert r['ate_start']['max'] == pytest.approx(0.0, abs=1e-9)
    assert r['yaw_err_deg']['max'] == pytest.approx(0.0, abs=1e-6)


def test_linear_drift_is_measured_and_not_hidden():
    gt = straight()
    est = [(x, 0.02 * x, 0.0, 0.0) for (x, _, _, _) in gt]   # 2 cm lateral per metre
    r = lm.evaluate(samples(gt, est))
    assert r['final_error_m'] == pytest.approx(0.2, abs=1e-6)
    assert r['drift_pct'] == pytest.approx(2.0, rel=1e-3)
    assert r['rpe']['1m']['median'] == pytest.approx(0.02, rel=0.05)
    # SE(2) alignment rotates the drift away: it must be smaller - this is why it is never used for acceptance
    assert r['ate_se2']['rmse'] < r['ate_start']['rmse']


def test_heading_error_is_wrapped():
    gt = [(0.0, 0.0, 0.0, math.radians(179.0)), (0.0, 0.0, 0.0, math.radians(179.0))]
    est = [(0.0, 0.0, 0.0, math.radians(179.0)), (0.0, 0.0, 0.0, math.radians(-179.0))]
    r = lm.evaluate(samples(gt, est))
    assert r['yaw_err_deg']['max'] == pytest.approx(2.0, abs=1e-6)


def test_missing_estimate_gives_none_not_zero():
    gt = straight(11)
    r = lm.evaluate(samples(gt, [None] * 11))
    assert r['availability'] == 0.0
    assert r['ate_start'] is None and r['drift_pct'] is None and r['yaw_err_deg'] is None
    assert r['longest_gap_s'] == pytest.approx(1.0)


def test_partial_availability_and_gap():
    gt = straight(21)
    est = [g if (i < 5 or i > 14) else None for i, g in enumerate(gt)]
    r = lm.evaluate(samples(gt, est))
    assert r['availability'] == pytest.approx(11 / 21)
    assert r['longest_gap_s'] == pytest.approx(1.0)


def test_rpe_on_a_curve_ignores_global_rotation():
    th = np.linspace(0, math.pi, 200)
    gt = [(5 * math.sin(t), 5 - 5 * math.cos(t), 0.0, t) for t in th]
    rot = lm.se2(1.0, -2.0, 0.3)
    est = []
    for x, y, _, yaw in gt:
        m = rot @ lm.se2(x, y, yaw)
        est.append((m[0, 2], m[1, 2], 0.0, math.atan2(m[1, 0], m[0, 0])))
    r = lm.evaluate(samples(gt, est))
    assert r['rpe']['2m']['max'] == pytest.approx(0.0, abs=1e-9)
    assert r['ate_start']['max'] == pytest.approx(0.0, abs=1e-9)


# ------------------------------------------------------------------ guard
def test_guard_isolated_clean():
    graph = {'/eval/ground_truth': dict(subs=['/vigil_evaluator', '/vigil_regression_monitor'], pubs=['/ros_gz_bridge']),
             '/sim/ground_truth': dict(subs=['/terrain_mapper'], pubs=[]),
             '/tf': dict(subs=[], pubs=['/robot_state_publisher'])}
    j = guard_core.judge('isolation', graph)
    assert j['verdict'] == 'ISOLATED'
    assert j['dormant'] and '/terrain_mapper' in j['dormant'][0]


def test_guard_flags_rogue_reader_and_republished_truth_and_oracle():
    graph = {'/eval/ground_truth': dict(subs=['/vigil_evaluator', '/cliff_navigator'], pubs=['/b']),
             '/sim/ground_truth': dict(subs=['/terrain_mapper'], pubs=['/relay']),
             '/camera/segmentation/labels_map': dict(subs=['/terrain_mapper'], pubs=['/segmentation_bridge'])}
    j = guard_core.judge('isolation', graph)
    assert j['verdict'] == 'VIOLATION'
    text = ' '.join(j['violations'])
    assert '/cliff_navigator' in text and 'has publisher' in text and 'segmentation' in text


def test_guard_baseline_lists_truth_users():
    graph = {'/sim/ground_truth': dict(subs=['/terrain_mapper', '/cliff_navigator', '/vigil_regression_monitor'],
                                       pubs=['/ros_gz_bridge'])}
    j = guard_core.judge('baseline', graph)
    assert j['verdict'] == 'GROUND_TRUTH_ASSISTED'
    assert any('/cliff_navigator' in u for u in j['truth_users'])
    assert not any('monitor' in u for u in j['truth_users'])


def test_guard_cli_readers_are_allowed_but_contact_readers_are_not():
    graph = {'/eval/ground_truth': dict(subs=['/_ros2cli_12345'], pubs=['/b']),
             '/suspension/contacts/L1': dict(subs=['/sneaky'], pubs=['/b'])}
    j = guard_core.judge('isolation', graph)
    assert j['verdict'] == 'VIOLATION'
    assert all('_ros2cli' not in v for v in j['violations'])


def test_guard_merge_any_violation_wins():
    ok = dict(verdict='ISOLATED', violations=[], warnings=[], truth_users=[], dormant=[])
    bad = dict(verdict='VIOLATION', violations=['x'], warnings=[], truth_users=[], dormant=[])
    assert guard_core.merge([ok, ok, bad, ok])['verdict'] == 'VIOLATION'
    assert guard_core.merge([ok, ok])['verdict'] == 'ISOLATED'
    assert guard_core.merge([])['verdict'] == 'NO_DATA'


# ------------------------------------------------------------------ static scan
def test_truth_scan_finds_the_audited_operational_users():
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    rows = truth_scan.scan(root)
    op = {r['file'] for r in rows if r['category'] == 'operational'}
    for f in ('Rock Terrain/src/vigil_rough_terrain/scripts/terrain_mapper.py',
              'Rock Terrain/src/vigil_rough_terrain/scripts/cliff_navigator.py',
              'Military Search and Rescue/sar_ws/src/vigil_sar/scripts/drive.py',
              'Agriculture/src/agri_ugv/scripts/crop_row_driver.py'):
        assert f in op
    assert 'Rock Terrain/tools/regression/monitor.py' not in op
    assert not any(f.startswith('tools/') for f in op)
