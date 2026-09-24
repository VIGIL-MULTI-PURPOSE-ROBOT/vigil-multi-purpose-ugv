"""Offline tests of the agriculture perception (no ROS, no Gazebo):

    cd ros2_ws && python3 -m pytest -q tests/test_agri_vision.py

The main rule under test: the MAIN CROP IS NEVER CLASSIFIED AS A WEED. Uncertain -> UNKNOWN.
Scenes are ray-cast (tests/synthetic_field.py) with the robot straddling a crop row, exactly as the
existing row mission drives: camera 0.6 m high, 320 x 240, 87 deg.
"""
import hashlib
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'src/agri_ugv/scripts'))

from synthetic_field import CROP_BGR, Scene  # noqa: E402
from agri_vision_core import (CROP, HUMAN, OTHER, UNKNOWN, WEED, Detector, Field, Tracker,  # noqa: E402
                              draw_overlay, load_config)

CFG = load_config()
F = Field(CFG['field'])
ROW = 4                              # the robot straddles crop row 5 (index 4)
RY = float(F.row_y(ROW))
START = -10.5


def run(scene, poses=((START, RY, 0.0),), frames=5, cfg=CFG):
    """Run detector + tracker over the poses; return (all detections, tracker)."""
    det, trk, out = Detector(cfg), Tracker(cfg), []
    t = 0.0
    for x, y, yaw in poses:
        fr, _, _ = scene.render(x, y, yaw)
        for _ in range(frames):
            t += 0.25
            fr.t = t
            dets, masks, _ = det.process(fr)
            for d in dets:
                d['t'] = t
            trk.update(dets, t)
            out += dets
    return out, trk


def truth(scene, x, y):
    """Label of the ground-truth object nearest to (x, y)."""
    best, bd = None, 1e9
    for kind, lab, p in scene.objs:
        c = p['c']
        d = math.hypot(c[0] - x, c[1] - y)
        if d < bd:
            best, bd = lab, d
    return best, bd


def assert_no_crop_as_weed(scene, dets, trk):
    for d in dets:
        if d['cls'] == WEED:
            lab, dist = truth(scene, d['x'], d['y'])
            assert lab == 'WEED', f'WEED detection at ({d["x"]}, {d["y"]}) is really {lab} ({dist:.2f} m)'
            _, off = F.offset(d['x'], d['y'])
            assert abs(off) >= CFG['crop']['row_position_tolerance'], d
    for tr in trk.tracks:
        if tr.cls == WEED:
            lab, _ = truth(scene, tr.x, tr.y)
            assert lab == 'WEED', (tr.id, tr.x, tr.y, lab)


def field(seed=0, rows=(3, 4, 5), x0=-9.6, x1=-3.0, **kw):
    s = Scene(seed)
    for k in rows:
        s.crop_row(float(F.row_y(k)), x0, x1, **kw)
    return s


# ------------------------------------------------------------------ the 10 required scenarios
def test_01_main_crop_alone():
    s = field()
    dets, trk = run(s)
    assert not [d for d in dets if d['cls'] == WEED]
    counts = trk.summary(99, 5)[0]
    assert counts[CROP] >= 10 and counts[WEED] == 0


def test_02_weed_alone():
    s = Scene(2)
    s.weed(-8.0, RY + 0.61)
    s.weed(-7.0, RY - 0.60, size=0.16)
    dets, trk = run(s)
    assert trk.summary(99, 5)[0][WEED] == 2
    assert_no_crop_as_weed(s, dets, trk)


def test_03_crop_surrounded_by_weeds():
    s = field(3)
    for i, x in enumerate(np.arange(-9.0, -4.0, 0.7)):
        s.weed(float(x), RY + (0.61 if i % 2 else -0.60), size=0.1 + 0.02 * (i % 3))
    s.weed(-7.9, RY + 0.18, size=0.1)          # weed INSIDE the crop row band: must not be WEED
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    assert trk.summary(99, 5)[0][WEED] >= 5
    inside = [d for d in dets if abs(d['x'] + 7.9) < 0.15 and abs(d['y'] - (RY + 0.18)) < 0.2]
    assert all(d['cls'] in (CROP, UNKNOWN) for d in inside)


def test_04_dense_crop_row_and_overhanging_leaves():
    s = field(4, spacing=0.2, dense=True)
    # canopy that hangs over the aisle (leaves up in the air, wide)
    s.plant(-7.0, RY, 0.5, 0.2, 0.55, CROP_BGR, 'CROP')
    s.plant(-5.5, RY, 0.5, 0.2, 0.55, CROP_BGR, 'CROP')
    dets, trk = run(s)
    assert not [d for d in dets if d['cls'] == WEED], [d for d in dets if d['cls'] == WEED]
    assert trk.summary(99, 5)[0][CROP] >= 10


def test_05_small_weed():
    s = field(5)
    s.weed(-8.4, RY + 0.61, size=0.05, h=0.04)
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    assert trk.summary(99, 5)[0][WEED] == 1


def test_06_large_weed_and_oversized_plant():
    s = field(6)
    s.weed(-7.5, RY + 0.61, size=0.30, h=0.25)          # large weed (whole plant in the aisle) -> WEED
    s.weed(-6.0, RY - 0.62, size=0.95, h=0.40)          # bigger than any weed -> UNKNOWN, never CROP/WEED
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    weeds = [tr for tr in trk.tracks if tr.cls == WEED]
    assert len(weeds) == 1 and abs(weeds[0].x + 7.5) < 0.3
    big = [d for d in dets if abs(d['x'] + 6.0) < 0.5 and abs(d['y'] - (RY - 0.62)) < 0.5]
    assert big and all(d['cls'] == UNKNOWN for d in big)


@pytest.mark.parametrize('light', [0.35, 0.6, 1.5])
def test_07_different_lighting(light):
    s = field(7)
    s.weed(-8.0, RY + 0.61)
    s.weed(-6.6, RY - 0.60)
    s.light = light
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    c = trk.summary(99, 5)[0]
    assert c[CROP] >= 10 and c[WEED] == 2


def test_08_shadows():
    s = field(8)
    s.weed(-8.0, RY + 0.61)
    s.weed(-6.6, RY - 0.60)
    s.shadow = (-8.5, -6.0)                  # hard shadow across crops and one weed
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    c = trk.summary(99, 5)[0]
    assert c[CROP] >= 10 and c[WEED] == 2


def test_09_bare_soil():
    s = Scene(9)
    dets, trk = run(s)
    assert not [d for d in dets if d['cls'] in (CROP, WEED, HUMAN)]


def test_10_human_between_crop_rows():
    s = field(10)
    s.human(-6.0, RY + 0.61)
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    humans = [tr for tr in trk.tracks if tr.cls == HUMAN]
    assert len(humans) == 1
    h = humans[0]
    assert math.hypot(h.x + 6.0, h.y - (RY + 0.61)) < 0.35           # located in the world
    assert h.info['dist'] == pytest.approx(4.5, abs=0.4)             # distance from the robot
    assert not [d for d in dets if d['cls'] == WEED]
    assert trk.summary(99, 5)[0][CROP] >= 10


# ------------------------------------------------------------------ extra crop-protection cases
def test_crop_coloured_weed_is_found_by_geometry():
    s = field(11)
    s.weed(-7.6, RY + 0.61, size=0.12, h=0.10, bgr=CROP_BGR)   # same colour as cotton, but a low weed
    dets, trk = run(s)
    assert_no_crop_as_weed(s, dets, trk)
    assert trk.summary(99, 5)[0][WEED] == 1


def test_crop_like_plant_between_rows_is_unknown_not_weed():
    s = field(12)
    s.plant(-7.2, RY + 0.62, 0.42, 0.14, 0.14, CROP_BGR, 'CROP')   # volunteer cotton in the aisle
    dets, trk = run(s)
    near = [d for d in dets if abs(d['x'] + 7.2) < 0.3 and abs(d['y'] - (RY + 0.62)) < 0.3]
    assert near and all(d['cls'] == UNKNOWN for d in near), near
    assert not [tr for tr in trk.tracks if tr.cls == WEED]


def test_no_depth_or_pose_never_weed():
    s = field(13)
    s.weed(-8.0, RY + 0.61)
    fr, _, _ = s.render(START, RY, 0.0)
    fr.depth[:] = np.inf                                        # depth missing
    dets, _, _ = Detector(CFG).process(fr)
    assert not [d for d in dets if d['cls'] in (WEED, CROP)]


# ------------------------------------------------------------------ tracker rules
def _det(cls, x, y, t, conf=0.9):
    return dict(cls=cls, sub=cls.lower(), conf=conf, x=x, y=y, dist=1.0, bearing=0, bbox=[0, 0, 1, 1], t=t)


def test_tracker_crop_never_becomes_weed():
    trk = Tracker(CFG)
    for i in range(3):
        trk.update([_det(CROP, 1.0, RY, i)], i)
    for i in range(3, 20):
        trk.update([_det(WEED, 1.0, RY, i)], i)
    assert trk.tracks[0].cls == CROP


def test_tracker_weed_downgraded_by_crop_vote_and_confirmation():
    trk = Tracker(CFG)
    ev = []
    for i in range(2):
        ev += trk.update([_det(WEED, 2.0, RY + 0.6, i)], i)
    assert trk.tracks[0].cls != WEED                           # not yet confirmed (needs 3 frames)
    ev += trk.update([_det(WEED, 2.0, RY + 0.6, 2)], 2)
    assert trk.tracks[0].cls == WEED and any(e['msg'] == 'WEED W1 DETECTED' for e in ev)
    ev += trk.update([_det(CROP, 2.05, RY + 0.6, 3)], 3)
    assert trk.tracks[0].cls in (CROP, UNKNOWN)


def test_tracker_counts_each_object_once():
    s = field(14)
    s.weed(-6.0, RY + 0.61)
    s.weed(-5.2, RY - 0.60)
    s.human(-3.5, RY - 0.61)
    poses = [(x, RY, 0.0) for x in (-10.5, -10.2, -9.9, -9.6, -9.3)]
    dets, trk = run(s, poses=poses, frames=2)
    c = trk.summary(99, 5)[0]
    assert c[HUMAN] == 1 and c[WEED] == 2
    assert_no_crop_as_weed(s, dets, trk)


def test_reverse_direction_row():
    """West-bound rows (the mission alternates direction)."""
    s = field(15, x0=-3.0, x1=4.0)
    s.weed(0.5, RY + 0.61)
    s.human(-0.5, RY - 0.61)
    dets, trk = run(s, poses=((6.0, RY, math.pi),))
    assert_no_crop_as_weed(s, dets, trk)
    c = trk.summary(99, 5)[0]
    assert c[WEED] == 1 and c[HUMAN] == 1


def test_overlay_draws():
    s = field(16)
    s.human(-6.0, RY + 0.61)
    s.weed(-8.0, RY - 0.6)
    fr, _, _ = s.render(START, RY, 0.0)
    dets, masks, _ = Detector(CFG).process(fr)
    img = draw_overlay(fr.bgr, dets, masks, 640, dict(human_active=True, weed_now=True, status_line='ROW 5/23'))
    assert img.shape == (480, 640, 3)


# ------------------------------------------------------------------ navigation stays locked
ROOT = HERE.parent / 'src/agri_ugv'
LOCKED = {  # sha256 of the movement code as it was before the perception/dashboard was added
    'scripts/drive.py': 'a444747e9f9a7285e3812cc21875511a7a40de6c3eed79d0141fc934745b4b01',
    'config/navigation.yaml': '99b3fba2084d971411084b884e0360a9b915b9bb16cb3cfc4f9a9106a0114e3c',
    '../agri_ugv_description/config/controllers.yaml': 'efecb632f158e172596f70cb22d98ac7fc440e307fe89aac74c329428a410533',
    'worlds/field.world': '998885a7394a22ed5901db8abb377eea07ebe83eac6487bd2f06f69eef86c9f4',
}


@pytest.mark.parametrize('rel', sorted(LOCKED))
def test_movement_code_unchanged(rel):
    digest = hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()
    assert digest == LOCKED[rel], f'{rel} changed: the row navigation must stay exactly as it was'


def test_row_control_law_is_the_original_one():
    """crop_row_driver.py was extended (whole field, row ends, avoidance); its row-centring law,
    speeds and U-turn command are still the original ones."""
    src = (ROOT / 'scripts/crop_row_driver.py').read_text()
    for line in ("steering = 1.6 * wrap(heading - yaw) - d * 2.4 * math.atan(error_y / 1.2)",
                 "out.linear.x = float(speed * (1.0 - 0.55 * min(abs(error_y) / self.guard, 1.0)))",
                 "out.angular.z = float(np.clip(steering, -self.speed / 2.0, self.speed / 2.0))",
                 "out.angular.z = self.turn_sign * self.turn_speed / self.turn_radius",
                 "cruise_speed_mps: 1.20", "headland_turn_radius_m: 0.85", "headland_turn_speed_mps: 0.45"):
        assert line in src or line in (ROOT / 'config/agriculture.yaml').read_text(), line


def test_launch_keeps_driver_parameters_and_only_reroutes_through_gate():
    text = (ROOT / 'launch/field.launch.py').read_text()
    assert "node('agri_ugv','crop_row_driver.py',parameters=[{'use_sim_time':True,'row_count':int(LaunchConfiguration('row_count').perform(context)),'config':vision_cfg}]" in text
    assert "gate_remap=[('cmd_vel_nav','crop_row/cmd_vel_request')] if dashboard else []" in text
    gate = (ROOT / 'scripts/row_start_gate.py').read_text()
    assert "self.out.publish(msg)                  # unchanged" in gate
