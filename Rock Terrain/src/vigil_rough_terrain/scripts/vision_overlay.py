#!/usr/bin/python3
"""OpenCV overlay for the RGB camera: terrain classes, cliff edges, path, HUD."""
import math

import cv2
import numpy as np

from terrain_core import (CLASS_BGR, CLASS_NAMES, UNKNOWN, SAFE, CLIFF, OBSTACLE, STEEP,
                          POSSIBLE_DROP, CLIMBABLE)

STATE_BGR = {
    'NAVIGATING': (90, 200, 60), 'SAFE PATH FOUND': (90, 200, 60), 'GOAL REACHED': (255, 200, 0),
    'CLIFF DETECTED': (40, 40, 255), 'REPLANNING': (0, 170, 255), 'NO SAFE PATH': (40, 40, 255),
    'EMERGENCY STOP': (40, 40, 255), 'WAITING': (160, 160, 160), 'PATH UPDATED': (0, 200, 255),
    'CLIMBING': (255, 150, 170), 'CLIFF_AVOIDANCE': (0, 120, 255), 'RECOVERY': (0, 200, 255),
    'GOAL_REACHED': (255, 200, 0)}
DIR_BGR = {'CLIMBING': (255, 150, 170), 'CLIFF_AVOIDANCE': (0, 140, 255), 'RECOVERY': (0, 200, 255)}
LUT = np.zeros((256, 3), np.uint8)
for _k, _v in CLASS_BGR.items():
    LUT[_k] = _v
ALPHA = np.zeros(256, np.float32)
ALPHA[[SAFE]] = 0.22
ALPHA[2] = 0.35          # UNEVEN
ALPHA[POSSIBLE_DROP] = 0.45
ALPHA[[STEEP, OBSTACLE]] = 0.5
ALPHA[CLIFF] = 0.6
ALPHA[CLIMBABLE] = 0.35


def _ground_z(mapper, x, y, default):
    i, j = mapper.cell(np.asarray(x), np.asarray(y))
    ok = mapper.inside(i, j)
    z = np.full(np.shape(x), default, np.float32)
    h = mapper.height
    zi = np.where(ok, h[np.clip(i, 0, mapper.n - 1), np.clip(j, 0, mapper.n - 1)], np.nan)
    return np.where(np.isfinite(zi), zi, z)


def _poly(img, cam, pose, pts3, color, thick, scale, dashed=False):
    if pts3 is None or len(pts3) < 2:
        return
    uv, front = cam.project(pts3, pose)
    uv = uv * scale
    h, w = img.shape[:2]
    for k in range(len(uv) - 1):
        if not (front[k] and front[k + 1]):
            continue
        if dashed and k % 4 >= 2:
            continue
        a, b = uv[k], uv[k + 1]
        if max(abs(a[0]), abs(a[1]), abs(b[0]), abs(b[1])) > 4 * max(w, h):
            continue
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), color, thick, cv2.LINE_AA)


def draw(rgb, frame, mapper, pose, cam, path=None, previous_path=None, nav_status=None,
         terrain=None, target=None, scale=2):
    """Return a BGR overlay image.

    Geometry (classes, cliffs, path) comes from the depth frame (H x W). The output is drawn at
    the colour image's resolution when a higher-resolution camera is available (same FOV and
    aspect, e.g. the 1920x1440 HD display camera), otherwise at `scale` x the depth size.
    """
    p = mapper.p
    H, W = frame['valid'].shape
    if rgb is not None and rgb.shape[1] > W * scale:
        S = rgb.shape[1] / float(W)          # HD camera: draw at its native resolution
    else:
        S = float(scale)
    Wo, Ho = int(round(W * S)), int(round(H * S))
    f = S / 2.0                               # text / line size factor (1.0 = 640x480 layout)
    if rgb is None:
        base = np.full((Ho, Wo, 3), 60, np.uint8)
    elif rgb.shape[:2] != (Ho, Wo):
        base = cv2.resize(rgb, (Wo, Ho), interpolation=cv2.INTER_LINEAR)
    else:
        base = rgb
    pts, valid, void = frame['pts'], frame['valid'], frame['void']
    i, j = mapper.cell(pts[..., 0], pts[..., 1])
    ok = valid & mapper.inside(i, j)
    cls = np.full((H, W), UNKNOWN, np.uint8)
    cls[ok] = mapper.classes[i[ok], j[ok]]
    dist = np.hypot(pts[..., 0] - frame['origin'][0], pts[..., 1] - frame['origin'][1])
    cls[valid & (dist > p.cliff_detection_distance + 1.0)] = UNKNOWN
    cls[void] = CLIFF
    # blend in native OpenCV (fast at HD): class colour/alpha are computed at depth
    # resolution and upsampled with nearest-neighbour
    w2 = cv2.resize(ALPHA[cls], (Wo, Ho), interpolation=cv2.INTER_NEAREST)
    col_o = cv2.resize(LUT[cls], (Wo, Ho), interpolation=cv2.INTER_NEAREST)
    out = cv2.blendLinear(np.ascontiguousarray(base), col_o, 1.0 - w2, w2)
    lw = max(1, int(round(2 * f)))
    # cliff / obstacle / steep outlines (found at depth resolution, scaled up)
    for k, col in ((CLIFF, (0, 0, 255)), (OBSTACLE, (220, 0, 220)), (STEEP, (0, 90, 255))):
        m = (cls == k).astype(np.uint8)
        if m.sum() < 8:
            continue
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [(c.astype(np.float32) * S).astype(np.int32) for c in cnts if cv2.contourArea(c) > 15]
        cv2.drawContours(out, cnts, -1, col, lw, cv2.LINE_AA)
        if k == CLIFF and cnts:
            c = max(cnts, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            label = 'HIGH CLIFF'
            if terrain and terrain.get('drop'):
                label += f"  drop {terrain['drop']:.2f} m"
            _label(out, label, (x + int(4 * f), max(int(18 * f), y - int(6 * f))), (0, 0, 255), f)
    # path (projected on the mapped ground)
    gz = pose.ground_z(p)
    for pth, col, th, dashed in ((previous_path, (150, 150, 150), 2, True), (path, (255, 255, 0), 3, False)):
        if pth is not None and len(pth) > 1:
            q = np.asarray(pth, float)[::2]
            z = _ground_z(mapper, q[:, 0], q[:, 1], gz) + 0.03
            _poly(out, cam, pose, np.column_stack([q, z]), col, max(1, int(round(th * f))), S, dashed)
    # robot forward direction (white) and selected direction (arrow)
    yaw = pose.yaw
    fwd = np.array([[pose.x + math.cos(yaw) * d, pose.y + math.sin(yaw) * d] for d in np.linspace(1.2, 3.5, 12)])
    zf = _ground_z(mapper, fwd[:, 0], fwd[:, 1], gz) + 0.02
    _poly(out, cam, pose, np.column_stack([fwd, zf]), (255, 255, 255), max(1, int(round(f))), S, dashed=True)
    if target is not None:
        tz = float(_ground_z(mapper, np.array([target[0]]), np.array([target[1]]), gz)[0]) + 0.05
        uv, front = cam.project(np.array([[target[0], target[1], tz]]), pose)
        if front[0]:
            u, v = (uv[0] * S).astype(int)
            if 0 <= u < Wo and 0 <= v < Ho:
                st = (nav_status or {}).get('state', '')
                col = DIR_BGR.get(st, (255, 255, 0))
                cv2.arrowedLine(out, (Wo // 2, Ho - int(8 * f)), (int(u), int(v)), col,
                                max(1, int(round(3 * f))), cv2.LINE_AA, tipLength=0.08)
                if st == 'CLIMBING':
                    _label(out, f"CLIMBING {nav_status.get('climb_slope', 0):.0f} deg", (int(u) + int(8 * f), int(v)), col, f)
                elif st == 'CLIFF_AVOIDANCE':
                    _label(out, 'SEARCHING ROUTE TO B', (int(u) + int(8 * f), int(v)), col, f)
    _hud(out, nav_status or {}, terrain or {}, f)
    return out


def _label(img, text, org, color, f=1.0):
    fs, th_ = 0.5 * f, max(1, int(round(f)))
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, th_)
    x, y = org
    pad = int(3 * f)
    cv2.rectangle(img, (x - pad, y - th - 2 * pad), (x + tw + pad, y + int(4 * f)), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, fs, color, th_, cv2.LINE_AA)


def _hud(img, nav, ter, f=1.0):
    h, w = img.shape[:2]
    bh = int(28 * f)
    img[:bh] = (img[:bh].astype(np.float32) * 0.35).astype(np.uint8)
    t1, t2 = max(1, int(round(f))), max(1, int(round(2 * f)))
    yb = int(19 * f)
    state = nav.get('state', '---')
    tclass = ter.get('terrain', '---')
    tcol = {'SAFE': (90, 200, 60), 'UNEVEN': (0, 200, 255), 'HIGH CLIFF': (40, 40, 255),
            'POSSIBLE DROP': (0, 120, 255), 'OBSTACLE': (220, 0, 220), 'STEEP': (0, 90, 255),
            'CLIMBABLE': (255, 150, 170)}.get(tclass, (200, 200, 200))
    cv2.putText(img, tclass, (int(8 * f), yb), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * f, tcol, t2, cv2.LINE_AA)
    col = STATE_BGR.get(state, (200, 200, 200))
    (tw, _), _ = cv2.getTextSize(state, cv2.FONT_HERSHEY_SIMPLEX, 0.5 * f, t2)
    cv2.putText(img, state, (w - tw - int(10 * f), yb), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * f, col, t2, cv2.LINE_AA)
    # readout sits between the terrain label and the state, sized so it never overlaps either
    metrics = (f"DROP {ter.get('drop', 0):.2f} m  SLOPE {ter.get('slope_ahead', 0):.0f} deg  "
               f"SAFE {ter.get('safe_distance', 0):.1f} m")
    (lw_, _), _ = cv2.getTextSize(tclass, cv2.FONT_HERSHEY_SIMPLEX, 0.5 * f, t2)
    left, right = int(8 * f) + lw_ + int(16 * f), w - tw - int(26 * f)
    ms = 0.42 * f
    (mw, _), _ = cv2.getTextSize(metrics, cv2.FONT_HERSHEY_SIMPLEX, ms, t1)
    if mw > right - left:
        ms *= max(0.5, (right - left) / float(mw))
        (mw, _), _ = cv2.getTextSize(metrics, cv2.FONT_HERSHEY_SIMPLEX, ms, t1)
    cv2.putText(img, metrics, (left + max(0, (right - left - mw) // 2), yb), cv2.FONT_HERSHEY_SIMPLEX, ms,
                (230, 230, 230), t1, cv2.LINE_AA)
    # why the robot is doing what it does (HILL AHEAD / CLIFF AHEAD / PATH BLOCKED ...)
    why = nav.get('reason')
    if why:
        why = why if len(why) <= 70 else why[:67] + '...'
        _label(img, why, (int(8 * f), h - int(30 * f)), (230, 230, 230), f)
    # legend
    y = h - int(10 * f)
    x = int(8 * f)
    for name, k in (('SAFE', SAFE), ('UNEVEN', 2), ('HILL', CLIMBABLE), ('POSSIBLE DROP', POSSIBLE_DROP),
                    ('STEEP', STEEP), ('CLIFF', CLIFF), ('OBSTACLE', OBSTACLE)):
        cv2.rectangle(img, (x, y - int(9 * f)), (x + int(10 * f), y + int(f)), tuple(int(c) for c in CLASS_BGR[k]), -1)
        cv2.putText(img, name, (x + int(14 * f), y), cv2.FONT_HERSHEY_SIMPLEX, 0.38 * f, (240, 240, 240), t1, cv2.LINE_AA)
        x += int((20 + len(name) * 7.2) * f)
    return CLASS_NAMES
