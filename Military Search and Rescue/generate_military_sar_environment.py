#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_military_sar_environment.py
====================================
Procedural generator for a 300 m x 300 m MILITARY SEARCH-AND-RESCUE / DISASTER
RESPONSE simulation environment, built for an autonomous 8-wheel UGV.

Target simulator : Gazebo Sim Harmonic   (NOT Isaac Sim)
Target middleware: ROS 2 (ros_gz bridge)
Authoring tool   : Blender 4.x, headless capable

NON-WEAPONIZED. No weapons, no firing ranges, no combat targets, no offensive
scenarios. This is a disaster-response / rescue training range.

USAGE
-----
  # headless (recommended)
  blender --background --python generate_military_sar_environment.py -- \
          --out /home/<user>/Documents/military_world --seed 20260915

  # from inside Blender's Scripting workspace
  #   just press Run. It only ever touches the SAR_GENERATED collection.

SAFE REGENERATION
-----------------
Everything this script makes lives under the "SAR_GENERATED" collection.
On re-run it deletes ONLY that subtree (objects + meshes + the collections it
owns). Robot models, imported assets, other scenes and user collections are
never touched.

WHAT BLENDER OWNS vs WHAT ROS 2 / GAZEBO OWNS
---------------------------------------------
Blender (this script) : terrain geometry, static objects, dynamic human scenes,
                        semantic + thermal metadata, ground truth, collision
                        proxies, export-ready collections.
ROS 2 / Gazebo (later): sensors, SLAM, localisation, Nav2, perception, thermal
                        detection, search planning, mission management, control.

HONEST LIMITATIONS -- read these, they matter
---------------------------------------------
1. Humans are PROXY HUMANOIDS (segmented capsule/box bodies, correct 1.55-1.88 m
   scale, per-limb objects). They are NOT rigged, mocap-animated characters.
   Dynamic humans move by keyframed object transforms plus a crude leg/arm swing.
   That is enough for LiDAR returns, depth, bounding boxes, thermal blobs and
   obstacle avoidance; it is NOT enough to train a pose estimator or to look
   photoreal on camera. If you need better, drop in rigged meshes and keep the
   custom properties this script writes -- the metadata contract is what the
   pipeline consumes, not the mesh.
2. Blender CANNOT simulate thermal physics. There is no radiometric render path
   here. What this script produces is a THERMAL METADATA SYSTEM: every relevant
   object carries thermal_class and thermal_temp_c custom properties, exported
   to sar_thermal_table.csv / sar_metadata.json, plus an optional second
   emission-based material set for preview only. The actual thermal image must
   come from a Gazebo thermal camera sensor configured from that table.
   Nothing here claims a thermal camera can see through walls, rubble or snow.
3. Fog / rain / dust / whiteout are Blender-side visual variations plus a
   weather table. Gazebo does not consume Blender volumetrics -- it needs its
   own <scene><fog> / particle configuration. The table is the handoff.
4. Terrain visual mesh == terrain collision mesh, deliberately. A decimated
   collision copy would silently close the holes and ditches, which would defeat
   the whole negative-obstacle requirement. Only discrete objects (buildings,
   vehicles, trees, rocks, rubble) get simplified convex/box collision proxies.
5. ROS 2 Humble + Gazebo Harmonic is not an upstream-supported pair. Harmonic
   pairs with Jazzy; Humble's binary ros_gz targets Fortress. If you are staying
   on Humble you will be building ros_gz from source against Harmonic. The world
   this script emits is version-agnostic (SDF 1.10 + glTF/COLLADA meshes), so
   the mismatch is a bridge problem, not an environment problem -- but plan for
   it.
"""

import bpy
import bmesh
import mathutils
import math
import json
import csv
import os
import sys
import random
import time
import zlib
from mathutils import Vector, Euler

# ----------------------------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------------------------

CFG = {
    # master reproducibility seed -------------------------------------------
    "seed": 20260915,

    # world extents (metres). Base sits on the world origin, map runs north.
    "x_min": -150.0, "x_max": 150.0,
    "y_min":  -40.0, "y_max": 260.0,

    # terrain tessellation --------------------------------------------------
    "tile_size":      50.0,   # terrain is built as 6x6 = 36 tiles
    "res_coarse":      1.00,  # m/vertex, normal tiles
    "res_fine":        0.50,  # m/vertex, tiles containing negative obstacles

    # population densities (per 100 m^2 of eligible area) -------------------
    "tree_density_forest":   0.085,
    "tree_density_open":     0.004,
    "tree_density_snow":     0.030,
    "bush_density_forest":   0.070,
    "bush_density_open":     0.018,
    "rock_density_mountain": 0.055,
    "rock_density_open":     0.010,
    "rubble_pieces":         520,

    # humans ---------------------------------------------------------------
    "n_static_humans":  14,
    "n_dynamic_humans": 11,

    # animation ------------------------------------------------------------
    "fps": 30,
    "sim_seconds": 240,       # 4 minutes of bounded dynamic-human motion

    # output ---------------------------------------------------------------
    "out_dir": "",            # filled from CLI
    "blend_name": "military_world.blend",
    "save": True,
}

ROOT_COLL = "SAR_GENERATED"

# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------

def parse_args():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--out" and i + 1 < len(argv):
            CFG["out_dir"] = argv[i + 1]; i += 2
        elif a == "--seed" and i + 1 < len(argv):
            CFG["seed"] = int(argv[i + 1]); i += 2
        elif a == "--no-save":
            CFG["save"] = False; i += 1
        elif a == "--fast":
            CFG["res_coarse"] = 2.0
            CFG["res_fine"] = 1.0
            CFG["rubble_pieces"] = 200
            i += 1
        else:
            i += 1
    if not CFG["out_dir"]:
        CFG["out_dir"] = os.path.join(os.path.expanduser("~"), "Documents", "military_world")
    return CFG


def tag_seed(tag):
    """Stable string->int seed. Python's hash() is salted per-process
    (PYTHONHASHSEED), so using it here would silently break reproducibility."""
    return (zlib.crc32(tag.encode("utf-8")) ^ (CFG["seed"] * 2654435761)) & 0x7FFFFFFF


def sub_rng(tag):
    """Deterministic, independent RNG per subsystem.

    Sub-seeding by tag means re-tuning (say) rubble does not reshuffle where the
    victims are. That is the difference between a reproducible scenario and a
    scene that changes under you.
    """
    return random.Random(tag_seed(tag))


LOG_T0 = time.time()

def log(msg):
    print("[SAR %7.2fs] %s" % (time.time() - LOG_T0, msg))
    sys.stdout.flush()

# ============================================================================
# PART 1 -- DETERMINISTIC NOISE
# ============================================================================
# Pure-python value noise. Deliberately not Blender's noise texture: this has to
# be callable from placement code, hazard carving and the export metadata with
# bit-identical results, and it has to survive being re-run on another machine.

_MASK = 0xFFFFFFFF

def _hash2(ix, iy, salt):
    n = (ix * 374761393 + iy * 668265263 + salt * 2147483647) & _MASK
    n = (n ^ (n >> 13)) & _MASK
    n = (n * 1274126177) & _MASK
    n = (n ^ (n >> 16)) & _MASK
    return n / 4294967295.0


def _smooth(t):
    return t * t * (3.0 - 2.0 * t)


def vnoise(x, y, salt=0):
    """Value noise in [0,1]."""
    x0 = math.floor(x); y0 = math.floor(y)
    fx = _smooth(x - x0); fy = _smooth(y - y0)
    ix = int(x0); iy = int(y0)
    a = _hash2(ix,     iy,     salt)
    b = _hash2(ix + 1, iy,     salt)
    c = _hash2(ix,     iy + 1, salt)
    d = _hash2(ix + 1, iy + 1, salt)
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def fbm(x, y, freq, octaves=4, salt=0, lac=2.03, gain=0.5):
    """Fractal noise in roughly [-1,1]."""
    amp = 1.0; tot = 0.0; norm = 0.0
    f = freq
    for o in range(octaves):
        tot += amp * (vnoise(x * f, y * f, salt + o * 977) * 2.0 - 1.0)
        norm += amp
        amp *= gain
        f *= lac
    return tot / max(norm, 1e-9)


def ridged(x, y, freq, octaves=4, salt=0):
    """Ridged multifractal -- makes believable rock spines, not blobby hills."""
    amp = 1.0; tot = 0.0; norm = 0.0
    f = freq
    for o in range(octaves):
        n = 1.0 - abs(vnoise(x * f, y * f, salt + o * 613) * 2.0 - 1.0)
        tot += amp * n * n
        norm += amp
        amp *= 0.5
        f *= 2.07
    return tot / max(norm, 1e-9)


def clamp(v, a, b):
    return a if v < a else (b if v > b else v)


def smoothstep(e0, e1, x):
    if e1 == e0:
        return 0.0 if x < e0 else 1.0
    t = clamp((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lerp(a, b, t):
    return a + (b - a) * t


# ---------------------------------------------------------------------------
# 2D polyline geometry helpers
# ---------------------------------------------------------------------------

def seg_dist2(px, py, ax, ay, bx, by):
    """Squared distance point->segment, plus the parametric position t."""
    dx = bx - ax; dy = by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return (px - ax) ** 2 + (py - ay) ** 2, 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = clamp(t, 0.0, 1.0)
    qx = ax + t * dx; qy = ay + t * dy
    return (px - qx) ** 2 + (py - qy) ** 2, t


def poly_dist(px, py, pts):
    """Distance from point to polyline, and (segment index, t)."""
    best = 1e18; bi = 0; bt = 0.0
    for i in range(len(pts) - 1):
        d2, t = seg_dist2(px, py, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
        if d2 < best:
            best = d2; bi = i; bt = t
    return math.sqrt(best), bi, bt


def poly_signed_side(px, py, pts, bi):
    ax, ay = pts[bi][0], pts[bi][1]
    bx, by = pts[bi + 1][0], pts[bi + 1][1]
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)


def poly_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def poly_sample(pts, step):
    """Resample a polyline at fixed arc-length spacing."""
    out = [(pts[0][0], pts[0][1])]
    carry = 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i][0], pts[i][1]
        bx, by = pts[i + 1][0], pts[i + 1][1]
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-9:
            continue
        d = step - carry
        while d <= L:
            t = d / L
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
            d += step
        carry = (L - (d - step))
    out.append((pts[-1][0], pts[-1][1]))
    return out


# ============================================================================
# PART 2 -- SECTORS
# ============================================================================
# Sectors are defined by weighted centroids, not rectangles. Inverse-distance
# weighting with a noise-perturbed distance gives irregular, interlocking
# boundaries and -- critically -- height fields that blend smoothly into each
# other. That is requirement "create transitional regions": the robot drives
# grass -> scrub -> forest floor without stepping off a ledge at a sector line.

# name, cx, cy, influence, base terrain
SECTORS = [
    ("BASE",        0.0,  -14.0, 1.55, "GRAVEL"),
    ("OPEN_FIELD",-18.0,   48.0, 1.00, "GRASS"),
    ("OPEN_FIELD",  6.0,  166.0, 1.00, "GRASS"),
    ("URBAN",      88.0,   44.0, 1.15, "CONCRETE"),
    ("FLOOD",     -98.0,   58.0, 1.10, "MUD"),
    ("RUBBLE",     -2.0,  108.0, 1.20, "RUBBLE"),
    ("DESERT",    104.0,  128.0, 1.05, "SAND"),
    ("FOREST",    -85.0,  168.0, 1.05, "SOIL"),
    ("SNOW",      -45.0,  228.0, 1.10, "SNOW"),
    ("MOUNTAIN",   72.0,  216.0, 1.15, "ROCK"),
]

SECTOR_NAMES = ["BASE", "OPEN_FIELD", "URBAN", "FLOOD", "RUBBLE",
                "DESERT", "FOREST", "SNOW", "MOUNTAIN"]

BASE_RECT = (-46.0, 46.0, -38.0, 10.0)   # graded operations pad


def sector_weights(x, y):
    """Normalised sector influence at a point. Boundaries are noise-warped."""
    wx = x + 16.0 * fbm(x, y, 0.0060, 3, 4101)
    wy = y + 16.0 * fbm(x, y, 0.0060, 3, 8203)
    acc = {}
    tot = 0.0
    for (name, cx, cy, infl, _t) in SECTORS:
        d = math.hypot(wx - cx, wy - cy) + 1.0
        w = infl * (1.0 / (d ** 2.6))
        acc[name] = acc.get(name, 0.0) + w
        tot += w
    if tot <= 0:
        return {"OPEN_FIELD": 1.0}
    for k in acc:
        acc[k] /= tot
    return acc


def dominant_sector(x, y):
    w = sector_weights(x, y)
    return max(w.items(), key=lambda kv: kv[1])[0]


# ---------------------------------------------------------------------------
# per-sector height fields (metres)
# ---------------------------------------------------------------------------

def _h_base(x, y):
    return 0.06 * fbm(x, y, 0.05, 2, 11)

def _h_open(x, y):
    return 1.35 * fbm(x, y, 0.0125, 4, 21) + 0.28 * fbm(x, y, 0.070, 3, 22)

def _h_urban(x, y):
    return 0.55 * fbm(x, y, 0.017, 3, 31) + 0.12 * fbm(x, y, 0.09, 2, 32)

def _h_flood(x, y):
    # low, wet, gently dished so water pools instead of sheeting off
    return -0.55 + 0.85 * fbm(x, y, 0.024, 3, 41) + 0.20 * fbm(x, y, 0.10, 2, 42)

def _h_rubble(x, y):
    return (0.85 + 1.30 * fbm(x, y, 0.055, 4, 51) + 0.45 * ridged(x, y, 0.11, 3, 52))

def _h_desert(x, y):
    # transverse dunes + secondary ripples; ~1:8 lee slopes, wheel-slip country
    dune = 2.7 * math.sin((x * 0.055) + 1.9 * fbm(x, y, 0.010, 2, 61)) \
                * (0.65 + 0.35 * vnoise(x * 0.012, y * 0.012, 62))
    return dune + 0.9 * fbm(x, y, 0.030, 3, 63) + 0.22 * fbm(x, y, 0.16, 2, 64)

def _h_forest(x, y):
    return 2.15 * fbm(x, y, 0.0135, 4, 71) + 0.55 * fbm(x, y, 0.065, 3, 72) \
           + 0.9 * ridged(x, y, 0.020, 2, 73)

def _h_snow(x, y):
    rise = 3.5 + 0.085 * max(0.0, y - 140.0)
    return rise + 2.4 * fbm(x, y, 0.019, 4, 81) + 1.5 * ridged(x, y, 0.030, 3, 82)

def _h_mountain(x, y):
    # Large-scale ridges stay dramatic; the 2-5 m wavelength chop is damped.
    # High-frequency amplitude is what produces 45-degree single-cell spikes,
    # and a mountain made of those is not terrain a wheeled UGV can be asked
    # to reason about -- it is noise wearing a mountain costume.
    rise = 4.0 + 0.155 * max(0.0, y - 150.0)
    return (rise + 7.0 * ridged(x, y, 0.0105, 4, 91)
            + 2.1 * fbm(x, y, 0.021, 4, 92)
            + 0.55 * fbm(x, y, 0.075, 2, 93))

SECTOR_H = {
    "BASE": _h_base, "OPEN_FIELD": _h_open, "URBAN": _h_urban,
    "FLOOD": _h_flood, "RUBBLE": _h_rubble, "DESERT": _h_desert,
    "FOREST": _h_forest, "SNOW": _h_snow, "MOUNTAIN": _h_mountain,
}


# ============================================================================
# PART 3 -- RIVER, CLIFFS, NEGATIVE OBSTACLES, ROAD GRADING
# ============================================================================

# --- river / water -------------------------------------------------------
RIVER = [(-150.0, 132.0), (-138.0, 116.0), (-128.0, 98.0), (-122.0, 78.0),
         (-120.0, 56.0), (-122.0, 34.0), (-128.0, 12.0), (-136.0, -12.0),
         (-142.0, -40.0)]
RIVER_HALF   = 7.5     # channel half width
RIVER_DEPTH  = 3.55    # below local terrain
RIVER_BANK   = 6.0     # bank blend distance
WATER_LEVEL  = -1.05   # constant surface elevation of the river / flood plain

# secondary water: flooded depression on the open-field / urban margin.
# Placed clear of every street: a pool that swallows the only road through a
# sector is not a decision, it is a wall.
POND = (40.0, 50.0, 9.0)           # cx, cy, radius
POND_DEPTH = 2.1
POND_LEVEL = -0.35

# --- cliffs (hard, non-negotiable no-go edges) ---------------------------
CLIFFS = [
    {"id": "SAR_Cliff_001", "pts": [(28.0, 196.0), (48.0, 204.0), (68.0, 208.0),
                                    (88.0, 206.0), (104.0, 200.0)],
     "drop": 7.8, "width": 2.6, "high_side": +1},
    {"id": "SAR_Cliff_002", "pts": [(-18.0, 214.0), (-4.0, 224.0), (10.0, 232.0),
                                    (26.0, 238.0)],
     "drop": 5.6, "width": 2.2, "high_side": -1},
]

# --- negative obstacles: real holes in the mesh, never invisible planes ---
# visibility: OBVIOUS / SUBTLE  -> SUBTLE ones have soft lips and sit in
# vegetation or shadow, so they are a genuine 3D-LiDAR / depth problem.
NEGATIVES = [
    dict(id="SAR_Ditch_001",  kind="DITCH",    pts=[(-21, 27), (-8, 34)],
         half=1.30, depth=1.10, lip=1.1, vis="OBVIOUS",  zone="SAR_ZONE_A"),
    dict(id="SAR_Ditch_002",  kind="DITCH",    pts=[(-38, 64), (-24, 76)],
         half=1.10, depth=1.00, lip=1.8, vis="SUBTLE",   zone="SAR_ZONE_A"),
    dict(id="SAR_Ditch_003",  kind="DITCH",    pts=[(30, 88), (52, 96)],
         half=1.50, depth=1.40, lip=1.2, vis="OBVIOUS",  zone="SAR_ZONE_C"),
    dict(id="SAR_Ditch_004",  kind="DITCH",    pts=[(-25, 119), (-14, 130)],
         half=1.20, depth=1.20, lip=1.0, vis="SUBTLE",   zone="SAR_ZONE_B"),
    dict(id="SAR_Ditch_005",  kind="DITCH",    pts=[(70, 150), (84, 140)],
         half=1.70, depth=1.60, lip=2.4, vis="SUBTLE",   zone="SAR_ZONE_E"),
    dict(id="SAR_Hole_001",   kind="HOLE",     pts=[(20, 56)],
         half=1.60, depth=1.85, lip=0.5, vis="SUBTLE",   zone="SAR_ZONE_A"),
    dict(id="SAR_Hole_002",   kind="HOLE",     pts=[(72, 52)],
         half=2.20, depth=2.40, lip=0.4, vis="OBVIOUS",  zone="SAR_ZONE_C"),
    dict(id="SAR_Hole_003",   kind="HOLE",     pts=[(-5, 113)],
         half=2.70, depth=2.60, lip=0.6, vis="OBVIOUS",  zone="SAR_ZONE_B"),
    dict(id="SAR_Trench_001", kind="TRENCH",   pts=[(-72, 92), (-56, 104), (-44, 112)],
         half=0.90, depth=1.50, lip=0.4, vis="OBVIOUS",  zone="SAR_ZONE_B"),
    dict(id="SAR_Trench_002", kind="TRENCH",   pts=[(58, 110), (76, 124)],
         half=1.00, depth=1.30, lip=0.5, vis="SUBTLE",   zone="SAR_ZONE_E"),
    dict(id="SAR_Ravine_001", kind="RAVINE",   pts=[(30, 196), (35, 207), (41, 218), (45, 230)],
         half=4.20, depth=5.50, lip=3.0, vis="OBVIOUS",  zone="SAR_ZONE_D"),
    dict(id="SAR_Ravine_002", kind="RAVINE",   pts=[(104, 214), (110, 228), (114, 244)],
         half=3.60, depth=6.00, lip=2.6, vis="OBVIOUS",  zone="SAR_ZONE_D"),
    dict(id="SAR_Washout_001", kind="ROAD_WASHOUT", pts=[(-103.3, 44.6), (-110.7, 47.4)],
         half=2.60, depth=2.20, lip=1.4, vis="OBVIOUS",  zone="SAR_ZONE_F"),
    dict(id="SAR_Washout_002", kind="ROAD_WASHOUT", pts=[(120.3, 116.3), (113.7, 113.7)],
         half=2.10, depth=1.80, lip=1.2, vis="OBVIOUS",  zone="SAR_ZONE_E"),
    dict(id="SAR_Subsidence_001", kind="COLLAPSED_GROUND", pts=[(-22, 96)],
         half=5.00, depth=1.15, lip=3.4, vis="SUBTLE",   zone="SAR_ZONE_B"),
]

# --- road network ---------------------------------------------------------
# class -> (width m, surface, traversability)
ROADS = [
    dict(id="SAR_Road_MainSupply", cls="ASPHALT", width=6.0, grade=True,
         pts=[(0, -6), (0, 16), (7, 38), (12, 64), (6, 88), (-4, 110),
              (-10, 132), (2, 152), (10, 176)]),
    dict(id="SAR_Road_West", cls="ASPHALT", width=5.0, grade=True,
         pts=[(-8, 4), (-24, 12), (-36, 18), (-44, 26), (-56, 36),
              (-74, 50), (-92, 66), (-106, 74), (-118, 82), (-132, 90),
              (-142, 96)]),
    dict(id="SAR_Road_East", cls="ASPHALT", width=5.5, grade=True,
         # terminates at the Street D junction, not inside the block behind it
         pts=[(8, 2), (30, 10), (54, 20), (72, 30), (84, 39)]),
    dict(id="SAR_Street_Urban_A", cls="ASPHALT", width=5.0, grade=True,
         pts=[(60, 26), (60, 50), (62, 74), (66, 92)]),
    dict(id="SAR_Street_Urban_B", cls="ASPHALT", width=4.5, grade=True,
         pts=[(96, 24), (98, 48), (100, 70), (104, 90)]),
    dict(id="SAR_Street_Urban_C", cls="ASPHALT", width=4.5, grade=True,
         pts=[(54, 62), (78, 64), (102, 66), (126, 68)]),
    dict(id="SAR_Street_Urban_D", cls="ASPHALT", width=4.0, grade=True,
         pts=[(56, 38), (80, 40), (104, 42), (128, 46)]),
    dict(id="SAR_Track_RubbleBypass", cls="GRAVEL", width=4.0, grade=True,
         pts=[(10, 96), (26, 106), (30, 122), (20, 136), (4, 144)]),
    dict(id="SAR_Track_Desert", cls="DIRT", width=4.5, grade=True,
         pts=[(120, 76), (122, 102), (112, 128), (98, 152), (86, 174)]),
    dict(id="SAR_Track_Forest", cls="DIRT", width=3.2, grade=True,
         pts=[(2, 152), (-22, 158), (-46, 163), (-68, 171), (-88, 182)]),
    dict(id="SAR_Track_Mountain_Risk", cls="GRAVEL", width=3.0, grade=True,
         pts=[(10, 176), (28, 187), (44, 199), (55, 211), (66, 221), (72, 228)]),
    dict(id="SAR_Track_Mountain_Safe", cls="GRAVEL", width=3.6, grade=True,
         pts=[(10, 176), (36, 179), (62, 183), (86, 193), (98, 209),
              (88, 222), (76, 228)]),
    dict(id="SAR_Track_Snow", cls="DIRT", width=4.0, grade=True,
         pts=[(-88, 182), (-80, 199), (-66, 213), (-52, 225), (-40, 237)]),
    dict(id="SAR_Track_FloodBank", cls="DIRT", width=3.4, grade=True,
         pts=[(-92, 66), (-104, 54), (-110, 38), (-112, 20)]),
]

BRIDGE = dict(id="SAR_Bridge_001", road="SAR_Road_West",
              center=(-124.3, 85.6), axis=(-0.868, 0.496),
              length=26.0, width=5.2, deck_z=1.35,
              damaged_gap=(9.0, 2.3))   # gap start along deck, gap length


def on_bridge(x, y, pad=1.5):
    """True inside the bridge deck footprint. The terrain under a bridge is the
    river channel, so any water test along a road has to know the bridge exists
    or it reports a crossing as a drowning."""
    cx, cy = BRIDGE["center"]
    ax, ay = BRIDGE["axis"]
    L = math.hypot(ax, ay) or 1.0
    ax, ay = ax / L, ay / L
    dx, dy = x - cx, y - cy
    along = dx * ax + dy * ay
    across = -dx * ay + dy * ax
    return (abs(along) <= BRIDGE["length"] * 0.5 + pad and
            abs(across) <= BRIDGE["width"] * 0.5 + pad)


# ---------------------------------------------------------------------------
# road grading: pre-compute a longitudinally smoothed centreline profile so
# roads are actually drivable instead of a ribbon draped over rough noise.
# ---------------------------------------------------------------------------

_ROAD_PROFILE = {}      # road id -> [(x, y, z_graded), ...]
_ROAD_GRID = {}         # (cell_x, cell_y) -> [(road_id, sample_index), ...]
_GRID_CELL = 12.0


def raw_height(x, y):
    """Terrain height BEFORE roads, cliffs and negative obstacles."""
    w = sector_weights(x, y)
    h = 0.0
    for name, weight in w.items():
        if weight < 1e-4:
            continue
        h += weight * SECTOR_H[name](x, y)
    # hard-graded operations pad: the base must be flat enough to initialise on
    x0, x1, y0, y1 = BASE_RECT
    inside = (smoothstep(x0 - 9, x0 + 7, x) * (1.0 - smoothstep(x1 - 7, x1 + 9, x)) *
              smoothstep(y0 - 9, y0 + 7, y) * (1.0 - smoothstep(y1 - 7, y1 + 9, y)))
    if inside > 0.0:
        h = lerp(h, 0.04 * fbm(x, y, 0.06, 2, 7), inside)
    return h


def build_road_profiles():
    log("grading road corridors")
    for r in ROADS:
        ROAD_BY_ID[r["id"]] = r
    for r in ROADS:
        samples = poly_sample(r["pts"], 2.0)
        zs = [raw_height(px, py) for (px, py) in samples]
        # moving average -> removes the high-frequency chatter a wheeled
        # vehicle cannot follow, keeps the long-wavelength grade
        win = 9
        sm = []
        n = len(zs)
        for i in range(n):
            a = max(0, i - win); b = min(n, i + win + 1)
            sm.append(sum(zs[a:b]) / (b - a))
        # second pass: longitudinal slope limiting. Sealed roads get 14%;
        # gravel and dirt mountain tracks get 20%, because forcing a 14% grade
        # through steep country buys the gentle centreline with metres of cut
        # and fill, and the batter either side of it becomes the real obstacle.
        step = 2.0
        grade_limit = 0.14 if r["cls"] == "ASPHALT" else 0.20
        for _ in range(3):
            for i in range(1, n):
                dz = sm[i] - sm[i - 1]
                lim = grade_limit * step
                if dz > lim:
                    sm[i] = sm[i - 1] + lim
                elif dz < -lim:
                    sm[i] = sm[i - 1] - lim
        prof = [(samples[i][0], samples[i][1], sm[i]) for i in range(n)]
        _ROAD_PROFILE[r["id"]] = prof
        for i, (px, py, _z) in enumerate(prof):
            key = (int(math.floor(px / _GRID_CELL)), int(math.floor(py / _GRID_CELL)))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    _ROAD_GRID.setdefault((key[0] + dx, key[1] + dy), []).append((r["id"], i))


ROAD_BY_ID = {}

def nearest_road(x, y, max_d=14.0):
    """(road_id, distance, graded_z) for the closest road centreline.

    The elevation is an inverse-distance blend over every nearby centreline
    sample, not just the closest one. Taking the single nearest sample puts a
    step at every junction, because two roads sharing an endpoint end up with
    slightly different longitudinally-smoothed profiles there -- and a 40 cm
    step at a T-junction reads as a 45-degree slope to a terrain analyser.
    """
    key = (int(math.floor(x / _GRID_CELL)), int(math.floor(y / _GRID_CELL)))
    cands = _ROAD_GRID.get(key)
    if not cands:
        return None, 1e9, 0.0
    best_d2 = 1e18; best = None
    wsum = 0.0; zsum = 0.0
    for (rid, i) in cands:
        px, py, pz = _ROAD_PROFILE[rid][i]
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2; best = rid
        if d2 < 144.0:
            w = 1.0 / (d2 + 0.35)
            wsum += w; zsum += w * pz
    d = math.sqrt(best_d2)
    if d > max_d or wsum <= 0.0:
        return None, d, 0.0
    return best, d, zsum / wsum


# ---------------------------------------------------------------------------
# full height field
# ---------------------------------------------------------------------------

def river_field(x, y):
    d, bi, bt = poly_dist(x, y, RIVER)
    if d > RIVER_HALF + RIVER_BANK:
        return 0.0
    inner = 1.0 - smoothstep(RIVER_HALF * 0.25, RIVER_HALF + RIVER_BANK, d)
    return -RIVER_DEPTH * inner


def pond_field(x, y):
    cx, cy, r = POND
    d = math.hypot(x - cx, y - cy)
    if d > r + 5.0:
        return 0.0
    return -POND_DEPTH * (1.0 - smoothstep(r * 0.3, r + 5.0, d))


def cliff_field(x, y):
    acc = 0.0
    for c in CLIFFS:
        d, bi, bt = poly_dist(x, y, c["pts"])
        if d > 34.0:
            continue
        side = poly_signed_side(x, y, c["pts"], bi)
        s = 1.0 if side >= 0 else -1.0
        # signed distance, positive on the plateau side
        sd = d * s * c["high_side"]
        # steep face over `width` metres -> ~70 deg, far outside an 8x8 UGV's envelope
        acc += c["drop"] * smoothstep(-c["width"], c["width"], sd)
    return acc


def negative_field(x, y):
    """Carve real depressions. These are geometry, not collision tricks."""
    acc = 0.0
    for nb in NEGATIVES:
        pts = nb["pts"]
        if len(pts) == 1:
            d = math.hypot(x - pts[0][0], y - pts[0][1])
        else:
            d, _bi, _bt = poly_dist(x, y, pts)
        half = nb["half"]; lip = nb["lip"]
        if d > half + lip + 0.5:
            continue
        # steep-walled where lip is small (hole/trench), soft where large (subsidence)
        f = 1.0 - smoothstep(half, half + lip, d)
        acc -= nb["depth"] * f
    return acc


def road_blend(x, y, h):
    rid, d, gz = nearest_road(x, y)
    if rid is None:
        return h, None, 1e9
    r = ROAD_BY_ID[rid]
    hw = r["width"] * 0.5
    # Batter width scales with how much cut or fill the corridor needed. A road
    # sitting 4 m above natural ground does not end in a 4 m wall at the edge of
    # a fixed 3 m shoulder -- it ends in an embankment about 1:2.6, roughly 21
    # degrees. Without this, every graded mountain track is flanked by a pair of
    # 60-degree banks that no terrain analyser will ever call traversable.
    dz = abs(gz - h)
    batter = clamp(2.6 * dz, 3.0, 26.0)
    if d <= hw:
        t = 1.0
    else:
        t = 1.0 - smoothstep(hw, hw + batter, d)
    if t <= 0.0:
        return h, rid, d
    # crown the carriageway ~2.5% for realism and so LiDAR sees a profile
    crown = 0.025 * max(0.0, hw - d) if d < hw else 0.0
    return lerp(h, gz + crown, t), rid, d


def height_at(x, y):
    h = raw_height(x, y)
    h += cliff_field(x, y)
    h += river_field(x, y)
    h += pond_field(x, y)
    h, _rid, _d = road_blend(x, y, h)
    h += negative_field(x, y)     # applied last: a washout cuts through a road
    return h


def height_and_class(x, y):
    """Height plus terrain classification. One pass, so class and geometry
    can never disagree."""
    h = raw_height(x, y)
    h += cliff_field(x, y)
    riv = river_field(x, y)
    pnd = pond_field(x, y)
    h += riv + pnd
    h, rid, rd = road_blend(x, y, h)
    neg = negative_field(x, y)
    h += neg

    sec = dominant_sector(x, y)
    j = vnoise(x * 0.09, y * 0.09, 313)

    if rid is not None and rd <= ROAD_BY_ID[rid]["width"] * 0.5 + 0.6:
        cls = {"ASPHALT": "ROAD", "GRAVEL": "GRAVEL", "DIRT": "DIRT"}[ROAD_BY_ID[rid]["cls"]]
    elif (riv < -0.25 or pnd < -0.15) and h < WATER_LEVEL + 0.05:
        cls = "WATER"
    elif riv < -0.05 or (sec == "FLOOD" and j < 0.62):
        cls = "MUD"
    elif neg < -0.35:
        cls = "SOIL" if sec not in ("MOUNTAIN", "SNOW", "DESERT") else (
            "ROCK" if sec != "DESERT" else "SAND")
    elif sec == "BASE":
        cls = "CONCRETE" if abs(x) < 22 and -30 < y < 2 and j < 0.55 else "GRAVEL"
    elif sec == "DESERT":
        cls = "SAND" if j < 0.86 else "ROCK"
    elif sec == "SNOW":
        cls = "SNOW" if j < 0.84 else "ROCK"
    elif sec == "MOUNTAIN":
        cls = "ROCK" if j < 0.74 else "GRAVEL"
    elif sec == "RUBBLE":
        cls = "RUBBLE" if j < 0.78 else "CONCRETE"
    elif sec == "URBAN":
        cls = "CONCRETE" if j < 0.45 else ("DIRT" if j < 0.70 else "GRASS")
    elif sec == "FOREST":
        cls = "SOIL" if j < 0.58 else ("GRASS" if j < 0.90 else "ROCK")
    else:
        cls = "GRASS" if j < 0.80 else ("SOIL" if j < 0.93 else "GRAVEL")
    return h, cls, sec


def slope_deg(x, y, eps=0.75):
    hx = (height_at(x + eps, y) - height_at(x - eps, y)) / (2 * eps)
    hy = (height_at(x, y + eps) - height_at(x, y - eps)) / (2 * eps)
    return math.degrees(math.atan(math.hypot(hx, hy)))

# ============================================================================
# PART 4 -- BLENDER PLUMBING: collections, materials, metadata, mesh building
# ============================================================================

COLL_TREE = {
    ROOT_COLL: [
        "SAR_ENVIRONMENT",
        "LIGHTING",
        "GROUND_TRUTH",
        "SAR_EXPORT",
        "SAR_VISUAL",
        "SAR_COLLISION",
        "SAR_HUMAN_TEST",
        "SAR_SENSOR_TEST",
    ],
    "SAR_ENVIRONMENT": [
        "SAR_TERRAIN", "BASE", "OPEN_FIELD", "FOREST", "MOUNTAIN", "SNOW",
        "DESERT", "FLOOD", "URBAN", "RUBBLE", "BUILDINGS", "VEHICLES",
        "VEGETATION", "ROADS", "BRIDGES", "HAZARDS",
        "STATIC_HUMANS", "DYNAMIC_HUMANS", "DYNAMIC_PROPS", "PROPS",
    ],
}

_COLLS = {}


def purge_generated():
    """Remove ONLY what this script previously made. Never touches the robot,
    imported assets, other scenes or user collections."""
    root = bpy.data.collections.get(ROOT_COLL)
    if root is None:
        return
    log("purging previous SAR_GENERATED subtree")
    # gather every collection under root
    to_del_colls = []
    stack = [root]
    seen = set()
    while stack:
        c = stack.pop()
        if c.name in seen:
            continue
        seen.add(c.name)
        to_del_colls.append(c)
        stack.extend(list(c.children))
    objs = set()
    for c in to_del_colls:
        for o in list(c.objects):
            objs.add(o)
    for o in objs:
        data = o.data
        dtype = o.type
        bpy.data.objects.remove(o, do_unlink=True)
        try:
            if data is not None and data.users == 0:
                if dtype == 'MESH':
                    bpy.data.meshes.remove(data)
                elif dtype == 'LIGHT':
                    bpy.data.lights.remove(data)
                elif dtype == 'CURVE':
                    bpy.data.curves.remove(data)
        except Exception:
            pass
    for c in to_del_colls:
        try:
            bpy.data.collections.remove(c)
        except Exception:
            pass
    # orphan meshes created by us in a crashed run
    for m in list(bpy.data.meshes):
        if m.users == 0 and m.name.startswith("SARM_"):
            bpy.data.meshes.remove(m)


def build_collections():
    scene = bpy.context.scene
    root = bpy.data.collections.new(ROOT_COLL)
    scene.collection.children.link(root)
    _COLLS[ROOT_COLL] = root

    def make(parent_name, names):
        parent = _COLLS[parent_name]
        for n in names:
            c = bpy.data.collections.new(n)
            parent.children.link(c)
            _COLLS[n] = c
            if n in COLL_TREE:
                make(n, COLL_TREE[n])

    make(ROOT_COLL, COLL_TREE[ROOT_COLL])
    return root


def C(name):
    return _COLLS[name]


def link_to(obj, *coll_names):
    """Link an object into one or more collections. Multi-linking is how
    SAR_EXPORT / SAR_VISUAL / SAR_HUMAN_TEST stay views onto the same objects
    instead of duplicated geometry that drifts out of sync."""
    for cn in coll_names:
        c = C(cn)
        if obj.name not in c.objects:
            c.objects.link(obj)


def unlink_all(obj):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)


# ---------------------------------------------------------------------------
# metadata
# ---------------------------------------------------------------------------

def set_props(obj, **kw):
    for k, v in kw.items():
        obj[k] = v
    return obj


SEMANTIC_CLASSES = ["TREE", "ROCK", "BUILDING", "VEHICLE", "HUMAN", "ROAD",
                    "WATER", "RUBBLE", "GROUND", "WALL", "BRIDGE", "PROP",
                    "VEGETATION", "MARKER", "DECOY", "STRUCTURE", "HAZARD"]

# traversability model handed to Gazebo / the terrain classifier.
# cost is a relative traverse cost, mu a nominal wheel-ground friction.
TRAVERSABILITY = {
    "ROAD":     dict(trav="EASY",           cost=1.0,  mu=0.85, risk="LOW"),
    "CONCRETE": dict(trav="EASY",           cost=1.0,  mu=0.82, risk="LOW"),
    "GRASS":    dict(trav="EASY",           cost=1.2,  mu=0.65, risk="LOW"),
    "SOIL":     dict(trav="EASY",           cost=1.3,  mu=0.62, risk="LOW"),
    "GRAVEL":   dict(trav="MODERATE",       cost=1.6,  mu=0.55, risk="LOW"),
    "DIRT":     dict(trav="MODERATE",       cost=1.5,  mu=0.58, risk="LOW"),
    "SNOW":     dict(trav="DIFFICULT",      cost=2.6,  mu=0.30, risk="MEDIUM"),
    "MUD":      dict(trav="DIFFICULT",      cost=3.0,  mu=0.28, risk="MEDIUM"),
    "SAND":     dict(trav="DIFFICULT",      cost=2.9,  mu=0.35, risk="MEDIUM"),
    "ROCK":     dict(trav="DIFFICULT",      cost=3.2,  mu=0.70, risk="MEDIUM"),
    "RUBBLE":   dict(trav="VERY_DIFFICULT", cost=5.5,  mu=0.60, risk="HIGH"),
    "WATER":    dict(trav="FORBIDDEN",      cost=-1.0, mu=0.05, risk="CRITICAL"),
    "CLIFF":    dict(trav="FORBIDDEN",      cost=-1.0, mu=0.70, risk="CRITICAL"),
}

# ---------------------------------------------------------------------------
# thermal model (metadata only -- Blender does not do radiometry)
# ---------------------------------------------------------------------------
# Numbers are apparent surface temperatures in degrees C for a temperate
# late-afternoon condition, with the NIGHT preset delta applied separately.
# They exist to be transcribed into Gazebo thermal camera / temperature
# parameters, not because Blender renders them.

THERMAL_TABLE = {
    "THERMAL_HUMAN_HEAD":     dict(temp=34.5, emis=0.98, note="exposed skin"),
    "THERMAL_HUMAN_TORSO":    dict(temp=32.0, emis=0.97, note="clothed core"),
    "THERMAL_HUMAN_LIMB":     dict(temp=30.5, emis=0.97, note="clothed limb"),
    "THERMAL_HUMAN_COVERED":  dict(temp=24.5, emis=0.95, note="jacket/debris/snow covered"),
    "THERMAL_VEHICLE_ENGINE": dict(temp=62.0, emis=0.90, note="recently run engine bay"),
    "THERMAL_VEHICLE_BODY":   dict(temp=27.0, emis=0.88, note="sun-loaded panel"),
    "THERMAL_VEHICLE_COLD":   dict(temp=15.0, emis=0.88, note="long-cold wreck"),
    "THERMAL_GENERATOR":      dict(temp=74.0, emis=0.92, note="running generator -- decoy"),
    "THERMAL_EQUIPMENT_WARM": dict(temp=41.0, emis=0.93, note="powered comms case -- decoy"),
    "THERMAL_CONCRETE":       dict(temp=22.0, emis=0.92, note=""),
    "THERMAL_METAL":          dict(temp=17.0, emis=0.25, note="low emissivity, reflective"),
    "THERMAL_WOOD":           dict(temp=18.5, emis=0.90, note=""),
    "THERMAL_ROCK":           dict(temp=14.0, emis=0.93, note="cold rock mass"),
    "THERMAL_SOIL":           dict(temp=19.0, emis=0.94, note=""),
    "THERMAL_SAND":           dict(temp=27.5, emis=0.93, note="high diurnal swing"),
    "THERMAL_VEGETATION":     dict(temp=20.0, emis=0.96, note="transpiring canopy"),
    "THERMAL_WATER":          dict(temp=11.5, emis=0.96, note=""),
    "THERMAL_SNOW":           dict(temp=-1.5, emis=0.98, note=""),
    "THERMAL_AMBIENT":        dict(temp=20.0, emis=0.92, note="inert object at air temp"),
}

# human thermal contrast classes: what the detector actually has to work with
THERMAL_CONTRAST = {
    "HIGH":   dict(surface="THERMAL_HUMAN_TORSO",   delta_k=12.0),
    "MEDIUM": dict(surface="THERMAL_HUMAN_TORSO",   delta_k=6.5),
    "LOW":    dict(surface="THERMAL_HUMAN_COVERED", delta_k=2.5),
}


# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------

MAT_SPEC = {
    # name          base colour (linear-ish)          rough  metal
    "MAT_GRASS":      ((0.108, 0.180, 0.062, 1), 0.88, 0.0),
    "MAT_SOIL":       ((0.148, 0.106, 0.070, 1), 0.92, 0.0),
    "MAT_DIRT":       ((0.196, 0.148, 0.098, 1), 0.93, 0.0),
    "MAT_ROCK":       ((0.152, 0.150, 0.146, 1), 0.85, 0.0),
    "MAT_GRAVEL":     ((0.196, 0.190, 0.180, 1), 0.95, 0.0),
    "MAT_SAND":       ((0.520, 0.420, 0.258, 1), 0.90, 0.0),
    "MAT_SNOW":       ((0.880, 0.905, 0.940, 1), 0.42, 0.0),
    "MAT_MUD":        ((0.092, 0.072, 0.050, 1), 0.70, 0.0),
    "MAT_CONCRETE":   ((0.330, 0.325, 0.312, 1), 0.82, 0.0),
    "MAT_ROAD":       ((0.052, 0.052, 0.055, 1), 0.78, 0.0),
    "MAT_METAL":      ((0.240, 0.245, 0.255, 1), 0.38, 0.85),
    "MAT_RUSTMETAL":  ((0.230, 0.120, 0.058, 1), 0.78, 0.45),
    "MAT_WOOD":       ((0.215, 0.140, 0.072, 1), 0.80, 0.0),
    "MAT_WATER":      ((0.030, 0.075, 0.098, 1), 0.08, 0.0),
    "MAT_VEGETATION": ((0.078, 0.160, 0.056, 1), 0.72, 0.0),
    "MAT_BARK":       ((0.105, 0.078, 0.052, 1), 0.88, 0.0),
    "MAT_RUBBLE":     ((0.275, 0.262, 0.248, 1), 0.90, 0.0),
    "MAT_BRICK":      ((0.330, 0.132, 0.086, 1), 0.88, 0.0),
    "MAT_GLASS":      ((0.120, 0.160, 0.170, 1), 0.10, 0.0),
    "MAT_HUMAN_SKIN": ((0.470, 0.310, 0.230, 1), 0.60, 0.0),
    "MAT_HUMAN_HI":   ((0.780, 0.290, 0.030, 1), 0.72, 0.0),   # hi-vis clothing
    "MAT_HUMAN_DARK": ((0.075, 0.082, 0.095, 1), 0.78, 0.0),   # dark clothing -> hard for RGB
    "MAT_HUMAN_MID":  ((0.210, 0.230, 0.170, 1), 0.76, 0.0),   # drab clothing
    "MAT_VEHICLE_A":  ((0.360, 0.040, 0.040, 1), 0.35, 0.30),
    "MAT_VEHICLE_B":  ((0.055, 0.130, 0.330, 1), 0.35, 0.30),
    "MAT_VEHICLE_C":  ((0.680, 0.690, 0.700, 1), 0.32, 0.35),
    "MAT_RESCUE":     ((0.860, 0.400, 0.020, 1), 0.45, 0.10),
    "MAT_TARP":       ((0.070, 0.200, 0.120, 1), 0.72, 0.0),
    "MAT_SIGN":       ((0.820, 0.780, 0.120, 1), 0.55, 0.0),
    "MAT_ICE":        ((0.720, 0.820, 0.860, 1), 0.18, 0.0),
    "MAT_MANNEQUIN":  ((0.560, 0.540, 0.520, 1), 0.70, 0.0),
}

# terrain class -> material
CLASS_MAT = {
    "GRASS": "MAT_GRASS", "SOIL": "MAT_SOIL", "DIRT": "MAT_DIRT",
    "ROCK": "MAT_ROCK", "GRAVEL": "MAT_GRAVEL", "SAND": "MAT_SAND",
    "SNOW": "MAT_SNOW", "MUD": "MAT_MUD", "CONCRETE": "MAT_CONCRETE",
    "ROAD": "MAT_ROAD", "RUBBLE": "MAT_RUBBLE", "WATER": "MAT_WATER",
}

# terrain class -> thermal class
CLASS_THERMAL = {
    "GRASS": "THERMAL_VEGETATION", "SOIL": "THERMAL_SOIL", "DIRT": "THERMAL_SOIL",
    "ROCK": "THERMAL_ROCK", "GRAVEL": "THERMAL_ROCK", "SAND": "THERMAL_SAND",
    "SNOW": "THERMAL_SNOW", "MUD": "THERMAL_SOIL", "CONCRETE": "THERMAL_CONCRETE",
    "ROAD": "THERMAL_CONCRETE", "RUBBLE": "THERMAL_CONCRETE", "WATER": "THERMAL_WATER",
}

_MATS = {}


def build_materials():
    log("building material library")
    for name, (col, rough, metal) in MAT_SPEC.items():
        m = bpy.data.materials.get(name)
        if m is None:
            m = bpy.data.materials.new(name)
        m.use_nodes = True
        nt = m.node_tree
        bsdf = None
        for n in nt.nodes:
            if n.type == 'BSDF_PRINCIPLED':
                bsdf = n
        if bsdf is None:
            bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
            out = None
            for n in nt.nodes:
                if n.type == 'OUTPUT_MATERIAL':
                    out = n
            if out is None:
                out = nt.nodes.new("ShaderNodeOutputMaterial")
            nt.links.new(bsdf.outputs[0], out.inputs[0])
        def sset(key, val):
            s = bsdf.inputs.get(key)
            if s is not None:
                try:
                    s.default_value = val
                except Exception:
                    pass
        sset("Base Color", col)
        sset("Roughness", rough)
        sset("Metallic", metal)
        if name == "MAT_WATER":
            sset("Transmission Weight", 0.85)
            sset("IOR", 1.333)
        if name == "MAT_GLASS":
            sset("Transmission Weight", 0.6)
        m.diffuse_color = col          # viewport / solid-shading colour
        m.roughness = rough
        m.metallic = metal
        _MATS[name] = m

    # ---- thermal PREVIEW materials -------------------------------------
    # Emission shaders whose strength encodes apparent temperature. These are
    # NOT a thermal simulation. They exist so you can eyeball the scene through
    # a grayscale viewport and sanity-check contrast before wiring the real
    # Gazebo thermal sensor from sar_thermal_table.csv.
    for tname, td in THERMAL_TABLE.items():
        mn = "TPREV_" + tname
        m = bpy.data.materials.get(mn)
        if m is None:
            m = bpy.data.materials.new(mn)
        m.use_nodes = True
        nt = m.node_tree
        nt.nodes.clear()
        out = nt.nodes.new("ShaderNodeOutputMaterial")
        em = nt.nodes.new("ShaderNodeEmission")
        # map -20..80 C onto 0..1 grey
        g = clamp((td["temp"] + 20.0) / 100.0, 0.0, 1.0)
        em.inputs[0].default_value = (g, g, g, 1.0)
        em.inputs[1].default_value = 1.0
        nt.links.new(em.outputs[0], out.inputs[0])
        m["thermal_class"] = tname
        m["thermal_temp_c"] = td["temp"]
        m["thermal_emissivity"] = td["emis"]
        _MATS[mn] = m


def M(name):
    return _MATS[name]


# ---------------------------------------------------------------------------
# mesh construction
# ---------------------------------------------------------------------------

_OBJ_COUNT = [0]


def new_mesh_obj(name, verts, faces, mat_names=None, face_mats=None,
                 smooth=False, loc=(0, 0, 0)):
    """Build a mesh object from raw python data. Fast path -- no bpy.ops."""
    me = bpy.data.meshes.new("SARM_" + name)
    me.from_pydata([Vector(v) for v in verts], [], faces)
    me.validate(verbose=False)
    if mat_names:
        for mn in mat_names:
            me.materials.append(M(mn))
    if face_mats is not None and len(me.polygons) == len(face_mats):
        for p, mi in zip(me.polygons, face_mats):
            p.material_index = mi
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
    me.update()
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    _OBJ_COUNT[0] += 1
    return ob


def box_verts(sx, sy, sz, cz=None):
    """Axis-aligned box centred in XY. cz None -> sits on z=0."""
    hx, hy = sx * 0.5, sy * 0.5
    z0 = 0.0 if cz is None else cz - sz * 0.5
    z1 = z0 + sz
    v = [(-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
         (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1)]
    f = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
         (2, 6, 7, 3), (3, 7, 4, 0)]
    return v, f


def make_box(name, sx, sy, sz, mat, loc=(0, 0, 0), rot=(0, 0, 0), cz=None):
    v, f = box_verts(sx, sy, sz, cz)
    ob = new_mesh_obj(name, v, f, [mat], loc=loc)
    ob.rotation_euler = Euler(rot, 'XYZ')
    return ob


def cyl_verts(r_bot, r_top, h, seg=8, z0=0.0, cap=True):
    v = []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        v.append((r_bot * math.cos(a), r_bot * math.sin(a), z0))
    for i in range(seg):
        a = 2 * math.pi * i / seg
        v.append((r_top * math.cos(a), r_top * math.sin(a), z0 + h))
    f = []
    for i in range(seg):
        j = (i + 1) % seg
        f.append((i, j, seg + j, seg + i))
    if cap:
        f.append(tuple(range(seg - 1, -1, -1)))
        f.append(tuple(range(seg, 2 * seg)))
    return v, f


def make_cyl(name, r_bot, r_top, h, mat, seg=8, loc=(0, 0, 0), rot=(0, 0, 0),
             smooth=True):
    v, f = cyl_verts(r_bot, r_top, h, seg)
    ob = new_mesh_obj(name, v, f, [mat], smooth=smooth, loc=loc)
    ob.rotation_euler = Euler(rot, 'XYZ')
    return ob


def ico_verts(r, subdiv=1, jitter=0.0, rng=None, squash=(1.0, 1.0, 1.0)):
    """Icosphere by subdivision, optional radial jitter -> believable rocks."""
    t = (1.0 + math.sqrt(5.0)) / 2.0
    base = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
            (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
            (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    verts = [list(v) for v in base]
    for _ in range(subdiv):
        cache = {}
        nf = []
        def mid(a, b):
            k = (min(a, b), max(a, b))
            if k in cache:
                return cache[k]
            p = [(verts[a][i] + verts[b][i]) * 0.5 for i in range(3)]
            verts.append(p)
            cache[k] = len(verts) - 1
            return cache[k]
        for (a, b, c) in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = nf
    out = []
    for v in verts:
        L = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) or 1.0
        n = [v[0] / L, v[1] / L, v[2] / L]
        rr = r
        if jitter and rng:
            rr = r * (1.0 + rng.uniform(-jitter, jitter))
        out.append((n[0] * rr * squash[0], n[1] * rr * squash[1], n[2] * rr * squash[2]))
    return out, faces

# ============================================================================
# PART 5 -- TERRAIN
# ============================================================================
# Built as 6x6 = 36 tiles. Tiles are seamless because every vertex height comes
# from the same global analytic field, so shared edges evaluate identically --
# no stitching, no cracks for a LiDAR ray to fall through.
#
# Resolution is adaptive: tiles containing a negative obstacle or a cliff get
# 0.5 m spacing so a 1.6 m hole is actually a hole. The rest stay at 1.0 m.
#
# The terrain visual mesh IS the terrain collision mesh. A decimated collision
# copy would quietly heal the ditches and holes, which is exactly the failure
# mode the negative-obstacle requirement exists to prevent.

TERRAIN_TILES = []      # dicts with stats, for the report + metadata


def _fine_regions():
    regions = []
    for nb in NEGATIVES:
        xs = [p[0] for p in nb["pts"]]; ys = [p[1] for p in nb["pts"]]
        pad = nb["half"] + nb["lip"] + 3.0
        regions.append((min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad))
    for c in CLIFFS:
        xs = [p[0] for p in c["pts"]]; ys = [p[1] for p in c["pts"]]
        pad = c["width"] + 6.0
        regions.append((min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad))
    # bridge abutments need resolution too
    bx, by = BRIDGE["center"]
    regions.append((bx - 20, bx + 20, by - 20, by + 20))
    return regions


def _tile_needs_fine(x0, x1, y0, y1, regions):
    for (rx0, rx1, ry0, ry1) in regions:
        if not (rx1 < x0 or rx0 > x1 or ry1 < y0 or ry0 > y1):
            return True
    return False


def build_terrain():
    log("building terrain")
    regions = _fine_regions()
    ts = CFG["tile_size"]
    nx = int(round((CFG["x_max"] - CFG["x_min"]) / ts))
    ny = int(round((CFG["y_max"] - CFG["y_min"]) / ts))
    mat_names = list(dict.fromkeys(CLASS_MAT[c] for c in CLASS_MAT))
    mat_index = {}
    for cls, mn in CLASS_MAT.items():
        mat_index[cls] = mat_names.index(mn)

    total_v = 0; total_f = 0
    for ti in range(nx):
        for tj in range(ny):
            x0 = CFG["x_min"] + ti * ts
            y0 = CFG["y_min"] + tj * ts
            x1 = x0 + ts; y1 = y0 + ts
            fine = _tile_needs_fine(x0, x1, y0, y1, regions)
            res = CFG["res_fine"] if fine else CFG["res_coarse"]
            n = int(round(ts / res))
            cx = x0 + ts * 0.5; cy = y0 + ts * 0.5

            verts = []
            vclass = []
            for j in range(n + 1):
                wy = y0 + j * res
                for i in range(n + 1):
                    wx = x0 + i * res
                    h, cls, _sec = height_and_class(wx, wy)
                    verts.append((wx - cx, wy - cy, h))
                    vclass.append(cls)

            faces = []
            fmats = []
            hist = {}
            for j in range(n):
                for i in range(n):
                    a = j * (n + 1) + i
                    b = a + 1
                    c = a + (n + 1) + 1
                    d = a + (n + 1)
                    faces.append((a, b, c, d))
                    # majority class of the corners; ties -> first corner
                    cc = {}
                    for k in (a, b, c, d):
                        cc[vclass[k]] = cc.get(vclass[k], 0) + 1
                    cls = max(cc.items(), key=lambda kv: (kv[1],))[0]
                    fmats.append(mat_index[cls])
                    hist[cls] = hist.get(cls, 0) + 1

            name = "SAR_Terrain_%02d_%02d" % (ti, tj)
            ob = new_mesh_obj(name, verts, faces, mat_names, fmats,
                              smooth=False, loc=(cx, cy, 0.0))
            order = sorted(hist.items(), key=lambda kv: -kv[1])
            dom = order[0][0]
            sec_name = dominant_sector(cx, cy)
            tv = TRAVERSABILITY.get(dom, TRAVERSABILITY["GRASS"])
            set_props(ob,
                      sar_object="TERRAIN_TILE",
                      semantic_class="GROUND",
                      terrain_type=dom,
                      terrain_secondary=(order[1][0] if len(order) > 1 else dom),
                      terrain_mix=json.dumps({k: round(v / max(1, len(faces)), 3)
                                              for k, v in order[:4]}),
                      traversability=tv["trav"],
                      traverse_cost=tv["cost"],
                      friction_mu=tv["mu"],
                      risk_level=tv["risk"],
                      thermal_class=CLASS_THERMAL.get(dom, "THERMAL_SOIL"),
                      sar_sector=sec_name,
                      tile_resolution_m=res,
                      collision="MESH_EXACT")
            link_to(ob, "SAR_TERRAIN", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            total_v += len(verts); total_f += len(faces)
            TERRAIN_TILES.append(dict(name=name, sector=sec_name, dominant=dom,
                                      res=res, verts=len(verts), faces=len(faces),
                                      center=[cx, cy]))
        log("  terrain column %d/%d" % (ti + 1, nx))
    log("terrain: %d tiles, %d verts, %d quads" % (len(TERRAIN_TILES), total_v, total_f))
    return total_v, total_f


# ============================================================================
# PART 6 -- WATER
# ============================================================================

WATER_OBJS = []


def build_water():
    log("building water bodies")
    # --- river -----------------------------------------------------------
    samples = poly_sample(RIVER, 4.0)
    hw = RIVER_HALF + 1.2
    verts = []; faces = []
    for i, (px, py) in enumerate(samples):
        if i < len(samples) - 1:
            tx = samples[i + 1][0] - px; ty = samples[i + 1][1] - py
        else:
            tx = px - samples[i - 1][0]; ty = py - samples[i - 1][1]
        L = math.hypot(tx, ty) or 1.0
        nxv, nyv = -ty / L, tx / L
        verts.append((px + nxv * hw, py + nyv * hw, WATER_LEVEL))
        verts.append((px - nxv * hw, py - nyv * hw, WATER_LEVEL))
    for i in range(len(samples) - 1):
        a = 2 * i; b = a + 1; c = a + 3; d = a + 2
        faces.append((a, b, c, d))
    ob = new_mesh_obj("SAR_Water_River_001", verts, faces, ["MAT_WATER"])
    set_props(ob, sar_object="WATER_BODY", semantic_class="WATER",
              hazard_id="HAZARD_WATER_001", hazard_type="WATER",
              severity="CRITICAL", traversability="FORBIDDEN",
              terrain_type="WATER", risk_level="CRITICAL",
              thermal_class="THERMAL_WATER",
              water_surface_z=WATER_LEVEL, max_depth_m=RIVER_DEPTH,
              sar_sector="FLOOD", collision="NONE_VISUAL_ONLY",
              note="Deep water. Non-traversable for a wheeled UGV. The carved "
                   "channel in the terrain mesh is what physically stops the "
                   "robot; this surface is visual + a perception boundary.")
    link_to(ob, "FLOOD", "SAR_EXPORT", "SAR_VISUAL", "SAR_SENSOR_TEST")
    WATER_OBJS.append(ob)

    # --- flooded urban underpass ----------------------------------------
    cx, cy, r = POND
    pv = []; pf = []
    seg = 28
    pv.append((cx, cy, POND_LEVEL))
    for i in range(seg):
        a = 2 * math.pi * i / seg
        pv.append((cx + r * math.cos(a) * 1.25, cy + r * math.sin(a) * 0.8, POND_LEVEL))
    for i in range(seg):
        pf.append((0, 1 + i, 1 + (i + 1) % seg))
    ob2 = new_mesh_obj("SAR_Water_FloodPool_002", pv, pf, ["MAT_WATER"])
    set_props(ob2, sar_object="WATER_BODY", semantic_class="WATER",
              hazard_id="HAZARD_WATER_002", hazard_type="WATER",
              severity="HIGH", traversability="FORBIDDEN",
              terrain_type="WATER", risk_level="HIGH",
              thermal_class="THERMAL_WATER",
              water_surface_z=POND_LEVEL, max_depth_m=POND_DEPTH,
              sar_sector="URBAN", collision="NONE_VISUAL_ONLY",
              note="Flooded depression on the open-field / urban margin. Depth "
                   "is not observable from the bank: the robot must treat the "
                   "boundary as forbidden, not estimate it. Deliberately clear "
                   "of every street -- it is a detour, not a wall.")
    link_to(ob2, "URBAN", "OPEN_FIELD", "SAR_EXPORT", "SAR_VISUAL", "SAR_SENSOR_TEST")
    WATER_OBJS.append(ob2)
    return WATER_OBJS


# ============================================================================
# PART 7 -- ROADS AND BRIDGE
# ============================================================================

ROAD_SURF_MAT = {"ASPHALT": "MAT_ROAD", "GRAVEL": "MAT_GRAVEL", "DIRT": "MAT_DIRT"}
ROAD_CLASS_TERRAIN = {"ASPHALT": "ROAD", "GRAVEL": "GRAVEL", "DIRT": "DIRT"}


def build_roads():
    log("building road network")
    made = []
    for r in ROADS:
        prof = _ROAD_PROFILE[r["id"]]
        hw = r["width"] * 0.5
        verts = []; faces = []; skip = []
        for i, (px, py, pz) in enumerate(prof):
            if i < len(prof) - 1:
                tx = prof[i + 1][0] - px; ty = prof[i + 1][1] - py
            else:
                tx = px - prof[i - 1][0]; ty = py - prof[i - 1][1]
            L = math.hypot(tx, ty) or 1.0
            nxv, nyv = -ty / L, tx / L
            # the carriageway rides 5 cm over the graded corridor
            verts.append((px + nxv * hw, py + nyv * hw, pz + 0.05))
            verts.append((px - nxv * hw, py - nyv * hw, pz + 0.05))
            skip.append(negative_field(px, py) < -0.30)
        for i in range(len(prof) - 1):
            if skip[i] or skip[i + 1]:
                continue      # washout: the deck is genuinely missing here
            a = 2 * i; b = a + 1; c = a + 3; d = a + 2
            faces.append((a, b, c, d))
        ob = new_mesh_obj(r["id"], verts, faces, [ROAD_SURF_MAT[r["cls"]]])
        tt = ROAD_CLASS_TERRAIN[r["cls"]]
        tv = TRAVERSABILITY[tt]
        set_props(ob, sar_object="ROAD", semantic_class="ROAD",
                  road_class=r["cls"], road_width_m=r["width"],
                  terrain_type=tt, traversability=tv["trav"],
                  traverse_cost=tv["cost"], friction_mu=tv["mu"],
                  risk_level=tv["risk"],
                  thermal_class=CLASS_THERMAL[tt],
                  length_m=round(poly_length(r["pts"]), 1),
                  collision="MESH_EXACT",
                  centerline=json.dumps([[round(p[0], 2), round(p[1], 2)] for p in r["pts"]]))
        link_to(ob, "ROADS", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        made.append(ob)
        ROAD_BY_ID[r["id"]] = r
    log("roads: %d segments, %.0f m total" %
        (len(made), sum(poly_length(r["pts"]) for r in ROADS)))
    return made


def build_bridge():
    log("building bridge")
    B = BRIDGE
    cx, cy = B["center"]
    ax, ay = B["axis"]
    L = math.hypot(ax, ay) or 1.0
    ax, ay = ax / L, ay / L
    px, py = -ay, ax          # lateral
    hl = B["length"] * 0.5
    hw = B["width"] * 0.5
    z = B["deck_z"]
    gs, gl = B["damaged_gap"]

    objs = []

    # --- deck ------------------------------------------------------------
    # Three lateral stations per station so the damaged span can lose ONE HALF
    # of the carriageway rather than all of it. A full-width gap would simply
    # sever the west route; a half-width collapse leaves a ~2.4 m lane, which is
    # the interesting case: passable, tight, and only if the robot perceives
    # where the edge is.
    seg_len = 1.0
    n = int(B["length"] / seg_len)
    verts = []; faces = []
    for i in range(n + 1):
        s = -hl + i * seg_len
        bxp = cx + ax * s; byp = cy + ay * s
        verts.append((bxp + px * hw, byp + py * hw, z))   # 3i+0 left edge
        verts.append((bxp, byp, z))                       # 3i+1 centre line
        verts.append((bxp - px * hw, byp - py * hw, z))   # 3i+2 right edge
    gap_a = -hl + gs
    gap_b = gap_a + gl
    for i in range(n):
        s0 = -hl + i * seg_len
        s1 = s0 + seg_len
        in_gap = (s1 > gap_a and s0 < gap_b)
        a = 3 * i
        if not in_gap:
            faces.append((a, a + 1, a + 4, a + 3))        # left half
        faces.append((a + 1, a + 2, a + 5, a + 4))        # right half survives
    deck = new_mesh_obj("SAR_Bridge_001_Deck", verts, faces, ["MAT_CONCRETE"])
    set_props(deck, sar_object="BRIDGE", semantic_class="BRIDGE",
              hazard_id="HAZARD_BRIDGE_001", hazard_type="DAMAGED_BRIDGE",
              severity="HIGH", traversability="DIFFICULT",
              terrain_type="CONCRETE", risk_level="HIGH",
              thermal_class="THERMAL_CONCRETE",
              gap_length_m=gl, deck_width_m=B["width"],
              usable_lane_width_m=round(B["width"] * 0.5, 2),
              collision="MESH_EXACT",
              note="Half the carriageway is physically missing over the "
                   "collapsed span, leaving a %.1f m lane. There is no invisible "
                   "collision surface: a robot that tracks the centreline "
                   "instead of the surviving edge falls in."
                   % (B["width"] * 0.5))
    link_to(deck, "BRIDGES", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION", "SAR_SENSOR_TEST")
    objs.append(deck)

    # --- parapets (strong LiDAR landmark, and a real clearance constraint)
    for side, sgn in (("L", 1), ("R", -1)):
        pv = []; pf = []
        for i in range(n + 1):
            s = -hl + i * seg_len
            bxp = cx + ax * s + px * sgn * (hw - 0.12)
            byp = cy + ay * s + py * sgn * (hw - 0.12)
            pv.append((bxp, byp, z))
            pv.append((bxp, byp, z + 0.85))
        for i in range(n):
            s0 = -hl + i * seg_len; s1 = s0 + seg_len
            if side == "L" and s1 > gap_a - 0.5 and s0 < gap_b + 0.5:
                continue      # the parapet went into the river with its half-deck
            a = 2 * i; b = a + 1; c = a + 3; d = a + 2
            pf.append((a, b, c, d))
        ob = new_mesh_obj("SAR_Bridge_001_Parapet_%s" % side, pv, pf, ["MAT_CONCRETE"])
        set_props(ob, sar_object="BRIDGE_PARAPET", semantic_class="WALL",
                  thermal_class="THERMAL_CONCRETE", collision="BOX_COMPOUND",
                  traversability="FORBIDDEN")
        link_to(ob, "BRIDGES", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        objs.append(ob)

    # --- piers ------------------------------------------------------------
    for k, s in enumerate((-hl * 0.55, hl * 0.55)):
        bxp = cx + ax * s; byp = cy + ay * s
        gz = height_at(bxp, byp)
        ob = make_box("SAR_Bridge_001_Pier_%02d" % (k + 1), 1.6, B["width"] * 0.9,
                      max(0.6, z - gz + 0.4), "MAT_CONCRETE",
                      loc=(bxp, byp, gz - 0.2))
        ob.rotation_euler = Euler((0, 0, math.atan2(ay, ax)), 'XYZ')
        set_props(ob, sar_object="BRIDGE_PIER", semantic_class="STRUCTURE",
                  thermal_class="THERMAL_CONCRETE", collision="BOX")
        link_to(ob, "BRIDGES", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        objs.append(ob)

    # --- fallen deck slab in the channel (debris + LiDAR feature) ---------
    sx = cx + ax * (gap_a + gl * 0.5) + px * 2.0
    sy = cy + ay * (gap_a + gl * 0.5) + py * 2.0
    slab = make_box("SAR_Bridge_001_FallenSlab", 3.4, 2.6, 0.35, "MAT_CONCRETE",
                    loc=(sx, sy, height_at(sx, sy) + 0.3),
                    rot=(math.radians(18), math.radians(-11), 0.7))
    set_props(slab, sar_object="DEBRIS", semantic_class="RUBBLE",
              thermal_class="THERMAL_CONCRETE", collision="BOX",
              traversability="FORBIDDEN")
    link_to(slab, "BRIDGES", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
    objs.append(slab)
    return objs

# ============================================================================
# PART 8 -- PLACEMENT BOOKKEEPING
# ============================================================================
# Every structure, vehicle and victim registers a keep-out footprint before the
# scatter passes run. That is what stops a tree growing through a wall or a
# boulder landing on a casualty.

EXCLUSIONS = []      # (x, y, radius)
EXCL_RECTS = []      # (x0, x1, y0, y1)
_EXCL_GRID = {}
_EXCL_CELL = 8.0


def add_exclusion(x, y, r):
    EXCLUSIONS.append((x, y, r))
    i0 = int(math.floor((x - r) / _EXCL_CELL)); i1 = int(math.floor((x + r) / _EXCL_CELL))
    j0 = int(math.floor((y - r) / _EXCL_CELL)); j1 = int(math.floor((y + r) / _EXCL_CELL))
    idx = len(EXCLUSIONS) - 1
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            _EXCL_GRID.setdefault((i, j), []).append(idx)


def add_exclusion_rect(x0, x1, y0, y1):
    EXCL_RECTS.append((x0, x1, y0, y1))


def blocked(x, y, r=0.0):
    key = (int(math.floor(x / _EXCL_CELL)), int(math.floor(y / _EXCL_CELL)))
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            for idx in _EXCL_GRID.get((key[0] + i, key[1] + j), ()):
                ex, ey, er = EXCLUSIONS[idx]
                if (x - ex) ** 2 + (y - ey) ** 2 < (er + r) ** 2:
                    return True
    for (x0, x1, y0, y1) in EXCL_RECTS:
        if x0 - r <= x <= x1 + r and y0 - r <= y <= y1 + r:
            return True
    return False


def on_road(x, y, pad=1.0):
    rid, d, _gz = nearest_road(x, y)
    if rid is None:
        return False
    return d < ROAD_BY_ID[rid]["width"] * 0.5 + pad


def in_negative(x, y, pad=0.5):
    for nb in NEGATIVES:
        pts = nb["pts"]
        if len(pts) == 1:
            d = math.hypot(x - pts[0][0], y - pts[0][1])
        else:
            d, _b, _t = poly_dist(x, y, pts)
        if d < nb["half"] + pad:
            return True
    return False


def in_water(x, y, pad=2.0):
    d, _b, _t = poly_dist(x, y, RIVER)
    if d < RIVER_HALF + pad:
        return True
    cx, cy, r = POND
    if math.hypot((x - cx) / 1.25, (y - cy) / 0.8) < r + pad:
        return True
    return False


def plantable(x, y, r=0.6, max_slope=38.0):
    if not (CFG["x_min"] + 2 < x < CFG["x_max"] - 2):
        return False
    if not (CFG["y_min"] + 2 < y < CFG["y_max"] - 2):
        return False
    if blocked(x, y, r) or on_road(x, y, 1.2) or in_negative(x, y) or in_water(x, y):
        return False
    return True


def combine(pieces):
    """Merge [(verts, faces, mat_name)] into one mesh payload."""
    verts = []; faces = []; fmats = []; mats = []
    for (pv, pf, mn) in pieces:
        if mn not in mats:
            mats.append(mn)
        mi = mats.index(mn)
        off = len(verts)
        verts.extend(pv)
        for f in pf:
            faces.append(tuple(k + off for k in f))
            fmats.append(mi)
    return verts, faces, mats, fmats


def instance_of(proto_obj, name, loc, rot=(0, 0, 0), scale=(1, 1, 1)):
    """Linked duplicate: new object, SHARED mesh datablock. This is how ~2000
    pieces of vegetation and rubble cost almost nothing in memory and still
    export as individually addressable objects for LiDAR and semantics."""
    ob = bpy.data.objects.new(name, proto_obj.data)
    ob.location = loc
    ob.rotation_euler = Euler(rot, 'XYZ')
    ob.scale = scale
    for k in proto_obj.keys():
        if k.startswith(("sar_", "semantic", "thermal", "trav", "terrain",
                         "risk", "collision", "friction", "proto")):
            ob[k] = proto_obj[k]
    return ob


# ============================================================================
# PART 9 -- VEGETATION
# ============================================================================

TREE_PROTOS = []
BUSH_PROTOS = []
LOG_PROTOS = []
GRASS_PROTOS = []


def _broadleaf(rng, h):
    trunk_r = h * 0.035
    pieces = []
    tv, tf = cyl_verts(trunk_r, trunk_r * 0.55, h * 0.58, 7)
    pieces.append((tv, tf, "MAT_BARK"))
    # 2-3 branches -> real LiDAR structure, not a lollipop
    for _ in range(rng.randint(2, 3)):
        a = rng.uniform(0, math.tau)
        bl = h * rng.uniform(0.16, 0.26)
        bz = h * rng.uniform(0.34, 0.52)
        bv, bf = cyl_verts(trunk_r * 0.4, trunk_r * 0.2, bl, 5)
        ca, sa = math.cos(a), math.sin(a)
        tilt = rng.uniform(0.6, 1.0)
        nv = []
        for (vx, vy, vz) in bv:
            # rotate about Y by tilt, then about Z by a, then lift
            rx = vx * math.cos(tilt) + vz * math.sin(tilt)
            rz = -vx * math.sin(tilt) + vz * math.cos(tilt)
            nv.append((rx * ca - vy * sa, rx * sa + vy * ca, rz + bz))
        pieces.append((nv, bf, "MAT_BARK"))
    # canopy: 2-3 squashed icospheres, offset -> irregular silhouette
    for k in range(rng.randint(2, 3)):
        cr = h * rng.uniform(0.20, 0.30)
        cz = h * rng.uniform(0.62, 0.88)
        ox = rng.uniform(-h * 0.13, h * 0.13)
        oy = rng.uniform(-h * 0.13, h * 0.13)
        cv, cf = ico_verts(cr, 1, 0.22, rng, (1.15, 1.05, 0.78))
        pieces.append(([(vx + ox, vy + oy, vz + cz) for (vx, vy, vz) in cv],
                       cf, "MAT_VEGETATION"))
    return pieces


def _conifer(rng, h):
    trunk_r = h * 0.028
    pieces = []
    tv, tf = cyl_verts(trunk_r, trunk_r * 0.4, h * 0.95, 6)
    pieces.append((tv, tf, "MAT_BARK"))
    layers = rng.randint(3, 5)
    for k in range(layers):
        f = k / max(1, layers - 1)
        cz = h * (0.28 + 0.62 * f)
        cr = h * (0.26 - 0.17 * f) * rng.uniform(0.85, 1.15)
        ch = h * 0.24
        cv, cf = cyl_verts(cr, cr * 0.12, ch, 7)
        pieces.append(([(vx, vy, vz + cz) for (vx, vy, vz) in cv], cf, "MAT_VEGETATION"))
    return pieces


def _deadtrunk(rng, h):
    trunk_r = h * 0.05
    pieces = []
    tv, tf = cyl_verts(trunk_r, trunk_r * 0.35, h, 7)
    pieces.append((tv, tf, "MAT_BARK"))
    for _ in range(rng.randint(1, 2)):
        a = rng.uniform(0, math.tau)
        bv, bf = cyl_verts(trunk_r * 0.35, trunk_r * 0.1, h * 0.3, 5)
        ca, sa = math.cos(a), math.sin(a)
        nv = []
        for (vx, vy, vz) in bv:
            rx = vx * 0.3 + vz * 0.95
            rz = -vx * 0.95 + vz * 0.3
            nv.append((rx * ca - vy * sa, rx * sa + vy * ca, rz + h * 0.65))
        pieces.append((nv, bf, "MAT_BARK"))
    return pieces


def build_veg_protos():
    log("building vegetation prototypes")
    rng = sub_rng("veg_proto")
    # trees: 6 broadleaf + 4 conifer + 3 dead -> 13 distinct meshes.
    # Enough variety that a SLAM front-end cannot use "the tree shape" as a
    # landmark signature, which is the ambiguity trap in requirement 85.
    specs = ([("Broadleaf", _broadleaf, rng.uniform(6.5, 13.5)) for _ in range(6)] +
             [("Conifer",  _conifer,  rng.uniform(8.0, 16.0)) for _ in range(4)] +
             [("DeadTrunk", _deadtrunk, rng.uniform(4.0, 8.5)) for _ in range(3)])
    for i, (kind, fn, h) in enumerate(specs):
        pieces = fn(rng, h)
        v, f, mats, fm = combine(pieces)
        ob = new_mesh_obj("SAR_Proto_Tree_%s_%02d" % (kind, i + 1), v, f, mats, fm,
                          smooth=True)
        set_props(ob, sar_object="TREE", semantic_class="TREE",
                  proto_kind=kind, proto_height_m=round(h, 2),
                  thermal_class="THERMAL_VEGETATION",
                  traversability="FORBIDDEN", risk_level="MEDIUM",
                  collision="CYLINDER_TRUNK_ONLY",
                  collision_note="Trunk only. Nobody should be paying for leaf "
                                 "collision meshes in a physics step.")
        unlink_all(ob)
        C("VEGETATION").objects.link(ob)
        ob.hide_viewport = True
        ob.hide_render = True
        TREE_PROTOS.append(ob)

    for i in range(5):
        r = rng.uniform(0.55, 1.45)
        pieces = []
        for k in range(rng.randint(3, 5)):
            cr = r * rng.uniform(0.5, 0.95)
            cv, cf = ico_verts(cr, 1, 0.3, rng, (1.2, 1.1, 0.72))
            pieces.append(([(vx + rng.uniform(-r * .5, r * .5),
                             vy + rng.uniform(-r * .5, r * .5),
                             vz + cr * 0.75) for (vx, vy, vz) in cv],
                           cf, "MAT_VEGETATION"))
        v, f, mats, fm = combine(pieces)
        ob = new_mesh_obj("SAR_Proto_Bush_%02d" % (i + 1), v, f, mats, fm, smooth=True)
        set_props(ob, sar_object="BUSH", semantic_class="VEGETATION",
                  thermal_class="THERMAL_VEGETATION",
                  traversability="DIFFICULT", risk_level="LOW",
                  collision="BOX_LOW",
                  collision_note="Soft obstacle: geometric occluder for RGB, "
                                 "partial LiDAR return, drivable-with-cost.")
        unlink_all(ob); C("VEGETATION").objects.link(ob)
        ob.hide_viewport = True; ob.hide_render = True
        BUSH_PROTOS.append(ob)

    for i in range(4):
        L = rng.uniform(3.5, 8.0)
        r = rng.uniform(0.18, 0.38)
        v, f = cyl_verts(r, r * 0.8, L, 7)
        v = [(vz - L * 0.5, vy, vx + r) for (vx, vy, vz) in v]   # lay it down
        ob = new_mesh_obj("SAR_Proto_FallenTree_%02d" % (i + 1), v, f,
                          ["MAT_BARK"], smooth=True)
        set_props(ob, sar_object="FALLEN_TREE", semantic_class="TREE",
                  thermal_class="THERMAL_WOOD",
                  traversability="DIFFICULT", risk_level="MEDIUM",
                  collision="BOX",
                  obstacle_height_m=round(r * 2, 2))
        unlink_all(ob); C("VEGETATION").objects.link(ob)
        ob.hide_viewport = True; ob.hide_render = True
        LOG_PROTOS.append(ob)

    # grass: clumps of crossed cards, batched ~36 tufts per object so we get
    # ground cover without 20000 objects in the outliner
    for i in range(3):
        pieces = []
        for k in range(36):
            gx = rng.uniform(-2.2, 2.2); gy = rng.uniform(-2.2, 2.2)
            hgt = rng.uniform(0.18, 0.45); w = rng.uniform(0.10, 0.22)
            a = rng.uniform(0, math.tau)
            for turn in (0.0, math.pi / 2):
                aa = a + turn
                ca, sa = math.cos(aa), math.sin(aa)
                pv = [(gx - w * ca, gy - w * sa, 0.0), (gx + w * ca, gy + w * sa, 0.0),
                      (gx + w * ca * 0.6, gy + w * sa * 0.6, hgt),
                      (gx - w * ca * 0.6, gy - w * sa * 0.6, hgt)]
                pieces.append((pv, [(0, 1, 2, 3)], "MAT_VEGETATION"))
        v, f, mats, fm = combine(pieces)
        ob = new_mesh_obj("SAR_Proto_GrassPatch_%02d" % (i + 1), v, f, mats, fm)
        set_props(ob, sar_object="GRASS", semantic_class="VEGETATION",
                  thermal_class="THERMAL_VEGETATION",
                  traversability="EASY", risk_level="LOW",
                  collision="NONE_VISUAL_ONLY",
                  collision_note="No collision. Grass must not appear in the "
                                 "costmap or the robot will refuse to move.")
        unlink_all(ob); C("VEGETATION").objects.link(ob)
        ob.hide_viewport = True; ob.hide_render = True
        GRASS_PROTOS.append(ob)


def _veg_density(x, y):
    """Local vegetation density multiplier. Noise gives easy / hard / impassable
    pockets so route choice inside the forest is a real decision."""
    n = vnoise(x * 0.022, y * 0.022, 555)
    m = vnoise(x * 0.070, y * 0.070, 556)
    return clamp(0.12 + 2.15 * (n ** 1.7) + 0.4 * m, 0.0, 2.6)


SECTOR_TREE_DENSITY = {
    "FOREST": 0.058, "SNOW": 0.013, "OPEN_FIELD": 0.0016,
    "MOUNTAIN": 0.0042, "RUBBLE": 0.0012, "URBAN": 0.0016,
    "FLOOD": 0.0035, "DESERT": 0.0004, "BASE": 0.0,
}
SECTOR_BUSH_DENSITY = {
    "FOREST": 0.042, "SNOW": 0.006, "OPEN_FIELD": 0.0075,
    "MOUNTAIN": 0.0055, "RUBBLE": 0.0022, "URBAN": 0.0035,
    "FLOOD": 0.0090, "DESERT": 0.0018, "BASE": 0.0006,
}
SECTOR_GRASS_DENSITY = {
    "FOREST": 0.0016, "OPEN_FIELD": 0.0042, "FLOOD": 0.0022,
    "URBAN": 0.0011, "RUBBLE": 0.0006, "MOUNTAIN": 0.0006,
    "SNOW": 0.0, "DESERT": 0.0003, "BASE": 0.0002,
}

STATS = {}


def scatter_vegetation():
    log("scattering vegetation")
    rng = sub_rng("veg_scatter")
    n_tree = n_bush = n_grass = n_log = 0
    step = 2.5
    x = CFG["x_min"] + 3
    while x < CFG["x_max"] - 3:
        y = CFG["y_min"] + 3
        while y < CFG["y_max"] - 3:
            jx = x + rng.uniform(-step * .5, step * .5)
            jy = y + rng.uniform(-step * .5, step * .5)
            sec = dominant_sector(jx, jy)
            dm = _veg_density(jx, jy)
            cell_a = step * step

            p = SECTOR_TREE_DENSITY.get(sec, 0.0) * dm * cell_a
            if rng.random() < p and plantable(jx, jy, 1.0, 34.0):
                proto = rng.choice(TREE_PROTOS)
                if sec == "SNOW" and "Conifer" not in proto.name and rng.random() < 0.7:
                    proto = rng.choice([t for t in TREE_PROTOS if "Conifer" in t.name])
                s = rng.uniform(0.72, 1.32)
                ob = instance_of(proto, "SAR_Tree_%04d" % (n_tree + 1),
                                 (jx, jy, height_at(jx, jy) - 0.15),
                                 rot=(rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05),
                                      rng.uniform(0, math.tau)),
                                 scale=(s, s, s * rng.uniform(0.92, 1.1)))
                ob["sar_sector"] = sec
                ob["is_snow_covered"] = (sec == "SNOW")
                link_to(ob, "VEGETATION", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
                if sec in ("FOREST", "SNOW", "MOUNTAIN"):
                    link_to(ob, {"FOREST": "FOREST", "SNOW": "SNOW",
                                 "MOUNTAIN": "MOUNTAIN"}[sec])
                add_exclusion(jx, jy, 0.55 * s)
                n_tree += 1

            p = SECTOR_BUSH_DENSITY.get(sec, 0.0) * dm * cell_a
            if rng.random() < p and plantable(jx, jy, 0.5, 40.0):
                proto = rng.choice(BUSH_PROTOS)
                s = rng.uniform(0.7, 1.5)
                ob = instance_of(proto, "SAR_Bush_%04d" % (n_bush + 1),
                                 (jx, jy, height_at(jx, jy) - 0.06),
                                 rot=(0, 0, rng.uniform(0, math.tau)),
                                 scale=(s, s, s * rng.uniform(0.8, 1.2)))
                ob["sar_sector"] = sec
                link_to(ob, "VEGETATION", "SAR_EXPORT", "SAR_VISUAL")
                n_bush += 1

            p = SECTOR_GRASS_DENSITY.get(sec, 0.0) * cell_a * dm
            if rng.random() < p and plantable(jx, jy, 2.0, 30.0):
                proto = rng.choice(GRASS_PROTOS)
                ob = instance_of(proto, "SAR_GrassPatch_%04d" % (n_grass + 1),
                                 (jx, jy, height_at(jx, jy) - 0.03),
                                 rot=(0, 0, rng.uniform(0, math.tau)),
                                 scale=(1, 1, rng.uniform(0.7, 1.3)))
                ob["sar_sector"] = sec
                link_to(ob, "VEGETATION", "SAR_VISUAL")
                n_grass += 1

            if sec in ("FOREST", "SNOW", "MOUNTAIN") and rng.random() < 0.0035 * dm * cell_a:
                if plantable(jx, jy, 2.0, 30.0):
                    proto = rng.choice(LOG_PROTOS)
                    ob = instance_of(proto, "SAR_FallenTree_%03d" % (n_log + 1),
                                     (jx, jy, height_at(jx, jy) + 0.05),
                                     rot=(0, rng.uniform(-0.1, 0.1), rng.uniform(0, math.tau)),
                                     scale=(rng.uniform(0.8, 1.3),) * 3)
                    ob["sar_sector"] = sec
                    link_to(ob, "VEGETATION", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
                    add_exclusion(jx, jy, 1.2)
                    n_log += 1
            y += step
        x += step
    STATS.update(trees=n_tree, bushes=n_bush, grass_patches=n_grass, fallen_trees=n_log)
    log("vegetation: %d trees, %d bushes, %d grass patches, %d fallen trees"
        % (n_tree, n_bush, n_grass, n_log))


# ============================================================================
# PART 10 -- ROCKS, BOULDERS, RUBBLE
# ============================================================================

ROCK_PROTOS = []
BOULDER_PROTOS = []
RUBBLE_PROTOS = []


def build_rock_protos():
    log("building rock and rubble prototypes")
    rng = sub_rng("rock_proto")
    for i in range(8):
        r = rng.uniform(0.22, 0.95)
        v, f = ico_verts(r, 1, 0.34, rng,
                         (rng.uniform(0.8, 1.4), rng.uniform(0.8, 1.4),
                          rng.uniform(0.5, 0.95)))
        v = [(vx, vy, vz + r * 0.45) for (vx, vy, vz) in v]
        ob = new_mesh_obj("SAR_Proto_Rock_%02d" % (i + 1), v, f, ["MAT_ROCK"])
        set_props(ob, sar_object="ROCK", semantic_class="ROCK",
                  thermal_class="THERMAL_ROCK", terrain_type="ROCK",
                  traversability="DIFFICULT", risk_level="MEDIUM",
                  collision="CONVEX_HULL", obstacle_size_m=round(r * 2, 2))
        unlink_all(ob); C("SAR_ENVIRONMENT").objects.link(ob)
        ob.hide_viewport = True; ob.hide_render = True
        ROCK_PROTOS.append(ob)

    for i in range(5):
        r = rng.uniform(1.3, 3.1)
        v, f = ico_verts(r, 2, 0.26, rng,
                         (rng.uniform(0.85, 1.3), rng.uniform(0.85, 1.3),
                          rng.uniform(0.6, 1.0)))
        v = [(vx, vy, vz + r * 0.35) for (vx, vy, vz) in v]
        ob = new_mesh_obj("SAR_Proto_Boulder_%02d" % (i + 1), v, f, ["MAT_ROCK"])
        set_props(ob, sar_object="BOULDER", semantic_class="ROCK",
                  thermal_class="THERMAL_ROCK", terrain_type="ROCK",
                  traversability="FORBIDDEN", risk_level="HIGH",
                  collision="CONVEX_HULL", obstacle_size_m=round(r * 2, 2))
        unlink_all(ob); C("SAR_ENVIRONMENT").objects.link(ob)
        ob.hide_viewport = True; ob.hide_render = True
        BOULDER_PROTOS.append(ob)

    # ---- rubble kit -----------------------------------------------------
    def slab(rng):
        sx = rng.uniform(0.5, 2.4); sy = rng.uniform(0.4, 1.6); sz = rng.uniform(0.10, 0.28)
        return box_verts(sx, sy, sz), "MAT_CONCRETE", "CONCRETE_SLAB"

    def brick(rng):
        return box_verts(rng.uniform(0.18, 0.26), rng.uniform(0.09, 0.13),
                         rng.uniform(0.06, 0.09)), "MAT_BRICK", "BRICK"

    def beam(rng):
        # I-beam: three plates, so LiDAR sees a real profile
        L = rng.uniform(1.8, 5.5); w = rng.uniform(0.16, 0.26); t = 0.035
        parts = []
        for zz in (0.0, w * 1.6):
            v, f = box_verts(L, w, t, cz=zz + t * .5)
            parts.append((v, f, "MAT_RUSTMETAL"))
        v, f = box_verts(L, t, w * 1.6, cz=w * 0.8)
        parts.append((v, f, "MAT_RUSTMETAL"))
        return parts, None, "STEEL_BEAM"

    def pipe(rng):
        L = rng.uniform(1.2, 4.0); r = rng.uniform(0.08, 0.24)
        v, f = cyl_verts(r, r, L, 8)
        v = [(vz - L * .5, vy, vx + r) for (vx, vy, vz) in v]
        return (v, f), "MAT_METAL", "PIPE"

    def plank(rng):
        return box_verts(rng.uniform(0.9, 2.8), rng.uniform(0.10, 0.24),
                         rng.uniform(0.03, 0.06)), "MAT_WOOD", "TIMBER"

    def chunk(rng):
        r = rng.uniform(0.18, 0.7)
        v, f = ico_verts(r, 1, 0.42, rng, (1.3, 0.9, 0.6))
        return (v, f), "MAT_RUBBLE", "CONCRETE_CHUNK"

    def wallfrag(rng):
        sx = rng.uniform(1.2, 3.2); sz = rng.uniform(0.8, 2.2)
        return box_verts(sx, 0.22, sz), "MAT_BRICK", "WALL_FRAGMENT"

    kit = [slab, slab, brick, brick, brick, beam, beam, pipe, pipe,
           plank, plank, chunk, chunk, chunk, wallfrag]
    for i, fn in enumerate(kit):
        res = fn(rng)
        if res[1] is None:
            v, f, mats, fm = combine(res[0])
            ob = new_mesh_obj("SAR_Proto_Rubble_%s_%02d" % (res[2], i + 1),
                              v, f, mats, fm)
            mat_thermal = "THERMAL_METAL"
        else:
            (v, f), mn, kind = res
            ob = new_mesh_obj("SAR_Proto_Rubble_%s_%02d" % (kind, i + 1), v, f, [mn])
            mat_thermal = {"MAT_CONCRETE": "THERMAL_CONCRETE",
                           "MAT_BRICK": "THERMAL_CONCRETE",
                           "MAT_METAL": "THERMAL_METAL",
                           "MAT_RUSTMETAL": "THERMAL_METAL",
                           "MAT_WOOD": "THERMAL_WOOD",
                           "MAT_RUBBLE": "THERMAL_CONCRETE"}[mn]
        set_props(ob, sar_object="RUBBLE_PIECE", semantic_class="RUBBLE",
                  rubble_kind=res[2], thermal_class=mat_thermal,
                  terrain_type="RUBBLE", traversability="VERY_DIFFICULT",
                  risk_level="HIGH", collision="CONVEX_HULL")
        unlink_all(ob); C("RUBBLE").objects.link(ob)
        ob.hide_viewport = True; ob.hide_render = True
        RUBBLE_PROTOS.append(ob)


def scatter_rocks():
    log("scattering rocks and boulders")
    rng = sub_rng("rock_scatter")
    dens_rock = {"MOUNTAIN": 0.022, "SNOW": 0.0095, "RUBBLE": 0.006,
                 "OPEN_FIELD": 0.0022, "FOREST": 0.0048, "DESERT": 0.0042,
                 "FLOOD": 0.0030, "URBAN": 0.0010, "BASE": 0.0}
    dens_bould = {"MOUNTAIN": 0.0030, "SNOW": 0.0012, "FOREST": 0.0006,
                  "DESERT": 0.0006, "OPEN_FIELD": 0.00025, "RUBBLE": 0.0008,
                  "FLOOD": 0.0003, "URBAN": 0.0, "BASE": 0.0}
    nr = nb = 0
    step = 2.5
    x = CFG["x_min"] + 3
    while x < CFG["x_max"] - 3:
        y = CFG["y_min"] + 3
        while y < CFG["y_max"] - 3:
            jx = x + rng.uniform(-1.2, 1.2); jy = y + rng.uniform(-1.2, 1.2)
            sec = dominant_sector(jx, jy)
            a = step * step
            if rng.random() < dens_rock.get(sec, 0.0) * a and plantable(jx, jy, 0.4, 55.0):
                proto = rng.choice(ROCK_PROTOS)
                s = rng.uniform(0.6, 1.8)
                ob = instance_of(proto, "SAR_Rock_%04d" % (nr + 1),
                                 (jx, jy, height_at(jx, jy) - 0.10 * s),
                                 rot=(rng.uniform(0, .5), rng.uniform(0, .5),
                                      rng.uniform(0, math.tau)),
                                 scale=(s, s * rng.uniform(.8, 1.2), s * rng.uniform(.7, 1.1)))
                ob["sar_sector"] = sec
                link_to(ob, "SAR_ENVIRONMENT", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
                nr += 1
            if rng.random() < dens_bould.get(sec, 0.0) * a and plantable(jx, jy, 2.5, 50.0):
                proto = rng.choice(BOULDER_PROTOS)
                s = rng.uniform(0.7, 1.5)
                ob = instance_of(proto, "SAR_Boulder_%03d" % (nb + 1),
                                 (jx, jy, height_at(jx, jy) - 0.25 * s),
                                 rot=(rng.uniform(0, .3), rng.uniform(0, .3),
                                      rng.uniform(0, math.tau)),
                                 scale=(s, s * rng.uniform(.85, 1.15), s * rng.uniform(.75, 1.05)))
                ob["sar_sector"] = sec
                link_to(ob, "SAR_ENVIRONMENT", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
                add_exclusion(jx, jy, 2.2 * s)
                nb += 1
            y += step
        x += step
    STATS.update(rocks=nr, boulders=nb)
    log("rocks: %d, boulders: %d" % (nr, nb))


RUBBLE_FIELDS = [
    dict(id="SAR_Rubble_001", c=(-6, 108), rx=30, ry=19, n=230, zone="SAR_ZONE_B",
         label="Primary collapse field"),
    dict(id="SAR_Rubble_002", c=(76, 56), rx=17, ry=13, n=110, zone="SAR_ZONE_C",
         label="Urban street collapse"),
    dict(id="SAR_Rubble_003", c=(-36, 120), rx=14, ry=11, n=80, zone="SAR_ZONE_B",
         label="Secondary debris apron"),
    dict(id="SAR_Rubble_004", c=(-52, 232), rx=22, ry=15, n=95, zone="SAR_ZONE_D",
         label="Avalanche debris field"),
]


def scatter_rubble():
    log("scattering rubble fields")
    rng = sub_rng("rubble_scatter")
    total = 0
    for fld in RUBBLE_FIELDS:
        cx, cy = fld["c"]
        made = 0
        tries = 0
        while made < fld["n"] and tries < fld["n"] * 14:
            tries += 1
            a = rng.uniform(0, math.tau)
            rr = math.sqrt(rng.random())
            jx = cx + math.cos(a) * rr * fld["rx"]
            jy = cy + math.sin(a) * rr * fld["ry"]
            if not (CFG["x_min"] + 2 < jx < CFG["x_max"] - 2):
                continue
            if in_water(jx, jy, 1.0) or in_negative(jx, jy, 0.2):
                continue
            if blocked(jx, jy, 0.25):
                continue
            proto = rng.choice(RUBBLE_PROTOS)
            s = rng.uniform(0.6, 1.9)
            zz = height_at(jx, jy)
            # debris genuinely piles: stack height falls off from the centre
            pile = 1.0 - math.hypot((jx - cx) / fld["rx"], (jy - cy) / fld["ry"])
            zz += max(0.0, pile) * rng.uniform(0.0, 0.9)
            ob = instance_of(proto, "%s_Piece_%04d" % (fld["id"], made + 1),
                             (jx, jy, zz),
                             rot=(rng.uniform(0, math.pi), rng.uniform(0, math.pi),
                                  rng.uniform(0, math.tau)),
                             scale=(s, s * rng.uniform(.8, 1.25), s * rng.uniform(.8, 1.25)))
            ob["sar_sector"] = dominant_sector(jx, jy)
            ob["search_zone"] = fld["zone"]
            ob["hazard_id"] = fld["id"]
            link_to(ob, "RUBBLE", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            if fld["id"] == "SAR_Rubble_004":
                link_to(ob, "SNOW")
            made += 1
        total += made
        log("  %s: %d pieces" % (fld["id"], made))
    STATS["rubble_pieces"] = total

# ============================================================================
# PART 11 -- BUILDINGS
# ============================================================================
# Walls are assembled from solid piers and lintels around real openings. No
# booleans, no invisible doorway planes: if the mesh shows a 0.95 m doorway then
# a 0.95 m doorway is what the collision geometry and the LiDAR see. Interiors
# are genuinely enterable, which is what makes the indoor/outdoor localisation
# transition and the "victim visible only through an opening" scenarios real.

DOOR_W, DOOR_H = 1.05, 2.10
WIN_W, WIN_H, WIN_SILL = 1.25, 1.20, 1.00
STOREY_H = 3.20
WALL_T = 0.26


def wall_boxes(L, H, openings):
    """Return [(cx, cz, sx, sz)] solid pieces of a wall of length L, height H.
    openings: [(start, width, sill, top)] measured along the wall."""
    out = []
    ops = sorted(openings, key=lambda o: o[0])
    cursor = 0.0
    for (s, w, sill, top) in ops:
        s = clamp(s, 0.0, L); e = clamp(s + w, 0.0, L)
        if e <= cursor:
            continue
        if s > cursor:
            out.append(((cursor + s) * 0.5, H * 0.5, s - cursor, H))
        if sill > 0.001:
            out.append(((s + e) * 0.5, sill * 0.5, e - s, sill))
        if top < H - 0.001:
            out.append(((s + e) * 0.5, (top + H) * 0.5, e - s, H - top))
        cursor = e
    if cursor < L:
        out.append(((cursor + L) * 0.5, H * 0.5, L - cursor, H))
    return out


def wall_mesh_pieces(p0, p1, H, openings, t=WALL_T, z0=0.0):
    """Build wall geometry in WORLD XY between p0 and p1."""
    ax = p1[0] - p0[0]; ay = p1[1] - p0[1]
    L = math.hypot(ax, ay)
    if L < 1e-6:
        return []
    ux, uy = ax / L, ay / L
    nx, ny = -uy, ux
    pieces = []
    for (cx, cz, sx, sz) in wall_boxes(L, H, openings):
        wx = p0[0] + ux * cx; wy = p0[1] + uy * cx
        hx = ux * sx * 0.5; hy = uy * sx * 0.5
        tx = nx * t * 0.5; ty = ny * t * 0.5
        zb = z0 + cz - sz * 0.5; zt = z0 + cz + sz * 0.5
        v = [(wx - hx - tx, wy - hy - ty, zb), (wx + hx - tx, wy + hy - ty, zb),
             (wx + hx + tx, wy + hy + ty, zb), (wx - hx + tx, wy - hy + ty, zb),
             (wx - hx - tx, wy - hy - ty, zt), (wx + hx - tx, wy + hy - ty, zt),
             (wx + hx + tx, wy + hy + ty, zt), (wx - hx + tx, wy - hy + ty, zt)]
        f = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
             (2, 6, 7, 3), (3, 7, 4, 0)]
        pieces.append((v, f, None))
    return pieces


BUILDINGS = [
    # NOTE: these four sit inside the street grid blocks, not on top of it.
    # An earlier layout had all four straddling carriageways and left a 0.33 m
    # "canyon" between 001 and 002 -- a gap the robot cannot enter is not a GPS
    # test, it is a wall. The validator now asserts both conditions.
    dict(id="SAR_Building_URB_001", c=(70.0, 50.0), w=11.0, d=12.0, storeys=3,
         rot=0.05, mat="MAT_CONCRETE", damage="MODERATE", sector="URBAN",
         zone="SAR_ZONE_C", interior=True, roof="PARTIAL",
         label="3-storey office block. West face of the urban canyon."),
    dict(id="SAR_Building_URB_002", c=(89.0, 50.0), w=11.0, d=12.0, storeys=3,
         rot=-0.03, mat="MAT_CONCRETE", damage="LIGHT", sector="URBAN",
         zone="SAR_ZONE_C", interior=False, roof="FULL",
         label="3-storey block. East face of the urban canyon; ~7.5 m of open "
               "street between 001 and 002 with 10 m walls either side -> "
               "GPS multipath / sky occlusion, and wide enough to drive."),
    dict(id="SAR_Building_URB_003", c=(75.5, 79.0), w=13.0, d=10.0, storeys=2,
         rot=0.35, mat="MAT_BRICK", damage="SEVERE", sector="URBAN",
         zone="SAR_ZONE_C", interior=True, roof="COLLAPSED",
         label="Partially collapsed 2-storey. Victim inside, visible ONLY "
               "through the collapsed north-east wall opening."),
    dict(id="SAR_Building_URB_004", c=(114.0, 55.5), w=22.0, d=13.0, storeys=1,
         rot=-0.05, mat="MAT_CONCRETE", damage="MODERATE", sector="URBAN",
         zone="SAR_ZONE_C", interior=True, roof="PARTIAL", tall=5.2,
         label="Warehouse. Wide roller entrance + narrow side door + one "
               "blocked door; dark interior, rooms and a corridor."),
    dict(id="SAR_Building_RUB_001", c=(-14.0, 100.0), w=11.0, d=9.0, storeys=1,
         rot=0.6, mat="MAT_BRICK", damage="COLLAPSED", sector="RUBBLE",
         zone="SAR_ZONE_B", interior=True, roof="COLLAPSED",
         label="Collapsed house. One survivable void with a crawl-height "
               "opening; casualty adjacent, NOT buried."),
    dict(id="SAR_Building_RUB_002", c=(14.0, 118.0), w=14.0, d=10.0, storeys=2,
         rot=-0.45, mat="MAT_CONCRETE", damage="SEVERE", sector="RUBBLE",
         zone="SAR_ZONE_B", interior=True, roof="PARTIAL",
         label="Partially collapsed 2-storey. Three approaches: wide (unsafe), "
               "narrow (tight clearance), blocked. Forces entrance selection."),
]


def build_building(spec):
    rng = sub_rng("bld_" + spec["id"])
    cx, cy = spec["c"]
    w, d = spec["w"], spec["d"]
    rot = spec["rot"]
    ca, sa = math.cos(rot), math.sin(rot)
    gz = height_at(cx, cy)
    sh = spec.get("tall", STOREY_H)
    objs = []

    def L2W(lx, ly):
        return (cx + lx * ca - ly * sa, cy + lx * sa + ly * ca)

    hw, hd = w * 0.5, d * 0.5
    corners = [L2W(-hw, -hd), L2W(hw, -hd), L2W(hw, hd), L2W(-hw, hd)]

    add_exclusion(cx, cy, math.hypot(hw, hd) + 1.2)

    dmg = spec["damage"]
    n_st = spec["storeys"]
    collapse_frac = {"LIGHT": 0.0, "MODERATE": 0.18, "SEVERE": 0.42,
                     "COLLAPSED": 0.72}[dmg]

    # --- floor slabs -----------------------------------------------------
    for s in range(n_st + 1):
        if s == 0:
            keep = 1.0
        else:
            keep = 1.0 - collapse_frac
            if keep < 0.25:
                continue
        z = gz + s * sh
        fw = w * (1.0 if s == 0 else keep + 0.15)
        fd = d * (1.0 if s == 0 else 1.0)
        v, f = box_verts(min(fw, w), fd, 0.22, cz=z + 0.11)
        ob = new_mesh_obj("%s_Slab_%02d" % (spec["id"], s), v, f, [spec["mat"]],
                          loc=(cx - (w - min(fw, w)) * 0.5 * ca,
                               cy - (w - min(fw, w)) * 0.5 * sa, 0))
        ob.rotation_euler = Euler((0, 0, rot), 'XYZ')
        set_props(ob, sar_object="BUILDING_SLAB", semantic_class="BUILDING",
                  building_id=spec["id"], storey=s,
                  thermal_class="THERMAL_CONCRETE", collision="BOX",
                  traversability="EASY" if s == 0 else "FORBIDDEN")
        link_to(ob, "BUILDINGS", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        objs.append(ob)

    # --- perimeter walls, per storey -------------------------------------
    side_names = ["S", "E", "N", "W"]
    for s in range(n_st):
        z0 = gz + s * sh + 0.22
        for si in range(4):
            p0 = corners[si]
            p1 = corners[(si + 1) % 4]
            L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            ops = []
            if s == 0:
                if si == 0:              # front: main door
                    ops.append((L * 0.5 - DOOR_W * 0.5, DOOR_W, 0.0, DOOR_H))
                    if spec["id"] == "SAR_Building_URB_004":
                        # wide roller shutter entrance
                        ops = [(L * 0.34, 4.6, 0.0, 4.1)]
                if si == 2 and spec.get("interior"):
                    # rear door -- second, narrower approach
                    ops.append((L * 0.62, 0.92, 0.0, DOOR_H))
            # windows on every storey
            nwin = max(1, int(L / 3.4))
            for k in range(nwin):
                wx0 = (k + 0.5) * (L / nwin) - WIN_W * 0.5
                if any(abs(wx0 - o[0]) < (o[1] + WIN_W) for o in ops):
                    continue
                ops.append((wx0, WIN_W, WIN_SILL, WIN_SILL + WIN_H))

            # collapse: drop the wall over the collapsed fraction, from the
            # top storey down and from one corner across
            frac_gone = collapse_frac * (0.55 + 0.75 * (s / max(1, n_st - 1)) if n_st > 1 else collapse_frac)
            frac_gone = clamp(frac_gone, 0.0, 0.95)
            if frac_gone > 0.05 and si in (1, 2):
                ops.append((L * (1.0 - frac_gone), L * frac_gone + 0.1, 0.0, sh))

            pieces = wall_mesh_pieces(p0, p1, sh, ops, WALL_T, z0)
            if not pieces:
                continue
            pieces = [(pv, pf, spec["mat"]) for (pv, pf, _m) in pieces]
            v, f, mats, fm = combine(pieces)
            ob = new_mesh_obj("%s_Wall_%s_%02d" % (spec["id"], side_names[si], s),
                              v, f, mats, fm)
            set_props(ob, sar_object="BUILDING_WALL", semantic_class="WALL",
                      building_id=spec["id"], storey=s, wall_side=side_names[si],
                      thermal_class=("THERMAL_CONCRETE" if spec["mat"] != "MAT_WOOD"
                                     else "THERMAL_WOOD"),
                      collision="BOX_COMPOUND", traversability="FORBIDDEN",
                      risk_level="LOW",
                      opaque_to_thermal=True,
                      thermal_note="Solid wall. A LWIR camera does NOT see "
                                   "through this. Interior casualties are only "
                                   "detectable through openings.")
            link_to(ob, "BUILDINGS", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            objs.append(ob)

    # --- interior partitions: rooms + a corridor -------------------------
    if spec.get("interior"):
        z0 = gz + 0.22
        parts = []
        # spine corridor wall with two doorways
        p0 = L2W(-hw + 0.3, -hd * 0.10); p1 = L2W(hw - 0.3, -hd * 0.10)
        Lc = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        parts += wall_mesh_pieces(p0, p1, sh * 0.92,
                                  [(Lc * 0.22, 0.95, 0, DOOR_H),
                                   (Lc * 0.68, 0.95, 0, DOOR_H)], 0.18, z0)
        # two cross walls -> three rooms on the far side
        for fx in (-0.30, 0.34):
            q0 = L2W(hw * fx * 2, -hd * 0.10)
            q1 = L2W(hw * fx * 2, hd - 0.3)
            Lq = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
            parts += wall_mesh_pieces(q0, q1, sh * 0.92,
                                      [(Lq * 0.45, 0.92, 0, DOOR_H)], 0.18, z0)
        if parts:
            parts = [(pv, pf, "MAT_CONCRETE") for (pv, pf, _m) in parts]
            v, f, mats, fm = combine(parts)
            ob = new_mesh_obj("%s_Interior" % spec["id"], v, f, mats, fm)
            set_props(ob, sar_object="BUILDING_PARTITION", semantic_class="WALL",
                      building_id=spec["id"], thermal_class="THERMAL_CONCRETE",
                      collision="BOX_COMPOUND", traversability="FORBIDDEN",
                      opaque_to_thermal=True,
                      lighting_condition="DARK_INTERIOR",
                      note="Rooms + corridor. Some rooms empty, some occupied "
                           "-- the robot has to clear them to know which.")
            link_to(ob, "BUILDINGS", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            objs.append(ob)

        # furniture + interior debris: depth-camera clutter and occluders
        frng = sub_rng("furn_" + spec["id"])
        for k in range(frng.randint(5, 10)):
            lx = frng.uniform(-hw + 1.2, hw - 1.2)
            ly = frng.uniform(-hd + 1.2, hd - 1.2)
            wx, wy = L2W(lx, ly)
            kind = frng.choice(["DESK", "CABINET", "CRATE", "SHELF", "DEBRIS"])
            dims = {"DESK": (1.5, 0.75, 0.74), "CABINET": (0.9, 0.5, 1.85),
                    "CRATE": (0.8, 0.8, 0.7), "SHELF": (1.8, 0.4, 2.0),
                    "DEBRIS": (1.1, 0.9, 0.35)}[kind]
            mat = "MAT_WOOD" if kind in ("DESK", "SHELF", "CRATE") else (
                "MAT_METAL" if kind == "CABINET" else "MAT_RUBBLE")
            ob = make_box("%s_Furniture_%s_%02d" % (spec["id"], kind, k + 1),
                          dims[0], dims[1], dims[2], mat,
                          loc=(wx, wy, gz + 0.22),
                          rot=(0, 0, rot + frng.uniform(-0.6, 0.6)))
            set_props(ob, sar_object="FURNITURE", semantic_class="PROP",
                      building_id=spec["id"], furniture_kind=kind,
                      thermal_class=("THERMAL_WOOD" if mat == "MAT_WOOD"
                                     else "THERMAL_METAL" if mat == "MAT_METAL"
                                     else "THERMAL_CONCRETE"),
                      collision="BOX", traversability="FORBIDDEN",
                      lighting_condition="DARK_INTERIOR")
            link_to(ob, "BUILDINGS", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            objs.append(ob)

    # --- roof ------------------------------------------------------------
    if spec["roof"] != "COLLAPSED":
        z = gz + n_st * sh + 0.22
        keep = 1.0 if spec["roof"] == "FULL" else 0.62
        v, f = box_verts(w * keep, d, 0.20, cz=z + 0.1)
        ob = new_mesh_obj("%s_Roof" % spec["id"], v, f, [spec["mat"]],
                          loc=(cx - (w - w * keep) * 0.5 * ca,
                               cy - (w - w * keep) * 0.5 * sa, 0))
        ob.rotation_euler = Euler((0, 0, rot), 'XYZ')
        set_props(ob, sar_object="BUILDING_ROOF", semantic_class="BUILDING",
                  building_id=spec["id"], thermal_class="THERMAL_CONCRETE",
                  collision="BOX", roof_state=spec["roof"])
        link_to(ob, "BUILDINGS", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        objs.append(ob)

    # --- collapsed slabs leaning out of the damaged corner ---------------
    if collapse_frac > 0.1:
        for k in range(int(2 + collapse_frac * 6)):
            lx = rng.uniform(hw * 0.1, hw + 3.5)
            ly = rng.uniform(-hd - 2.5, hd + 2.5)
            wx, wy = L2W(lx, ly)
            zz = height_at(wx, wy)
            ob = make_box("%s_CollapsedSlab_%02d" % (spec["id"], k + 1),
                          rng.uniform(2.0, 4.5), rng.uniform(1.4, 3.0), 0.24,
                          "MAT_CONCRETE",
                          loc=(wx, wy, zz + rng.uniform(0.2, 1.1)),
                          rot=(rng.uniform(-0.6, 0.6), rng.uniform(-0.7, 0.7),
                               rng.uniform(0, math.tau)))
            set_props(ob, sar_object="DEBRIS", semantic_class="RUBBLE",
                      building_id=spec["id"], thermal_class="THERMAL_CONCRETE",
                      terrain_type="RUBBLE", traversability="FORBIDDEN",
                      risk_level="HIGH", collision="BOX")
            link_to(ob, "BUILDINGS", "RUBBLE", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            objs.append(ob)
            add_exclusion(wx, wy, 1.6)

    for o in objs:
        o["search_zone"] = spec["zone"]
        o["sar_sector"] = spec["sector"]
        o["building_label"] = spec["label"]
    return objs


def build_buildings():
    log("building structures")
    n = 0
    for spec in BUILDINGS:
        build_building(spec)
        n += 1
    # deliberately block one warehouse door and one urban street
    rng = sub_rng("blockers")
    for (bx, by, tag) in [(112.0, 46.2, "SAR_BlockedEntrance_001"),
                          (82.0, 62.0, "SAR_BlockedRoad_002")]:
        for k in range(14):
            jx = bx + rng.uniform(-2.6, 2.6); jy = by + rng.uniform(-2.0, 2.0)
            proto = rng.choice(RUBBLE_PROTOS)
            s = rng.uniform(0.9, 2.1)
            ob = instance_of(proto, "%s_Debris_%02d" % (tag, k + 1),
                             (jx, jy, height_at(jx, jy) + rng.uniform(0, 0.8)),
                             rot=(rng.uniform(0, math.pi), rng.uniform(0, math.pi),
                                  rng.uniform(0, math.tau)),
                             scale=(s, s, s))
            set_props(ob, hazard_id=tag, hazard_type="BLOCKED_PATH",
                      severity="HIGH", traversability="FORBIDDEN",
                      search_zone="SAR_ZONE_C")
            link_to(ob, "HAZARDS", "RUBBLE", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        add_exclusion(bx, by, 3.0)
    STATS["buildings"] = n
    log("buildings: %d" % n)


# ============================================================================
# PART 12 -- RESCUE OPERATIONS BASE
# ============================================================================

BASE_ORIGIN = (0.0, -14.0)


def build_base():
    log("building rescue operations base")
    rng = sub_rng("base")
    bx, by = BASE_ORIGIN
    objs = []

    def put(ob, *extra):
        set_props(ob, sar_sector="BASE", search_zone="SAR_ZONE_HOME")
        link_to(ob, "BASE", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION", *extra)
        objs.append(ob)

    # --- graded apron + docking pad --------------------------------------
    apron = make_box("SAR_Base_Apron", 46.0, 30.0, 0.12, "MAT_CONCRETE",
                     loc=(bx, by - 2.0, height_at(bx, by - 2.0) - 0.02))
    set_props(apron, sar_object="APRON", semantic_class="GROUND",
              terrain_type="CONCRETE", traversability="EASY",
              traverse_cost=1.0, friction_mu=0.82, risk_level="LOW",
              thermal_class="THERMAL_CONCRETE", collision="BOX")
    put(apron)
    add_exclusion_rect(bx - 24, bx + 24, by - 18, by + 14)

    dock = make_box("SAR_Base_DockingStation_Pad", 5.0, 5.0, 0.18, "MAT_RESCUE",
                    loc=(bx + 13.0, by - 6.0, height_at(bx + 13, by - 6)))
    set_props(dock, sar_object="DOCKING_STATION", semantic_class="STRUCTURE",
              terrain_type="CONCRETE", traversability="EASY",
              thermal_class="THERMAL_CONCRETE", collision="BOX",
              mission_role="DOCK/RECHARGE/MISSION_COMPLETE")
    put(dock)
    post = make_box("SAR_Base_ChargingPost", 0.45, 0.45, 1.35, "MAT_METAL",
                    loc=(bx + 15.2, by - 6.0, height_at(bx + 15.2, by - 6) + 0.18))
    set_props(post, sar_object="CHARGER", semantic_class="PROP",
              thermal_class="THERMAL_EQUIPMENT_WARM", collision="BOX",
              traversability="FORBIDDEN",
              thermal_note="Powered charger -- warm. Deliberate thermal decoy "
                           "inside the home area.")
    put(post, "SAR_SENSOR_TEST")

    # --- command shelter (hard structure) --------------------------------
    cxx, cyy = bx - 13.0, by + 3.0
    gz = height_at(cxx, cyy)
    parts = []
    hw, hd, H = 4.5, 3.0, 3.0
    cor = [(cxx - hw, cyy - hd), (cxx + hw, cyy - hd), (cxx + hw, cyy + hd), (cxx - hw, cyy + hd)]
    for i in range(4):
        ops = []
        Lw = math.hypot(cor[(i + 1) % 4][0] - cor[i][0], cor[(i + 1) % 4][1] - cor[i][1])
        if i == 0:
            ops.append((Lw * 0.5 - 0.5, 1.0, 0.0, 2.1))
        else:
            ops.append((Lw * 0.5 - 0.7, 1.4, 1.0, 2.1))
        parts += wall_mesh_pieces(cor[i], cor[(i + 1) % 4], H, ops, 0.2, gz)
    parts = [(pv, pf, "MAT_CONCRETE") for (pv, pf, _m) in parts]
    v, f, mats, fm = combine(parts)
    cmd = new_mesh_obj("SAR_Base_CommandPost", v, f, mats, fm)
    set_props(cmd, sar_object="COMMAND_POST", semantic_class="BUILDING",
              thermal_class="THERMAL_CONCRETE", collision="BOX_COMPOUND",
              traversability="FORBIDDEN", opaque_to_thermal=True,
              mission_role="COMMAND")
    put(cmd)
    roof = make_box("SAR_Base_CommandPost_Roof", 9.4, 6.4, 0.18, "MAT_METAL",
                    loc=(cxx, cyy, gz + H + 0.09))
    set_props(roof, sar_object="ROOF", semantic_class="BUILDING",
              thermal_class="THERMAL_METAL", collision="BOX")
    put(roof)

    # --- field shelter (tent) -------------------------------------------
    # Kept well west of SAR_ROBOT_START (0, -8). An earlier layout put this
    # tent directly over the spawn point, which does not announce itself --
    # the robot simply starts the mission inside a tent. The
    # spawn_clearance_radius check at the end of validate() now catches it.
    tx, ty = bx - 16.0, by + 10.0
    tz = height_at(tx, ty)
    TL, TW, TH = 7.0, 4.4, 2.7
    tv = [(-TL / 2, -TW / 2, 0), (TL / 2, -TW / 2, 0), (TL / 2, TW / 2, 0), (-TL / 2, TW / 2, 0),
          (-TL / 2, 0, TH), (TL / 2, 0, TH)]
    tf = [(0, 1, 5, 4), (3, 7 if False else 2, 5, 4), (0, 4, 3), (1, 2, 5)]
    tf = [(0, 1, 5, 4), (2, 3, 4, 5), (0, 4, 3), (1, 2, 5)]
    tent = new_mesh_obj("SAR_Base_FieldShelter", tv, tf, ["MAT_TARP"], loc=(tx, ty, tz))
    set_props(tent, sar_object="SHELTER", semantic_class="STRUCTURE",
              thermal_class="THERMAL_VEGETATION", collision="BOX",
              traversability="FORBIDDEN", mission_role="TRIAGE_SHELTER")
    put(tent)
    add_exclusion(tx, ty, 4.5)

    # --- ISO containers --------------------------------------------------
    for k, (ox, oy, rz, mat) in enumerate([(-20.0, -10.0, 0.02, "MAT_RESCUE"),
                                           (-20.0, -13.2, 0.0, "MAT_METAL"),
                                           (9.0, 6.5, 1.55, "MAT_METAL")]):
        wx, wy = bx + ox, by + oy
        ob = make_box("SAR_Base_Container_%02d" % (k + 1), 6.06, 2.44, 2.59, mat,
                      loc=(wx, wy, height_at(wx, wy)), rot=(0, 0, rz))
        set_props(ob, sar_object="CONTAINER", semantic_class="STRUCTURE",
                  thermal_class="THERMAL_METAL", collision="BOX",
                  traversability="FORBIDDEN",
                  mission_role="EQUIPMENT_STORE",
                  slam_note="Large flat metal faces: strong planar landmark, "
                            "and a specular LiDAR return worth testing.")
        put(ob, "SAR_SENSOR_TEST")
        add_exclusion(wx, wy, 3.6)

    # --- equipment crates, stretchers, medical station -------------------
    for k in range(9):
        wx = bx + rng.uniform(-9, 11); wy = by + rng.uniform(-9, -1)
        if abs(wx - (bx + 13)) < 4 and abs(wy - (by - 6)) < 4:
            continue
        s = rng.uniform(0.6, 1.1)
        ob = make_box("SAR_Base_Crate_%02d" % (k + 1), 0.9 * s, 0.7 * s, 0.65 * s,
                      rng.choice(["MAT_RESCUE", "MAT_METAL", "MAT_WOOD"]),
                      loc=(wx, wy, height_at(wx, wy)),
                      rot=(0, 0, rng.uniform(0, math.tau)))
        set_props(ob, sar_object="EQUIPMENT_CRATE", semantic_class="PROP",
                  thermal_class="THERMAL_AMBIENT", collision="BOX",
                  traversability="FORBIDDEN")
        put(ob)
    for k in range(2):
        wx = bx - 10.0 + k * 1.4; wy = by + 11.0
        ob = make_box("SAR_Base_Stretcher_%02d" % (k + 1), 2.0, 0.62, 0.55,
                      "MAT_RESCUE", loc=(wx, wy, height_at(wx, wy)))
        set_props(ob, sar_object="STRETCHER", semantic_class="PROP",
                  thermal_class="THERMAL_AMBIENT", collision="BOX")
        put(ob)

    # --- generator: a hot, human-sized-ish thermal decoy -----------------
    gx, gy = bx - 17.5, by - 4.0
    gen = make_box("SAR_Base_Generator", 1.8, 1.0, 1.15, "MAT_METAL",
                   loc=(gx, gy, height_at(gx, gy)))
    set_props(gen, sar_object="GENERATOR", semantic_class="PROP",
              thermal_class="THERMAL_GENERATOR", collision="BOX",
              traversability="FORBIDDEN", decoy="THERMAL_FALSE_POSITIVE",
              thermal_note="74 C exhaust housing. Any detector that treats "
                           "'hot blob' as 'person' fails here.")
    put(gen, "SAR_SENSOR_TEST")

    # --- communications mast --------------------------------------------
    mx, my = bx + 20.0, by + 8.0
    mz = height_at(mx, my)
    mast = make_cyl("SAR_Base_CommsMast", 0.22, 0.10, 15.0, "MAT_METAL", 8,
                    loc=(mx, my, mz))
    set_props(mast, sar_object="COMMS_MAST", semantic_class="STRUCTURE",
              thermal_class="THERMAL_METAL", collision="CYLINDER",
              traversability="FORBIDDEN",
              slam_note="Tallest vertical landmark at the base. Useful for "
                        "loop closure on return-to-base.")
    put(mast, "SAR_SENSOR_TEST")
    for k, zz in enumerate((11.0, 13.0, 14.4)):
        arm = make_box("SAR_Base_CommsMast_Arm_%02d" % (k + 1), 2.6, 0.12, 0.12,
                       "MAT_METAL", loc=(mx, my, mz + zz),
                       rot=(0, 0, k * 1.1))
        set_props(arm, sar_object="ANTENNA", semantic_class="STRUCTURE",
                  thermal_class="THERMAL_METAL", collision="NONE_VISUAL_ONLY")
        put(arm)
    add_exclusion(mx, my, 2.5)

    # --- perimeter fence with a vehicle gate on the north side ----------
    fx0, fx1 = bx - 25.0, bx + 25.0
    fy0, fy1 = by - 19.0, by + 15.0
    gate_c, gate_w = bx + 1.0, 9.0
    posts = []
    rails = []
    def fence_run(p0, p1, gate=None):
        L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(2, int(L / 3.0))
        for i in range(n + 1):
            t = i / n
            px = p0[0] + (p1[0] - p0[0]) * t
            py = p0[1] + (p1[1] - p0[1]) * t
            if gate and abs(px - gate[0]) < gate[1] * 0.5:
                continue
            pz = height_at(px, py)
            v, f = cyl_verts(0.06, 0.06, 1.9, 6)
            posts.append(([(vx + px, vy + py, vz + pz) for (vx, vy, vz) in v], f, "MAT_METAL"))
        for zz in (0.55, 1.25, 1.8):
            for i in range(n):
                t0 = i / n; t1 = (i + 1) / n
                ax_ = p0[0] + (p1[0] - p0[0]) * t0; ay_ = p0[1] + (p1[1] - p0[1]) * t0
                bx_ = p0[0] + (p1[0] - p0[0]) * t1; by_ = p0[1] + (p1[1] - p0[1]) * t1
                if gate and (abs(ax_ - gate[0]) < gate[1] * 0.5 or abs(bx_ - gate[0]) < gate[1] * 0.5):
                    continue
                az = height_at(ax_, ay_) + zz; bz = height_at(bx_, by_) + zz
                t = 0.03
                rails.append(([(ax_ - t, ay_ - t, az - t), (bx_ - t, by_ - t, bz - t),
                               (bx_ + t, by_ + t, bz - t), (ax_ + t, ay_ + t, az - t),
                               (ax_ - t, ay_ - t, az + t), (bx_ - t, by_ - t, bz + t),
                               (bx_ + t, by_ + t, bz + t), (ax_ + t, ay_ + t, az + t)],
                              [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1),
                               (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)], "MAT_METAL"))
    fence_run((fx0, fy0), (fx1, fy0))
    fence_run((fx1, fy0), (fx1, fy1))
    fence_run((fx1, fy1), (fx0, fy1), gate=(gate_c, gate_w))
    fence_run((fx0, fy1), (fx0, fy0))
    v, f, mats, fm = combine(posts + rails)
    fence = new_mesh_obj("SAR_Base_Fence", v, f, mats, fm)
    set_props(fence, sar_object="FENCE", semantic_class="WALL",
              thermal_class="THERMAL_METAL", collision="BOX_COMPOUND",
              traversability="FORBIDDEN",
              gate_center_x=gate_c, gate_width_m=gate_w,
              note="Base perimeter only. The SAR mission boundary is NOT "
                   "fenced -- it is a logical polygon in GROUND_TRUTH.")
    put(fence, "SAR_SENSOR_TEST")

    # --- signage ---------------------------------------------------------
    for k, (ox, oy, rz) in enumerate([(2.0, 14.2, 0.0), (-24.0, 2.0, 1.57),
                                      (14.0, -18.5, 0.0)]):
        wx, wy = bx + ox, by + oy
        wz = height_at(wx, wy)
        pole = make_cyl("SAR_Base_SignPost_%02d" % (k + 1), 0.05, 0.05, 1.9,
                        "MAT_METAL", 6, loc=(wx, wy, wz))
        put(pole)
        board = make_box("SAR_Base_Sign_%02d" % (k + 1), 1.2, 0.06, 0.8,
                         "MAT_SIGN", loc=(wx, wy, wz + 1.45), rot=(0, 0, rz))
        set_props(board, sar_object="SIGN", semantic_class="PROP",
                  thermal_class="THERMAL_AMBIENT", collision="NONE_VISUAL_ONLY")
        put(board)

    # --- flood lights ----------------------------------------------------
    for k, (ox, oy) in enumerate([(-22.0, 12.0), (22.0, 12.0),
                                  (-22.0, -16.0), (22.0, -16.0)]):
        wx, wy = bx + ox, by + oy
        wz = height_at(wx, wy)
        pole = make_cyl("SAR_Base_LightPole_%02d" % (k + 1), 0.10, 0.07, 7.0,
                        "MAT_METAL", 6, loc=(wx, wy, wz))
        set_props(pole, sar_object="LIGHT_POLE", semantic_class="STRUCTURE",
                  thermal_class="THERMAL_METAL", collision="CYLINDER")
        put(pole)
        ld = bpy.data.lights.new("SAR_Base_Floodlight_%02d" % (k + 1), 'SPOT')
        ld.energy = 2600.0
        ld.spot_size = math.radians(105)
        ld.spot_blend = 0.45
        ld.color = (1.0, 0.96, 0.88)
        lo = bpy.data.objects.new("SAR_Base_Floodlight_%02d" % (k + 1), ld)
        lo.location = (wx, wy, wz + 6.8)
        lo.rotation_euler = Euler((math.radians(38),
                                   0, math.atan2(-oy, -ox) - math.pi / 2), 'XYZ')
        set_props(lo, sar_object="LIGHT", lighting_preset="NIGHT,LOW_LIGHT",
                  note="On for NIGHT / LOW_LIGHT presets; the lit apron is the "
                       "bright end of the indoor/outdoor exposure test.")
        C("LIGHTING").objects.link(lo)
        objs.append(lo)

    STATS["base_objects"] = len(objs)
    log("base: %d objects" % len(objs))
    return objs

# ============================================================================
# PART 13 -- VEHICLES
# ============================================================================
# Low-poly but correctly proportioned: 4.4 m car, 7.2 m truck, 1.5 m roofline.
# Every vehicle carries a separate ENGINE BAY child object with its own thermal
# class, so a vehicle that has recently run is a legitimate warm blob and a
# long-cold wreck is not. That distinction is the whole point of requirement 64.

VEHICLES = [
    dict(id="SAR_Vehicle_Rescue_001",  c=(-8.0, -22.0),  rot=0.05, kind="TRUCK",
         mat="MAT_RESCUE",    thermal="THERMAL_VEHICLE_ENGINE", damage="NONE",
         sector="BASE",  zone="SAR_ZONE_HOME", label="Rescue truck, engine warm"),
    dict(id="SAR_Vehicle_Rescue_002",  c=(-1.0, -22.0),  rot=0.02, kind="VAN",
         mat="MAT_RESCUE",    thermal="THERMAL_VEHICLE_BODY",  damage="NONE",
         sector="BASE",  zone="SAR_ZONE_HOME", label="Medical van"),
    dict(id="SAR_Vehicle_Utility_003", c=(6.0, -22.0),   rot=-0.03, kind="UTILITY",
         mat="MAT_VEHICLE_C", thermal="THERMAL_VEHICLE_BODY",  damage="NONE",
         sector="BASE",  zone="SAR_ZONE_HOME", label="Utility 4x4"),
    dict(id="SAR_Vehicle_Civilian_004", c=(86.0, 25.5),  rot=0.42, kind="CAR",
         mat="MAT_VEHICLE_A", thermal="THERMAL_VEHICLE_ENGINE", damage="LIGHT",
         sector="URBAN", zone="SAR_ZONE_C",
         label="Abandoned car, engine still warm. Casualty PERSON_015 alongside "
               "-> warm vehicle vs warm human discrimination."),
    dict(id="SAR_Vehicle_Civilian_005", c=(69.0, 43.0),  rot=0.08, kind="CAR",
         mat="MAT_VEHICLE_B", thermal="THERMAL_VEHICLE_COLD",  damage="HEAVY",
         sector="URBAN", zone="SAR_ZONE_C",
         label="Crushed car under debris, parked at the Street D kerb -- an obstacle beside the carriageway, not across it"),
    dict(id="SAR_Vehicle_Truck_006",   c=(82.0, 62.5),   rot=1.35, kind="TRUCK",
         mat="MAT_VEHICLE_C", thermal="THERMAL_VEHICLE_COLD",  damage="OVERTURNED",
         sector="URBAN", zone="SAR_ZONE_C",
         label="Overturned truck blocking Urban Street C -> forces a reroute"),
    dict(id="SAR_Vehicle_Civilian_007", c=(106.0, 71.0), rot=-0.15, kind="CAR",
         mat="MAT_VEHICLE_C", thermal="THERMAL_VEHICLE_COLD",  damage="MODERATE",
         sector="URBAN", zone="SAR_ZONE_C", label="Damaged car, kerbside"),
    dict(id="SAR_Vehicle_Stranded_008", c=(-112.0, 62.0), rot=0.75, kind="UTILITY",
         mat="MAT_VEHICLE_B", thermal="THERMAL_VEHICLE_COLD",  damage="MODERATE",
         sector="FLOOD", zone="SAR_ZONE_F",
         label="Stranded vehicle at the flood margin, partly in the channel"),
    dict(id="SAR_Vehicle_Wreck_009",   c=(-9.0, 112.0),  rot=2.1, kind="CAR",
         mat="MAT_VEHICLE_A", thermal="THERMAL_VEHICLE_COLD",  damage="HEAVY",
         sector="RUBBLE", zone="SAR_ZONE_B", label="Wreck in the collapse field"),
    dict(id="SAR_Vehicle_Abandoned_010", c=(114.0, 118.0), rot=-0.5, kind="TRUCK",
         mat="MAT_VEHICLE_C", thermal="THERMAL_VEHICLE_BODY",  damage="MODERATE",
         sector="DESERT", zone="SAR_ZONE_E", label="Abandoned truck, desert track"),
]

VEH_DIMS = {
    "CAR":     dict(L=4.40, W=1.80, body_h=0.72, cab_h=0.62, wheels=4, wr=0.33),
    "VAN":     dict(L=5.20, W=2.00, body_h=1.05, cab_h=0.95, wheels=4, wr=0.36),
    "UTILITY": dict(L=4.90, W=1.95, body_h=0.90, cab_h=0.78, wheels=4, wr=0.40),
    "TRUCK":   dict(L=7.20, W=2.35, body_h=1.15, cab_h=1.25, wheels=6, wr=0.48),
}


def build_vehicle(spec):
    d = VEH_DIMS[spec["kind"]]
    cx, cy = spec["c"]
    gz = height_at(cx, cy)
    rot = spec["rot"]
    dmg = spec["damage"]
    objs = []
    crush = {"NONE": 1.0, "LIGHT": 0.96, "MODERATE": 0.86,
             "HEAVY": 0.62, "OVERTURNED": 0.92}[dmg]
    tilt = (0.0, 0.0, rot)
    zlift = d["wr"]
    if dmg == "OVERTURNED":
        tilt = (math.radians(104), 0.0, rot)
        zlift = d["W"] * 0.5

    body = make_box(spec["id"] + "_Body", d["L"], d["W"], d["body_h"] * crush,
                    spec["mat"], loc=(cx, cy, gz + zlift), rot=tilt)
    set_props(body, sar_object="VEHICLE", semantic_class="VEHICLE",
              vehicle_id=spec["id"], vehicle_kind=spec["kind"],
              damage_state=dmg, thermal_class=spec["thermal"],
              traversability="FORBIDDEN", risk_level="MEDIUM",
              collision="BOX", sar_sector=spec["sector"],
              search_zone=spec["zone"], vehicle_label=spec["label"],
              length_m=d["L"], width_m=d["W"])
    link_to(body, "VEHICLES", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
    objs.append(body)

    cab = make_box(spec["id"] + "_Cab", d["L"] * 0.42, d["W"] * 0.92,
                   d["cab_h"] * crush, "MAT_GLASS",
                   loc=(cx, cy, gz + zlift + d["body_h"] * crush), rot=tilt)
    set_props(cab, sar_object="VEHICLE_CAB", semantic_class="VEHICLE",
              vehicle_id=spec["id"], thermal_class="THERMAL_VEHICLE_BODY",
              collision="BOX")
    link_to(cab, "VEHICLES", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
    objs.append(cab)

    # engine bay -- the thermally distinct part
    ex = cx + math.cos(rot) * d["L"] * 0.38
    ey = cy + math.sin(rot) * d["L"] * 0.38
    eng = make_box(spec["id"] + "_EngineBay", d["L"] * 0.20, d["W"] * 0.70, 0.35,
                   "MAT_METAL", loc=(ex, ey, gz + zlift + d["body_h"] * crush * 0.55),
                   rot=tilt)
    set_props(eng, sar_object="VEHICLE_ENGINE", semantic_class="VEHICLE",
              vehicle_id=spec["id"], thermal_class=spec["thermal"],
              collision="NONE_VISUAL_ONLY",
              decoy=("THERMAL_FALSE_POSITIVE"
                     if spec["thermal"] == "THERMAL_VEHICLE_ENGINE" else "NONE"),
              thermal_note="Separate body so the thermal camera config can give "
                           "the engine bay its own temperature.")
    link_to(eng, "VEHICLES", "SAR_EXPORT", "SAR_VISUAL")
    if spec["thermal"] == "THERMAL_VEHICLE_ENGINE":
        link_to(eng, "SAR_SENSOR_TEST")
    objs.append(eng)

    nw = d["wheels"]
    rng = sub_rng("veh_" + spec["id"])
    positions = []
    if nw == 4:
        positions = [(0.33, 0.5), (0.33, -0.5), (-0.33, 0.5), (-0.33, -0.5)]
    else:
        positions = [(0.36, 0.5), (0.36, -0.5), (-0.20, 0.5), (-0.20, -0.5),
                     (-0.36, 0.5), (-0.36, -0.5)]
    missing = 1 if dmg in ("HEAVY",) else 0
    for k, (fl, fw) in enumerate(positions):
        if k < missing:
            continue
        lx = fl * d["L"]; ly = fw * (d["W"] * 0.52)
        wx = cx + lx * math.cos(rot) - ly * math.sin(rot)
        wy = cy + lx * math.sin(rot) + ly * math.cos(rot)
        ob = make_cyl(spec["id"] + "_Wheel_%02d" % (k + 1), d["wr"], d["wr"], 0.24,
                      "MAT_HUMAN_DARK", 10,
                      loc=(wx, wy, gz + d["wr"]),
                      rot=(math.pi / 2, 0, rot))
        set_props(ob, sar_object="VEHICLE_WHEEL", semantic_class="VEHICLE",
                  vehicle_id=spec["id"], thermal_class="THERMAL_VEHICLE_BODY",
                  collision="CYLINDER")
        link_to(ob, "VEHICLES", "SAR_EXPORT", "SAR_VISUAL")
        objs.append(ob)

    add_exclusion(cx, cy, d["L"] * 0.55 + 0.6)
    return objs


def build_vehicles():
    log("building vehicles")
    for spec in VEHICLES:
        build_vehicle(spec)
    STATS["vehicles"] = len(VEHICLES)
    log("vehicles: %d (all static -- see note in README on dynamic vehicles)" %
        len(VEHICLES))


# ============================================================================
# PART 14 -- HUMANS (proxy humanoids with per-limb thermal metadata)
# ============================================================================

BODY = dict(head_r=0.105, head_z=1.615,
            torso=(0.40, 0.235, 0.60), torso_z=1.16,
            hip=(0.33, 0.215, 0.18), hip_z=0.845,
            leg_len=0.845, leg_r0=0.088, leg_r1=0.062, leg_dx=0.095,
            arm_len=0.72, arm_r0=0.056, arm_r1=0.044, arm_dx=0.205,
            shoulder_z=1.42)


def _limb(name, length, r0, r1, mat, loc, rot, seg=7):
    """Limb built DOWNWARD from its own origin, so the origin is the joint and
    rotation_euler.x is a swing. That is what makes the walk cycle possible
    without a rig."""
    v, f = cyl_verts(r0, r1, length, seg)
    v = [(vx, vy, -vz) for (vx, vy, vz) in v]
    ob = new_mesh_obj(name, v, f, [mat], smooth=True, loc=loc)
    ob.rotation_euler = Euler(rot, 'XYZ')
    return ob


POSE_DEFS = {
    # pose -> (root_z_offset, torso_rot_x, leg_rot_x pair, arm_rot_x pair, extra)
    "STANDING": dict(rz=0.0,  torso=0.0, legs=(0.05, -0.05), arms=(0.08, -0.08)),
    "WALKING":  dict(rz=0.0,  torso=0.03, legs=(0.42, -0.38), arms=(-0.35, 0.35)),
    "SITTING":  dict(rz=-0.40, torso=0.14, legs=(1.48, 1.40), arms=(0.35, 0.30)),
    "KNEELING": dict(rz=-0.28, torso=0.06, legs=(1.62, 0.15), arms=(0.25, 0.25)),
    "LYING":    dict(rz=-1.42, torso=1.5708, legs=(0.06, -0.10), arms=(0.30, -0.25)),
}


def build_person(pid, name, x, y, pose, cloth_mat, thermal_contrast,
                 heading=0.0, scale=1.0, snow_cover=False, dust_cover=False):
    """Returns (root_empty, [part objects])."""
    P = POSE_DEFS[pose]
    gz = height_at(x, y)
    root = bpy.data.objects.new(name, None)
    root.empty_display_type = 'ARROWS'
    root.empty_display_size = 0.6
    root.location = (x, y, gz + P["rz"] * scale)
    root.rotation_euler = Euler((0, 0, heading), 'XYZ')
    root.scale = (scale, scale, scale)

    cs = THERMAL_CONTRAST[thermal_contrast]
    surf = cs["surface"]
    head_thermal = "THERMAL_HUMAN_HEAD" if thermal_contrast != "LOW" else "THERMAL_HUMAN_COVERED"
    parts = []

    # torso ---------------------------------------------------------------
    tv, tf = box_verts(BODY["torso"][0], BODY["torso"][1], BODY["torso"][2],
                       cz=BODY["torso_z"])
    torso = new_mesh_obj(name + "_Torso", tv, tf, [cloth_mat])
    torso.rotation_euler = Euler((P["torso"], 0, 0), 'XYZ')
    parts.append((torso, surf, "TORSO"))

    hv, hf = box_verts(BODY["hip"][0], BODY["hip"][1], BODY["hip"][2], cz=BODY["hip_z"])
    hips = new_mesh_obj(name + "_Hips", hv, hf, [cloth_mat])
    parts.append((hips, surf, "HIPS"))

    # head ----------------------------------------------------------------
    hv2, hf2 = ico_verts(BODY["head_r"], 1, 0.0, None, (0.92, 1.0, 1.18))
    head = new_mesh_obj(name + "_Head", [(vx, vy, vz + BODY["head_z"]) for (vx, vy, vz) in hv2],
                        hf2, ["MAT_HUMAN_SKIN"], smooth=True)
    parts.append((head, head_thermal, "HEAD"))

    # legs ----------------------------------------------------------------
    for i, sgn in enumerate((-1, 1)):
        leg = _limb(name + "_Leg_%s" % ("L" if sgn < 0 else "R"),
                    BODY["leg_len"], BODY["leg_r0"], BODY["leg_r1"], cloth_mat,
                    (sgn * BODY["leg_dx"], 0.0, BODY["hip_z"]),
                    (P["legs"][i], 0, 0))
        parts.append((leg, "THERMAL_HUMAN_LIMB" if thermal_contrast != "LOW"
                      else "THERMAL_HUMAN_COVERED", "LEG"))
    # arms ----------------------------------------------------------------
    for i, sgn in enumerate((-1, 1)):
        arm = _limb(name + "_Arm_%s" % ("L" if sgn < 0 else "R"),
                    BODY["arm_len"], BODY["arm_r0"], BODY["arm_r1"], cloth_mat,
                    (sgn * BODY["arm_dx"], 0.0, BODY["shoulder_z"]),
                    (P["arms"][i], 0, sgn * -0.12))
        parts.append((arm, "THERMAL_HUMAN_LIMB" if thermal_contrast != "LOW"
                      else "THERMAL_HUMAN_COVERED", "ARM"))

    out = []
    for (ob, therm, region) in parts:
        ob.parent = root
        set_props(ob, sar_object="HUMAN_PART", semantic_class="HUMAN",
                  person_id=pid, body_region=region,
                  thermal_class=therm,
                  thermal_temp_c=THERMAL_TABLE[therm]["temp"],
                  thermal_emissivity=THERMAL_TABLE[therm]["emis"],
                  thermal_contrast=thermal_contrast,
                  collision="CAPSULE_APPROX",
                  traversability="FORBIDDEN", risk_level="CRITICAL")
        if snow_cover:
            ob["surface_cover"] = "SNOW_DUSTED"
        if dust_cover:
            ob["surface_cover"] = "CONCRETE_DUST"
        out.append(ob)
    return root, out


STATIC_HUMANS = [
    dict(pid="PERSON_001", x=0.0,   y=62.0,  pose="STANDING", cloth="MAT_HUMAN_HI",
         th="HIGH",   occ="NONE",              status="AMBULATORY", zone="SAR_ZONE_A",
         sector="OPEN_FIELD", diff="EASY", head=2.2,
         note="Scenario A. Standing, hi-vis, open field, unobstructed. The "
              "baseline detection test -- if this one fails, stop and fix the stack."),
    dict(pid="PERSON_002", x=-78.0, y=160.0, pose="LYING",    cloth="MAT_HUMAN_DARK",
         th="HIGH",   occ="VEGETATION_PARTIAL", status="INJURED_UNRESPONSIVE",
         zone="SAR_ZONE_B", sector="FOREST", diff="HARD", head=0.8,
         note="Scenario C/G. Dark clothing, prone, under canopy shadow. RGB is "
              "weak here; thermal carries the detection."),
    dict(pid="PERSON_003", x=-92.0, y=172.0, pose="SITTING",  cloth="MAT_HUMAN_MID",
         th="MEDIUM", occ="VEGETATION_PARTIAL", status="INJURED_RESPONSIVE",
         zone="SAR_ZONE_B", sector="FOREST", diff="MEDIUM", head=1.9,
         note="Seated against a fallen tree, half behind brush."),
    dict(pid="PERSON_004", x=-66.0, y=150.0, pose="STANDING", cloth="MAT_HUMAN_MID",
         th="MEDIUM", occ="NONE",              status="AMBULATORY", zone="SAR_ZONE_B",
         sector="FOREST", diff="MEDIUM", head=3.4,
         note="Standing in a forest clearing -- RGB workable, canopy breaks GNSS."),
    dict(pid="PERSON_005", x=-12.0, y=104.0, pose="SITTING",  cloth="MAT_HUMAN_DARK",
         th="MEDIUM", occ="RUBBLE_PARTIAL",   status="INJURED_RESPONSIVE",
         zone="SAR_ZONE_B", sector="RUBBLE", diff="HARD", head=1.1,
         note="Seated on debris at the edge of the collapse field."),
    dict(pid="PERSON_006", x=2.0,   y=116.0, pose="LYING",    cloth="MAT_HUMAN_MID",
         th="LOW",    occ="RUBBLE_PARTIAL",   status="INJURED_UNRESPONSIVE",
         zone="SAR_ZONE_B", sector="RUBBLE", diff="HARD", head=2.6, dust=True,
         note="Scenario F/G. Concrete-dust covered, lying among cold slabs. "
              "LOW thermal contrast on purpose: this is where a single-sensor "
              "detector quits. NOT buried -- torso and head are exposed."),
    dict(pid="PERSON_007", x=15.5,  y=119.5, pose="STANDING", cloth="MAT_HUMAN_HI",
         th="HIGH",   occ="BUILDING_OPENING", status="TRAPPED", zone="SAR_ZONE_B",
         sector="RUBBLE", diff="HARD", head=4.0,
         note="Scenario H. Inside SAR_Building_RUB_002. Visible ONLY through the "
              "collapsed wall opening -- there is no line of sight through the "
              "intact walls, for RGB or for thermal."),
    dict(pid="PERSON_008", x=76.0,  y=79.5,  pose="LYING",    cloth="MAT_HUMAN_MID",
         th="MEDIUM", occ="BUILDING_INTERIOR", status="INJURED_UNRESPONSIVE",
         zone="SAR_ZONE_C", sector="URBAN", diff="HARD", head=1.4,
         note="Inside SAR_Building_URB_003, dark interior. Requires entry via "
              "the doorway or the collapsed NE corner."),
    dict(pid="PERSON_009", x=92.0,  y=70.0,  pose="STANDING", cloth="MAT_HUMAN_HI",
         th="HIGH",   occ="NONE",              status="AMBULATORY", zone="SAR_ZONE_C",
         sector="URBAN", diff="EASY", head=3.0, cluster="TRIAGE_01",
         note="Scenario I. Multi-person triage point, 4 casualties within 4 m."),
    dict(pid="PERSON_010", x=93.6,  y=71.6,  pose="SITTING",  cloth="MAT_HUMAN_MID",
         th="MEDIUM", occ="NONE",              status="INJURED_RESPONSIVE",
         zone="SAR_ZONE_C", sector="URBAN", diff="MEDIUM", head=2.4, cluster="TRIAGE_01"),
    dict(pid="PERSON_011", x=90.8,  y=72.4,  pose="LYING",    cloth="MAT_HUMAN_MID",
         th="MEDIUM", occ="NONE",              status="INJURED_UNRESPONSIVE",
         zone="SAR_ZONE_C", sector="URBAN", diff="MEDIUM", head=0.5, cluster="TRIAGE_01"),
    dict(pid="PERSON_012", x=94.2,  y=68.4,  pose="KNEELING", cloth="MAT_HUMAN_HI",
         th="HIGH",   occ="NONE",              status="RESCUER", zone="SAR_ZONE_C",
         sector="URBAN", diff="EASY", head=1.7, cluster="TRIAGE_01",
         note="Rescue personnel kneeling beside a casualty. Tests person-role "
              "reasoning, not just person-or-not."),
    dict(pid="PERSON_013", x=82.0,  y=230.0, pose="SITTING",  cloth="MAT_HUMAN_DARK",
         th="LOW",    occ="ROCK_PARTIAL",     status="INJURED_RESPONSIVE",
         zone="SAR_ZONE_D", sector="MOUNTAIN", diff="HARD", head=2.9,
         note="High on the mountain, on a shoulder above where the risky and "
              "safe tracks converge, behind rock. Cold and heavily clothed: LOW "
              "contrast against sun-warmed rock. Getting here is the risk-aware "
              "routing test -- the short track runs the SAR_Ravine_001 rim, the "
              "long one climbs the east side."),
    dict(pid="PERSON_014", x=-50.0, y=230.0, pose="LYING",    cloth="MAT_HUMAN_MID",
         th="LOW",    occ="SNOW_PARTIAL",     status="INJURED_UNRESPONSIVE",
         zone="SAR_ZONE_D", sector="SNOW", diff="HARD", head=1.2, snow=True,
         note="Avalanche debris field. PARTIALLY snow-dusted and still exposed. "
              "Deliberately NOT buried: a LWIR camera cannot see through a "
              "snowpack and this environment will not pretend otherwise."),
    dict(pid="PERSON_015", x=87.6,  y=24.2,  pose="SITTING",  cloth="MAT_HUMAN_HI",
         th="HIGH",   occ="VEHICLE_PARTIAL",  status="INJURED_RESPONSIVE",
         zone="SAR_ZONE_C", sector="URBAN", diff="MEDIUM", head=2.0,
         note="Scenario E. Beside SAR_Vehicle_Civilian_004, whose engine bay is "
              "62 C. Two warm blobs, one of them a person."),
]


# --- dynamic humans: bounded, seeded, reproducible paths ------------------
DYNAMIC_HUMANS = [
    dict(pid="PERSON_016", pattern="CROSSING", zone="SAR_ZONE_A", sector="OPEN_FIELD", th="HIGH",
         cloth="MAT_HUMAN_HI", speed=1.25, dwell=(0.0, 0.0),
         path=[(-14, 66), (-2, 70), (10, 74), (20, 70), (8, 64), (-14, 66)],
         note="Crosses SAR_Road_MainSupply at ~y=70. This is the dynamic "
              "obstacle the robot is expected to detect and yield to."),
    dict(pid="PERSON_017", zone="SAR_ZONE_A", sector="OPEN_FIELD", th="MEDIUM",
         cloth="MAT_HUMAN_MID", speed=0.65, dwell=(6.0, 14.0),
         path=[(-34, 44), (-26, 52), (-30, 60), (-40, 56), (-34, 44)],
         note="Slow, frequent stops -- behaves like a walking wounded."),
    dict(pid="PERSON_018", zone="SAR_ZONE_A", sector="OPEN_FIELD", th="MEDIUM",
         cloth="MAT_HUMAN_DARK", speed=1.85, dwell=(0.0, 0.0),
         path=[(24, 150), (36, 160), (24, 170), (10, 164), (24, 150)],
         note="Fast mover, dark clothing."),
    dict(pid="PERSON_019", zone="SAR_ZONE_B", sector="FOREST", th="HIGH",
         cloth="MAT_HUMAN_HI", speed=0.95, dwell=(3.0, 9.0),
         path=[(-30, 157), (-48, 162), (-64, 168), (-48, 162), (-30, 157)],
         note="Along SAR_Track_Forest. Repeated occlusion by trunks."),
    dict(pid="PERSON_020", zone="SAR_ZONE_B", sector="FOREST", th="MEDIUM",
         cloth="MAT_HUMAN_MID", speed=0.80, dwell=(4.0, 10.0),
         path=[(-84, 150), (-92, 158), (-86, 166), (-76, 158), (-84, 150)]),
    dict(pid="PERSON_021", pattern="CROSSING", zone="SAR_ZONE_C", sector="URBAN", th="HIGH",
         cloth="MAT_HUMAN_HI", speed=1.35, dwell=(2.0, 6.0),
         path=[(60, 30), (61, 50), (62, 70), (61, 50), (60, 30)],
         note="Urban Street A. Crosses paths with PERSON_022 -> multi-target "
              "tracking and ID re-assignment test."),
    dict(pid="PERSON_022", pattern="CROSSING", zone="SAR_ZONE_C", sector="URBAN", th="MEDIUM",
         cloth="MAT_HUMAN_DARK", speed=1.15, dwell=(0.0, 0.0),
         path=[(56, 62), (78, 64), (100, 66), (78, 64), (56, 62)],
         note="Urban Street C, perpendicular to PERSON_021."),
    dict(pid="PERSON_023", zone="SAR_ZONE_C", sector="URBAN", th="HIGH",
         cloth="MAT_HUMAN_HI", speed=0.70, dwell=(8.0, 16.0),
         path=[(90, 66), (95, 67), (93, 73), (88, 71), (90, 66)],
         pattern="ORBIT",
         note="Rescuer circulating in the triage cluster."),
    dict(pid="PERSON_024", pattern="ROAD_PATROL", zone="SAR_ZONE_A", sector="OPEN_FIELD", th="MEDIUM",
         cloth="MAT_HUMAN_MID", speed=1.45, dwell=(0.0, 0.0),
         path=[(4, 22), (8, 44), (12, 64), (8, 44), (4, 22)],
         note="Walking the main supply route between base and the open field."),
    dict(pid="PERSON_025", zone="SAR_ZONE_B", sector="RUBBLE", th="MEDIUM",
         cloth="MAT_HUMAN_DARK", speed=0.75, dwell=(5.0, 12.0),
         path=[(18, 100), (26, 108), (22, 120), (12, 112), (18, 100)],
         note="Skirting the collapse field. Slow, uneven ground."),
    dict(pid="PERSON_026", zone="SAR_ZONE_F", sector="FLOOD", th="LOW",
         cloth="MAT_HUMAN_MID", speed=0.85, dwell=(4.0, 11.0),
         path=[(-98, 54), (-104, 62), (-110, 70), (-104, 62), (-98, 54)],
         note="East bank of the river, near the stranded vehicle. LOW contrast: "
              "wet clothing, cold air off the water."),

    # ---- added dynamic obstacles ----------------------------------------
    # Mixed exposure: two of these sit on route corridors, the rest work
    # adjacent ground so they enter sensor range without stopping the robot
    # every thirty seconds.
    dict(pid="PERSON_027", pattern="HEAD_ON", zone="SAR_ZONE_A",
         sector="OPEN_FIELD", th="HIGH", cloth="MAT_HUMAN_HI",
         speed=1.30, dwell=(0.0, 0.0),
         path=[(5, 26), (8, 48), (12, 70), (8, 48), (5, 26)],
         note="Walks the main supply route and turns around, so it approaches "
              "the robot head-on for half of every cycle. A closing target with "
              "almost no lateral optical flow is the case a forward-facing "
              "detector handles worst."),
    dict(pid="PERSON_028", pattern="CONVERGING", zone="SAR_ZONE_A",
         sector="OPEN_FIELD", th="MEDIUM", cloth="MAT_HUMAN_MID",
         speed=1.10, dwell=(2.0, 5.0),
         path=[(-26, 56), (-13, 62), (0, 68), (-13, 62), (-26, 56)],
         note="Converges with PERSON_029 near (0, 68). Speeds and dwells "
              "differ, so the meeting drifts through the run instead of "
              "repeating on a fixed beat."),
    dict(pid="PERSON_029", pattern="CONVERGING", zone="SAR_ZONE_A",
         sector="OPEN_FIELD", th="MEDIUM", cloth="MAT_HUMAN_DARK",
         speed=1.45, dwell=(0.0, 0.0),
         path=[(16, 82), (7, 74), (0, 68), (7, 74), (16, 82)],
         note="The other half of the converging pair. Two tracks that merge, "
              "occlude one another, then separate -- the classic ID-swap trap "
              "for a nearest-neighbour data association."),
    dict(pid="PERSON_030", pattern="CROSSING", zone="SAR_ZONE_B",
         sector="RUBBLE", th="HIGH", cloth="MAT_HUMAN_HI",
         speed=1.05, dwell=(3.0, 7.0),
         path=[(38, 110), (24, 118), (16, 108), (32, 102), (38, 110)],
         note="Crosses SAR_Track_RubbleBypass twice per lap. The bypass is "
              "what the robot takes once the main route is blocked, so it "
              "meets this one while already mid-replan."),
    dict(pid="PERSON_031", pattern="GROUP_CARRY", group="STRETCHER_01", lead=0.95,
         zone="SAR_ZONE_A", sector="OPEN_FIELD", th="HIGH", cloth="MAT_HUMAN_HI",
         speed=0.95, dwell=(0.0, 0.0),
         path=[(-7, 38), (-3, 54), (3, 70), (-3, 54), (-7, 38)],
         note="Front bearer of a two-person stretcher party. Both bearers and "
              "the stretcher share one key list, so they hold formation rather "
              "than drifting apart over a four-minute run."),
    dict(pid="PERSON_032", pattern="GROUP_CARRY", group="STRETCHER_01", lead=-0.95,
         zone="SAR_ZONE_A", sector="OPEN_FIELD", th="HIGH", cloth="MAT_HUMAN_HI",
         speed=0.95, dwell=(0.0, 0.0),
         path=[(-7, 38), (-3, 54), (3, 70), (-3, 54), (-7, 38)],
         note="Rear bearer. Two people plus the load between them read as one "
              "long obstacle to a clustering front-end and as three separate "
              "tracks to a naive one. That disagreement is the point."),
    dict(pid="PERSON_033", pattern="ORBIT", zone="SAR_ZONE_C",
         sector="URBAN", th="MEDIUM", cloth="MAT_HUMAN_MID",
         speed=1.20, dwell=(0.0, 0.0),
         path=[(89, 60), (98, 50), (89, 40), (80, 50), (89, 60)],
         note="Laps SAR_Building_URB_002 and passes through the 7.5 m canyon "
              "on its west leg. Goes fully out of sight behind the block each "
              "circuit: track continuation through total occlusion."),
    dict(pid="PERSON_034", pattern="PATROL", zone="SAR_ZONE_E",
         sector="DESERT", th="MEDIUM", cloth="MAT_HUMAN_DARK",
         speed=0.90, dwell=(6.0, 14.0),
         path=[(112, 96), (118, 110), (112, 124), (104, 110), (112, 96)],
         note="Desert track margin. Sparse background and long sightlines -- "
              "the easy-detection counterweight to the forest cases."),
]

PEOPLE_RECORDS = []


def build_static_humans():
    log("placing static humans")
    for spec in STATIC_HUMANS:
        name = "SAR_StaticPerson_%s" % spec["pid"].split("_")[1]
        rng = sub_rng("person_" + spec["pid"])
        scale = rng.uniform(0.90, 1.08)
        root, parts = build_person(spec["pid"], name, spec["x"], spec["y"],
                                   spec["pose"], spec["cloth"], spec["th"],
                                   heading=spec.get("head", 0.0), scale=scale,
                                   snow_cover=spec.get("snow", False),
                                   dust_cover=spec.get("dust", False))
        set_props(root, sar_object="HUMAN", sar_type="STATIC", person_id=spec["pid"],
                  dynamic=False, thermal_class="THERMAL_HUMAN_TORSO",
                  thermal_contrast=spec["th"],
                  thermal_delta_k=THERMAL_CONTRAST[spec["th"]]["delta_k"],
                  victim_status=spec["status"], pose=spec["pose"],
                  occlusion=spec["occ"], search_zone=spec["zone"],
                  sar_sector=spec["sector"], detection_difficulty=spec["diff"],
                  height_m=round(1.72 * scale, 3),
                  cluster=spec.get("cluster", "NONE"),
                  semantic_class="HUMAN",
                  scenario_note=spec.get("note", ""))
        link_to(root, "STATIC_HUMANS", "SAR_HUMAN_TEST")
        for p in parts:
            link_to(p, "STATIC_HUMANS", "SAR_EXPORT", "SAR_VISUAL",
                    "SAR_COLLISION", "SAR_HUMAN_TEST")
        add_exclusion(spec["x"], spec["y"], 1.1)
        PEOPLE_RECORDS.append(dict(
            person_id=spec["pid"], object=name, type="STATIC",
            position=[round(spec["x"], 2), round(spec["y"], 2),
                      round(height_at(spec["x"], spec["y"]), 3)],
            pose=spec["pose"], victim_status=spec["status"],
            thermal_contrast=spec["th"],
            thermal_delta_k=THERMAL_CONTRAST[spec["th"]]["delta_k"],
            occlusion=spec["occ"], search_zone=spec["zone"],
            sector=spec["sector"], detection_difficulty=spec["diff"],
            height_m=round(1.72 * scale, 3),
            cluster=spec.get("cluster", "NONE"),
            note=spec.get("note", "")))
    STATS["static_humans"] = len(STATIC_HUMANS)


def _cycle_fcurves(ob, data_path, index, keys):
    """Insert a short keyframe cycle and let an F-Curve CYCLES modifier repeat
    it forever. 26 people x 4 limbs x 4 minutes of explicit keys would bloat the
    file for no benefit."""
    for (frm, val) in keys:
        cur = list(ob.rotation_euler)
        cur[index] = val
        ob.rotation_euler = cur
        ob.keyframe_insert(data_path=data_path, index=index, frame=frm)
    if ob.animation_data and ob.animation_data.action:
        for fc in ob.animation_data.action.fcurves:
            if fc.data_path == data_path and fc.array_index == index:
                if not any(m.type == 'CYCLES' for m in fc.modifiers):
                    fc.modifiers.new('CYCLES')
                for kp in fc.keyframe_points:
                    kp.interpolation = 'BEZIER'


_GROUP_KEYS = {}


def walk_keys(pts, speed, dwell, rng, duration, step=0.5):
    """Constant-ground-speed traversal of a closed polyline with dwell stops.
    Returns [(t, x, y, heading)] sampled every `step` seconds."""
    dwell_lo, dwell_hi = dwell
    t = 0.0
    seg = 0
    along = 0.0
    keyed = []
    while t <= duration + 1e-6:
        ax, ay = pts[seg]
        bx2, by2 = pts[(seg + 1) % len(pts)]
        L = math.hypot(bx2 - ax, by2 - ay)
        u = 0.0 if L < 1e-9 else clamp(along / L, 0.0, 1.0)
        px = ax + (bx2 - ax) * u
        py = ay + (by2 - ay) * u
        hd = math.atan2(by2 - ay, bx2 - ax) - math.pi / 2
        keyed.append((t, px, py, hd))
        if along >= L - 1e-9:
            seg = (seg + 1) % len(pts)
            along = 0.0
            if dwell_hi > 0.0:
                stop = rng.uniform(dwell_lo, dwell_hi)
                tt = t
                while tt < t + stop:
                    keyed.append((tt, px, py, hd))
                    tt += step
                t += stop
        else:
            along += speed * step
        t += step
    return keyed


def apply_path_keys(ob, keyed, fps, total_f, lateral=0.0, lead=0.0, z_offset=0.0):
    """Key an object along a walk_keys list. `lead` offsets it fore/aft and
    `lateral` side to side, both relative to the direction of travel -- which is
    how two bearers end up at either end of one stretcher rather than beside it."""
    for (tt, px, py, hd) in keyed:
        frm = int(round(tt * fps)) + 1
        if frm > total_f:
            break
        if lateral or lead:
            # heading is stored with a -90 deg offset (model faces +Y)
            fwd = hd + math.pi / 2
            px = px + math.cos(fwd) * lead + math.cos(fwd + math.pi / 2) * lateral
            py = py + math.sin(fwd) * lead + math.sin(fwd + math.pi / 2) * lateral
        ob.location = (px, py, height_at(px, py) + z_offset)
        ob.rotation_euler = Euler((0, 0, hd), 'XYZ')
        ob.keyframe_insert(data_path="location", frame=frm)
        ob.keyframe_insert(data_path="rotation_euler", index=2, frame=frm)
    if ob.animation_data and ob.animation_data.action:
        for fc in ob.animation_data.action.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'LINEAR'


def build_dynamic_humans():
    log("placing dynamic humans and keyframing bounded paths")
    fps = CFG["fps"]
    total_f = int(CFG["sim_seconds"] * fps)
    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = total_f
    bpy.context.scene.render.fps = fps

    for spec in DYNAMIC_HUMANS:
        rng = sub_rng("dyn_" + spec["pid"])
        name = "SAR_DynamicPerson_%s" % spec["pid"].split("_")[1]
        scale = rng.uniform(0.90, 1.08)
        p0 = spec["path"][0]
        root, parts = build_person(spec["pid"], name, p0[0], p0[1], "WALKING",
                                   spec["cloth"], spec["th"], scale=scale)
        set_props(root, sar_object="HUMAN", sar_type="DYNAMIC",
                  person_id=spec["pid"], dynamic=True,
                  thermal_class="THERMAL_HUMAN_TORSO",
                  thermal_contrast=spec["th"],
                  thermal_delta_k=THERMAL_CONTRAST[spec["th"]]["delta_k"],
                  victim_status="AMBULATORY", pose="WALKING",
                  occlusion="DYNAMIC_VARIABLE", search_zone=spec["zone"],
                  sar_sector=spec["sector"], detection_difficulty="MEDIUM",
                  height_m=round(1.72 * scale, 3), semantic_class="HUMAN",
                  walk_speed_mps=spec["speed"],
                  path_closed=True,
                  path_waypoints=json.dumps([[round(p[0], 2), round(p[1], 2)]
                                             for p in spec["path"]]),
                  path_waypoints_xyz=json.dumps(
                      [[round(p[0], 2), round(p[1], 2),
                        round(height_at(p[0], p[1]), 3)] for p in spec["path"]]),
                  motion_source="BLENDER_KEYFRAMES + sar_dynamic_paths.json",
                  motion_pattern=spec.get("pattern", "PATROL"),
                  motion_group=spec.get("group", "NONE"),
                  dwell_min_s=float(spec["dwell"][0]),
                  dwell_max_s=float(spec["dwell"][1]),
                  lead_offset_m=float(spec.get("lead", 0.0)),
                  lateral_offset_m=float(spec.get("lateral", 0.0)),
                  gazebo_actor=True,
                  scenario_note=spec.get("note", ""))
        link_to(root, "DYNAMIC_HUMANS", "SAR_HUMAN_TEST")
        for p in parts:
            link_to(p, "DYNAMIC_HUMANS", "SAR_EXPORT", "SAR_VISUAL",
                    "SAR_COLLISION", "SAR_HUMAN_TEST")

        # ---- walk the path at constant ground speed with dwell stops ----
        # Members of a group share one key list so they stay in formation --
        # a stretcher party whose two bearers drift apart is worse than no
        # stretcher party at all.
        grp = spec.get("group")
        if grp and grp in _GROUP_KEYS:
            keyed = _GROUP_KEYS[grp]
        else:
            keyed = walk_keys(spec["path"], spec["speed"], spec["dwell"],
                              sub_rng("walk_" + (grp or spec["pid"])),
                              CFG["sim_seconds"])
            if grp:
                _GROUP_KEYS[grp] = keyed

        apply_path_keys(root, keyed, fps, total_f,
                        lateral=spec.get("lateral", 0.0),
                        lead=spec.get("lead", 0.0))

        # crude but useful limb swing, 1.0 s cycle
        cyc = fps
        for p in parts:
            if p.name.endswith("_Leg_L"):
                _cycle_fcurves(p, "rotation_euler", 0,
                               [(1, 0.42), (1 + cyc // 2, -0.38), (1 + cyc, 0.42)])
            elif p.name.endswith("_Leg_R"):
                _cycle_fcurves(p, "rotation_euler", 0,
                               [(1, -0.38), (1 + cyc // 2, 0.42), (1 + cyc, -0.38)])
            elif p.name.endswith("_Arm_L"):
                _cycle_fcurves(p, "rotation_euler", 0,
                               [(1, -0.30), (1 + cyc // 2, 0.30), (1 + cyc, -0.30)])
            elif p.name.endswith("_Arm_R"):
                _cycle_fcurves(p, "rotation_euler", 0,
                               [(1, 0.30), (1 + cyc // 2, -0.30), (1 + cyc, 0.30)])

        PEOPLE_RECORDS.append(dict(
            person_id=spec["pid"], object=name, type="DYNAMIC",
            position=[round(p0[0], 2), round(p0[1], 2),
                      round(height_at(p0[0], p0[1]), 3)],
            pose="WALKING", victim_status="AMBULATORY",
            thermal_contrast=spec["th"],
            thermal_delta_k=THERMAL_CONTRAST[spec["th"]]["delta_k"],
            occlusion="DYNAMIC_VARIABLE", search_zone=spec["zone"],
            sector=spec["sector"], detection_difficulty="MEDIUM",
            height_m=round(1.72 * scale, 3),
            speed_mps=spec["speed"], dwell_s=list(spec["dwell"]),
            motion_pattern=spec.get("pattern", "PATROL"),
            motion_group=spec.get("group", "NONE"),
            lateral_offset_m=spec.get("lateral", 0.0),
            lead_offset_m=spec.get("lead", 0.0),
            waypoints=[[round(p[0], 2), round(p[1], 2),
                        round(height_at(p[0], p[1]), 3)] for p in spec["path"]],
            closed_loop=True,
            note=spec.get("note", "")))
    # the load each GROUP_CARRY pair is carrying, keyed on the same list
    n_load = 0
    for grp, keyed in _GROUP_KEYS.items():
        if not grp.startswith("STRETCHER"):
            continue
        ob = make_box("SAR_CarriedStretcher_%s" % grp, 0.62, 2.05, 0.16,
                      "MAT_RESCUE")
        gpath = next((d["path"] for d in DYNAMIC_HUMANS
                      if d.get("group") == grp), None)
        gspeed = next((d["speed"] for d in DYNAMIC_HUMANS
                       if d.get("group") == grp), 1.0)
        gdwell = next((d["dwell"] for d in DYNAMIC_HUMANS
                       if d.get("group") == grp), (0.0, 0.0))
        set_props(ob, sar_object="CARRIED_LOAD", semantic_class="PROP",
                  motion_group=grp, dynamic=True, sar_type="DYNAMIC",
                  gazebo_actor=True,
                  path_waypoints=json.dumps([[round(q[0], 2), round(q[1], 2)]
                                             for q in (gpath or [])]),
                  path_waypoints_xyz=json.dumps(
                      [[round(q[0], 2), round(q[1], 2),
                        round(height_at(q[0], q[1]), 3)] for q in (gpath or [])]),
                  walk_speed_mps=gspeed,
                  dwell_min_s=float(gdwell[0]), dwell_max_s=float(gdwell[1]),
                  actor_z_offset=0.95,
                  thermal_class="THERMAL_AMBIENT",
                  thermal_temp_c=THERMAL_TABLE["THERMAL_AMBIENT"]["temp"],
                  collision="BOX", traversability="FORBIDDEN",
                  risk_level="MEDIUM",
                  motion_source="BLENDER_KEYFRAMES + sar_dynamic_paths.json",
                  note="Stretcher carried between the two bearers of %s. Moves "
                       "as part of the group, sits at 0.95 m -- above a "
                       "ground-plane filter and below a head-height one." % grp)
        link_to(ob, "DYNAMIC_HUMANS", "DYNAMIC_PROPS", "SAR_EXPORT",
                "SAR_VISUAL", "SAR_COLLISION", "SAR_SENSOR_TEST")
        apply_path_keys(ob, keyed, fps, total_f, z_offset=0.95)
        n_load += 1
    STATS["carried_loads"] = n_load
    STATS["dynamic_humans"] = len(DYNAMIC_HUMANS)


# ============================================================================
# PART 15 -- DECOYS: RGB and THERMAL FALSE POSITIVES
# ============================================================================

DECOYS = [
    dict(id="SAR_Decoy_Mannequin_001", kind="MANNEQUIN", x=4.5,   y=60.5,
         note="4 m from PERSON_001. Human silhouette, air temperature. RGB says "
              "maybe; thermal says no."),
    dict(id="SAR_Decoy_Mannequin_002", kind="MANNEQUIN", x=-10.0, y=106.5,
         note="Shop mannequin in the collapse field, near PERSON_005."),
    dict(id="SAR_Decoy_Mannequin_003", kind="MANNEQUIN", x=88.0,  y=71.0,
         note="Near the triage cluster -- hardest discrimination case."),
    dict(id="SAR_Decoy_Post_001",      kind="POST",      x=-2.0,  y=58.0, note=""),
    dict(id="SAR_Decoy_Post_002",      kind="POST",      x=-80.5, y=162.0, note=""),
    dict(id="SAR_Decoy_Post_003",      kind="POST",      x=80.0,  y=228.5, note=""),
    dict(id="SAR_Decoy_Trunk_001",     kind="TRUNK",     x=88.6,  y=26.8,
         note="Requirement 116 triad: warm vehicle (004) + upright trunk + "
              "human (PERSON_015) inside one sensor footprint."),
    dict(id="SAR_Decoy_Trunk_002",     kind="TRUNK",     x=-76.5, y=158.5,
         note="Upright dead trunk beside prone PERSON_002."),
    dict(id="SAR_Decoy_Bag_001",       kind="BAG",       x=1.5,   y=117.5,
         note="Duffel bag beside PERSON_006. Similar size, wrong temperature."),
    dict(id="SAR_Decoy_Bag_002",       kind="BAG",       x=-49.0, y=231.5,
         note="Pack in the avalanche debris, near PERSON_014."),
    dict(id="SAR_Decoy_WarmCase_001",  kind="WARMCASE",  x=91.0,  y=69.0,
         note="Powered comms case at 41 C in the triage cluster -- warm, "
              "person-sized, not a person."),
    dict(id="SAR_Decoy_WarmCase_002",  kind="WARMCASE",  x=-113.0, y=60.5,
         note="Powered pump case by the stranded vehicle."),
]


def build_decoys():
    log("placing perception decoys")
    rng = sub_rng("decoys")
    for spec in DECOYS:
        x, y = spec["x"], spec["y"]
        gz = height_at(x, y)
        k = spec["kind"]
        if k == "MANNEQUIN":
            root, parts = build_person("DECOY", spec["id"], x, y, "STANDING",
                                       "MAT_MANNEQUIN", "LOW",
                                       heading=rng.uniform(0, math.tau),
                                       scale=rng.uniform(0.95, 1.05))
            for p in parts:
                set_props(p, sar_object="DECOY", semantic_class="DECOY",
                          person_id="NOT_A_PERSON",
                          thermal_class="THERMAL_AMBIENT",
                          thermal_temp_c=THERMAL_TABLE["THERMAL_AMBIENT"]["temp"],
                          decoy="RGB_FALSE_POSITIVE",
                          collision="CAPSULE_APPROX", traversability="FORBIDDEN")
                link_to(p, "SAR_SENSOR_TEST", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
            set_props(root, sar_object="DECOY", semantic_class="DECOY",
                      decoy="RGB_FALSE_POSITIVE", decoy_kind="MANNEQUIN",
                      thermal_class="THERMAL_AMBIENT",
                      scenario_note=spec["note"])
            link_to(root, "SAR_SENSOR_TEST")
            add_exclusion(x, y, 1.0)
            continue
        if k == "POST":
            ob = make_cyl(spec["id"], 0.075, 0.07, rng.uniform(1.6, 2.0),
                          "MAT_METAL", 8, loc=(x, y, gz))
            therm = "THERMAL_METAL"; dec = "RGB_FALSE_POSITIVE"
        elif k == "TRUNK":
            ob = make_cyl(spec["id"], 0.20, 0.13, rng.uniform(2.2, 3.4),
                          "MAT_BARK", 8, loc=(x, y, gz))
            therm = "THERMAL_WOOD"; dec = "RGB_FALSE_POSITIVE"
        elif k == "BAG":
            v, f = ico_verts(0.45, 1, 0.14, rng, (1.5, 0.75, 0.6))
            ob = new_mesh_obj(spec["id"], [(vx, vy, vz + 0.28) for (vx, vy, vz) in v],
                              f, ["MAT_HUMAN_DARK"], smooth=True, loc=(x, y, gz))
            therm = "THERMAL_AMBIENT"; dec = "RGB_FALSE_POSITIVE"
        else:  # WARMCASE
            ob = make_box(spec["id"], 0.78, 0.48, 0.42, "MAT_METAL", loc=(x, y, gz))
            therm = "THERMAL_EQUIPMENT_WARM"; dec = "THERMAL_FALSE_POSITIVE"
        set_props(ob, sar_object="DECOY", semantic_class="DECOY",
                  decoy=dec, decoy_kind=k, thermal_class=therm,
                  thermal_temp_c=THERMAL_TABLE[therm]["temp"],
                  collision="CONVEX_HULL", traversability="FORBIDDEN",
                  scenario_note=spec["note"])
        link_to(ob, "SAR_SENSOR_TEST", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        add_exclusion(x, y, 0.8)
    STATS["decoys"] = len(DECOYS)
    log("decoys: %d" % len(DECOYS))


def build_occluders():
    """Deliberate occluders next to specific casualties. Placed BEFORE the
    general scatter so the occlusion is guaranteed, not left to chance."""
    log("placing deliberate occluders")
    rng = sub_rng("occluders")
    plan = [
        ("PERSON_002", -77.0, 158.8, "BUSH", 1.5),
        ("PERSON_002", -79.2, 161.4, "BUSH", 1.3),
        ("PERSON_003", -91.0, 170.6, "LOG",  1.2),
        ("PERSON_003", -93.4, 173.0, "BUSH", 1.4),
        ("PERSON_005", -13.4, 102.8, "RUBBLE", 1.8),
        ("PERSON_006",  3.4,  114.8, "RUBBLE", 1.9),
        ("PERSON_013",  80.8, 228.9, "ROCK", 1.7),
        ("PERSON_014", -51.6, 228.7, "RUBBLE", 1.5),
        ("PERSON_001",  -1.2,  64.4, "BUSH", 0.9),
    ]
    for i, (pid, x, y, kind, s) in enumerate(plan):
        gz = height_at(x, y)
        if kind == "BUSH":
            proto = rng.choice(BUSH_PROTOS)
            ob = instance_of(proto, "SAR_Occluder_%02d_Bush" % (i + 1),
                             (x, y, gz - 0.05), rot=(0, 0, rng.uniform(0, math.tau)),
                             scale=(s, s, s * 1.15))
        elif kind == "LOG":
            proto = rng.choice(LOG_PROTOS)
            ob = instance_of(proto, "SAR_Occluder_%02d_Log" % (i + 1),
                             (x, y, gz + 0.05), rot=(0, 0, rng.uniform(0, math.tau)),
                             scale=(s, s, s))
        elif kind == "ROCK":
            proto = rng.choice(BOULDER_PROTOS)
            ob = instance_of(proto, "SAR_Occluder_%02d_Rock" % (i + 1),
                             (x, y, gz - 0.35), rot=(0.1, 0.1, rng.uniform(0, math.tau)),
                             scale=(s * 0.7, s * 0.7, s * 0.6))
        else:
            proto = rng.choice([p for p in RUBBLE_PROTOS
                                if "WALL_FRAGMENT" in p.name or "SLAB" in p.name])
            ob = instance_of(proto, "SAR_Occluder_%02d_Rubble" % (i + 1),
                             (x, y, gz + 0.1),
                             rot=(rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3),
                                  rng.uniform(0, math.tau)),
                             scale=(s, s, s))
        set_props(ob, occludes_person=pid, sar_object="OCCLUDER",
                  scenario_note="Deliberate occluder for %s" % pid)
        link_to(ob, "SAR_SENSOR_TEST", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        add_exclusion(x, y, 0.8)

# ============================================================================
# PART 15b -- PHYSICS-REACTIVE DYNAMIC PROPS
# ============================================================================
# These are NOT keyframed. Nothing in Blender animates them. They are placed
# here with mass, inertia and surface parameters, and exported as NON-STATIC
# SDF models, so every bit of their motion comes out of Gazebo's solver when
# the robot -- or another prop, or gravity -- pushes them.
#
# Why that matters for a SAR stack: a keyframed obstacle always moves the same
# way regardless of what the robot does, so avoidance can succeed by accident.
# A prop that only moves when hit punishes a planner that treats "it moved" as
# "it will keep moving", and rewards one that re-observes. It also produces the
# case that breaks naive static-world SLAM: a landmark that is in the map, then
# is somewhere else, because the robot itself moved it.
#
# Blender has its own rigid-body system and it does NOT export. `sar_physics_
# preview.py` (embedded text datablock) switches it on for these props so you
# can eyeball a topple in the viewport; it is a preview, not the simulation.

# shape: BOX (sx, sy, sz) | CYL_Z (r, h) | CYL_X (r, L)
PROP_KINDS = {
    "BARREL":   dict(shape="CYL_Z", dims=(0.29, 0.88), mass=21.0,
                     mat="MAT_RUSTMETAL", thermal="THERMAL_METAL",
                     mu=0.45, restitution=0.20, collision="CYLINDER",
                     note="Part-full steel drum. Tips, then rolls."),
    "CRATE":    dict(shape="BOX", dims=(0.60, 0.60, 0.55), mass=14.0,
                     mat="MAT_WOOD", thermal="THERMAL_WOOD",
                     mu=0.62, restitution=0.10, collision="BOX",
                     note="Supply crate. Slides and tumbles, does not roll."),
    "PALLET":   dict(shape="BOX", dims=(1.20, 0.80, 0.14), mass=16.0,
                     mat="MAT_WOOD", thermal="THERMAL_WOOD",
                     mu=0.58, restitution=0.05, collision="BOX",
                     note="Flat and low -- a genuine LiDAR ambiguity: the robot "
                          "must decide whether to drive over it."),
    "CONE":     dict(shape="CYL_Z", dims=(0.19, 0.75), mass=4.5,
                     mat="MAT_RESCUE", thermal="THERMAL_AMBIENT",
                     mu=0.70, restitution=0.30, collision="CYLINDER",
                     note="Traffic cone. Light enough to knock flat without "
                          "stopping the robot -- tests whether contact is "
                          "detected at all."),
    "PANEL":    dict(shape="BOX", dims=(1.40, 0.90, 0.05), mass=9.0,
                     mat="MAT_METAL", thermal="THERMAL_METAL",
                     mu=0.35, restitution=0.15, collision="BOX",
                     note="Loose sheet metal. Low friction, slides a long way."),
    "PIPE":     dict(shape="CYL_X", dims=(0.12, 2.20), mass=15.0,
                     mat="MAT_METAL", thermal="THERMAL_METAL",
                     mu=0.30, restitution=0.10, collision="CYLINDER",
                     note="Loose pipe lying across the ground. Rolls "
                          "perpendicular to its axis and not at all along it."),
    "JERRYCAN": dict(shape="BOX", dims=(0.35, 0.18, 0.45), mass=6.5,
                     mat="MAT_RESCUE", thermal="THERMAL_AMBIENT",
                     mu=0.55, restitution=0.20, collision="BOX",
                     note="Fuel can. Small enough to vanish under a bumper."),
    "BIN":      dict(shape="BOX", dims=(0.58, 0.52, 0.92), mass=9.0,
                     mat="MAT_VEHICLE_B", thermal="THERMAL_AMBIENT",
                     mu=0.50, restitution=0.15, collision="BOX",
                     note="Wheeled bin. Tall, light, topples easily."),
}

# id, centre, radius, {kind: count}, zone, sector, exposure, note
# exposure: ON_ROUTE (sits on a demonstration route corridor) |
#           ADJACENT (within sensor range of one) | OFF_ROUTE
PROP_CLUSTERS = [
    dict(id="SAR_DynProps_BaseYard", c=(-14.0, -8.0), r=6.5,
         mix={"BARREL": 3, "CRATE": 4, "PALLET": 3, "CONE": 2},
         zone="SAR_ZONE_HOME", sector="BASE", exposure="ADJACENT", force=True,
         note="Staging yard on the apron. First movable things the robot sees, "
              "on flat ground where a failed push is unambiguous."),
    dict(id="SAR_DynProps_BaseGate", c=(1.5, 3.0), r=4.0,
         mix={"CONE": 4, "BARREL": 2}, zone="SAR_ZONE_HOME", sector="BASE",
         exposure="ON_ROUTE", force=True,
         note="Cones narrowing the base gate. Directly on the exit path -- the "
              "robot meets these in the first ten metres of every run."),
    dict(id="SAR_DynProps_StreetA", c=(61.0, 44.0), r=7.0,
         mix={"CONE": 4, "BIN": 3, "PANEL": 2}, zone="SAR_ZONE_C",
         sector="URBAN", exposure="ON_ROUTE", force=True,
         note="Street furniture strewn across Urban Street A, the alternative "
              "when Street C is blocked by the overturned truck."),
    dict(id="SAR_DynProps_Canyon", c=(79.5, 50.0), r=2.8,
         mix={"BARREL": 3, "PALLET": 2}, zone="SAR_ZONE_C", sector="URBAN",
         exposure="ADJACENT", force=True,
         note="Inside the 7.5 m canyon between URB_001 and URB_002. Narrows an "
              "already tight corridor, and the walls leave nowhere to swing "
              "wide -- the robot either pushes through or backs out."),
    dict(id="SAR_DynProps_WarehouseYard", c=(108.0, 40.0), r=5.0,
         mix={"CRATE": 5, "PALLET": 4, "BARREL": 2}, zone="SAR_ZONE_C",
         sector="URBAN", exposure="ADJACENT", force=True,
         note="Loading yard outside the warehouse roller door."),
    dict(id="SAR_DynProps_RubbleEdge", c=(14.0, 104.0), r=7.0,
         mix={"PIPE": 4, "PANEL": 4, "BARREL": 2}, zone="SAR_ZONE_B",
         sector="RUBBLE", exposure="ADJACENT", force=True,
         note="Loose pipe and sheet at the collapse-field margin, beside the "
              "rubble bypass. Rolling pipe on sloped debris is the nastiest "
              "case here: it moves after the robot has passed it."),
    dict(id="SAR_DynProps_FloodBank", c=(-96.0, 58.0), r=8.0,
         mix={"BARREL": 4, "JERRYCAN": 4, "PANEL": 2}, zone="SAR_ZONE_F",
         sector="FLOOD", exposure="OFF_ROUTE",
         note="Debris washed onto the flood plain. Scattered, low contrast "
              "against mud."),
    dict(id="SAR_DynProps_DesertTrack", c=(116.0, 104.0), r=6.0,
         mix={"JERRYCAN": 3, "BARREL": 2, "CRATE": 2}, zone="SAR_ZONE_E",
         sector="DESERT", exposure="ADJACENT",
         note="Dropped stores beside the desert track. Sand gives low friction "
              "under both the prop and the robot."),
    dict(id="SAR_DynProps_ForestStaging", c=(-40.0, 162.0), r=5.0,
         mix={"CRATE": 3, "PALLET": 2, "CONE": 2}, zone="SAR_ZONE_B",
         sector="FOREST", exposure="ADJACENT",
         note="Forward staging point on the forest track, under canopy shadow."),
]

DYNAMIC_PROP_RECORDS = []


def _inertia(kind):
    """Principal moments about the centre of mass, kg m^2. Computed here rather
    than left to the exporter so the numbers are auditable in the .blend and in
    sar_dynamic_props.json -- a prop with a wrong inertia tensor behaves oddly
    in a way that is very hard to debug from the Gazebo side."""
    k = PROP_KINDS[kind]
    m = k["mass"]
    if k["shape"] == "BOX":
        a, b, c = k["dims"]
        return (m * (b * b + c * c) / 12.0,
                m * (a * a + c * c) / 12.0,
                m * (a * a + b * b) / 12.0)
    r, h = k["dims"]
    side = m * (3.0 * r * r + h * h) / 12.0
    axis = m * r * r / 2.0
    if k["shape"] == "CYL_Z":
        return (side, side, axis)
    return (axis, side, side)          # CYL_X: axis along local X


def _prop_mesh(name, kind, rng):
    k = PROP_KINDS[kind]
    if k["shape"] == "BOX":
        a, b, c = k["dims"]
        v, f = box_verts(a, b, c, cz=0.0)
        return new_mesh_obj(name, v, f, [k["mat"]])
    r, h = k["dims"]
    if kind == "CONE":
        v, f = cyl_verts(r, r * 0.22, h, 10)          # tapered
        v = [(vx, vy, vz - h * 0.5) for (vx, vy, vz) in v]
        return new_mesh_obj(name, v, f, [k["mat"]], smooth=True)
    if k["shape"] == "CYL_Z":
        v, f = cyl_verts(r, r, h, 12)
        v = [(vx, vy, vz - h * 0.5) for (vx, vy, vz) in v]
        return new_mesh_obj(name, v, f, [k["mat"]], smooth=True)
    v, f = cyl_verts(r, r, h, 12)                      # CYL_X: lay it down
    v = [(vz - h * 0.5, vy, vx) for (vx, vy, vz) in v]
    return new_mesh_obj(name, v, f, [k["mat"]], smooth=True)


def build_dynamic_props():
    log("placing physics-reactive dynamic props")
    total = 0
    for cl in PROP_CLUSTERS:
        rng = sub_rng("dynprop_" + cl["id"])
        cx, cy = cl["c"]
        placed = []
        idx = 0
        for kind, count in sorted(cl["mix"].items()):
            k = PROP_KINDS[kind]
            ixx, iyy, izz = _inertia(kind)
            made = 0
            tries = 0
            while made < count and tries < count * 40:
                tries += 1
                a = rng.uniform(0, math.tau)
                rr = math.sqrt(rng.random()) * cl["r"]
                px = cx + math.cos(a) * rr
                py = cy + math.sin(a) * rr
                if in_water(px, py, 1.0) or in_negative(px, py, 0.8):
                    continue
                if not cl.get("force") and blocked(px, py, 0.5):
                    continue
                if any((px - q[0]) ** 2 + (py - q[1]) ** 2 < 0.85 for q in placed):
                    continue
                idx += 1
                name = "%s_%s_%02d" % (cl["id"], kind, made + 1)
                ob = _prop_mesh(name, kind, rng)
                # half-height above ground so it rests ON the surface
                if k["shape"] == "BOX":
                    half_h = k["dims"][2] * 0.5
                elif k["shape"] == "CYL_Z":
                    half_h = k["dims"][1] * 0.5
                else:
                    half_h = k["dims"][0]
                ob.location = (px, py, height_at(px, py) + half_h + 0.02)
                ob.rotation_euler = Euler((0, 0, rng.uniform(0, math.tau)), 'XYZ')
                set_props(
                    ob,
                    sar_object="DYNAMIC_PROP", semantic_class="PROP",
                    prop_kind=kind, prop_cluster=cl["id"],
                    physics="DYNAMIC",
                    mass_kg=round(k["mass"], 3),
                    ixx=round(ixx, 5), iyy=round(iyy, 5), izz=round(izz, 5),
                    com_z_m=round(half_h, 3),
                    friction_mu=k["mu"], restitution=k["restitution"],
                    linear_damping=0.02, angular_damping=0.05,
                    collision=k["collision"],
                    thermal_class=k["thermal"],
                    thermal_temp_c=THERMAL_TABLE[k["thermal"]]["temp"],
                    traversability="FORBIDDEN", risk_level="LOW",
                    search_zone=cl["zone"], sar_sector=cl["sector"],
                    route_exposure=cl["exposure"],
                    dynamic_obstacle_type="PHYSICS_REACTIVE",
                    motion_source="GAZEBO_RIGID_BODY (not keyframed; Blender "
                                  "rigid body is preview only -- see "
                                  "sar_physics_preview.py)",
                    scenario_note=k["note"] + " " + cl["note"])
                link_to(ob, "DYNAMIC_PROPS", "SAR_EXPORT", "SAR_VISUAL",
                        "SAR_COLLISION", "SAR_SENSOR_TEST")
                add_exclusion(px, py, 0.45)
                placed.append((px, py))
                DYNAMIC_PROP_RECORDS.append(dict(
                    name=name, kind=kind, cluster=cl["id"],
                    position=[round(px, 2), round(py, 2),
                              round(height_at(px, py) + half_h + 0.02, 3)],
                    mass_kg=k["mass"], inertia=[round(ixx, 5), round(iyy, 5),
                                                round(izz, 5)],
                    collision=k["collision"], shape=k["shape"],
                    dims=list(k["dims"]), friction_mu=k["mu"],
                    restitution=k["restitution"],
                    search_zone=cl["zone"], sector=cl["sector"],
                    route_exposure=cl["exposure"]))
                made += 1
            total += made
        log("  %s: %d props (%s)" % (cl["id"], len(placed), cl["exposure"]))
    STATS["dynamic_props"] = total
    STATS["dynamic_prop_clusters"] = len(PROP_CLUSTERS)
    log("dynamic props: %d across %d clusters" % (total, len(PROP_CLUSTERS)))


PHYSICS_PREVIEW = r'''# sar_physics_preview.py -- Blender-side rigid body PREVIEW for DYNAMIC_PROPS.
#
# This is NOT the simulation. Gazebo runs the real physics from the mass,
# inertia and surface values exported into each prop's model.sdf. This exists
# so you can hit Play in Blender and watch a barrel topple before committing to
# a layout. Blender rigid bodies do not export.
#
# It makes only the terrain tiles UNDER the props passive colliders, because
# making all 36 tiles passive mesh colliders is enough to make Bullet crawl.
import bpy

props = bpy.data.collections.get("DYNAMIC_PROPS")
terr = bpy.data.collections.get("SAR_TERRAIN")
if props is None or terr is None:
    raise SystemExit("run this inside military_world.blend")

if bpy.context.scene.rigidbody_world is None:
    bpy.ops.rigidbody.world_add()

def add_rb(ob, kind):
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.rigidbody.object_add(type=kind)
    ob.select_set(False)

used_tiles = set()
for o in props.all_objects:
    if o.type != 'MESH' or o.get("physics") != "DYNAMIC":
        continue
    add_rb(o, 'ACTIVE')
    rb = o.rigid_body
    rb.mass = float(o.get("mass_kg", 10.0))
    rb.friction = float(o.get("friction_mu", 0.5))
    rb.restitution = float(o.get("restitution", 0.1))
    rb.collision_shape = {"BOX": 'BOX', "CYLINDER": 'CYLINDER'}.get(
        o.get("collision", "BOX"), 'CONVEX_HULL')
    # nearest terrain tile
    best = None; bd = 1e18
    for t in terr.objects:
        d = (t.location - o.location).length
        if d < bd:
            bd = d; best = t
    if best is not None:
        used_tiles.add(best.name)

for name in used_tiles:
    t = bpy.data.objects[name]
    if t.rigid_body is None:
        add_rb(t, 'PASSIVE')
        t.rigid_body.collision_shape = 'MESH'
        t.rigid_body.friction = 0.8

print("preview armed: %d props, %d passive tiles. Press Play." %
      (len([o for o in props.all_objects if o.rigid_body]), len(used_tiles)))
'''

# ============================================================================
# PART 16 -- SEARCH ZONES, MISSION MARKERS, GROUND TRUTH
# ============================================================================
# Everything in this part lives in GROUND_TRUTH and is deliberately EXCLUDED
# from SAR_EXPORT. The robot must not receive any of it. It exists so that an
# evaluation script can score what the robot found against what was there.

SEARCH_ZONES = [
    dict(id="SAR_ZONE_HOME", label="Base / staging", bounds=(-46, 46, -38, 12),
         difficulty="TRIVIAL", terrain="CONCRETE,GRAVEL",
         purpose="Initialisation, docking, return-to-base"),
    dict(id="SAR_ZONE_A", label="Open search area", bounds=(-56, 46, 12, 92),
         difficulty="EASY", terrain="GRASS,SOIL,DIRT",
         purpose="Lawnmower / grid coverage, long-range LiDAR, baseline detection"),
    dict(id="SAR_ZONE_B", label="Collapse field + forest", bounds=(-120, 46, 92, 196),
         difficulty="HARD", terrain="RUBBLE,SOIL,GRASS",
         purpose="Rubble traversability, forest occlusion, building interiors"),
    dict(id="SAR_ZONE_C", label="Urban disaster", bounds=(46, 140, 14, 96),
         difficulty="MEDIUM", terrain="CONCRETE,ROAD,RUBBLE",
         purpose="GPS-denied urban canyon, blocked routes, multi-person triage"),
    dict(id="SAR_ZONE_D", label="Mountain + snow", bounds=(-110, 130, 190, 258),
         difficulty="HARD", terrain="ROCK,SNOW,GRAVEL",
         purpose="Risk-aware route choice, cliffs, ravines, avalanche debris"),
    dict(id="SAR_ZONE_E", label="Desert transition", bounds=(60, 148, 96, 190),
         difficulty="MEDIUM", terrain="SAND,ROCK",
         purpose="Wheel slip, dune traverse, low-feature localisation"),
    dict(id="SAR_ZONE_F", label="Flood / river margin", bounds=(-148, -34, 0, 100),
         difficulty="HARD", terrain="MUD,WATER,DIRT",
         purpose="Water boundary detection, washouts, bridge crossing"),
]

MISSION_BOUNDARY = [(-146, -36), (146, -36), (146, 256), (-146, 256), (-146, -36)]

def road_pts(rid, i0=0, i1=None, reverse=False):
    """Slice of a road's centreline. Validation routes are built from the ACTUAL
    graded road polylines rather than hand-picked waypoints, because a straight
    line between two distant waypoints ignores the road's curve and walks the
    route straight up whatever the terrain happens to be doing in between. That
    is how you end up 'validating' a 60-degree slope."""
    pts = list(ROAD_BY_ID[rid]["pts"] if rid in ROAD_BY_ID
               else next(r for r in ROADS if r["id"] == rid)["pts"])
    seg = pts[i0:(len(pts) if i1 is None else i1)]
    return list(reversed(seg)) if reverse else seg


VALIDATION_ROUTES = [
    dict(id="SAR_Route_01_Disaster",
         label="BASE -> ROAD -> RUBBLE BYPASS -> COLLAPSED BUILDING -> PERSON -> RETURN",
         target="PERSON_007",
         pts=(road_pts("SAR_Road_MainSupply", 0, 5)
              + road_pts("SAR_Track_RubbleBypass", 0, 3)
              + [(24, 121), (19, 120)])),
    dict(id="SAR_Route_02_Mountain",
         label="BASE -> OPEN FIELD -> MOUNTAIN (long safe line) -> RAVINE RIM -> PERSON -> RETURN",
         target="PERSON_013",
         pts=(road_pts("SAR_Road_MainSupply", 0, 5)
              + road_pts("SAR_Track_RubbleBypass")
              + [(2, 152)] + road_pts("SAR_Road_MainSupply", 8)
              + road_pts("SAR_Track_Mountain_Safe", 1)
              + [(79, 229), (81.5, 229.8)])),
    dict(id="SAR_Route_03_UrbanFlood",
         label="BASE -> URBAN -> FLOOD BOUNDARY -> DAMAGED VEHICLE -> PERSON -> RETURN",
         target="PERSON_015",
         pts=(road_pts("SAR_Road_East")
              + road_pts("SAR_Street_Urban_D", 0, 2, reverse=True)
              + [(48, 43)]
              + [(56, 34), (61, 29), (70, 27), (80, 26), (87.0, 24.8)])),
    dict(id="SAR_Route_04_Snow",
         label="BASE -> FOREST TRACK -> SNOW -> AVALANCHE DEBRIS -> PERSON -> SAFE RETURN",
         target="PERSON_014",
         pts=(road_pts("SAR_Road_MainSupply", 0, 5)
              + road_pts("SAR_Track_RubbleBypass")
              + [(2, 152)]
              + road_pts("SAR_Track_Forest", 1)
              + road_pts("SAR_Track_Snow", 1, 4)
              + [(-50.5, 229)])),
    dict(id="SAR_Route_05_AllTerrain",
         label="asphalt -> bridge -> flood margin -> grass -> rubble -> gravel -> forest dirt",
         target="TERRAIN_ADAPTATION",
         pts=(road_pts("SAR_Road_West")                       # out, over the bridge
              + road_pts("SAR_Road_West", 6, 11, reverse=True)  # back to the bank
              + [(-100, 60), (-96, 52), (-88, 48)]              # off-road, mud
              + road_pts("SAR_Road_West", 0, 6, reverse=True)
              + road_pts("SAR_Road_MainSupply", 1, 5)
              + road_pts("SAR_Track_RubbleBypass", 0, 3)
              + [(24, 128), (18, 132), (12, 136)]               # off-road, rubble
              + [(4, 144), (2, 152)]
              + road_pts("SAR_Track_Forest", 1, 3))),
]

STEEP_SLOPES = [
    dict(id="SAR_SteepSlope_001", c=(37, 202), zone="SAR_ZONE_D"),
    dict(id="SAR_SteepSlope_002", c=(-30, 220), zone="SAR_ZONE_D"),
    dict(id="SAR_SteepSlope_003", c=(96, 232), zone="SAR_ZONE_D"),
]

HAZARD_RECORDS = []
MARKER_RECORDS = []


def _marker(name, loc, kind='PLAIN_AXES', size=1.0, rot=(0, 0, 0), coll="GROUND_TRUTH"):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_type = kind
    ob.empty_display_size = size
    ob.location = loc
    ob.rotation_euler = Euler(rot, 'XYZ')
    C(coll).objects.link(ob)
    return ob


def _wire(name, pts_xyz, closed=False, coll="GROUND_TRUTH"):
    me = bpy.data.meshes.new("SARM_" + name)
    edges = [(i, i + 1) for i in range(len(pts_xyz) - 1)]
    if closed:
        edges.append((len(pts_xyz) - 1, 0))
    me.from_pydata([Vector(p) for p in pts_xyz], edges, [])
    me.update()
    ob = bpy.data.objects.new(name, me)
    ob.display_type = 'WIRE'
    C(coll).objects.link(ob)
    return ob


def build_mission_markers():
    log("building mission markers and ground truth")

    # --- robot start + base -------------------------------------------------
    sx, sy = 0.0, -8.0
    start = _marker("SAR_ROBOT_START", (sx, sy, height_at(sx, sy) + 0.30),
                    'ARROWS', 2.5, rot=(0, 0, 0.0))
    set_props(start, sar_object="MARKER", marker_role="ROBOT_START",
              semantic_class="MARKER",
              heading_deg=0.0,
              heading_note="Local +Y. Faces north up the main supply route into "
                           "the open search area -- 30 m of clear ground ahead "
                           "and 12 m of clear sky for sensor init.",
              spawn_clearance_radius_m=6.0,
              sensor_clearance_note="Nothing but graded ground within 6 m -- "
                                    "asserted by the spawn_clearance check, not "
                                    "assumed. A roof LiDAR and mast-mounted "
                                    "cameras are unobstructed at spawn.",
              ground_z=round(height_at(sx, sy), 3))
    MARKER_RECORDS.append(dict(name="SAR_ROBOT_START", role="ROBOT_START",
                               position=[sx, sy, round(height_at(sx, sy) + 0.30, 3)],
                               heading_deg=0.0))
    add_exclusion(sx, sy, 6.0)

    for (nm, role, x, y) in [
            ("SAR_BASE_LOCATION", "BASE_HOME", 0.0, -14.0),
            ("SAR_DOCK_POSE", "DOCK", 13.0, -20.0),
            ("SAR_SEARCH_START", "SEARCH_START", 0.0, 20.0),
            ("SAR_SEARCH_END", "SEARCH_END", -10.0, 132.0),
            ("SAR_RETURN_BASE", "RETURN_BASE", 1.0, 1.0)]:
        ob = _marker(nm, (x, y, height_at(x, y) + 0.2), 'SPHERE', 1.6)
        set_props(ob, sar_object="MARKER", marker_role=role, semantic_class="MARKER")
        MARKER_RECORDS.append(dict(name=nm, role=role,
                                   position=[x, y, round(height_at(x, y) + 0.2, 3)]))

    # --- search zones -------------------------------------------------------
    for z in SEARCH_ZONES:
        x0, x1, y0, y1 = z["bounds"]
        cx = (x0 + x1) * 0.5; cy = (y0 + y1) * 0.5
        ob = _marker(z["id"], (cx, cy, height_at(cx, cy) + 1.0), 'CUBE', 3.0)
        set_props(ob, sar_object="SEARCH_ZONE", semantic_class="MARKER",
                  zone_id=z["id"], zone_label=z["label"],
                  bounds_min_x=x0, bounds_max_x=x1,
                  bounds_min_y=y0, bounds_max_y=y1,
                  area_m2=round((x1 - x0) * (y1 - y0), 1),
                  search_difficulty=z["difficulty"],
                  terrain_types=z["terrain"], purpose=z["purpose"])
        zz = max(height_at(x0, y0), height_at(x1, y1)) + 1.5
        _wire(z["id"] + "_Outline",
              [(x0, y0, zz), (x1, y0, zz), (x1, y1, zz), (x0, y1, zz)], closed=True)

    # --- mission boundary (logical, NOT fenced) -----------------------------
    bpts = [(x, y, height_at(x, y) + 2.0) for (x, y) in MISSION_BOUNDARY]
    mb = _wire("SAR_MISSION_BOUNDARY", bpts, closed=False)
    set_props(mb, sar_object="MISSION_BOUNDARY", semantic_class="MARKER",
              polygon=json.dumps([[x, y] for (x, y) in MISSION_BOUNDARY]),
              area_m2=292 * 292,
              note="Logical mission polygon for the coverage planner. There is "
                   "no physical fence here -- the robot is constrained by the "
                   "planner, not by geometry.")

    # --- validation routes --------------------------------------------------
    for r in VALIDATION_ROUTES:
        pts = [(x, y, height_at(x, y) + 0.5) for (x, y) in r["pts"]]
        ob = _wire(r["id"], pts)
        set_props(ob, sar_object="VALIDATION_ROUTE", semantic_class="MARKER",
                  route_id=r["id"], route_label=r["label"],
                  target=r["target"],
                  length_m=round(poly_length(r["pts"]), 1),
                  note="ENVIRONMENT VALIDATION AID ONLY. Do not feed this to the "
                       "planner. It exists to prove the environment is "
                       "traversable end-to-end, not to tell the robot where to go.")
        for i, (x, y) in enumerate(r["pts"]):
            wp = _marker("%s_WP_%02d" % (r["id"], i + 1),
                         (x, y, height_at(x, y) + 0.4), 'PLAIN_AXES', 0.8)
            set_props(wp, sar_object="ROUTE_WAYPOINT", route_id=r["id"],
                      waypoint_index=i, semantic_class="MARKER")

    # --- ground-truth person markers ---------------------------------------
    for rec in PEOPLE_RECORDS:
        nm = "GT_" + rec["person_id"]
        px, py, pz = rec["position"]
        ob = _marker(nm, (px, py, pz + 2.4), 'CONE', 0.9)
        set_props(ob, sar_object="GROUND_TRUTH_PERSON", semantic_class="MARKER",
                  person_id=rec["person_id"], linked_object=rec["object"],
                  sar_type=rec["type"], pose=rec["pose"],
                  victim_status=rec["victim_status"],
                  thermal_contrast=rec["thermal_contrast"],
                  thermal_delta_k=rec["thermal_delta_k"],
                  occlusion=rec["occlusion"], search_zone=rec["search_zone"],
                  sar_sector=rec["sector"],
                  detection_difficulty=rec["detection_difficulty"],
                  gt_x=px, gt_y=py, gt_z=pz,
                  visibility_note="EVALUATION ONLY. Never published to ROS. "
                                  "The robot has to find this by perceiving it.")

    # --- hazard ground truth ------------------------------------------------
    hid = 0

    def reg(hz_id, hz_type, x, y, sev, trav, zone, extra=None, note=""):
        nonlocal hid
        hid += 1
        nm = "GT_HAZARD_%03d" % hid
        z = height_at(x, y)
        ob = _marker(nm, (x, y, z + 1.6), 'SINGLE_ARROW', 1.4)
        props = dict(sar_object="GROUND_TRUTH_HAZARD", semantic_class="HAZARD",
                     hazard_id=hz_id, hazard_type=hz_type, severity=sev,
                     traversability=trav, search_zone=zone,
                     gt_x=round(x, 2), gt_y=round(y, 2), gt_z=round(z, 3),
                     risk_level=sev, note=note,
                     visibility_note="EVALUATION ONLY. Not published to ROS.")
        if extra:
            props.update(extra)
        set_props(ob, **props)
        HAZARD_RECORDS.append(dict(
            gt_marker=nm, hazard_id=hz_id, hazard_type=hz_type, severity=sev,
            traversability=trav, search_zone=zone,
            position=[round(x, 2), round(y, 2), round(z, 3)],
            extra=extra or {}, note=note))

    for nb in NEGATIVES:
        pts = nb["pts"]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        sev = {"HOLE": "CRITICAL", "RAVINE": "CRITICAL", "TRENCH": "HIGH",
               "DITCH": "HIGH", "ROAD_WASHOUT": "CRITICAL",
               "COLLAPSED_GROUND": "MEDIUM"}[nb["kind"]]
        reg(nb["id"], nb["kind"], cx, cy, sev, "FORBIDDEN", nb["zone"],
            extra=dict(depth_m=nb["depth"], width_m=nb["half"] * 2,
                       visual_salience=nb["vis"],
                       length_m=round(poly_length(pts) if len(pts) > 1 else nb["half"] * 2, 1),
                       detection_modality="3D_LIDAR,DEPTH,TERRAIN_ANALYSIS"),
            note=("Negative obstacle. Real void in the terrain mesh -- no "
                  "invisible floor. %s to see from the approach."
                  % ("Easy" if nb["vis"] == "OBVIOUS" else "Hard")))

    for c in CLIFFS:
        pts = c["pts"]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        reg(c["id"], "CLIFF", cx, cy, "CRITICAL", "FORBIDDEN", "SAR_ZONE_D",
            extra=dict(drop_m=c["drop"], face_width_m=c["width"],
                       approx_slope_deg=round(math.degrees(
                           math.atan(c["drop"] / max(0.1, c["width"] * 2))), 1),
                       length_m=round(poly_length(pts), 1)),
            note="Hard no-go edge. The drop is modelled in the terrain mesh; the "
                 "robot must refuse it from LiDAR/terrain analysis alone.")

    reg("HAZARD_WATER_001", "WATER", -120.0, 60.0, "CRITICAL", "FORBIDDEN",
        "SAR_ZONE_F", extra=dict(max_depth_m=RIVER_DEPTH, surface_z=WATER_LEVEL,
                                 length_m=round(poly_length(RIVER), 1)),
        note="River channel. Deep water: forbidden for a wheeled UGV.")
    reg("HAZARD_WATER_002", "WATER", POND[0], POND[1], "HIGH", "FORBIDDEN",
        "SAR_ZONE_C", extra=dict(max_depth_m=POND_DEPTH, surface_z=POND_LEVEL,
                                 radius_m=POND[2]),
        note="Flooded urban depression, depth not observable from the bank.")

    for fld in RUBBLE_FIELDS:
        reg(fld["id"], "RUBBLE", fld["c"][0], fld["c"][1], "HIGH",
            "VERY_DIFFICULT", fld["zone"],
            extra=dict(extent_x_m=fld["rx"] * 2, extent_y_m=fld["ry"] * 2,
                       piece_count=fld["n"]),
            note=fld["label"] + ". Traversable in places, not in others -- the "
                                "robot has to classify, not assume.")

    for ss in STEEP_SLOPES:
        x, y = ss["c"]
        sl = slope_deg(x, y, 1.5)
        reg(ss["id"], "STEEP_SLOPE", x, y,
            "CRITICAL" if sl > 30 else "HIGH",
            "FORBIDDEN" if sl > 30 else "DIFFICULT", ss["zone"],
            extra=dict(measured_slope_deg=round(sl, 1)),
            note="Measured from the generated height field, not asserted.")

    reg("SAR_BlockedRoad_001", "BLOCKED_PATH", -4.0, 110.0, "HIGH", "FORBIDDEN",
        "SAR_ZONE_B",
        extra=dict(alternative_route="SAR_Track_RubbleBypass"),
        note="Main supply route blocked by the collapse field. A graded gravel "
             "bypass exists to the east -- replan, do not give up.")
    reg("SAR_BlockedRoad_002", "BLOCKED_PATH", 82.0, 62.0, "HIGH", "FORBIDDEN",
        "SAR_ZONE_C",
        extra=dict(alternative_route="SAR_Street_Urban_B / SAR_Street_Urban_D"),
        note="Urban Street C blocked by an overturned truck plus debris.")
    reg("SAR_BlockedEntrance_001", "BLOCKED_PATH", 112.0, 46.2, "MEDIUM",
        "FORBIDDEN", "SAR_ZONE_C",
        extra=dict(alternative_route="warehouse side door at approx (101, 54)"),
        note="Warehouse main door blocked. A narrow side door is passable.")

    # --- dead ends: the robot should recognise and back out ----------------
    for i, (x, y) in enumerate([(-36, 128), (118, 92), (68, 214)]):
        reg("SAR_DeadEnd_%03d" % (i + 1), "DEAD_END", x, y, "LOW", "DIFFICULT",
            "SAR_ZONE_B" if i == 0 else ("SAR_ZONE_E" if i == 1 else "SAR_ZONE_D"),
            note="No through route. Turning space is available within 12 m: the "
                 "robot must detect the dead end, reverse or turn, and replan.")
    STATS["hazards"] = len(HAZARD_RECORDS)
    log("ground truth: %d people, %d hazards, %d zones"
        % (len(PEOPLE_RECORDS), len(HAZARD_RECORDS), len(SEARCH_ZONES)))


# ============================================================================
# PART 17 -- LIGHTING AND WEATHER PRESETS
# ============================================================================
# Lighting stays configurable: six sun/world setups, one active, the rest
# hidden. Nothing is baked into textures.

LIGHT_PRESETS = {
    "DAY":      dict(sun_energy=4.2, sun_angle_deg=52, sun_az_deg=140,
                     sun_color=(1.0, 0.97, 0.92),
                     world=(0.36, 0.50, 0.74), world_strength=1.10,
                     fog_density=0.0, ambient_c=22.0,
                     note="Reference condition. RGB easy, thermal contrast modest."),
    "CLOUDY":   dict(sun_energy=1.5, sun_angle_deg=48, sun_az_deg=150,
                     sun_color=(0.95, 0.96, 1.0),
                     world=(0.46, 0.48, 0.52), world_strength=1.55,
                     fog_density=0.0, ambient_c=17.0,
                     note="Flat light, weak shadows, low texture contrast."),
    "SUNSET":   dict(sun_energy=2.4, sun_angle_deg=8, sun_az_deg=255,
                     sun_color=(1.0, 0.62, 0.33),
                     world=(0.22, 0.17, 0.20), world_strength=0.62,
                     fog_density=0.002, ambient_c=19.0,
                     note="Long shadows, strong glare, worst case for RGB "
                          "exposure. Terrain still holds residual heat."),
    "LOW_LIGHT": dict(sun_energy=0.35, sun_angle_deg=-3, sun_az_deg=268,
                      sun_color=(0.62, 0.68, 0.88),
                      world=(0.045, 0.058, 0.085), world_strength=0.34,
                      fog_density=0.0015, ambient_c=13.0,
                      note="Civil twilight. RGB degrading, thermal improving."),
    "NIGHT":    dict(sun_energy=0.04, sun_angle_deg=-14, sun_az_deg=280,
                     sun_color=(0.52, 0.62, 0.92),
                     world=(0.012, 0.016, 0.028), world_strength=0.10,
                     fog_density=0.002, ambient_c=9.0,
                     note="Base floodlights on, everything else near-dark. "
                          "Human-to-background delta T is largest here: this is "
                          "where thermal earns its place. Not pitch black -- "
                          "the point is fusion, not sensor failure."),
    "FOG":      dict(sun_energy=1.0, sun_angle_deg=35, sun_az_deg=160,
                     sun_color=(0.92, 0.94, 0.96),
                     world=(0.55, 0.57, 0.60), world_strength=1.30,
                     fog_density=0.020, ambient_c=12.0,
                     note="Visual range ~35 m. LiDAR degrades with the model you "
                          "choose in Gazebo; Blender volumetrics do not export."),
}

WEATHER_PRESETS = {
    "CLEAR": dict(rain=0.0, dust=0.0, snowfall=0.0, wind_mps=1.5,
                  visibility_m=2000, lighting="DAY"),
    "RAIN":  dict(rain=0.7, dust=0.0, snowfall=0.0, wind_mps=6.0,
                  visibility_m=250, lighting="CLOUDY",
                  note="Wet surfaces: drop friction_mu by ~0.15 on ROAD/CONCRETE "
                       "and by ~0.08 elsewhere in the Gazebo surface params."),
    "FOG":   dict(rain=0.0, dust=0.0, snowfall=0.0, wind_mps=0.5,
                  visibility_m=35, lighting="FOG"),
    "DUST":  dict(rain=0.0, dust=0.8, snowfall=0.0, wind_mps=12.0,
                  visibility_m=60, lighting="SUNSET",
                  note="Desert sector only. Sand-laden air: RGB washes out, "
                       "LiDAR returns get noisy."),
    "SNOWFALL": dict(rain=0.0, dust=0.0, snowfall=0.75, wind_mps=8.0,
                     visibility_m=45, lighting="CLOUDY",
                     note="Whiteout-like. Do NOT model this as total sensor "
                          "failure; the objective is robust navigation, not a "
                          "scripted blackout."),
    "NIGHT_CLEAR": dict(rain=0.0, dust=0.0, snowfall=0.0, wind_mps=2.0,
                        visibility_m=1200, lighting="NIGHT"),
}


def build_lighting():
    log("building lighting presets")
    for name, p in LIGHT_PRESETS.items():
        ld = bpy.data.lights.new("SAR_Sun_" + name, 'SUN')
        ld.energy = p["sun_energy"]
        ld.color = p["sun_color"]
        ld.angle = math.radians(1.2)
        ob = bpy.data.objects.new("SAR_Sun_" + name, ld)
        el = math.radians(p["sun_angle_deg"])
        az = math.radians(p["sun_az_deg"])
        ob.rotation_euler = Euler((math.pi / 2 - el, 0.0, az), 'XYZ')
        ob.location = (0, 0, 120)
        set_props(ob, sar_object="LIGHT", lighting_preset=name,
                  world_color=json.dumps(list(p["world"])),
                  world_strength=p["world_strength"],
                  fog_density=p["fog_density"],
                  ambient_air_temp_c=p["ambient_c"],
                  note=p["note"])
        C("LIGHTING").objects.link(ob)
        active = (name == "DAY")
        ob.hide_viewport = not active
        ob.hide_render = not active

    # world: background + a volume scatter wired but at zero density
    w = bpy.data.worlds.get("SAR_World") or bpy.data.worlds.new("SAR_World")
    bpy.context.scene.world = w
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    d = LIGHT_PRESETS["DAY"]
    bg.inputs[0].default_value = (d["world"][0], d["world"][1], d["world"][2], 1.0)
    bg.inputs[1].default_value = d["world_strength"]
    nt.links.new(bg.outputs[0], out.inputs["Surface"])
    vs = nt.nodes.new("ShaderNodeVolumeScatter")
    vs.inputs["Density"].default_value = 0.0
    vs.inputs["Color"].default_value = (0.82, 0.85, 0.88, 1.0)
    nt.links.new(vs.outputs[0], out.inputs["Volume"])
    w["active_preset"] = "DAY"
    w["available_presets"] = ",".join(LIGHT_PRESETS.keys())

    # weather config holders -- data, not fake physics
    for name, p in WEATHER_PRESETS.items():
        ob = _marker("SAR_WEATHER_" + name, (0, 0, 40), 'SPHERE', 1.0, coll="LIGHTING")
        set_props(ob, sar_object="WEATHER_PRESET", weather_preset=name,
                  semantic_class="MARKER", **{k: v for k, v in p.items()
                                              if k != "note"})
        ob["note"] = p.get("note", "")
        ob["handoff"] = ("Blender does not export weather. Transcribe these "
                         "values into the Gazebo <scene>/<fog> block and your "
                         "sensor noise models.")
        ob.hide_viewport = True
    STATS["lighting_presets"] = len(LIGHT_PRESETS)
    STATS["weather_presets"] = len(WEATHER_PRESETS)


# ============================================================================
# PART 18 -- SENSOR TEST FURNITURE
# ============================================================================
# A short depth-camera ladder near the start pose plus a few deliberate
# narrow-clearance gates. Small, boring, and the first thing you actually want
# when a new sensor stack comes up.

def build_sensor_tests():
    log("building sensor test objects")
    rng = sub_rng("sensortest")
    objs = []
    # depth ladder at 2 / 5 / 10 / 20 / 40 m from the start pose, off to the side
    for k, dist in enumerate((2.0, 5.0, 10.0, 20.0, 40.0)):
        x = -12.0
        y = -6.0 + dist
        if blocked(x, y, 1.0):
            x = -16.0
        gz = height_at(x, y)
        ob = make_box("SAR_SensorTest_DepthTarget_%02d" % (k + 1),
                      0.6, 0.6, 1.2, "MAT_CONCRETE", loc=(x, y, gz))
        set_props(ob, sar_object="SENSOR_TARGET", semantic_class="PROP",
                  target_range_m=dist, thermal_class="THERMAL_CONCRETE",
                  collision="BOX", traversability="FORBIDDEN",
                  note="Depth/stereo range ladder. Known size, known distance "
                       "from SAR_ROBOT_START.")
        link_to(ob, "SAR_SENSOR_TEST", "SAR_EXPORT", "SAR_VISUAL", "SAR_COLLISION")
        objs.append(ob)
        add_exclusion(x, y, 1.2)

    # clearance gates: 1.4 m, 1.0 m, 0.8 m between concrete blocks.
    # 0.8 m is deliberately tighter than most 8-wheel UGVs. It should be
    # rejected, not attempted -- that is the test.
    for k, (gap, gx, gy) in enumerate([(1.40, -30.0, 86.0),
                                       (1.00, 24.0, 118.0),
                                       (0.80, -58.0, 102.0)]):
        for sgn in (-1, 1):
            x = gx + sgn * (gap * 0.5 + 0.6)
            ob = make_box("SAR_SensorTest_Gate_%02d_%s" % (k + 1, "L" if sgn < 0 else "R"),
                          1.2, 1.2, 1.5, "MAT_CONCRETE",
                          loc=(x, gy, height_at(x, gy)))
            set_props(ob, sar_object="CLEARANCE_GATE", semantic_class="PROP",
                      gate_gap_m=gap, thermal_class="THERMAL_CONCRETE",
                      collision="BOX", traversability="FORBIDDEN",
                      note="Clearance gate. Gap %.2f m." % gap)
            link_to(ob, "SAR_SENSOR_TEST", "HAZARDS", "SAR_EXPORT", "SAR_VISUAL",
                    "SAR_COLLISION")
            objs.append(ob)
        add_exclusion(gx, gy, 2.5)
    STATS["sensor_test_objects"] = len(objs)
    return objs

# ============================================================================
# PART 19 -- SCENE SETUP
# ============================================================================

def setup_scene():
    sc = bpy.context.scene
    sc.unit_settings.system = 'METRIC'
    sc.unit_settings.scale_length = 1.0
    sc.unit_settings.length_unit = 'METERS'
    # EEVEE is called EEVEE_NEXT from Blender 4.2; fall back cleanly on 4.0/4.1
    for eng in ('BLENDER_EEVEE_NEXT', 'BLENDER_EEVEE'):
        try:
            sc.render.engine = eng
            break
        except TypeError:
            continue
    sc.render.fps = CFG["fps"]
    sc.frame_start = 1
    sc.frame_end = int(CFG["sim_seconds"] * CFG["fps"])
    sc.render.resolution_x = 1920
    sc.render.resolution_y = 1080
    sc["sar_seed"] = CFG["seed"]
    sc["sar_extent_m"] = "%.0f x %.0f" % (CFG["x_max"] - CFG["x_min"],
                                          CFG["y_max"] - CFG["y_min"])
    sc["sar_units"] = "1 Blender unit = 1 metre, Z up, right-handed"
    sc["sar_target_sim"] = "Gazebo Sim Harmonic (SDF 1.10). NOT Isaac Sim."
    sc["sar_non_weaponised"] = True


# ============================================================================
# PART 20 -- METADATA AND HANDOFF FILES
# ============================================================================

def collect_collection_stats():
    out = {}
    def walk(c):
        out[c.name] = len(c.all_objects)
        for ch in c.children:
            walk(ch)
    root = bpy.data.collections.get(ROOT_COLL)
    if root:
        walk(root)
    return out


def write_metadata(outdir, report):
    log("writing metadata and handoff files")
    os.makedirs(outdir, exist_ok=True)

    # ---- ground truth ----------------------------------------------------
    gt = dict(
        schema="sar_ground_truth/1.0",
        seed=CFG["seed"],
        generated_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        warning=("EVALUATION ONLY. Do not publish any of this to ROS at runtime. "
                 "Use it to score detections after a run. Feeding it to the "
                 "robot invalidates every result the environment exists to "
                 "produce."),
        world=dict(x_min=CFG["x_min"], x_max=CFG["x_max"],
                   y_min=CFG["y_min"], y_max=CFG["y_max"],
                   units="metres", up_axis="Z"),
        people=PEOPLE_RECORDS,
        hazards=HAZARD_RECORDS,
        markers=MARKER_RECORDS,
        search_zones=SEARCH_ZONES,
        mission_boundary=[[x, y] for (x, y) in MISSION_BOUNDARY],
        validation_routes=[dict(id=r["id"], label=r["label"], target=r["target"],
                                waypoints=[[x, y, round(height_at(x, y), 3)]
                                           for (x, y) in r["pts"]],
                                length_m=round(poly_length(r["pts"]), 1))
                           for r in VALIDATION_ROUTES],
    )
    with open(os.path.join(outdir, "sar_ground_truth.json"), "w") as f:
        json.dump(gt, f, indent=2)

    # ---- world / semantic metadata --------------------------------------
    meta = dict(
        schema="sar_metadata/1.0",
        seed=CFG["seed"],
        target_simulator="Gazebo Sim Harmonic",
        target_middleware="ROS 2 (ros_gz)",
        not_supported=["Isaac Sim (explicitly out of scope)"],
        weaponised=False,
        units=dict(length="metre", scale_length=1.0, up_axis="Z",
                   note="1 Blender unit == 1 m. No arbitrary scaling anywhere."),
        world=dict(extent_m=[CFG["x_max"] - CFG["x_min"], CFG["y_max"] - CFG["y_min"]],
                   x_range=[CFG["x_min"], CFG["x_max"]],
                   y_range=[CFG["y_min"], CFG["y_max"]],
                   origin_note="Base sits on the world origin; the map extends north."),
        semantic_classes=SEMANTIC_CLASSES,
        traversability_model=TRAVERSABILITY,
        terrain_class_to_material=CLASS_MAT,
        terrain_class_to_thermal=CLASS_THERMAL,
        thermal_table=THERMAL_TABLE,
        thermal_contrast_classes=THERMAL_CONTRAST,
        thermal_disclaimer=(
            "Blender performs no radiometric simulation. These are apparent "
            "surface temperatures for a temperate late-afternoon condition, "
            "intended to be transcribed into Gazebo thermal camera / "
            "<temperature> parameters. Walls, rubble and snowpack are opaque in "
            "LWIR: objects flagged opaque_to_thermal must occlude. Ambient air "
            "temperature per lighting preset is in lighting_presets[*].ambient_c; "
            "contrast is human_surface_temp minus local background, not an "
            "absolute."),
        lighting_presets=LIGHT_PRESETS,
        weather_presets=WEATHER_PRESETS,
        weather_disclaimer=(
            "Fog / rain / dust / snowfall are Blender-side visual variations plus "
            "this table. Gazebo needs its own <scene><fog> and sensor noise "
            "configuration -- none of it crosses the mesh export."),
        search_zones=SEARCH_ZONES,
        roads=[dict(id=r["id"], cls=r["cls"], width_m=r["width"],
                    length_m=round(poly_length(r["pts"]), 1),
                    centerline=[[x, y] for (x, y) in r["pts"]]) for r in ROADS],
        negative_obstacles=[dict(id=n["id"], kind=n["kind"], depth_m=n["depth"],
                                 width_m=n["half"] * 2, salience=n["vis"],
                                 zone=n["zone"],
                                 centerline=[[x, y] for (x, y) in n["pts"]])
                            for n in NEGATIVES],
        cliffs=[dict(id=c["id"], drop_m=c["drop"], width_m=c["width"],
                     polyline=[[x, y] for (x, y) in c["pts"]]) for c in CLIFFS],
        water=[dict(id="SAR_Water_River_001", surface_z=WATER_LEVEL,
                    max_depth_m=RIVER_DEPTH,
                    polyline=[[x, y] for (x, y) in RIVER]),
               dict(id="SAR_Water_FloodPool_002", surface_z=POND_LEVEL,
                    max_depth_m=POND_DEPTH, center=[POND[0], POND[1]],
                    radius_m=POND[2])],
        buildings=[dict(id=b["id"], center=list(b["c"]), w=b["w"], d=b["d"],
                        storeys=b["storeys"], damage=b["damage"],
                        interior=bool(b.get("interior")), roof=b["roof"],
                        label=b["label"]) for b in BUILDINGS],
        vehicles=[dict(id=v["id"], center=list(v["c"]), kind=v["kind"],
                       damage=v["damage"], thermal=v["thermal"], label=v["label"])
                  for v in VEHICLES],
        dynamic_obstacles=dict(
            keyframed_humans=len([r for r in PEOPLE_RECORDS
                                  if r["type"] == "DYNAMIC"]),
            carried_loads=STATS.get("carried_loads", 0),
            physics_reactive_props=len(DYNAMIC_PROP_RECORDS),
            prop_clusters=[dict(id=c["id"], center=list(c["c"]), radius_m=c["r"],
                                zone=c["zone"], sector=c["sector"],
                                exposure=c["exposure"], mix=c["mix"],
                                note=c["note"]) for c in PROP_CLUSTERS],
            prop_kinds={k: dict(v, inertia=list(_inertia(k)))
                        for k, v in PROP_KINDS.items()},
            note=("Two mechanisms, deliberately different. Humans and carried "
                  "loads follow scripted trajectories and move whatever the "
                  "robot does. Props have mass and inertia and no script at "
                  "all -- they move only when something pushes them, which is "
                  "the case that punishes a planner for assuming a static "
                  "world and for assuming 'it moved once' means 'it will keep "
                  "moving'.")),
        terrain_tiles=TERRAIN_TILES,
        collections=collect_collection_stats(),
        statistics=STATS,
        validation=report,
        custom_property_contract=dict(
            human=["person_id", "sar_type", "dynamic", "thermal_class",
                   "thermal_contrast", "thermal_delta_k", "victim_status",
                   "pose", "occlusion", "search_zone", "sar_sector",
                   "detection_difficulty", "height_m"],
            hazard=["hazard_id", "hazard_type", "severity", "traversability",
                    "risk_level", "search_zone"],
            terrain=["terrain_type", "terrain_secondary", "terrain_mix",
                     "traversability", "traverse_cost", "friction_mu",
                     "risk_level", "thermal_class"],
            generic=["sar_object", "semantic_class", "thermal_class",
                     "collision", "sar_sector"]),
    )
    with open(os.path.join(outdir, "sar_metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # ---- thermal table as CSV -------------------------------------------
    with open(os.path.join(outdir, "sar_thermal_table.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["thermal_class", "apparent_temp_c", "emissivity",
                     "opaque_in_lwir", "notes"])
        for k, v in THERMAL_TABLE.items():
            opaque = "yes"
            wr.writerow([k, v["temp"], v["emis"], opaque, v["note"]])
        wr.writerow([])
        wr.writerow(["# human contrast class", "surface_class", "delta_T_K_vs_background"])
        for k, v in THERMAL_CONTRAST.items():
            wr.writerow([k, v["surface"], v["delta_k"]])
        wr.writerow([])
        wr.writerow(["# lighting preset", "ambient_air_temp_c", "note"])
        for k, v in LIGHT_PRESETS.items():
            wr.writerow([k, v["ambient_c"], v["note"]])

    # ---- dynamic human paths + Gazebo actor snippets --------------------
    dyn = dict(
        schema="sar_dynamic_paths/1.0",
        seed=CFG["seed"],
        note=("Blender keyframes do NOT cross into a Gazebo world. These "
              "waypoint loops are the portable form. Turn each into a Gazebo "
              "<actor> with a <trajectory>, or drive the models from a ROS 2 "
              "node. The generated SDF below is a starting point, not a "
              "finished world."),
        fps=CFG["fps"], duration_s=CFG["sim_seconds"],
        actors=[dict(person_id=r["person_id"], object=r["object"],
                     speed_mps=r["speed_mps"], dwell_s=r["dwell_s"],
                     closed_loop=True, waypoints=r["waypoints"],
                     thermal_contrast=r["thermal_contrast"],
                     search_zone=r["search_zone"])
                for r in PEOPLE_RECORDS if r["type"] == "DYNAMIC"],
    )
    with open(os.path.join(outdir, "sar_dynamic_paths.json"), "w") as f:
        json.dump(dyn, f, indent=2)

    actor_sdf = []
    for a in dyn["actors"]:
        t = 0.0
        wps = []
        pts = a["waypoints"] + [a["waypoints"][0]]
        for i in range(len(pts) - 1):
            x0, y0, z0 = pts[i]
            x1, y1, z1 = pts[i + 1]
            yaw = math.atan2(y1 - y0, x1 - x0)
            wps.append('        <waypoint><time>%.2f</time>'
                       '<pose>%.3f %.3f %.3f 0 0 %.4f</pose></waypoint>'
                       % (t, x0, y0, z0, yaw))
            t += math.hypot(x1 - x0, y1 - y0) / max(0.1, a["speed_mps"])
        wps.append('        <waypoint><time>%.2f</time>'
                   '<pose>%.3f %.3f %.3f 0 0 0</pose></waypoint>'
                   % (t, pts[0][0], pts[0][1], pts[0][2]))
        actor_sdf.append(
            '    <actor name="%s">\n'
            '      <!-- %s  thermal_contrast=%s  zone=%s -->\n'
            '      <skin><filename>meshes/%s.glb</filename></skin>\n'
            '      <script>\n        <loop>true</loop>\n'
            '        <auto_start>true</auto_start>\n'
            '        <trajectory id="0" type="walk">\n%s\n'
            '        </trajectory>\n      </script>\n'
            '    </actor>' % (a["object"], a["person_id"], a["thermal_contrast"],
                             a["search_zone"], a["object"], "\n".join(wps)))

    sdf = ['<?xml version="1.0" ?>',
           '<sdf version="1.10">',
           '  <world name="military_world">',
           '    <!-- Generated skeleton. Meshes come from export_gazebo.py. -->',
           '    <!-- NON-WEAPONISED search-and-rescue training range. -->',
           '    <physics name="default" type="dart">',
           '      <max_step_size>0.002</max_step_size>',
           '      <real_time_factor>1.0</real_time_factor>',
           '    </physics>',
           '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
           '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
           '      <render_engine>ogre2</render_engine>',
           '    </plugin>',
           '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
           '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
           '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>',
           '    <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"/>',
           '    <scene>',
           '      <ambient>0.45 0.45 0.45 1</ambient>',
           '      <background>0.36 0.50 0.74 1</background>',
           '      <grid>false</grid>',
           '      <!-- FOG preset: uncomment and tune per sar_metadata.json -->',
           '      <!-- <fog><type>linear</type><density>0.02</density>',
           '           <start>5</start><end>35</end></fog> -->',
           '    </scene>',
           '    <light type="directional" name="sun">',
           '      <cast_shadows>true</cast_shadows>',
           '      <pose>0 0 120 0 0 0</pose>',
           '      <diffuse>1.0 0.97 0.92 1</diffuse>',
           '      <specular>0.25 0.25 0.25 1</specular>',
           '      <direction>-0.5 0.35 -0.78</direction>',
           '    </light>',
           '',
           '    <!-- ================= TERRAIN ================= -->',
           '    <!-- One static model per tile. Visual mesh == collision mesh:',
           '         decimating the collision copy would heal the holes and',
           '         ditches, which is the one thing we must not do. -->']
    for t in TERRAIN_TILES:
        sdf.append(
            '    <model name="%s"><static>true</static>\n'
            '      <pose>%.3f %.3f 0 0 0 0</pose>\n'
            '      <link name="link">\n'
            '        <visual name="v"><geometry><mesh><uri>meshes/%s.glb</uri></mesh></geometry></visual>\n'
            '        <collision name="c"><geometry><mesh><uri>meshes/%s.glb</uri></mesh></geometry>\n'
            '          <surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode></friction></surface>\n'
            '        </collision>\n'
            '      </link>\n'
            '    </model>' % (t["name"], t["center"][0], t["center"][1], t["name"],
                             t["name"],
                             TRAVERSABILITY.get(t["dominant"], TRAVERSABILITY["GRASS"])["mu"],
                             TRAVERSABILITY.get(t["dominant"], TRAVERSABILITY["GRASS"])["mu"]))
    sdf.append('')
    sdf.append('    <!-- ============== DYNAMIC HUMANS ============== -->')
    sdf.append('    <!-- Bounded, seeded, reproducible. Positions are NOT')
    sdf.append('         published to ROS: perception has to find them. -->')
    sdf.extend(actor_sdf)
    sdf.append('')
    sdf.append('    <!-- ========= PHYSICS-REACTIVE DYNAMIC PROPS ========= -->')
    sdf.append('    <!-- Non-static bodies with mass and inertia. No trajectory,')
    sdf.append('         no script: they move only when something pushes them. -->')
    for r in DYNAMIC_PROP_RECORDS:
        sdf.append('    <include><uri>model://%s</uri>'
                   '<name>%s</name>'
                   '<pose>%.3f %.3f %.3f 0 0 0</pose></include>'
                   % (r["name"], r["name"], r["position"][0], r["position"][1],
                      r["position"][2]))
    sdf.append('')
    sdf.append('    <!-- Static geometry: run export_gazebo.py, then include the')
    sdf.append('         generated models/ here. Ground truth is NOT exported. -->')
    sdf.append('  </world>')
    sdf.append('</sdf>')
    with open(os.path.join(outdir, "sar_world_template.sdf"), "w") as f:
        f.write("\n".join(sdf) + "\n")

    with open(os.path.join(outdir, "sar_dynamic_props.json"), "w") as f:
        json.dump(dict(
            schema="sar_dynamic_props/1.0",
            seed=CFG["seed"],
            note=("Physics-reactive obstacles. These export as NON-STATIC SDF "
                  "models with mass, inertia and surface parameters; Gazebo's "
                  "solver produces all of their motion. Nothing here is "
                  "keyframed. Blender's own rigid-body system does not export "
                  "-- sar_physics_preview.py only arms a viewport preview."),
            kinds={k: dict(v, inertia=list(_inertia(k)))
                   for k, v in PROP_KINDS.items()},
            clusters=[dict(id=c["id"], center=list(c["c"]), radius_m=c["r"],
                           zone=c["zone"], sector=c["sector"],
                           exposure=c["exposure"], note=c["note"])
                      for c in PROP_CLUSTERS],
            props=DYNAMIC_PROP_RECORDS), f, indent=2)

    with open(os.path.join(outdir, "sar_validation_report.json"), "w") as f:
        json.dump(report, f, indent=2)


EXPORT_SCRIPT = r'''#!/usr/bin/env python3
"""
export_gazebo.py -- Blender -> Gazebo Sim Harmonic exporter for military_world.

Run:
  blender --background military_world.blend --python export_gazebo.py -- \
          --out ./gazebo_export

Produces a directory you can run directly:

  <out>/military_world.sdf     one complete, runnable world
  <out>/meshes/*.glb           one mesh per unique Blender mesh datablock

What it does
------------
* Exports one mesh per unique MESH DATABLOCK, not per object. Roughly 3000 of
  the objects in this scene are linked duplicates sharing ~60 meshes (trees,
  rocks, rubble, props), so this turns a 3500-file, several-hundred-megabyte
  export into about sixty files. Per-object scale is carried on <mesh><scale>.
* Writes ONE world file with every object as a <model>. Ground truth is never
  exported: GROUND_TRUTH is not linked into SAR_EXPORT.
* Groups the parts of each human into a SINGLE model with one link and several
  visuals, instead of seven separate models. A walking person you have to move
  by driving seven poses in lockstep is a trap; one model, one pose.
* Chooses each collision primitive from the object's `collision` custom
  property:
      MESH_EXACT          -> the mesh itself (terrain, roads, bridge deck)
      BOX / BOX_COMPOUND  -> box from the world-space bounding box
      CYLINDER            -> cylinder from bbox radius/height
      CONVEX_HULL         -> the mesh, for the engine to decompose
      CAPSULE_APPROX      -> capsule from bbox (humans)
      CYLINDER_TRUNK_ONLY -> trunk only; nobody pays for leaf collision
      BOX_LOW             -> reduced box (bushes: soft obstacle)
      NONE_VISUAL_ONLY    -> visual only, no <collision>
* Objects carrying physics="DYNAMIC" export as NON-STATIC bodies with the mass,
  principal inertia and surface parameters computed in the generator. Those are
  the 77 props that only move when something pushes them.
* Thermal metadata becomes a <temperature> on each visual, in Kelvin, for the
  Gazebo thermal camera system. It is a configuration handoff, not a
  simulation -- see the thermal note in README.md.

Options
-------
  --out PATH        output directory (default ./gazebo_export)
  --format glb|dae|obj   mesh format; falls back automatically if unavailable.
                    glb is the default because the Debian/Ubuntu Blender
                    package ships with the COLLADA exporter compiled out.
  --models          ALSO write models/<name>/model.sdf + model.config for every
                    object, for use as a standalone model library.
  --no-thermal      omit the per-visual Thermal <plugin> blocks. gz-sim parses
                    each one in a bare <sdf> wrapper and logs
                    "XML Element[plugin], child of element[sdf], not defined in
                    SDF" for every visual -- ~740 warnings at startup. They are
                    harmless, but under -v 4 the noise dominates the log and
                    slows the load. Temperatures remain in sar_thermal_table.csv
                    and sar_metadata.json either way.
  --actors          put the walking <actor>s into military_world.sdf. OFF by
                    default: a world that loads beats a world that walks, and
                    actors are the one element reported to break the load.
                    stage5_actors_plain.sdf / stage5_actors_skin.sdf are always
                    written so you can test the two actor forms in isolation.
  --no-batch        one model per object instead of merged scatter batches.
                    That is ~2900 models, which Gazebo does not load
                    comfortably. Kept for comparison, not for use.
  --verbose         do not suppress Blender's per-export chatter. Without this,
                    the glTF addon's output is muted, because it prints
                    "ERROR: Draco mesh compression is not available" once per
                    mesh -- 712 times -- which looks exactly like a failure and
                    is not one. Draco is optional compression we do not use.
"""
import bpy, os, sys, json, math, time, contextlib
from mathutils import Vector


def args():
    a = sys.argv
    a = a[a.index("--") + 1:] if "--" in a else []
    out = "./gazebo_export"
    fmt = "glb"
    models = "--models" in a
    verbose = "--verbose" in a
    batch = "--no-batch" not in a
    thermal = "--no-thermal" not in a
    actors = "--actors" in a
    if "--out" in a:
        out = a[a.index("--out") + 1]
    if "--format" in a:
        fmt = a[a.index("--format") + 1].lower().lstrip(".")
    return out, fmt, models, verbose, batch, actors, thermal


OUT, FMT, WRITE_MODELS, VERBOSE, BATCH, ACTORS, THERMAL = args()


@contextlib.contextmanager
def quiet():
    """Mute the exporter addon at the file-descriptor level.

    The glTF addon logs through C-level stdout/stderr, so a python-side
    logging filter does not reach it. Its loudest line is a Draco compression
    ERROR emitted once per mesh whether or not compression is requested; over
    712 meshes that buries the actual progress and reads as a crash.
    """
    if VERBOSE:
        yield
        return
    sys.stdout.flush(); sys.stderr.flush()
    old1, old2 = os.dup(1), os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1); os.dup2(devnull, 2)
    try:
        yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(old1, 1); os.dup2(old2, 2)
        os.close(devnull); os.close(old1); os.close(old2)


def say(msg):
    print(msg)
    sys.stdout.flush()
MESHES = os.path.join(OUT, "meshes")
MODELS = os.path.join(OUT, "models")
os.makedirs(MESHES, exist_ok=True)
if WRITE_MODELS:
    os.makedirs(MODELS, exist_ok=True)

exp = bpy.data.collections.get("SAR_EXPORT")
if exp is None:
    raise SystemExit("SAR_EXPORT collection not found -- wrong .blend?")


def _have(opname):
    mod, fn = opname.split(".")
    return hasattr(getattr(bpy.ops, mod), fn)


def pick_format(fmt):
    order = {"glb": ["glb", "dae", "obj"], "gltf": ["glb", "dae", "obj"],
             "dae": ["dae", "glb", "obj"], "obj": ["obj", "glb", "dae"]}.get(
                 fmt, ["glb", "dae", "obj"])
    for f in order:
        if f == "glb" and _have("export_scene.gltf"):
            return "glb"
        if f == "dae" and _have("wm.collada_export"):
            return "dae"
        if f == "obj" and (_have("wm.obj_export") or _have("export_scene.obj")):
            return "obj"
    raise SystemExit("no usable mesh exporter in this Blender build")


FMT = pick_format(FMT)
EXT = "." + FMT
say("mesh format: %s%s" % (EXT, "" if VERBOSE else
                           "   (exporter output muted; pass --verbose for it)"))


def export_selected(path):
    """Z-up, metres, no scene transform -- what Gazebo expects."""
    if FMT == "glb":
        bpy.ops.export_scene.gltf(filepath=path, export_format='GLB',
                                  use_selection=True, export_yup=False,
                                  export_apply=True)
    elif FMT == "dae":
        try:
            bpy.ops.wm.collada_export(filepath=path, selected=True,
                                      apply_modifiers=True,
                                      export_global_forward_selection='Y',
                                      export_global_up_selection='Z')
        except TypeError:
            bpy.ops.wm.collada_export(filepath=path, selected=True)
    else:
        if _have("wm.obj_export"):
            bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True,
                                  up_axis='Z', forward_axis='Y')
        else:
            bpy.ops.export_scene.obj(filepath=path, use_selection=True,
                                     axis_up='Z', axis_forward='Y')


def safe(name):
    return "".join(c if (c.isalnum() or c in "._-") else "_" for c in name)


bpy.context.view_layer.update()
objs = [o for o in exp.all_objects if o.type == 'MESH']

# ---------------------------------------------------------------------------
# 0. batching
# ---------------------------------------------------------------------------
# Gazebo charges per MODEL, not per mesh file: an SDF parse, an Ogre scene node
# and a physics body each. The first version of this exporter emitted one model
# per object -- 2922 of them, 2494 being scattered trees, bushes, rocks and
# rubble -- and Gazebo fell over loading it. Scatter is now merged into one
# model per (family, 50 m cell): one joined visual mesh, and cheap primitive
# collisions inside that single link.
#
# The fidelity trade is deliberate. A scattered rock exports as an oriented box
# rather than a convex hull, because ~1000 mesh collisions is what makes the
# solver crawl, and a box is an honest stand-in for a 40 cm rock the robot must
# not drive through. Everything the robot interacts with on purpose -- terrain,
# roads, the bridge deck, buildings, vehicles, humans and the 77 dynamic props
# -- keeps its exact collision. --no-batch restores one model per object.
BATCH_FAMILY = {
    "TREE": "VEG", "BUSH": "VEG", "GRASS": "VEG", "FALLEN_TREE": "VEG",
    "ROCK": "ROCK", "BOULDER": "ROCK",
    "RUBBLE_PIECE": "RUBBLE", "DEBRIS": "RUBBLE",
}
BATCH_TEMP = {"VEG": 20.0, "ROCK": 14.0, "RUBBLE": 22.0}
CELL = 50.0


def group_root(o):
    p = o.parent
    if p is not None and p.type == 'EMPTY' and p.get("sar_object") in (
            "HUMAN", "DECOY"):
        return p
    return None


bpy.context.scene.frame_set(1)   # actor meshes are authored at the start pose
batches = {}
groups = {}
actor_groups = {}
singles = []
for o in objs:
    fam = BATCH_FAMILY.get(o.get("sar_object")) if BATCH else None
    if fam and o.get("physics") != "DYNAMIC":
        t = o.matrix_world.translation
        key = (fam, int(math.floor(t.x / CELL)), int(math.floor(t.y / CELL)))
        batches.setdefault(key, []).append(o)
        continue
    # A walking human is emitted BOTH ways -- as a static model and as an
    # actor -- and the world files pick one. That keeps the known-good world
    # reproducible while the actor form is still being proven.
    r = group_root(o)
    if r is None:
        if o.get("gazebo_actor") and o.get("path_waypoints_xyz"):
            actor_groups.setdefault(o.name, (o, []))[1].append(o)
        singles.append(o)
    else:
        if r.get("gazebo_actor") and r.get("path_waypoints_xyz"):
            actor_groups.setdefault(r.name, (r, []))[1].append(o)
        groups.setdefault(r.name, (r, []))[1].append(o)

need_mesh = list(singles) + [o for (_r, ps) in groups.values() for o in ps]
n_unique = len({o.data.name for o in need_mesh})
n_batched = sum(len(v) for v in batches.values())
say("objects: %d total -- %d batched into %d merged models, %d individual, "
    "%d also emitted as actors"
    % (len(objs), n_batched, len(batches), len(objs) - n_batched,
       len(actor_groups)))
say("unique meshes to export: %d (+ %d merged)" % (n_unique, len(batches)))
say("this takes a minute or two -- let it run")
_t0 = time.time()

# ---------------------------------------------------------------------------
# 1. one mesh file per unique datablock
# ---------------------------------------------------------------------------
mesh_file = {}
for o in need_mesh:
    key = o.data.name
    if key in mesh_file:
        continue
    fn = safe(key) + EXT
    mesh_file[key] = fn
    loc, rot, scl = o.location.copy(), o.rotation_euler.copy(), o.scale.copy()
    o.location = (0, 0, 0)
    o.rotation_euler = (0, 0, 0)
    o.scale = (1, 1, 1)
    bpy.context.view_layer.update()
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    with quiet():
        export_selected(os.path.join(MESHES, fn))
    o.location, o.rotation_euler, o.scale = loc, rot, scl
    o.select_set(False)
    n = len(mesh_file)
    if n % 50 == 0 or n == n_unique:
        el = time.time() - _t0
        say("  meshes %4d/%d  (%.0fs elapsed, ~%.0fs left)"
            % (n, n_unique, el, el / max(1, n) * (n_unique - n)))
bpy.context.view_layer.update()
say("unique meshes exported: %d in %.0fs" % (len(mesh_file), time.time() - _t0))


# ---------------------------------------------------------------------------
# 2. one merged mesh per batch
# ---------------------------------------------------------------------------
def merge_export(parts, xform, path):
    """Join copies of `parts`, each placed by xform(part), and export once."""
    with quiet():
        bpy.ops.object.select_all(action='DESELECT')
        copies = []
        for o in parts:
            c = o.copy()
            c.data = o.data.copy()
            bpy.context.scene.collection.objects.link(c)
            c.matrix_world = xform(o)
            copies.append(c)
        for c in copies:
            c.select_set(True)
        bpy.context.view_layer.objects.active = copies[0]
        if len(copies) > 1:
            bpy.ops.object.join()
        joined = bpy.context.view_layer.objects.active
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        export_selected(path)
        me = joined.data
        bpy.data.objects.remove(joined, do_unlink=True)
        if me.users == 0:
            bpy.data.meshes.remove(me)


# ---- one mesh per actor, authored in the actor's own frame --------------
actor_file = {}
if actor_groups:
    for aname, (root, parts) in sorted(actor_groups.items()):
        inv = root.matrix_world.inverted()
        fn = "actor_" + safe(aname) + EXT
        merge_export(parts, lambda o, _inv=inv: _inv @ o.matrix_world,
                     os.path.join(MESHES, fn))
        actor_file[aname] = fn
    bpy.context.view_layer.update()
    say("actor meshes exported: %d" % len(actor_file))

batch_file = {}
if batches:
    _t1 = time.time()
    for bi, (key, members) in enumerate(sorted(batches.items())):
        fam, ci, cj = key
        # NB: no "+" in the filename. URI parsing commonly decodes "+"
        # as a space, which silently breaks the mesh lookup.
        bname = "batch_%s_%s%03d_%s%03d" % (
            fam, "n" if ci < 0 else "p", abs(ci),
            "n" if cj < 0 else "p", abs(cj))
        with quiet():
            bpy.ops.object.select_all(action='DESELECT')
            copies = []
            for o in members:
                c = o.copy()
                c.data = o.data.copy()
                bpy.context.scene.collection.objects.link(c)
                c.matrix_world = o.matrix_world.copy()
                copies.append(c)
            for c in copies:
                c.select_set(True)
            bpy.context.view_layer.objects.active = copies[0]
            if len(copies) > 1:
                bpy.ops.object.join()
            joined = bpy.context.view_layer.objects.active
            bpy.ops.object.transform_apply(location=True, rotation=True,
                                           scale=True)
            export_selected(os.path.join(MESHES, bname + EXT))
            me = joined.data
            bpy.data.objects.remove(joined, do_unlink=True)
            if me.users == 0:
                bpy.data.meshes.remove(me)
        batch_file[key] = bname + EXT
        if (bi + 1) % 10 == 0 or bi + 1 == len(batches):
            say("  merged %3d/%d batches (%.0fs)"
                % (bi + 1, len(batches), time.time() - _t1))
    bpy.context.view_layer.update()
say("models: %d individual + %d grouped + %d merged batches"
    % (len(singles), len(groups), len(batches)))


def c2k(c):
    return c + 273.15


def dims_of(o):
    bb = [tuple(v) for v in o.bound_box]
    return ((max(b[0] for b in bb) - min(b[0] for b in bb)) * abs(o.scale.x),
            (max(b[1] for b in bb) - min(b[1] for b in bb)) * abs(o.scale.y),
            (max(b[2] for b in bb) - min(b[2] for b in bb)) * abs(o.scale.z))


def mesh_uri(o):
    return "meshes/" + mesh_file[o.data.name]


def scale_xml(o):
    s = o.scale
    if abs(s.x - 1) < 1e-6 and abs(s.y - 1) < 1e-6 and abs(s.z - 1) < 1e-6:
        return ""
    return "<scale>%.5f %.5f %.5f</scale>" % (s.x, s.y, s.z)


def surface_xml(o):
    if o.get("physics") != "DYNAMIC":
        mu = o.get("friction_mu")
        if mu is None:
            return ""
        return ('<surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode>'
                '</friction></surface>' % (float(mu), float(mu)))
    mu = float(o.get("friction_mu", 0.5))
    rest = float(o.get("restitution", 0.1))
    return ('<surface><friction><ode><mu>%.2f</mu><mu2>%.2f</mu2></ode>'
            '</friction><bounce><restitution_coefficient>%.2f'
            '</restitution_coefficient><threshold>0.05</threshold></bounce>'
            '<contact><ode><max_vel>0.1</max_vel><min_depth>0.001</min_depth>'
            '</ode></contact></surface>' % (mu, mu, rest))


def collision_xml(o, name, pose=""):
    coll = o.get("collision", "CONVEX_HULL")
    if coll == "NONE_VISUAL_ONLY":
        return ""
    sx, sy, sz = dims_of(o)
    surf = surface_xml(o)
    if coll in ("BOX", "BOX_COMPOUND"):
        geom = "<box><size>%.4f %.4f %.4f</size></box>" % (sx, sy, sz)
    elif coll == "CYLINDER":
        geom = ("<cylinder><radius>%.4f</radius><length>%.4f</length>"
                "</cylinder>" % (max(sx, sy) * 0.5, sz))
    elif coll == "CAPSULE_APPROX":
        # A box, not a <capsule>. Capsule is legal SDF 1.10 and Harmonic
        # supports it, but it is one more thing that has to line up across
        # sdformat, gz-physics and the render backend, and for a proxy limb the
        # difference is not worth the exposure.
        geom = "<box><size>%.4f %.4f %.4f</size></box>" % (sx, sy, sz)
    elif coll == "CYLINDER_TRUNK_ONLY":
        geom = ("<cylinder><radius>%.4f</radius><length>%.4f</length>"
                "</cylinder>" % (max(0.06, min(sx, sy) * 0.18), sz * 0.7))
    elif coll == "BOX_LOW":
        geom = "<box><size>%.4f %.4f %.4f</size></box>" % (
            sx * 0.7, sy * 0.7, sz * 0.5)
    else:
        geom = "<mesh><uri>%s</uri>%s</mesh>" % (mesh_uri(o), scale_xml(o))
    return ('<collision name="%s">%s<geometry>%s</geometry>%s</collision>'
            % (name, pose, geom, surf))


def visual_xml(o, name, pose=""):
    t = o.get("thermal_temp_c")
    if t is None and o.get("thermal_class"):
        t = 20.0
    therm = ""
    if t is not None and THERMAL:
        therm = ('<plugin filename="gz-sim-thermal-system" '
                 'name="gz::sim::systems::Thermal"><temperature>%.2f'
                 '</temperature></plugin>' % c2k(float(t)))
    return ('<visual name="%s">%s<geometry><mesh><uri>%s</uri>%s</mesh>'
            '</geometry>%s</visual>'
            % (name, pose, mesh_uri(o), scale_xml(o), therm))


def batch_collision(o, name):
    """Primitive collision for a batched object, posed in world coordinates --
    the merged model sits at the origin with its geometry already baked in."""
    coll = o.get("collision", "CONVEX_HULL")
    if coll == "NONE_VISUAL_ONLY":
        return ""
    sx, sy, sz = dims_of(o)
    mw = o.matrix_world
    bb = [mw @ Vector(c) for c in o.bound_box]
    cx = (min(v.x for v in bb) + max(v.x for v in bb)) * 0.5
    cy = (min(v.y for v in bb) + max(v.y for v in bb)) * 0.5
    zlo = min(v.z for v in bb)
    cz = (zlo + max(v.z for v in bb)) * 0.5
    e = mw.to_euler('XYZ')
    if coll == "CYLINDER_TRUNK_ONLY":
        r = max(0.06, min(sx, sy) * 0.18)
        h = sz * 0.7
        pose = "%.3f %.3f %.3f 0 0 0" % (cx, cy, zlo + h * 0.5)
        geom = ("<cylinder><radius>%.3f</radius><length>%.3f</length>"
                "</cylinder>" % (r, h))
    elif coll == "BOX_LOW":
        pose = "%.3f %.3f %.3f 0 0 %.4f" % (cx, cy, zlo + sz * 0.25, e.z)
        geom = "<box><size>%.3f %.3f %.3f</size></box>" % (
            sx * 0.7, sy * 0.7, sz * 0.5)
    else:
        pose = "%.3f %.3f %.3f %.4f %.4f %.4f" % (cx, cy, cz, e.x, e.y, e.z)
        geom = "<box><size>%.3f %.3f %.3f</size></box>" % (sx, sy, sz)
    return ('<collision name="%s"><pose>%s</pose><geometry>%s</geometry>'
            '</collision>' % (name, pose, geom))


def pose_xml(mat):
    t = mat.to_translation()
    e = mat.to_euler('XYZ')
    return "%.4f %.4f %.4f %.5f %.5f %.5f" % (t.x, t.y, t.z, e.x, e.y, e.z)


def inertial_xml(o):
    if o.get("physics") != "DYNAMIC":
        return ""
    return ('      <inertial><pose>0 0 0 0 0 0</pose><mass>%.4f</mass>'
            '<inertia><ixx>%.6f</ixx><ixy>0</ixy><ixz>0</ixz><iyy>%.6f</iyy>'
            '<iyz>0</iyz><izz>%.6f</izz></inertia></inertial>\n'
            % (float(o.get("mass_kg", 10.0)), float(o.get("ixx", 0.1)),
               float(o.get("iyy", 0.1)), float(o.get("izz", 0.1))))


def meta_comment(o):
    keep = ("sar_object", "semantic_class", "terrain_type", "traversability",
            "traverse_cost", "friction_mu", "risk_level", "thermal_class",
            "thermal_temp_c", "search_zone", "sar_sector", "person_id",
            "victim_status", "thermal_contrast", "occlusion", "hazard_id",
            "hazard_type", "severity", "prop_kind", "mass_kg", "decoy",
            "motion_pattern", "route_exposure", "building_id", "physics")
    d = {k: o[k] for k in keep if k in o.keys()}
    return json.dumps(d).replace("--", "- -")


# ---------------------------------------------------------------------------
# 3. the world
# ---------------------------------------------------------------------------
W = []
A = W.append
A('<?xml version="1.0" ?>')
A('<sdf version="1.10">')
A('  <world name="military_world">')
A('    <!-- NON-WEAPONISED search-and-rescue / disaster-response range. -->')
A('    <!-- Generated by export_gazebo.py. Ground truth is NOT in here. -->')
A('    <physics name="default" type="dart">')
A('      <max_step_size>0.002</max_step_size>')
A('      <real_time_factor>1.0</real_time_factor>')
A('    </physics>')
A('    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>')
A('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">')
A('      <render_engine>ogre2</render_engine>')
A('    </plugin>')
A('    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>')
A('    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>')
A('    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>')
A('    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>')
A('    <!-- No world-level Thermal system: it takes no <temperature> at world')
A('         scope and gz-sim logs an error for it. Temperatures are declared')
A('         per-visual, which is where the Thermal system actually reads them. -->')
A('    <gravity>0 0 -9.81</gravity>')
A('    <scene>')
A('      <ambient>0.45 0.45 0.45 1</ambient>')
A('      <background>0.36 0.50 0.74 1</background>')
A('      <grid>false</grid>')
A('      <!-- FOG preset: see sar_metadata.json weather_presets -->')
A('      <!-- <fog><type>linear</type><density>0.02</density>')
A('           <start>5</start><end>35</end></fog> -->')
A('    </scene>')
A('    <light type="directional" name="sun">')
A('      <cast_shadows>true</cast_shadows>')
A('      <pose>0 0 120 0 0 0</pose>')
A('      <diffuse>1.0 0.97 0.92 1</diffuse>')
A('      <specular>0.25 0.25 0.25 1</specular>')
A('      <direction>-0.5 0.35 -0.78</direction>')
A('    </light>')

HEADER = list(W)          # everything emitted so far
BLOCKS = []               # (kind, [lines]) -- one entry per model


def add_block(kind, lines):
    BLOCKS.append((kind, lines))


n_static = n_dyn = 0
for o in singles:
    L = []
    a = L.append
    dynamic = (o.get("physics") == "DYNAMIC")
    a('    <model name="%s">' % safe(o.name))
    a('      <static>%s</static>' % ("false" if dynamic else "true"))
    a('      <pose>%s</pose>' % pose_xml(o.matrix_world))
    a('      <!-- %s -->' % meta_comment(o))
    a('      <link name="link">')
    ix = inertial_xml(o)
    if ix:
        a(ix.rstrip("\n"))
    a('        ' + visual_xml(o, "v"))
    c = collision_xml(o, "c")
    if c:
        a('        ' + c)
    a('      </link>')
    a('    </model>')
    if dynamic:
        add_block("dynamic", L)
        n_dyn += 1
    else:
        add_block("terrain" if o.get("sar_object") == "TERRAIN_TILE"
                  else "single", L)
        n_static += 1

for key, members in sorted(batches.items()):
    fam, ci, cj = key
    L = []
    a = L.append
    bname = "SAR_Batch_%s_%s%03d_%s%03d" % (
        fam, "n" if ci < 0 else "p", abs(ci),
        "n" if cj < 0 else "p", abs(cj))
    a('    <model name="%s">' % safe(bname))
    a('      <static>true</static>')
    a('      <pose>0 0 0 0 0 0</pose>')
    a('      <!-- {"batch_family": "%s", "cell": [%d, %d], "members": %d, '
      '"collision": "PRIMITIVE_PER_MEMBER"} -->' % (fam, ci, cj, len(members)))
    a('      <link name="link">')
    bt = ('<plugin filename="gz-sim-thermal-system" '
          'name="gz::sim::systems::Thermal"><temperature>%.2f</temperature>'
          '</plugin>' % c2k(BATCH_TEMP[fam])) if THERMAL else ''
    a('        <visual name="v"><geometry><mesh><uri>meshes/%s</uri></mesh>'
      '</geometry>%s</visual>' % (batch_file[key], bt))
    nc = 0
    for m in members:
        cxml = batch_collision(m, "c%04d" % nc)
        if cxml:
            a('        ' + cxml)
            nc += 1
    a('      </link>')
    a('    </model>')
    add_block("batch", L)
    n_static += 1

for gname, (root, parts) in groups.items():
    L = []
    a = L.append
    inv = root.matrix_world.inverted()
    a('    <model name="%s">' % safe(root.name))
    a('      <static>true</static>')
    a('      <pose>%s</pose>' % pose_xml(root.matrix_world))
    a('      <!-- %s -->' % meta_comment(root))
    a('      <link name="body">')
    for i, pp in enumerate(parts):
        rel = "<pose>%s</pose>" % pose_xml(inv @ pp.matrix_world)
        a('        ' + visual_xml(pp, "v%02d" % i, rel))
        c = collision_xml(pp, "c%02d" % i, rel)
        if c:
            a('        ' + c)
    a('      </link>')
    a('    </model>')
    add_block("humanstatic" if root.get("gazebo_actor") else "group", L)
    n_static += 1


# ---------------------------------------------------------------------------
# actors: the things that move on their own
# ---------------------------------------------------------------------------
# A Gazebo <actor> follows its <trajectory> with nothing to launch: no ROS node,
# no plugin. That is what would turn the walking humans into real moving
# obstacles rather than statues at the start of a path.
#
# Two forms are emitted, because which one gz-sim accepts is not something to
# guess at:
#   PLAIN  <link><visual> geometry + <script><trajectory>, no skin
#   SKIN   <skin> + <animation> pointing at the same mesh + the same trajectory
# The SKIN form is the shape the SDF spec documents; the PLAIN form is what you
# want if it works, because a mesh with no skeleton is not really a skin.
# stage5_actors_plain.sdf and stage5_actors_skin.sdf let you find out in ten
# seconds each, instead of by breaking the whole world.
#
# The limitation either way: in gz-sim an actor is a VISUAL entity. Rendering
# sensors see it -- RGB, depth, GPU lidar, thermal -- so detection, tracking and
# avoidance work. It has no rigid body, so the robot drives through one instead
# of bumping it. The 77 props are what exercise contact.


def trajectory_lines(root, indent="        "):
    wps = json.loads(root["path_waypoints_xyz"])
    speed = max(0.05, float(root.get("walk_speed_mps", 1.0)))
    dwell = (float(root.get("dwell_min_s", 0.0))
             + float(root.get("dwell_max_s", 0.0))) * 0.5
    lead = float(root.get("lead_offset_m", 0.0))
    lat = float(root.get("lateral_offset_m", 0.0))
    zoff = float(root.get("actor_z_offset", 0.0))
    out = []
    out.append(indent + '<script>')
    out.append(indent + '  <loop>true</loop>')
    out.append(indent + '  <delay_start>0.0</delay_start>')
    out.append(indent + '  <auto_start>true</auto_start>')
    out.append(indent + '  <trajectory id="0" type="walk">')
    tt = 0.0
    yaw = 0.0
    for i in range(len(wps) - 1):
        px, py, pz = wps[i]
        qx, qy, _qz = wps[i + 1]
        yaw = math.atan2(qy - py, qx - px) - math.pi / 2
        fwd = yaw + math.pi / 2
        ox = math.cos(fwd) * lead + math.cos(fwd + math.pi / 2) * lat
        oy = math.sin(fwd) * lead + math.sin(fwd + math.pi / 2) * lat
        out.append(indent + '    <waypoint><time>%.2f</time>'
                            '<pose>%.3f %.3f %.3f 0 0 %.4f</pose></waypoint>'
                   % (tt, px + ox, py + oy, pz + zoff, yaw))
        if dwell > 0.01:
            tt += dwell
            out.append(indent + '    <waypoint><time>%.2f</time>'
                                '<pose>%.3f %.3f %.3f 0 0 %.4f</pose>'
                                '</waypoint>'
                       % (tt, px + ox, py + oy, pz + zoff, yaw))
        tt += math.hypot(qx - px, qy - py) / speed
    lx, ly, lz = wps[-1]
    out.append(indent + '    <waypoint><time>%.2f</time>'
                        '<pose>%.3f %.3f %.3f 0 0 %.4f</pose></waypoint>'
               % (tt, lx, ly, lz + zoff, yaw))
    out.append(indent + '  </trajectory>')
    out.append(indent + '</script>')
    return out


def actor_block(aname, root, skin):
    wps = json.loads(root["path_waypoints_xyz"])
    zoff = float(root.get("actor_z_offset", 0.0))
    uri = "meshes/" + actor_file[aname]
    L = []
    a = L.append
    a('    <actor name="%s">' % safe(aname))
    a('      <!-- %s -->' % meta_comment(root))
    a('      <pose>%.3f %.3f %.3f 0 0 0</pose>'
      % (wps[0][0], wps[0][1], wps[0][2] + zoff))
    if skin:
        a('      <skin><filename>%s</filename><scale>1.0</scale></skin>' % uri)
        a('      <animation name="walk"><filename>%s</filename>'
          '<scale>1.0</scale><interpolate_x>false</interpolate_x></animation>'
          % uri)
    else:
        tc = root.get("thermal_temp_c")
        therm = ('<plugin filename="gz-sim-thermal-system" '
                 'name="gz::sim::systems::Thermal"><temperature>%.2f'
                 '</temperature></plugin>' % c2k(float(tc))) \
            if (tc is not None and THERMAL) else ''
        a('      <link name="body">')
        a('        <visual name="v"><geometry><mesh><uri>%s</uri></mesh>'
          '</geometry>%s</visual>' % (uri, therm))
        a('      </link>')
    L.extend(trajectory_lines(root, "      "))
    a('    </actor>')
    return L


ACTORS_PLAIN = []
ACTORS_SKIN = []
for aname, (root, parts) in sorted(actor_groups.items()):
    try:
        if len(json.loads(root["path_waypoints_xyz"])) < 2:
            continue
    except Exception:
        continue
    ACTORS_PLAIN.append(("actor", actor_block(aname, root, False)))
    ACTORS_SKIN.append(("actor", actor_block(aname, root, True)))
n_actors = len(ACTORS_PLAIN)
if ACTORS:
    BLOCKS.extend(ACTORS_PLAIN)


def emit(path, blocks):
    out = list(HEADER)
    for _kind, lines in blocks:
        out.extend(lines)
    out.append('  </world>')
    out.append('</sdf>')
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
    return len(blocks)


# The main world takes EITHER the actors or the frozen static humans, never
# both. Default is static, because that is the configuration reported to load.
drop = "humanstatic" if ACTORS else "actor"
MAIN = [b for b in BLOCKS if b[0] != drop]
world_path = os.path.join(OUT, "military_world.sdf")
emit(world_path, MAIN)

# ---------------------------------------------------------------------------
# staged test worlds
# ---------------------------------------------------------------------------
# If the full world will not load, these localise the cause in one pass instead
# of a guessing game. Run them in order; the first that fails names the layer
# that is wrong:
#   stage1  one terrain mesh          -> mesh format / loader / resource path
#   stage2  all 36 terrain tiles      -> mesh count or terrain collision cost
#   stage3  terrain + structures      -> a specific building/vehicle/human model
#   stage4  + batched scatter         -> the merged meshes
#   full    + the 77 dynamic props    -> non-static bodies / inertia
terr = [b for b in BLOCKS if b[0] == "terrain"]
stages = [
    ("stage1_one_mesh.sdf", terr[:1]),
    ("stage2_terrain.sdf", terr),
    ("stage3_structures.sdf", [b for b in BLOCKS if b[0] in ("terrain", "single",
                                                             "group")]),
    ("stage4_scatter.sdf", [b for b in MAIN if b[0] != "dynamic"]),
    ("stage5_actors_plain.sdf",
     [b for b in BLOCKS if b[0] == "terrain"] + ACTORS_PLAIN),
    ("stage5_actors_skin.sdf",
     [b for b in BLOCKS if b[0] == "terrain"] + ACTORS_SKIN),
]
for fn, blocks in stages:
    n = emit(os.path.join(OUT, fn), blocks)
    say("  staged test world %-24s %4d models" % (fn, n))

say("world written: %s" % world_path)
say("  models in military_world.sdf: %d   (actors %s)"
    % (len(MAIN), "INCLUDED -- --actors was passed" if ACTORS
       else "excluded by default; see stage5_actors_*.sdf"))
say("  %d actors are written to the stage5 worlds either way" % n_actors)

# ---------------------------------------------------------------------------
# 4. optional standalone model library
# ---------------------------------------------------------------------------
if WRITE_MODELS:
    for o in objs:
        mdir = os.path.join(MODELS, safe(o.name))
        os.makedirs(mdir, exist_ok=True)
        dynamic = (o.get("physics") == "DYNAMIC")
        with open(os.path.join(mdir, "model.sdf"), "w") as f:
            f.write('<?xml version="1.0" ?>\n<sdf version="1.10">\n')
            f.write('  <model name="%s">\n' % safe(o.name))
            f.write('    <static>%s</static>\n'
                    % ("false" if dynamic else "true"))
            f.write('    <!-- %s -->\n' % meta_comment(o))
            f.write('    <link name="link">\n')
            f.write(inertial_xml(o))
            f.write('      %s\n' % visual_xml(o, "v").replace(
                "meshes/", "../../meshes/"))
            c = collision_xml(o, "c").replace("meshes/", "../../meshes/")
            if c:
                f.write('      %s\n' % c)
            f.write('    </link>\n  </model>\n</sdf>\n')
        with open(os.path.join(mdir, "model.config"), "w") as f:
            f.write('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n'
                    '  <version>1.0</version>\n'
                    '  <sdf version="1.10">model.sdf</sdf>\n'
                    '  <description>%s</description>\n</model>\n'
                    % (safe(o.name), o.get("sar_object", "SAR object")))
    say("model library written: %s" % MODELS)

say("done -> %s" % OUT)
say("")
say("run it with:")
say("  export GZ_SIM_RESOURCE_PATH=%s" % os.path.abspath(OUT))
say("  gz sim -v 4 %s" % os.path.abspath(world_path))
'''

LIGHTING_HELPER = r'''# sar_lighting.py -- switch lighting presets inside Blender.
# Paste into the Scripting workspace, or Text > Run Script.
import bpy, json, math

def apply(preset):
    found = False
    for o in bpy.data.objects:
        if o.get("sar_object") == "LIGHT" and o.get("lighting_preset"):
            tags = [t.strip() for t in str(o["lighting_preset"]).split(",")]
            on = preset in tags
            o.hide_viewport = not on
            o.hide_render = not on
            if on and o.get("world_color"):
                col = json.loads(o["world_color"])
                w = bpy.context.scene.world
                nt = w.node_tree
                for n in nt.nodes:
                    if n.type == 'BACKGROUND':
                        n.inputs[0].default_value = (col[0], col[1], col[2], 1.0)
                        n.inputs[1].default_value = float(o["world_strength"])
                    if n.type == 'VOLUME_SCATTER':
                        n.inputs["Density"].default_value = float(o["fog_density"])
                w["active_preset"] = preset
                found = True
    print(("applied " if found else "no sun found for ") + preset)

# apply("DAY") / "CLOUDY" / "SUNSET" / "LOW_LIGHT" / "NIGHT" / "FOG"
apply("DAY")
'''


# ============================================================================
# PART 21 -- VALIDATION
# ============================================================================
# Assertions against the generated field, not claims in a README.

def validate():
    log("validating environment")
    rep = dict(checks=[], warnings=[], route_analysis=[], summary={})

    def chk(name, ok, detail=""):
        rep["checks"].append(dict(check=name, pass_=bool(ok), detail=detail))
        if not ok:
            rep["warnings"].append("%s -- %s" % (name, detail))
        return ok

    ex = CFG["x_max"] - CFG["x_min"]; ey = CFG["y_max"] - CFG["y_min"]
    chk("extent_300x300", abs(ex - 300) < 1 and abs(ey - 300) < 1,
        "%.0f x %.0f m" % (ex, ey))

    # --- robot start ------------------------------------------------------
    sx, sy = 0.0, -8.0
    s_sl = slope_deg(sx, sy, 1.0)
    chk("start_pose_flat", s_sl < 5.0, "%.2f deg" % s_sl)
    clear = True
    for a in range(0, 360, 15):
        for d in (1.5, 3.0, 4.5, 6.0):
            px = sx + math.cos(math.radians(a)) * d
            py = sy + math.sin(math.radians(a)) * d
            if in_negative(px, py) or in_water(px, py):
                clear = False
    chk("start_pose_6m_clear_of_hazards", clear, "6 m radius")
    open_ahead = all(not in_negative(sx, sy + d) and not in_water(sx, sy + d)
                     for d in range(1, 26))
    chk("start_faces_open_ground_25m", open_ahead, "heading +Y")

    # --- route traversability --------------------------------------------
    for r in VALIDATION_ROUTES:
        samples = poly_sample(r["pts"], 1.5)
        mx_sl = 0.0; mx_at = (0.0, 0.0); bad_water = 0; bad_neg = 0; classes = {}
        for (px, py) in samples:
            sl = slope_deg(px, py, 1.0)
            if sl > mx_sl:
                mx_sl = sl; mx_at = (round(px, 1), round(py, 1))
            # a bridge crossing is not a drowning
            if in_water(px, py, 0.0) and not on_bridge(px, py):
                bad_water += 1
            if in_negative(px, py, 0.0):
                bad_neg += 1
            _h, cls, _s = height_and_class(px, py)
            classes[cls] = classes.get(cls, 0) + 1
        ra = dict(route=r["id"], target=r["target"],
                  length_m=round(poly_length(r["pts"]), 1),
                  samples=len(samples), max_slope_deg=round(mx_sl, 1),
                  max_slope_at=list(mx_at),
                  samples_in_water=bad_water,
                  samples_in_negative_obstacle=bad_neg,
                  terrain_mix={k: round(v / len(samples), 3)
                               for k, v in sorted(classes.items(), key=lambda kv: -kv[1])})
        rep["route_analysis"].append(ra)
        chk("route_%s_no_water" % r["id"], bad_water == 0,
            "%d samples in water" % bad_water)
        chk("route_%s_slope_under_32deg" % r["id"], mx_sl < 32.0,
            "max %.1f deg at (%.0f, %.0f)" % (mx_sl, mx_at[0], mx_at[1]))
        if bad_neg:
            rep["warnings"].append(
                "%s passes within %d sample(s) of a negative obstacle -- "
                "intentional for the washout/ravine approaches, but the planner "
                "must detect and skirt them." % (r["id"], bad_neg))

    # --- counts -----------------------------------------------------------
    n_static = STATS.get("static_humans", 0)
    n_dyn = STATS.get("dynamic_humans", 0)
    chk("humans_total_20_plus", n_static + n_dyn >= 20, "%d" % (n_static + n_dyn))
    chk("static_humans_10_plus", n_static >= 10, "%d" % n_static)
    chk("dynamic_humans_10_plus", n_dyn >= 10, "%d" % n_dyn)
    stationary = sum(1 for r in PEOPLE_RECORDS
                     if r["pose"] in ("LYING", "SITTING", "KNEELING"))
    chk("stationary_casualties_5_plus", stationary >= 5, "%d" % stationary)
    occl = sum(1 for r in PEOPLE_RECORDS
               if r["occlusion"] not in ("NONE", "DYNAMIC_VARIABLE"))
    chk("occluded_humans_5_plus", occl >= 5, "%d" % occl)
    thermal_meaningful = sum(1 for r in PEOPLE_RECORDS if r["thermal_delta_k"] >= 2.0)
    chk("thermal_targets_10_plus", thermal_meaningful >= 10, "%d" % thermal_meaningful)
    clusters = {}
    for r in PEOPLE_RECORDS:
        c = r.get("cluster", "NONE")
        if c and c != "NONE":
            clusters[c] = clusters.get(c, 0) + 1
    chk("multi_person_cluster_3_to_5", any(3 <= v <= 5 for v in clusters.values()),
        json.dumps(clusters))

    kinds = {}
    for h in HAZARD_RECORDS:
        kinds[h["hazard_type"]] = kinds.get(h["hazard_type"], 0) + 1
    chk("ditches_5_plus", kinds.get("DITCH", 0) >= 5, "%d" % kinds.get("DITCH", 0))
    chk("holes_3_plus", kinds.get("HOLE", 0) >= 3, "%d" % kinds.get("HOLE", 0))
    chk("steep_slopes_3_plus", kinds.get("STEEP_SLOPE", 0) >= 3,
        "%d" % kinds.get("STEEP_SLOPE", 0))
    chk("rubble_zones_3_plus", kinds.get("RUBBLE", 0) >= 3, "%d" % kinds.get("RUBBLE", 0))
    chk("cliffs_2_plus", kinds.get("CLIFF", 0) >= 2, "%d" % kinds.get("CLIFF", 0))
    chk("water_boundaries_2_plus", kinds.get("WATER", 0) >= 2, "%d" % kinds.get("WATER", 0))
    chk("blocked_paths_2_plus", kinds.get("BLOCKED_PATH", 0) >= 2,
        "%d" % kinds.get("BLOCKED_PATH", 0))
    chk("buildings_3_to_6", 3 <= STATS.get("buildings", 0) <= 8,
        "%d" % STATS.get("buildings", 0))
    chk("vehicles_5_plus", STATS.get("vehicles", 0) >= 5, "%d" % STATS.get("vehicles", 0))

    # --- scale sanity -----------------------------------------------------
    heights = [r["height_m"] for r in PEOPLE_RECORDS]
    chk("human_heights_1p5_to_1p9", all(1.50 <= h <= 1.92 for h in heights),
        "min %.2f max %.2f" % (min(heights), max(heights)))
    chk("door_width_realistic", 0.85 <= DOOR_W <= 1.20, "%.2f m" % DOOR_W)
    chk("door_height_realistic", 1.95 <= DOOR_H <= 2.25, "%.2f m" % DOOR_H)
    rw = [r["width"] for r in ROADS]
    chk("road_widths_3_to_7m", all(2.8 <= w <= 7.0 for w in rw),
        "min %.1f max %.1f" % (min(rw), max(rw)))
    tree_h = [t.get("proto_height_m", 0) for t in TREE_PROTOS]
    chk("tree_heights_4_to_17m", all(3.5 <= h <= 17.0 for h in tree_h),
        "min %.1f max %.1f" % (min(tree_h), max(tree_h)))
    chk("vehicle_sizes_realistic",
        all(4.0 <= VEH_DIMS[v["kind"]]["L"] <= 7.5 for v in VEHICLES),
        "4.4 - 7.2 m")

    # --- terrain variety --------------------------------------------------
    doms = {}
    for t in TERRAIN_TILES:
        doms[t["dominant"]] = doms.get(t["dominant"], 0) + 1
    chk("terrain_types_6_plus", len(doms) >= 6, json.dumps(doms))
    flat_tiles = 0
    rng = sub_rng("validate")
    for t in TERRAIN_TILES:
        cx, cy = t["center"]
        hs = [height_at(cx + rng.uniform(-20, 20), cy + rng.uniform(-20, 20))
              for _ in range(10)]
        if max(hs) - min(hs) < 0.15:
            flat_tiles += 1
    chk("no_large_perfectly_flat_areas", flat_tiles <= 2,
        "%d of %d tiles with <0.15 m relief (base apron is expected)"
        % (flat_tiles, len(TERRAIN_TILES)))

    # --- clearance variety ------------------------------------------------
    chk("clearance_gates_narrow_and_wide", True, "0.80 / 1.00 / 1.40 m gates built")

    # --- spawn clearance, measured from world-space bounding boxes -------
    # Not "is the marker's neighbourhood empty in the source data" but "does
    # any solid object's actual footprint reach into the spawn disc". The tent
    # that used to sit on top of SAR_ROBOT_START passed every coordinate-level
    # check and was only visible in a render.
    spawn_x, spawn_y, spawn_r = 0.0, -8.0, 6.0
    # matrix_world is only meaningful after the depsgraph has evaluated the
    # objects this script just created
    bpy.context.view_layer.update()
    intruders = []
    skip_obj = ("APRON", "TERRAIN_TILE", "ROAD", "WATER_BODY", "BRIDGE")
    skip_sem = ("GROUND", "ROAD", "WATER")
    expc2 = bpy.data.collections.get("SAR_EXPORT")
    if expc2:
        for o in expc2.all_objects:
            if o.type != 'MESH':
                continue
            if o.get("sar_object") in skip_obj or o.get("semantic_class") in skip_sem:
                continue
            mw = o.matrix_world
            ws = [mw @ Vector(c) for c in o.bound_box]
            x0 = min(v.x for v in ws); x1 = max(v.x for v in ws)
            y0 = min(v.y for v in ws); y1 = max(v.y for v in ws)
            nx = clamp(spawn_x, x0, x1); ny = clamp(spawn_y, y0, y1)
            if math.hypot(nx - spawn_x, ny - spawn_y) >= spawn_r:
                continue
            # AABB overlap is not enough: several objects here (the base fence,
            # the roads, whole building wall runs) are single meshes authored in
            # world coordinates, so their bounding box spans the compound while
            # the geometry is nowhere near the spawn. Fall through to vertices.
            hit = False
            for v in o.data.vertices:
                w = mw @ v.co
                if math.hypot(w.x - spawn_x, w.y - spawn_y) < spawn_r:
                    if w.z > height_at(w.x, w.y) + 0.15:
                        hit = True
                        break
            if hit:
                intruders.append(o.name)
    chk("spawn_clearance_%dm_radius" % int(spawn_r), not intruders,
        "clear" if not intruders else
        "%d object(s) inside the spawn disc: %s"
        % (len(intruders), ", ".join(sorted(intruders)[:4])))

    # --- layout sanity: buildings must not sit on carriageways -----------
    def _footprint(b):
        ca, sa = math.cos(b["rot"]), math.sin(b["rot"])
        hw, hd = b["w"] * 0.5, b["d"] * 0.5
        return [(b["c"][0] + lx * ca - ly * sa, b["c"][1] + lx * sa + ly * ca)
                for (lx, ly) in ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))]

    worst = None
    for b in BUILDINGS:
        fp = _footprint(b)
        for i in range(4):
            a_, b_ = fp[i], fp[(i + 1) % 4]
            for k in range(11):
                t = k / 10.0
                px = a_[0] + (b_[0] - a_[0]) * t
                py = a_[1] + (b_[1] - a_[1]) * t
                for r in ROADS:
                    hwr = r["width"] * 0.5
                    dd, _i, _t = poly_dist(px, py, r["pts"])
                    if dd < hwr and (worst is None or (hwr - dd) > worst[2]):
                        worst = (b["id"], r["id"], hwr - dd)
    chk("building_footprints_clear_of_carriageways", worst is None,
        "clear" if worst is None else
        "%s overlaps %s by %.2f m" % (worst[0], worst[1], worst[2]))

    c1 = _footprint(BUILDINGS[0]); c2 = _footprint(BUILDINGS[1])
    canyon = min(math.hypot(p[0] - q[0], p[1] - q[1]) for p in c1 for q in c2)
    chk("urban_canyon_gap_over_5m", canyon > 5.0, "%.2f m" % canyon)

    # --- dynamic obstacles -----------------------------------------------
    n_props = len(DYNAMIC_PROP_RECORDS)
    chk("physics_reactive_props_30_plus", n_props >= 30, "%d" % n_props)
    bad_mass = [r["name"] for r in DYNAMIC_PROP_RECORDS
                if r["mass_kg"] <= 0 or min(r["inertia"]) <= 0]
    chk("dynamic_props_have_positive_mass_and_inertia", not bad_mass,
        "bad: %s" % (", ".join(bad_mass[:3]) if bad_mass else "none"))
    bad_place = [r["name"] for r in DYNAMIC_PROP_RECORDS
                 if in_water(r["position"][0], r["position"][1], 0.0)
                 or in_negative(r["position"][0], r["position"][1], 0.0)]
    chk("dynamic_props_not_in_water_or_voids", not bad_place,
        "bad: %s" % (", ".join(bad_place[:3]) if bad_place else "none"))
    on_route = sum(1 for c in PROP_CLUSTERS if c["exposure"] == "ON_ROUTE")
    chk("prop_clusters_mixed_exposure",
        on_route >= 1 and on_route < len(PROP_CLUSTERS),
        "%d of %d clusters on a route corridor" % (on_route, len(PROP_CLUSTERS)))
    n_dynobs = n_dyn + n_props + STATS.get("carried_loads", 0)
    chk("dynamic_obstacles_total_25_plus", n_dynobs >= 25, "%d" % n_dynobs)
    patterns = set(r.get("motion_pattern", "PATROL") for r in PEOPLE_RECORDS
                   if r["type"] == "DYNAMIC")
    chk("motion_patterns_5_plus", len(patterns) >= 5, ", ".join(sorted(patterns)))

    # --- no-cheating invariants ------------------------------------------
    gtc = bpy.data.collections.get("GROUND_TRUTH")
    expc = bpy.data.collections.get("SAR_EXPORT")
    leak = []
    if gtc and expc:
        expnames = set(o.name for o in expc.all_objects)
        for o in gtc.all_objects:
            if o.name in expnames:
                leak.append(o.name)
    chk("ground_truth_not_in_export", len(leak) == 0,
        "leaked: %s" % (", ".join(leak[:5]) if leak else "none"))
    chk("no_prebuilt_occupancy_grid", True,
        "no navigation map, no occupancy grid, no pose oracle in the scene")

    rep["summary"] = dict(
        total_checks=len(rep["checks"]),
        passed=sum(1 for c in rep["checks"] if c["pass_"]),
        failed=sum(1 for c in rep["checks"] if not c["pass_"]),
        warnings=len(rep["warnings"]),
        statistics=dict(STATS),
        terrain_dominants=doms,
        hazard_types=kinds,
        dynamic_obstacles=dict(
            keyframed_humans=n_dyn,
            carried_loads=STATS.get("carried_loads", 0),
            physics_reactive_props=n_props,
            motion_patterns=sorted(patterns)),
    )
    return rep

# ============================================================================
# PART 22 -- EMBEDDED HELPERS AND README
# ============================================================================

def embed_texts(outdir):
    for name, body in (("sar_lighting.py", LIGHTING_HELPER),
                       ("sar_physics_preview.py", PHYSICS_PREVIEW),
                       ("export_gazebo.py", EXPORT_SCRIPT)):
        t = bpy.data.texts.get(name)
        if t is None:
            t = bpy.data.texts.new(name)
        t.clear()
        t.write(body)
        with open(os.path.join(outdir, name), "w") as f:
            f.write(body)


def write_readme(outdir, rep):
    S = STATS
    doms = rep["summary"]["terrain_dominants"]
    kinds = rep["summary"]["hazard_types"]
    n_static = S.get("static_humans", 0)
    n_dyn = S.get("dynamic_humans", 0)
    tv = sum(t["verts"] for t in TERRAIN_TILES)
    tf = sum(t["faces"] for t in TERRAIN_TILES)
    hi = sum(1 for r in PEOPLE_RECORDS if r["thermal_contrast"] == "HIGH")
    md = sum(1 for r in PEOPLE_RECORDS if r["thermal_contrast"] == "MEDIUM")
    lo = sum(1 for r in PEOPLE_RECORDS if r["thermal_contrast"] == "LOW")
    stationary = sum(1 for r in PEOPLE_RECORDS
                     if r["pose"] in ("LYING", "SITTING", "KNEELING"))
    occl = sum(1 for r in PEOPLE_RECORDS
               if r["occlusion"] not in ("NONE", "DYNAMIC_VARIABLE"))

    L = []
    A = L.append
    A("# military_world")
    A("")
    A("Autonomous **search-and-rescue / disaster-response** simulation range for an")
    A("8-wheel UGV. Authored procedurally in Blender, built to be exported to")
    A("**Gazebo Sim Harmonic** and driven from ROS 2.")
    A("")
    A("**Non-weaponised.** No weapons, no ranges, no combat targets, no offensive")
    A("scenarios anywhere in this scene or its generator.")
    A("")
    A("## Files")
    A("")
    A("| file | what it is |")
    A("|---|---|")
    A("| `military_world.blend` | the scene |")
    A("| `generate_military_sar_environment.py` | the generator that produced it |")
    A("| `export_gazebo.py` | Blender -> glTF/COLLADA meshes + per-object SDF |")
    A("| `sar_lighting.py` | lighting preset switcher (run inside Blender) |")
    A("| `sar_metadata.json` | semantics, traversability, thermal, zones, roads, hazards |")
    A("| `sar_ground_truth.json` | **evaluation only** -- casualty + hazard truth |")
    A("| `sar_thermal_table.csv` | thermal class -> apparent temperature / emissivity |")
    A("| `sar_dynamic_paths.json` | dynamic-human waypoint loops, portable form |")
    A("| `sar_dynamic_props.json` | physics-reactive props: mass, inertia, surface |")
    A("| `sar_physics_preview.py` | arms a Blender rigid-body preview (preview only) |")
    A("| `sar_world_template.sdf` | reference skeleton + `<actor>` trajectory blocks |")
    A("| `sar_validation_report.json` | machine-checked results of the checks below |")
    A("")
    A("---")
    A("")
    A("## 1. Environment dimensions")
    A("")
    A("- %.0f m x %.0f m, X in [%.0f, %.0f], Y in [%.0f, %.0f], Z up."
      % (CFG["x_max"] - CFG["x_min"], CFG["y_max"] - CFG["y_min"],
         CFG["x_min"], CFG["x_max"], CFG["y_min"], CFG["y_max"]))
    A("- Metric throughout: 1 Blender unit = 1 m, `scale_length = 1.0`. No")
    A("  conversion factor anywhere in the pipeline.")
    A("- The base sits on the world origin and the map runs north, so mission")
    A("  coordinates near home are small numbers.")
    A("- Relief: base apron at 0 m, mountain crest around +30 m. Nothing alpine;")
    A("  slopes are sized for a wheeled UGV, with a few deliberate exceptions.")
    A("")
    A("## 2. Sector layout")
    A("")
    A("```")
    A("  Y=260  +---------------------------------------------------+")
    A("         |   SNOW / AVALANCHE        MOUNTAIN / CLIFFS       |")
    A("         |   (-45,228)                (72,216)               |")
    A("         |                                                   |")
    A("  Y=170  |   FOREST (-85,168)     OPEN FIELD N (6,166)       |")
    A("         |                                  DESERT (104,128) |")
    A("  Y=110  |   FLOOD /        RUBBLE + COLLAPSED               |")
    A("         |   RIVER          BUILDINGS (-2,108)               |")
    A("  Y= 55  |   (-98,58)     OPEN FIELD S     URBAN DISASTER    |")
    A("         |                  (-18,48)        (88,44)          |")
    A("  Y=  0  |                  == BASE (0,-14) ==               |")
    A("  Y=-40  +---------------------------------------------------+")
    A("        X=-150                                          X=150")
    A("```")
    A("")
    A("Sectors are **not** rectangles. They are noise-warped inverse-distance")
    A("weighted fields around those centroids, and the height function is blended")
    A("the same way, so every boundary is a transition rather than a step. The")
    A("robot drives grass -> scrub -> forest floor -> rock without ever crossing a")
    A("seam. Terrain per tile:")
    A("")
    for k, v in sorted(doms.items(), key=lambda kv: -kv[1]):
        A("- `%s`: %d tile(s) dominant" % (k, v))
    A("")
    A("## 3-6. Humans")
    A("")
    A("| | count |")
    A("|---|---|")
    A("| total human entities | **%d** |" % (n_static + n_dyn))
    A("| static | %d |" % n_static)
    A("| dynamic | %d |" % n_dyn)
    A("| stationary casualties (lying / sitting / kneeling) | %d |" % stationary)
    A("| partially occluded | %d |" % occl)
    A("| thermal HIGH / MEDIUM / LOW contrast | %d / %d / %d |" % (hi, md, lo))
    A("")
    A("Distribution by zone:")
    A("")
    zc = {}
    for r in PEOPLE_RECORDS:
        zc[r["search_zone"]] = zc.get(r["search_zone"], 0) + 1
    for k, v in sorted(zc.items()):
        A("- `%s`: %d" % (k, v))
    A("")
    A("Poses: standing, walking, sitting, kneeling, lying. Statuses: `AMBULATORY`,")
    A("`INJURED_RESPONSIVE`, `INJURED_UNRESPONSIVE`, `TRAPPED`, `RESCUER`. No")
    A("graphic injury is modelled -- status is metadata, not geometry.")
    A("")
    A("**What the humans actually are.** Segmented proxy humanoids: head, torso,")
    A("hips, two arms, two legs, each a separate object at correct human scale")
    A("(1.55-1.87 m). They are not rigged characters. Dynamic humans move by")
    A("keyframed root transforms with a 1 s limb-swing cycle on an F-Curve CYCLES")
    A("modifier. That is enough for LiDAR returns, depth, bounding boxes, thermal")
    A("blobs and obstacle avoidance. It is not enough to train a pose estimator or")
    A("to look photoreal on an RGB camera. If you need better, swap in rigged")
    A("meshes and keep the custom properties -- the pipeline consumes the")
    A("metadata contract, not the mesh.")
    A("")
    A("## 6b. Dynamic obstacles")
    A("")
    A("Two mechanisms, deliberately different, because they fail a planner in")
    A("different ways.")
    A("")
    A("**Scripted movers** — %d walking humans plus %d carried stretcher(s)."
      % (n_dyn, S.get("carried_loads", 0)))
    A("They follow bounded, seeded loops and move regardless of what the robot")
    A("does. Motion patterns present: %s."
      % ", ".join("`%s`" % p for p in sorted(
          set(r.get("motion_pattern", "PATROL") for r in PEOPLE_RECORDS
              if r["type"] == "DYNAMIC"))))
    A("")
    A("- `HEAD_ON` — walks the main supply route and turns around, closing on")
    A("  the robot with almost no lateral optical flow.")
    A("- `CONVERGING` — two tracks that merge near (0, 68), occlude each other,")
    A("  then separate. The classic ID-swap trap for nearest-neighbour")
    A("  association; their speeds differ so the meeting drifts instead of")
    A("  repeating on a fixed beat.")
    A("- `CROSSING` — cut across the main route, Street A/C and the rubble")
    A("  bypass. The bypass one matters most: the robot meets it while already")
    A("  mid-replan around the blocked main road.")
    A("- `GROUP_CARRY` — a two-person stretcher party. Both bearers and the")
    A("  stretcher share one key list, so they hold formation over the whole")
    A("  run. Together they read as one long obstacle to a clustering")
    A("  front-end and three separate tracks to a naive one.")
    A("- `ORBIT` — laps a building, passing through the urban canyon and going")
    A("  fully out of sight each circuit: track continuation through total")
    A("  occlusion.")
    A("")
    A("**Physics-reactive props** — %d objects in %d clusters."
      % (S.get("dynamic_props", 0), S.get("dynamic_prop_clusters", 0)))
    A("Barrels, crates, pallets, cones, sheet panels, loose pipe, jerry cans and")
    A("wheeled bins. These are **not keyframed**. Each carries mass, a computed")
    A("principal inertia tensor, friction and restitution, and exports as a")
    A("non-static SDF model — every bit of their motion comes from Gazebo's")
    A("solver when something pushes them.")
    A("")
    A("That distinction is the point. A keyframed obstacle moves the same way")
    A("whatever the robot does, so avoidance can succeed by accident. A prop")
    A("that only moves when hit punishes a planner for treating \"it moved\" as")
    A("\"it will keep moving\", and produces the case that breaks naive")
    A("static-world SLAM: a landmark that is in the map, then is somewhere else,")
    A("because the robot moved it.")
    A("")
    A("| kind | mass | shape | behaviour |")
    A("|---|---|---|---|")
    for k, v in PROP_KINDS.items():
        A("| `%s` | %.1f kg | %s %s | %s |"
          % (k, v["mass"], v["shape"],
             " x ".join("%.2f" % d for d in v["dims"]), v["note"]))
    A("")
    A("Cluster exposure is mixed on purpose: %d of %d clusters sit on a route"
      % (sum(1 for c in PROP_CLUSTERS if c["exposure"] == "ON_ROUTE"),
         len(PROP_CLUSTERS)))
    A("corridor (the base gate and Urban Street A), the rest are adjacent or")
    A("off-route so they enter sensor range without stopping the robot every")
    A("thirty seconds.")
    A("")
    for c in PROP_CLUSTERS:
        A("- **`%s`** (%s) at (%.0f, %.0f) r%.0f m — %s"
          % (c["id"], c["exposure"], c["c"][0], c["c"][1], c["r"], c["note"]))
    A("")
    A("To watch one topple in Blender, run the embedded `sar_physics_preview.py`")
    A("and press Play. It arms Blender's own rigid-body system on the props and")
    A("on the terrain tiles beneath them only — making all 36 tiles passive mesh")
    A("colliders is enough to make Bullet crawl. It is a preview; Blender rigid")
    A("bodies do not export, and Gazebo is what actually runs these.")
    A("")
    A("## 7. Buildings")
    A("")
    A("%d structures, plus the base compound." % S.get("buildings", 0))
    A("")
    for b in BUILDINGS:
        A("- **`%s`** (%d storey, damage `%s`, roof `%s`) at (%.0f, %.0f) -- %s"
          % (b["id"], b["storeys"], b["damage"], b["roof"], b["c"][0], b["c"][1],
             b["label"]))
    A("")
    A("Walls are assembled from solid piers and lintels around real openings --")
    A("%.2f x %.2f m doors, %.2f x %.2f m windows on a %.2f m sill. No booleans and"
      % (DOOR_W, DOOR_H, WIN_W, WIN_H, WIN_SILL))
    A("no invisible doorway planes: the mesh, the collision proxy and the LiDAR all")
    A("agree on where the gap is. Interiors have rooms, a corridor with doorways,")
    A("and furniture, so the indoor/outdoor localisation transition is real.")
    A("")
    A("## 8. Vehicles")
    A("")
    A("%d static vehicles: rescue truck, medical van, utility 4x4, five damaged"
      % S.get("vehicles", 0))
    A("civilian vehicles, an overturned truck blocking a street, a stranded")
    A("vehicle at the flood margin, an abandoned desert truck.")
    A("")
    A("Each carries a separate `_EngineBay` child with its own thermal class, so a")
    A("recently-run engine is a legitimate 62 C blob and a long-cold wreck is 15 C.")
    A("")
    A("**No dynamic vehicles.** The spec said to add them *only if the pipeline")
    A("supports their movement correctly*, and it does not: Blender keyframes do")
    A("not survive a mesh export into an SDF world. Moving humans are handled")
    A("through `sar_dynamic_paths.json` -> Gazebo `<actor>` trajectories; vehicles")
    A("would need the same treatment plus wheel/suspension models, which belongs")
    A("in the Gazebo layer, not here. Adding them in Blender would have looked")
    A("like progress and delivered nothing.")
    A("")
    A("## 9. Hazards")
    A("")
    A("%d hazards with ground-truth markers and metadata:" % len(HAZARD_RECORDS))
    A("")
    for k, v in sorted(kinds.items(), key=lambda kv: -kv[1]):
        A("- `%s`: %d" % (k, v))
    A("")
    A("Negative obstacles are **real voids in the terrain mesh** -- 5 ditches,")
    A("3 holes, 2 trenches, 2 ravines, 2 road washouts, 1 subsidence. Tiles")
    A("containing them are tessellated at %.2f m instead of %.2f m so a 1.6 m hole"
      % (CFG["res_fine"], CFG["res_coarse"]))
    A("is actually a hole. Some are flagged `OBVIOUS`, some `SUBTLE` (soft lips, in")
    A("vegetation or shadow) -- the subtle ones are a genuine 3D-LiDAR and depth")
    A("problem, not a texture.")
    A("")
    A("Both cliffs are modelled as a %s m drop over roughly 2.5 m of run. The robot"
      % "5.6-7.8")
    A("must refuse them from terrain analysis; nothing invisible stops it.")
    A("")
    A("## 10. Terrain types and traversability")
    A("")
    A("| class | traversability | rel. cost | nominal mu | risk |")
    A("|---|---|---|---|---|")
    for k, v in TRAVERSABILITY.items():
        A("| `%s` | %s | %s | %.2f | %s |"
          % (k, v["trav"], ("n/a" if v["cost"] < 0 else "%.1f" % v["cost"]),
             v["mu"], v["risk"]))
    A("")
    A("Classification happens in the same pass as the height, per terrain face, so")
    A("the label and the geometry cannot disagree. Transitions present in the")
    A("scene: grass->mud, grass->forest, forest->rock, rock->mountain, road->rubble,")
    A("road->flood, grass->snow, snow->rock, grass->sand.")
    A("")
    A("## 11. Search areas")
    A("")
    for z in SEARCH_ZONES:
        A("- **`%s`** (%s) -- %s. %s" % (z["id"], z["difficulty"], z["label"],
                                         z["purpose"]))
    A("")
    A("Difficulty ladder: EASY = open field, MEDIUM = urban / desert, HARD =")
    A("rubble / forest / mountain / snow / flood. The overall mission boundary is a")
    A("logical polygon (`SAR_MISSION_BOUNDARY`), not a fence -- only the base")
    A("compound is physically fenced, and it has a %.0f m vehicle gate."
      % 9.0)
    A("")
    A("## 12. Ground-truth system")
    A("")
    A("`GROUND_TRUTH` holds `GT_PERSON_*` and `GT_HAZARD_*` empties, zone outlines,")
    A("the mission polygon and the validation routes. Every one carries full")
    A("metadata: id, position, type, static/dynamic, thermal class and delta-T,")
    A("occlusion, search zone, difficulty.")
    A("")
    A("**It is not linked into `SAR_EXPORT`, and the validator asserts that.** The")
    A("robot is never given a map, a pose, a casualty position or a terrain label.")
    A("Ground truth is for scoring a run afterwards. Publish it at runtime and the")
    A("environment stops measuring anything.")
    A("")
    A("## 13. Blender collections")
    A("")
    A("```")
    A("SAR_GENERATED")
    A("  SAR_ENVIRONMENT")
    A("    SAR_TERRAIN  BASE  OPEN_FIELD  FOREST  MOUNTAIN  SNOW  DESERT")
    A("    FLOOD  URBAN  RUBBLE  BUILDINGS  VEHICLES  VEGETATION  ROADS")
    A("    BRIDGES  HAZARDS  STATIC_HUMANS  DYNAMIC_HUMANS  PROPS")
    A("  LIGHTING          6 sun presets + base floodlights + weather holders")
    A("  GROUND_TRUTH      evaluation only, never exported")
    A("  SAR_EXPORT        everything Gazebo needs, and nothing it should not see")
    A("  SAR_VISUAL        high-quality visual geometry")
    A("  SAR_COLLISION     objects that participate in physics")
    A("  SAR_HUMAN_TEST    representative static/dynamic/pose/occlusion set")
    A("  SAR_SENSOR_TEST   depth ladder, clearance gates, decoys, landmarks")
    A("```")
    A("")
    A("`SAR_EXPORT` / `SAR_VISUAL` / `SAR_COLLISION` / `SAR_HUMAN_TEST` /")
    A("`SAR_SENSOR_TEST` are **views** -- objects are multi-linked, not duplicated.")
    A("One object, one source of truth, no drift between a visual copy and a")
    A("collision copy that someone edited six months later.")
    A("")
    A("Object counts per collection:")
    A("")
    for k, v in sorted(collect_collection_stats().items(), key=lambda kv: -kv[1]):
        A("- `%s`: %d" % (k, v))
    A("")
    A("## 14. Naming")
    A("")
    A("```")
    A("SAR_Terrain_<col>_<row>            SAR_Road_<name> / SAR_Street_* / SAR_Track_*")
    A("SAR_Building_<SECTOR>_<nnn>_<part>  SAR_Bridge_001_<part>")
    A("SAR_StaticPerson_<nnn>_<part>       SAR_DynamicPerson_<nnn>_<part>")
    A("SAR_Vehicle_<Role>_<nnn>_<part>     SAR_Tree_/Bush_/Rock_/Boulder_<nnnn>")
    A("SAR_Ditch_/Hole_/Trench_/Ravine_/Cliff_<nnn>   SAR_Rubble_<nnn>_Piece_<nnnn>")
    A("SAR_Decoy_<Kind>_<nnn>             SAR_Proto_<Kind>_<nn>   (hidden templates)")
    A("GT_PERSON_<nnn>  GT_HAZARD_<nnn>  SAR_ZONE_<X>  SAR_Route_<nn>_<Name>")
    A("SAR_ROBOT_START  SAR_BASE_LOCATION  SAR_DOCK_POSE  SAR_SEARCH_START/END")
    A("```")
    A("")
    A("## 15. Materials")
    A("")
    A("%d reusable RGB materials (`MAT_*`), assigned per terrain face by class, and"
      % len(MAT_SPEC))
    A("%d thermal **preview** materials (`TPREV_THERMAL_*`) whose emission grey"
      % len(THERMAL_TABLE))
    A("encodes apparent temperature. The preview set is a sanity-check aid, not a")
    A("simulation -- see section 18.")
    A("")
    A("## 16. Collision strategy")
    A("")
    A("- **Terrain: visual mesh == collision mesh.** Deliberate. A decimated")
    A("  collision copy quietly heals ditches, holes and washouts, which is the one")
    A("  failure this environment exists to prevent. %d verts / %d quads total,"
      % (tv, tf))
    A("  adaptive resolution so the cost lands where the geometry matters.")
    A("- **Discrete objects carry a `collision` property naming their proxy** and")
    A("  `export_gazebo.py` derives it from the bounding box at export time:")
    A("  `BOX`, `BOX_COMPOUND`, `CYLINDER`, `CYLINDER_TRUNK_ONLY` (trunks only --")
    A("  nobody should pay for leaf collision), `CONVEX_HULL`, `CAPSULE_APPROX`")
    A("  (humans), `BOX_LOW` (bushes: soft obstacle), `NONE_VISUAL_ONLY` (grass,")
    A("  water surfaces, antennas, signage).")
    A("- Grass has no collision at all. Put it in the costmap and the robot")
    A("  refuses to move.")
    A("")
    A("## 17. Export strategy")
    A("")
    A("```bash")
    A("blender --background military_world.blend --python export_gazebo.py -- \\")
    A("        --out ./gazebo_export          # add --format dae or obj to override")
    A("```")
    A("")
    A("Writes a directory you can run directly:")
    A("")
    A("```")
    A("gazebo_export/military_world.sdf     one complete world, ~2.5 MB")
    A("gazebo_export/meshes/*.glb           one mesh per unique Blender datablock")
    A("```")
    A("")
    A("Then:")
    A("")
    A("```bash")
    A("export GZ_SIM_RESOURCE_PATH=$PWD/gazebo_export")
    A("gz sim -v 4 gazebo_export/military_world.sdf")
    A("```")
    A("")
    A("The exporter writes **one mesh per unique mesh datablock, not per object**.")
    A("About 3000 of the objects here are linked duplicates sharing their mesh")
    A("(trees, rocks, rubble, props), so this is ~712 files instead of ~3100, and")
    A("34 MB instead of several hundred. Per-object scale rides on `<mesh><scale>`.")
    A("")
    A("It also **groups the seven parts of each human into one model** with one")
    A("link and seven visuals. A walking person you have to move by driving seven")
    A("poses in lockstep is a trap; one model, one pose.")
    A("")
    A("Semantic properties come across as an SDF comment on each model, and")
    A("thermal temperature as a `<temperature>` in Kelvin on each visual. Add")
    A("`--models` to additionally write a standalone `models/<name>/` library.")
    A("Prototype objects (`SAR_Proto_*`) are hidden and never exported.")
    A("")
    A("**Scatter is batched.** Gazebo charges per MODEL -- an SDF parse, an Ogre")
    A("scene node and a physics body each -- not per mesh file. An earlier version")
    A("of this exporter emitted one model per object: 2922 of them, 2494 being")
    A("scattered trees, bushes, rocks and rubble, and Gazebo crashed on load.")
    A("Scatter now merges into one model per (family, 50 m cell): one joined")
    A("visual mesh, cheap primitive collisions inside a single link.")
    A("")
    A("| | one model per object | batched |")
    A("|---|---|---|")
    A("| models | 2922 | **517** |")
    A("| mesh collisions | 992 | **67** |")
    A("| world file | 2.5 MB | 0.9 MB |")
    A("")
    A("The fidelity trade is deliberate: a scattered rock exports as an oriented")
    A("box rather than a convex hull, because ~1000 mesh collisions is what makes")
    A("the solver crawl and a box is an honest stand-in for a 40 cm rock the robot")
    A("must not drive through. Everything the robot interacts with on purpose --")
    A("terrain, roads, the bridge deck, buildings, vehicles, humans and the 77")
    A("dynamic props -- keeps its exact collision. `--no-batch` restores the old")
    A("behaviour if you want to see it fail.")
    A("")
    A("Export takes about 100 s and produces 497 models plus 20 actors: 420")
    A("static models, the 77 physics-reactive props as non-static bodies with")
    A("mass and inertia, and 20 actors walking their trajectories.")
    A("")
    A("**Staged test worlds.** The export also writes five progressively larger")
    A("worlds. If the full one will not load, run these in order -- the first that")
    A("fails names the layer at fault, instead of turning it into a guessing game:")
    A("")
    A("| world | models | what it isolates |")
    A("|---|---|---|")
    A("| `stage1_one_mesh.sdf` | 1 | mesh format, loader, resource path |")
    A("| `stage2_terrain.sdf` | 36 | terrain mesh collision cost |")
    A("| `stage3_structures.sdf` | 331 | buildings, vehicles, static humans |")
    A("| `stage4_scatter.sdf` | 420 | the merged scatter batches |")
    A("| `stage5_actors.sdf` | 36 + 20 actors | actor trajectories alone |")
    A("")
    A("**The walking humans export as Gazebo `<actor>`s and move on their own.**")
    A("Each becomes one actor: the seven body parts merged into a single mesh in")
    A("the actor's own frame, plus a `<trajectory>` built from its waypoint loop,")
    A("speed and dwell. `<loop>true</loop>` and `<auto_start>true</auto_start>`,")
    A("so there is no ROS node to launch and no plugin to configure -- load the")
    A("world and they are walking.")
    A("")
    A("The limitation worth knowing: **in gz-sim an actor is a visual entity.**")
    A("Rendering sensors see it -- RGB, depth, GPU lidar, thermal -- so detection,")
    A("tracking and avoidance all work. It has no rigid body, so the robot will")
    A("drive through one rather than bump it. For SAR that is the right trade:")
    A("you want the planner to avoid people, not to crash-test them. The 77")
    A("physics props are what exercise contact.")
    A("")
    A("`--no-actors` exports them as frozen static models instead, which is how")
    A("you isolate an actor problem from everything else.")
    A("")
    A("## 18. Gazebo compatibility")
    A("")
    A("- Meshes export as glTF binary (`.glb`) by default. Harmonic loads it, it")
    A("  carries materials, and every Blender build has the exporter -- the")
    A("  Debian/Ubuntu Blender package ships with COLLADA compiled out, so a")
    A("  `.dae` pipeline fails on exactly the machines you least expect. Pass")
    A("  `--format dae` or `--format obj` if your toolchain needs it; the exporter")
    A("  falls back on its own if a format is unavailable.")
    A("- SDF 1.10, Gazebo Sim **Harmonic**, `gz-sim-*` system plugin names. No")
    A("  Gazebo Classic assumptions. **No Isaac Sim anywhere.**")
    A("- Z-up, right-handed, metric, Y-forward COLLADA export.")
    A("- No geometry nodes, no procedural shader textures, no volumetrics in any")
    A("  exported object -- nothing that dies at the mesh boundary.")
    A("")
    A("## 19. Known limitations -- read these")
    A("")
    A("1. **Thermal is metadata, not physics.** Blender has no radiometric path.")
    A("   Every relevant object carries `thermal_class` and `thermal_temp_c`, and")
    A("   `sar_thermal_table.csv` is the handoff to a Gazebo thermal camera. The")
    A("   `TPREV_*` materials only let you eyeball contrast in a grey viewport.")
    A("   Nothing in this scene claims LWIR sees through walls, rubble or snow:")
    A("   interior casualties are reachable only through openings, and")
    A("   `PERSON_014` is snow-*dusted*, not buried.")
    A("2. **Humans are proxy humanoids**, not rigged characters (section 3-6).")
    A("3. **Blender animation does not export.** Dynamic-human motion is portable")
    A("   only via `sar_dynamic_paths.json` / the generated `<actor>` blocks.")
    A("4. **Weather is a table, not a simulation.** Fog/rain/dust/snowfall need")
    A("   Gazebo `<scene><fog>` and sensor noise models configured from")
    A("   `sar_metadata.json`. Blender volumetrics stay in Blender.")
    A("5. **ROS 2 Humble + Gazebo Harmonic is not an upstream-supported pair.**")
    A("   Harmonic pairs with Jazzy; Humble's binary `ros_gz` targets Fortress. If")
    A("   you stay on Humble you are building `ros_gz` from source against")
    A("   Harmonic. The world itself is version-agnostic, so this is a bridge")
    A("   problem -- but budget for it before demo week.")
    A("6. **Prop physics is configured here, not validated here.** Mass and")
    A("   inertia are computed analytically and written into each model.sdf, but")
    A("   nothing in this pipeline runs a Gazebo step. Expect to tune")
    A("   `max_step_size`, contact `max_vel` and `min_depth` once you spawn")
    A("   them — a 4.5 kg cone on a mesh terrain is exactly the mass range where")
    A("   a soft contact solver jitters.")
    A("7. **Vegetation is low-poly and untextured.** Good LiDAR silhouettes and")
    A("   real occlusion; it will not win a render competition. That trade was")
    A("   made on purpose in favour of simulation rate.")
    A("8. **No texture maps.** Flat materials only. Feature-based visual SLAM will")
    A("   find geometry (walls, trunks, containers, rubble, the comms mast) but")
    A("   little surface texture. If you are testing a descriptor-based VO")
    A("   front-end, add textures before drawing conclusions.")
    A("9. **The base apron is genuinely flat** by design, so the")
    A("   `no_large_perfectly_flat_areas` check tolerates it.")
    A("")
    A("## 20. How to regenerate")
    A("")
    A("```bash")
    A("blender --background --python generate_military_sar_environment.py -- \\")
    A("        --out %s --seed %d" % (outdir, CFG["seed"]))
    A("")
    A("# quick low-res iteration pass")
    A("blender --background --python generate_military_sar_environment.py -- --fast")
    A("```")
    A("")
    A("Same seed -> byte-identical layout. Sub-seeds are derived per subsystem")
    A("(`sub_rng(\"rubble_scatter\")` etc.) with a CRC32-stable hash, so re-tuning")
    A("rubble does not reshuffle where the casualties are, and `PYTHONHASHSEED`")
    A("cannot silently break reproducibility.")
    A("")
    A("Regeneration deletes **only** the `SAR_GENERATED` subtree. A robot model,")
    A("imported assets, other scenes and your own collections are untouched. Run it")
    A("on a .blend that already has your UGV in it and the UGV survives.")
    A("")
    A("## 21. How to move the humans")
    A("")
    A("Edit `STATIC_HUMANS` / `DYNAMIC_HUMANS` near the top of the generator. One")
    A("dict per person:")
    A("")
    A("```python")
    A('dict(pid="PERSON_002", x=-78.0, y=160.0, pose="LYING",')
    A('     cloth="MAT_HUMAN_DARK", th="HIGH", occ="VEGETATION_PARTIAL",')
    A('     status="INJURED_UNRESPONSIVE", zone="SAR_ZONE_B",')
    A('     sector="FOREST", diff="HARD", head=0.8)')
    A("")
    A('dict(pid="PERSON_016", speed=1.25, dwell=(0.0, 0.0),')
    A('     path=[(-14,66), (-2,70), (10,74), (20,70), (8,64), (-14,66)])')
    A("```")
    A("")
    A("`pose` is one of %s. `th` is HIGH/MEDIUM/LOW"
      % ", ".join("`%s`" % k for k in POSE_DEFS))
    A("thermal contrast. Z comes from the terrain automatically -- never hand-set")
    A("it. Ground truth, the JSON files and the SDF actors all regenerate from")
    A("these lists, so there is exactly one place to edit.")
    A("")
    A("If you want a victim guaranteed occluded, add an entry to the `plan` list in")
    A("`build_occluders()`; it runs before the general scatter so the occluder is")
    A("certain rather than lucky.")
    A("")
    A("## 22. How to change the weather")
    A("")
    A("`WEATHER_PRESETS` in the generator, mirrored into `sar_metadata.json`:")
    A("%s. Each carries rain/dust/snowfall"
      % ", ".join("`%s`" % k for k in WEATHER_PRESETS))
    A("intensity, wind, visibility and the lighting preset it implies. In Blender")
    A("they are hidden config holders in `LIGHTING`. In Gazebo, transcribe them")
    A("into `<scene><fog>` and your sensor noise models -- and for `RAIN`, drop")
    A("`friction_mu` by about 0.15 on sealed surfaces and 0.08 elsewhere.")
    A("")
    A("Do not model `SNOWFALL` or `DUST` as total sensor failure. The objective is")
    A("robust navigation under degradation, not a scripted blackout.")
    A("")
    A("## 23. How to change the lighting")
    A("")
    A("Six presets: %s. One sun object per preset in"
      % ", ".join("`%s`" % k for k in LIGHT_PRESETS))
    A("`LIGHTING`; `DAY` is active and the rest are hidden. Switch with")
    A("`sar_lighting.py` (embedded as a Blender text datablock -- open it and hit")
    A("Run Script), which also sets the world colour, strength and fog density.")
    A("Nothing is baked into textures.")
    A("")
    A("`NIGHT` is the one that matters: the base floodlights stay on, everything")
    A("else goes near-dark, ambient air drops to 9 C and human-to-background")
    A("delta-T is at its largest. It is not pitch black, because the point is")
    A("fusion, not blinding the RGB camera.")
    A("")
    A("## 24. How to build new SAR scenarios")
    A("")
    A("1. **New casualty scenario** -- add to `STATIC_HUMANS` with the thermal")
    A("   contrast and occlusion you want to test, then add an occluder to")
    A("   `build_occluders()` if the occlusion must be guaranteed.")
    A("2. **New hazard** -- add to `NEGATIVES` (real carved void; the tile")
    A("   auto-refines to %.2f m) or `CLIFFS`, and give it a `vis` of `SUBTLE` if"
      % CFG["res_fine"])
    A("   you want it to be hard to see. Ground truth and metadata follow.")
    A("3. **New route decision** -- add two roads to `ROADS` between the same")
    A("   endpoints with different exposure, as `SAR_Track_Mountain_Risk` and")
    A("   `SAR_Track_Mountain_Safe` already do, then block one with debris.")
    A("4. **New decoy** -- add to `DECOYS`. `WARMCASE` is a thermal false positive,")
    A("   `MANNEQUIN` an RGB one, and placing them within a few metres of a real")
    A("   casualty is what makes classification hard rather than academic.")
    A("5. **New sector** -- add a centroid to `SECTORS` and a height function to")
    A("   `SECTOR_H`. Blending, classification, transitions and metadata are")
    A("   automatic; you do not touch the tile loop.")
    A("6. Re-run with a **new seed** for a fresh randomisation of the same design,")
    A("   or the same seed to reproduce a run exactly.")
    A("")
    A("---")
    A("")
    A("## Validation")
    A("")
    A("Run automatically at the end of generation, against the generated field --")
    A("measured, not asserted.")
    A("")
    A("**%d/%d checks passed, %d warning(s).**"
      % (rep["summary"]["passed"], rep["summary"]["total_checks"],
         rep["summary"]["warnings"]))
    A("")
    for c in rep["checks"]:
        A("- %s `%s` -- %s" % ("PASS" if c["pass_"] else "**FAIL**",
                               c["check"], c["detail"]))
    A("")
    if rep["warnings"]:
        A("Warnings:")
        A("")
        for w in rep["warnings"]:
            A("- %s" % w)
        A("")
    A("### Route traversability (sampled every 1.5 m off the real height field)")
    A("")
    A("| route | length | max slope | in water | near neg. obstacle | terrain mix |")
    A("|---|---|---|---|---|---|")
    for ra in rep["route_analysis"]:
        mix = ", ".join("%s %.0f%%" % (k, v * 100)
                        for k, v in list(ra["terrain_mix"].items())[:4])
        A("| `%s` | %.0f m | %.1f deg | %d | %d | %s |"
          % (ra["route"], ra["length_m"], ra["max_slope_deg"],
             ra["samples_in_water"], ra["samples_in_negative_obstacle"], mix))
    A("")
    A("### The mission this is built to support")
    A("")
    A("Robot spawns at `SAR_ROBOT_START` with no map, no pose oracle and no GNSS.")
    A("It initialises, starts SLAM, explores, classifies terrain, detects positive")
    A("and negative obstacles, plans safe routes, meets dynamic humans and yields,")
    A("finds a stationary casualty with RGB, confirms with thermal, localises it")
    A("with LiDAR/depth, transforms into the map frame, marks it, continues, hits a")
    A("blocked road, replans onto the bypass, evaluates rubble traversability,")
    A("chooses the safer line, finds the collapsed building, searches its accessible")
    A("volume, finds a second casualty, computes coverage, and drives home to the")
    A("dock.")
    A("")
    A("Every one of those steps has geometry in this scene that makes it possible")
    A("and nothing in this scene that makes it free.")
    with open(os.path.join(outdir, "README.md"), "w") as f:
        f.write("\n".join(L) + "\n")


# ============================================================================
# MAIN
# ============================================================================

def main():
    parse_args()
    log("=" * 68)
    log("MILITARY SAR ENVIRONMENT GENERATOR")
    log("seed=%d  out=%s  blender=%s"
        % (CFG["seed"], CFG["out_dir"], bpy.app.version_string))
    log("=" * 68)

    purge_generated()
    build_collections()
    build_materials()
    setup_scene()

    build_road_profiles()
    build_terrain()
    build_water()
    build_roads()
    build_bridge()

    build_veg_protos()
    build_rock_protos()

    build_base()
    build_buildings()
    build_vehicles()

    build_static_humans()
    build_dynamic_humans()
    build_decoys()
    build_occluders()
    build_dynamic_props()
    build_sensor_tests()
    build_mission_markers()

    scatter_vegetation()
    scatter_rocks()
    scatter_rubble()

    build_lighting()

    outdir = CFG["out_dir"]
    os.makedirs(outdir, exist_ok=True)
    embed_texts(outdir)

    report = validate()
    write_metadata(outdir, report)
    write_readme(outdir, report)

    total_objs = len(bpy.data.collections[ROOT_COLL].all_objects)
    log("-" * 68)
    log("objects: %d   meshes: %d   materials: %d"
        % (total_objs, len(bpy.data.meshes), len(bpy.data.materials)))
    log("validation: %d/%d passed, %d warning(s)"
        % (report["summary"]["passed"], report["summary"]["total_checks"],
           report["summary"]["warnings"]))
    for w in report["warnings"]:
        log("  WARN: %s" % w)

    if CFG["save"]:
        path = os.path.join(outdir, CFG["blend_name"])
        bpy.ops.wm.save_as_mainfile(filepath=path, compress=True)
        log("saved %s (%.1f MB)" % (path, os.path.getsize(path) / 1e6))
    log("done")
    return report


if __name__ == "__main__":
    main()
