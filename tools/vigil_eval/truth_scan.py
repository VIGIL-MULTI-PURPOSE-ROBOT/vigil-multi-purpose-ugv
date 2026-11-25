#!/usr/bin/env python3
"""Static scan: which Python files can read simulator truth (source-level half of the guard).

Finds references to the truth channels of the Phase 1 audit in every tracked .py file of the four
environments and classifies each file:
  evaluation   allowed: reads truth only to measure (regression monitor, tests, tools, this package)
  simulation   allowed: simulates the world or an environment sensor (listed in ALLOWED_SIMULATION)
  operational  NOT allowed in GPS-denied mode: steers, maps, decides or displays from truth

  python3 tools/vigil_eval/truth_scan.py [--json out.json] [--strict]
--strict exits 1 when any operational file is found (use once the migration is complete).
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

PATTERNS = {
    'ground_truth': re.compile(r"['\"]/?sim/ground_truth['\"]"),
    'link_poses': re.compile(r"['\"]/?sim/link_poses['\"]"),
    'segmentation_oracle': re.compile(r"['\"]/?camera/segmentation/(labels_map|colored_map)['\"]"),
    'contact_positions': re.compile(r"['\"]/?suspension/contacts/"),
    'gz_transport': re.compile(r'^\s*(from|import)\s+gz\.transport', re.M),
    'world_file_read': re.compile(r'ET\.parse\(|rows_from_world\('),
}
ENVS = ('Rock Terrain', 'Agriculture', 'Military Search and Rescue', 'New Mining')
EVAL_MARKERS = ('/test/', '/tests/', '/tools/', 'test_', 'motion_test.py', 'check_', '_eval.py',
                'find_crash.py', 'scale_check.py', 'rover_stability.py', 'build_sar_world.py',
                'scenario_worlds.py', 'sim_speed.py', 'make_test_worlds.py', 'sar_physics.py')
# Files that read truth to SIMULATE the world or an environment sensor (Phase 1 audit, 3.2).
ALLOWED_SIMULATION = {
    'Agriculture/src/agri_ugv/scripts/agri_moisture_sensor.py': 'simulated soil-moisture field',
    'Agriculture/src/agri_ugv/scripts/environment.py': 'worker motion and lighting zone (world simulation)',
    'New Mining/src/vigil_new_mining/scripts/mining_runtime.py': 'gas/temperature/humidity field (sensor simulation); '
                                                                 'NOTE: same file also navigates - to be split',
    'Agriculture/src/agri_ugv/scripts/agri_mission_core.py': 'reads the field model (prior map) - see audit 3.2',
}


def tracked_py(root):
    out = subprocess.run(['git', '-C', str(root), 'ls-files', '*.py'], capture_output=True, text=True, check=True)
    return [p for p in out.stdout.splitlines() if p.startswith(ENVS)]


def classify(path):
    if path in ALLOWED_SIMULATION:
        return 'simulation'
    if any(m in path for m in EVAL_MARKERS) or Path(path).name.startswith(('test_', 'check_')):
        return 'evaluation'
    return 'operational'


def scan(root):
    rows = []
    for rel in tracked_py(root):
        text = (Path(root) / rel).read_text(errors='ignore')
        hits = {}
        for k, rx in PATTERNS.items():
            lines = [i + 1 for i, ln in enumerate(text.splitlines()) if rx.search(ln)]
            if k == 'gz_transport' and rx.search(text):
                lines = lines or [1]
            if lines:
                hits[k] = lines
        if hits:
            rows.append(dict(file=rel, category=classify(rel), hits=hits,
                             note=ALLOWED_SIMULATION.get(rel, '')))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=str(Path(__file__).resolve().parents[2]))
    ap.add_argument('--json', default='')
    ap.add_argument('--strict', action='store_true')
    a = ap.parse_args()
    rows = scan(a.root)
    for cat in ('operational', 'simulation', 'evaluation'):
        sel = [r for r in rows if r['category'] == cat]
        print(f'\n{cat.upper()} ({len(sel)} files)')
        for r in sel:
            h = '; '.join(f"{k} l.{','.join(map(str, v[:4]))}" for k, v in r['hits'].items())
            print(f"  {r['file']}: {h}{'  [' + r['note'] + ']' if r['note'] else ''}")
    if a.json:
        Path(a.json).write_text(json.dumps(rows, indent=1))
    n_op = sum(r['category'] == 'operational' for r in rows)
    print(f'\noperational files with a truth channel: {n_op}')
    sys.exit(1 if (a.strict and n_op) else 0)


if __name__ == '__main__':
    main()
