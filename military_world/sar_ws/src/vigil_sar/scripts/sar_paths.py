#!/usr/bin/python3
"""Locate the military_world project and read the single SAR configuration file."""
import os
from pathlib import Path

import yaml

MARKER = Path('gazebo_export') / 'military_world.sdf'


def find_military_world(configured=''):
    """configured path -> $MILITARY_WORLD_DIR -> a parent of this file (the workspace
    lives in military_world/sar_ws) -> ~/Documents/military_world."""
    cands = []
    if configured:
        cands.append(Path(os.path.expanduser(configured)))
    if os.environ.get('MILITARY_WORLD_DIR'):
        cands.append(Path(os.path.expanduser(os.environ['MILITARY_WORLD_DIR'])))
    here = Path(os.path.realpath(__file__))
    cands += list(here.parents)
    cands.append(Path.home() / 'Documents' / 'military_world')
    for c in cands:
        if (c / MARKER).exists():
            return c.resolve()
    raise FileNotFoundError('military_world not found: set world.military_world_dir in '
                            'config/sar_mission.yaml or export MILITARY_WORLD_DIR')


def load_config(path):
    """Plain dict of the '/**: ros__parameters:' block of sar_mission.yaml."""
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['/**']['ros__parameters']
