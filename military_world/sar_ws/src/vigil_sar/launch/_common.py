"""Shared helpers for the vigil_sar launch files."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

PKG = 'vigil_sar'


def params(context):
    """[config file, overrides] for every node (config/sar_mission.yaml is the only source)."""
    share = Path(get_package_share_directory(PKG))
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    cfg = lc('config') or str(share / 'config/sar_mission.yaml')
    over = {}
    for arg, key in (('goal_x', 'navigation.goal_x'), ('goal_y', 'navigation.goal_y')):
        try:
            v = lc(arg)
        except Exception:
            v = ''
        if v:
            over[key] = float(v)
    return [cfg, over] if over else [cfg]
