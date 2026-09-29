#!/usr/bin/python3
"""Verify installed dependencies; return failure for every incomplete setup."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError, get_package_prefix, get_package_share_directory,
)


def main():
    failures = []
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        failures.append('Source /opt/ros/jazzy/setup.bash first')
    manifest = Path(get_package_share_directory('agri_ugv_setup')) / 'package.xml'
    root = ET.parse(manifest).getroot()
    names = sorted({e.text for e in root if e.tag in ('depend', 'exec_depend', 'buildtool_depend')})
    for name in names:
        try:
            print(f'OK package {name}: {get_package_prefix(name)}')
        except PackageNotFoundError:
            failures.append(f'Missing ROS package: {name}')
    for name in ('colcon', 'rosdep', 'cmake', 'c++', 'check_urdf', 'gz'):
        if not shutil.which(name):
            failures.append(f'Missing executable: {name}')
    if shutil.which('gz'):
        result = subprocess.run(['gz', 'sim', '--versions'], text=True, capture_output=True, timeout=20)
        versions = result.stdout.strip()
        print(f'Gazebo Sim versions: {versions}')
        if result.returncode or not any(v.strip().startswith('8.') for v in versions.splitlines()):
            failures.append('Gazebo Harmonic (Sim 8) not detected')
    for failure in failures:
        print(f'FAIL: {failure}', file=sys.stderr)
    print('PASS: environment dependencies ready' if not failures else 'FAIL: environment incomplete')
    return int(bool(failures))


if __name__ == '__main__':
    sys.exit(main())
