#!/usr/bin/env python3
"""VIGIL master launcher - a menu that starts ONE of three independent ROS 2 / Gazebo projects.

    vigil                                             (after  bash ~/Documents/vigil/setup_vigil.sh)
    python3 ~/Documents/vigil/master_launcher/vigil_launcher.py

It contains no robot, world, navigation or dashboard code. For the chosen project it only:
  1. starts a clean shell (ROS variables from other workspaces removed from the environment),
  2. sources /opt/ros/jazzy/setup.bash and THAT project's install/setup.bash (never the others),
  3. runs that project's own, unchanged launch file from its own workspace,
  4. gives it the terminal (Ctrl+C goes to the project), and when it ends stops anything it left
     behind and shows the menu again.
Before a start, processes still running from any of the three projects are stopped, so only one
environment is ever active.
"""
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

HOME = Path.home()
VIGIL = Path(__file__).resolve().parent.parent          # ~/Documents/vigil

# Where each project lives and which of ITS OWN launch files starts it. The first existing path wins:
# the shortcut links in vigil/ (setup_vigil.sh), then the folder next to vigil/ (a clone of the
# GitHub repository), then the original location on the author's PC.
REPO = VIGIL.parent                                     # the repository root when cloned from GitHub
# ROS_DOMAIN_ID and environment are the values each project documents for itself (run.sh / README).
PROJECTS = {
    '1': dict(name='Military Search and Rescue', short='Military SAR',
              roots=[VIGIL / 'military_sar/military_world/sar_ws', REPO / 'military_world/sar_ws', HOME / 'Documents/military_world/sar_ws'],
              package='vigil_sar', launch='sar_mission.launch.py', domain='72', env={},
              marks=[r'GZ_PARTITION=vigil_sar$'], libs=['lib/vigil_sar/']),
    '2': dict(name='Agriculture', short='Agriculture',
              roots=[VIGIL / 'agriculture/ros2_ws', REPO / 'ros2_ws', HOME / 'Documents/robot/ros2_ws'],
              package='agri_ugv', launch='field.launch.py', domain='91',
              env={'GZ_PARTITION': 'agri_ugv', 'ROS_LOG_DIR': '{ws}/log/runtime'},
              lock='run/simulation.lock',                      # the same guard run.sh / stop.sh use
              marks=[r'GZ_PARTITION=agri_ugv$'], libs=['lib/agri_ugv/', 'lib/agri_ugv_setup/']),
    '3': dict(name='Rock Terrain', short='Rock Terrain',
              roots=[VIGIL / 'rock_terrain/vigil_rough_terrain_ws', REPO / 'vigil_rough_terrain_ws', HOME / 'Documents/robot/vigil_rough_terrain_ws',
                     HOME / 'Documents/robot/ros2_ws/vigil_rough_terrain_ws'],
              package='vigil_rough_terrain', launch='vision_nav.launch.py', domain='71', env={},
              marks=[r'GZ_PARTITION=vigil_\d+$'], libs=['lib/vigil_rough_terrain/']),
}

# Variables a sourced ROS workspace sets. They are removed before sourcing, so a workspace sourced in
# ~/.bashrc (or by a previous project) can never leak into the selected one.
ROS_VARS = ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'ROS_PACKAGE_PATH', 'ROS_DISTRO',
            'ROS_VERSION', 'ROS_PYTHON_VERSION', 'ROS_DOMAIN_ID', 'GZ_PARTITION', 'GZ_SIM_RESOURCE_PATH',
            'GZ_SIM_SYSTEM_PLUGIN_PATH', 'GZ_SIM_PLUGIN_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'ROS_LOG_DIR',
            'VIGIL_LAUNCHER_PROJECT', 'VIGIL_LAUNCHER_SESSION')
PATH_VARS = ('PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'PKG_CONFIG_PATH')

BANNER = """
=============================================
          VIGIL SIMULATION LAUNCHER
=============================================

Select Environment:

1. Military Search and Rescue
2. Agriculture
3. Rock Terrain
4. Exit
"""


# ------------------------------------------------------------------------ project lookup
def ros_setup():
    for distro in (os.environ.get('ROS_DISTRO', ''), 'jazzy', 'humble', 'rolling'):
        p = Path('/opt/ros') / distro / 'setup.bash'
        if distro and p.exists():
            return p
    found = sorted(Path('/opt/ros').glob('*/setup.bash')) if Path('/opt/ros').exists() else []
    return found[0] if found else None


def locate(p):
    """(workspace, launch file, error). The launch file must exist in the project's source tree AND
    in its install tree (ros2 launch runs the installed copy)."""
    for root in p['roots']:
        ws = Path(root).expanduser()
        src = ws / 'src' / p['package'] / 'launch' / p['launch']
        if src.exists():
            ws = ws.resolve()
            inst = ws / 'install' / p['package'] / 'share' / p['package'] / 'launch' / p['launch']
            if not (ws / 'install/setup.bash').exists() or not inst.exists():
                return ws, src, (f"{p['short']} workspace is not built ({inst} missing).\n"
                                 f"Build it once:  cd {ws} && source /opt/ros/jazzy/setup.bash && colcon build "
                                 f"--symlink-install")
            return ws, src.resolve(), None
    return None, None, f"{p['short']} launch file not found."


# ------------------------------------------------------------------------ processes
SHELLS = {'bash', '-bash', 'sh', 'zsh', '-zsh', 'fish', '/bin/bash', '/usr/bin/bash', '/bin/sh', '/usr/bin/zsh'}


def ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return out


def own_processes():
    """[(pid, environ bytes, cmdline str)] of this user's processes, except this launcher, the
    shells it was started from and any plain interactive shell (a terminal where someone exported
    GZ_PARTITION by hand must never be killed)."""
    skip, uid, out = ancestors(), os.getuid(), []
    for d in os.listdir('/proc'):
        if not d.isdigit() or int(d) in skip:
            continue
        try:
            if os.stat(f'/proc/{d}').st_uid != uid:
                continue
            with open(f'/proc/{d}/environ', 'rb') as f:
                env = f.read()
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='ignore').strip()
            if cmd in SHELLS:
                continue
            out.append((int(d), env, cmd))
        except OSError:
            continue
    return out


def project_of(env, cmd):
    """Which of the three projects a process belongs to, or None."""
    for key, p in PROJECTS.items():
        items = env.split(b'\0')
        if f'VIGIL_LAUNCHER_PROJECT={key}'.encode() in items:
            return key
        for mark in p['marks']:
            if any(re.match(mark, e.decode(errors='ignore')) for e in items if e.startswith(b'GZ_PARTITION=')):
                return key
        if any(lib in cmd for lib in p['libs']):
            return key
    return None


def stop(pids, grace=8.0):
    """SIGINT (clean ROS shutdown), then SIGTERM, then SIGKILL."""
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
        alive = [p for p in pids if Path(f'/proc/{p}').exists()]
        if not alive:
            return
        for p in alive:
            try:
                os.kill(p, sig)
            except OSError:
                pass
        end = time.monotonic() + wait
        while time.monotonic() < end and any(Path(f'/proc/{p}').exists() for p in alive):
            time.sleep(0.2)


def stop_running_projects():
    """Only one environment at a time: stop whatever is still running from any of the three."""
    found = {}
    for pid, env, cmd in own_processes():
        key = project_of(env, cmd)
        if key:
            found.setdefault(key, []).append(pid)
    for key, pids in found.items():
        print(f"Stopping {PROJECTS[key]['name']} processes still running ({len(pids)})...")
        stop(pids)


def stop_session(session):
    left = [pid for pid, env, _ in own_processes() if f'VIGIL_LAUNCHER_SESSION={session}'.encode() in env.split(b'\0')]
    if left:
        print(f'Stopping {len(left)} process(es) left behind...')
        stop(left, grace=5.0)


# ------------------------------------------------------------------------ launch
def clean_env(key, p, ws, session):
    env = {k: v for k, v in os.environ.items() if k not in ROS_VARS and not k.startswith('AMENT_')}
    for var in PATH_VARS:
        if var in env:
            keep = [e for e in env[var].split(':') if e and not e.startswith('/opt/ros/')
                    and '/install/' not in e + '/' and not e.startswith(str(ws))]
            env[var] = ':'.join(keep)
    env['ROS_DOMAIN_ID'] = p['domain']
    for k, v in p['env'].items():
        env[k] = v.format(ws=ws)
    env['VIGIL_LAUNCHER_PROJECT'] = key            # marks every process of this run (inherited)
    env['VIGIL_LAUNCHER_SESSION'] = session
    return env


def script(p, ws, setup):
    lines = ['set -e',
             f'source "{setup}"',
             f'source "{ws}/install/setup.bash"',
             f'cd "{ws}"']
    if p.get('lock'):                               # same duplicate-run guard as the project's run.sh
        lines += ['mkdir -p run log', f'exec 9>"{p["lock"]}"',
                  'flock -w 2 9 || { echo "This workspace already has a simulation running." >&2; exit 1; }',
                  'echo "$$" > run/launch.pid']
    lines += [f'echo "ROS {os.path.basename(os.path.dirname(setup))} | workspace {ws} | ROS_DOMAIN_ID=$ROS_DOMAIN_ID"',
              f'exec ros2 launch {p["package"]} {p["launch"]}']
    return '\n'.join(lines)


def run(key):
    p = PROJECTS[key]
    ws, launch, err = locate(p)
    if err:
        print(f'\nERROR:\n{err}')
        return
    setup = ros_setup()
    if setup is None:
        print('\nERROR:\nROS 2 Jazzy not found (/opt/ros/jazzy/setup.bash).')
        return
    stop_running_projects()
    print(f"\nStarting {p['name']}...")
    print(f'Launch file: {launch}')
    print('Press Ctrl+C to stop it and return to this menu.\n')
    session = uuid.uuid4().hex
    tty = sys.stdin.isatty()
    fd = sys.stdin.fileno() if tty else None

    def child_setup():                              # own process group, and it owns the terminal
        os.setpgid(0, 0)
        if tty:
            signal.signal(signal.SIGTTOU, signal.SIG_IGN)
            try:
                os.tcsetpgrp(fd, os.getpgrp())
            except OSError:
                pass
            signal.signal(signal.SIGTTOU, signal.SIG_DFL)
    proc = subprocess.Popen(['bash', '-c', script(p, ws, setup)], env=clean_env(key, p, ws, session),
                            cwd=str(ws), preexec_fn=child_setup)
    old_int = signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C belongs to the project now
    old_ttou = signal.signal(signal.SIGTTOU, signal.SIG_IGN)
    try:
        if tty:
            try:
                os.tcsetpgrp(fd, proc.pid)
            except OSError:
                pass
        rc = proc.wait()
    finally:
        if tty:
            try:
                os.tcsetpgrp(fd, os.getpgrp())      # take the terminal back
            except OSError:
                pass
        signal.signal(signal.SIGTTOU, old_ttou)
        signal.signal(signal.SIGINT, old_int)
    stop_session(session)
    print(f"\n{p['name']} stopped (exit code {rc}). Returning to the menu.")


def main():
    while True:
        print(BANNER)
        try:
            choice = input('Select option [1-4]: ').strip()
        except (EOFError, KeyboardInterrupt):
            print('\nExiting...')
            return 0
        if choice == '4':
            print('Exiting...')
            return 0
        if choice in PROJECTS:
            run(choice)
        else:
            print(f'\nInvalid option "{choice}". Enter 1, 2, 3 or 4.')


if __name__ == '__main__':
    sys.exit(main())
