#!/usr/bin/env python3
"""find_crash.py - find WHAT makes Gazebo crash on this PC, then write a render profile that works.

Started by sar_ws/find_crash.sh (which builds the workspace and sources ROS first).

Why this and not the old rig tests: gz-sensors only renders a camera that has a subscriber, so a
camera sitting in a world with nothing listening never renders and never crashes. Every trial here
is therefore the REAL launch (rover + ros_gz bridge subscribing to every camera), just headless and
short, with one sensor set at a time. Each trial ends in one of:

    OK      the server ran the whole trial and the enabled sensors published frames
    CRASH   the gz server died on a signal (the bug we are chasing)
    ERROR   something else (no rover, launch error) - printed with the last log lines

Phases
  1  all sensors, server under gdb  -> the crash backtrace (needs gdb; skipped if not installed)
  2  no rendering sensor at all     -> separates physics/robot crashes from rendering crashes
  3  one sensor at a time           -> names the sensor that cannot render this world here
  4  all sensors that passed alone  -> the profile the mission will use
  5  world-content bisect (only if a single sensor crashed) -> names the model group that triggers it

Result: sar_ws/generated/render_profile.yaml (used automatically by every vigil_sar launch) and
sar_ws/diagnosis/find_crash.log (the file to send back).
"""
import argparse
import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

SENSORS = ('rgbd', 'lidar', 'segmentation', 'hd_camera', 'thermal')
TOPICS = {'rgbd': ['/camera/image', '/camera/depth_image'],
          'lidar': ['/lidar/points'],
          'segmentation': ['/camera/segmentation/labels_map'],
          'hd_camera': ['/camera/hd/image'],
          'thermal': ['/thermal/image_raw']}
# model-name prefixes, for the world-content bisect
GROUPS = {'ground': ('SAR_Terrain', 'SAR_Track', 'SAR_Road', 'SAR_Street', 'SAR_Bridge', 'SAR_Water'),
          'buildings': ('SAR_Building',),
          'scatter': ('SAR_Batch',),
          'props': ('SAR_DynProps', 'SAR_Occluder'),
          'vehicles': ('SAR_Vehicle',),
          'base': ('SAR_Base',),
          'people': ('SAR_DynamicPerson', 'SAR_StaticPerson', 'SAR_BuildingPerson', 'SAR_CarriedStretcher'),
          'decoys': ('SAR_Decoy', 'SAR_SensorTest')}
BISECT_GROUPS = [g for g in GROUPS if g != 'ground']

LOG = None


def say(*msg):
    text = ' '.join(str(m) for m in msg)
    print(text, flush=True)
    if LOG:
        LOG.write(text + '\n')
        LOG.flush()


def sh(cmd, timeout=30):
    try:
        r = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True, timeout=timeout, text=True)
        return (r.stdout + r.stderr).strip()
    except (subprocess.TimeoutExpired, OSError) as exc:
        return f'({exc.__class__.__name__})'


class Trial:
    """One headless run of sim.launch.py."""

    def __init__(self, ws, out, seconds, verbose=False):
        self.ws, self.out, self.seconds, self.verbose = ws, out, seconds, verbose

    def stop_leftovers(self):
        script = self.ws / 'stop_sar.sh'
        if script.exists():
            sh(['bash', str(script)], timeout=60)
        time.sleep(1.0)

    def run(self, name, sensors, extra=(), debug=False, seconds=None):
        seconds = seconds or self.seconds
        log_path = self.out / f'{name}.log'
        args = [f'{s}:=' + ('true' if sensors.get(s) else 'false') for s in SENSORS]
        cmd = ['ros2', 'launch', 'vigil_sar', 'sim.launch.py', 'gui:=false', 'profile:=none',
               'cleanup:=true'] + args + list(extra) + (['debug:=true'] if debug else [])
        env = dict(os.environ, GZ_PARTITION='vigil_sar', PYTHONUNBUFFERED='1')
        self.stop_leftovers()
        started = time.time()
        with open(log_path, 'wb') as handle:
            proc = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT, env=env,
                                    start_new_session=True)
            spawned_at = self._wait_for_spawn(proc, log_path, started)
            rendered = {}
            if spawned_at and proc.poll() is None:
                rendered = self._sample(sensors, env)
            while proc.poll() is None and time.time() - started < seconds:
                time.sleep(0.5)
            alive = proc.poll() is None
            self._stop(proc)
        text = log_path.read_text(errors='ignore')
        result = dict(name=name, sensors={s: bool(sensors.get(s)) for s in SENSORS}, extra=list(extra),
                      log=str(log_path), rendered=rendered, spawned=bool(spawned_at),
                      seconds=round(time.time() - started, 1))
        crash = re.search(r'Gazebo server CRASHED \(signal (\d+)\)', text)
        gdb_crash = re.search(r'Program received signal (SIG\w+)', text)
        if crash or gdb_crash:
            result['status'] = 'CRASH'
            result['signal'] = crash.group(1) if crash else gdb_crash.group(1)
            if spawned_at:
                result['after_spawn'] = round(self._crash_time(text) - spawned_at, 1) if self._crash_time(text) else None
        elif not spawned_at:
            result['status'] = 'ERROR'
            result['why'] = 'the rover never spawned'
        elif alive:
            result['status'] = 'OK'
        else:
            result['status'] = 'ERROR'
            result['why'] = 'the launch stopped early'
        if result['status'] != 'OK':
            result['tail'] = [ln for ln in text.splitlines()[-25:] if ln.strip()]
        self.stop_leftovers()
        return result

    def _wait_for_spawn(self, proc, log_path, started, limit=90):
        """The rover is in the world once ros_gz_sim create finishes."""
        while time.time() - started < limit:
            if proc.poll() is not None:
                return None
            text = log_path.read_text(errors='ignore')
            if 'creation successful' in text.lower() or re.search(r'\[create-\d+\]: process has finished cleanly', text):
                return time.time()
            if 'Rover spawn failed' in text:
                return None
            time.sleep(0.5)
        return None

    def _sample(self, sensors, env):
        """Proof that the sensor really rendered: one message on its Gazebo topic."""
        out = {}
        for sensor in SENSORS:
            if not sensors.get(sensor):
                continue
            for topic in TOPICS[sensor]:
                got = sh(f'gz topic -e -n 1 -t {topic} 2>/dev/null | head -c 120', timeout=35)
                out[topic] = bool(got) and '(' not in got[:1]
        return out

    @staticmethod
    def _crash_time(text):
        match = re.search(r'^(\d+\.\d+) \[ERROR\].*process has died', text, re.M)
        return float(match.group(1)) if match else None

    @staticmethod
    def _stop(proc):
        if proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT)
            except OSError:
                pass
            for _ in range(60):
                if proc.poll() is not None:
                    break
                time.sleep(0.5)
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except OSError:
                pass
            proc.wait(timeout=10)


def report(result):
    marks = ' '.join(('+' if ok else '-') + topic for topic, ok in result.get('rendered', {}).items())
    line = f"    {result['status']:5s}  {result['name']}"
    if result['status'] == 'CRASH':
        line += f"  signal {result.get('signal')}"
        if result.get('after_spawn'):
            line += f", {result['after_spawn']} s after the rover appeared"
    elif result['status'] == 'OK':
        line += f"  {result['seconds']} s, frames: {marks or 'no camera enabled'}"
    else:
        line += f"  ({result.get('why', '?')})"
    say(line)
    if result['status'] == 'ERROR':
        for row in result.get('tail', [])[-8:]:
            say('        |', row[:160])


def world_variant(source, keep, dest):
    """Copy the generated world keeping only <model>s of the named groups (ground is always kept)."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source, parser=parser)
    world = tree.getroot().find('world')
    prefixes = tuple(p for group in ('ground',) + tuple(keep) for p in GROUPS[group])
    removed = 0
    for model in list(world.findall('model')):
        name = model.get('name', '')
        if not name.startswith(prefixes):
            world.remove(model)
            removed += 1
    tree.write(dest, encoding='utf-8', xml_declaration=True)
    return removed


def environment(ws):
    say('=== this PC ===')
    say('  date        :', datetime.datetime.now().isoformat(timespec='seconds'))
    say('  gz          :', sh('gz sim --versions', 10).splitlines()[0] if sh('gz sim --versions', 10) else '?')
    say('  ROS         :', os.environ.get('ROS_DISTRO', '?'))
    say('  session     :', os.environ.get('XDG_SESSION_TYPE', '?'), 'QT_QPA_PLATFORM=' + os.environ.get('QT_QPA_PLATFORM', 'unset'))
    say('  GPUs        :', '; '.join(ln.split(': ', 1)[-1] for ln in sh("lspci | grep -iE 'vga|3d controller'", 10).splitlines()))
    nvidia = sh('nvidia-smi --query-gpu=name,driver_version --format=csv,noheader', 15)
    say('  nvidia      :', nvidia if nvidia and 'not found' not in nvidia.lower() else 'no nvidia-smi')
    say('  GL renderer :', sh("glxinfo -B 2>/dev/null | grep 'OpenGL renderer'", 15) or 'glxinfo missing (sudo apt install mesa-utils)')
    say('  gdb         :', shutil.which('gdb') or 'not installed (sudo apt install gdb) - no backtrace')
    say('  free RAM MB :', sh("free -m | awk '/^Mem:/{print $7}'", 10))
    say('  workspace   :', ws)
    say('')


def main():
    global LOG
    ap = argparse.ArgumentParser()
    ap.add_argument('--ws', default='', help='the sar_ws folder')
    ap.add_argument('--seconds', type=int, default=55, help='length of one trial (default 55 s)')
    ap.add_argument('--skip-gdb', action='store_true')
    ap.add_argument('--skip-bisect', action='store_true')
    args = ap.parse_args()
    ws = Path(args.ws or Path(__file__).resolve().parents[3])
    out = ws / 'diagnosis'
    out.mkdir(parents=True, exist_ok=True)
    generated = ws / 'generated'
    generated.mkdir(parents=True, exist_ok=True)
    LOG = open(out / 'find_crash.log', 'w')
    say('=== vigil_sar find_crash: what can this PC render? ===')
    say(f'    every trial = the real launch, headless, {args.seconds} s, cameras actually rendering')
    say(f'    full log: {out / "find_crash.log"}')
    say('')
    environment(ws)
    trial = Trial(ws, out, args.seconds)
    results = []
    all_on = {s: True for s in SENSORS}

    # ---- 1. the backtrace
    backtrace = []
    if shutil.which('gdb') and not args.skip_gdb:
        say('=== 1. all sensors, gz server under gdb (slow to start, up to 3 min) ===')
        res = trial.run('1_all_sensors_gdb', all_on, debug=True, seconds=max(args.seconds, 120))
        results.append(res)
        report(res)
        text = Path(res['log']).read_text(errors='ignore')
        if res['status'] == 'CRASH':
            start = text.find('Program received signal')
            if start < 0:
                start = text.find('vigil_sar: gz server stopped')
            backtrace = text[start:start + 6000].splitlines() if start >= 0 else []
            say('    --- backtrace (also in ' + res['log'] + ') ---')
            for row in backtrace[:60]:
                say('      ', row[:200])
        say('')
    else:
        say('=== 1. skipped (no gdb: sudo apt install gdb gives the exact crash location) ===\n')

    # ---- 2. physics only
    say('=== 2. rover with NO rendering sensor (is it rendering at all?) ===')
    none_res = trial.run('2_no_sensors', {s: False for s in SENSORS})
    results.append(none_res)
    report(none_res)
    say('')
    if none_res['status'] == 'CRASH':
        say('    -> Gazebo crashes with no camera at all, so this is NOT a camera problem:')
        say('       it is the world + rover in the physics engine. Phase 5 bisects the world.')

    # ---- 3. one sensor at a time
    say('=== 3. one sensor at a time ===')
    singles = {}
    for sensor in SENSORS:
        res = trial.run(f'3_{sensor}', {s: s == sensor for s in SENSORS})
        results.append(res)
        report(res)
        singles[sensor] = res
    say('')
    if singles['hd_camera']['status'] == 'CRASH':
        say('=== 3b. 4K camera crashed - trying it at 1920x1080 ===')
        res = trial.run('3b_hd_1080p', {s: s == 'hd_camera' for s in SENSORS},
                        extra=('hd_width:=1920', 'hd_height:=1080'))
        results.append(res)
        report(res)
        singles['hd_1080'] = res
        say('')

    good = [s for s in SENSORS if singles[s]['status'] == 'OK']
    bad = [s for s in SENSORS if singles[s]['status'] == 'CRASH']
    hd_1080 = 'hd_camera' in bad and singles.get('hd_1080', {}).get('status') == 'OK'

    # ---- 4. everything that survived alone, together
    profile = {s: s in good for s in SENSORS}
    if hd_1080:
        profile['hd_camera'] = True
    say('=== 4. all sensors that survived alone, together ===')
    combined = trial.run('4_profile', profile,
                         extra=('hd_width:=1920', 'hd_height:=1080') if hd_1080 else ())
    results.append(combined)
    report(combined)
    order = ['hd_camera', 'segmentation', 'lidar', 'rgbd', 'thermal']   # thermal last: SAR needs it
    while combined['status'] != 'OK' and any(profile[s] for s in order):
        drop = next(s for s in order if profile[s])
        profile[drop] = False
        say(f'    together they still crash -> dropping {drop} and trying again')
        combined = trial.run('4_profile_without_' + drop, profile,
                             extra=('hd_width:=1920', 'hd_height:=1080') if hd_1080 and profile['hd_camera'] else ())
        results.append(combined)
        report(combined)
    say('')

    # ---- 5. which part of the world triggers it
    culprit = None
    world_sdf = generated / 'military_sar.sdf'
    probe = bad[0] if bad else (None if none_res['status'] != 'CRASH' else 'none')
    if probe and not args.skip_bisect and world_sdf.exists():
        sensors_for_probe = {s: s == probe for s in SENSORS}
        say(f'=== 5. which models trigger it (probe: {probe}) ===')
        variant = out / 'world_ground_only.sdf'
        removed = world_variant(world_sdf, [], variant)
        res = trial.run('5_ground_only', sensors_for_probe, extra=(f'world_file:={variant}',))
        results.append(res)
        report(res)
        if res['status'] == 'CRASH':
            culprit = 'ground/terrain (or the rover itself)'
        else:
            remaining = list(BISECT_GROUPS)
            while len(remaining) > 1:
                half = remaining[:len(remaining) // 2]
                variant = out / ('world_' + '_'.join(half) + '.sdf')
                world_variant(world_sdf, half, variant)
                res = trial.run('5_' + '_'.join(half), sensors_for_probe, extra=(f'world_file:={variant}',))
                results.append(res)
                report(res)
                remaining = half if res['status'] == 'CRASH' else remaining[len(remaining) // 2:]
            variant = out / ('world_' + remaining[0] + '.sdf')
            world_variant(world_sdf, remaining, variant)
            res = trial.run('5_' + remaining[0], sensors_for_probe, extra=(f'world_file:={variant}',))
            results.append(res)
            report(res)
            culprit = remaining[0] if res['status'] == 'CRASH' else 'no single group (only the full world crashes)'
        say(f'    -> {culprit}')
        say('')

    # ---- profile + summary
    tested = datetime.datetime.now().isoformat(timespec='minutes')
    lines = ['# vigil_sar render profile - written by find_crash.sh, read by every vigil_sar launch.',
             '# Delete this file (or launch with profile:=none) to go back to all sensors on.',
             f'tested: "{tested}"',
             f'gz: "{sh("gz sim --versions", 10).splitlines()[0] if sh("gz sim --versions", 10) else "?"}"',
             'sensors:']
    lines += [f'  {s}: {str(profile[s]).lower()}' for s in SENSORS]
    if hd_1080 and profile['hd_camera']:
        lines += ['hd_width: 1920', 'hd_height: 1080']
    if culprit:
        lines.append(f'# world models that trigger the crash: {culprit}')
    profile_path = generated / 'render_profile.yaml'
    if none_res['status'] == 'CRASH':
        # Switching sensors off cannot help a crash that happens without any sensor, and a profile
        # full of "false" would only cripple the mission. Leave the launch at its defaults.
        profile_path.unlink(missing_ok=True)
    else:
        profile_path.write_text('\n'.join(lines) + '\n')
    (out / 'find_crash.json').write_text(json.dumps(dict(results=results, profile=profile, culprit=culprit,
                                                         backtrace=backtrace), indent=1))

    say('=== RESULT ===')
    if none_res['status'] == 'CRASH':
        say('  Gazebo crashes even with no camera: the crash is in the world/physics, not a sensor.')
    elif bad:
        say('  crashes this PC:', ', '.join(bad))
    else:
        say('  no sensor crashed on its own.')
    if combined['status'] == 'OK':
        say('  working set  :', ', '.join(s for s in SENSORS if profile[s]) or 'none',
            '(1920x1080 display camera)' if hd_1080 and profile['hd_camera'] else '')
    else:
        say('  nothing ran cleanly - send the two files below, they say why.')
    if culprit:
        say('  world models :', culprit)
    if not profile['thermal']:
        say('  NOTE thermal is off in this profile, so SAR human detection cannot run yet.')
    if profile_path.exists():
        say('  profile      :', profile_path, '(used automatically by every launch)')
    else:
        say('  profile      : none written (sensors are not the problem); launches keep all sensors on')
    say('  send back    :', out / 'find_crash.log', 'and', out / 'find_crash.json')
    say('')
    say('  now run the mission:  ros2 launch vigil_sar sar_mission.launch.py')
    LOG.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
