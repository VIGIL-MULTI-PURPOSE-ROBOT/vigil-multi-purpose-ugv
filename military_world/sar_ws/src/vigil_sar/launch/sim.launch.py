"""military_world + ONE VIGIL rover + sensors (Gazebo Harmonic, ROS 2 Jazzy).

  ros2 launch vigil_sar sim.launch.py                 # GUI
  ros2 launch vigil_sar sim.launch.py gui:=false      # server only
  ros2 launch vigil_sar sim.launch.py debug:=true     # server under gdb: prints the crash backtrace

1. builds <military_world>/sar_ws/generated/military_sar.sdf from the untouched export
   (human heat signatures, casualties inside buildings, physics step) - build_sar_world.py
2. starts gz sim on that world, spawns the rover (from urdf/agri_ugv.urdf.xacro with the
   4K RGB camera and the thermal camera sized from config/sar_mission.yaml) at point A
3. bridge (config/bridge.yaml), robot_state_publisher, wheel controllers, drive.py
Only this launch spawns a robot, and it spawns exactly one: 'agri_ugv'.

Sensors: rgbd, lidar, segmentation, hd_camera, thermal are each true/false. An empty value means
"use sar_ws/generated/render_profile.yaml" (written by sar_ws/find_crash.sh after it has tested
which sensors this PC's Gazebo can render), else on. profile:=none ignores that file.
If the Gazebo server dies, the Gazebo GUI is killed at once (no frozen window / Force Quit dialog)
and the whole launch stops within ~2 s.
"""
import os
import shutil
import signal
import sys
import time
from pathlib import Path

import math

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'vigil_sar'
PARTITION = 'vigil_sar'
NODE_SCRIPTS = ('terrain_mapper.py', 'cliff_navigator.py', 'ugv_dashboard.py', 'drive.py',
                'thermal_human_detector.py', 'human_tracker.py', 'sar_manager.py', 'build_sar_world.py')


def ensure_executable():
    """ros2 launch only starts scripts with the executable bit (copying files can drop it)."""
    lib = Path(get_package_prefix(PKG)) / 'lib' / PKG
    fixed = []
    for name in NODE_SCRIPTS:
        path = lib / name
        if path.exists():
            real = Path(os.path.realpath(path))
            try:
                if real.stat().st_mode & 0o111 != 0o111:
                    real.chmod(real.stat().st_mode | 0o755)
                    fixed.append(name)
            except OSError:
                pass
    return fixed


def stop_stale_simulation():
    """A Gazebo server left over from an earlier launch keeps its own controller_manager and
    rover (a second, unwanted rover). Stop every process started with GZ_PARTITION=vigil_sar."""
    me = {os.getpid(), os.getppid()}
    victims = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) in me:
            continue
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                if f'GZ_PARTITION={PARTITION}'.encode() not in f.read().split(b'\0'):
                    continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='ignore').strip()
            os.kill(int(pid), signal.SIGTERM)
            victims.append((int(pid), cmd[:90]))
        except (OSError, ValueError):
            continue
    if victims:
        time.sleep(2.0)
        for pid, _ in victims:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    return victims


NVIDIA_EGL = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'


def gpu_env(choice):
    """Hybrid laptops (Intel iGPU + NVIDIA dGPU) render on the Intel iGPU by default. On this
    machine (Arrow Lake iGPU, Mesa) Gazebo's ogre2 renderer segfaults as soon as a camera renders
    military_world (700 meshes), while the same cameras in a tiny world work. PRIME render
    offload moves Gazebo (server sensors + GUI) to the NVIDIA GPU."""
    have_nv = os.path.exists(NVIDIA_EGL) or shutil.which('nvidia-smi') is not None
    if choice == 'intel' or (choice == 'auto' and not have_nv):
        return {}, 'default GPU (integrated)'
    env = {'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
           '__VK_LAYER_NV_optimus': 'NVIDIA_only'}
    if os.path.exists(NVIDIA_EGL):
        env['__EGL_VENDOR_LIBRARY_FILENAMES'] = NVIDIA_EGL
    return env, 'NVIDIA GPU (PRIME render offload)'


SENSOR_ARGS = ('rgbd', 'lidar', 'segmentation', 'hd_camera', 'thermal')


def load_profile(path):
    """sar_ws/generated/render_profile.yaml from find_crash.sh: {sensors: {name: bool}, hd_width, ...}."""
    try:
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError, ImportError):
        return {}
    except Exception:   # noqa: BLE001  a broken profile must never stop the launch
        return {}


def gdb_command(server_args):
    """Run the gz server under gdb so a segfault prints WHERE it crashed (gz sim itself prints
    nothing). 'gz' is a Ruby script that runs the server in-process, so gdb runs ruby."""
    gdb, gz = shutil.which('gdb'), shutil.which('gz')
    if not gdb or not gz:
        return None
    try:
        with open(os.path.realpath(gz), 'rb') as f:
            head = f.readline()
    except OSError:
        return None
    target = [gz]
    if head.startswith(b'#!') and b'ruby' in head:
        ruby = shutil.which('ruby')
        if not ruby:
            return None
        target = [ruby, gz]
    return [gdb, '-q', '-batch', '-ex', 'set pagination off', '-ex', 'set print thread-events off',
            '-ex', 'set print inferior-events off',
            '-ex', 'handle SIGPIPE SIGUSR1 SIGUSR2 SIGALRM SIGVTALRM SIGCHLD SIGWINCH nostop noprint pass',
            '-ex', 'run',
            '-ex', 'echo \\n===== vigil_sar: gz server stopped - backtrace of the stopping thread =====\\n',
            '-ex', 'bt 60',
            '-ex', 'echo \\n===== vigil_sar: all threads =====\\n',
            '-ex', 'thread apply all bt 12',
            '-ex', 'echo \\n===== vigil_sar: end of backtrace =====\\n',
            '--args'] + target + server_args


def kill_process(action):
    """SIGKILL a launched process right now (used for the Gazebo GUI when the server is gone:
    a GUI without its server freezes and the desktop shows 'not responding - Force Quit')."""
    try:
        details = action.process_details
    except Exception:   # noqa: BLE001
        details = None
    pid = details.get('pid') if details else None
    if pid:
        try:
            os.kill(pid, signal.SIGKILL)
            return True
        except OSError:
            pass
    return False


def setup(context):
    share = Path(get_package_share_directory(PKG))
    sys.path.insert(0, str(Path(get_package_prefix(PKG)) / 'lib' / PKG))
    from sar_paths import find_military_world, load_config   # noqa: E402
    from build_sar_world import build                        # noqa: E402
    lc = lambda k: LaunchConfiguration(k).perform(context).strip()   # noqa: E731
    cfg_path = lc('config') or str(share / 'config/sar_mission.yaml')
    cfg = load_config(cfg_path)
    phys_path = lc('physics') or str(share / 'config/physics.yaml')
    phy = load_config(phys_path).get('physics', {})
    w, rgb, tc, sp = cfg['world'], cfg['rgb_camera'], cfg['thermal_camera'], cfg['spawn']
    mw = find_military_world(lc('military_world') or w.get('military_world_dir', ''))
    actions = []
    eng = phy.get('engine', {})
    w['physics_step'] = float(eng.get('max_step_size', w.get('physics_step', 0.001)))
    w['gravity'] = float(eng.get('gravity', 9.81))
    # engine settings as vigil_rough_terrain (config/physics.yaml engine): DART defaults, ground mu 1.0
    w['collision_detector'] = str(eng.get('collision_detector', w.get('collision_detector', 'default')))
    w['physics_solver'] = str(eng.get('solver', w.get('physics_solver', 'default')))
    w['ground_friction'] = eng.get('ground_friction', w.get('ground_friction', 1.0))
    if lc('fast') == 'true':
        # The 300 m world with 1 ms physics, 20 walking people and five cameras runs at a few
        # percent of real time on this class of laptop. fast:=true drops the walking people and the
        # 4K display camera; the physics itself (1 ms step, gravity, friction) stays exactly the
        # vigil_rough_terrain physics - it used to double the step to 2 ms, which made the rover's
        # springs and contacts behave differently from vigil_rough_terrain.
        w['moving_people'] = False
        # the 4K display camera is 24.9 MB per frame over the bridge - the heaviest single item
        rgb['width'], rgb['height'] = 1920, 1080
        actions.append(LogInfo(msg='[vigil_sar] fast:=true - walking people off, display camera 1920x1080; '
                                   'physics unchanged (vigil_rough_terrain: 1 ms step, 9.81 m/s2, ground mu 1.0). '
                                   'The dashboard header shows the simulation speed.'))
    if lc('cleanup') == 'true':
        actions += [LogInfo(msg=f'[vigil_sar] stopped leftover process {p}: {c}') for p, c in stop_stale_simulation()]
    actions += [LogInfo(msg=f'[vigil_sar] made executable: {n}') for n in ensure_executable()]

    # ---- sensor selection: command line > render profile (find_crash.sh) > defaults
    prof_arg = lc('profile') or 'auto'
    prof_path = Path(prof_arg) if prof_arg not in ('auto', 'none') else mw / 'sar_ws/generated/render_profile.yaml'
    profile = load_profile(prof_path) if prof_arg != 'none' else {}
    psens = profile.get('sensors', {}) if isinstance(profile.get('sensors'), dict) else {}
    defaults = {'rgbd': True, 'lidar': True, 'segmentation': True, 'hd_camera': True,
                'thermal': bool(tc.get('enabled', True))}
    on, source = {}, {}
    for k in SENSOR_ARGS:
        v = lc(k).lower()
        if v in ('true', 'false'):
            on[k], source[k] = v == 'true', 'command line'
        elif k in psens:
            on[k], source[k] = bool(psens[k]), 'profile'
        else:
            on[k], source[k] = defaults[k], 'default'
    hd_w = lc('hd_width') or str(profile.get('hd_width', '') or int(rgb.get('width', 3840)))
    hd_h = lc('hd_height') or str(profile.get('hd_height', '') or int(rgb.get('height', 2160)))
    gpu_choice = lc('gpu') or str(profile.get('gpu', 'auto'))
    if profile:
        actions.append(LogInfo(msg=f"[vigil_sar] render profile {prof_path} (tested {profile.get('tested', '?')}): "
                                   + ', '.join(f"{k}={'on' if on[k] else 'OFF'}" for k in SENSOR_ARGS)))
    off = [k for k in SENSOR_ARGS if not on[k]]
    if 'thermal' in off:
        actions.append(LogInfo(msg='[vigil_sar] WARNING thermal camera is OFF: SAR human detection needs it.'))

    # ---- world
    world_file = lc('world_file')
    if world_file:
        actions.append(LogInfo(msg=f'[vigil_sar] world (prebuilt, not rebuilt): {world_file}'))
    else:
        rep = build(cfg, mw)
        world_file = rep['output']
        actions.append(LogInfo(msg=f"[vigil_sar] world {world_file}: {rep['n_humans']} humans with heat signature "
                                   f"({len(rep['building_humans'])} inside buildings), {rep['movers']} walking, "
                                   f"{rep.get('thermal_plugins', '?')} thermal plugins "
                                   f"({rep.get('ambient_plugins_dropped', 0)} ambient-temperature ones dropped)"))
        ph = rep.get('physics', {})
        actions.append(LogInfo(msg=f"[vigil_sar] physics: gravity {ph.get('gravity')} m/s2, "
                                   f"{ph.get('step', 0) * 1000:.0f} ms step, "
                                   f"{ph.get('collision_detector')} collision detector, {ph.get('solver')} solver"))
        gfr = rep.get('ground_friction', {})
        actions.append(LogInfo(msg=f"[vigil_sar] ground friction mu {gfr.get('value')} on {gfr.get('surfaces')} "
                                   f"drivable surfaces, as vigil_rough_terrain (export values were {gfr.get('changed')})"))
        z = [w.get(k) for k in ('zone_x_min', 'zone_x_max', 'zone_y_min', 'zone_y_max')]
        if None not in z:
            actions.append(LogInfo(msg=f"[vigil_sar] operational zone x [{z[0]:g}, {z[1]:g}] y [{z[2]:g}, {z[3]:g}] "
                                       f"({z[1] - z[0]:g} x {z[3] - z[2]:g} m) inside the 300 x 300 m world (1 unit = 1 m, "
                                       f"no rescaling); {len(rep.get('frozen_outside_zone', []))} walkers outside it stand still"))
        for fix in rep.get('collision_fixes', []):
            actions.append(LogInfo(msg=f"[vigil_sar] collision fix: {fix['model']} -> {fix['action']} ({fix['reason']})"))
        if rep.get('start_blocked_by'):
            actions.append(LogInfo(msg=f"[vigil_sar] WARNING something stands on the robot start: {rep['start_blocked_by']}"))
        if rep['in_band_non_human']:
            actions.append(LogInfo(msg=f"[vigil_sar] WARNING non-human objects in the human band: {rep['in_band_non_human']}"))
    plugin_dir = mw / 'build'
    if w.get('moving_people', True) and not (plugin_dir / 'libsar-waypoint-system.so').exists():
        actions.append(LogInfo(msg='[vigil_sar] WARNING military_world/build/libsar-waypoint-system.so missing: '
                                   'people will not walk. Build it once: cd military_world && bash run_gazebo.sh --check'))

    # ---- rover model: the vigil_rough_terrain rover (urdf/agri_ugv.urdf.xacro). Two values are
    # passed in from physics.drive: the wheel speed limit (the fastest wheel at full speed AND full
    # yaw rate - the outer wheels of a turn - on the smallest wheel r 0.17655 m, +10 %) and the
    # wheel torque limit.
    drv = phy.get('drive', {})
    max_v = float(drv.get('max_linear', 0.7))
    max_w = float(drv.get('max_angular', 0.6))
    r_small = float(phy.get('rover', {}).get('wheel_radius_small', 0.17655))
    wheel_max = max(15.0, round((max_v + max_w * float(drv.get('skid_factor', 0.65))) / r_small * 1.10, 1))
    wheel_effort = float(drv.get('wheel_torque_limit', 400.0))
    actions.append(LogInfo(msg=(
        f"[vigil_sar] rover: vigil_rough_terrain model (mu 0.85), wheel torque limit {wheel_effort:g} N.m, "
        f"wheel speed limit {wheel_max} rad/s; drive: wheel {drv.get('mode', 'velocity')} control, max "
        f"{max_v} m/s / {max_w} rad/s, accel {drv.get('max_accel', '?')} m/s2, brake {drv.get('max_decel', '?')} "
        f"m/s2, jerk {drv.get('max_jerk', '?')} m/s3, yaw ramp {drv.get('ramp_angular', 0.5)} rad/s2; "
        f"gravity {phy.get('engine', {}).get('gravity', 9.81)} m/s2")))

    tf = lambda b: 'true' if b else 'false'   # noqa: E731
    description = xacro.process_file(str(share / 'urdf/agri_ugv.urdf.xacro'), mappings={
        'simulation': 'true', 'vision_sensors': 'true',
        'rgbd_camera': tf(on['rgbd']), 'lidar': tf(on['lidar']),
        'segmentation_camera': tf(on['segmentation']), 'hd_camera': tf(on['hd_camera']),
        'hd_width': hd_w, 'hd_height': hd_h, 'hd_rate': str(float(rgb.get('fps', 5.0))),
        'thermal_camera': tf(on['thermal']),
        'thermal_width': str(int(tc.get('resolution_width', 320))),
        'thermal_height': str(int(tc.get('resolution_height', 240))),
        'thermal_hfov': str(float(tc.get('horizontal_fov', 1.0472))),
        'thermal_rate': str(float(tc.get('update_rate', 10.0))),
        'thermal_x': str(float(tc.get('mount_x', 0.52))), 'thermal_y': str(float(tc.get('mount_y', 0.0))),
        'thermal_height_above_ground': str(float(tc.get('camera_height', 1.0))),
        'thermal_pitch': str(float(tc.get('camera_pitch', 0.0))),
        'thermal_near': str(float(tc.get('near', 0.2))), 'thermal_far': str(float(tc.get('far', 120.0))),
        'thermal_noise': str(float(tc.get('noise_stddev', 0.0))),
        # A <noise> element on a thermal_camera segfaults gz-sensors 8 (null noise model in
        # ThermalCameraSensor::CreateCamera). Off unless the config says this Gazebo is fixed.
        'thermal_noise_sdf': 'true' if tc.get('noise_in_sdf', False) else 'false',
        'wheel_max_rad_s': str(wheel_max),
        'wheel_effort': f'{wheel_effort:g}',
    }).toxml()
    env = [
        SetEnvironmentVariable('GZ_PARTITION', PARTITION),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(
            [str(share.parent), str(mw / 'gazebo_export'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join(
            [get_package_prefix('gz_ros2_control') + '/lib', str(plugin_dir),
             os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')])),
    ]
    genv, gname = gpu_env(gpu_choice)

    # ---- Gazebo server. emulate_tty: line-buffered, so the last lines before a crash are not lost;
    # output 'both': those lines also land in ~/.ros/log/<run>/launch.log.
    server_args = ['sim', '-s', '-r', '-v', '3'] + (['--headless-rendering'] if lc('headless') == 'true' else []) \
        + [world_file]
    debug = lc('debug') == 'true'
    server_cmd = gdb_command(server_args) if debug else None
    if debug and server_cmd is None:
        actions.append(LogInfo(msg='[vigil_sar] debug:=true needs gdb (sudo apt install gdb) - running without it'))
    sim = ExecuteProcess(cmd=server_cmd or (['gz'] + server_args), name='gz_server', output='both',
                         emulate_tty=True, additional_env=genv,
                         sigterm_timeout='20' if server_cmd else '5', sigkill_timeout='5')
    sensors = [k for k in SENSOR_ARGS if on[k]] + ['imu']
    if on['hd_camera']:
        sensors[sensors.index('hd_camera')] = f'hd_camera {hd_w}x{hd_h}'
    actions.append(LogInfo(msg=f"[vigil_sar] rendering on {gname}; sensors: {', '.join(sensors)}"
                               + (f"; OFF: {', '.join(off)}" if off else '')
                               + ('; server under gdb (debug)' if server_cmd else '')))

    # ---- Gazebo GUI. Wayland: the Qt/Ogre GUI needs the X11 (XWayland) backend - military_world's
    # own run_gazebo.sh sets the same. Started only after the rover exists, so it never waits on a
    # server that is still loading. 'own_log': its thousands of harmless sdformat warnings go to
    # ~/.ros/log/<run>/gz_gui-*.log instead of the terminal. Short stop timeouts: never a frozen window.
    gui_env = dict(genv, QT_QPA_PLATFORM=os.environ.get('QT_QPA_PLATFORM', 'xcb'))
    # config/gui.config drops the Entity Tree and the Component Inspector: with 520 models they
    # keep the Qt main thread busy for many seconds while the world loads, which is exactly when
    # the desktop offers "not responding - Force Quit / Wait". gui_config:=default restores them.
    gui_cfg = lc('gui_config') or str(share / 'config/gui.config')
    gui_cmd = ['gz', 'sim', '-g', '-v', '2'] + ([] if gui_cfg == 'default' else ['--gui-config', gui_cfg])
    gui = ExecuteProcess(cmd=gui_cmd, name='gz_gui', output='own_log',
                         additional_env=gui_env, sigterm_timeout='1', sigkill_timeout='1')
    x, y = lc('spawn_x') or str(sp.get('x', 0.0)), lc('spawn_y') or str(sp.get('y', -8.0))
    z, yaw = lc('spawn_z') or str(sp.get('z', 0.8)), str(sp.get('yaw', 1.5708))
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', w.get('world_name', 'military_world'), '-name', 'agri_ugv',
                            '-topic', 'robot_description', '-x', x, '-y', y, '-z', z, '-Y', yaw])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'wheel_controller', '--controller-manager-timeout', '180',
                                  '--switch-timeout', '120', '--service-call-timeout', '150'],
                       sigterm_timeout='2', sigkill_timeout='2')
    want_gui = lc('gui') == 'true'
    # Gazebo GUI camera follows the rover (gz CameraTracking, in config/gui.config): a 1.5 m rover
    # is a speck in the default view of a 300 m world. Retries until the GUI is up. To look around
    # freely:  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean
    # --timeout 2000 --req 'data: ""'   (or set world.gui_follow_rover: false).
    follow = []
    if bool(w.get('gui_follow_rover', True)):
        off = [float(v) for v in (w.get('gui_follow_offset') or [-5.0, 0.0, 2.5])]
        script = ('for i in $(seq 90); do gz service -s /gui/follow --reqtype gz.msgs.StringMsg '
                  '--reptype gz.msgs.Boolean --timeout 2000 --req \'data: "agri_ugv"\' >/dev/null 2>&1 && '
                  'gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean '
                  f'--timeout 2000 --req \'x: {off[0]}, y: {off[1]}, z: {off[2]}\' >/dev/null 2>&1 && '
                  'echo "[vigil_sar] GUI camera follows the rover" && exit 0; sleep 2; done; '
                  'echo "[vigil_sar] GUI camera follow not available (CameraTracking plugin missing?)"')
        follow = [ExecuteProcess(cmd=['bash', '-c', script], name='gui_follow', output='screen')]
    state = {'server_dead': False}

    def spawned(event, _ctx):
        if event.returncode != 0:
            return [LogInfo(msg='Rover spawn failed.'), EmitEvent(event=Shutdown(reason='Rover spawn failed'))]
        if state['server_dead']:
            return []
        return [controllers] + ([gui] + follow if want_gui else [])

    def server_exit(event, _ctx):
        state['server_dead'] = True
        gui_killed = kill_process(gui) if want_gui else False
        rc = event.returncode
        if rc is not None and rc < 0:
            msg = (f'Gazebo server CRASHED (signal {-rc}) - a Gazebo rendering/physics crash, not a vigil_sar node. '
                   f'Run  bash {mw}/sar_ws/find_crash.sh  once: it finds the sensor this PC cannot render, '
                   f'saves a working render profile and records the crash backtrace.')
        elif server_cmd:
            msg = 'Gazebo server stopped (debug: if it crashed, the backtrace is printed just above).'
        else:
            msg = f'Gazebo server stopped (exit code {rc}).'
        out = [LogInfo(msg='[vigil_sar] ' + msg)]
        if gui_killed:
            out.append(LogInfo(msg='[vigil_sar] Gazebo GUI closed at once, so it cannot freeze (no Force Quit dialog).'))
        return out + [EmitEvent(event=Shutdown(reason=msg))]

    def gui_exit(event, _ctx):
        if state['server_dead']:
            return []
        return [EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))]

    actions += env + [
        LogInfo(msg=f'[vigil_sar] point A = spawn ({x}, {y}), config {cfg_path}'),
        sim,
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=server_exit)),
        RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=gui_exit)),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
        spawn,
        Node(package=PKG, executable='drive.py', parameters=[phys_path, {'use_sim_time': True}], output='screen'),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('config', default_value='', description='sar_mission.yaml (empty = package default)'),
        DeclareLaunchArgument('physics', default_value='', description='physics.yaml (empty = package default)'),
        DeclareLaunchArgument('military_world', default_value='', description='military_world folder (empty = auto)'),
        DeclareLaunchArgument('spawn_x', default_value=''),
        DeclareLaunchArgument('spawn_y', default_value=''),
        DeclareLaunchArgument('spawn_z', default_value=''),
        DeclareLaunchArgument('rgbd', default_value='', description='true/false; empty = render profile, else on'),
        DeclareLaunchArgument('lidar', default_value='', description='true/false; empty = render profile, else on'),
        DeclareLaunchArgument('segmentation', default_value='',
                              description='segmentation camera (cliff void test); empty = profile, else on'),
        DeclareLaunchArgument('hd_camera', default_value='', description='4K display camera; empty = profile, else on'),
        DeclareLaunchArgument('thermal', default_value='',
                              description='thermal camera; empty = profile, else thermal_camera.enabled'),
        DeclareLaunchArgument('hd_width', default_value='', description='empty = profile / rgb_camera.width'),
        DeclareLaunchArgument('hd_height', default_value='', description='empty = profile / rgb_camera.height'),
        DeclareLaunchArgument('profile', default_value='auto',
                              description='auto = sar_ws/generated/render_profile.yaml if present | none | <file>'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='run the gz server under gdb and print the crash backtrace'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='use this prebuilt SDF instead of building military_sar.sdf'),
        DeclareLaunchArgument('headless', default_value='false', description='server renders with EGL (--headless-rendering)'),
        DeclareLaunchArgument('gpu', default_value='', description='auto (NVIDIA if present) | nvidia | intel; empty = profile/auto'),
        DeclareLaunchArgument('gui_config', default_value='',
                              description="empty = the light config/gui.config | default = Gazebo's own | <file>"),
        DeclareLaunchArgument('fast', default_value='false',
                              description='no walking people, 1080p display camera: a faster run (physics unchanged)'),
        DeclareLaunchArgument('cleanup', default_value='true',
                              description='stop Gazebo/rovers left over from an earlier vigil_sar launch'),
        OpaqueFunction(function=setup),
    ])
