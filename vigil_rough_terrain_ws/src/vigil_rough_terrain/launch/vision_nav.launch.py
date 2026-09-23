"""Vision cliff detection + A-to-B navigation + dashboard on top of terrain_sim.

  ros2 launch vigil_rough_terrain vision_nav.launch.py                      # rock terrain, B from yaml
  ros2 launch vigil_rough_terrain vision_nav.launch.py goal_x:=2.0 goal_y:=4.0
  ros2 launch vigil_rough_terrain vision_nav.launch.py scenario:=cliff_front  # worlds/test/*.sdf

Starts: terrain_sim.launch.py (Gazebo, rover, bridge, drive.py - unchanged),
segmentation bridge, terrain_mapper, cliff_navigator, ugv_dashboard (http://localhost:8080).
"""
import importlib.util
import os
import re
import signal
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LABEL = ('<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
         '<label>1</label></plugin>')


def labelled_rock_world(share):
    """Copy of rock_terrain.sdf with a segmentation label on the terrain (original untouched)."""
    text = (share / 'worlds/rock_terrain.sdf').read_text()
    text = re.sub(r'(<name>rocky_terrain_n4</name>\s*<pose>[^<]*</pose>)', r'\1' + LABEL, text, count=1)
    f = tempfile.NamedTemporaryFile('w', suffix='.sdf', prefix='vigil_vision_', delete=False)
    f.write(text)
    f.close()
    return f.name


def stop_stale_simulation():
    """Stop processes left over from earlier launches of this package.

    A leftover Gazebo server keeps its own /controller_manager on the same ROS domain; the
    new launch's spawner then talks to the OLD one ("can not be configured from 'active'
    state"), the new rover's wheel controllers never start and the robot does not move.
    Our launch files give every process GZ_PARTITION=vigil..., so only those are stopped."""
    me = {os.getpid(), os.getppid()}
    victims = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) in me:
            continue
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = f.read().split(b'\0')
            if not any(e.startswith(b'GZ_PARTITION=vigil') for e in env):
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


NODE_SCRIPTS = ('terrain_mapper.py', 'cliff_navigator.py', 'ugv_dashboard.py', 'drive.py',
                'comparison_driver.py', 'make_test_worlds.py')


def ensure_executable():
    """ros2 launch only starts scripts with the executable bit. Copying updated files onto
    the workspace can drop it ("executable 'terrain_mapper.py' not found on the libexec
    directory"), so restore it on the real files behind the install/ symlinks."""
    fixed = []
    lib = Path(get_package_prefix('vigil_rough_terrain')) / 'lib' / 'vigil_rough_terrain'
    for name in NODE_SCRIPTS:
        path = lib / name
        if not path.exists():
            continue
        real = Path(os.path.realpath(path))
        try:
            mode = real.stat().st_mode
            if mode & 0o111 != 0o111:
                real.chmod(mode | 0o755)
                fixed.append(name)
        except OSError:
            pass
    return fixed


def setup(context):
    share = Path(get_package_share_directory('vigil_rough_terrain'))
    made_exec = ensure_executable()
    stale = stop_stale_simulation() if LaunchConfiguration('cleanup').perform(context) == 'true' else []
    cfg = str(share / 'config/vision_nav.yaml')
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    scenario = lc('scenario')
    seg = lc('segmentation')
    overrides = {}
    if scenario:
        spec = importlib.util.spec_from_file_location('scenarios', share / 'scripts/scenarios.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if scenario not in mod.SCENARIOS:
            return [LogInfo(msg=f'unknown scenario {scenario}; choose from {sorted(mod.SCENARIOS)}')]
        sc = mod.SCENARIOS[scenario]
        world = str(share / f'worlds/test/{scenario}.sdf')
        spawn = ('0.0', '0.0', f"{sc['spawn_z'] + 0.35:.2f}")
        overrides['navigation.goal_x'], overrides['navigation.goal_y'] = map(float, sc['goal'])
        info = f"scenario '{scenario}': {sc['desc']}  expected: {sc['expect']['outcome']}"
    else:
        world = labelled_rock_world(share) if seg == 'true' else str(share / 'worlds/rock_terrain.sdf')
        spawn = (lc('spawn_x'), lc('spawn_y'), lc('spawn_z'))
        info = 'rock terrain'
    if lc('goal_x'):
        overrides['navigation.goal_x'] = float(lc('goal_x'))
    if lc('goal_y'):
        overrides['navigation.goal_y'] = float(lc('goal_y'))
    params = [cfg, overrides] if overrides else [cfg]
    # A private Gazebo partition per launch: a Gazebo server left running by an earlier
    # launch (same world name) can then never receive this launch's rover or controllers.
    partition = lc('partition') or f'vigil_{os.getpid()}'
    actions = [LogInfo(msg=f'[vision_nav] stopped leftover process {pid}: {cmd}') for pid, cmd in stale]
    actions += [LogInfo(msg=f'[vision_nav] made executable: {n}') for n in made_exec]
    actions += [
        SetEnvironmentVariable('GZ_PARTITION', partition),
        LogInfo(msg=f'[vision_nav] {info}; world={world}; GZ_PARTITION={partition}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/terrain_sim.launch.py')),
            launch_arguments={'gui': lc('gui'), 'world': world, 'spawn_x': spawn[0], 'spawn_y': spawn[1],
                              'spawn_z': spawn[2], 'vision_sensors': 'true', 'partition': partition,
                              'camera_width': lc('camera_width'), 'camera_height': lc('camera_height'),
                              'camera_rate': lc('camera_rate')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='hd_camera_bridge',
             arguments=['/camera/hd/image@sensor_msgs/msg/Image[gz.msgs.Image'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='vigil_rough_terrain', executable='terrain_mapper.py', name='terrain_mapper',
             parameters=params, output='screen'),
        Node(package='vigil_rough_terrain', executable='cliff_navigator.py', name='cliff_navigator',
             parameters=params, output='screen'),
        Node(package='vigil_rough_terrain', executable='ugv_dashboard.py', name='ugv_dashboard',
             parameters=params, output='screen', condition=IfCondition(LaunchConfiguration('dashboard'))),
    ]
    if seg == 'true':
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='segmentation_bridge',
                            arguments=['/camera/segmentation/labels_map@sensor_msgs/msg/Image[gz.msgs.Image',
                                       '/camera/segmentation/colored_map@sensor_msgs/msg/Image[gz.msgs.Image'],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='',
                              description='test world name from scripts/scenarios.py (empty = rock terrain)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('segmentation', default_value='true'),
        DeclareLaunchArgument('goal_x', default_value='', description='override navigation.goal_x'),
        DeclareLaunchArgument('goal_y', default_value='', description='override navigation.goal_y'),
        DeclareLaunchArgument('spawn_x', default_value='-5.4'),
        DeclareLaunchArgument('spawn_y', default_value='-3.6'),
        DeclareLaunchArgument('spawn_z', default_value='1.2'),
        # HD display camera. 1920x1440 by default; 8K is possible (camera_width:=7680
        # camera_height:=5760 camera_rate:=2) but ~100 MB per frame will slow the simulation.
        DeclareLaunchArgument('camera_width', default_value='1920'),
        DeclareLaunchArgument('camera_height', default_value='1440'),
        DeclareLaunchArgument('camera_rate', default_value='10'),
        DeclareLaunchArgument('cleanup', default_value='true',
                              description='stop Gazebo/nodes left over from earlier launches of this package'),
        DeclareLaunchArgument('partition', default_value='',
                              description='Gazebo partition (empty = unique per launch)'),
        OpaqueFunction(function=setup),
    ])
