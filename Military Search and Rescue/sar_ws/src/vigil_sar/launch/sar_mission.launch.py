"""Complete SAR UGV mission in ONE command:
military_world + rover + sensors + navigation + SAR layer + dashboard.

  ros2 launch vigil_sar sar_mission.launch.py                  # then open http://localhost:8080
  ros2 launch vigil_sar sar_mission.launch.py gui:=false goal_x:=80 goal_y:=40
  bash ~/Documents/military_world/sar_ws/find_crash.sh      # once, if Gazebo crashes on this PC

Mission: robot starts at A (spawn) -> SET GOAL B on the map (or goal_x/goal_y) -> navigates
A -> B -> waits -> press the SAR switch -> 10 search points + building scans with the
thermal camera -> HUMAN DETECTED markers H1, H2 ... -> SAR COMPLETE.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = Path(__file__).resolve().parent


def inc(name, args, condition=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(HERE / name)),
                                    launch_arguments={k: LaunchConfiguration(k) for k in args}.items(),
                                    condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('physics', default_value=''),
        DeclareLaunchArgument('military_world', default_value=''),
        DeclareLaunchArgument('goal_x', default_value=''),
        DeclareLaunchArgument('goal_y', default_value=''),
        DeclareLaunchArgument('spawn_x', default_value=''),
        DeclareLaunchArgument('spawn_y', default_value=''),
        DeclareLaunchArgument('spawn_z', default_value=''),
        DeclareLaunchArgument('cleanup', default_value='true'),
        # sensors: true/false, empty = sar_ws/generated/render_profile.yaml (find_crash.sh), else on
        DeclareLaunchArgument('rgbd', default_value=''),
        DeclareLaunchArgument('lidar', default_value=''),
        DeclareLaunchArgument('segmentation', default_value=''),
        DeclareLaunchArgument('hd_camera', default_value=''),
        DeclareLaunchArgument('thermal', default_value=''),
        DeclareLaunchArgument('hd_width', default_value=''),
        DeclareLaunchArgument('hd_height', default_value=''),
        DeclareLaunchArgument('profile', default_value='auto'),
        DeclareLaunchArgument('debug', default_value='false', description='gz server under gdb (crash backtrace)'),
        DeclareLaunchArgument('world_file', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gpu', default_value=''),
        DeclareLaunchArgument('fast', default_value='false'),
        DeclareLaunchArgument('sim_speed', default_value='', description='1 / 2 / 4 (empty = world.simulation_speed)'),
        DeclareLaunchArgument('gui_config', default_value=''),
        inc('sim.launch.py', ['gui', 'config', 'physics', 'military_world', 'spawn_x', 'spawn_y', 'spawn_z', 'cleanup',
                             'rgbd', 'lidar', 'segmentation', 'hd_camera', 'thermal', 'hd_width', 'hd_height',
                             'profile', 'debug', 'world_file', 'headless', 'gpu', 'fast', 'gui_config', 'sim_speed']),
        inc('navigation.launch.py', ['config', 'goal_x', 'goal_y']),
        inc('sar.launch.py', ['config']),
        inc('dashboard.launch.py', ['config'], IfCondition(LaunchConfiguration('dashboard'))),
    ])
