"""Launch only the supplied terrain and the copied VIGIL rover."""
import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.actions import EmitEvent, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('vigil_rough_terrain'))

    def state_publisher(context):
        # vision_sensors:=true adds the segmentation camera (default: unchanged model).
        description = xacro.process_file(
            str(share / 'urdf/agri_ugv.urdf.xacro'),
            mappings={'simulation': 'true',
                      'vision_sensors': LaunchConfiguration('vision_sensors').perform(context),
                      'hd_width': LaunchConfiguration('camera_width').perform(context),
                      'hd_height': LaunchConfiguration('camera_height').perform(context),
                      'hd_rate': LaunchConfiguration('camera_rate').perform(context)}).toxml()
        return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen')]
    def bridge(context):
        # truth_isolation:=true (VIGIL 2.0 GPS-denied evaluation): simulator truth is bridged only as
        # /eval/ground_truth and /eval/link_poses, for the isolated evaluator. Default: unchanged bridge.
        isolated = LaunchConfiguration('truth_isolation').perform(context) == 'true'
        cfg = share / ('config/bridge_truth_isolated.yaml' if isolated else 'config/bridge.yaml')
        return [LogInfo(msg=f'[terrain_sim] bridge config: {cfg.name}'),
                Node(package='ros_gz_bridge', executable='parameter_bridge',
                     parameters=[{'config_file': str(cfg), 'use_sim_time': True}], output='screen')]
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'vigil_rock_terrain', '-name', 'agri_ugv',
                            '-topic', 'robot_description',
                            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                            '-z', LaunchConfiguration('spawn_z'), '-Y', '0'])
    # Controller spawner with bounded retries. On the Legion (25 Nov 2026, 2 of 18 launches) the
    # controller manager's response to the spawner's list_controllers call was lost ("failed to
    # send response ... (timeout)"); with a 100 s service-call timeout and no retry the wheel
    # controller was never activated and the rover could not move. A normal spawn takes ~4 s.
    max_attempts = 4

    def spawner_chain(attempt):
        spawner = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'wheel_controller',
                                  '--controller-manager-timeout', '120',
                                  '--switch-timeout', '90', '--service-call-timeout', '20'])

        def finished(event, context):
            if event.returncode == 0:
                return [LogInfo(msg=f'Controllers active (spawner attempt {attempt}/{max_attempts}).')]
            if attempt >= max_attempts:
                return [LogInfo(msg=f'Controller spawner failed {max_attempts} times (last exit code '
                                    f'{event.returncode}): the wheels cannot be driven - shutting down.'),
                        EmitEvent(event=Shutdown(reason='wheel controller could not be activated'))]
            return [LogInfo(msg=f'Controller spawner attempt {attempt}/{max_attempts} failed (exit code '
                                f'{event.returncode}) - retrying.')] + spawner_chain(attempt + 1)
        return [RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=finished)), spawner]

    def spawned(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Rover spawn failed.'),
                    EmitEvent(event=Shutdown(reason='Rover spawn failed'))]
        return spawner_chain(1)
    sim = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '2',
                              LaunchConfiguration('world')], output='screen')
    gui = ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2'],
                         condition=IfCondition(LaunchConfiguration('gui')), output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='-5.4'),
        DeclareLaunchArgument('spawn_y', default_value='-3.6'),
        DeclareLaunchArgument('spawn_z', default_value='1.2'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds/rock_terrain.sdf')),
        DeclareLaunchArgument('partition', default_value='vigil_rough_terrain'),
        DeclareLaunchArgument('vision_sensors', default_value='false'),
        DeclareLaunchArgument('truth_isolation', default_value='false',
                              description='true: bridge simulator truth only under /eval/ (GPS-denied evaluation)'),
        # HD display camera (only with vision_sensors:=true). 4:3 keeps it aligned with depth.
        DeclareLaunchArgument('camera_width', default_value='1920'),
        DeclareLaunchArgument('camera_height', default_value='1440'),
        DeclareLaunchArgument('camera_rate', default_value='10'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join([
            str(share / 'models'), str(share.parent),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')])),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.pathsep.join([
            get_package_prefix('gz_ros2_control') + '/lib',
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')])),
        sim, gui,
        RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo closed'))])),
        OpaqueFunction(function=state_publisher),
        OpaqueFunction(function=bridge),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
        spawn,
        Node(package='vigil_rough_terrain', executable='drive.py',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
