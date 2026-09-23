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
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'vigil_rock_terrain', '-name', 'agri_ugv',
                            '-topic', 'robot_description',
                            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                            '-z', LaunchConfiguration('spawn_z'), '-Y', '0'])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'wheel_controller',
                                  '--controller-manager-timeout', '120',
                                  '--switch-timeout', '90', '--service-call-timeout', '100'])
    def spawned(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Rover spawn failed.'),
                    EmitEvent(event=Shutdown(reason='Rover spawn failed'))]
        return [controllers]
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
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
        spawn,
        Node(package='vigil_rough_terrain', executable='drive.py',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
