"""Sequential bringup; stage selects the highest enabled build step."""
import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    stage=int(LaunchConfiguration('stage').perform(context))
    headless=LaunchConfiguration('headless').perform(context)=='true'
    p=Path(get_package_share_directory('agri_ugv'))
    d=Path(get_package_share_directory('agri_ugv_description'))
    description=xacro.process_file(str(d/'urdf/agri_ugv.urdf.xacro'),mappings={'simulation':'true'}).toxml()
    def node(package,executable,**kw):
        return Node(package=package,executable=executable,output='screen',**kw)
    sim_args=['gz','sim','-r','-v','2','--gui-config',str(p/'config/gazebo_gui.config')]+(['-s','--headless-rendering'] if headless else [])+[str(p/'worlds/field.world')]
    # Crop row 1 is y=-13.418 m. The wheel centres sit at +/-0.50 m, placing
    # each tyre in an adjacent empty 1.22 m-pitch aisle while the chassis
    # straddles the plants.  x=-14.5 m is before the crop-row endpoint.
    spawn=node('ros_gz_sim','create',arguments=['-name','agri_ugv','-topic','robot_description','-x','-14.5','-y','-13.418','-z','0.10','-Y','0'])
    controllers=node('controller_manager','spawner',arguments=['joint_state_broadcaster','wheel_controller','suspension_controller','--controller-manager-timeout','120','--switch-timeout','90','--service-call-timeout','100'])
    actions=[
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(p/'models')+':'+str(d.parent)+':'+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',get_package_prefix('gz_ros2_control')+'/lib:'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
        ExecuteProcess(cmd=sim_args,output='screen'),
        node('robot_state_publisher','robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}]),
        node('ros_gz_bridge','parameter_bridge',parameters=[{'config_file':str(p/'config/bridge.yaml'),'use_sim_time':True}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=[controllers])),spawn,
        node('agri_ugv','drive.py',parameters=[{'use_sim_time':True}]),
    ]
    if stage>=4:
        actions.append(node('agri_ugv','visualization.py',parameters=[{'use_sim_time':True}]))
        if LaunchConfiguration('rviz').perform(context)=='true':
            actions.append(node('rviz2','rviz2',respawn=True,respawn_delay=3.,arguments=['-d',str(p/'config/field.rviz'),'-f',('map' if stage>=6 else 'odom' if stage>=5 else 'base_footprint')],parameters=[{'use_sim_time':True}]))
    if stage>=5:
        actions.extend([node('agri_ugv','odometry.py',parameters=[{'use_sim_time':True}]),node('robot_localization','ekf_node',name='ekf_filter_node',parameters=[str(p/'config/ekf.yaml')],remappings=[('odometry/filtered','odom')])])
    if stage>=6:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch/slam.launch.py'))))
    if stage>=7:
        actions.append(node('agri_ugv','perception.py',parameters=[{'use_sim_time':True}]))
    if stage>=8:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch/navigation.launch.py'))))
        actions.append(node('agri_ugv','environment.py',parameters=[{'use_sim_time':True,'lighting_enabled':stage>=10}]))
    if stage>=9:
        if LaunchConfiguration('row_mission').perform(context)=='true':
            actions.append(node('agri_ugv','crop_row_driver.py',parameters=[{'use_sim_time':True,'row_count':int(LaunchConfiguration('row_count').perform(context))}]))
        else:
            actions.append(node('agri_ugv','exploration.py',parameters=[{'use_sim_time':True,'autostart':LaunchConfiguration('explore').perform(context)=='true','visit_b':True}]))
    if stage>=10:
        actions.append(node('agri_ugv','lighting.py',parameters=[{'use_sim_time':True}]))
    if stage>=11:
        actions.append(node('agri_ugv','suspension.py',parameters=[{'use_sim_time':True}]))
    if stage>=6:
        guard=node('agri_ugv','resource_guard.py')
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=guard,on_exit=[EmitEvent(event=Shutdown(reason='Simulation resource guard exited'))])),guard])
    return actions

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('stage',default_value='12'),DeclareLaunchArgument('headless',default_value='false'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('explore',default_value='true'),DeclareLaunchArgument('row_mission',default_value='true'),DeclareLaunchArgument('row_count',default_value='23'),OpaqueFunction(function=setup)])
