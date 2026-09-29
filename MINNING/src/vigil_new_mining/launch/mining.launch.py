from pathlib import Path
import os,json,time
import xacro
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,SetEnvironmentVariable,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('vigil_new_mining'))
    root=Path(os.environ.get('VIGIL_NEW_MINING_HOME',str(Path.home()/'Documents/vigil/MINNING')))
    (root/'log/runtime').mkdir(parents=True,exist_ok=True)
    s=json.loads((p/'config/mission.json').read_text())['spawn']
    desc=xacro.process_file(str(p/'urdf/agri_ugv.urdf.xacro'),mappings={'simulation':'true','controllers_file':str(p/'config/controllers.yaml')}).toxml()
    sim=ExecuteProcess(cmd=['gz','sim','-s','-r','--headless-rendering','-v','2',str(p/'worlds/mine.sdf')],output='screen')
    spawn=Node(package='ros_gz_sim',executable='create',arguments=['-world','vigil_new_mining','-name','agri_ugv','-topic','robot_description','-x',str(s[0]),'-y',str(s[1]),'-z',str(s[2]),'-Y',str(s[3])],output='screen')
    controllers=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','wheel_controller','--controller-manager-timeout','120','--switch-timeout','90','--service-call-timeout','90'],output='screen')
    common={'use_sim_time':True,'frame_id':'base_footprint','approx_sync':True,'sync_queue_size':30,'qos':2}
    remap=[('rgb/image','/camera/image'),('depth/image','/camera/depth_image'),('rgb/camera_info','/camera/camera_info'),('imu','/imu/data')]
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        SetEnvironmentVariable('GZ_PARTITION','vigil_new_mining'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(p/'models')+os.pathsep+str(p.parent)),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',get_package_prefix('gz_ros2_control')+'/lib'),
        sim,ExecuteProcess(cmd=['gz','sim','-g'],condition=IfCondition(LaunchConfiguration('gui')),output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':desc,'use_sim_time':True}],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(p/'config/bridge.yaml'),'use_sim_time':True}],output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=[controllers])),spawn,
        Node(package='vigil_new_mining',executable='drive.py',parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='rtabmap_odom',executable='rgbd_odometry',name='rgbd_odometry',parameters=[common,{'odom_frame_id':'odom','publish_tf':False,'wait_imu_to_init':True,'Odom/Strategy':'0','Vis/MinInliers':'12'}],remappings=remap+[('odom','/visual/odom'),('odom_info','/visual/odom_info')],output='screen'),
        Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[str(p/'config/ekf.yaml')],remappings=[('odometry/filtered','/odom')],output='screen'),
        Node(package='rtabmap_slam',executable='rtabmap',name='rtabmap',parameters=[common,{'subscribe_depth':True,'subscribe_rgb':True,'subscribe_odom_info':False,'database_path':str(root/'log'/('mining_'+time.strftime('%Y%m%d_%H%M%S')+'.db')),'Reg/Strategy':'0','Grid/FromDepth':'true','Grid/3D':'true','Grid/RayTracing':'true','Grid/CellSize':'0.15','Grid/ClusterRadius':'0.35','Grid/MinClusterSize':'3','Grid/DepthDecimation':'2','GridGlobal/FootprintRadius':'0.8','Grid/MaxGroundHeight':'0.0','Grid/MaxObstacleHeight':'1.8','Grid/MaxGroundAngle':'25','Grid/RangeMax':'12','RGBD/LinearUpdate':'0.2','RGBD/AngularUpdate':'0.15','Vis/MinInliers':'12'}],remappings=remap+[('grid_map','/map')],output='screen'),
        Node(package='vigil_new_mining',executable='mining_runtime.py',parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(p/'rviz/mining.rviz')],parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')),output='screen')])
