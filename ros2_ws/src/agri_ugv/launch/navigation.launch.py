from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('agri_ugv'));config=str(p/'config/navigation.yaml')
    specs=[('nav2_controller','controller_server','controller_server'),('nav2_planner','planner_server','planner_server'),('nav2_smoother','smoother_server','smoother_server'),('nav2_behaviors','behavior_server','behavior_server'),('nav2_bt_navigator','bt_navigator','bt_navigator'),('nav2_velocity_smoother','velocity_smoother','velocity_smoother'),('nav2_collision_monitor','collision_monitor','collision_monitor')]
    actions=[]
    for pkg,exe,name in specs:
        extra={}
        if name=='bt_navigator':extra={'default_nav_to_pose_bt_xml':str(p/'behavior_trees/navigate.xml'),'default_nav_through_poses_bt_xml':str(p/'behavior_trees/through_poses.xml')}
        remap=[('cmd_vel','cmd_vel_nav')] if name in ['controller_server','behavior_server','velocity_smoother'] else []
        actions.append(Node(package=pkg,executable=exe,name=name,parameters=[config,extra],remappings=remap,output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':10.,'node_names':[s[2] for s in specs]}],output='screen'))
    return LaunchDescription(actions)
