from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    p=Path(get_package_share_directory('agri_ugv_description'))
    description=xacro.process_file(str(p/'urdf/agri_ugv.urdf.xacro')).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[{'zeros.L_rocker':-0.06283185307179587,'zeros.R_rocker':-0.06283185307179587}]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(p/'rviz/cad_preview.rviz')]),
    ])
