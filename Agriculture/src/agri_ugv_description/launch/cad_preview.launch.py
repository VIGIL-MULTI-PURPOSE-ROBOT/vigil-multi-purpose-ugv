"""Inspect original assembled geometry; does not start physics or motion."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('agri_ugv_description'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share / 'urdf/cad_preview.urdf').read_text()}],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='cad_preview',
             arguments=['-d', str(share / 'rviz/cad_preview.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
