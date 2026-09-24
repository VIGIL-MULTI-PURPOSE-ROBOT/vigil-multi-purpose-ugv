"""Terrain/cliff perception + A-to-B navigation (ported unchanged from vigil_rough_terrain).

  ros2 launch vigil_sar navigation.launch.py [goal_x:=80 goal_y:=40]
Nodes: terrain_mapper (RGB/depth/segmentation -> terrain classes, cliffs, overlay),
       obstacle_tracker (depth + segmentation + pose -> tracked people / vehicles / moving obstacles),
       cliff_navigator (footprint-aware A*, replanning, hill climbing, SAR scan hold, dynamic-obstacle
       prediction and avoidance).
"""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import PKG, params  # noqa: E402


def setup(context):
    p = params(context)
    return [Node(package=PKG, executable='terrain_mapper.py', name='terrain_mapper', parameters=p, output='screen'),
            Node(package=PKG, executable='obstacle_tracker.py', name='obstacle_tracker', parameters=p, output='screen'),
            Node(package=PKG, executable='cliff_navigator.py', name='cliff_navigator', parameters=p, output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=''),
                              DeclareLaunchArgument('goal_x', default_value=''),
                              DeclareLaunchArgument('goal_y', default_value=''),
                              OpaqueFunction(function=setup)])
