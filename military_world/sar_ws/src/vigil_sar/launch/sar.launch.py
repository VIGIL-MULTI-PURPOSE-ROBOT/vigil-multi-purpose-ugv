"""Search and rescue layer: thermal human detection -> tracking -> SAR manager.

  ros2 launch vigil_sar sar.launch.py
SAR stays OFF until the dashboard switch (or: ros2 topic pub --once /sar/command std_msgs/String "data: START").
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
    return [Node(package=PKG, executable='thermal_human_detector.py', name='thermal_human_detector',
                 parameters=p, output='screen'),
            Node(package=PKG, executable='human_tracker.py', name='human_tracker', parameters=p, output='screen'),
            Node(package=PKG, executable='sar_manager.py', name='sar_manager', parameters=p, output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=''),
                              OpaqueFunction(function=setup)])
