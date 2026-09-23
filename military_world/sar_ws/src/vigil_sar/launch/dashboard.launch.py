"""SAR control station: http://localhost:8080 (port: dashboard.port)."""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import PKG, params  # noqa: E402


def setup(context):
    return [Node(package=PKG, executable='ugv_dashboard.py', name='ugv_dashboard', parameters=params(context),
                 output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=''),
                              OpaqueFunction(function=setup)])
