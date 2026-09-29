"""Physical A-to-B demonstration; outcomes come from measured motion."""
import importlib.util
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('vigil_rough_terrain'))
    spec=importlib.util.spec_from_file_location('four_model',share/'scripts/make_four_wheel.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    f=tempfile.NamedTemporaryFile(mode='w',suffix='.sdf',prefix='vigil_four_',delete=False)
    f.write(module.generate());f.close()
    bridge=['/comparison/four/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry']
    bridge += [f'/comparison/four/{s}{i}/velocity@std_msgs/msg/Float64]gz.msgs.Double' for s in 'LR' for i in (1,3)]
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('output',default_value='/tmp/vigil-comparison.json'),
        SetEnvironmentVariable('GZ_PARTITION','vigil_rough_terrain'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/terrain_sim.launch.py')),launch_arguments={'gui':LaunchConfiguration('gui')}.items()),
        Node(package='ros_gz_sim',executable='create',arguments=['-world','vigil_rock_terrain','-name','four_wheel_rover','-file',f.name,'-x','-5.4','-y','-4.536','-z','1.2'],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=bridge,parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='vigil_rough_terrain',executable='comparison_driver.py',parameters=[{'use_sim_time':True,'output':LaunchConfiguration('output')}],output='screen'),
    ])
