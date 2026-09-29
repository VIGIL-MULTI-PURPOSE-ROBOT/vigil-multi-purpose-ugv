from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    p=Path(get_package_share_directory('agri_ugv'));out=Path.cwd()/'run';out.mkdir(exist_ok=True)
    mapper=Node(package='rtabmap_slam',executable='rtabmap',name='rtabmap',output='screen',arguments=['--ros-args','--log-level','warn'],
        parameters=[str(p/'config/slam.yaml'),{'database_path':str(out/('field_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.db'))}],
        remappings=[('rgb/image','/camera/image'),('depth/image','/camera/depth_image'),('rgb/camera_info','/camera/camera_info'),('scan_cloud','/lidar/points'),('odom','/odom'),('imu','/imu/with_covariance'),('grid_map','/map'),('cloud_map','/rtabmap/cloud_map')])
    return LaunchDescription([RegisterEventHandler(OnProcessExit(target_action=mapper,on_exit=[EmitEvent(event=Shutdown(reason='SLAM process exited; stopping navigation and simulation'))])),mapper])
