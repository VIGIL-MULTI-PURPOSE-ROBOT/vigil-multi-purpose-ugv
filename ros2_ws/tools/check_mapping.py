#!/usr/bin/python3
import time,json
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2,Image
from tf2_ros import Buffer,TransformListener
from cv_bridge import CvBridge
import cv2
rclpy.init();n=rclpy.create_node('mapping_evidence');data={};buf=Buffer();listener=TransformListener(buf,n)
def grid(m):
    a=np.array(m.data).reshape(m.info.height,m.info.width)
    data['map']={'size':[m.info.width,m.info.height],'resolution':m.info.resolution,'known':int((a>=0).sum()),'free':int((a==0).sum()),'occupied':int((a>50).sum())}
    cv2.imwrite('reports/slam_grid.png',np.flipud(np.where(a<0,127,np.where(a>50,0,255))).astype(np.uint8))
def cloud(m):data['cloud_points']=m.width*m.height
def camera(m):
    if 'camera' not in data:
        img=CvBridge().imgmsg_to_cv2(m,'bgr8')
        if float(img.std())>2:
            cv2.imwrite('reports/live_camera.png',img);data['camera']=True
n.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(PointCloud2,'/rtabmap/cloud_map',cloud,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Image,'/camera/image',camera,10)
end=time.monotonic()+40
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
    if data.get('camera') and data.get('cloud_points',0)>100 and data.get('map',{}).get('free',0)>100 and buf.can_transform('map','base_link',rclpy.time.Time()):break
data['map_tf']=bool(buf.can_transform('map','base_link',rclpy.time.Time()))
open('reports/stage6_mapping.json','w').write(json.dumps(data,indent=2));print(json.dumps(data,indent=2))
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if data.get('cloud_points',0)>100 and data.get('map_tf') else 1)
