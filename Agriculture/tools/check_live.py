#!/usr/bin/python3
"""Bounded live sensor and motion evidence, with JSON output."""
import argparse,json,time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,Imu,JointState,PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation

p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=25);p.add_argument('--output',default='/tmp/agri_live.json');p.add_argument('--drive',action='store_true');a=p.parse_args()
rclpy.init();n=rclpy.create_node('live_evidence');data={};counts={};poses=[]
def save(name,result):data[name]=result;counts[name]=counts.get(name,0)+1
def cloud(m):
    v=point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)
    save('lidar',{'valid':int(np.isfinite(v).all(axis=1).sum()),'frame':m.header.frame_id,'total':m.width*m.height})
def color(m):save('rgb',{'shape':[m.width,m.height],'std':float(np.frombuffer(m.data,dtype=np.uint8).std()),'frame':m.header.frame_id})
def depth(m):
    v=np.frombuffer(m.data,dtype='<f4');v=v[np.isfinite(v)&(v>0)]
    save('depth',{'finite_positive':len(v),'median':float(np.median(v)) if len(v) else None})
def imu(m):save('imu',{'q':[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],'accel':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]})
def enc(m):save('encoders',{'names':list(m.name),'velocities':list(m.velocity)})
def pose(m):
    q=m.pose.pose.orientation;v=m.pose.pose.position
    rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True).tolist()
    poses.append([v.x,v.y,v.z]);save('ground_truth',{'position':poses[-1],'rpy_deg':rpy})
def links(m):save('link_poses',[(t.header.frame_id,t.child_frame_id) for t in m.transforms])
for typ,topic,cb in [(PointCloud2,'/lidar/points',cloud),(Image,'/camera/image',color),(Image,'/camera/depth_image',depth),(Imu,'/imu/data',imu),(JointState,'/encoders',enc),(Odometry,'/sim/ground_truth',pose),(TFMessage,'/sim/link_poses',links)]:n.create_subscription(typ,topic,cb,qos_profile_sensor_data)
pub=n.create_publisher(Twist,'/cmd_vel',10)
start=time.monotonic();end=start+a.seconds
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.02)
    if a.drive:
        m=Twist()
        if 2<time.monotonic()-start<a.seconds-3:m.linear.x=.25;m.angular.z=.04
        pub.publish(m)
if a.drive:pub.publish(Twist())
data['counts']=counts
if poses:data['motion_delta']=(np.array(poses[-1])-poses[0]).tolist()
open(a.output,'w').write(json.dumps(data,indent=2));print(json.dumps(data,indent=2))
ok=all(k in data for k in ['lidar','rgb','depth','imu','encoders','ground_truth'])
if ok:
    ok=(data['lidar']['valid']>100 and data['rgb']['std']>1 and data['depth']['finite_positive']>100 and len(data['encoders']['names'])==6 and np.isfinite(data['imu']['q']).all())
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if ok else 1)
