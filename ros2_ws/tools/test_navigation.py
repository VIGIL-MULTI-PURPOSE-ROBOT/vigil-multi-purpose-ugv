#!/usr/bin/python3
import argparse,json,time,math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--x',type=float,default=4.);p.add_argument('--y',type=float,default=1.5);p.add_argument('--yaw',type=float,default=0.);p.add_argument('--seconds',type=float,default=180.);p.add_argument('--output',default='reports/stage8_navigation.json');a=p.parse_args()
rclpy.init();n=rclpy.create_node('navigation_test');client=ActionClient(n,NavigateToPose,'navigate_to_pose');b=Buffer();l=TransformListener(b,n)
data={'commands':[],'truth':[],'plan_count':0};start=time.monotonic()
def cmd(m):data['commands'].append([m.linear.x,m.angular.z])
def truth(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 data['truth'].append([p.x,p.y,p.z,*Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')])
def plan(m):data['plan_count']+=1;data['last_plan']=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
n.create_subscription(Twist,'drive/limited',cmd,10);n.create_subscription(Odometry,'sim/ground_truth',truth,10);n.create_subscription(Path,'plan',plan,10)
if not client.wait_for_server(timeout_sec=20):raise RuntimeError('Nav2 action server unavailable')
g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=a.x;g.pose.pose.position.y=a.y;g.pose.pose.orientation.z=math.sin(a.yaw/2);g.pose.pose.orientation.w=math.cos(a.yaw/2)
future=client.send_goal_async(g);rclpy.spin_until_future_complete(n,future,timeout_sec=10)
handle=future.result()
if not handle or not handle.accepted:raise RuntimeError('Navigation goal rejected')
result=handle.get_result_async();next_report=start+20
while rclpy.ok() and not result.done() and time.monotonic()-start<a.seconds:
 rclpy.spin_once(n,timeout_sec=.1)
 if time.monotonic()>next_report:
  print('Navigation in progress; plans=',data['plan_count'],flush=True);next_report+=20
if not result.done():
 c=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,c,timeout_sec=5);data['status']='timeout'
else:data['status']=result.result().status;data['result']=str(result.result().result)
commands=np.array(data['commands']);data['no_spin']=bool(len(commands) and np.all(abs(commands[:,1])<=abs(commands[:,0])/2+1e-5))
try:
 t=b.lookup_transform('map','base_footprint',rclpy.time.Time());data['position_error_m']=math.hypot(t.transform.translation.x-a.x,t.transform.translation.y-a.y)
except Exception:pass
open(a.output,'w').write(json.dumps(data,indent=2));print({k:v for k,v in data.items() if k not in ['commands','truth','last_plan']})
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if data['status']==4 and data['no_spin'] else 1)
