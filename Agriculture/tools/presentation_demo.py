#!/usr/bin/python3
"""Short staged drive + Dijkstra return demo; never claims field completion."""
import time,json,math
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('presentation_demo');data={};history=[]
def status(m):
 d=json.loads(m.data);history.append(d);data.update(d)
n.create_subscription(String,'exploration/status',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));pub=n.create_publisher(String,'exploration/command',10);client=ActionClient(n,NavigateToPose,'navigate_to_pose')
end=time.monotonic()+60
while time.monotonic()<end and ('pose' not in data or not client.server_is_ready()):rclpy.spin_once(n,timeout_sec=.1)
if 'pose' not in data:raise SystemExit('No recorded start. Launch a fresh run with explore:=false.')
if data.get('message')!='Recorded start pose':raise SystemExit('Use a fresh paused simulation for this staged demo.')
x,y,yaw=data['pose'];g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=x+3*math.cos(yaw)-.5*math.sin(yaw);g.pose.pose.position.y=y+3*math.sin(yaw)+.5*math.cos(yaw);g.pose.pose.orientation.z=math.sin((yaw+.15)/2);g.pose.pose.orientation.w=math.cos((yaw+.15)/2)
print('Driving a short curved route with obstacle checks enabled.',flush=True)
f=client.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result()
if not h or not h.accepted:raise SystemExit('Navigation goal was rejected.')
f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=150)
if not f.done():
 c=h.cancel_goal_async();rclpy.spin_until_future_complete(n,c,timeout_sec=5);raise SystemExit('Drive timed out; completion was not claimed.')
if f.result().status!=4:raise SystemExit('Drive did not succeed; inspect Nav2 and perception before retrying.')
print('Drive succeeded. Requesting Dijkstra-guided return.',flush=True);pub.publish(String(data='return'));end=time.monotonic()+150
while time.monotonic()<end and data.get('state')!='arrived':rclpy.spin_once(n,timeout_sec=.1)
ok=data.get('state')=='arrived';open('reports/presentation_demo.json','w').write(json.dumps({'pass':ok,'mission_events':history},indent=2));print('PASS: curved drive and return' if ok else 'Return not completed; inspect mission status.',flush=True)
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if ok else 1)
