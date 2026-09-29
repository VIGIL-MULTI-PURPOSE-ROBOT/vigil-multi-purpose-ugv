#!/usr/bin/python3
import rclpy,time,json
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from rcl_interfaces.msg import Log
rclpy.init();n=rclpy.create_node('motion_diagnostic');d={};logs=[]
for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel','/drive/limited']:
 n.create_subscription(Twist,topic,lambda m,t=topic:d.update({t:[m.linear.x,m.angular.z]}),10)
n.create_subscription(Odometry,'/odom',lambda m:d.update(odom=[m.pose.pose.position.x,m.pose.pose.position.y]),10)
n.create_subscription(CollisionMonitorState,'/collision_monitor_state',lambda m:d.update(collision=str(m)),10)
n.create_subscription(Log,'/rosout',lambda m:logs.append(m.msg) if m.level>=30 else None,100)
t=time.monotonic()+8
while time.monotonic()<t:rclpy.spin_once(n,timeout_sec=.1)
print(d);print('\n'.join(list(dict.fromkeys(logs))[-10:]))
