#!/usr/bin/python3
"""Deliver one ROS demo command after discovery, then exit."""
import argparse,time,rclpy
from std_msgs.msg import String,Float32
p=argparse.ArgumentParser();p.add_argument('command',choices=['pause','start','return','day','night','auto-light','level','terrain','hybrid']);a=p.parse_args()
if a.command in ['pause','start','return']:topic='exploration/command';cls=String;msg=String(data=a.command)
elif a.command in ['level','terrain','hybrid']:topic='suspension/mode';cls=String;msg=String(data=a.command)
else:topic='simulation/ambient_level';cls=Float32;msg=Float32(data={'day':1.,'night':.03,'auto-light':-1.}[a.command])
rclpy.init();n=rclpy.create_node('presentation_command');pub=n.create_publisher(cls,topic,10);end=time.monotonic()+10
while not pub.get_subscription_count() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
if not pub.get_subscription_count():raise SystemExit('No subscriber: start the simulation and use the same ROS_DOMAIN_ID.')
pub.publish(msg);end=time.monotonic()+1
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
print('Sent',a.command,'to',topic);n.destroy_node();rclpy.shutdown()
