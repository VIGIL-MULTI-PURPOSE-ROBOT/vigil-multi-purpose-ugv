#!/usr/bin/python3
"""Measured, low-speed waypoint comparison. Never writes poses or joint positions."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# Course scaled x0.72 with the terrain (was 25 x 25 m, now 18 x 18 m).
ROUTE=[[-4.896,-3.024],[-4.32,-2.16],[-3.024,-2.16]]

def angles(q):
    return (math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

class Demo(Node):
    def __init__(self):
        super().__init__('rover_comparison')
        self.output=self.declare_parameter('output','/tmp/vigil-comparison.json').value
        self.data={k:dict(status='waiting',waypoint=0,samples=[],max_tilt_deg=0.,last_progress=0.,best_distance=1e9) for k in ('eight','four')}
        self.pose={};self.received={};self.start=None;self.done=False
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.wheels=[self.create_publisher(Float64,f'/comparison/four/{s}{i}/velocity',10) for s in 'LR' for i in (1,3)]
        for k,t in [('eight','/sim/ground_truth'),('four','/comparison/four/odom')]:
            self.create_subscription(Odometry,t,lambda m,k=k:self.odometry(k,m),qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
    def odometry(self,k,msg):
        p=msg.pose.pose.position;self.pose[k]=(p.x,p.y,p.z,*angles(msg.pose.pose.orientation));self.received[k]=self.get_clock().now().nanoseconds*1e-9
    def command(self,k,v,w):
        if k=='eight':
            m=Twist();m.linear.x=v;m.angular.z=w;self.pub.publish(m)
        else:
            for pub,side,r in zip(self.wheels,[1,1,-1,-1],[.23463,.17655,.23463,.17655]):pub.publish(Float64(data=(v-side*w*.65)/r))
    def save(self):
        p=Path(self.output);p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(json.dumps(dict(route=ROUTE,goal_tolerance_m=.3,robots=self.data),indent=2))
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.start is None:
            if len(self.pose)<2:return
            self.start=now
            self.get_logger().info('Both rovers spawned. Settling, then following the same A-to-B waypoints.')
        elapsed=now-self.start
        for k,d in self.data.items():
            if d['status'] in ('reached','flipped','stuck','timeout','telemetry_lost'):
                self.command(k,0.,0.);continue
            if now-self.received.get(k,-1e9)>.5:
                d['status']='telemetry_lost';self.command(k,0.,0.);continue
            x,y,z,roll,pitch,yaw=self.pose[k]
            tilt=math.degrees(max(abs(roll),abs(pitch)))
            d['max_tilt_deg']=max(d['max_tilt_deg'],tilt)
            d['samples'].append([round(elapsed,3),x,y,z,roll,pitch,yaw])
            # A trailing start keeps two physical robots from colliding at point A.
            delay=5. if k=='eight' else 23.
            if elapsed<delay:self.command(k,0.,0.);continue
            if tilt>75:
                d['status']='flipped';self.command(k,0.,0.);self.get_logger().info(f'{k}: physically tipped (>75 degrees)');continue
            if elapsed>180:
                d['status']='timeout';self.command(k,0.,0.);continue
            target=[-3.384,-1.08] if d['waypoint']==len(ROUTE) else ROUTE[d['waypoint']]
            dx,dy=target[0]-x,target[1]-y
            distance=math.hypot(dx,dy)
            if distance<.3:
                if d['waypoint']==len(ROUTE):
                    d['status']='reached';d['parked_clear']=True;self.command(k,0.,0.);continue
                d['waypoint']+=1;d['best_distance']=1e9;d['last_progress']=elapsed
                if d['waypoint']==len(ROUTE):
                    d['arrival_s']=elapsed-delay
                    self.get_logger().info(f'{k}: reached B; driving clear to prevent blocking the following rover')
                    if k=='four':
                        d['status']='reached';self.command(k,0.,0.);continue
                target=[-3.384,-1.08] if d['waypoint']==len(ROUTE) else ROUTE[d['waypoint']]
                dx,dy=target[0]-x,target[1]-y;distance=math.hypot(dx,dy)
            if distance<d['best_distance']-.05:d['best_distance']=distance;d['last_progress']=elapsed
            if elapsed-d['last_progress']>25:
                d['status']='stuck';self.command(k,0.,0.);continue
            error=math.atan2(math.sin(math.atan2(dy,dx)-yaw),math.cos(math.atan2(dy,dx)-yaw))
            # Identical speed/heading policy, with a ramp over the first 2 seconds.
            ramp=min(1.,(elapsed-delay)/2.)
            self.command(k,.18*ramp if abs(error)<.65 else 0.,max(-.35,min(.35,error))*ramp)
            d['status']='clearing_goal' if d['waypoint']==len(ROUTE) else 'driving'
        if int(elapsed*20)%100==0:
            self.save()
            self.get_logger().info(' | '.join(f"{k}: {d['status']} waypoint {d['waypoint']}/{len(ROUTE)}" for k,d in self.data.items()))
        if all(d['status'] in ('reached','flipped','stuck','timeout','telemetry_lost') for d in self.data.values()) and not self.done:
            self.done=True;self.save();self.get_logger().info('Comparison complete: '+str({k:d['status'] for k,d in self.data.items()}))

def main():
    rclpy.init();node=Demo()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            for k in node.data:node.command(k,0.,0.)
        node.save();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
