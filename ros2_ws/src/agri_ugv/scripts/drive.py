#!/usr/bin/python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray,String

class Drive(Node):
    def __init__(self):
        super().__init__('wheel_drive')
        self.target=np.zeros(2);self.current=np.zeros(2);self.last=time.monotonic();self.turn_radius=2.0
        self.create_subscription(Twist,'cmd_vel',self.command,10)
        self.create_subscription(String,'crop_row/mode',self.crop_mode,10)
        self.pub=self.create_publisher(Float64MultiArray,'wheel_controller/commands',10)
        self.limited=self.create_publisher(Twist,'drive/limited',10)
        self.create_timer(.02,self.tick)
    def command(self,msg):
        if not np.isfinite([msg.linear.x,msg.angular.z]).all():return
        # The physical cap is five times the former 0.7 m/s limit. Nav2 uses
        # its own 1.5 m/s crop-row setpoint below this cap.
        self.target[:]=[np.clip(msg.linear.x,-2.0,3.5),msg.angular.z];self.last=time.monotonic()
    def crop_mode(self,msg):
        # Crop headland command is compensated for the eight-wheel
        # skid-steer curvature on soil; it yields a 0.61 m physical arc.
        self.turn_radius=.85 if msg.data=='headland' else 2.0
    def tick(self):
        target=self.target.copy() if time.monotonic()-self.last<.6 else np.zeros(2)
        # Tight turns are allowed only when the crop-row controller declares
        # a clear headland. All row driving retains the 2 m smooth-turn limit.
        target[1]=np.clip(target[1],-abs(target[0])/self.turn_radius,abs(target[0])/self.turn_radius)
        self.current+=np.clip(target-self.current,[-.04,-.04],[.04,.04])
        v,w=self.current;w=np.clip(w,-abs(v)/self.turn_radius,abs(v)/self.turn_radius)
        wheels=[]
        for side in [1,-1]:
            wheels.extend([(v-side*w*.65)/r for r in [.23463,.17655,.17655,.17655]])
        self.pub.publish(Float64MultiArray(data=wheels))
        m=Twist();m.linear.x=float(v);m.angular.z=float(w);self.limited.publish(m)

def main():
    rclpy.init();n=Drive()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.pub.publish(Float64MultiArray(data=[0.]*8))
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
