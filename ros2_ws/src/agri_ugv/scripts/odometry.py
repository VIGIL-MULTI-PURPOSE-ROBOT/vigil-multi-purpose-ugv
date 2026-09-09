#!/usr/bin/python3
"""Six measured wheel rates; rear wheel poses never enter estimation."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,Imu
from nav_msgs.msg import Odometry

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('six_encoder_odometry')
        self.create_subscription(JointState,'encoders',self.encoders,qos_profile_sensor_data)
        self.create_subscription(Imu,'imu/data',self.imu,qos_profile_sensor_data)
        self.pub=self.create_publisher(Odometry,'wheel/odometry',10)
        self.ipub=self.create_publisher(Imu,'imu/with_covariance',10)
        self.last=None;self.x=self.y=self.yaw=0.;self.gyro=0.
    def imu(self,m):
        self.gyro=m.angular_velocity.z
        m.orientation_covariance=[.003,0.,0.,0.,.003,0.,0.,0.,.01]
        m.angular_velocity_covariance=[.0004,0.,0.,0.,.0004,0.,0.,0.,.0004]
        m.linear_acceleration_covariance=[.04,0.,0.,0.,.04,0.,0.,0.,.04]
        self.ipub.publish(m)
    def encoders(self,m):
        rates=dict(zip(m.name,m.velocity));names=[s+str(i)+'_joint' for s in ['L','R'] for i in [1,2,3]]
        if not all(k in rates and math.isfinite(rates[k]) for k in names):return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.last is None:self.last=stamp;return
        dt=stamp-self.last
        if dt<0:self.last=stamp;return
        if dt<.02:return
        self.last=stamp
        if dt>.5:return
        left=np.array([rates['L'+str(i)+'_joint']*r for i,r in [(1,.23463),(2,.17655),(3,.17655)]])
        right=np.array([rates['R'+str(i)+'_joint']*r for i,r in [(1,.23463),(2,.17655),(3,.17655)]])
        v=float((np.median(left)+np.median(right))/2);w=float((np.median(right)-np.median(left))/1.3)
        slip=float(np.var(left)+np.var(right)+(w-self.gyro)**2)
        self.x+=v*math.cos(self.yaw+w*dt/2)*dt;self.y+=v*math.sin(self.yaw+w*dt/2)*dt;self.yaw+=w*dt
        out=Odometry();out.header.stamp=m.header.stamp;out.header.frame_id='odom';out.child_frame_id='base_footprint'
        out.pose.pose.position.x=self.x;out.pose.pose.position.y=self.y
        out.pose.pose.orientation.z=math.sin(self.yaw/2);out.pose.pose.orientation.w=math.cos(self.yaw/2)
        out.twist.twist.linear.x=v;out.twist.twist.angular.z=w
        out.pose.covariance=np.diag([1.,1.,1e6,1e6,1e6,.5]).flatten().tolist()
        out.twist.covariance=np.diag([.02+slip,.08,1e6,1e6,1e6,.03+4*slip]).flatten().tolist()
        self.pub.publish(out)
def main():
    rclpy.init();n=WheelOdometry()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
