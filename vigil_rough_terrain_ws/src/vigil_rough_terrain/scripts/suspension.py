#!/usr/bin/python3
"""Bounded IMU -> arm angle -> joint effort cascade; hybrid by default."""
import json,math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,Imu
from std_msgs.msg import String,Float64MultiArray
from scipy.spatial.transform import Rotation
from hydraulics import cylinder_from_notes

class Suspension(Node):
    def __init__(self):
        super().__init__('active_suspension');self.mode='hybrid';self.q=None;self.velocity=np.zeros(2);self.orientation=None;self.stamp={}
        self.standing=self.declare_parameter('standing_angle',-0.06283185307179587).value
        self.fixed=self.declare_parameter('cylinder_fixed_anchor',[-.10,0.,.25]).value;self.arm=self.declare_parameter('cylinder_arm_anchor',[.45,0.,.05]).value
        self.max_effort=self.declare_parameter('max_effort',800.).value;self.angle=np.full(2,self.standing);self.integral=np.zeros(2);self.outer=np.zeros(2);self.effort=np.zeros(2);self.last=None
        self.pub=self.create_publisher(Float64MultiArray,'suspension_controller/commands',10);self.debug=self.create_publisher(String,'suspension/debug',10)
        self.create_subscription(JointState,'joint_states',self.joints,10);self.create_subscription(Imu,'imu/with_covariance',self.imu,qos_profile_sensor_data);self.create_subscription(String,'suspension/mode',self.switch,10);self.create_subscription(String,'levelling_node/set_mode',self.switch,10);self.create_timer(.02,self.tick)
    def joints(self,msg):
        names=['L_rocker','R_rocker']
        if not all(k in msg.name for k in names):return
        ii=[msg.name.index(k) for k in names];self.q=np.asarray([msg.position[k] for k in ii]);self.velocity=np.asarray([msg.velocity[k] if k<len(msg.velocity) else 0. for k in ii]);self.stamp['joints']=self.get_clock().now().nanoseconds*1e-9
    def imu(self,msg):
        q=msg.orientation;a=[q.x,q.y,q.z,q.w]
        if not np.isfinite(a).all() or np.linalg.norm(a)<.5:return
        self.orientation=Rotation.from_quat(a).as_euler('xyz')[:2];self.stamp['imu']=self.get_clock().now().nanoseconds*1e-9
    def switch(self,msg):
        if msg.data not in ['level','terrain','hybrid']:self.get_logger().warning('Mode must be level, terrain, or hybrid');return
        self.mode=msg.data;self.integral*=0;self.outer*=0;self.get_logger().info('Suspension mode: '+self.mode)
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9;dt=.02 if self.last is None else np.clip(now-self.last,.001,.1);self.last=now
        if self.q is None or self.orientation is None or any(now-self.stamp.get(k,-100)>.5 for k in ['imu','joints']):
            self.pub.publish(Float64MultiArray(data=[0.,0.]));return
        roll,pitch=self.orientation;err=np.where(abs(self.orientation)<math.radians(.5),0.,self.orientation)
        self.outer=np.clip(self.outer+err*dt,-.15,.15)
        common=.8*err[1]+.08*self.outer[1];differential=.7*err[0]+.06*self.outer[0]
        level=np.array([self.standing+common-differential,self.standing+common+differential])
        target=level if self.mode=='level' else self.q if self.mode=='terrain' else .65*level+.35*self.q
        target=np.clip(target,self.standing-.16,self.standing+.16);self.angle+=np.clip(target-self.angle,-.08*dt,.08*dt)
        error=self.angle-self.q;candidate=np.clip(self.integral+error*dt,-1.,1.)
        effort=5000.*error+220.*candidate-220.*self.velocity
        if self.mode=='terrain':effort=-100.*self.velocity;candidate*=0
        limited=np.clip(effort,-self.max_effort,self.max_effort);self.integral=np.where(abs(effort)<self.max_effort,candidate,self.integral)
        # Soft limits add restoring effort before hard joint stops.
        limited+=np.clip((self.standing-.19-self.q)*10000.,0.,self.max_effort)
        limited-=np.clip((self.q-self.standing-.19)*10000.,0.,self.max_effort)
        self.effort+=np.clip(limited-self.effort,-2000.*dt,2000.*dt);self.effort=np.clip(self.effort,-self.max_effort,self.max_effort)
        diagnostics=[]
        for i in range(2):
            try:diagnostics.append(cylinder_from_notes(float(self.q[i]),float(self.effort[i])))
            except ValueError:self.effort[i]=0.;diagnostics.append({'singular':True})
        self.pub.publish(Float64MultiArray(data=self.effort.tolist()));self.debug.publish(String(data=json.dumps({'mode':self.mode,'roll_rad':float(roll),'pitch_rad':float(pitch),'joint_rad':self.q.tolist(),'target_rad':self.angle.tolist(),'effort_nm':self.effort.tolist(),'cylinders':diagnostics,'geometry':'provisional'})))
def main():
    rclpy.init();n=Suspension()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():n.pub.publish(Float64MultiArray(data=[0.,0.]))
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
