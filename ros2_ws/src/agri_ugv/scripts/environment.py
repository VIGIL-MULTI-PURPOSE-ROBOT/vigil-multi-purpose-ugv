#!/usr/bin/python3
"""Simulation-only worker motion and controllable illumination test zone.

LDR uses commanded scene illumination, not a physical photometric ray sensor.
Ground truth is used only for the lighting zone, never for robot localization.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import Float32,Bool,String
from nav_msgs.msg import Odometry
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.light_pb2 import Light

class Environment(Node):
    def __init__(self):
        super().__init__('farm_environment');self.gz=GzNode();self.start=None;self.warned=False;self.lights={};self.position=None;self.override=None;self.headlight=False;self.applied={}
        self.lighting=self.declare_parameter('lighting_enabled',False).value
        self.ldr=self.create_publisher(Float32,'ldr/illuminance',10);self.state=self.create_publisher(String,'simulation/lighting_status',10)
        self.create_subscription(Odometry,'sim/ground_truth',lambda m:setattr(self,'position',m.pose.pose.position),10)
        self.create_subscription(Float32,'simulation/ambient_level',self.ambient,10)
        self.create_subscription(Bool,'headlight/command',lambda m:setattr(self,'headlight',m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.2,self.tick)
    def ambient(self,m):
        if math.isfinite(m.data):self.override=None if m.data<0 else max(0.,min(1.,m.data))
    def discover(self):
        ok,s=self.gz.request('/world/vigil_cotton_farm/scene/info',Empty(),Empty,Scene,100)
        if not ok:return
        for l in s.light:
            if l.name=='sun':self.lights['sun']=l
        for m in s.model:
            if m.name=='agri_ugv':
                for link in m.link:
                    for light in link.light:
                        if light.name.endswith('headlight'):self.lights['headlight']=light
    def set_light(self,name,intensity):
        if name not in self.lights or self.applied.get(name)==intensity:return
        light=Light();light.CopyFrom(self.lights[name]);light.intensity=float(intensity)
        ok,result=self.gz.request('/world/vigil_cotton_farm/light_config',light,Light,Boolean,100)
        if ok and result.data:self.applied[name]=intensity
    def tick(self):
        t=self.get_clock().now().nanoseconds*1e-9
        if t<1:return
        if self.start is None:self.start=t
        p=Pose();p.name='worker_d_02';p.position.x=10.;p.position.y=-14.+3.*math.sin((t-self.start)*.20);p.position.z=-.2;p.orientation.w=1.
        success,response=self.gz.request('/world/vigil_cotton_farm/set_pose',p,Pose,Boolean,100)
        if (not success or not response.data) and not self.warned:self.get_logger().warning('Waiting for moving obstacle pose service discovery');self.warned=True
        if not self.lighting:return
        if len(self.lights)<2:self.discover()
        p=self.position;dark=p is not None and 6<p.x<12 and -27<p.y<-19
        level=self.override if self.override is not None else (.03 if dark else 1.)
        self.set_light('sun',level);self.set_light('headlight',8. if self.headlight else 0.)
        if self.applied.get('sun')==level:self.ldr.publish(Float32(data=level*1000.))
        self.state.publish(String(data=str({'ambient_level':level,'headlight_requested':self.headlight,'applied':self.applied})))
def main():
    rclpy.init();n=Environment()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
