#!/usr/bin/python3
"""Rear-wheel TF uses simulator link poses, never synthetic encoder values."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

class Visualization(Node):
    def __init__(self):
        super().__init__('simulation_visualization');self.tf=TransformBroadcaster(self)
        self.create_subscription(TFMessage,'sim/link_poses',self.poses,qos_profile_sensor_data)
        self.marker=self.create_publisher(Marker,'imu/orientation_marker',10)
        self.create_subscription(Imu,'imu/data',self.imu,qos_profile_sensor_data)
    def poses(self,msg):
        poses={t.child_frame_id.split('/')[-1]:t for t in msg.transforms if t.header.frame_id=='agri_ugv'}
        for side in ['L','R']:
            parent=poses.get(side+'4_carrier');child=poses.get(side+'4_wheel')
            if parent is None or child is None:continue
            def parts(t):
                q=t.transform.rotation;p=t.transform.translation
                return Rotation.from_quat([q.x,q.y,q.z,q.w]),np.array([p.x,p.y,p.z])
            rp,pp=parts(parent);rc,pc=parts(child)
            xyz=rp.inv().apply(pc-pp);q=(rp.inv()*rc).as_quat()
            t=TransformStamped();t.header.stamp=child.header.stamp;t.header.frame_id=side+'4_carrier';t.child_frame_id=side+'4_wheel'
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            self.tf.sendTransform(t)
    def imu(self,msg):
        m=Marker();m.header=msg.header;m.ns='imu';m.id=0;m.type=Marker.ARROW;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.scale.x=.5;m.scale.y=.04;m.scale.z=.04;m.color.g=1.;m.color.a=1.
        self.marker.publish(m)
def main():
    rclpy.init();n=Visualization()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
