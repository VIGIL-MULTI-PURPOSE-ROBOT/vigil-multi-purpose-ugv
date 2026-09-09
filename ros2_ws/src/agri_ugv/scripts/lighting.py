#!/usr/bin/python3
"""LDR hysteresis and RGB gain mode; Gazebo supplies the light-level input."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from std_msgs.msg import Float32,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Lighting(Node):
    def __init__(self):
        super().__init__('automatic_headlight');self.on=False;self.bridge=CvBridge()
        self.low=self.declare_parameter('on_threshold',150.).value;self.high=self.declare_parameter('off_threshold',250.).value
        self.command=self.create_publisher(Bool,'headlight/command',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));self.command.publish(Bool(data=False))
        self.image=self.create_publisher(Image,'camera/low_light_image',10)
        self.create_subscription(Float32,'ldr/illuminance',self.light,10);self.create_subscription(Image,'camera/image',self.camera,qos_profile_sensor_data)
    def light(self,msg):
        if not np.isfinite(msg.data):return
        on=msg.data<self.high if self.on else msg.data<self.low
        if on!=self.on:self.on=on;self.command.publish(Bool(data=on));self.get_logger().info('Headlight ON; low-light gain enabled' if on else 'Headlight OFF; normal RGB mode')
    def camera(self,msg):
        a=self.bridge.imgmsg_to_cv2(msg,'rgb8')
        if self.on:a=np.clip(a.astype(np.float32)*2.5,0,255).astype(np.uint8)
        out=self.bridge.cv2_to_imgmsg(a,'rgb8');out.header=msg.header;self.image.publish(out)
def main():
    rclpy.init();n=Lighting()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
