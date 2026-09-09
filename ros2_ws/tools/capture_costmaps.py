#!/usr/bin/python3
import rclpy,time,numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy
rclpy.init();n=rclpy.create_node('costmap_snapshot');seen=set()
def cb(m,key):
 a=np.asarray(m.data).reshape(m.info.height,m.info.width);np.savez('reports/'+key+'.npz',a=a,x=m.info.origin.position.x,y=m.info.origin.position.y,r=m.info.resolution);seen.add(key)
for topic,key in [('/map','slam'),('/global_costmap/costmap','global'),('/local_costmap/costmap','local')]:n.create_subscription(OccupancyGrid,topic,lambda m,k=key:cb(m,k),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
end=time.monotonic()+10
while time.monotonic()<end and len(seen)<3:rclpy.spin_once(n,timeout_sec=.1)
print(seen)
