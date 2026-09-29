#!/usr/bin/python3
import time,json,ast
import numpy as np,cv2,rclpy
from std_msgs.msg import String,Float32,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('lighting_evidence');data={};b=CvBridge();p=n.create_publisher(Float32,'simulation/ambient_level',10)
n.create_subscription(String,'simulation/lighting_status',lambda m:data.update(state=ast.literal_eval(m.data)),10)
n.create_subscription(Image,'camera/image',lambda m:data.update(raw=b.imgmsg_to_cv2(m,'bgr8')),qos_profile_sensor_data)
n.create_subscription(Image,'camera/low_light_image',lambda m:data.update(gain=b.imgmsg_to_cv2(m,'bgr8')),qos_profile_sensor_data)
results=[]
for level in [1.,.03,1.]:
 end=time.monotonic()+7
 while time.monotonic()<end:
  p.publish(Float32(data=level));rclpy.spin_once(n,timeout_sec=.1)
 r={'level':level,'state':data.get('state')}
 for key in ['raw','gain']:
  if key in data:r[key+'_mean']=float(data[key].mean());cv2.imwrite('reports/light_'+str(len(results))+'_'+key+'.png',data[key])
 results.append(r)
p.publish(Float32(data=-1.));n.destroy_node();rclpy.shutdown()
ok=all(r.get('state',{}).get('applied',{}).get('headlight')==(8. if r['level']<.1 else 0.) for r in results)
open('reports/stage10_lighting.json','w').write(json.dumps({'pass':ok,'samples':results},indent=2));print(results);raise SystemExit(0 if ok else 1)
