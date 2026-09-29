#!/usr/bin/python3
import argparse,json,time
import rclpy
from tf2_ros import Buffer,TransformListener
p=argparse.ArgumentParser();p.add_argument('--root',default='base_footprint');p.add_argument('--seconds',type=float,default=35);p.add_argument('--output',default='reports/stage4_tf.json');a=p.parse_args()
rclpy.init();n=rclpy.create_node('tf_validation');b=Buffer();l=TransformListener(b,n)
frames=['base_link','lidar_link','camera_optical_frame','imu_link']+[s+str(i)+'_wheel' for s in ['L','R'] for i in range(1,5)]
end=time.monotonic()+a.seconds
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
    if all(b.can_transform(a.root,f,rclpy.time.Time()) for f in frames):break
result={f:bool(b.can_transform(a.root,f,rclpy.time.Time())) for f in frames}
print(json.dumps(result,indent=2));open(a.output,'w').write(json.dumps(result,indent=2))
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if all(result.values()) else 1)
