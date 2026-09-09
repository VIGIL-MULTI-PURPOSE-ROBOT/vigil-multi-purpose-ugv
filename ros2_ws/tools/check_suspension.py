#!/usr/bin/python3
import rclpy,time,json,math
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('suspension_evidence');samples=[]
n.create_subscription(String,'suspension/debug',lambda m:samples.append(json.loads(m.data)),100);p=n.create_publisher(String,'suspension/mode',10)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
for mode in ['terrain','level','hybrid']:
 p.publish(String(data=mode));end=time.monotonic()+7
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
valid=bool(samples) and all(all(math.isfinite(x) and abs(x)<=800.01 for x in s['effort_nm']) and all('force_n' in c and math.isfinite(c['force_n']) for c in s['cylinders']) for s in samples)
d={'pass':valid,'modes':sorted(set(s['mode'] for s in samples)),'first':samples[:1],'last':samples[-1:],'max_abs_roll_deg':max((abs(s['roll_rad'])*180/math.pi for s in samples),default=0),'max_abs_pitch_deg':max((abs(s['pitch_rad'])*180/math.pi for s in samples),default=0)}
open('reports/stage11_suspension.json','w').write(json.dumps(d,indent=2));print(d);raise SystemExit(0 if valid and len(d['modes'])==3 else 1)
