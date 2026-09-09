#!/usr/bin/python3
import json,time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
n=Node();poses=[]
def cb(m):
 for p in m.pose:
  if p.name=='worker_d_02':poses.append([p.position.x,p.position.y,p.position.z])
n.subscribe(Pose_V,'/world/vigil_cotton_farm/pose/info',cb)
time.sleep(4)
d={'samples':len(poses),'first':poses[:1],'last':poses[-1:],'moving':len(poses)>2 and abs(poses[-1][1]-poses[0][1])>.05}
open('reports/stage8_dynamic.json','w').write(json.dumps(d,indent=2));print(d)
raise SystemExit(0 if d['moving'] else 1)
