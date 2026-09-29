#!/usr/bin/python3
"""Sensor-driven exploration. Ground truth is used ONLY to sample environmental fields.
Navigation uses the visual SLAM TF and map; no world geometry enters its planner.
"""
import json, math, time, threading, heapq
from navigation_core import GridPlanner, PlanningError, track, GOAL_TOLERANCE
from collections import deque
from pathlib import Path as FilePath
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, Imu, Temperature, RelativeHumidity, JointState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float64
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def field(cfg,x,y):
    gas=50+sum(z['peak_ppm']*math.exp(-((x-z['x'])**2+(y-z['y'])**2)/(2*z['sigma']**2)) for z in cfg['gas_zones'])
    temp=cfg['temperature_base']+8*math.exp(-((x-12)**2+(y-12)**2)/100)+2*math.sin(y/12)
    rh=np.clip(cfg['humidity_base']+24*math.exp(-((x+30)**2+(y-18)**2)/170)+5*math.sin(x/9),25,99)
    return float(gas),float(temp),float(rh)

def frontier_plan(grid,start,res,hazards):
    """Dijkstra through known free cells, inflated for a 1.65m footprint.
    Unknown cells are never traversed. Frontiers are approached from known space.
    """
    occupied=(grid>=35).astype(np.uint8);r=max(1,math.ceil(.88/res))
    danger=cv2.dilate(occupied,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*r+1,2*r+1)))>0
    allowed=(grid>=0)&(grid<35)&~danger
    for hx,hy in hazards:
        cv2.circle(danger,(hx,hy),max(r,int(2/res)),1,-1) if danger.dtype==np.uint8 else None
        yy,xx=np.ogrid[:grid.shape[0],:grid.shape[1]]
        allowed[(xx-hx)**2+(yy-hy)**2<(2/res)**2]=False
    sx,sy=start;h,w=grid.shape
    if not(0<=sx<w and 0<=sy<h):return []
    if not allowed[sy,sx]:return []
    near_unknown=cv2.dilate((grid<0).astype(np.uint8),np.ones((3,3),np.uint8))>0
    frontier=allowed&near_unknown
    queue=[(0,sx,sy)];cost={(sx,sy):0};parent={};target=None
    while queue:
        d,x,y=heapq.heappop(queue)
        if d!=cost[(x,y)]:continue
        if frontier[y,x] and d*res>1.5:target=(x,y);break
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx,ny=x+dx,y+dy
            if 0<=nx<w and 0<=ny<h and allowed[ny,nx]:
                nd=d+1
                if nd<cost.get((nx,ny),float('inf')):
                    cost[nx,ny]=nd;parent[(nx,ny)]=(x,y);heapq.heappush(queue,(nd,nx,ny))
    if target is None:return []
    result=[target]
    while result[-1]!=start:result.append(parent[result[-1]])
    return result[::-1]

class Mining(Node):
    def __init__(self):
        super().__init__('mining_runtime')
        self.share=FilePath(get_package_share_directory('vigil_new_mining'))
        self.cfg=json.loads((self.share/'config/mission.json').read_text())
        self.cv=CvBridge();self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.motion_lock=threading.RLock();self.goal=None;self.point_a=None;self.goal_settle=0;self.goal_last_progress=0.;self.best_goal_distance=float('inf');self.goal_blocked_since=None
        self.visual_stamp=0.;self.visual_good=False;self.enabled=False;self.pose=None;self.odom=None;self.grid=None;self.grid_msg=None
        self.depth=None;self.depth_stamp=0.;self.rgb=None;self.rgb_stamp=0.;self.images={};self.events=deque(maxlen=80)
        self.hazards=[];self.people=[];self.route=[];self.roll=0.;self.pitch=0.;self.imu_stamp=0.
        self.last_plan=0.;self.last_detection=0.;self.last_progress=time.monotonic();self.progress_pose=None
        self.temp_min=float('inf');self.temp_max=-float('inf');self.truth_stamp=0.
        self.status={'mode':'EXPLORE','goal':None,'goal_distance_m':None,'goal_tolerance_m':GOAL_TOLERANCE,'navigation_algorithm':'Clearance-aware A* + regulated pure pursuit','state':'READY','reason':'Press START to explore','battery':None,'exploration_percent':None,'tunnels':None,'humans':0,'thermal_candidates':0,'yolo_count':0,'replans':0,'recoveries':0,'gas':None,'temperature':None,'humidity':None,'speed':None,'localization':'WAITING','wheel_status':'WAITING','yolo':'LOADING'}
        self.pub=self.create_publisher(Twist,'cmd_vel',10)
        self.env=self.create_publisher(String,'mining/environment',10)
        self.gas_pub=self.create_publisher(Float64,'mining/gas_ppm',10)
        self.temp_pub=self.create_publisher(Temperature,'mining/temperature',10)
        self.rh_pub=self.create_publisher(RelativeHumidity,'mining/humidity',10)
        self.state_pub=self.create_publisher(String,'mining/state',10)
        self.detect_pub=self.create_publisher(String,'mining/detections',10)
        self.mark_pub=self.create_publisher(MarkerArray,'mining/markers',10)
        self.path_pub=self.create_publisher(Path,'mining/path',10)
        self.traj_pub=self.create_publisher(Path,'mining/trajectory',10);self.traj=Path();self.traj.header.frame_id='map'
        self.wheel_pub=self.create_publisher(Odometry,'wheel/odom',10);self.wheel_pose=[0.,0.,0.];self.wheel_time=None
        for topic,typ,cb in [('sim/ground_truth',Odometry,self.environment),('odom',Odometry,self.on_odom),('visual/odom',Odometry,self.visual_odom),('joint_states',JointState,self.wheels),('imu/data',Imu,self.imu),('camera/image',Image,self.rgb_cb),('camera/depth_image',Image,self.depth_cb),('thermal/image',Image,self.thermal_cb),('map',OccupancyGrid,self.map_cb)]:
            self.create_subscription(typ,topic,cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='map' else qos_profile_sensor_data)
        self.create_service(SetBool,'mining/set_running',self.set_running)
        self.net=None
        try:
            self.net=cv2.dnn.readNetFromDarknet(str(self.share/'config/yolov3-tiny.cfg'),str(self.share/'config/yolov3-tiny.weights'));self.status['yolo']='READY'
        except Exception as e:self.status['yolo']='UNAVAILABLE';self.event('YOLO unavailable: '+str(e)[:100])
        self.create_timer(.1,self.control);self.create_timer(.5,self.telemetry);self.create_timer(1.,self.perception)
        self.http()
    def event(self,text):
        if not self.events or self.events[-1]['text']!=text:self.events.append({'time':time.strftime('%H:%M:%S'),'text':text})
    def state(self,s,reason):
        if self.status['state']!=s:self.event(s+': '+reason)
        self.status.update(state=s,reason=reason)
    def planner(self):
        if self.grid is None or self.grid_msg is None:raise PlanningError('Wait for the visual SLAM map.')
        info=self.grid_msg.info
        return GridPlanner(self.grid,info.resolution,(info.origin.position.x,info.origin.position.y),self.hazards+self.people)
    def publish_path(self):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for x,y in self.route:
            ps=PoseStamped();ps.header=path.header;ps.pose.position.x=x;ps.pose.position.y=y;ps.pose.orientation.w=1.;path.poses.append(ps)
        self.path_pub.publish(path)
    def select_goal(self,x,y):
        with self.motion_lock:
            if self.pose is None:raise PlanningError('Wait for robot localization.')
            route=self.planner().plan(self.pose[:2],(x,y))
            self.enabled=False;self.pub.publish(Twist());self.goal=(x,y);self.point_a=self.pose[:2];self.route=route;self.goal_settle=0
            self.status.update(mode='POINT_B',goal=list(self.goal),point_a=list(self.point_a),goal_distance_m=round(math.dist(self.pose[:2],self.goal),3))
            self.state('GOAL_READY','Point B route preview ready. Press GO TO B to move.');self.publish_path()
            return {'accepted':True,'goal':self.goal,'route_points':len(route)}
    def set_running(self,req,res):
        with self.motion_lock:
            self.enabled=req.data;self.pub.publish(Twist());self.last_plan=0.;self.goal_settle=0
            self.goal_last_progress=time.monotonic();self.best_goal_distance=float('inf');self.goal_blocked_since=None
            if not self.goal:self.route=[]
            self.state('NAVIGATE' if self.goal and req.data else 'SEARCH' if req.data else 'READY',
                       'Navigating to Point B' if self.goal and req.data else 'Waiting for visual localization' if req.data else 'Operator STOP')
            res.success=True;res.message=self.status['state'];return res
    def goal_control(self,now,clearance):
        cmd=Twist();distance=math.dist(self.pose[:2],self.goal);self.status['goal_distance_m']=round(distance,3)
        if distance<=GOAL_TOLERANCE:
            self.pub.publish(cmd)
            self.goal_settle=self.goal_settle+1 if abs(self.status.get('speed') or 0)<.035 else 0
            self.state('ARRIVING','Braking and checking final position')
            if self.goal_settle>=5:
                self.enabled=False;self.state('GOAL_REACHED',f'Point B reached within {distance:.2f} m');self.event('Point B arrival confirmed')
            return
        self.goal_settle=0
        if distance<self.best_goal_distance-.08:self.best_goal_distance=distance;self.goal_last_progress=now
        if now-self.goal_last_progress>30:
            self.enabled=False;self.state('GOAL_BLOCKED','No measurable progress for 30 seconds. Select a different safe point or resume after inspection.');self.pub.publish(cmd);return
        if now-self.last_plan>=2 or not self.route:
            if now-self.last_plan>=1:
                self.last_plan=now;self.status['replans']+=1
                try:self.route=self.planner().plan(self.pose[:2],self.goal);self.publish_path();self.goal_blocked_since=None
                except PlanningError as e:
                    self.route=[];self.publish_path();self.state('REPLAN',str(e));self.pub.publish(cmd);return
        if not self.route:self.pub.publish(cmd);return
        planner=self.planner()
        v,w,d,self.route=track(self.pose,self.route,self.status.get('speed') or 0.)
        # Slow near obstacles; stop before the drive controller's braking distance.
        if clearance<.65:v=0.;w=0.;self.last_plan=0.;self.state('AVOID','Obstacle within stopping clearance; replanning')
        else:
            v*=min(1.,max(0.,(clearance-.65)/.8))
            if not planner.arc_safe(self.pose,v,w):v=w=0.;self.last_plan=0.;self.state('REPLAN','Predicted motion leaves the safe mapped corridor')
            else:self.state('NAVIGATE','Tracking the safe path to Point B')
        if now-self.visual_stamp>2:v=min(v,.12)
        cmd.linear.x=float(v);cmd.angular.z=float(w);self.pub.publish(cmd)
    def environment(self,m):
        p=m.pose.pose.position;gas,temp,rh=field(self.cfg,p.x,p.y);self.truth_stamp=time.monotonic()
        severity='CRITICAL' if gas>=15000 else 'DANGEROUS' if gas>=5000 else 'WARNING' if gas>=1000 else 'SAFE'
        old=self.status.get('gas_status');self.temp_min=min(temp,self.temp_min);self.temp_max=max(temp,self.temp_max)
        self.status.update(gas=round(gas,1),gas_status=severity,gas_type='Methane (simulated ppm)',temperature=round(temp,1),temperature_min=round(self.temp_min,1),temperature_max=round(self.temp_max,1),temperature_status='HIGH' if temp>28 else 'NORMAL',humidity=round(rh,1),humidity_status='HIGH' if rh>80 else 'LOW' if rh<35 else 'NORMAL')
        self.gas_pub.publish(Float64(data=gas))
        t=Temperature();t.header=m.header;t.temperature=temp;t.variance=.04;self.temp_pub.publish(t)
        h=RelativeHumidity();h.header=m.header;h.relative_humidity=rh/100;h.variance=.0004;self.rh_pub.publish(h)
        if old!=severity:self.event('Gas '+severity)
        if gas>=5000 and self.pose:
            if all(math.hypot(self.pose[0]-x,self.pose[1]-y)>2 for x,y in self.hazards):self.hazards.append(self.pose[:2]);self.route=[];self.event('Hazard zone mapped')
        self.env.publish(String(data=json.dumps({k:self.status[k] for k in ['gas','gas_status','gas_type','temperature','humidity']})))
    def visual_odom(self,m):
        self.visual_good=m.pose.covariance[0]<1 and m.pose.covariance[0]>=0
        if self.visual_good:self.visual_stamp=time.monotonic()
    def on_odom(self,m):self.odom=m;self.status['speed']=round(m.twist.twist.linear.x,3)
    def wheels(self,m):
        data=dict(zip(m.name,m.velocity));left=[];right=[]
        for side,out in [('L',left),('R',right)]:
            for i,r in enumerate([.23463,.17655,.17655,.17655],1):
                if f'{side}{i}_joint' in data:out.append(data[f'{side}{i}_joint']*r)
        if len(left)!=4 or len(right)!=4:return
        self.status['wheel_status']='8 encoders online';t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if self.wheel_time is None:self.wheel_time=t;return
        dt=t-self.wheel_time;self.wheel_time=t
        if not 0<dt<.5:return
        v=(np.mean(left)+np.mean(right))/2;w=(np.mean(right)-np.mean(left))/1.3
        x,y,a=self.wheel_pose;x+=v*math.cos(a)*dt;y+=v*math.sin(a)*dt;a=wrap(a+w*dt);self.wheel_pose=[x,y,a]
        o=Odometry();o.header=m.header;o.header.frame_id='wheel_odom';o.child_frame_id='base_footprint';o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.z=math.sin(a/2);o.pose.pose.orientation.w=math.cos(a/2);o.twist.twist.linear.x=float(v);o.twist.twist.angular.z=float(w)
        o.pose.covariance[0]=o.pose.covariance[7]=.05;o.pose.covariance[35]=.1;o.twist.covariance[0]=.03;o.twist.covariance[35]=.08;self.wheel_pub.publish(o)
    def imu(self,m):
        q=m.orientation;self.roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));self.pitch=math.asin(np.clip(2*(q.w*q.y-q.z*q.x),-1,1));self.imu_stamp=time.monotonic()
    def jpeg(self,key,img):
        ok,b=cv2.imencode('.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,80])
        if ok:self.images[key]=b.tobytes()
    def rgb_cb(self,m):
        self.rgb=self.cv.imgmsg_to_cv2(m,'bgr8');self.rgb_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;self.jpeg('rgb',self.rgb)
    def depth_cb(self,m):
        a=self.cv.imgmsg_to_cv2(m,'passthrough').astype(np.float32)
        if m.encoding=='16UC1':a/=1000
        self.depth=a;self.depth_stamp=time.monotonic();self.depth_ros_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        pic=np.uint8(np.clip(np.nan_to_num(a,nan=0,posinf=0)/15,0,1)*255);self.jpeg('depth',cv2.applyColorMap(pic,cv2.COLORMAP_TURBO))
    def thermal_cb(self,m):
        raw=self.cv.imgmsg_to_cv2(m,'passthrough');kelvin=raw.astype(np.float32)*.01
        hot=((kelvin>303)&(kelvin<315)).astype(np.uint8)*255
        contours,_=cv2.findContours(hot,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        boxes=[cv2.boundingRect(c) for c in contours if cv2.contourArea(c)>15]
        self.status['thermal_candidates']=len(boxes)
        pic=cv2.applyColorMap(np.uint8(np.clip((kelvin-280)/35,0,1)*255),cv2.COLORMAP_INFERNO)
        for x,y,w,h in boxes:cv2.rectangle(pic,(x,y),(x+w,y+h),(255,255,255),1)
        self.jpeg('thermal',pic)
    def map_cb(self,m):
        self.grid_msg=m;self.grid=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        self.status['known_area_m2']=round(float(np.count_nonzero(self.grid>=0)*m.info.resolution**2),1)
    def perception(self):
        if self.net is None or self.rgb is None or self.depth is None:return
        if abs(self.rgb_stamp-self.depth_ros_stamp)>.2:return
        img=self.rgb.copy();h,w=img.shape[:2];self.net.setInput(cv2.dnn.blobFromImage(img,1/255.,(416,416),swapRB=True,crop=False))
        outs=self.net.forward(self.net.getUnconnectedOutLayersNames());boxes=[];scores=[];classes=[]
        for out in outs:
            for row in out:
                k=int(np.argmax(row[5:]));score=float(row[4]*row[5+k])
                if score<.35:continue
                cx,cy,bw,bh=row[:4]*[w,h,w,h];boxes.append([int(cx-bw/2),int(cy-bh/2),int(bw),int(bh)]);scores.append(score);classes.append(k)
        indices=cv2.dnn.NMSBoxes(boxes,scores,.35,.45);det=[]
        for i in np.array(indices).flatten():
            x,y,bw,bh=boxes[i];u=max(0,min(w-1,x+bw//2));v=max(0,min(h-1,y+bh//2));patch=self.depth[max(0,v-3):v+4,max(0,u-3):u+4];valid=patch[np.isfinite(patch)&(patch>.2)]
            d=float(np.median(valid)) if valid.size else None
            item={'class_id':classes[i],'label':'person' if classes[i]==0 else 'COCO class '+str(classes[i]),'confidence':round(scores[i],3),'depth_m':d}
            det.append(item);cv2.rectangle(img,(x,y),(x+bw,y+bh),(0,200,255),2);cv2.putText(img,item['label'],(x,max(y,15)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,200,255),1)
            if classes[i]==0 and d and self.pose:
                # Camera pinhole projection, then full TF camera optical -> map.
                try:
                    tr=self.tf.lookup_transform('map','camera_optical_frame',rclpy.time.Time());q=tr.transform.rotation;t=tr.transform.translation
                    f=w/(2*math.tan(1.5184/2));vec=np.array([(u-w/2)*d/f,(v-h/2)*d/f,d]);qv=np.array([q.x,q.y,q.z]);rot=vec+2*np.cross(qv,np.cross(qv,vec)+q.w*vec);point=rot+np.array([t.x,t.y,t.z])
                    if all(math.hypot(point[0]-a,point[1]-b)>1.5 for a,b in self.people):self.people.append((float(point[0]),float(point[1])));self.event('Visual human detection localized')
                    item['map_position']=point.tolist()
                except Exception:pass
        self.status['yolo_count']=len(det);self.status['humans']=len(self.people);self.detect_pub.publish(String(data=json.dumps(det)));self.jpeg('rgb',img)
    def control(self):
        with self.motion_lock:self.control_locked()
    def control_locked(self):
        cmd=Twist();now=time.monotonic()
        try:
            tr=self.tf.lookup_transform('map','base_footprint',rclpy.time.Time());p=tr.transform.translation
            age=(self.get_clock().now().nanoseconds-(tr.header.stamp.sec*10**9+tr.header.stamp.nanosec))/1e9
            if age>1.5:raise RuntimeError('stale localization')
            self.pose=(p.x,p.y,yaw(tr.transform.rotation));self.status['localization']='VISUAL + WHEEL/IMU' if now-self.visual_stamp<2 else 'WHEEL/IMU PREDICTION';self.status['position']=[round(p.x,2),round(p.y,2),round(p.z,2)];self.status['heading']=round(math.degrees(self.pose[2]),1)
        except Exception:
            self.status['localization']='LOST'
            if self.enabled:self.state('SEARCH','Waiting for current visual SLAM localization')
            self.pub.publish(cmd);return
        if not self.enabled:self.pub.publish(cmd);return
        if now-self.visual_stamp>20:
            self.state('RECOVER','Visual features unavailable: controlled re-observation')
            if self.goal is None and now-self.depth_stamp<1 and now-self.imu_stamp<1 and abs(self.roll)<.3 and abs(self.pitch)<.3:cmd.angular.z=.12
            self.pub.publish(cmd);return
        if now-self.depth_stamp>1 or now-self.imu_stamp>1 or now-self.truth_stamp>2:
            self.state('SEARCH','Sensor stream stale; safe stop');self.pub.publish(cmd);return
        if abs(self.roll)>.45 or abs(self.pitch)>.5:
            self.state('RECOVER','Unsafe body inclination; operator inspection required');self.pub.publish(cmd);return
        # Forward camera upper/middle region avoids mistaking the floor for a wall.
        a=self.depth;roi=a[a.shape[0]//4:a.shape[0]*2//3,a.shape[1]//3:a.shape[1]*2//3];v=roi[np.isfinite(roi)&(roi>.2)]
        clearance=float(np.percentile(v,8)) if v.size>20 else 0
        self.status['forward_clearance_m']=round(clearance,2)
        if self.goal is not None:self.goal_control(now,clearance);return
        if clearance<.85:
            self.route=[];self.state('AVOID','Near obstacle: turning toward clearer observed sector')
            sectors=[a[a.shape[0]//4:a.shape[0]//2,:a.shape[1]//3],a[a.shape[0]//4:a.shape[0]//2,a.shape[1]*2//3:]]
            values=[float(np.nanmedian(np.where((s>.2)&np.isfinite(s),s,np.nan))) for s in sectors]
            if clearance>.35:cmd.angular.z=.22 if values[0]>values[1] else -.22
            self.pub.publish(cmd);return
        if self.grid is None:self.state('SEARCH','Waiting for visual SLAM map');self.pub.publish(cmd);return
        info=self.grid_msg.info;r=info.resolution;ox=info.origin.position.x;oy=info.origin.position.y
        if now-self.last_plan>4:
            self.last_plan=now;start=(int((self.pose[0]-ox)/r),int((self.pose[1]-oy)/r));haz=[(int((x-ox)/r),int((y-oy)/r)) for x,y in self.hazards]
            cells=frontier_plan(self.grid,start,r,haz)
            self.route=[(ox+(x+.5)*r,oy+(y+.5)*r) for x,y in cells];self.status['replans']+=1
            path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
            for x,y in self.route:
                ps=PoseStamped();ps.header=path.header;ps.pose.position.x=x;ps.pose.position.y=y;ps.pose.orientation.w=1.;path.poses.append(ps)
            self.path_pub.publish(path)
        while self.route and math.hypot(self.route[0][0]-self.pose[0],self.route[0][1]-self.pose[1])<.65:self.route.pop(0)
        if not self.route:
            self.state('SEARCH','No reachable frontier: scan for new visual observations');cmd.angular.z=.18
        else:
            gx,gy=self.route[0];err=wrap(math.atan2(gy-self.pose[1],gx-self.pose[0])-self.pose[2]);cmd.angular.z=float(np.clip(err,-.35,.35));cmd.linear.x=.25*max(0,1-abs(err)/.7)*min(1,(clearance-.85)/1.2)
            self.state('EXPLORE','Following reachable frontier in visual SLAM map')
        if self.status.get('gas_status') in ['DANGEROUS','CRITICAL']:
            self.state('HAZARD_DETECTED','Hazard mapped; seeking a safe observed route');cmd.linear.x=min(cmd.linear.x,.15)
        self.pub.publish(cmd)
    def telemetry(self):
        self.status['hazard_zones']=len(self.hazards);self.status['events']=list(self.events)
        self.status['simulation_data_age_s']=round(time.monotonic()-self.truth_stamp,1) if self.truth_stamp else None
        self.state_pub.publish(String(data=json.dumps(self.status)))
        if self.pose:
            ps=PoseStamped();ps.header.frame_id='map';ps.header.stamp=self.get_clock().now().to_msg();ps.pose.position.x=self.pose[0];ps.pose.position.y=self.pose[1];ps.pose.orientation.w=1.
            if not self.traj.poses or math.hypot(ps.pose.position.x-self.traj.poses[-1].pose.position.x,ps.pose.position.y-self.traj.poses[-1].pose.position.y)>.15:self.traj.poses.append(ps)
            self.traj.header.stamp=ps.header.stamp;self.traj_pub.publish(self.traj)
        marks=MarkerArray()
        for ns,pts,color in [('hazards',self.hazards,(1.,.2,.05)),('humans',self.people,(.1,1.,.5))]:
            for i,(x,y) in enumerate(pts):
                m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.ns=ns;m.id=i;m.type=Marker.CYLINDER;m.action=Marker.ADD;m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.5;m.pose.orientation.w=1.;m.scale.x=m.scale.y=1.;m.scale.z=1.;m.color.r,m.color.g,m.color.b=color;m.color.a=.6;marks.markers.append(m)
        self.mark_pub.publish(marks)
    def http(self):
        node=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                path=self.path.split('?')[0]
                if path=='/api/state':body=json.dumps(node.status).encode();typ='application/json'
                elif path=='/api/map':
                    m=node.grid_msg
                    body=json.dumps({'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],'data':list(m.data),'pose':node.pose,'goal':node.goal,'point_a':node.point_a,'route':node.route,'hazards':node.hazards,'humans':node.people,'trajectory':[[p.pose.position.x,p.pose.position.y] for p in node.traj.poses]} if m else {}).encode();typ='application/json'
                elif path.startswith('/image/'):
                    body=node.images.get(path.split('/')[-1]);typ='image/jpeg'
                    if body is None:self.send_error(503,'Waiting for sensor');return
                elif path=='/':body=(node.share/'dashboard/index.html').read_bytes();typ='text/html'
                else:self.send_error(404);return
                self.send_response(200);self.send_header('Content-Type',typ);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def do_POST(self):
                if self.path not in ['/start','/stop','/goal','/cancel','/explore']:self.send_error(404);return
                origin=self.headers.get('Origin')
                if origin and origin not in ['http://localhost:8080','http://127.0.0.1:8080']:self.send_error(403);return
                try:
                    with node.motion_lock:
                        if self.path=='/goal':
                            length=int(self.headers.get('Content-Length','0'))
                            if not 0<length<=1024:raise ValueError('Invalid request size')
                            data=json.loads(self.rfile.read(length));x,y=data.get('x'),data.get('y')
                            if isinstance(x,bool) or isinstance(y,bool) or not isinstance(x,(int,float)) or not isinstance(y,(int,float)):raise ValueError('Point B requires numeric X and Y')
                            result=node.select_goal(float(x),float(y))
                        else:
                            if self.path in ['/cancel','/explore']:
                                node.goal=None;node.point_a=None;node.route=[];node.status.update(mode='EXPLORE',goal=None,point_a=None,goal_distance_m=None);node.publish_path()
                            req=SetBool.Request();req.data=self.path in ['/start','/explore'];node.set_running(req,SetBool.Response());result={'accepted':True,'state':node.status['state']}
                    payload=json.dumps(result).encode();code=200
                except (ValueError,TypeError,KeyError) as e:payload=json.dumps({'accepted':False,'error':str(e)}).encode();code=422
                self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
        self.server=ThreadingHTTPServer(('127.0.0.1',8080),Handler);threading.Thread(target=self.server.serve_forever,daemon=True).start()

def main():
    rclpy.init();node=Mining()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.pub.publish(Twist())
        node.server.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
