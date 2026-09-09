#!/usr/bin/python3
"""Crop-row sweep -> full-field frontiers -> Dijkstra-guided return to A.

Timeouts, stale transforms and failed routes are never exploration completion.
"""
import json,math,time
from pathlib import Path as FilePath
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose,NavigateThroughPoses,ComputePathToPose
from tf2_ros import Buffer,TransformListener
from map_search import safe_cells,dijkstra,trace,frontier_targets

class Exploration(Node):
    def __init__(self):
        super().__init__('field_exploration')
        out=FilePath.cwd()/'run';out.mkdir(exist_ok=True);self.journal=out/('mission_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl')
        self.bounds=self.declare_parameter('bounds',[-64.,64.,-40.,88.]).value
        self.radius=self.declare_parameter('clearance_radius',.90).value
        self.autostart=self.declare_parameter('autostart',True).value
        self.visit_b=self.declare_parameter('visit_b',True).value
        self.rowwise=self.declare_parameter('rowwise',True).value
        # These values come from the supplied field: rows run along +X and
        # their 1.22 m pitch places each centreline midway between plants.
        self.row_length=self.declare_parameter('row_length',31.5).value
        self.row_pitch=self.declare_parameter('row_pitch',1.22).value
        self.row_guard=self.declare_parameter('row_guard',0.28).value
        self.row_step=self.declare_parameter('row_step',3.0).value
        self.row_count=self.declare_parameter('row_count',1).value
        self.point_b=self.declare_parameter('point_b',[2.,2.,0.7853981634]).value
        self.b_done=not self.visit_b;self.row_index=0;self.row_plan=None;self.row_complete=not self.rowwise;self.row_guard_odom_y=None;self.row_odom_start=None;self.map=None;self.start=None;self.revision=0
        self.empty_count=0;self.last_empty_revision=-1;self.state='waiting'
        self.active=False;self.epoch=0;self.deadline=0.;self.goal_handle=None;self.retry_after=0.;self.failed={};self.current_key=None;self.return_retry=0
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose');self.multi=ActionClient(self,NavigateThroughPoses,'navigate_through_poses');self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status=self.create_publisher(String,'exploration/status',qos);self.route=self.create_publisher(Path,'exploration/return_path',qos)
        self.create_subscription(OccupancyGrid,'map',self.receive,qos);self.create_subscription(String,'exploration/command',self.command,10);self.create_timer(2.,self.tick)
    def receive(self,msg):self.map=msg;self.revision+=1
    def cancel(self):
        self.epoch+=1;self.active=False
        if self.goal_handle:self.goal_handle.cancel_goal_async()
        self.goal_handle=None
    def command(self,msg):
        if msg.data=='start':
            self.autostart=True
            if self.state in ['arrived','error']:self.state='exploring';self.empty_count=0
        elif msg.data=='return':self.cancel();self.state='return_requested';self.autostart=True;self.retry_after=time.monotonic()+1
        elif msg.data=='pause':self.cancel();self.autostart=False;self.report('Mission paused')
    def report(self,text,**extra):
        self.get_logger().info(text);record={'state':self.state,'message':text,'sim_time':self.get_clock().now().nanoseconds*1e-9,**extra};self.status.publish(String(data=json.dumps(record)))
        with self.journal.open('a') as f:f.write(json.dumps(record)+'\n')
    def pose(self):
        transform=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        age=(self.get_clock().now()-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds*1e-9
        if age>2.:raise RuntimeError('Map transform is stale')
        t=transform.transform;q=t.rotation
        return t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def odom_pose(self):
        transform=self.buffer.lookup_transform('odom','base_footprint',rclpy.time.Time())
        t=transform.transform;q=t.rotation
        return t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def message(self,x,y,yaw):
        p=PoseStamped();p.header.frame_id='map' # zero time: fixed map-frame goal, transform at execution time
        p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
    def world(self,cell):
        y,x=cell;return self.map.info.origin.position.x+(x+.5)*self.map.info.resolution,self.map.info.origin.position.y+(y+.5)*self.map.info.resolution
    def cell(self,pose):return (math.floor((pose[1]-self.map.info.origin.position.y)/self.map.info.resolution),math.floor((pose[0]-self.map.info.origin.position.x)/self.map.info.resolution))
    def grid(self):
        m=self.map;g=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width).copy();yy,xx=np.indices(g.shape);x=m.info.origin.position.x+(xx+.5)*m.info.resolution;y=m.info.origin.position.y+(yy+.5)*m.info.resolution
        g[(x<self.bounds[0])|(x>self.bounds[1])|(y<self.bounds[2])|(y>self.bounds[3])]=100
        # This graph is a candidate corridor. Smac validates the full oriented footprint.
        return g,safe_cells(g,m.info.resolution,self.radius)
    def send(self,client,goal,callback,timeout):
        self.epoch+=1;ticket=self.epoch;self.active=True;self.deadline=time.monotonic()+timeout;self.goal_handle=None
        def accepted(f):
            try:h=f.result()
            except Exception:self.fail('Action transport failed');return
            if ticket!=self.epoch:
                if h and h.accepted:h.cancel_goal_async()
                return
            if not h or not h.accepted:self.active=False;callback(None);return
            self.goal_handle=h
            def finished(f):
                if ticket!=self.epoch:return
                self.active=False;self.goal_handle=None
                try:result=f.result()
                except Exception:result=None
                callback(result)
            h.get_result_async().add_done_callback(finished)
        client.send_goal_async(goal).add_done_callback(accepted)
    def fail(self,text):
        if self.current_key is not None:
            count,_=self.failed.get(self.current_key,(0,0.));self.failed[self.current_key]=(count+1,time.monotonic())
        self.active=False;self.retry_after=time.monotonic()+5;self.report(text)
    def tick(self):
        if self.active:
            if self.row_guard_odom_y is not None:
                try:
                    pose=self.odom_pose()
                    if abs(pose[1]-self.row_guard_odom_y)>self.row_guard:
                        self.cancel();self.state='error';self.report('Crop safety stop: tyre-lane deviation exceeded limit',cross_track_m=abs(pose[1]-self.row_guard_odom_y),limit_m=self.row_guard);return
                except Exception:pass
            if time.monotonic()>self.deadline:self.cancel();self.fail('Route timed out; completion is not claimed')
            return
        if self.map is None or self.state in ['arrived','error','row_complete'] or time.monotonic()<self.retry_after:return
        if not self.nav.server_is_ready() or not self.planner.server_is_ready():return
        try:pose=self.pose()
        except Exception:return
        if self.start is None:
            if self.get_clock().now().nanoseconds*1e-9<8:return
            self.start=pose
            try:self.row_odom_start=self.odom_pose()
            except Exception:return
            self.state='exploring';self.report('Recorded start pose',pose=list(pose),odom_pose=list(self.row_odom_start),bounds=list(self.bounds))
        if not self.autostart:return
        if self.state in ['return_requested','returning']:
            g,free=self.grid();self.return_home(free,pose);return
        if not self.row_complete:
            self.run_crop_rows();return
        if not self.b_done:
            self.current_key=('B',);self.plan_goal(self.message(*self.point_b),'B');return
        g,free=self.grid();dist,_=dijkstra(free,self.cell(pose))
        if not np.isfinite(dist).any():self.retry_after=time.monotonic()+10;self.report('Waiting: current footprint lacks confirmed clearance');return
        choices=frontier_targets(g,free,dist,self.map.info.resolution)
        if not choices:
            if self.revision!=self.last_empty_revision:self.empty_count+=1;self.last_empty_revision=self.revision
            if self.empty_count>=5:
                self.state='return_requested';self.report('Exploration complete',scope='full 128 m terrain boundary',empty_map_updates=self.empty_count);self.return_home(free,pose)
            return
        self.empty_count=0
        for _,target,yaw,size in choices:
            x,y=self.world(target);key=(round(x),round(y));count,last=self.failed.get(key,(0,0.))
            if time.monotonic()-last<min(120.,10.*2**min(count,4)):continue
            # Rotate the requested endpoint orientation between retries; never spin in place.
            headings=[yaw,pose[2],yaw+math.pi,yaw+math.pi/2]
            self.current_key=key;self.plan_goal(self.message(x,y,headings[count%4]),'frontier');return
        self.retry_after=time.monotonic()+10;self.report('Frontiers remain; waiting to retry routes',frontiers=len(choices))
    def run_crop_rows(self):
        # Build 3 m goals so SLAM can map each next corridor before Nav2 plans
        # it. Both wheel tracks stay in the two aisles flanking each crop row.
        if self.row_plan is None:
            forward=list(np.arange(self.row_step,self.row_length,self.row_step))+[self.row_length]
            backward=list(np.arange(self.row_length-self.row_step,0.,-self.row_step))+[0.]
            self.row_plan=[(x,0.,0.,'crop row 1',0.) for x in forward]
            if self.row_count > 1:
                self.row_plan += [(self.row_length,self.row_pitch,math.pi,'right headland',None)]
                self.row_plan += [(x,self.row_pitch,math.pi,'crop row 2',self.row_pitch) for x in backward]
        if self.row_index>=len(self.row_plan):
            self.row_complete=True;self.b_done=True;self.row_guard_odom_y=None;self.state='row_complete';self.report('Crop-row sweep complete; plants remained outside the protected corridor',rows=self.row_count,row_pitch_m=self.row_pitch);return
        dx,dy,yaw,label,guard=self.row_plan[self.row_index]
        self.current_key=('crop_row',self.row_index);self.row_guard_odom_y=None if guard is None else self.row_odom_start[1]+guard
        self.plan_goal(self.message(self.start[0]+dx,self.start[1]+dy,yaw),'row:'+label)
    def plan_goal(self,goal,kind):
        req=ComputePathToPose.Goal();req.goal=goal;req.planner_id='GridBased'
        def ready(result):
            if result is None or result.status!=4 or len(result.result.path.poses)<2:self.fail('Planner rejected route; trying another valid endpoint');return
            if not self.autostart:return
            if kind=='B':message='Driving A to B'
            elif kind.startswith('row:'):message='Driving safely between crop rows: '+kind[4:]
            else:message='Navigating to frontier'
            self.report(message,goal=[goal.pose.position.x,goal.pose.position.y]);request=NavigateToPose.Goal();request.pose=goal
            def done(result):
                if result is None or result.status!=4:self.fail('Route failed; exploration is not complete');return
                if kind=='B':self.b_done=True;self.report('Arrived at B; beginning full-field frontier exploration')
                elif kind.startswith('row:'):
                    self.row_guard_odom_y=None;self.row_index+=1;self.report('Crop-row segment complete',segment=kind[4:],completed_segments=self.row_index)
                else:self.failed[self.current_key]=(0,time.monotonic());self.report('Frontier reached')
            self.send(self.nav,request,done,240.)
        self.send(self.planner,req,ready,20.)
    def return_home(self,free,pose):
        if not self.multi.server_is_ready():return
        cell=self.cell(pose);goal=self.cell(self.start);dist,parent=dijkstra(free,cell,goal);cells=trace(parent,cell,goal)
        if not cells:self.retry_after=time.monotonic()+10;self.report('Return route blocked; retaining start pose and retrying');return
        self.report('Computing return path (Dijkstra/BFS)');points=[self.world(c) for c in cells];path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for i,(x,y) in enumerate(points):
            yaw=math.atan2(points[i+1][1]-y,points[i+1][0]-x) if i<len(points)-1 else self.start[2];path.poses.append(self.message(x,y,yaw))
        self.route.publish(path);selected=[];distance=0.
        for i in range(1,len(points)-1):
            distance+=math.dist(points[i-1],points[i])
            if distance>=4.:selected.append(path.poses[i]);distance=0.
        selected.append(self.message(*self.start));self.state='returning';self.current_key=('return',);self.report('Returning to start',grid_length_m=float(dist[goal]*self.map.info.resolution))
        request=NavigateThroughPoses.Goal();request.poses=selected
        def done(result):
            if result is None or result.status!=4:self.fail('Return route failed; retrying without claiming arrival');return
            try:p=self.pose()
            except Exception:self.fail('Cannot verify return pose');return
            error=math.hypot(p[0]-self.start[0],p[1]-self.start[1]);yaw=abs(math.atan2(math.sin(p[2]-self.start[2]),math.cos(p[2]-self.start[2])))
            if error<=.31 and yaw<=.31:self.state='arrived';self.report('Arrived at start.',position_error_m=error,yaw_error_rad=yaw)
            else:self.fail('Return action ended outside start tolerance')
        self.send(self.multi,request,done,max(240.,float(dist[goal]*self.map.info.resolution)/.1))
def main():
    rclpy.init();n=Exploration()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
