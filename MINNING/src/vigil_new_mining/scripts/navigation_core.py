"""Deterministic grid planning and regulated pure-pursuit tracking, independent of ROS.
Uses a conservative 1.0 m swept radius for the 1.56 x 1.12 m rover.
"""
import heapq
import math
import numpy as np
import cv2

RADIUS = 1.0
GOAL_TOLERANCE = .25
MAX_SPEED = .32

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class PlanningError(ValueError):pass

class GridPlanner:
    def __init__(self,grid,resolution,origin,hazards=(),radius=RADIUS):
        self.grid=np.asarray(grid);self.res=float(resolution);self.ox,self.oy=origin;self.radius=radius
        if self.grid.ndim!=2 or self.res<=0:raise PlanningError('Invalid map')
        self.h,self.w=self.grid.shape
        # Distance to a measured obstacle, in metres. Unknown centres are not traversed.
        free_obstacles=(self.grid<35).astype(np.uint8)
        self.clearance=cv2.distanceTransform(free_obstacles,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*self.res
        self.allowed=(self.grid>=0)&(self.grid<35)&(self.clearance>radius)
        self.hazard=np.zeros_like(self.allowed)
        yy,xx=np.ogrid[:self.h,:self.w]
        for x,y in hazards:
            cx,cy=self.cell((x,y));self.hazard|=(xx-cx)**2+(yy-cy)**2<(2./self.res)**2
        self.allowed&=~self.hazard
    def cell(self,p):return math.floor((p[0]-self.ox)/self.res),math.floor((p[1]-self.oy)/self.res)
    def world(self,c):return self.ox+(c[0]+.5)*self.res,self.oy+(c[1]+.5)*self.res
    def inside(self,c):return 0<=c[0]<self.w and 0<=c[1]<self.h
    def safe(self,p):
        x,y=self.cell(p);return self.inside((x,y)) and bool(self.allowed[y,x])
    def validate(self,p,name='Point B'):
        if len(p)!=2 or not all(math.isfinite(v) for v in p):raise PlanningError(name+' must contain finite X and Y coordinates')
        x,y=self.cell(p)
        if not self.inside((x,y)) or self.grid[y,x]<0:raise PlanningError(name+' is in unexplored space. Explore and map it first.')
        if self.hazard[y,x]:raise PlanningError(name+' is inside a mapped gas hazard.')
        if not self.allowed[y,x]:raise PlanningError(name+' is too close to an obstacle for the robot footprint.')
    def segment_safe(self,a,b):
        n=max(1,math.ceil(math.dist(a,b)/(self.res*.35)))
        return all(self.safe((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n)) for i in range(n+1))
    def plan(self,start,goal):
        self.validate(start,'Robot position');self.validate(goal)
        first,last=self.cell(start),self.cell(goal)
        queue=[(math.dist(first,last),0.,first)];cost={first:0.};parent={};found=False
        directions=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))]
        while queue:
            _,g,c=heapq.heappop(queue)
            if g>cost[c]+1e-9:continue
            if c==last:found=True;break
            for dx,dy,d in directions:
                n=(c[0]+dx,c[1]+dy);x,y=n
                if not self.inside(n) or not self.allowed[y,x]:continue
                if dx and dy and (not self.allowed[c[1],x] or not self.allowed[y,c[0]]):continue
                ng=g+d*(1+.35/max(.15,float(self.clearance[y,x])-self.radius))
                if ng<cost.get(n,float('inf')):
                    cost[n]=ng;parent[n]=c;heapq.heappush(queue,(ng+math.dist(n,last),ng,n))
        if not found:raise PlanningError('No collision-free route to Point B in the current map.')
        cells=[last]
        while cells[-1]!=first:cells.append(parent[cells[-1]])
        route=[tuple(start)]+[self.world(c) for c in cells[::-1][1:-1]]+[tuple(goal)]
        # Collision-checked shortcutting reduces grid stair steps. Limit shortcuts to 1.5m.
        smooth=[route[0]];i=0
        while i<len(route)-1:
            j=i+1
            while j+1<len(route) and math.dist(route[i],route[j+1])<=1.5 and self.segment_safe(route[i],route[j+1]):j+=1
            smooth.append(route[j]);i=j
        dense=[smooth[0]]
        for a,b in zip(smooth,smooth[1:]):
            n=max(1,math.ceil(math.dist(a,b)/.15))
            dense.extend((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n) for i in range(1,n+1))
        return dense
    def arc_safe(self,pose,v,w,horizon=1.2):
        x,y,a=pose;dt=.08
        for _ in range(math.ceil(horizon/dt)):
            x+=v*math.cos(a)*dt;y+=v*math.sin(a)*dt;a+=w*dt
            if not self.safe((x,y)):return False
        return True

def track(pose,path,speed=0.,max_speed=MAX_SPEED):
    if not path:return 0.,0.,0.,[]
    x,y,a=pose
    nearest=min(range(len(path)),key=lambda i:math.dist((x,y),path[i]))
    route=path[nearest:];remaining=math.dist((x,y),route[0])+sum(math.dist(p,q) for p,q in zip(route,route[1:]))
    distance=math.dist((x,y),path[-1])
    if distance<=GOAL_TOLERANCE:return 0.,0.,distance,route
    lookahead=max(.5,min(.95,.5+abs(speed)*1.4));target=route[-1];length=0.
    for p,q in zip(route,route[1:]):
        length+=math.dist(p,q)
        if length>=lookahead:target=q;break
    dx,dy=target[0]-x,target[1]-y
    local_x=math.cos(a)*dx+math.sin(a)*dy;local_y=-math.sin(a)*dx+math.cos(a)*dy
    error=math.atan2(local_y,local_x)
    if abs(error)>.7:return 0.,float(np.clip(error*.6,-.28,.28)),distance,route
    curvature=2*local_y/max(.05,dx*dx+dy*dy)
    velocity=min(max_speed,max_speed/(1+1.8*abs(curvature)),max(.04,.5*(distance-.12)))
    angular=float(np.clip(velocity*curvature,-.28,.28))
    return velocity,angular,distance,route
