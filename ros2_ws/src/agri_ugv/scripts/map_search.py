#!/usr/bin/python3
"""Conservative occupancy-grid graph operations, independent of ROS."""
import heapq,math
import numpy as np
from scipy.ndimage import distance_transform_edt,binary_dilation,label

def safe_cells(grid,resolution,radius):
    return (grid==0)&(distance_transform_edt(np.pad(grid==0,1))[1:-1,1:-1]*resolution>radius)

def dijkstra(free,start,goal=None):
    h,w=free.shape;dist=np.full((h,w),np.inf);parent={}
    y,x=start
    if not (0<=y<h and 0<=x<w and free[y,x]):return dist,parent
    dist[y,x]=0;heap=[(0.,y,x)]
    while heap:
        cost,y,x=heapq.heappop(heap)
        if cost!=dist[y,x]:continue
        if goal==(y,x):break
        for dy,dx in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            yy,xx=y+dy,x+dx
            if not (0<=yy<h and 0<=xx<w and free[yy,xx]):continue
            if dy and dx and not (free[y,xx] and free[yy,x]):continue
            new=cost+math.hypot(dy,dx)
            if new<dist[yy,xx]:dist[yy,xx]=new;parent[(yy,xx)]=(y,x);heapq.heappush(heap,(new,yy,xx))
    return dist,parent

def trace(parent,start,goal):
    path=[goal]
    while path[-1]!=start:
        if path[-1] not in parent:return []
        path.append(parent[path[-1]])
    return path[::-1]

def frontier_targets(grid,free,dist,resolution,min_cluster=8):
    edge=(grid==0)&binary_dilation(grid<0)
    groups,count=label(edge,np.ones((3,3)))
    candidates=[];reachable=np.isfinite(dist)&free
    for k in range(1,count+1):
        boundary=np.argwhere(groups==k)
        if len(boundary)<min_cluster:continue
        # Stand back from unknown cells by a complete footprint radius.
        distance=distance_transform_edt(groups!=k)*resolution
        region=reachable&(distance<2.5)&(distance>1.2)&(dist*resolution>1.)
        yy,xx=np.where(region)
        if not len(yy):continue
        score=dist[yy,xx]*resolution+distance[yy,xx]*.4
        picked=[]
        for j in np.argsort(score):
            y,x=int(yy[j]),int(xx[j])
            if any(math.hypot(y-a,x-b)*resolution<2. for a,b in picked):continue
            near=boundary[np.argmin(np.sum((boundary-[y,x])**2,axis=1))]
            candidates.append((float(score[j]),(y,x),math.atan2(near[0]-y,near[1]-x),len(boundary)));picked.append((y,x))
            if len(picked)>=3:break
    return sorted(candidates)
