import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/agri_ugv/scripts'))
import numpy as np
from map_search import dijkstra,trace,safe_cells,frontier_targets

def test_shortest_route_and_blocked_corner():
    f=np.ones((6,6),bool);f[1:5,3]=False
    d,p=dijkstra(f,(2,1),(2,5));route=trace(p,(2,1),(2,5));assert route and d[2,5]>4
    for a,b in zip(route,route[1:]):
        assert f[b]
        if a[0]!=b[0] and a[1]!=b[1]:assert f[a[0],b[1]] and f[b[0],a[1]]
    d,p=dijkstra(np.array([[1,0],[0,1]],bool),(0,0),(1,1));assert not trace(p,(0,0),(1,1))

def test_unknown_clearance_and_frontiers():
    g=np.full((60,60),-1);g[5:50,5:50]=0;f=safe_cells(g,.2,1.25)
    assert not f[5,10] and f[25,25]
    d,_=dijkstra(f,(25,25));assert frontier_targets(g,f,d,.2)
    g[g<0]=100;assert not frontier_targets(g,f,d,.2)
