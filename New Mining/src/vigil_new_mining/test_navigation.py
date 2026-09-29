import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent / 'scripts'))
import numpy as np
from navigation_core import GridPlanner,PlanningError,track,GOAL_TOLERANCE
class NavigationTests(unittest.TestCase):
 def setUp(self):
  self.grid=np.zeros((100,100),np.int16);self.grid[[0,-1],:]=100;self.grid[:,[0,-1]]=100
 def test_avoids_wall_and_preserves_endpoint(self):
  self.grid[10:75,48:52]=100;p=GridPlanner(self.grid,.1,(0,0));route=p.plan((2,2),(8,2));self.assertEqual(route[-1],(8,2));self.assertTrue(all(p.safe(x) for x in route));self.assertGreater(max(y for x,y in route),8)
 def test_rejects_unknown_obstacle_and_nonfinite(self):
  self.grid[30:45,30:45]=-1;p=GridPlanner(self.grid,.1,(0,0))
  for goal in [(3.5,3.5),(.1,.1),(float('nan'),3),(float('inf'),3)]:
   with self.assertRaises(PlanningError):p.plan((2,2),goal)
 def test_blocked_route(self):
  self.grid[:,48:52]=100;p=GridPlanner(self.grid,.1,(0,0))
  with self.assertRaises(PlanningError):p.plan((2,2),(8,2))
 def test_gas_hazard_excluded(self):
  p=GridPlanner(self.grid,.1,(0,0),[(5,5)]);route=p.plan((2,5),(8,5));self.assertTrue(all(math.dist(q,(5,5))>=1.9 for q in route))
 def test_tracker_converges_and_brakes(self):
  pose=[2.,2.,.1];path=[(2+i*.1,2.) for i in range(41)];speed=0
  for _ in range(1000):
   v,w,d,path=track(pose,path,speed);speed=v;pose[0]+=v*math.cos(pose[2])*.1;pose[1]+=v*math.sin(pose[2])*.1;pose[2]+=w*.1
   if d<=GOAL_TOLERANCE:break
  self.assertLessEqual(math.dist(pose[:2],(6,2)),GOAL_TOLERANCE);self.assertEqual(v,0)
 def test_turn_and_arc_collision(self):
  v,w,_,_=track((2,2,math.pi),[(2,2),(4,2)]);self.assertEqual(v,0);self.assertLessEqual(abs(w),.28)
  p=GridPlanner(self.grid,.1,(0,0));self.assertFalse(p.arc_safe((1.1,2,math.pi),.3,0))
if __name__=='__main__':unittest.main()
