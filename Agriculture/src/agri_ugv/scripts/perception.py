#!/usr/bin/python3
"""Classical RGB-D traversability; conservative missing-depth handling."""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header,String
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from tf2_ros import Buffer,TransformListener
from message_filters import Subscriber,ApproximateTimeSynchronizer

class Perception(Node):
    def __init__(self):
        super().__init__('traversability');self.bridge=CvBridge();self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.plane=np.array([0.,0.,1.,0.]);self.rng=np.random.default_rng(8)
        self.hazards=self.create_publisher(PointCloud2,'perception/hazards',10)
        self.clear=self.create_publisher(PointCloud2,'perception/clearing',10)
        self.lidar=self.create_publisher(PointCloud2,'perception/lidar_obstacles',10)
        self.grid=self.create_publisher(OccupancyGrid,'perception/traversability',10)
        self.stats=self.create_publisher(String,'perception/status',10)
        self.overlay=self.create_publisher(Image,'perception/overlay',10)
        subs=[Subscriber(self,t,topic,qos_profile=qos_profile_sensor_data) for t,topic in [(Image,'camera/image'),(Image,'camera/depth_image'),(CameraInfo,'camera/camera_info')]]
        self.sync=ApproximateTimeSynchronizer(subs,10,.05);self.sync.registerCallback(self.rgbd)
        self.create_subscription(PointCloud2,'lidar/points',self.cloud,qos_profile_sensor_data)
    def transform(self,points,header):
        t=self.buffer.lookup_transform('base_footprint',header.frame_id,rclpy.time.Time.from_msg(header.stamp)).transform
        q=t.rotation;p=t.translation
        return Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[p.x,p.y,p.z]
    def publish_cloud(self,pub,points,stamp):
        pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='base_footprint'),points.astype(np.float32)))
    def rgbd(self,rgb,depth,info):
        image=self.bridge.imgmsg_to_cv2(rgb,'rgb8');d=self.bridge.imgmsg_to_cv2(depth,'32FC1')
        yy,xx=np.mgrid[0:d.shape[0]:4,0:d.shape[1]:4];z=d[yy,xx].ravel();u=xx.ravel();v=yy.ravel()
        rays=np.stack(((u-info.k[2])/info.k[0],(v-info.k[5])/info.k[4],np.ones(len(u))),axis=1)
        valid=np.isfinite(z)&(z>.25)&(z<10)
        try:
            pts=self.transform(rays[valid]*z[valid,None],depth.header)
            origin=self.transform(np.zeros((1,3)),depth.header)[0]
            directions=self.transform(rays,depth.header)-origin
        except Exception:return
        near=(pts[:,0]>.7)&(pts[:,0]<5)&(abs(pts[:,1])<2)&(pts[:,2]<.45)&(pts[:,2]>-.8)
        sample=pts[near];best=0
        if len(sample)>40:
            for _ in range(50):
                a,b,c=sample[self.rng.choice(len(sample),3,replace=False)];normal=np.cross(b-a,c-a);norm=np.linalg.norm(normal)
                if norm<1e-7:continue
                normal/=norm
                if normal[2]<0:normal=-normal
                if normal[2]<.90:continue
                offset=-normal@a;score=int((abs(sample@normal+offset)<.06).sum())
                if score>best:best=score;self.plane=np.r_[normal,offset]
        height=pts@self.plane[:3]+self.plane[3]
        colors=image[v[valid],u[valid]].astype(float);red,green,blue=colors.T
        vegetation=(2*green-red-blue>35)&(green>1.12*red)&(green>1.05*blue)
        # Low cotton foliage is the protected row being straddled, not a
        # drivable obstacle. Tall items, rocks/depressions and people remain
        # hazards for the crop-row controller and Nav2.
        # Vegetation is a protected crop boundary, not a collision obstacle.
        # Keep non-vegetation high objects (people/equipment), rocks and
        # negative-height terrain returns as live hazards.
        unsafe=((height>.60)|(height<-.16))&~vegetation
        # Do not send the rover's own wheels, bumper, or suspension to Nav2 as
        # obstacles.  This envelope is deliberately just larger than the
        # physical footprint; hazards beyond it are still marked normally.
        self_envelope=(pts[:,0]>-1.20)&(pts[:,0]<1.20)&(abs(pts[:,1])<.95)
        # In crop-row mode, plants are lateral lane boundaries.  They guide
        # the row-centre mission but must not be marked as obstacles inside
        # the rover's traversable centre corridor. Objects in the corridor
        # (workers, rocks, equipment) remain live Nav2 obstacles.
        infront=(pts[:,0]>.65)&(pts[:,0]<8)&(abs(pts[:,1])<.40)&~self_envelope
        hazards=pts[unsafe&infront]
        # Invalid returns below the ground horizon are unknown, never free.
        denom=directions@self.plane[:3];distance=-(origin@self.plane[:3]+self.plane[3])/np.where(abs(denom)>.001,denom,np.nan)
        missing=(~valid)&np.isfinite(distance)&(distance>0)&(distance<3)&(v>d.shape[0]*.60)&(u>d.shape[1]*.15)&(u<d.shape[1]*.85)
        holes=origin+directions[missing]*distance[missing,None]
        holes=holes[(abs(holes[:,1])<.40)&~((holes[:,0]>-1.20)&(holes[:,0]<1.20)&(abs(holes[:,1])<.95))]
        if len(holes):hazards=np.vstack((hazards,holes))
        if len(hazards):
            hazards=hazards[~((hazards[:,0]>-1.20)&(hazards[:,0]<1.20)&(abs(hazards[:,1])<.95))]
        # Reject isolated RGB-D speckles; dense depressions and vegetation remain.
        if len(hazards):
            support=cKDTree(hazards).query_ball_point(hazards,.30,return_length=True)
            hazards=hazards[support>=5]
        # Project hazards above local ground so 2D marking includes ditches.
        if len(hazards):hazards[:,2]=np.maximum(hazards[:,2],.25)
        self.publish_cloud(self.hazards,hazards,depth.header.stamp);self.publish_cloud(self.clear,pts,depth.header.stamp)
        grid=np.full((120,160),-1,dtype=np.int8)
        def mark(points,value):
            ij=np.floor((points[:,:2]-[-2.,-6.])/.1).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<160)&(ij[:,1]>=0)&(ij[:,1]<120)
            grid[ij[ok,1],ij[ok,0]]=value
        mark(pts[(abs(height)<.10)&(~vegetation)],0);mark(hazards,100)
        m=OccupancyGrid();m.header=Header(stamp=depth.header.stamp,frame_id='base_footprint');m.info.resolution=.1;m.info.width=160;m.info.height=120;m.info.origin.position.x=-2.;m.info.origin.position.y=-6.;m.info.origin.orientation.w=1.;m.data=grid.ravel().tolist();self.grid.publish(m)
        out=image.copy();out[v[valid][unsafe],u[valid][unsafe]]=[255,0,0];out[v[valid][~unsafe],u[valid][~unsafe]]=[0,220,50]
        overlay=self.bridge.cv2_to_imgmsg(out,'rgb8');overlay.header=rgb.header;self.overlay.publish(overlay)
        self.stats.publish(String(data=json.dumps({'ground_height_m':float(-self.plane[3]/self.plane[2]),'ground_inliers':best,'depth_hazards':int(unsafe.sum()),'missing_depth_hazards':len(holes),'traversable_samples':int((~unsafe).sum())})))
    def cloud(self,msg):
        p=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
        try:p=self.transform(p,msg.header)
        except Exception:return
        height=p@self.plane[:3]+self.plane[3]
        # Cotton foliage can exceed the depth-plane estimate at row ends.
        # Require a human/equipment-scale return for the row-driving safety
        # stop; the simulated worker remains 1.6 m tall.
        mask=(height>1.10)&(height<2.5)&(abs(p[:,1])<.40)&~((abs(p[:,1])<.95)&(p[:,0]>-1.20)&(p[:,0]<1.20))
        self.publish_cloud(self.lidar,p[mask],msg.header.stamp)
def main():
    rclpy.init();n=Perception()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
