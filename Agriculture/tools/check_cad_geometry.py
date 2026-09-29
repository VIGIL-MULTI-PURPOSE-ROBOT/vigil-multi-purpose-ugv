#!/usr/bin/python3
"""Verify the CAD reconstruction through the URDF tree, then render a preview."""
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
from import_rover_cad import split,DTYPE,ROTATION,OFFSET
ws=Path(__file__).resolve().parents[1];package=ws/'src/agri_ugv_description'
manifest=json.loads((package/'config/cad_geometry.json').read_text())
data=(package/'meshes/assembly_preview.stl').read_bytes()
assert hashlib.sha256(data).hexdigest()==manifest['sha256']
raw,labels=split(data);expected=raw@ROTATION.T*.001+OFFSET
robot=ET.fromstring(xacro.process_file(str(package/'urdf/agri_ugv.urdf.xacro'),mappings={'simulation':'true'}).toxml())
parents={j.find('child').get('link'):j for j in robot.findall('joint')}
def pose(link,delta=0.):
    if link=='base_link':return np.eye(4)
    joint=parents[link];origin=joint.find('origin')
    xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
    rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz
    if joint.get('name').endswith('_rocker'):
        q=np.eye(4);q[:3,:3]=Rotation.from_euler('y',-np.pi/50+delta).as_matrix();t=t@q
    return pose(joint.find('parent').get('link'),delta)@t
count=0;max_error=0.;render_parts=[]
for part in manifest['parts']:
    link=robot.find("link[@name='"+part['link']+"']")
    visual=next(v for v in link.findall('visual') if v.find('geometry/mesh').get('filename').endswith('/'+part['mesh']))
    origin=visual.find('origin');visual_pose=np.eye(4)
    if origin is not None:
        visual_pose[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        visual_pose[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
    local=np.frombuffer((package/'meshes'/part['mesh']).read_bytes(),dtype=DTYPE,offset=84)['v']
    transform=pose(part['link'])@visual_pose;assembled=local@transform[:3,:3].T+transform[:3,3]
    source=expected[np.isin(labels,part['components'])]
    error=float(np.max(np.abs(assembled-source)));assert error<2e-7,(part['mesh'],error)
    max_error=max(max_error,error);count+=len(local);render_parts.append((assembled,part['material']))
assert count==manifest['triangles']
for side in ['L','R']:
    pivot=np.array(manifest['rocker_pivots_m'][side])
    for i in range(1,5):
        link=f'{side}{i}_wheel'
        assert np.isclose(np.linalg.norm(pose(link)[:3,3]-pivot),np.linalg.norm(pose(link,.18)[:3,3]-pivot))
report={'pass':True,'source_sha256':manifest['sha256'],'triangles_verified':count,'max_coordinate_error_m':max_error,'wheel_axles':8,'rocker_joints':2}
output=ws/'reports/cad_update';output.mkdir(exist_ok=True)
(output/'geometry_check.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(14,7),facecolor='#f1f4f6');ax=fig.add_subplot(121,projection='3d');side=fig.add_subplot(122)
for points,material in render_parts:
    color='#242a30' if material=='rubber' else '#93a9b9'
    ax.add_collection3d(Poly3DCollection(points+[0,0,.5872],facecolor=color,edgecolor='none'))
    visible=points[points[:,:,1].mean(1)>0]
    side.add_collection(PolyCollection(visible[:,:,[0,2]]+[0,.5872],facecolors=color,linewidths=0))
ax.set(xlim=(-.9,.85),ylim=(-.7,.7),zlim=(0,1.2));ax.set_box_aspect((1.75,1.4,1.2));ax.view_init(22,40)
ax.set_title('Updated agriculture rover',fontsize=16,pad=18);ax.set_axis_off()
side.autoscale();side.set_aspect('equal');side.set_ylim(0,1.2);side.set_title('CAD side profile • front →',fontsize=16);side.set_xlabel('Forward position (m)');side.set_ylabel('Height above reference ground (m)');side.grid(alpha=.2)
fig.suptitle('full shhhh_activesuspension.stl | all 557,698 triangles retained',fontsize=14)
fig.tight_layout();fig.savefig(output/'rover_preview.png',dpi=150)
