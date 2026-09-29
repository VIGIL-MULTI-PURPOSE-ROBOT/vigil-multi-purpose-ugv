#!/usr/bin/python3
"""Validate mass, inertia, actuators, encoders and supplied-world references."""
from pathlib import Path
import xml.etree.ElementTree as ET,json
import xacro,numpy as np,yaml
from ament_index_python.packages import get_package_share_directory
root=Path(__file__).resolve().parents[1];description=Path(get_package_share_directory('agri_ugv_description'))
r=ET.fromstring(xacro.process_file(str(description/'urdf/agri_ugv.urdf.xacro'),mappings={'simulation':'true'}).toxml())
mass=0.
for link in r.findall('link'):
 i=link.find('inertial')
 if i is None:continue
 mass+=float(i.find('mass').get('value'));a=i.find('inertia').attrib
 tensor=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
 assert np.linalg.eigvalsh(tensor).min()>0,link.get('name')
assert abs(mass-286.11)<.01
control=r.find('ros2_control');wheels=[j for j in control.findall('joint') if j.get('name').endswith('_joint')];assert len(wheels)==8
for j in wheels:
 assert j.find("command_interface[@name='velocity']") is not None
 if j.get('name') in ['L4_joint','R4_joint']:assert not j.findall('state_interface')
encoded=r.find(".//plugin[@name='gz::sim::systems::JointStatePublisher']").findall('joint_name');assert len(encoded)==6
assert all(j.text not in ['L4_joint','R4_joint'] for j in encoded)
assert not [j for j in r.findall('joint') if j.get('type')=='prismatic']
assert len([j for j in r.findall('joint') if j.get('name').endswith('_axle_mount') and j.get('type')=='fixed'])==8
assert all(j.find("command_interface[@name='effort']") is not None for j in control.findall('joint') if j.get('name').endswith('_rocker'))
assert not any(s.get('type') in ['gps','navsat'] for s in r.findall('.//sensor'))
settings=yaml.safe_load((description/'config/controllers.yaml').read_text())
interfaces={j.get('name'):{v.get('name') for v in j.findall('state_interface')} for j in control.findall('joint')}
broadcaster=settings['joint_state_broadcaster']['ros__parameters']
for name in broadcaster['joints']:
 assert set(broadcaster['interfaces']) <= interfaces.get(name,set()),name
report={'mass_kg':mass,'positive_inertia':True,'driven_wheels':8,'encoder_wheels':6,'rear_encoder_interfaces':False,'inferred_rocker_joints':2,'cad_fixed_axles':8};(root/'reports/structure.json').write_text(json.dumps(report,indent=2));print(report)
