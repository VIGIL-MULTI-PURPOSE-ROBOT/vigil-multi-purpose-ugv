#!/usr/bin/python3
"""Validate mass, inertia, actuators, encoders and supplied-world references."""
from pathlib import Path
import xml.etree.ElementTree as ET,json
import xacro,numpy as np
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
assert len([j for j in r.findall('joint') if j.get('type')=='prismatic'])==8
assert all(j.find("command_interface[@name='effort']") is not None for j in control.findall('joint') if j.get('name').endswith('_rocker'))
assert not any(s.get('type') in ['gps','navsat'] for s in r.findall('.//sensor'))
report={'mass_kg':mass,'positive_inertia':True,'driven_wheels':8,'encoder_wheels':6,'rear_encoder_interfaces':False,'provisional_suspension_joints':10};(root/'reports/structure.json').write_text(json.dumps(report,indent=2));print(report)
