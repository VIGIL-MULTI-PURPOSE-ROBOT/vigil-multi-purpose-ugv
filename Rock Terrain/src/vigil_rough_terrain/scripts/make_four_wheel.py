#!/usr/bin/python3
"""Create a mass-matched rigid four-wheel comparator from the existing CAD rover.
Retains front (1) and rear ground-bearing (3) wheels, original hull and clearance.
Removed assembly mass is carried centrally as payload. No unstable COM is invented.
"""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory

def generate():
    p = Path(get_package_share_directory('vigil_rough_terrain'))
    root = ET.fromstring(xacro.process_file(str(p/'urdf/agri_ugv.urdf.xacro'), mappings={'simulation':'false'}).toxml())
    root.set('name', 'four_wheel_rover')
    removed = {f'{s}{i}_{part}' for s in 'LR' for i in (2,4) for part in ('wheel','carrier','guide')}
    mass = sum(float(e.find('inertial/mass').get('value')) for e in root.findall('link') if e.get('name') in removed)
    for e in list(root):
        if (e.tag == 'link' and e.get('name') in removed) or (e.tag == 'joint' and e.find('child').get('link') in removed) or e.tag == 'gazebo':
            root.remove(e)
    payload = root.find("link[@name='payload']/inertial")
    old = float(payload.find('mass').get('value'))
    payload.find('mass').set('value', str(old+mass))
    for key, value in payload.find('inertia').attrib.items():
        payload.find('inertia').set(key, str(float(value)*(old+mass)/old))
    # Lock only comparator suspension at the calibrated flat-ground configuration.
    positions = {'L1':.002766,'L3':-.049648,'R1':.016465,'R3':-.046022}
    for j in root.findall('joint'):
        name = j.get('name')
        if name.endswith('_rocker') or name.endswith('_suspension'):
            j.set('type','fixed')
            if name.endswith('_suspension'):
                ET.SubElement(j, 'origin', xyz=f"0 0 {positions[name.split('_')[0]]}")
            for tag in ('axis','limit','dynamics'):
                e=j.find(tag)
                if e is not None:j.remove(e)
    with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
        f.write(ET.tostring(root));f.flush()
        sdf=ET.fromstring(subprocess.check_output(['gz','sdf','-p',f.name],text=True))
    model=sdf.find('model')
    for side in 'LR':
        for i in (1,3):
            name=f'{side}{i}'
            plugin=ET.SubElement(model,'plugin',filename='gz-sim-joint-controller-system',name='gz::sim::systems::JointController')
            ET.SubElement(plugin,'joint_name').text=name+'_joint'
            ET.SubElement(plugin,'topic').text=f'/comparison/four/{name}/velocity'
            link=model.find(f"link[@name='{name}_wheel']")
            for collision in link.findall('collision'):
                surface=collision.find('surface')
                if surface is None:surface=ET.SubElement(collision,'surface')
                friction=surface.find('friction')
                if friction is None:friction=ET.SubElement(surface,'friction')
                ode=friction.find('ode')
                if ode is None:ode=ET.SubElement(friction,'ode')
                for key in ('mu','mu2'):
                    el=ode.find(key)
                    if el is None:el=ET.SubElement(ode,key)
                    el.text='.85'
    plugin=ET.SubElement(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for key,value in dict(odom_frame='world',robot_base_frame='base_footprint',odom_topic='/comparison/four/odom',dimensions='3',odom_publish_frequency='20').items():
        ET.SubElement(plugin,key).text=value
    return ET.tostring(sdf,encoding='unicode')

if __name__=='__main__':
    import sys
    Path(sys.argv[1]).write_text(generate())
