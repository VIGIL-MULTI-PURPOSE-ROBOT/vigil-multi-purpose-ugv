"""Build small contact fixtures using the actual exported people and colliders."""
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sar_motion import timed_route, plugin_element
from sar_physics import configure_world

source = Path('gazebo_export/dynamic_demo.sdf')
root = ET.parse(source).getroot()
prototype = root.find("world/model[@name='SAR_DynamicPerson_016']")
if prototype is None:
    raise SystemExit(f'{source} has no SAR_DynamicPerson_016 to copy; '
                     'run "bash run_gazebo.sh --demo --check" first.')

# These fixtures run headless inside TestFixture, which has no display and no
# render loop. Systems that build an Ogre scene have nothing to draw into and
# take the process down with them, so they never belong in a contact test.
RENDERING_SYSTEMS = ('gz-sim-sensors-system', 'gz-sim-thermal-system')

for kind in ('free', 'wall', 'people'):
    scene = copy.deepcopy(root)
    world = scene.find('world')
    for plugin in list(world.findall('plugin')):
        if plugin.get('filename') in RENDERING_SYSTEMS:
            world.remove(plugin)
    for visual in scene.findall('.//visual'):
        for plugin in list(visual.findall('plugin')):
            visual.remove(plugin)
    for model in list(world.findall('model')):
        if model.get('name') != 'demo_ground':
            world.remove(model)
    for name, start, end in [('SAR_DynamicPerson_test_a', -4, 4)] + (
            [('SAR_DynamicPerson_test_b', 4, -4)] if kind == 'people' else []):
        person = copy.deepcopy(prototype)
        person.set('name', name)
        person.remove(person.find("plugin[@name='sar::WaypointSystem']"))
        keys = timed_route([(start, 0, 0.05), (end, 0, 0.05)], 1.0)
        # Hold the destination beyond the eight-second test window.
        keys.insert(2, (30.0, keys[1][1]))
        keys[-1] = (38.0, keys[-1][1])
        person.find('pose').text = ' '.join(map(str, keys[0][1]))
        person.append(plugin_element(keys))
        sensor = ET.SubElement(person.find('link'), 'sensor', name='touch', type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        contact = ET.SubElement(sensor, 'contact')
        for collision in person.findall('link/collision'):
            ET.SubElement(contact, 'collision').text = collision.get('name')
        world.append(person)
    if kind == 'wall':
        world.append(ET.fromstring('''<model name="test_wall"><static>true</static>
          <pose>0 0 2 0 0 0</pose><link name="wall">
          <collision name="wall_collision"><geometry><box><size>0.2 20 4</size></box></geometry></collision>
          <visual name="wall_visual"><geometry><box><size>0.2 20 4</size></box></geometry></visual>
          </link></model>'''))
    # Contact is the whole point of these fixtures, so they always use the
    # contact-aware controller regardless of what the main world is set to.
    configure_world(scene, controller=2)
    for uri in scene.findall('.//mesh/uri'):
        uri.text = str((source.parent / uri.text).resolve())
    ET.ElementTree(scene).write(f'validation/contact_{kind}.sdf', encoding='utf-8', xml_declaration=True)
