#!/usr/bin/env python3
"""Create a Phobos-friendly copy of the generated Gazebo SDF model."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'models/robot/model.sdf'
target = ROOT / 'models/robot/model_phobos_import.sdf'
tree = ET.parse(source)
model = tree.getroot().find('model')
if model is None:
    raise ValueError('No model in model.sdf')
for link in model.findall('link'):
    for sensor in link.findall('sensor'):
        pose = sensor.find('pose')
        if pose is not None:
            pose.set('relative_to', link.get('name'))
for uri in model.findall('.//mesh/uri'):
    value = uri.text or ''
    prefix = 'model://robot/'
    if value.startswith(prefix):
        relative = value[len(prefix):]
        if not (target.parent / relative).is_file():
            raise FileNotFoundError(relative)
        uri.text = relative
ET.indent(tree, space='  ')
tree.write(target, encoding='utf-8', xml_declaration=True)
print(target)
