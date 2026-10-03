#!/usr/bin/env python3
"""Check asset integrity and physical invariants without ROS or Gazebo."""
from pathlib import Path
import json
import math
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def numbers(text):
    return list(map(float,text.split()))


def main():
    model=ET.parse(ROOT/'models/robot/model.sdf').getroot().find('model')
    world=ET.parse(ROOT/'worlds/test_world.sdf').getroot().find('world')
    links={l.attrib['name']:l for l in model.findall('link')}
    assert set(links)=={'base_link','left_track','right_track'}
    mass=0
    weighted_center=[0.,0.,0.]
    for name,link in links.items():
        inertial=link.find('inertial');m=float(inertial.findtext('mass'));mass+=m
        assert math.isfinite(m) and m>0,(name,'invalid mass')
        i=inertial.find('inertia');xx=float(i.findtext('ixx'));yy=float(i.findtext('iyy'));zz=float(i.findtext('izz'))
        xy=float(i.findtext('ixy'));xz=float(i.findtext('ixz'));yz=float(i.findtext('iyz'))
        determinant=xx*yy*zz+2*xy*xz*yz-xx*yz*yz-yy*xz*xz-zz*xy*xy
        assert all(math.isfinite(x) for x in [xx,yy,zz,xy,xz,yz])
        assert xx>0 and xx*yy-xy*xy>0 and determinant>0,(name,'non-positive inertia')
        assert xx<=yy+zz+1e-9 and yy<=xx+zz+1e-9 and zz<=xx+yy+1e-9,(name,'nonphysical moments')
        lp=numbers(link.findtext('pose','0 0 0 0 0 0'));ip=numbers(inertial.findtext('pose'))
        for axis in range(3):weighted_center[axis]+=m*(lp[axis]+ip[axis])
        for collision in link.findall('collision'):
            pose=numbers(collision.findtext('pose'));g=collision.find('geometry')
            if g.find('box') is not None:
                size=numbers(g.findtext('box/size'));assert min(size)>0
                bottom=lp[2]+pose[2]-size[2]/2
            else:
                r=float(g.findtext('cylinder/radius'));length=float(g.findtext('cylinder/length'))
                assert r>0 and length>0
                assert abs(abs(pose[3])-math.pi/2)<1e-8
                bottom=lp[2]+pose[2]-r
            assert bottom>=-1e-6,(name,collision.attrib['name'],'below ground',bottom)
    for joint in model.findall('joint'):
        assert joint.findtext('parent') in links and joint.findtext('child') in links
        assert joint.attrib['type']=='fixed'
    for mesh in model.findall('.//mesh'):
        uri=mesh.findtext('uri');assert uri.startswith('model://robot/')
        path=ROOT/'models/robot'/uri.removeprefix('model://robot/')
        assert path.is_file(),uri
    triangles={}
    for path in sorted((ROOT/'models/robot/meshes').glob('*.stl')):
        content=path.read_bytes();count=struct.unpack_from('<I',content,80)[0]
        assert len(content)==84+50*count and count>0,(path.name,'truncated STL')
        for row in struct.iter_unpack('<12fH',content[84:]):
            assert all(math.isfinite(x) for x in row[:12])
            assert all(abs(x)<2 for x in row[3:12]),(path.name,'not metre scale')
        triangles[path.name]=count
    colors=('metal','yellow','black','blue','green','glass')
    assert sum(triangles[f'body_{color}.stl'] for color in colors)==triangles['body.stl']
    for color in colors:
        visual=model.find(f".//visual[@name='cad_body_{color}']")
        assert visual is not None
        assert visual.findtext('geometry/mesh/uri')==f'model://robot/meshes/body_{color}.stl'
    assert float(model.find(".//visual[@name='cad_body_glass']").findtext('transparency'))>0
    include=world.find('include');assert include.findtext('uri')=='model://robot'
    assert numbers(include.findtext('pose'))[2]>0
    total_com=[v/mass for v in weighted_center]
    g=json.loads((ROOT/'config/geometry.json').read_text())
    assert abs(total_com[0])<g['track_contact_axis_distance_m']/2
    assert abs(total_com[1])<g['tracks_separation_m']/2
    camera=model.find('.//sensor[@type="camera"]');assert camera is not None
    assert camera.findtext('camera/camera_info_topic')=='/robot/camera/camera_info'
    assert camera.findtext('topic')=='/robot/camera/image'
    report={'asset_checks':'passed','total_mass_kg':mass,'total_center_of_mass_proxy_m':total_com,'mesh_triangles':triangles,
            'gazebo_runtime_motion_and_camera':'not covered by asset validation; run check_motion.sh with start_sim.sh',
            'runtime_check_command':'bash check_motion.sh (after bash start_sim.sh)'}
    print(json.dumps(report,ensure_ascii=False,indent=2))
    (ROOT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    main()
