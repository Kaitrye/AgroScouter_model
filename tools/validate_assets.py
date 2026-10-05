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



def triangle_intersects_box(points,center,half):
    vertices=[[point[i]-center[i] for i in range(3)] for point in points]
    edges=[[vertices[(j+1)%3][i]-vertices[j][i] for i in range(3)] for j in range(3)]
    axes=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    axes.append([edges[0][1]*edges[1][2]-edges[0][2]*edges[1][1],
                 edges[0][2]*edges[1][0]-edges[0][0]*edges[1][2],
                 edges[0][0]*edges[1][1]-edges[0][1]*edges[1][0]])
    unit=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    axes.extend([[e[1]*u[2]-e[2]*u[1],e[2]*u[0]-e[0]*u[2],e[0]*u[1]-e[1]*u[0]]
                 for e in edges for u in unit])
    for axis in axes:
        if sum(v*v for v in axis)<1e-18:
            continue
        projections=[sum(v[i]*axis[i] for i in range(3)) for v in vertices]
        radius=sum(half[i]*abs(axis[i]) for i in range(3))
        if min(projections)>radius+1e-8 or max(projections)<-radius-1e-8:
            return False
    return True

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
                if abs(abs(pose[3])-math.pi/2)<1e-8:
                    bottom=lp[2]+pose[2]-r
                else:
                    assert abs(pose[3])<1e-8
                    bottom=lp[2]+pose[2]-length/2
            assert bottom>=-1e-6,(name,collision.attrib['name'],'below ground',bottom)
    for joint in model.findall('joint'):
        assert joint.findtext('parent') in links and joint.findtext('child') in links
        assert joint.attrib['type']=='fixed'
    for mesh in model.findall('.//mesh'):
        uri=mesh.findtext('uri');assert uri.startswith('model://robot/')
        path=ROOT/'models/robot'/uri.removeprefix('model://robot/')
        assert path.is_file(),uri
    triangles={}
    mesh_bounds={}
    body_visual_top=-math.inf
    for path in sorted((ROOT/'models/robot/meshes').glob('*.stl')):
        content=path.read_bytes();count=struct.unpack_from('<I',content,80)[0]
        assert len(content)==84+50*count and count>0,(path.name,'truncated STL')
        bounds=[[math.inf,-math.inf] for _ in range(3)] if path.name in ('camera_mount_black.stl','camera_mount_imu.stl','camera_mount_yellow.stl','body_yellow.stl','body_lidar.stl','body_corner_caps.stl','body_center_camera.stl') else None
        for row in struct.iter_unpack('<12fH',content[84:]):
            assert all(math.isfinite(x) for x in row[:12])
            if bounds is not None:
                for axis in range(3):
                    for offset in (3,6,9):
                        value=row[offset+axis]
                        bounds[axis][0]=min(bounds[axis][0],value)
                        bounds[axis][1]=max(bounds[axis][1],value)
            assert all(abs(x)<2 for x in row[3:12]),(path.name,'not metre scale')
            if path.name.startswith('body_') and path.name != 'body_corner_caps.stl':
                body_visual_top=max(body_visual_top,row[5],row[8],row[11])
        triangles[path.name]=count
        if bounds is not None:mesh_bounds[path.name]=bounds
    assert body_visual_top<.816,(body_visual_top,'old upper lid or mast remains')
    colors=('metal','yellow','black','blue','green','glass')
    visible_triangles=sum(triangles.get(f'body_{color}.stl',0) for color in colors)
    assert 200000 < triangles['body.stl']-visible_triangles < 300000
    motor_vertices={-1:0,1:0}
    for row in struct.iter_unpack('<12fH', (ROOT/'models/robot/meshes/body_black.stl').read_bytes()[84:]):
        for offset in (3,6,9):
            x,y,z=row[offset:offset+3]
            if -.08 < x < .04 and .44 < z < .58 and .34 < abs(y) < .43:
                motor_vertices[1 if y > 0 else -1] += 1
    assert min(motor_vertices.values()) > 1000,('side motor mesh missing',motor_vertices)
    # The lower camera is an exact rotated copy of STEP component 13.
    assert triangles['cad_upper_camera.stl']==triangles['camera_mount_black.stl']==1668
    camera_mesh=mesh_bounds['camera_mount_black.stl']
    camera_size=[axis[1]-axis[0] for axis in camera_mesh]
    assert max(abs(a-b) for a,b in zip(camera_size,(.0486,.04,.022)))<.0002
    for color in colors:
        visual=model.find(f".//visual[@name='cad_body_{color}']")
        if color in ('blue','green') and f'body_{color}.stl' not in triangles:
            assert visual is None
            continue
        assert visual is not None
        assert visual.findtext('geometry/mesh/uri')==f'model://robot/meshes/body_{color}.stl'
    assert float(model.find(".//visual[@name='cad_body_glass']").findtext('transparency'))>0
    cap_params=json.loads((ROOT/'config/parameters.json').read_text())['upper_corner_caps']
    cap_visual=model.find(".//visual[@name='upper_corner_protectors']")
    assert cap_visual is not None
    assert cap_visual.findtext('geometry/mesh/uri')=='model://robot/meshes/body_corner_caps.stl'
    assert triangles['body_corner_caps.stl']==896
    cap_bounds=mesh_bounds['body_corner_caps.stl']
    expected_x=(cap_params['x_centers_m'][0]-cap_params['size_m'][0]/2,
                cap_params['x_centers_m'][1]+cap_params['size_m'][0]/2)
    assert max(abs(a-b) for a,b in zip(cap_bounds[0],expected_x))<1e-5
    assert abs(cap_bounds[1][0]+cap_params['y_abs_m']+cap_params['size_m'][1]/2)<1e-5
    assert abs(cap_bounds[1][1]-cap_params['y_abs_m']-cap_params['size_m'][1]/2)<1e-5
    assert abs(cap_bounds[2][1]-cap_params['z_center_m']-cap_params['size_m'][2]/2)<1e-5
    for position,cx in (('rear',cap_params['x_centers_m'][0]),
                        ('front',cap_params['x_centers_m'][1])):
        for side,sign in (('right',-1),('left',1)):
            collision=model.find(f".//collision[@name='upper_corner_{position}_{side}_collision']")
            assert collision is not None
            assert numbers(collision.findtext('geometry/box/size'))==cap_params['size_m']
            assert max(abs(a-b) for a,b in zip(numbers(collision.findtext('pose'))[:3],
                       (cx,sign*cap_params['y_abs_m'],cap_params['z_center_m'])))<1e-6
    camera=model.find(".//sensor[@name='down_camera']")
    assert camera is not None
    camera_pose=numbers(camera.findtext('pose'))
    assert abs(camera_pose[4]-math.pi/2)<1e-8
    mount_pose=[.245,0,.607]
    for color in ('yellow','green','black','dark'):
        visual=model.find(f".//visual[@name='camera_mount_{color}']")
        assert visual is not None
        assert numbers(visual.findtext('pose'))[:3]==mount_pose
        assert visual.findtext('geometry/mesh/uri')==f'model://robot/meshes/camera_mount_{color}.stl'
    plate_size=numbers(model.findtext(".//collision[@name='lower_camera_plate']/geometry/box/size"))
    assert camera_pose[1]>plate_size[1]/2+.001, 'working camera must hang beyond plate edge'
    housing=mesh_bounds['camera_mount_black.stl']
    for axis in (0,1):
        assert mount_pose[axis]+housing[axis][0] <= camera_pose[axis] <= mount_pose[axis]+housing[axis][1]
    assert 0 < mount_pose[2]+housing[2][0]-camera_pose[2] < .005
    imu=json.loads((ROOT/'config/parameters.json').read_text())['imu']
    imu_visual=model.find(".//visual[@name='imu_sensor_visual']")
    assert imu_visual is not None
    assert imu_visual.findtext('geometry/mesh/uri')=='model://robot/meshes/camera_mount_imu.stl'
    assert numbers(imu_visual.findtext('pose'))[:3]==mount_pose
    for color in ('blue','white','black','gold','red','green','yellow'):
        name=f'imu_sensor_imu_{color}'
        visual=model.find(f".//visual[@name='{name}']")
        assert visual is not None,name
        mesh=f'camera_mount_imu_{color}.stl'
        assert visual.findtext('geometry/mesh/uri')==f'model://robot/meshes/{mesh}'
        assert numbers(visual.findtext('pose'))[:3]==mount_pose
        assert triangles[mesh]>0
    plate_cfg=json.loads((ROOT/'config/parameters.json').read_text())['camera_mount_plate_collision']
    plate_collision=model.find(".//collision[@name='lower_camera_plate']")
    assert max(abs(a-b) for a,b in zip(numbers(plate_collision.findtext('pose'))[:3],plate_cfg['pose_m']))<1e-5
    assert max(abs(a-b) for a,b in zip(numbers(plate_collision.findtext('geometry/box/size')),plate_cfg['size_m']))<1e-5
    yellow=mesh_bounds['camera_mount_yellow.stl']
    assert abs((.245+yellow[0][0])+.190)<1e-5,'unified plate rear edge is misplaced'
    assert abs((.245+yellow[0][1])-.345)<1e-5,'yellow plate forward edge moved'
    assert abs(yellow[1][0]+plate_cfg['size_m'][1]/2)<1e-5
    assert abs(yellow[1][1]-plate_cfg['size_m'][1]/2)<1e-5
    assert abs(plate_cfg['pose_m'][2]-.570)<1e-5
    assert abs(plate_cfg['size_m'][1]-.370)<1e-5
    assert triangles['camera_mount_yellow.stl']==12, 'the lower plate must be one solid mesh'
    assert triangles['camera_mount_dark.stl']>100
    assert model.find(".//visual[@name='cad_body_yellow']") is not None
    center=numbers(plate_collision.findtext('pose'))[:3]
    half=[v/2 for v in numbers(plate_collision.findtext('geometry/box/size'))]
    metal=(ROOT/'models/robot/meshes/body_metal.stl').read_bytes()
    plate_frame_intersections=0
    for row in struct.iter_unpack('<12fH',metal[84:]):
        points=[row[3:6],row[6:9],row[9:12]]
        lo=[min(q[i] for q in points) for i in range(3)]
        hi=[max(q[i] for q in points) for i in range(3)]
        if any(lo[i]>center[i]+half[i] or hi[i]<center[i]-half[i] for i in range(3)):
            continue
        if triangle_intersects_box(points,center,half):
            plate_frame_intersections+=1
    assert plate_frame_intersections==0,('yellow plate intersects metal frame',plate_frame_intersections)
    imu_collision=model.find(".//collision[@name='imu_sensor_collision']")
    assert imu_collision is not None
    assert max(abs(a-b) for a,b in zip(numbers(imu_collision.findtext('pose'))[:3],imu['pose_m']))<1e-5
    assert max(abs(a-b) for a,b in zip(numbers(imu_collision.findtext('geometry/box/size')),imu['size_m']))<1e-5
    assert abs(imu['pose_m'][0]+imu['size_m'][0]/2-.345)<1e-5,'IMU is not at front plate edge'
    assert abs(imu['size_m'][0]-.048)<1e-5 and abs(imu['size_m'][1]-.060)<1e-5
    assert abs(imu['rpy_rad'][2]-math.pi/2)<1e-5
    imu_sensor=model.find(".//sensor[@name='front_imu']")
    assert imu_sensor is not None and imu_sensor.get('type')=='imu'
    assert imu_sensor.findtext('topic')=='/robot/imu/data'
    assert max(abs(a-b) for a,b in zip(numbers(imu_sensor.findtext('pose')),imu['pose_m']+imu['rpy_rad']))<1e-5
    assert float(imu_sensor.findtext('update_rate'))==imu['fps']
    assert imu_sensor.find('imu/orientation_reference_frame') is None
    assert world.find(".//plugin[@name='gz::sim::systems::Imu']") is not None
    imu_bridge=(ROOT/'config/imu_bridge.yaml').read_text()
    assert '/imu/data' in imu_bridge and '/robot/imu/data' in imu_bridge and 'sensor_msgs/msg/Imu' in imu_bridge
    under=json.loads((ROOT/'config/parameters.json').read_text())['underbody']
    assert model.find(".//visual[@name='soil_sensor_support']") is None
    # STEP components 9 and 27 are the lidar and highlighted central camera.
    # Side cameras 21–24 and the soil sensor remain hidden.
    parts=json.loads((ROOT/'config/cad_parts.json').read_text())
    for index,color,minimum in ((9,'lidar',900),(27,'center_camera',1448),
                                (21,'metal',0),(22,'black',0),(23,'metal',0),(24,'black',0),
                                (636,'black',0)):
        bbox=parts[index]['bbox']
        bounds=((bbox[2]+201.458483)/1000,(bbox[0]-595)/1000,(bbox[1]+76.445175)/1000,
                (bbox[5]+201.458483)/1000,(bbox[3]-595)/1000,(bbox[4]+76.445175)/1000)
        data=(ROOT/f'models/robot/meshes/body_{color}.stl').read_bytes()
        matching=sum(all(bounds[axis]-.0001<=row[offset+axis]<=bounds[axis+3]+.0001
                         for offset in (3,6,9) for axis in range(3))
                     for row in struct.iter_unpack('<12fH',data[84:]))
        assert (matching>=minimum if index in (9,27) else matching==0),(index,'unexpected CAD shell count',matching)
    assert triangles['body_lidar.stl']==900
    cad_lidar=model.find(".//visual[@name='cad_underbody_lidar']")
    assert cad_lidar is not None
    assert cad_lidar.findtext('geometry/mesh/uri')=='model://robot/meshes/body_lidar.stl'
    assert numbers(cad_lidar.findtext('pose'))[:3]==under['lidar']['cad_visual_offset_m']
    lower=under['lower_plate']
    assert max(abs(a-b) for a,b in zip(
        (lower['x_range_m'][0],lower['x_range_m'][1],
         lower['y_range_m'][0],lower['y_range_m'][1]),
        (.245+yellow[0][0],.245+yellow[0][1],yellow[1][0],yellow[1][1])))<1e-5
    assert lower['x_range_m'][1]-lower['x_range_m'][0]>.53
    assert .003<.5796-(lower['z_m']+lower['thickness_m']/2)<.015, 'lower plate should sit close below the raised iron bar'
    assert not model.findall(".//visual[@name='lower_yellow_plate_front']")
    assert not any(v.get('name','').startswith('lower_yellow_plate_') for v in model.findall('.//visual'))
    center_camera=model.find(".//visual[@name='cad_center_camera']")
    assert center_camera is not None
    assert center_camera.findtext('geometry/mesh/uri')=='model://robot/meshes/body_center_camera.stl'
    assert abs(numbers(center_camera.findtext('pose'))[2]-under['center_camera_visual_offset_m'])<1e-6
    assert triangles['body_center_camera.stl']==1448
    center_bounds=mesh_bounds['body_center_camera.stl']
    assert abs(center_bounds[2][1]+under['center_camera_visual_offset_m']-(lower['z_m']-lower['thickness_m']/2))<1e-5
    electronics=model.find(".//visual[@name='cad_electronics_black']")
    assert electronics is not None and abs(numbers(electronics.findtext('pose'))[2])<1e-6
    assert model.find(".//visual[@name='lidar_rear_bracket']") is None
    assert abs(mesh_bounds['body_lidar.stl'][2][1]+under['lidar']['cad_visual_offset_m'][2]-(lower['z_m']-lower['thickness_m']/2))<1e-5
    for name in ('underbody_plate_extension','lidar_mount','lidar_housing','lidar_lower',
                 'lidar_status_ring'):
        assert model.find(f".//visual[@name='{name}']") is None,(name,'approximate duplicate')
    lidar=model.find(".//sensor[@name='underbody_lidar']")
    assert lidar is not None and lidar.get('type')=='gpu_lidar'
    assert lidar.findtext('topic')=='/robot/lidar/scan'
    scan_pose=numbers(lidar.findtext('pose'))[:3]
    assert max(abs(a-b) for a,b in zip(scan_pose,under['lidar']['pose_m']))<1e-6
    lidar_box=parts[9]['bbox']
    offset=under['lidar']['cad_visual_offset_m']
    assert offset[0]<-.2,('lidar must be under rear crossbar',offset)
    assert (lidar_box[2]+201.458483)/1000+offset[0]<scan_pose[0]<(lidar_box[5]+201.458483)/1000+offset[0]
    assert (lidar_box[0]-595)/1000+offset[1]<scan_pose[1]<(lidar_box[3]-595)/1000+offset[1]
    assert (lidar_box[1]+76.445175)/1000+offset[2]<scan_pose[2]<(lidar_box[4]+76.445175)/1000+offset[2]
    assert abs(scan_pose[0]+.1194)<1e-6
    assert int(lidar.findtext('ray/scan/horizontal/samples'))==under['lidar']['samples']
    side=under['side_cylinders']
    assert abs(side['z_m']-scan_pose[2])<1e-9
    for label,sign in (('left',1),('right',-1)):
        mount=model.find(f".//visual[@name='underbody_{label}_mount']")
        cylinder=model.find(f".//visual[@name='underbody_{label}_cylinder']")
        lens=model.find(f".//visual[@name='underbody_{label}_lens']")
        assert mount is not None and cylinder is not None and lens is not None
        position=numbers(cylinder.findtext('pose'))
        assert abs(position[0]-side['x_m'])<1e-6
        assert abs(position[1]-sign*side['y_abs_m'])<1e-6
        assert abs(position[2]-scan_pose[2])<1e-6

    bridge=(ROOT/'config/lidar_bridge.yaml').read_text()
    assert '/scan' in bridge and '/robot/lidar/scan' in bridge and 'sensor_msgs/msg/LaserScan' in bridge
    include=world.find('include');assert include.findtext('uri')=='model://robot'
    assert numbers(include.findtext('pose'))[2]>0
    total_com=[v/mass for v in weighted_center]
    g=json.loads((ROOT/'config/geometry.json').read_text())
    assert abs(total_com[0])<g['track_contact_axis_distance_m']/2
    assert abs(total_com[1])<g['tracks_separation_m']/2
    assert len(model.findall('.//sensor[@type="camera"]'))==1
    assert camera.findtext('camera/camera_info_topic')=='/robot/camera/camera_info'
    assert camera.findtext('topic')=='/robot/camera/image'
    report={'asset_checks':'passed','total_mass_kg':mass,'total_center_of_mass_proxy_m':total_com,'mesh_triangles':triangles,
            'imu_hz':imu['fps'],
            'lidar_scan':{'samples':under['lidar']['samples'],'fps':under['lidar']['fps'],'range_m':[under['lidar']['range_min_m'],under['lidar']['range_max_m']]},
            'gazebo_runtime_motion_camera_lidar_and_imu':'not covered by asset validation; run check_motion.sh with start_sim.sh',
            'runtime_check_command':'bash check_motion.sh (after bash start_sim.sh)'}
    print(json.dumps(report,ensure_ascii=False,indent=2))
    (ROOT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    main()
