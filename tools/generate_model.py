#!/usr/bin/env python3
"""Regenerate SDF from editable JSON parameters; no CAD packages required."""
import json
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sub(parent, tag, value=None, **attributes):
    e = ET.SubElement(parent, tag, {k: str(v) for k,v in attributes.items()})
    if value is not None:
        e.text = (' '.join(f'{x:.10g}' for x in value) if isinstance(value, (list,tuple)) else str(value))
    return e


def box_inertia(mass, size):
    x,y,z = size
    return [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]


def inertial(link, mass, size, center):
    e = sub(link, 'inertial')
    sub(e, 'pose', list(center)+[0,0,0]); sub(e, 'mass', mass)
    i = sub(e, 'inertia')
    for tag,value in zip(('ixx','iyy','izz'),box_inertia(mass,size)):
        sub(i,tag,value)
    for tag in ('ixy','ixz','iyz'):
        sub(i,tag,0)


def material(visual, rgba):
    m = sub(visual,'material')
    sub(m,'ambient',rgba); sub(m,'diffuse',rgba)
    if rgba[3]<1:
        sub(visual,'transparency',1-rgba[3])
    else:
        sub(m,'specular',[.3,.3,.3,1])


def primitive(link, kind, name, center, size=None, radius=None, length=None, rpy=(0,0,0), mu=None, color=None):
    e = sub(link,kind,name=name)
    sub(e,'pose',list(center)+list(rpy))
    g = sub(e,'geometry')
    if size is not None:
        sub(sub(g,'box'),'size',size)
    else:
        c=sub(g,'cylinder');sub(c,'radius',radius);sub(c,'length',length)
    if mu is not None:
        f=sub(sub(sub(e,'surface'),'friction'),'ode')
        sub(f,'mu',mu[0]); sub(f,'mu2',mu[1]);sub(f,'fdir1',[1,0,0])
    if color is not None:
        material(e,color)
    return e


def plugin(parent, system, filename):
    return sub(parent,'plugin',name='gz::sim::systems::'+system,filename='gz-sim-'+filename+'-system')


def fixed(model, parent, child):
    joint=sub(model,'joint',name=child+'_mount',type='fixed')
    sub(joint,'parent',parent);sub(joint,'child',child)


def write_xml(tree, file):
    ET.indent(tree,space='  ')
    ET.ElementTree(tree).write(file,encoding='utf-8',xml_declaration=True)


def track_contour(mesh_file):
    """Clockwise XZ silhouette of a binary CAD STL, starting at the front bottom."""
    with mesh_file.open('rb') as stream:
        stream.seek(80)
        triangle_count=struct.unpack('<I',stream.read(4))[0]
        triangles=stream.read()
    if len(triangles)!=triangle_count*50:
        raise ValueError(f'Invalid binary STL: {mesh_file}')
    points=set()
    for triangle in struct.iter_unpack('<12fH',triangles):
        for index in (3,6,9):
            points.add((triangle[index],triangle[index+2]))
    points=sorted(points)
    def cross(a,b,c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower=[]
    for point in points:
        while len(lower)>1 and cross(lower[-2],lower[-1],point)<=0:
            lower.pop()
        lower.append(point)
    upper=[]
    for point in reversed(points):
        while len(upper)>1 and cross(upper[-2],upper[-1],point)<=0:
            upper.pop()
        upper.append(point)
    contour=list(reversed(lower[:-1]+upper[:-1]))
    bottom=min(z for x,z in contour)
    start=max((i for i,(x,z) in enumerate(contour) if z<bottom+.003),
              key=lambda i:contour[i][0])
    return contour[start:]+contour[:start]


def contour_length(contour):
    return sum(math.dist(a,b) for a,b in zip(contour,contour[1:]+contour[:1]))


def tread_pose(distance, contour, center_z, outward_offset=.004):
    """Pose of a visual tread along the CAD silhouette at a given arc length."""
    remaining=distance%contour_length(contour)
    for (x1,z1),(x2,z2) in zip(contour,contour[1:]+contour[:1]):
        segment=math.hypot(x2-x1,z2-z1)
        if remaining<=segment:
            dx=(x2-x1)/segment;dz=(z2-z1)/segment
            x=x1+dx*remaining-dz*outward_offset
            z=z1+dz*remaining+dx*outward_offset
            return [x,0,z-center_z,0,math.atan2(-dz,dx),0]
        remaining-=segment
    raise AssertionError('Contour has no usable segments')



def write_corner_cap_mesh(config, path):
    """Write four rounded protective pads as one binary STL in base_link coordinates."""
    sx, sy, sz = config['size_m']
    radius = config['corner_radius_m']
    bevel = config['edge_bevel_m']
    if (len(config['x_centers_m']) != 2 or min(sx, sy, sz) <= 0
            or not 0 < bevel < radius < min(sx, sy) / 2
            or sz <= 2 * bevel or config['y_abs_m'] <= 0):
        raise ValueError('Invalid upper corner cap dimensions')

    def outline(cx, cy, half_x, half_y, r, z):
        points = []
        for ox, oy, start in ((half_x-r, half_y-r, 0),
                              (-half_x+r, half_y-r, 90),
                              (-half_x+r, -half_y+r, 180),
                              (half_x-r, -half_y+r, 270)):
            for step in range(7):
                a = math.radians(start + 90 * step / 6)
                points.append((cx + ox + r*math.cos(a),
                               cy + oy + r*math.sin(a), z))
        return points

    triangles = []
    for cx in config['x_centers_m']:
        for sign in (-1, 1):
            cy = sign * config['y_abs_m']
            z0 = config['z_center_m'] - sz / 2
            z1 = config['z_center_m'] + sz / 2
            rings = [outline(cx,cy,sx/2-bevel,sy/2-bevel,radius-bevel,z0),
                     outline(cx,cy,sx/2,sy/2,radius,z0+bevel),
                     outline(cx,cy,sx/2,sy/2,radius,z1-bevel),
                     outline(cx,cy,sx/2-bevel,sy/2-bevel,radius-bevel,z1)]
            count = len(rings[0])
            bottom = (cx, cy, z0)
            top = (cx, cy, z1)
            for j in range(count):
                k = (j + 1) % count
                triangles.append((bottom, rings[0][k], rings[0][j]))
                triangles.append((top, rings[-1][j], rings[-1][k]))
                for a, b in zip(rings, rings[1:]):
                    triangles.append((a[j], a[k], b[k]))
                    triangles.append((a[j], b[k], b[j]))
    with path.open('wb') as out:
        out.write(b'photo-based rounded upper corner protectors'.ljust(80,b' '))
        out.write(struct.pack('<I', len(triangles)))
        for a,b,c in triangles:
            u=[b[i]-a[i] for i in range(3)]
            v=[c[i]-a[i] for i in range(3)]
            normal=[u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2],
                    u[0]*v[1]-u[1]*v[0]]
            length=math.sqrt(sum(q*q for q in normal))
            if length <= 1e-15:
                raise ValueError('Degenerate corner protector mesh')
            out.write(struct.pack('<12fH', *(q/length for q in normal), *a, *b, *c, 0))


def generate():
    p=json.loads((ROOT/'config/parameters.json').read_text())
    g=json.loads((ROOT/'config/geometry.json').read_text())
    palette=p['visual_palette']
    for name in ('metal','yellow','black','blue','green','glass','rubber','imu',
                 'imu_blue','imu_white','imu_black','imu_gold','imu_red','imu_green','imu_yellow'):
        rgba=palette[name]
        if len(rgba)!=4 or any(not math.isfinite(v) or v<0 or v>1 for v in rgba):
            raise ValueError(f'Invalid visual color: {name}')
    write_corner_cap_mesh(p['upper_corner_caps'], ROOT/'models/robot/meshes/body_corner_caps.stl')
    if p['visual_mode'] not in ('cad','simplified'):
        raise ValueError('visual_mode must be cad or simplified')
    scalar_values=[p['body_mass_kg'],p['track_mass_kg_each'],p['steering_efficiency'],p['command_timeout_s'],p['max_linear_speed_m_s'],p['max_angular_speed_rad_s'],p['max_track_speed_m_s'],p['max_linear_acceleration_m_s2'],p['max_angular_acceleration_rad_s2']]
    if not all(math.isfinite(v) and v>0 for v in scalar_values):
        raise ValueError('Physical parameters must be positive finite numbers')
    if not 0<p['steering_efficiency']<=1:
        raise ValueError('steering_efficiency must be in (0,1]')
    if not all(math.isfinite(v) and v>=0 for v in p['track_friction']):
        raise ValueError('Friction must be nonnegative')
    camera=p['camera']
    if not 0<camera['horizontal_fov_rad']<math.pi or not 0<camera['near_m']<camera['far_m']:
        raise ValueError('Invalid camera FOV or clipping range')
    if camera['width_px']<=0 or camera['height_px']<=0 or camera['fps']<=0:
        raise ValueError('Invalid camera dimensions or frame rate')
    for b in p['body_collision_boxes']:
        if not all(math.isfinite(v) and v>0 for v in b['size_m']):
            raise ValueError('Collision box size must be positive')
    if not all(math.isfinite(v) and v>0 for v in p['body_inertia_box_size_m']):
        raise ValueError('Inertia proxy size must be positive')
    if not all(math.isfinite(v) for v in p['body_com_m']+camera['pose_m_rad']):
        raise ValueError('Centre of mass and camera pose must be finite')
    under=p['underbody'];lidar=under['lidar']
    if not 0<lidar['range_min_m']<lidar['range_max_m'] or lidar['samples']<2 or lidar['fps']<=0:
        raise ValueError('Invalid underbody lidar scan settings')
    imu=p['imu']
    if imu['fps']<=0 or not all(math.isfinite(v) for v in imu['pose_m']+imu['rpy_rad']):
        raise ValueError('Invalid IMU pose or update rate')
    sdf=ET.Element('sdf',version='1.9')
    model=sub(sdf,'model',name='robot',canonical_link='base_link')
    sub(model,'static','false');sub(model,'self_collide','false')
    body=sub(model,'link',name='base_link')
    inertial(body,p['body_mass_kg'],p['body_inertia_box_size_m'],p['body_com_m'])
    for i,b in enumerate(p['body_collision_boxes']):
        primitive(body,'collision',f'body_{i}',b['center_m'],size=b['size_m'])
        if p['visual_mode']=='simplified':
            primitive(body,'visual',f'body_{i}',b['center_m'],size=b['size_m'],color=palette['metal'])
    if p['visual_mode']=='cad':
        for color in ('metal','yellow','black','blue','green','glass'):
            mesh_file=ROOT/f'models/robot/meshes/body_{color}.stl'
            if not mesh_file.exists():
                if color in ('blue', 'green'):
                    continue  # These CAD equipment meshes may be absent after cleanup.
                raise FileNotFoundError(f'{mesh_file}; run python3 tools/colorize_mesh.py')
            visual=sub(body,'visual',name='cad_body_'+color)
            mesh=sub(sub(visual,'geometry'),'mesh')
            sub(mesh,'uri',f'model://robot/meshes/body_{color}.stl')
            material(visual,palette[color])
        lidar_mesh=ROOT/'models/robot/meshes/body_lidar.stl'
        if not lidar_mesh.is_file():
            raise FileNotFoundError(f'{lidar_mesh}; run python3 tools/colorize_mesh.py')
        lidar_visual=sub(body,'visual',name='cad_underbody_lidar')
        sub(lidar_visual,'pose',p['underbody']['lidar']['cad_visual_offset_m']+[0,0,0])
        mesh=sub(sub(lidar_visual,'geometry'),'mesh')
        sub(mesh,'uri','model://robot/meshes/body_lidar.stl')
        material(lidar_visual,palette['black'])
        for name,shift in (('center_camera',p['underbody']['center_camera_visual_offset_m']),('electronics_black',0)):
            mesh_file=ROOT/f'models/robot/meshes/body_{name}.stl'
            if not mesh_file.is_file():
                raise FileNotFoundError(f'{mesh_file}; run python3 tools/colorize_mesh.py')
            visual=sub(body,'visual',name='cad_'+name)
            sub(visual,'pose',[0,0,shift,0,0,0])
            mesh=sub(sub(visual,'geometry'),'mesh')
            sub(mesh,'uri',f'model://robot/meshes/body_{name}.stl')
            material(visual,palette['black'])
    caps_visual=sub(body,'visual',name='upper_corner_protectors')
    mesh=sub(sub(caps_visual,'geometry'),'mesh')
    sub(mesh,'uri','model://robot/meshes/body_corner_caps.stl')
    material(caps_visual,palette['black'])
    caps=p['upper_corner_caps']
    for position,cx in (('rear',caps['x_centers_m'][0]),('front',caps['x_centers_m'][1])):
        for side,sign in (('right',-1),('left',1)):
            primitive(body,'collision',f'upper_corner_{position}_{side}_collision',
                      [cx,sign*caps['y_abs_m'],caps['z_center_m']],size=caps['size_m'])
    # The editable front assembly is exported from robot_visual_edit.blend. Its
    # mesh coordinates are local to the centre of the yellow plate.
    plate=p['camera_mount_plate_collision']
    primitive(body,'collision','lower_camera_plate',plate['pose_m'],size=plate['size_m'])
    for color in ('yellow','green','black','dark'):
        mesh_file=ROOT/f'models/robot/meshes/camera_mount_{color}.stl'
        if not mesh_file.is_file():
            raise FileNotFoundError(f'{mesh_file}; export robot_visual_edit.blend first')
        visual=sub(body,'visual',name=f'camera_mount_{color}')
        sub(visual,'pose',[.245,0,.607,0,0,0])
        mesh=sub(sub(visual,'geometry'),'mesh')
        sub(mesh,'uri',f'model://robot/meshes/camera_mount_{color}.stl')
        material(visual,palette['black' if color=='dark' else color])
    imu=p['imu']
    for color in ('imu','imu_blue','imu_white','imu_black','imu_gold',
                  'imu_red','imu_green','imu_yellow'):
        visual=sub(body,'visual',name='imu_sensor_visual' if color=='imu' else f'imu_sensor_{color}')
        sub(visual,'pose',[.245,0,.607,0,0,0])
        mesh=sub(sub(visual,'geometry'),'mesh')
        sub(mesh,'uri',f'model://robot/meshes/camera_mount_{color}.stl')
        material(visual,palette[color])
    primitive(body,'collision','imu_sensor_collision',imu['pose_m'],size=imu['size_m'])
    imu_sensor=sub(body,'sensor',name='front_imu',type='imu')
    sub(imu_sensor,'pose',imu['pose_m']+imu['rpy_rad'])
    sub(imu_sensor,'always_on','true');sub(imu_sensor,'update_rate',imu['fps'])
    sub(imu_sensor,'topic','/robot/imu/data')
    under=p['underbody']
    lidar=under['lidar']
    lower=under['lower_plate']
    x0,x1=lower['x_range_m'];y0,y1=lower['y_range_m']
    if not x0<x1 or not y0<y1 or lower['thickness_m']<=0:
        raise ValueError('Invalid unified lower plate')
    if (max(abs(a-b) for a,b in zip(
            [x1-x0,y1-y0,lower['thickness_m']],plate['size_m']))>1e-5
            or max(abs(a-b) for a,b in zip(
                [(x0+x1)/2,(y0+y1)/2,lower['z_m']],plate['pose_m']))>1e-5):
        raise ValueError('Lower plate and Blender source disagree')
    offset=lidar['cad_visual_offset_m']
    if len(offset)!=3 or not all(math.isfinite(v) for v in offset):
        raise ValueError('Invalid CAD lidar offset')
    side=under['side_cylinders']
    for label,sign in (('left',1),('right',-1)):
        sx,sy,sz=side['x_m'],sign*side['y_abs_m'],side['z_m']
        mount_y=sign*(side['y_abs_m']+side['length_m']/2+.010)
        primitive(body,'visual',f'underbody_{label}_mount',[sx,mount_y,sz],size=[.075,.008,.075],color=palette['metal'])
        primitive(body,'visual',f'underbody_{label}_cylinder',[sx,sy,sz],radius=side['radius_m'],length=side['length_m'],rpy=(math.pi/2,0,0),color=palette['black'])
        primitive(body,'visual',f'underbody_{label}_lens',[sx,sign*(side['y_abs_m']-side['length_m']/2-.002),sz],radius=.021,length=.003,rpy=(math.pi/2,0,0),color=palette['glass'])
    sensor=sub(body,'sensor',name='underbody_lidar',type='gpu_lidar')
    sub(sensor,'pose',lidar['pose_m']+[0,0,0]);sub(sensor,'always_on','true');sub(sensor,'update_rate',lidar['fps'])
    sub(sensor,'topic','/robot/lidar/scan');sub(sensor,'visualize','false')
    ray=sub(sensor,'ray');scan=sub(ray,'scan')
    horizontal=sub(scan,'horizontal')
    sub(horizontal,'samples',lidar['samples']);sub(horizontal,'resolution',1)
    sub(horizontal,'min_angle',-math.pi);sub(horizontal,'max_angle',math.pi)
    vertical=sub(scan,'vertical')
    sub(vertical,'samples',1);sub(vertical,'resolution',1)
    sub(vertical,'min_angle',0);sub(vertical,'max_angle',0)
    scan_range=sub(ray,'range')
    sub(scan_range,'min',lidar['range_min_m']);sub(scan_range,'max',lidar['range_max_m']);sub(scan_range,'resolution',.01)
    track_size=g['track_envelope_size_m']
    axis_distance=g['track_contact_axis_distance_m']
    radius=(track_size[0]-axis_distance)/2
    if not 0<radius<track_size[2]/2:
        raise ValueError('Invalid track collision proxy dimensions')
    tread_count=48
    contours={side:track_contour(ROOT/f'models/robot/meshes/{side}_track.stl')
              for side in ('left','right')}
    for side,sign in [('left',1),('right',-1)]:
        name=side+'_track'
        contour=contours[side]
        tread_perimeter=contour_length(contour)
        tread_length=tread_perimeter/tread_count*.75
        position=[0,sign*g['tracks_separation_m']/2,track_size[2]/2]
        link=sub(model,'link',name=name)
        sub(link,'pose',position+[0,0,0])
        inertial(link,p['track_mass_kg_each'],track_size,[0,0,0])
        proxy=[('bottom',[0,0,radius-position[2]],[axis_distance,track_size[1],radius*2],None),
               ('front',[axis_distance/2,0,radius-position[2]],None,radius),
               ('rear',[-axis_distance/2,0,radius-position[2]],None,radius),
               ('upper',[0,0,track_size[2]*.71-position[2]],[track_size[0]*.52,track_size[1],track_size[2]*.56],None)]
        for cname,center,size,r in proxy:
            kwargs={'size':size} if size is not None else {'radius':r,'length':track_size[1],'rpy':(math.pi/2,0,0)}
            primitive(link,'collision',cname,center,mu=p['track_friction'],**kwargs)
            if p['visual_mode']=='simplified':
                primitive(link,'visual',cname,center,color=palette['rubber'],**kwargs)
        if p['visual_mode']=='cad':
            visual=sub(link,'visual',name='cad_'+name)
            sub(visual,'pose',[-v for v in position]+[0,0,0])
            mesh=sub(sub(visual,'geometry'),'mesh');sub(mesh,'uri',f'model://robot/meshes/{name}.stl')
            material(visual,palette['rubber'])
        for i in range(tread_count):
            pose=tread_pose(i*tread_perimeter/tread_count,contour,track_size[2]/2)
            primitive(link,'visual',f'tread_{name}_{i}',pose[:3],rpy=pose[3:],
                      size=[tread_length,.18,.012],color=palette['rubber'])
        fixed(model,'base_link',name)
        track=plugin(model,'TrackController','track-controller')
        sub(track,'link',name);sub(track,'track_orientation',[0,0,0])
        sub(track,'min_velocity',-p['max_track_speed_m_s']);sub(track,'max_velocity',p['max_track_speed_m_s'])
        sub(track,'max_command_age',p['command_timeout_s']+.3)
    vehicle=plugin(model,'TrackedVehicle','tracked-vehicle')
    sub(sub(vehicle,'left_track'),'link','left_track');sub(sub(vehicle,'right_track'),'link','right_track')
    sub(vehicle,'tracks_separation',g['tracks_separation_m'])
    sub(vehicle,'steering_efficiency',p['steering_efficiency'])
    sub(vehicle,'topic','/model/robot/cmd_vel');sub(vehicle,'odom_topic','/model/robot/odometry')
    sub(vehicle,'tf_topic','/model/robot/tf');sub(vehicle,'frame_id','odom');sub(vehicle,'child_frame_id','base_link')
    sub(vehicle,'odom_publish_frequency',30)
    for tag,speed,acc in [('linear_velocity',p['max_linear_speed_m_s'],p['max_linear_acceleration_m_s2']),('angular_velocity',p['max_angular_speed_rad_s'],p['max_angular_acceleration_rad_s2'])]:
        limiter=sub(vehicle,tag)
        for k,value in [('min_velocity',-speed),('max_velocity',speed),('min_acceleration',-acc),('max_acceleration',acc)]:sub(limiter,k,value)
    animation=sub(model,'plugin',name='robot_sim::TrackAnimation',filename='robot_track_animation.so')
    sub(animation,'tread_count',tread_count)
    sub(animation,'track_height',track_size[2])
    for side in ('left','right'):
        path=sub(animation,f'{side}_contour')
        for x,z in contours[side]:
            point=sub(path,'point')
            sub(point,'x',x);sub(point,'z',z)
    ground_truth=plugin(model,'OdometryPublisher','odometry-publisher')
    sub(ground_truth,'dimensions',3);sub(ground_truth,'odom_frame','world');sub(ground_truth,'robot_base_frame','base_link')
    sub(ground_truth,'odom_topic','/model/robot/ground_truth/odometry')
    sub(ground_truth,'tf_topic','/model/robot/ground_truth/tf');sub(ground_truth,'odom_publish_frequency',30)
    sensor=sub(body,'sensor',name='down_camera',type='camera')
    sub(sensor,'pose',camera['pose_m_rad']);sub(sensor,'always_on','true');sub(sensor,'update_rate',camera['fps'])
    sub(sensor,'topic','/robot/camera/image');sub(sensor,'visualize','false')
    c=sub(sensor,'camera');sub(c,'horizontal_fov',camera['horizontal_fov_rad'])
    image=sub(c,'image');sub(image,'width',camera['width_px']);sub(image,'height',camera['height_px']);sub(image,'format','R8G8B8')
    clip=sub(c,'clip');sub(clip,'near',camera['near_m']);sub(clip,'far',camera['far_m'])
    sub(c,'camera_info_topic','/robot/camera/camera_info')
    write_xml(sdf,ROOT/'models/robot/model.sdf')
    config=ET.Element('model');sub(config,'name','CAD tracked robot');sub(config,'version','0.1')
    sub(config,'sdf','model.sdf',version='1.9');sub(config,'description','CAD geometry with provisional physics for ROS 2 Jazzy / Gazebo Harmonic.')
    write_xml(config,ROOT/'models/robot/model.config')
    world_sdf=ET.Element('sdf',version='1.9');world=sub(world_sdf,'world',name='robot_test')
    sub(world,'gravity',[0,0,-9.81])
    physics=sub(world,'physics',name='1ms',type='ignored');sub(physics,'max_step_size',.001);sub(physics,'real_time_factor',1)
    for system,filename in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster')]:plugin(world,system,filename)
    sensors=plugin(world,'Sensors','sensors');sub(sensors,'render_engine','ogre2')
    plugin(world,'Imu','imu')
    scene=sub(world,'scene');sub(scene,'ambient',[.6,.6,.6,1]);sub(scene,'background',[.85,.9,.95,1]);sub(scene,'shadows','true')
    sun=sub(world,'light',name='sun',type='directional');sub(sun,'pose',[0,0,10,0,0,0])
    sub(sun,'direction',[-.5,.2,-1]);sub(sun,'diffuse',[.9,.9,.9,1]);sub(sun,'specular',[.2,.2,.2,1]);sub(sun,'cast_shadows','true')
    ground=sub(world,'model',name='ground');sub(ground,'static','true');l=sub(ground,'link',name='ground_link')
    primitive(l,'collision','ground',[0,0,-.025],size=[30,30,.05],mu=[1,1])
    primitive(l,'visual','ground',[0,0,-.025],size=[30,30,.05],color=[.68,.7,.72,1])
    for name,pose,size,color in [
        ('wall_a',[4,0,1],[.1,8,2],[.8,.75,.65,1]),
        ('wall_b',[0,4,1],[8,.1,2],[.65,.75,.8,1]),
        ('obstacle',[2,1,.2],[.5,.5,.4],[.75,.3,.2,1]),
        ('low_step',[1.8,-1,.03],[.6,.7,.06],[.4,.45,.5,1]),
    ]:
        obstacle=sub(world,'model',name=name);sub(obstacle,'static','true');sub(obstacle,'pose',pose+[0,0,0])
        l=sub(obstacle,'link',name='link');primitive(l,'collision','collision',[0,0,0],size=size)
        primitive(l,'visual','visual',[0,0,0],size=size,color=color)
    include=sub(world,'include');sub(include,'uri','model://robot');sub(include,'name','robot')
    sub(include,'pose',[0,0,.01,0,0,0])
    write_xml(world_sdf,ROOT/'worlds/test_world.sdf')
    print('Generated model.sdf and test_world.sdf')


if __name__=='__main__':
    generate()
