#!/usr/bin/env python3
"""Export the labelled front-plate assembly from the complete robot Blender model."""
import json
import math
import re
import struct
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / 'models/robot/meshes'
MOUNT_ORIGIN = Vector((0.245, 0.0, 0.607))
COLORS = ('yellow', 'green', 'black', 'dark', 'imu', 'imu_blue', 'imu_white',
          'imu_black', 'imu_gold', 'imu_red', 'imu_green', 'imu_yellow')
PART_COLORS = {
    'yellow_front_plate': 'yellow',
    'camera_mount_crossbar': 'green',
    'camera_mount_arm_left': 'green',
    'camera_mount_arm_right': 'green',
    'camera_mount_lens_ring': 'green',
    'camera_mount_lens_aperture': 'dark',
    'camera_mount_bolt_left_inner': 'dark',
    'camera_mount_bolt_left_outer': 'dark',
    'camera_mount_bolt_right_inner': 'dark',
    'camera_mount_bolt_right_outer': 'dark',
    'downward_camera_housing': 'black',
    'imu_sensor_module': 'imu',
    'imu_sensor_top_panel': 'imu_blue',
    'imu_sensor_border_front': 'imu_white',
    'imu_sensor_border_back': 'imu_white',
    'imu_sensor_border_left': 'imu_white',
    'imu_sensor_border_right': 'imu_white',
    'imu_sensor_axis_x': 'imu_white',
    'imu_sensor_axis_x_head_a': 'imu_white',
    'imu_sensor_axis_x_head_b': 'imu_white',
    'imu_sensor_axis_y': 'imu_white',
    'imu_sensor_axis_y_head_a': 'imu_white',
    'imu_sensor_axis_y_head_b': 'imu_white',
    'imu_sensor_axis_center': 'imu_white',
    'imu_sensor_status_red': 'imu_red',
    'imu_sensor_status_yellow': 'imu_yellow',
    'imu_sensor_status_green': 'imu_green',
    'imu_sensor_status_black': 'imu_black',
    'imu_sensor_gland_base': 'imu_black',
    'imu_sensor_gland_cap': 'imu_black',
    'imu_sensor_cable': 'imu_black',
    'imu_sensor_gold_port': 'imu_gold',
    'imu_sensor_gold_tip': 'imu_gold',
}


def triangles_for(obj):
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    try:
        mesh.calc_loop_triangles()
        for face in mesh.loop_triangles:
            points = [evaluated.matrix_world @ mesh.vertices[index].co - MOUNT_ORIGIN
                      for index in face.vertices]
            normal = (points[1] - points[0]).cross(points[2] - points[0]).normalized()
            yield (*normal, *points[0], *points[1], *points[2])
    finally:
        evaluated.to_mesh_clear()


def write_stl(path, triangles):
    with path.open('wb') as stream:
        stream.write(b'robot_visual_edit.blend'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack('<12fH', *triangle, 0))


def world_bounds(obj):
    points = [obj.matrix_world @ Vector(corner) - MOUNT_ORIGIN
              for corner in obj.bound_box]
    return ([min(point[i] for point in points) for i in range(3)],
            [max(point[i] for point in points) for i in range(3)])


def main():
    if Path(bpy.data.filepath).resolve() != (ROOT / 'robot_visual_edit.blend').resolve():
        raise ValueError('Open robot_visual_edit.blend before exporting')
    assembly = bpy.data.objects.get('front_plate_assembly')
    expected = set(PART_COLORS) | {'front_plate_assembly'}
    missing = expected - set(bpy.data.objects.keys())
    if missing or assembly is None:
        raise ValueError(f'Missing named assembly parts: {sorted(missing)}')
    if any(bpy.data.objects[name].parent != assembly for name in PART_COLORS):
        raise ValueError('All plate, mount, camera, and IMU objects must be grouped under front_plate_assembly')
    if any(not bpy.data.objects[name].get('label_ru') for name in PART_COLORS):
        raise ValueError('Every source mesh needs a Russian part label')
    if ('Камера нижняя' not in [c.name for c in bpy.data.objects['downward_camera_housing'].users_collection]
            or any('Крепление камеры зелёное' not in [c.name for c in bpy.data.objects[name].users_collection]
                   for name in ('camera_mount_crossbar','camera_mount_arm_left','camera_mount_arm_right'))):
        raise ValueError('Camera and green mount are not assigned to their named collections')

    groups = {color: [] for color in COLORS}
    by_color = {color: [] for color in COLORS}
    for name, color in PART_COLORS.items():
        obj = bpy.data.objects[name]
        by_color[color].append(obj)
        groups[color].extend(triangles_for(obj))
    if any(not groups[color] for color in COLORS) or len(by_color['black']) != 1 or len(by_color['imu']) != 1:
        raise ValueError('Expected plate, green mount, one camera housing, and one IMU package')
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for color in COLORS:
        write_stl(MESH_DIR / f'camera_mount_{color}.stl', groups[color])

    camera = by_color['black'][0]
    lower, upper = world_bounds(camera)
    camera_pose = [MOUNT_ORIGIN.x + (lower[0] + upper[0]) / 2,
                   MOUNT_ORIGIN.y + (lower[1] + upper[1]) / 2,
                   MOUNT_ORIGIN.z + lower[2] - 0.001, 0, math.pi / 2, 0]
    imu = by_color['imu'][0]
    lower, upper = world_bounds(imu)
    imu_pose = [MOUNT_ORIGIN[i] + (lower[i] + upper[i]) / 2 for i in range(3)]
    imu_size = [upper[i] - lower[i] for i in range(3)]
    imu_rpy = list(imu.matrix_world.to_euler('XYZ'))
    plate = by_color['yellow'][0]
    lower, upper = world_bounds(plate)
    plate_pose = [MOUNT_ORIGIN[i] + (lower[i] + upper[i]) / 2 for i in range(3)]
    plate_size = [upper[i] - lower[i] for i in range(3)]
    params_path = ROOT / 'config/parameters.json'
    params = json.loads(params_path.read_text())
    params['camera']['pose_m_rad'] = camera_pose
    params['imu'] = {'pose_m': imu_pose, 'size_m': imu_size, 'rpy_rad': imu_rpy,
                     'fps': params.get('imu', {}).get('fps', 100)}
    params['camera_mount_plate_collision'] = {'pose_m': plate_pose, 'size_m': plate_size}
    params['underbody']['lower_plate'] = {
        'x_range_m': [MOUNT_ORIGIN.x + lower[0], MOUNT_ORIGIN.x + upper[0]],
        'y_range_m': [MOUNT_ORIGIN.y + lower[1], MOUNT_ORIGIN.y + upper[1]],
        'z_m': plate_pose[2], 'thickness_m': plate_size[2]}
    params_path.write_text(json.dumps(params, ensure_ascii=False, indent=2) + '\n')
    print('Exported front-plate meshes:', {color: len(groups[color]) for color in COLORS})
    print('Camera pose in base_link:', camera_pose)
    print('IMU pose, size and RPY in base_link:', imu_pose, imu_size, imu_rpy)
    print('Plate collision pose and size in base_link:', plate_pose, plate_size)


if __name__ == '__main__':
    main()
