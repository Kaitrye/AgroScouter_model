#!/usr/bin/env python3
"""Export the labelled front-plate assembly from camera_mount.blend."""
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
COLORS = ('yellow', 'green', 'black', 'imu')
PART_COLORS = {
    'yellow_front_plate': 'yellow',
    'camera_mount_crossbar': 'green',
    'camera_mount_arm_left': 'green',
    'camera_mount_arm_right': 'green',
    'downward_camera_housing': 'black',
    'imu_sensor_module': 'imu',
}


def triangles_for(obj):
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh = evaluated.to_mesh()
    try:
        mesh.calc_loop_triangles()
        for face in mesh.loop_triangles:
            points = [evaluated.matrix_world @ mesh.vertices[index].co for index in face.vertices]
            normal = (points[1] - points[0]).cross(points[2] - points[0]).normalized()
            yield (*normal, *points[0], *points[1], *points[2])
    finally:
        evaluated.to_mesh_clear()


def write_stl(path, triangles):
    with path.open('wb') as stream:
        stream.write(b'camera_mount.blend'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack('<12fH', *triangle, 0))


def world_bounds(obj):
    points = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    return ([min(point[i] for point in points) for i in range(3)],
            [max(point[i] for point in points) for i in range(3)])


def main():
    if Path(bpy.data.filepath).resolve() != (ROOT / 'camera_mount.blend').resolve():
        raise ValueError('Open camera_mount.blend before exporting')
    assembly = bpy.data.objects.get('front_plate_assembly')
    expected = set(PART_COLORS) | {'front_plate_assembly'}
    missing = expected - set(bpy.data.objects.keys())
    if missing or assembly is None:
        raise ValueError(f'Missing named assembly parts: {sorted(missing)}')
    if any(bpy.data.objects[name].parent != assembly for name in PART_COLORS):
        raise ValueError('All plate, mount, camera, and IMU objects must be grouped under front_plate_assembly')
    if any(not bpy.data.objects[name].get('label_ru') for name in PART_COLORS):
        raise ValueError('Every source mesh needs a Russian part label')
    if (bpy.data.objects['downward_camera_housing'].users_collection[0].name != 'Камера нижняя'
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
    plate = by_color['yellow'][0]
    lower, upper = world_bounds(plate)
    plate_pose = [MOUNT_ORIGIN[i] + (lower[i] + upper[i]) / 2 for i in range(3)]
    plate_size = [upper[i] - lower[i] for i in range(3)]
    params_path = ROOT / 'config/parameters.json'
    params = json.loads(params_path.read_text())
    params['camera']['pose_m_rad'] = camera_pose
    params['imu'] = {'pose_m': imu_pose, 'size_m': imu_size}
    params['camera_mount_plate_collision'] = {'pose_m': plate_pose, 'size_m': plate_size}
    params_path.write_text(json.dumps(params, ensure_ascii=False, indent=2) + '\n')
    print('Exported front-plate meshes:', {color: len(groups[color]) for color in COLORS})
    print('Camera pose in base_link:', camera_pose)
    print('IMU pose and size in base_link:', imu_pose, imu_size)
    print('Plate collision pose and size in base_link:', plate_pose, plate_size)


if __name__ == '__main__':
    main()
