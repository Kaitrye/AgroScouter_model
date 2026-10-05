#!/usr/bin/env python3
"""Sync the labelled source assembly into robot_visual_edit.blend."""
from pathlib import Path
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
FULL = ROOT / 'robot_visual_edit.blend'
MOUNT = ROOT / 'camera_mount.blend'
ORIGIN = Vector((0.245, 0, 0.607))

if Path(bpy.data.filepath).resolve() != FULL.resolve():
    raise ValueError('Open robot_visual_edit.blend before syncing camera mount')
if not MOUNT.is_file():
    raise FileNotFoundError(MOUNT)

for color in ('metal', 'yellow', 'black', 'blue', 'glass'):
    name = 'cad_body_' + color
    mesh_file = ROOT / f'models/robot/meshes/body_{color}.stl'
    if not mesh_file.is_file():
        if name in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)
        continue
    if name not in bpy.data.objects:
        continue
    bpy.ops.wm.stl_import(filepath=str(mesh_file))
    imported = bpy.context.active_object
    bpy.data.objects[name].data = imported.data
    bpy.data.objects.remove(imported, do_unlink=True)

for obj in list(bpy.data.objects):
    if (obj.name.startswith('camera_mount_') or obj.name.startswith('imu_sensor_')
            or obj.name.startswith('downward_camera_') or obj.name in {
                'front_plate_assembly', 'yellow_front_plate', 'camera_mount_crossbar',
                'camera_mount_arm_left', 'camera_mount_arm_right'}):
        bpy.data.objects.remove(obj, do_unlink=True)

with bpy.data.libraries.load(str(MOUNT), link=False) as (source, target):
    target.objects = list(source.objects)

visuals = bpy.data.collections.get('visual')
if visuals is None:
    raise ValueError('The robot Blender file has no visual collection')
base_link = bpy.data.objects['base_link']
assembly = None
for obj in target.objects:
    if obj is None:
        continue
    visuals.objects.link(obj)
    if obj.name == 'front_plate_assembly':
        assembly = obj
    if obj.type == 'MESH':
        if obj.name == 'yellow_front_plate':
            group_name = 'Площадка жёлтая'
        elif obj.name.startswith('camera_mount_'):
            group_name = 'Крепление камеры зелёное'
        elif obj.name == 'downward_camera_housing':
            group_name = 'Камера нижняя'
        elif obj.name == 'imu_sensor_module':
            group_name = 'IMU'
        else:
            continue
        group = bpy.data.collections.get(group_name)
        if group is None:
            group = bpy.data.collections.new(group_name)
            bpy.context.scene.collection.children.link(group)
        group.objects.link(obj)
if assembly is None:
    raise ValueError('The source assembly has no front_plate_assembly root')

# Move only the assembly root; its child offsets and parent relationships stay intact.
source_offset = assembly.matrix_world.to_translation().copy()
assembly_world = Matrix.Translation(ORIGIN + source_offset)
assembly.parent = base_link
assembly.matrix_parent_inverse = base_link.matrix_world.inverted()
assembly.matrix_world = assembly_world
assembly['label_ru'] = 'Передняя площадка, зелёное крепление, камера и IMU'

bpy.context.view_layer.update()
for name in ('camera_mount_arm_left', 'camera_mount_arm_right', 'camera_mount_crossbar',
             'downward_camera_housing', 'imu_sensor_module', 'yellow_front_plate'):
    obj = bpy.data.objects[name]
    if obj.parent != assembly:
        raise ValueError(f'{name} was detached from front_plate_assembly')

bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=str(FULL))
print('SYNCED_FRONT_PLATE_ASSEMBLY', len(target.objects))
