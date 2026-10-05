#!/usr/bin/env python3
"""Refresh CAD details in the complete robot Blender model."""
from pathlib import Path
import bpy
import json
import math
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
FULL = ROOT / 'robot_visual_edit.blend'
ORIGIN = Vector((0.245, 0, 0.607))

if Path(bpy.data.filepath).resolve() != FULL.resolve():
    raise ValueError('Open robot_visual_edit.blend before syncing camera mount')

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

visuals = bpy.data.collections.get('visual')
if visuals is None:
    raise ValueError('The robot Blender file has no visual collection')
base_link = bpy.data.objects['base_link']
assembly = bpy.data.objects.get('front_plate_assembly')
if assembly is None or assembly.parent != base_link:
    raise ValueError('The complete robot is missing its front-plate assembly')
for name in ('camera_mount_arm_left', 'camera_mount_arm_right', 'camera_mount_crossbar',
             'downward_camera_housing', 'imu_sensor_module', 'yellow_front_plate'):
    obj = bpy.data.objects.get(name)
    if obj is None or obj.parent != assembly:
        raise ValueError(f'{name} was detached from front_plate_assembly')
if 'cad_body_blue' in bpy.data.objects:
    bpy.data.objects['cad_body_blue'].matrix_world = base_link.matrix_world.copy()

# Replace the former hand-built underside with the CAD meshes loaded above.
for obj in list(bpy.data.objects):
    if (obj.name.startswith(('underbody_', 'lidar_', 'soil_sensor_', 'lower_yellow_plate_'))
            or obj.name in ('cad_underbody_lidar', 'cad_center_camera',
                            'cad_electronics_black', 'upper_corner_protectors')):
        bpy.data.objects.remove(obj, do_unlink=True)

params = json.loads((ROOT / 'config/parameters.json').read_text())
side = params['underbody']['side_cylinders']
palette = params['visual_palette']
for color in ('metal', 'black', 'glass', 'yellow'):
    material = bpy.data.materials.get('underbody_' + color) or bpy.data.materials.new('underbody_' + color)
    material.diffuse_color = tuple(palette[color])

# The four photo-based soft caps form one editable mesh in the visual scene.
bpy.ops.wm.stl_import(filepath=str(ROOT / 'models/robot/meshes/body_corner_caps.stl'))
caps_obj = bpy.context.active_object
caps_obj.name = 'upper_corner_protectors'
caps_obj.data.materials.clear()
caps_obj.data.materials.append(bpy.data.materials['underbody_black'])
for collection in list(caps_obj.users_collection):
    collection.objects.unlink(caps_obj)
visuals.objects.link(caps_obj)
caps_obj.parent = base_link
caps_obj.matrix_world = base_link.matrix_world.copy()
caps_obj['label_ru'] = 'Четыре мягкие защитные накладки верхних углов'
corner_collection = bpy.data.collections.get('Защита верхних углов')
if corner_collection is None:
    corner_collection = bpy.data.collections.new('Защита верхних углов')
    bpy.context.scene.collection.children.link(corner_collection)
corner_collection.objects.link(caps_obj)

# Keep the moved STEP lidar as a separate editable CAD mesh.
bpy.ops.wm.stl_import(filepath=str(ROOT / 'models/robot/meshes/body_lidar.stl'))
lidar_obj = bpy.context.active_object
lidar_obj.name = 'cad_underbody_lidar'
lidar_obj.data.materials.clear()
lidar_obj.data.materials.append(bpy.data.materials['underbody_black'])
for collection in list(lidar_obj.users_collection):
    collection.objects.unlink(lidar_obj)
visuals.objects.link(lidar_obj)
lidar_obj.parent = base_link
lidar_obj.matrix_world = base_link.matrix_world @ Matrix.Translation(
    Vector(params['underbody']['lidar']['cad_visual_offset_m']))
lidar_obj['label_ru'] = 'Лидар STEP на задней перекладине'

for name, shift, label in (
        ('center_camera', params['underbody']['center_camera_visual_offset_m'], 'Центральная CAD-камера под нижней пластиной'),
        ('electronics_black', 0, 'Электроника на верхней отдельной площадке')):
    bpy.ops.wm.stl_import(filepath=str(ROOT / f'models/robot/meshes/body_{name}.stl'))
    obj = bpy.context.active_object
    obj.name = 'cad_' + name
    obj.data.materials.clear()
    obj.data.materials.append(bpy.data.materials['underbody_black'])
    for collection in list(obj.users_collection):
        collection.objects.unlink(obj)
    visuals.objects.link(obj)
    obj.parent = base_link
    obj.matrix_world = base_link.matrix_world @ Matrix.Translation(Vector((0,0,shift)))
    obj['label_ru'] = label

for label, sign, russian in (('left', 1, 'Левый'), ('right', -1, 'Правый')):
    x, y, z = side['x_m'], sign * side['y_abs_m'], side['z_m']
    components = (
        (f'underbody_{label}_mount', 'CUBE', (x, sign * (side['y_abs_m'] + side['length_m'] / 2 + .010), z), (.075, .008, .075), 'metal', f'{russian} кронштейн'),
        (f'underbody_{label}_cylinder', 'CYLINDER', (x, y, z), (side['radius_m'], side['length_m']), 'black', f'{russian} боковой цилиндр'),
        (f'underbody_{label}_lens', 'CYLINDER', (x, sign * (side['y_abs_m'] - side['length_m'] / 2 - .002), z), (.021, .003), 'glass', f'{russian} торец цилиндра'),
    )
    for name, kind, pose, size, color, description in components:
        if kind == 'CUBE':
            bpy.ops.mesh.primitive_cube_add(size=1, location=pose)
            obj = bpy.context.object
            obj.dimensions = size
            bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        else:
            bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=size[0], depth=size[1], location=pose, rotation=(math.pi / 2, 0, 0))
            obj = bpy.context.object
        obj.name = name
        obj.data.materials.clear()
        obj.data.materials.append(bpy.data.materials['underbody_' + color])
        for collection in list(obj.users_collection):
            collection.objects.unlink(obj)
        visuals.objects.link(obj)
        world = obj.matrix_world.copy()
        obj.parent = base_link
        obj.matrix_world = world
        obj['label_ru'] = description

bpy.context.preferences.filepaths.save_version = 0
bpy.ops.wm.save_as_mainfile(filepath=str(FULL))
print('SYNCED_COMPLETE_ROBOT', len(assembly.children))
