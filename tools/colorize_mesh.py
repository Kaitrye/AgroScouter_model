#!/usr/bin/env python3
"""Split the CAD body STL into material groups identified from the photos."""

import json
from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'models/robot/meshes/body.stl'
TRIANGLE = struct.Struct('<12fH')
SIDE_CAMERA_PARTS = {21, 22, 23, 24, 27}
REMOVED_EQUIPMENT_PARTS = SIDE_CAMERA_PARTS


PHOTO_PARTS = {
    'glass': {17, 18, 19, 20},
    'blue': {16},
    'green': {12},
    'black': {9, 10, 11, 13, 14, 15, 22, 24, 27},
}


def ros_bounds(part):
    x0, y0, z0, x1, y1, z1 = part['bbox']
    return ((z0 + 201.458483) / 1000, (x0 - 595) / 1000,
            (y0 + 76.445175) / 1000, (z1 + 201.458483) / 1000,
            (x1 - 595) / 1000, (y1 + 76.445175) / 1000)


def contained(inner, outer, tolerance=.001):
    return all(inner[i] >= outer[i] - tolerance for i in range(3)) and all(
        inner[i] <= outer[i] + tolerance for i in range(3, 6))


def obsolete_top_part(box):
    """The photographed robot has open top rails, without the CAD lid and mast."""
    x0, y0, z0, x1, y1, z1 = box
    lid = (x1 - x0 > .3 and y1 - y0 > .5 and
           .813 <= z0 <= .815 and .816 <= z1 <= .818)
    mast = z0 >= .816 and z1 > .82 and -.08 < y0 < .09 and y1 < .1
    return lid or mast


def unwanted_detail(box, reference):
    """Hide external cameras, exposed brackets, and the hanging CAD arm."""
    if any(contained(box, reference[index]) for index in REMOVED_EQUIPMENT_PARTS):
        return True
    if any(contained(box, reference[index]) for index in range(28, 637)):
        return True
    x0, y0, z0, x1, y1, z1 = box
    # Small brackets, pins and connectors above the inner equipment box.
    platform_equipment = any(contained(box, reference[index])
                             for index in (14, 15, 16))
    internal_fitting = (x0 > -.15 and x1 < .16 and y0 > -.23 and y1 < .23
                        and .45 < z0 < .8 and not platform_equipment)
    if internal_fitting:
        return True
    center_y = (y0 + y1) / 2
    if abs(center_y) > .30 and z0 > .35:
        motor_cylinder = (.09 < x1 - x0 < .11 and
                          .07 < y1 - y0 < .09 and
                          .09 < z1 - z0 < .11 and
                          .45 < z0 < .47)
        return not motor_cylinder
    return False


def partition():
    source = SOURCE.read_bytes()
    count = struct.unpack_from('<I', source, 80)[0]
    triangles = memoryview(source)[84:]
    if len(triangles) != count * 50:
        raise ValueError('Truncated body STL')

    # Triangles sharing an exact STL vertex belong to the same CAD shell.
    parent = list(range(count))
    owner = {}

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for index in range(count):
        base = index * 50
        for offset in (12, 24, 36):
            vertex = triangles[base + offset:base + offset + 12].tobytes()
            previous = owner.setdefault(vertex, index)
            a, b = find(index), find(previous)
            if a != b:
                parent[a] = b
    del owner

    bounds = {}
    for index in range(count):
        root = find(index)
        values = TRIANGLE.unpack_from(triangles, index * 50)
        box = bounds.setdefault(root, [float('inf')] * 3 + [float('-inf')] * 3)
        for offset in (3, 6, 9):
            for axis in range(3):
                value = values[offset + axis]
                box[axis] = min(box[axis], value)
                box[axis + 3] = max(box[axis + 3], value)

    parts = json.loads((ROOT / 'config/cad_parts.json').read_text())
    reference = {part['index']: ros_bounds(part) for part in parts}
    remove = {root: obsolete_top_part(box) or unwanted_detail(box, reference)
              for root, box in bounds.items()}
    material = {}
    for root, box in bounds.items():
        category = 'metal'
        for candidate, indices in PHOTO_PARTS.items():
            if any(contained(box, reference[index]) for index in indices):
                category = candidate
                break
        if category == 'metal':
            center_y = (box[1] + box[4]) / 2
            if abs(center_y) > .30 and box[5] < .62:
                category = 'black'
        material[root] = category

    groups = {category: bytearray() for category in
              ('metal', 'yellow', 'black', 'blue', 'green', 'glass')}
    removed = 0
    for index in range(count):
        root = find(index)
        if remove[root]:
            removed += 1
            continue
        category = material[root]
        if category == 'metal':
            box = bounds[root]
            is_frame = box[3] - box[0] > .35 and box[4] - box[1] > .5 and box[5] - box[2] > .7
            if is_frame:
                values = TRIANGLE.unpack_from(triangles, index * 50)
                z = (values[5], values[8], values[11])
                if all(.609 <= value <= .616 for value in z):
                    category = 'yellow'
        groups[category].extend(triangles[index * 50:(index + 1) * 50])

    for category, data in groups.items():
        output = SOURCE.with_name(f'body_{category}.stl')
        if not data:
            if category not in ('blue', 'green'):
                raise ValueError(f'No CAD faces matched material {category}')
            output.unlink(missing_ok=True)
            continue
        header = f'robot_sim photo palette: {category}'.encode().ljust(80, b' ')
        output.write_bytes(header + struct.pack('<I', len(data) // 50) + data)
        print(f'{output.name}: {len(data) // 50} triangles')
    assert sum(len(data) // 50 for data in groups.values()) + removed == count
    print(f'obsolete top and unwanted details: {removed} triangles removed')


if __name__ == '__main__':
    partition()
