#!/usr/bin/env python3
"""Generate the multi-zone Gazebo test arena used by the robot application."""

from pathlib import Path
import sys

COLORS = {
    "ground": "0.19 0.23 0.28 1",
    "flat": "0.16 0.31 0.40 1",
    "obstacle": "0.33 0.25 0.16 1",
    "rough": "0.26 0.22 0.35 1",
    "hill": "0.18 0.34 0.25 1",
    "maze": "0.26 0.29 0.34 1",
    "wall": "0.56 0.61 0.67 1",
    "yellow": "0.96 0.68 0.20 1",
    "blue": "0.22 0.62 0.92 1",
    "red": "0.83 0.29 0.28 1",
    "green": "0.22 0.72 0.48 1",
    "purple": "0.58 0.40 0.82 1",
}

models = []


def material(color):
    return f"""<material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.18 0.18 0.18 1</specular></material>"""


def box(name, pose, size, color, collision=True):
    collision_xml = ""
    if collision:
        collision_xml = f"""<collision name="collision"><geometry><box><size>{size}</size></box></geometry>
        <surface><friction><ode><mu>1.1</mu><mu2>1.1</mu2></ode></friction></surface></collision>"""
    models.append(f"""<model name="{name}"><static>true</static><pose>{pose}</pose><link name="link">
      {collision_xml}
      <visual name="visual"><geometry><box><size>{size}</size></box></geometry>{material(color)}</visual>
    </link></model>""")


def cylinder(name, pose, radius, length, color):
    models.append(f"""<model name="{name}"><static>true</static><pose>{pose}</pose><link name="link">
      <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>
      <visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>{material(color)}</visual>
    </link></model>""")


# Common floor and boundary.
box("arena_ground", "0 0 -0.06 0 0 0", "36 26 0.12", COLORS["ground"])
box("boundary_north", "0 13 0.35 0 0 0", "36 0.18 0.7", COLORS["wall"])
box("boundary_south", "0 -13 0.35 0 0 0", "36 0.18 0.7", COLORS["wall"])
box("boundary_west", "-18 0 0.35 0 0 0", "0.18 26 0.7", COLORS["wall"])
box("boundary_east", "18 0 0.35 0 0 0", "0.18 26 0.7", COLORS["wall"])

# Colored zone surfaces are visual markings, while the common floor owns collision.
for name, pose, size, color in [
    ("zone_flat", "-12 6 0.004 0 0 0", "10 10 0.008", COLORS["flat"]),
    ("zone_obstacles", "0 6 0.004 0 0 0", "10 10 0.008", COLORS["obstacle"]),
    ("zone_rough", "12 6 0.004 0 0 0", "10 10 0.008", COLORS["rough"]),
    ("zone_hills", "-9 -6 0.004 0 0 0", "16 10 0.008", COLORS["hill"]),
    ("zone_maze", "9 -6 0.004 0 0 0", "16 10 0.008", COLORS["maze"]),
]:
    box(name, pose, size, color, collision=False)

# Zone borders and start pads.
for i, x in enumerate((-6, 6)):
    box(f"divider_top_{i}", f"{x} 6 0.012 0 0 0", "0.06 10 0.016", COLORS["wall"], collision=False)
box("divider_horizontal", "0 0 0.012 0 0 0", "36 0.06 0.016", COLORS["wall"], collision=False)
box("divider_bottom", "0 -6 0.012 0 0 0", "0.06 10 0.016", COLORS["wall"], collision=False)

starts = [
    ("flat", -15.5, 6, COLORS["blue"]),
    ("obstacles", -4.0, 6, COLORS["yellow"]),
    ("rough", 8.0, 6, COLORS["purple"]),
    ("hills", -15.5, -6, COLORS["green"]),
    ("maze", 2.0, -9.5, COLORS["red"]),
]
for name, x, y, color in starts:
    box(f"start_pad_{name}", f"{x} {y} 0.014 0 0 0", "1.3 1.3 0.025", color, collision=False)
    box(f"start_arrow_{name}", f"{x + 0.5} {y} 0.029 0 0 0", "0.7 0.12 0.02", "0.92 0.96 1 1", collision=False)

# Obstacle slalom: alternate blocks and posts with usable gaps.
for i, (x, y) in enumerate([(-1.8, 4.1), (-0.3, 7.7), (1.3, 4.6), (2.7, 7.2), (4.0, 5.1)]):
    if i % 2:
        cylinder(f"obstacle_post_{i}", f"{x} {y} 0.5 0 0 0", 0.36, 1.0, COLORS["yellow"])
    else:
        box(f"obstacle_block_{i}", f"{x} {y} 0.45 0 0 {0.18 if i == 2 else -0.15}", "0.7 1.4 0.9", COLORS["yellow"])

# Rough ground: transverse bumps followed by individually tilted tiles.
for i in range(7):
    x = 8.8 + i * 0.55
    height = 0.07 + (i % 3) * 0.025
    box(f"rough_bump_{i}", f"{x:.2f} 6 {height / 2:.3f} 0 0 0", f"0.22 7 {height:.3f}", COLORS["purple"])
tile = 0
for row, y in enumerate((3.0, 4.4, 5.8, 7.2, 8.6)):
    for col, x in enumerate((13.1, 14.4, 15.7)):
        z = 0.045 + 0.018 * ((row + col) % 3)
        roll = (-0.045, 0.035, 0.0)[(row + col) % 3]
        pitch = (0.04, -0.03, 0.015)[(row * 2 + col) % 3]
        box(f"rough_tile_{tile}", f"{x} {y} {z:.3f} {roll} {pitch} 0", "1.18 1.18 0.09", COLORS["rough"])
        tile += 1

# Hill section: climb, raised platform and descent, plus two side ridges.
box("hill_ramp_up", "-11.5 -6 0.30 0 -0.1489 0", "4.05 4.0 0.16", COLORS["green"])
box("hill_platform", "-8.5 -6 0.58 0 0 0", "2.0 4.0 0.16", COLORS["green"])
box("hill_ramp_down", "-5.5 -6 0.30 0 0.1489 0", "4.05 4.0 0.16", COLORS["green"])
box("hill_ridge_left", "-8.5 -3.55 0.18 0 0 0", "9.5 0.7 0.36", COLORS["green"])
box("hill_ridge_right", "-8.5 -8.45 0.18 0 0 0", "9.5 0.7 0.36", COLORS["green"])

# Maze perimeter and internal walls. Entrance is at the south-west start pad.
maze_walls = [
    ("west_upper", 1.2, -4.5, 0.18, 5.0),
    ("west_lower", 1.2, -9.7, 0.18, 1.5),
    ("east", 16.8, -6.0, 0.18, 10.0),
    ("north", 9.0, -1.1, 15.8, 0.18),
    ("south_left", 4.0, -10.9, 5.8, 0.18),
    ("south_right", 12.8, -10.9, 8.0, 0.18),
    ("a", 4.0, -7.4, 0.18, 6.8),
    ("b", 6.7, -4.0, 5.6, 0.18),
    ("c", 8.0, -7.2, 0.18, 6.2),
    ("d", 10.7, -8.4, 5.5, 0.18),
    ("e", 12.0, -4.6, 0.18, 5.2),
    ("f", 14.2, -6.8, 4.3, 0.18),
]
for name, x, y, sx, sy in maze_walls:
    box(f"maze_wall_{name}", f"{x} {y} 0.42 0 0 0", f"{sx} {sy} 0.84", COLORS["wall"])

world = f"""<?xml version="1.0" encoding="utf-8"?>
<sdf version="1.9">
  <world name="robot_test">
    <gravity>0 0 -9.81</gravity>
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system"><render_engine>ogre2</render_engine></plugin>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <scene><ambient>0.58 0.62 0.68 1</ambient><background>0.76 0.83 0.90 1</background><shadows>true</shadows></scene>
    <light name="sun" type="directional"><pose>0 0 30 0 0 0</pose><direction>-0.45 0.2 -1</direction><diffuse>0.92 0.92 0.92 1</diffuse><specular>0.15 0.15 0.15 1</specular><cast_shadows>true</cast_shadows></light>
    {"".join(models)}
    <include><uri>model://robot</uri><name>robot</name><pose>-15.5 6 0.08 0 0 0</pose></include>
  </world>
</sdf>
"""
output = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "worlds/test_world.sdf"
output.write_text(world)
print(output)
