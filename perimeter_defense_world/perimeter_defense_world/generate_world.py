"""Generate the FOB-3 Gazebo (Harmonic) world SDF from zones.yaml.

The fence is modelled as visible posts plus a chain-link panel. The panel is
visible to cameras but carries a visibility flag that the robot lidar masks
out, so the lidar sees through the mesh like a real lidar sees through a
chain-link fence. Buildings block both lidar and camera.

Usage:
    ros2 run perimeter_defense_world generate_world --out worlds/ --textures models/fob3_textures/textures
"""

from __future__ import annotations

import argparse
import math
import os
import random
from typing import List, Optional

from perimeter_defense_world.geometry import BaseLayout, Rect, Segment, load_layout
from perimeter_defense_world.textures import write_textures

# Visual flag of the chain-link panel. The robot lidar uses
# visibility_mask = 0xFFFFFFFD, so it ignores visuals with only this bit set.
FENCE_MESH_VISIBILITY_FLAG = 0x2

TEXTURE_URI = 'model://fob3_textures/textures'

# Pitched-roof buildings: wall texture -> roof texture.
GABLE_ROOF = {'plaster_cream': 'roof_tile', 'plaster_ochre': 'roof_tin_green'}
ROOF_PITCH = math.radians(25.0)

BUILDING_TEXTURE = {
    'plaster_cream': 'plaster_cream', 'plaster_ochre': 'plaster_ochre',
    'hangar_arch': 'hangar_metal',
    'concrete': 'concrete', 'metal': 'metal', 'container_green': 'container_green',
    'container_tan': 'container_tan', 'bunker': 'bunker',
}
PROP_TEXTURE = {'sandbags': 'sandbags', 'vehicle': 'vehicle', 'tower': 'tower', 'hesco': 'hesco'}
# Roofs: textured for containers and metal buildings, tinted concrete otherwise.
ROOF_TEXTURE = {'metal': 'metal', 'container_green': 'container_green',
                'container_tan': 'container_tan', 'bunker': 'bunker_roof'}
ROOF_COLOR = {'concrete': '0.2 0.22 0.15 1', 'metal': '0.85 0.85 0.85 1',
              'container_green': '1 1 1 1', 'container_tan': '1 1 1 1', 'bunker': '0.8 0.8 0.75 1'}


def _material(texture: Optional[str], color: str = '1 1 1 1', roughness: float = 0.9,
              transparency: float = 0.0) -> str:
    pbr = ''
    if texture:
        pbr = (f'<pbr><metal><albedo_map>{TEXTURE_URI}/{texture}.png</albedo_map>'
               f'<roughness>{roughness}</roughness><metalness>0.0</metalness></metal></pbr>')
    return (f'<transparency>{transparency}</transparency>' if transparency > 0 else '') + (
        f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
        f'<specular>0.05 0.05 0.05 1</specular>{pbr}</material>')


def _box_link(name: str, pose: str, size: str, material: str, collision: bool = True,
              extra_visual: str = '') -> str:
    coll = (f'<collision name="{name}_c"><pose>{pose}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry></collision>') if collision else ''
    return (f'<visual name="{name}_v"><pose>{pose}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry>{material}{extra_visual}</visual>'
            f'{coll}')


def _gable_roof(sx: float, sy: float, wall_h: float, texture: str, wall_texture: str) -> str:
    """Two roof slabs meeting at a ridge along the longer side of the building."""
    along_x = sx >= sy
    span = sy if along_x else sx
    length = (sx if along_x else sy) + 0.5
    slope = (span / 2.0 + 0.35) / math.cos(ROOF_PITCH)
    rise = (span / 4.0) * math.tan(ROOF_PITCH)
    parts = []
    for i, side in enumerate((1.0, -1.0)):
        off = side * (span / 4.0 + 0.08)
        if along_x:
            pose = f'0 {off:.3f} {wall_h + rise:.3f} {-side * ROOF_PITCH:.4f} 0 0'
            size = f'{length:.3f} {slope:.3f} 0.08'
        else:
            pose = f'{off:.3f} 0 {wall_h + rise:.3f} 0 {side * ROOF_PITCH:.4f} 0'
            size = f'{slope:.3f} {length:.3f} 0.08'
        parts.append(_box_link(f'roof{i}', pose, size, _material(texture), collision=False))
    # A low box under the ridge closes the attic from the side, kept under the slabs.
    attic_h = 0.9 * rise
    attic = (f'{sx:.3f} {span * 0.5:.3f} {attic_h:.3f}' if along_x
             else f'{span * 0.5:.3f} {sy:.3f} {attic_h:.3f}')
    parts.append(_box_link('attic', f'0 0 {wall_h + attic_h / 2:.3f} 0 0 0', attic,
                           _material(wall_texture), collision=False))
    return ''.join(parts)


def building_model(b: Rect) -> str:
    (cx, cy), (sx, sy) = b.center, b.size
    tex = BUILDING_TEXTURE.get(b.kind, 'concrete')
    parts: List[str] = []
    if b.kind == 'hangar_arch':
        # Half cylinder (the lower half is under the ground) with a box collision
        # matching the map footprint.
        r = min(sx, sy) / 2.0
        length = max(sx, sy)
        rot = '0 1.5708 0' if sx >= sy else '1.5708 0 0'
        parts.append(
            f'<visual name="arch_v"><pose>0 0 0 {rot}</pose><geometry><cylinder>'
            f'<radius>{r:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>'
            f'{_material(tex)}</visual>'
            f'<collision name="arch_c"><pose>0 0 {r / 2:.3f} 0 0 0</pose><geometry><box>'
            f'<size>{sx:.3f} {sy:.3f} {r:.3f}</size></box></geometry></collision>')
        door_x = -(sx / 2 + 0.02) if sx >= sy else 0.0
        door_y = 0.0 if sx >= sy else -(sy / 2 + 0.02)
        parts.append(_box_link('door', f'{door_x:.3f} {door_y:.3f} 1.2 0 0 0',
                               '0.06 3.0 2.4' if sx >= sy else '3.0 0.06 2.4',
                               _material(None, '0.22 0.26 0.2 1'), collision=False))
    else:
        parts.append(_box_link('body', f'0 0 {b.height / 2:.3f} 0 0 0',
                               f'{sx:.3f} {sy:.3f} {b.height:.3f}', _material(tex)))
        if b.kind in GABLE_ROOF:
            parts.append(_gable_roof(sx, sy, b.height, GABLE_ROOF[b.kind], tex))
        else:
            parts.append(_box_link(
                'roof', f'0 0 {b.height + 0.06:.3f} 0 0 0', f'{sx + 0.2:.3f} {sy + 0.2:.3f} 0.12',
                _material(ROOF_TEXTURE.get(b.kind), ROOF_COLOR.get(b.kind, '0.4 0.4 0.38 1')),
                collision=False))
        # Door on the south face (north face for the depot, which faces the target point).
        door_y = (sy / 2 + 0.03) if b.kind == 'bunker' else -(sy / 2 + 0.03)
        parts.append(_box_link('door', f'0 {door_y:.3f} 1.1 0 0 0', '1.6 0.06 2.2',
                               _material(None, '0.30 0.24 0.18 1'), collision=False))
    return (f'<model name="{b.name}"><static>true</static><pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>'
            f'<link name="link">{"".join(parts)}</link></model>')


def prop_model(p: Rect) -> str:
    (cx, cy), (sx, sy) = p.center, p.size
    tex = PROP_TEXTURE.get(p.kind)
    parts: List[str] = []
    if p.kind == 'tower':
        # Four legs (seen by the lidar) and an elevated cabin.
        leg = 0.14
        for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            lx, ly = dx * (sx / 2 - leg / 2), dy * (sy / 2 - leg / 2)
            parts.append(_box_link(f'leg{i}', f'{lx:.3f} {ly:.3f} {p.height / 2:.3f} 0 0 0',
                                   f'{leg} {leg} {p.height:.3f}', _material(tex)))
        parts.append(_box_link('cabin', f'0 0 {p.height + 0.6:.3f} 0 0 0',
                               f'{sx + 0.6:.3f} {sy + 0.6:.3f} 1.2', _material(tex)))
    elif p.kind == 'vehicle':
        # Chassis, cabin and wheels approximated with boxes and cylinders.
        parts.append(_box_link('chassis', f'0 0 {0.45 + 0.5:.3f} 0 0 0', f'{sx:.3f} {sy:.3f} 1.0',
                               _material(tex)))
        cab_y = sy / 2 - 0.9
        parts.append(_box_link('cab', f'0 {cab_y:.3f} {p.height - 0.25:.3f} 0 0 0',
                               f'{sx - 0.1:.3f} 1.6 0.5', _material(tex)))
        for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            wx, wy = dx * (sx / 2 - 0.15), dy * (sy / 2 - 0.8)
            parts.append(
                f'<visual name="wheel{i}"><pose>{wx:.3f} {wy:.3f} 0.45 0 1.5708 0</pose>'
                '<geometry><cylinder><radius>0.45</radius><length>0.3</length></cylinder></geometry>'
                f'{_material(None, "0.08 0.08 0.08 1")}</visual>')
        # Camouflage net on poles above the vehicle (visual only, above the lidar plane).
        net_z = p.height + 0.4
        parts.append(_box_link('camo_net', f'0 0 {net_z:.3f} 0 0 0',
                               f'{sx + 0.4:.3f} {sy + 1.0:.3f} 0.03',
                               _material('camo_net', transparency=0.15), collision=False))
        for i, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            px, py = dx * (sx / 2 + 0.15), dy * (sy / 2 + 0.45)
            parts.append(_box_link(f'pole{i}', f'{px:.3f} {py:.3f} {net_z / 2:.3f} 0 0 0',
                                   f'0.05 0.05 {net_z:.3f}', _material(None, '0.25 0.22 0.16 1'),
                                   collision=False))
    elif p.kind == 'flagpole':
        r = min(sx, sy) / 2.0
        parts.append(
            f'<visual name="pole_v"><pose>0 0 {p.height / 2:.3f} 0 0 0</pose><geometry><cylinder>'
            f'<radius>{r * 0.4:.3f}</radius><length>{p.height:.3f}</length></cylinder></geometry>'
            f'{_material(None, "0.8 0.8 0.78 1")}</visual>'
            f'<collision name="pole_c"><pose>0 0 {p.height / 2:.3f} 0 0 0</pose><geometry>'
            f'<cylinder><radius>{r:.3f}</radius><length>{p.height:.3f}</length></cylinder>'
            '</geometry></collision>')
        parts.append(_box_link('flag', f'0.62 0 {p.height - 0.45:.3f} 0 0 0', '1.2 0.02 0.8',
                               _material('flag_ro'), collision=False))
    else:
        parts.append(_box_link('body', f'0 0 {p.height / 2:.3f} 0 0 0',
                               f'{sx:.3f} {sy:.3f} {p.height:.3f}', _material(tex)))
    return (f'<model name="{p.name}"><static>true</static><pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>'
            f'<link name="link">{"".join(parts)}</link></model>')


def fence_model(layout: BaseLayout) -> str:
    parts: List[str] = []
    h = layout.fence_height
    post = layout.post_size
    t = layout.fence_thickness
    mesh_mat = _material(None, '0.55 0.57 0.55 1', transparency=0.55)
    post_mat = _material(None, '0.62 0.62 0.6 1')   # concrete posts
    wire_mat = _material(None, '0.4 0.4 0.4 1', transparency=0.6)
    seg: Segment
    for i, seg in enumerate(layout.fence_segments()):
        (ax, ay), (bx, by) = seg.a, seg.b
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        # Chain-link panel: camera visible, lidar transparent, physically solid.
        parts.append(
            f'<visual name="mesh{i}"><pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 {yaw:.5f}</pose>'
            f'<geometry><box><size>{length:.3f} 0.02 {h:.3f}</size></box></geometry>{mesh_mat}'
            f'<visibility_flags>{FENCE_MESH_VISIBILITY_FLAG}</visibility_flags></visual>'
            f'<collision name="wall{i}"><pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 {yaw:.5f}</pose>'
            f'<geometry><box><size>{length:.3f} {t:.3f} {h:.3f}</size></box></geometry></collision>')
        # Top rail and concertina wire coil (above the lidar plane).
        parts.append(
            f'<visual name="rail{i}"><pose>{cx:.3f} {cy:.3f} {h:.3f} 0 0 {yaw:.5f}</pose>'
            f'<geometry><box><size>{length:.3f} 0.04 0.04</size></box></geometry>{post_mat}</visual>')
        parts.append(
            f'<visual name="wire{i}"><pose>{cx:.3f} {cy:.3f} {h + 0.25:.3f} 0 1.5708 {yaw:.5f}</pose>'
            f'<geometry><cylinder><radius>0.25</radius><length>{length:.3f}</length></cylinder>'
            f'</geometry>{wire_mat}<visibility_flags>{FENCE_MESH_VISIBILITY_FLAG}</visibility_flags>'
            '</visual>')
    # Main gate on the south fence (closed, visual only, same visibility as the mesh).
    gate = layout.surroundings.get('main_gate')
    if gate:
        gx, gw = (gate['from'] + gate['to']) / 2.0, gate['to'] - gate['from']
        gy = layout.fence_min[1]
        parts.append(
            f'<visual name="gate"><pose>{gx:.3f} {gy - 0.03:.3f} {h / 2:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{gw:.3f} 0.05 {h:.3f}</size></box></geometry>'
            f'{_material("gate")}<visibility_flags>{FENCE_MESH_VISIBILITY_FLAG}</visibility_flags>'
            '</visual>')
    # Posts (seen by the lidar and drawn in the localization map).
    for k, (px, py) in enumerate(layout.fence_posts()):
        parts.append(
            f'<visual name="post{k}"><pose>{px:.3f} {py:.3f} {h / 2:.3f} 0 0 0</pose>'
            f'<geometry><box><size>{post} {post} {h:.3f}</size></box></geometry>{post_mat}'
            '</visual>')
    return (f'<model name="perimeter_fence"><static>true</static>'
            f'<link name="link">{"".join(parts)}</link></model>')


def ground_model(layout: BaseLayout) -> str:
    w, h = layout.width, layout.height
    textured = (
        f'<visual name="base_ground"><pose>{w / 2:.3f} {h / 2:.3f} -0.025 0 0 0</pose>'
        f'<geometry><box><size>{w:.3f} {h:.3f} 0.05</size></box></geometry>'
        f'{_material("ground", roughness=1.0)}</visual>')
    surround = (
        f'<visual name="surround"><pose>{w / 2:.3f} {h / 2:.3f} -0.09 0 0 0</pose>'
        '<geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>'
        f'{_material(None, "0.38 0.42 0.27 1")}</visual>')
    collision = (
        '<collision name="ground_c"><geometry><plane><normal>0 0 1</normal>'
        '<size>300 300</size></plane></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2>'
        '</ode></friction></surface></collision>')
    terrain = ''
    if layout.surroundings:
        (x0, y0), (x1, y1) = layout.surroundings['bounds']['min'], layout.surroundings['bounds']['max']
        terrain = (
            f'<visual name="terrain"><pose>{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} -0.055 0 0 0</pose>'
            f'<geometry><box><size>{x1 - x0:.3f} {y1 - y0:.3f} 0.05</size></box></geometry>'
            f'{_material("terrain", roughness=1.0)}</visual>')
    return (f'<model name="ground"><static>true</static><link name="link">'
            f'{collision}{textured}{terrain}{surround}</link></model>')


def windbreak_model(layout: BaseLayout) -> str:
    """Rows of acacia trees along the roads and the canal (visual only)."""
    rows = layout.surroundings.get('tree_rows', [])
    if not rows:
        return ''
    rng = random.Random(layout.surroundings.get('farmland_seed', 21))
    trunk = _material(None, '0.30 0.24 0.18 1')
    crown_colors = ('0.24 0.34 0.16 1', '0.28 0.38 0.18 1', '0.20 0.30 0.14 1')
    crowns = [_material(None, c) for c in crown_colors]
    parts: List[str] = []
    k = 0
    for row in rows:
        (ax, ay), (bx, by) = row['from'], row['to']
        n = max(1, int(math.hypot(bx - ax, by - ay) / row['spacing']))
        for i in range(n + 1):
            x = ax + (bx - ax) * i / n + rng.uniform(-0.6, 0.6)
            y = ay + (by - ay) * i / n + rng.uniform(-0.6, 0.6)
            th = rng.uniform(2.5, 3.5)
            cr = rng.uniform(1.6, 2.4)
            parts.append(
                f'<visual name="trunk{k}"><pose>{x:.2f} {y:.2f} {th / 2:.2f} 0 0 0</pose>'
                f'<geometry><cylinder><radius>0.15</radius><length>{th:.2f}</length></cylinder>'
                f'</geometry>{trunk}</visual>'
                f'<visual name="crown{k}"><pose>{x:.2f} {y:.2f} {th + cr * 0.4:.2f} 0 0 '
                f'{rng.uniform(0, 3.14):.2f}</pose>'
                f'<geometry><ellipsoid><radii>{cr:.2f} {cr * 0.9:.2f} {cr * 0.6:.2f}</radii>'
                '</ellipsoid></geometry>'
                f'{crowns[k % len(crowns)]}</visual>')
            k += 1
    return (f'<model name="windbreaks"><static>true</static>'
            f'<link name="link">{"".join(parts)}</link></model>')


def lighting(layout: BaseLayout, night: bool) -> str:
    if not night:
        return (
            '<scene><ambient>0.55 0.55 0.55 1</ambient><background>0.62 0.72 0.82 1</background>'
            '<shadows>true</shadows><grid>false</grid></scene>'
            '<light type="directional" name="sun"><cast_shadows>true</cast_shadows>'
            '<pose>0 0 30 0 0 0</pose><diffuse>0.95 0.92 0.85 1</diffuse>'
            '<specular>0.3 0.3 0.3 1</specular><direction>-0.25 0.2 -0.95</direction></light>')
    # Night: weak moonlight plus flood lights on the buildings and the depot.
    parts = [
        '<scene><ambient>0.06 0.06 0.09 1</ambient><background>0.01 0.01 0.03 1</background>'
        '<shadows>true</shadows><grid>false</grid></scene>',
        '<light type="directional" name="moon"><cast_shadows>false</cast_shadows>'
        '<pose>0 0 30 0 0 0</pose><diffuse>0.10 0.11 0.16 1</diffuse>'
        '<specular>0.02 0.02 0.02 1</specular><direction>0.3 -0.4 -0.86</direction></light>',
    ]
    for i, b in enumerate(layout.buildings):
        cx, cy = b.center
        parts.append(
            f'<light type="spot" name="flood_{i}"><pose>{cx:.2f} {cy - b.size[1] / 2 - 0.3:.2f} '
            f'{b.height + 0.4:.2f} 0 0 0</pose><diffuse>1.0 0.85 0.6 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular><attenuation><range>14</range><constant>0.3</constant>'
            '<linear>0.08</linear><quadratic>0.01</quadratic></attenuation>'
            '<direction>0 -0.5 -0.86</direction><spot><inner_angle>0.6</inner_angle>'
            '<outer_angle>1.2</outer_angle><falloff>1.0</falloff></spot>'
            '<cast_shadows>false</cast_shadows></light>')
    return ''.join(parts)


def world_sdf(layout: BaseLayout, night: bool = False, name: str = 'fob3') -> str:
    models = [ground_model(layout), fence_model(layout)]
    models += [building_model(b) for b in layout.buildings]
    models += [prop_model(p) for p in layout.props]
    trees = windbreak_model(layout)
    if trees:
        models.append(trees)
    body = '\n    '.join(models)
    return f"""<?xml version="1.0"?>
<!-- {layout.name}: generated by perimeter_defense_world/generate_world.py from config/zones.yaml. Do not edit. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.81</gravity>
    {lighting(layout, night)}
    {body}
  </world>
</sdf>
"""


def generate(layout: BaseLayout, world_dir: str, texture_dir: Optional[str]) -> List[str]:
    os.makedirs(world_dir, exist_ok=True)
    written = []
    for night, suffix in ((False, 'day'), (True, 'night')):
        path = os.path.join(world_dir, f'fob3_{suffix}.sdf')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(world_sdf(layout, night))
        written.append(path)
    if texture_dir:
        write_textures(layout, texture_dir)
        written.append(texture_dir)
    return written


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--zones', default=None, help='path to zones.yaml')
    parser.add_argument('--out', required=True, help='output directory for the world files')
    parser.add_argument('--textures', default=None, help='output directory for the textures')
    args = parser.parse_args(argv)
    for path in generate(load_layout(args.zones), args.out, args.textures):
        print(f'written: {path}')


if __name__ == '__main__':
    main()
