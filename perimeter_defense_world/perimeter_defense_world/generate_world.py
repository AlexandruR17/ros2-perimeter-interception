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
from typing import List, Optional

from perimeter_defense_world.geometry import BaseLayout, Rect, Segment, load_layout
from perimeter_defense_world.textures import write_textures

# Visual flag of the chain-link panel. The robot lidar uses
# visibility_mask = 0xFFFFFFFD, so it ignores visuals with only this bit set.
FENCE_MESH_VISIBILITY_FLAG = 0x2

TEXTURE_URI = 'model://fob3_textures/textures'

BUILDING_TEXTURE = {
    'concrete': 'concrete', 'metal': 'metal', 'container_green': 'container_green',
    'container_tan': 'container_tan', 'bunker': 'bunker',
}
PROP_TEXTURE = {'sandbags': 'sandbags', 'vehicle': 'vehicle', 'tower': 'tower', 'hesco': 'hesco'}
# Roofs: textured for containers and metal buildings, tinted concrete otherwise.
ROOF_TEXTURE = {'metal': 'metal', 'container_green': 'container_green',
                'container_tan': 'container_tan', 'bunker': 'bunker'}
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


def building_model(b: Rect) -> str:
    (cx, cy), (sx, sy) = b.center, b.size
    tex = BUILDING_TEXTURE.get(b.kind, 'concrete')
    body = _box_link('body', f'0 0 {b.height / 2:.3f} 0 0 0', f'{sx:.3f} {sy:.3f} {b.height:.3f}',
                     _material(tex))
    roof = _box_link('roof', f'0 0 {b.height + 0.06:.3f} 0 0 0',
                     f'{sx + 0.2:.3f} {sy + 0.2:.3f} 0.12',
                     _material(ROOF_TEXTURE.get(b.kind), ROOF_COLOR.get(b.kind, '0.4 0.4 0.38 1')),
                     collision=False)
    # Door on the south face (north face for the depot, which faces the target point).
    door_y = (sy / 2 + 0.03) if b.kind == 'bunker' else -(sy / 2 + 0.03)
    door = _box_link('door', f'0 {door_y:.3f} 1.1 0 0 0', '1.6 0.06 2.2',
                     _material(None, '0.18 0.19 0.17 1'), collision=False)
    return (f'<model name="{b.name}"><static>true</static><pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>'
            f'<link name="link">{body}{roof}{door}</link></model>')


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
    post_mat = _material(None, '0.36 0.37 0.35 1')
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
        # Top rail (above the lidar plane).
        parts.append(
            f'<visual name="rail{i}"><pose>{cx:.3f} {cy:.3f} {h:.3f} 0 0 {yaw:.5f}</pose>'
            f'<geometry><box><size>{length:.3f} 0.04 0.04</size></box></geometry>{post_mat}</visual>')
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
        f'<visual name="surround"><pose>{w / 2:.3f} {h / 2:.3f} -0.06 0 0 0</pose>'
        '<geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>'
        f'{_material(None, "0.38 0.42 0.27 1")}</visual>')
    collision = (
        '<collision name="ground_c"><geometry><plane><normal>0 0 1</normal>'
        '<size>300 300</size></plane></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2>'
        '</ode></friction></surface></collision>')
    return (f'<model name="ground"><static>true</static><link name="link">'
            f'{collision}{textured}{surround}</link></model>')


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
    body = '\n    '.join(models)
    return f"""<?xml version="1.0"?>
<!-- Generated by perimeter_defense_world/generate_world.py from config/zones.yaml. Do not edit. -->
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
