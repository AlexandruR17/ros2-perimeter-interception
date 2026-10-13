"""The committed world files must match the generator output."""

import os
import xml.etree.ElementTree as ET

from perimeter_defense_world.generate_world import FENCE_MESH_VISIBILITY_FLAG, world_sdf
from perimeter_defense_world.geometry import load_layout

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ZONES = os.path.join(PKG, 'config', 'zones.yaml')


def test_committed_worlds_up_to_date():
    layout = load_layout(ZONES)
    for night, suffix in ((False, 'day'), (True, 'night')):
        with open(os.path.join(PKG, 'worlds', f'fob3_{suffix}.sdf'), encoding='utf-8') as f:
            assert f.read() == world_sdf(layout, night), f'fob3_{suffix}.sdf is stale'


def test_world_is_valid_xml_with_all_models():
    layout = load_layout(ZONES)
    root = ET.fromstring(world_sdf(layout))
    names = {m.get('name') for m in root.iter('model')}
    for obstacle in layout.obstacles():
        assert obstacle.name in names
    assert 'perimeter_fence' in names


def test_real_time_factor_is_one():
    root = ET.fromstring(world_sdf(load_layout(ZONES)))
    assert float(root.find('world/physics/real_time_factor').text) == 1.0


def test_fence_mesh_hidden_from_lidar():
    root = ET.fromstring(world_sdf(load_layout(ZONES)))
    flags = [int(v.find('visibility_flags').text) for v in root.iter('visual')
             if v.find('visibility_flags') is not None]
    assert flags and all(f == FENCE_MESH_VISIBILITY_FLAG for f in flags)
    # The robot lidar mask must exclude exactly that bit.
    mask = 4294967293
    assert mask & FENCE_MESH_VISIBILITY_FLAG == 0
