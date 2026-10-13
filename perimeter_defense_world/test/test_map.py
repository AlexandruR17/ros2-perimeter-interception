"""The Nav2 maps must match the geometry, and the committed files must be up to date."""

import filecmp
import math
import os

import numpy as np
import pytest

from perimeter_defense_world.generate_map import rasterize, write_maps
from perimeter_defense_world.geometry import load_layout

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ZONES = os.path.join(PKG, 'config', 'zones.yaml')
RES = 0.05


@pytest.fixture(scope='module')
def layout():
    return load_layout(ZONES)


@pytest.fixture(scope='module')
def nav_grid(layout):
    return rasterize(layout, RES, solid_fence=True)


@pytest.fixture(scope='module')
def loc_grid(layout):
    return rasterize(layout, RES, solid_fence=False)


def occupied(grid, x, y):
    return bool(grid.occupied[int(y / RES), int(x / RES)])


def clearance(grid, x, y, radius):
    """True if no occupied cell lies within radius of (x, y)."""
    r = int(math.ceil(radius / RES))
    ci, ri = int(x / RES), int(y / RES)
    window = grid.occupied[max(0, ri - r):ri + r + 1, max(0, ci - r):ci + r + 1]
    return not window.any()


def test_buildings_occupied(layout, nav_grid):
    for b in layout.obstacles():
        assert occupied(nav_grid, *b.center), b.name


def test_breaches_free(layout, nav_grid):
    assert clearance(nav_grid, 30.5, 26.0, 1.2)
    assert clearance(nav_grid, 2.0, 9.5, 1.2)


def test_fence_solid_in_nav_map(nav_grid):
    assert occupied(nav_grid, 20.0, 26.0)
    assert occupied(nav_grid, 2.0, 20.0)


def test_fence_posts_only_in_localization_map(layout, loc_grid):
    p0, p1 = layout.fence_posts()[0], layout.fence_posts()[1]
    assert occupied(loc_grid, *p1)
    assert not occupied(loc_grid, (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)


def test_patrol_waypoints_reachable(layout, nav_grid):
    # LUP UGV half diagonal is about 0.35 m.
    for name, route in layout.patrol_routes.items():
        for wp in route:
            assert clearance(nav_grid, wp[0], wp[1], 0.6), f'{name} {wp}'


def test_target_and_spawns_free(layout, nav_grid):
    assert clearance(nav_grid, *layout.objective_target, 0.3)
    for name, p in layout.intruder_spawns.items():
        assert clearance(nav_grid, p[0], p[1], 0.3), name


def test_committed_maps_up_to_date(layout, tmp_path):
    write_maps(layout, str(tmp_path / 'fob3'), RES)
    for name in ('fob3.pgm', 'fob3.yaml', 'fob3_localization.pgm', 'fob3_localization.yaml'):
        assert filecmp.cmp(tmp_path / name, os.path.join(PKG, 'maps', name), shallow=False), \
            f'{name} is stale, run generate_map'


def test_map_dimensions(nav_grid):
    assert nav_grid.occupied.shape == (560, 800)
    assert np.count_nonzero(nav_grid.occupied) > 0
