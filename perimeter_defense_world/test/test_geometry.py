"""Tests for the zones.yaml geometry model."""

import os

import pytest

from perimeter_defense_world.geometry import load_layout, point_in_polygon

ZONES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'config', 'zones.yaml')


@pytest.fixture(scope='module')
def layout():
    return load_layout(ZONES)


def test_fence_has_both_breaches(layout):
    north = layout.breach_by_name('north')
    west = layout.breach_by_name('west')
    assert (north.start, north.end) == (29.0, 32.0)
    assert (west.start, west.end) == (8.0, 11.0)


def test_fence_segments_leave_gaps_at_breaches(layout):
    def covered(x, y):
        for seg in layout.fence_segments():
            (ax, ay), (bx, by) = seg.a, seg.b
            if ay == by == y and min(ax, bx) <= x <= max(ax, bx):
                return True
            if ax == bx == x and min(ay, by) <= y <= max(ay, by):
                return True
        return False

    assert not covered(30.5, 26.0)   # north breach
    assert not covered(2.0, 9.5)     # west breach
    assert covered(20.0, 26.0)
    assert covered(2.0, 20.0)
    assert covered(38.0, 14.0)
    assert covered(20.0, 2.0)


def test_fence_posts_at_segment_ends(layout):
    posts = layout.fence_posts()
    for corner in [(2.0, 2.0), (38.0, 26.0), (29.0, 26.0), (32.0, 26.0), (2.0, 8.0), (2.0, 11.0)]:
        assert any(abs(p[0] - corner[0]) < 1e-9 and abs(p[1] - corner[1]) < 1e-9 for p in posts)


def test_objective(layout):
    assert layout.objective_target == (20.5, 9.2)
    assert layout.objective_radius == 2.5


def test_routes_start_points(layout):
    assert layout.patrol_routes['lup1'][0] == (3.5, 24.5)
    assert layout.patrol_routes['lup2'][0] == (36.0, 4.0)
    assert layout.patrol_routes['lup3'][0] == (13.0, 3.0)


def test_point_in_polygon():
    square = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    assert point_in_polygon((1.0, 1.0), square)
    assert not point_in_polygon((3.0, 1.0), square)


def test_depot_not_in_authorized_zones(layout):
    for poly in layout.authorized_zones.values():
        assert not point_in_polygon(layout.objective_target, poly)
