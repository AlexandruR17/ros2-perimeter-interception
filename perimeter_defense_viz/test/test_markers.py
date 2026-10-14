"""Tests for the marker helpers."""

import math

from perimeter_defense_viz.markers import circle, dashed


def test_dashed_covers_expected_length():
    segs = dashed([(0.0, 0.0), (10.0, 0.0)], dash=1.0, gap=1.0)
    assert len(segs) == 5
    assert all(abs(math.hypot(b[0] - a[0], b[1] - a[1]) - 1.0) < 1e-9 for a, b in segs)


def test_dashed_closed_polygon_includes_closing_edge():
    square = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in dashed(square, 1.0, 1.0, True))
    assert abs(total - 8.0) < 1e-6


def test_circle_radius():
    for x, y in circle(2.0, 3.0, 2.5, 36):
        assert abs(math.hypot(x - 2.0, y - 3.0) - 2.5) < 1e-9
