"""Lidar person detection."""

import math

import numpy as np

from perimeter_defense.core.clustering import (DetectorParams, cluster_ordered, detect_people,
                                               fuse_detections, scan_to_points,
                                               transform_points)


def ring(cx, cy, r, n, a0=0.0, a1=math.pi):
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


def no_static(pts, margins=None):
    return np.zeros(len(pts), dtype=bool)


def test_scan_to_points_drops_invalid():
    pts, rng = scan_to_points([1.0, float('inf'), 0.1, 2.0], 0.0, math.pi / 2, 0.25, 30.0)
    assert len(pts) == 2
    assert np.allclose(pts[0], [1.0, 0.0])
    assert np.allclose(pts[1], [0.0, -2.0], atol=1e-9)   # 4th beam at 3 pi / 2


def test_transform_points():
    out = transform_points(np.array([[1.0, 0.0]]), 2.0, 3.0, math.pi / 2)
    assert np.allclose(out, [[2.0, 4.0]])


def test_cluster_split_on_gaps():
    pts = np.array([[0, 0], [0.1, 0], [0.2, 0], [2.0, 0], [2.1, 0]], dtype=float)
    clusters = cluster_ordered(pts, 0.35)
    assert [len(c) for c in clusters] == [3, 2]


def test_person_detected_wall_rejected():
    person = ring(5.0, 0.0, 0.2, 6, math.pi / 2, 3 * math.pi / 2)    # side facing the sensor
    wall = [(8.0, y) for y in np.linspace(-2.0, 2.0, 30)]             # wide, not person sized
    pts = np.array(person + wall)
    dets = detect_people(pts, (0.0, 0.0), DetectorParams(), no_static)
    assert len(dets) == 1
    assert abs(dets[0].centroid[0] - 5.0) < 0.2 and abs(dets[0].centroid[1]) < 0.1


def test_static_map_points_removed():
    person = np.array(ring(5.0, 0.0, 0.2, 6, math.pi / 2, 3 * math.pi / 2))
    all_static = lambda p, m: np.ones(len(p), dtype=bool)   # noqa: E731
    assert detect_people(person, (0.0, 0.0), DetectorParams(), all_static) == []


def test_out_of_range_and_own_robot_removed():
    far = ring(12.0, 0.0, 0.2, 6, math.pi / 2, 3 * math.pi / 2)
    robot = ring(3.0, 0.0, 0.25, 6, math.pi / 2, 3 * math.pi / 2)
    pts = np.array(far + robot)
    assert detect_people(pts, (0.0, 0.0), DetectorParams(), no_static, [(3.0, 0.0)]) == []


def test_fuse_detections_from_two_robots():
    fused = fuse_detections([((5.0, 5.0), 0.3, 'lup1'), ((5.3, 5.1), 0.35, 'lup2'),
                             ((9.0, 9.0), 0.3, 'lup1')], radius=0.8)
    assert len(fused) == 2
    merged = [f for f in fused if len(f[2]) == 2][0]
    assert abs(merged[0][0] - 5.15) < 1e-9


def test_static_margin_grows_with_range():
    # A wall return 0.45 m from the mapped wall is clutter at 9 m but not at 2 m.
    params = DetectorParams(static_margin=0.35, static_margin_per_m=0.035)
    near = np.array(ring(2.0, 0.0, 0.2, 6, math.pi / 2, 3 * math.pi / 2))
    far = np.array(ring(9.0, 0.0, 0.2, 6, math.pi / 2, 3 * math.pi / 2))
    wall_dist = lambda p, m: 0.45 < m   # noqa: E731  every point 0.45 m from a wall
    assert len(detect_people(near, (0.0, 0.0), params, wall_dist)) == 1
    assert detect_people(far, (0.0, 0.0), params, wall_dist) == []
