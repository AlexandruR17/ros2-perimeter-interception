"""Interception point and driving distances."""

import math

import numpy as np
import pytest

from perimeter_defense.core.geodesic import GeodesicGrid
from perimeter_defense.core.interception import intercept_point, resample_along
from perimeter_defense.core.prediction import predict_path


def test_first_point_reached_before_the_target():
    # Target walks along y = 0 at 1.3 m/s; robot at (10, 4) drives 2 m/s in straight lines.
    path = predict_path((0.0, 0.0), (1.3, 0.0), 6.0, 0.2)
    robot = (10.0, 4.0)
    res = intercept_point(path, 0.2, lambda q: math.dist(robot, q), 2.0)
    assert res.feasible
    assert res.eta <= res.t
    # Ahead of the target, not where it is now.
    assert res.point[0] > 3.0
    # The previous step was not reachable in time.
    k = int(round(res.t / 0.2))
    assert math.dist(robot, path[k - 1]) / 2.0 > (k - 1) * 0.2


def test_unreachable_returns_last_point_not_feasible():
    path = predict_path((0.0, 0.0), (2.5, 0.0), 2.0, 0.2)
    res = intercept_point(path, 0.2, lambda q: math.dist((100.0, 0.0), q), 2.0)
    assert not res.feasible
    assert res.point == path[-1]


def test_margin_delays_the_choice():
    path = predict_path((0.0, 0.0), (1.3, 0.0), 6.0, 0.2)
    d = lambda q: math.dist((10.0, 4.0), q)   # noqa: E731
    assert intercept_point(path, 0.2, d, 2.0, margin=1.0).t > intercept_point(path, 0.2, d, 2.0).t


def wall_grid():
    # 20 x 10 m map at 0.05 m with a wall at x = 10 from y = 0 to y = 8 (gap at the top).
    occ = np.zeros((200, 400), dtype=bool)
    occ[0:160, 198:202] = True
    return GeodesicGrid(occ, 0.05, (0.0, 0.0), cell=0.25, inflation=0.3)


def test_driving_distance_goes_around_the_wall():
    g = wall_grid()
    field = g.distances_from((8.0, 2.0))
    straight = math.dist((8.0, 2.0), (12.0, 2.0))
    around = g.lookup(field, (12.0, 2.0))
    assert around > straight + 8.0          # up to the gap at y > 8 and back down
    assert g.lookup(field, (8.0, 5.0)) == pytest.approx(3.0, abs=0.4)


def test_bounds_keep_robots_inside():
    occ = np.zeros((200, 400), dtype=bool)
    g = GeodesicGrid(occ, 0.05, (0.0, 0.0), bounds=((2.0, 2.0), (18.0, 8.0)))
    field = g.distances_from((5.0, 5.0))
    assert math.isfinite(g.lookup(field, (15.0, 5.0)))
    assert not g.free[g.cell_of((1.0, 5.0))]


def test_resample_along_path_with_elapsed_time():
    path = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    pts = resample_along(path, 1.0, 1.0, start_time=8.0, horizon=6.0)
    assert pts[0] == pytest.approx((8.0, 0.0))
    assert pts[3] == pytest.approx((10.0, 1.0))
    assert pts[-1] == pytest.approx((10.0, 4.0))
    # Past the end: stays at the end.
    assert resample_along(path, 1.0, 1.0, 25.0, 6.0) == [(10.0, 10.0)]


def test_shortest_path_to_goal_goes_through_the_gap():
    g = wall_grid()
    tree = g.path_tree((12.0, 2.0))
    path = g.path_to_goal(tree, (8.0, 2.0))
    assert max(y for _, y in path) > 8.0           # through the gap at the top
    assert math.dist(path[-1], (12.0, 2.0)) < 0.3
