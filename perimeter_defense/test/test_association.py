"""Detection to track association (Hungarian with gating)."""

import numpy as np

from perimeter_defense.core.association import associate, euclidean_cost


def test_one_to_one_optimal_assignment():
    # Greedy nearest neighbour would give det 0 to track 0 (cost 1.0) and
    # leave track 1 with det 1 (cost 8.0), total 9.0. Optimal total is 4.0.
    cost = np.array([[1.0, 2.0],
                     [2.0, 8.0]])
    matches, ur, uc = associate(cost, gate=9.21)
    assert sorted(matches) == [(0, 1), (1, 0)]
    assert ur == [] and uc == []


def test_gating_rejects_far_pairs():
    cost = np.array([[0.5, 50.0],
                     [40.0, 60.0]])
    matches, ur, uc = associate(cost, gate=9.21)
    assert matches == [(0, 0)]
    assert ur == [1] and uc == [1]


def test_more_detections_than_tracks():
    cost = np.array([[3.0, 0.2, 7.0]])
    matches, ur, uc = associate(cost)
    assert matches == [(0, 1)]
    assert uc == [0, 2]


def test_empty_inputs():
    assert associate(np.zeros((0, 3))) == ([], [], [0, 1, 2])
    assert associate(np.zeros((2, 0))) == ([], [0, 1], [])


def test_crossing_targets_keep_identity():
    # Intruder passing next to a soldier: predicted positions 0.6 m apart,
    # each measurement closer to its own track in the joint sense.
    tracks = [(10.0, 10.0), (10.6, 10.0)]
    dets = [(10.55, 10.05), (10.05, 9.95)]
    matches, _, _ = associate(euclidean_cost(tracks, dets) / 0.2 ** 2, gate=9.21)
    assert sorted(matches) == [(0, 1), (1, 0)]
