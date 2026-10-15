"""Role allocation (M4)."""

import math

import pytest

from perimeter_defense.core.roles import (BLOCK, ENGAGE, FLANK, GUARD, ThreatInfo, allocate,
                                          point_along)

# Robots on a line: A near the threat, C near the objective.
ENGAGE_T = {('A', 1): 2.0, ('B', 1): 5.0, ('C', 1): 9.0,
            ('A', 2): 8.0, ('B', 2): 3.0, ('C', 2): 6.0}
GUARD_T = {'A': 9.0, 'B': 5.0, 'C': 1.0}


def et(r, t):
    return ENGAGE_T[(r, t)]


def gt(r):
    return GUARD_T[r]


def test_one_threat_intercept_guard_flank():
    out = allocate(['A', 'B', 'C'], [ThreatInfo(1, True, 10.0)], et, gt)
    assert out == {'A': (ENGAGE, 1), 'C': (GUARD, 0), 'B': (FLANK, 1)}


def test_alert_only_gives_block():
    out = allocate(['A', 'B', 'C'], [ThreatInfo(1, False, 20.0)], et, gt)
    assert out['A'] == (ENGAGE, 1) and out['C'] == (GUARD, 0) and out['B'] == (BLOCK, 1)


def test_two_threats_min_total_time_and_one_guard():
    out = allocate(['A', 'B', 'C'], [ThreatInfo(1, True, 10.0), ThreatInfo(2, True, 12.0)], et, gt)
    assert out['A'] == (ENGAGE, 1) and out['B'] == (ENGAGE, 2) and out['C'] == (GUARD, 0)


def test_always_one_guard_with_two_robots():
    out = allocate(['A', 'C'], [ThreatInfo(1, True, 10.0), ThreatInfo(2, True, 12.0)], et, gt)
    assert sorted(v[0] for v in out.values()) == [ENGAGE, GUARD]
    # The most urgent threat is the one engaged.
    assert (ENGAGE, 1) in out.values()


def test_single_robot_engages():
    assert allocate(['C'], [ThreatInfo(1, True, 10.0)], et, gt) == {'C': (ENGAGE, 1)}


def test_naive_everyone_to_closest_threat_no_guard():
    out = allocate(['A', 'B', 'C'], [ThreatInfo(1, True, 10.0), ThreatInfo(2, True, 12.0)], et, gt,
                   mode='naive')
    assert out == {'A': (ENGAGE, 1), 'B': (ENGAGE, 2), 'C': (ENGAGE, 2)}


def test_hysteresis_keeps_previous_unless_much_better():
    prev = {'B': (ENGAGE, 1), 'C': (GUARD, 0), 'A': (FLANK, 1)}
    # B is 3 s slower than A on threat 1: switch.
    assert allocate(['A', 'B', 'C'], [ThreatInfo(1, True, 10.0)], et, gt, previous=prev)['A'] == (ENGAGE, 1)
    # Only 0.5 s slower: keep B.
    close = dict(ENGAGE_T)
    close[('B', 1)] = 2.5
    out = allocate(['A', 'B', 'C'], [ThreatInfo(1, True, 10.0)], lambda r, t: close[(r, t)], gt, previous=prev)
    assert out['B'] == (ENGAGE, 1)


def test_point_along():
    path = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
    assert point_along(path, 5.0) == pytest.approx((3.0, 2.0))
    assert point_along(path, 50.0) == (3.0, 4.0)
    assert math.isclose(point_along(path, 0.0)[0], 0.0)
