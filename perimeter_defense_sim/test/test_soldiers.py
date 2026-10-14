"""Soldier observation, patrol and escort helpers."""

import math
import os

from perimeter_defense_sim.path_planning import GridPlanner, plan_route
from perimeter_defense_sim.soldiers import (ObserverParams, SightingLog, can_see, loop_waypoints,
                                            pick_captive, pursuit_step)
from perimeter_defense_world.geometry import load_layout

ZONES = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                     'perimeter_defense_world', 'config', 'zones.yaml')
LAYOUT = load_layout(ZONES)


def test_buildings_block_the_view_fence_does_not():
    # Barracks (5,5)-(11,13) between the two points.
    assert not can_see(LAYOUT, (4.0, 9.0), (12.0, 9.0), 12.0)
    assert can_see(LAYOUT, (4.0, 14.0), (12.0, 14.0), 12.0)
    assert not can_see(LAYOUT, (4.0, 14.0), (12.0, 14.0), 5.0)          # out of range
    assert not can_see(LAYOUT, (29.0, 24.0), (30.5, 27.2), 12.0)        # outside the fence


def test_report_after_continuous_sight_then_periodic():
    log = SightingLog(ObserverParams(observe_time=1.0, report_period=4.0))
    assert log.update({'i1': True}, 0.0) == []
    assert log.update({'i1': True}, 1.0) == ['i1']
    assert log.update({'i1': True}, 3.0) == []
    assert log.update({'i1': True}, 5.0) == ['i1']
    # Losing sight restarts the observation time.
    log.update({'i1': False}, 5.5)
    assert log.update({'i1': True}, 9.6) == []


def test_soldier_routes_walkable_and_authorized():
    planner = GridPlanner(LAYOUT)
    for s in LAYOUT.soldiers.values():
        path = plan_route(planner, loop_waypoints(s.route))
        assert len(path) >= 2
        for p in s.route:
            assert LAYOUT.in_authorized_zone(p), (s.callsign, p)


def test_detention_reachable_from_both_breaches():
    planner = GridPlanner(LAYOUT)
    assert LAYOUT.detention is not None
    for start in [(30.5, 24.0), (4.0, 9.5), (20.5, 11.0)]:
        assert planner.plan(start, LAYOUT.detention.entry) is not None
    cell_building = [b for b in LAYOUT.buildings if b.name == LAYOUT.detention.building][0]
    assert cell_building.contains(LAYOUT.detention.cell)
    assert not cell_building.contains(LAYOUT.detention.entry)


def test_pursuit_reaches_end():
    path = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    pose, seg = (0.0, 0.0), 0
    for _ in range(400):
        v, seg, done = pursuit_step(pose, path, seg, 0.6, 1.2)
        if done:
            break
        pose = (pose[0] + v[0] * 0.05, pose[1] + v[1] * 0.05)
    assert done and math.dist(pose, (5.0, 5.0)) < 0.3


def test_pick_captive():
    assert pick_captive({'a': (1.0, 1.0), 'b': (9.0, 9.0)}, (8.0, 8.0), 4.0) == 'b'
    assert pick_captive({'a': (1.0, 1.0)}, (8.0, 8.0), 4.0) is None
