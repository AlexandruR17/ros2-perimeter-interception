"""C2 picture helpers of the dashboard."""

import math
import os

import pytest

from perimeter_defense_dashboard.c2_model import (BatteryModel, ChangeWatch, EventLog, covariance_2x2,
                                                  downsample, link_quality, threat_level)
from perimeter_defense_dashboard.server_node import layout_config
from perimeter_defense_world.geometry import load_layout

ZONES = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                     'perimeter_defense_world', 'config', 'zones.yaml')


def test_threat_level():
    assert threat_level([]) == 'GREEN'
    assert threat_level([{'status': 'DETAINED'}]) == 'GREEN'
    assert threat_level([{'status': 'ALERT'}]) == 'ORANGE'
    assert threat_level([{'status': 'ALERT'}, {'status': 'TRACKED'}]) == 'RED'


def test_battery_drains_with_distance_and_time():
    b = BatteryModel(per_metre=0.1, per_second=0.01)
    b.update((0.0, 0.0), 0.0)
    assert b.update((10.0, 0.0), 10.0) == pytest.approx(100.0 - 1.0 - 0.1)


def test_link_quality_drops_with_distance_and_buildings():
    near = link_quality((1.0, 0.0), (0.0, 0.0), 0)
    far = link_quality((30.0, 0.0), (0.0, 0.0), 0)
    blocked = link_quality((30.0, 0.0), (0.0, 0.0), 2)
    assert near > far > blocked >= 5.0


def test_covariance_and_downsample():
    cov = [0.0] * 16
    cov[0], cov[1], cov[5] = 0.25, 0.05, 0.36
    assert covariance_2x2(cov) == [0.25, 0.05, 0.36]
    pts = [(float(i), 0.0) for i in range(100)]
    out = downsample(pts, 10)
    assert len(out) <= 11 and out[-1] == [99.0, 0.0]


def test_events_numbered_and_bounded():
    log = EventLog(capacity=3)
    for i in range(5):
        log.add(float(i), 'FENCE', 'WARNING', f'e{i}')
    assert [e['id'] for e in log.recent(10)] == [3, 4, 5]


def test_change_watch_reports_transitions_once():
    log, w = EventLog(), ChangeWatch()
    w.role(log, 0.0, 'LUP-2', 'PATROL', 0)          # initial patrol: no event
    w.role(log, 1.0, 'LUP-2', 'FOLLOW', 3)
    w.role(log, 1.3, 'LUP-2', 'FOLLOW', 3)
    w.track(log, 1.0, 3, 'TRACKED', 'UNKNOWN', (1.0, 1.0))
    w.track(log, 2.0, 3, 'TRACKED', 'HOSTILE', (1.0, 1.0))
    w.track(log, 2.5, 3, 'TRACKED', 'HOSTILE', (1.0, 1.0))
    w.custody_state(log, 3.0, 3, 'LUP-2', 1, (1.0, 1.0))
    w.custody_state(log, 3.5, 3, 'LUP-2', 1, (1.0, 1.0))
    kinds = [e['kind'] for e in log.recent(10)]
    assert kinds == ['ROLE', 'TRACK', 'CUSTODY']


def test_layout_config_for_the_browser():
    cfg = layout_config(load_layout(ZONES), ['lup2'], 9090)
    assert cfg['mgrs_origin'] == '35TNK1500038000'
    assert cfg['detention']['building'] == 'detention'
    assert {b['name'] for b in cfg['breaches']} == {'north', 'west'}
    assert math.isclose(cfg['objective']['radius'], 2.5)
