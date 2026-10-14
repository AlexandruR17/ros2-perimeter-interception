"""Multi-target tracker life cycle."""

import pytest

from perimeter_defense.core.prediction import predict_path
from perimeter_defense.core.tracker import (ALERT, LOST, NEUTRALIZED, TRACKED,
                                            MultiTargetTracker, TrackerParams)


def walk(tracker, start, vel, t0, n, dt=0.1):
    for k in range(n):
        t = t0 + k * dt
        tracker.update_detections([(start[0] + vel[0] * k * dt, start[1] + vel[1] * k * dt)], t)
    return t0 + (n - 1) * dt


def test_track_confirmed_after_hits():
    tr = MultiTargetTracker(TrackerParams(confirm_hits=3))
    tr.update_detections([(1.0, 1.0)], 0.0)
    assert tr.published() == []
    walk(tr, (1.0, 1.0), (1.3, 0.0), 0.1, 3)
    pub = tr.published()
    assert len(pub) == 1 and pub[0].state == TRACKED


def test_lost_then_deleted():
    p = TrackerParams(lost_timeout=1.5, delete_after=5.0)
    tr = MultiTargetTracker(p)
    t = walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    tr.predict_to(t + 1.6)
    assert tr.published()[0].state == LOST
    tr.predict_to(t + 1.6 + 5.1)
    assert tr.published() == []


def test_lost_track_keeps_predicting_with_decaying_speed():
    tr = MultiTargetTracker(TrackerParams(lost_timeout=1.5, lost_velocity_tau=2.0))
    t = walk(tr, (0.0, 0.0), (1.3, 0.0), 0.0, 30)
    x_before = tr.published()[0].kf.position[0]
    tr.predict_to(t + 1.4)
    assert tr.published()[0].kf.position[0] > x_before + 1.4   # still TRACKED, full speed
    for k in range(40):
        tr.predict_to(t + 1.6 + 0.1 * k)
    track = tr.published()[0]
    assert track.state == LOST
    assert tr.speed(track.id) < 0.3


def test_fence_alert_creates_alert_track_then_lidar_confirms():
    tr = MultiTargetTracker()
    tid = tr.update_fence_alert((30.5, 26.0), 0.0)
    assert tr.tracks[tid].state == ALERT
    tr.update_detections([(30.7, 25.2)], 0.5)
    assert tr.tracks[tid].state == TRACKED


def test_two_targets_two_tracks():
    tr = MultiTargetTracker()
    for k in range(10):
        t = 0.1 * k
        tr.update_detections([(0.1 * k, 0.0), (10.0, 0.1 * k)], t)
    assert len(tr.published()) == 2


def test_neutralize():
    tr = MultiTargetTracker()
    walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    tid = tr.published()[0].id
    assert tr.neutralize(tid, 1.0)
    assert tr.tracks[tid].state == NEUTRALIZED


def test_predict_path_stops_at_obstacle_and_objective():
    wall = lambda x, y: x >= 3.0   # noqa: E731
    path = predict_path((0.0, 0.0), (1.0, 0.0), 6.0, 0.2, blocked=wall)
    assert path[-1][0] < 3.0 and len(path) < 31
    path = predict_path((0.0, 0.0), (1.0, 0.0), 6.0, 0.2, objective=(2.0, 0.0), objective_radius=0.5)
    assert path[-1][0] == pytest.approx(1.6)


def test_neutralized_track_absorbs_detections():
    tr = MultiTargetTracker()
    t = walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    tid = tr.published()[0].id
    tr.neutralize(tid, t)
    for k in range(1, 30):
        tr.update_detections([(0.9, 0.0)], t + 0.1 * k)
    assert [x.id for x in tr.published()] == [tid]
    assert tr.tracks[tid].state == NEUTRALIZED
