"""Multi-target tracker life cycle."""

import pytest

from perimeter_defense.core.prediction import predict_path
from perimeter_defense.core.tracker import (ALERT, FRIEND, HOSTILE, LOST, NEUTRALIZED, TRACKED,
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


def test_alert_confirmed_by_hostile_lidar_track():
    tr = MultiTargetTracker()
    tid = tr.update_fence_alert((30.5, 26.0), 0.0)
    assert tr.tracks[tid].state == ALERT
    t = walk(tr, (30.7, 24.2), (0.0, -1.3), 2.0, 5)
    lidar = [x.id for x in tr.published() if x.id != tid][0]
    assert tr.tracks[tid].state == ALERT
    assert tr.set_classification(lidar, HOSTILE) == tid
    assert [x.id for x in tr.published()] == [tid]
    assert tr.tracks[tid].state == TRACKED and tr.tracks[tid].classification == HOSTILE
    # The detections keep updating the alert's id.
    walk(tr, (30.7, 24.2 - 1.3 * 0.5), (0.0, -1.3), t + 0.1, 5)
    assert [x.id for x in tr.published()] == [tid]


def test_friend_near_alert_does_not_confirm_it():
    tr = MultiTargetTracker()
    tid = tr.update_fence_alert((30.5, 26.0), 0.0)
    walk(tr, (28.0, 22.0), (0.0, 0.0), 1.0, 5)
    soldier = [x.id for x in tr.published() if x.id != tid][0]
    assert tr.set_classification(soldier, FRIEND, 'SLD-2') == soldier
    assert tr.tracks[tid].state == ALERT


def test_alert_does_not_grab_detections():
    tr = MultiTargetTracker()
    t = walk(tr, (34.0, 12.6), (1.0, 0.0), 0.0, 10)      # soldier, established track
    soldier = tr.published()[0].id
    tid = tr.update_fence_alert((30.5, 26.0), t)
    walk(tr, (35.0, 12.6), (1.0, 0.0), t + 0.1, 30)
    assert tr.tracks[tid].state == ALERT
    assert tr.tracks[soldier].state == TRACKED
    # Far from the alert: a hostile track does not confirm it.
    assert tr.set_classification(soldier, HOSTILE) == soldier


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


def test_neutralized_track_follows_escorted_person():
    tr = MultiTargetTracker()
    t = walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    tid = tr.published()[0].id
    tr.neutralize(tid, t)
    x = tr.tracks[tid].kf.position[0]
    for k in range(1, 60):
        tr.update_detections([(x + 1.0 * 0.1 * k, 0.0)], t + 0.1 * k)
    assert [x.id for x in tr.published()] == [tid]
    assert tr.tracks[tid].state == NEUTRALIZED
    assert tr.tracks[tid].kf.position[0] == pytest.approx(x + 5.9, abs=0.3)


def test_spot_report_creates_alert_and_skips_friends():
    tr = MultiTargetTracker()
    t = walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    friend = tr.published()[0].id
    tr.set_classification(friend, FRIEND, 'SLD-1')
    tid = tr.update_coarse((1.5, 0.0), 2.0, t + 0.1)
    assert tid != friend
    assert tr.tracks[tid].state == ALERT
    # A second report close to the first one updates the same ALERT track.
    assert tr.update_coarse((2.0, 0.5), 2.0, t + 2.0) == tid


def test_coarse_report_does_not_move_lidar_track():
    tr = MultiTargetTracker()
    t = walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    tid = tr.published()[0].id
    before = tr.tracks[tid].kf.position
    assert tr.update_coarse((2.0, 1.5), 2.0, t) == tid
    assert tr.tracks[tid].kf.position == pytest.approx(before)


def test_duplicate_track_of_one_person_is_merged():
    tr = MultiTargetTracker()
    t = walk(tr, (0.0, 0.0), (1.0, 0.0), 0.0, 10)
    first = tr.published()[0].id
    tr.set_classification(first, FRIEND, 'SLD-3')
    # Not seen for a while: LOST, the prediction runs on past the person ...
    tr.predict_to(t + 3.0)
    assert tr.tracks[first].state == LOST
    # ... who turned back: a second track is created on him, then the old
    # prediction (slowing down) comes back over him.
    x = tr.tracks[first].kf.position[0] - 2.0
    for k in range(30):
        tr.update_detections([(x, 0.0)], t + 3.1 + 0.1 * k)
    ids = [x.id for x in tr.published()]
    assert len(ids) == 1
    assert tr.tracks[ids[0]].classification == FRIEND


def test_two_people_side_by_side_not_merged():
    tr = MultiTargetTracker()
    for k in range(20):
        tr.update_detections([(0.1 * k, 0.0), (0.1 * k, 1.2)], 0.1 * k)
    assert len(tr.published()) == 2
