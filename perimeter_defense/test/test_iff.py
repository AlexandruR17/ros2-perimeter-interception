"""Transponder based identification."""

from perimeter_defense.core.iff import (FRIEND, HOSTILE, LOST, NEUTRALIZED, TRACKED, UNKNOWN, Fix,
                                        IffClassifier, IffParams, TrackView)


def view(tid, xy, state=TRACKED, v=(0.0, 0.0)):
    return TrackView(tid, state, xy, v)


def test_transponder_makes_friend_and_silence_makes_hostile():
    iff = IffClassifier(IffParams(hostile_after=2.0, friend_hold=3.0, min_matches=1))
    tracks = [view(1, (5.0, 5.0)), view(2, (10.0, 5.0))]
    res = iff.update(tracks, [Fix('SLD-1', (5.6, 5.4), 0.0)], 0.0)
    assert res[1].classification == FRIEND and res[1].transponder == 'SLD-1'
    assert res[2].classification == UNKNOWN
    res = iff.update(tracks, [], 2.5)
    assert res[1].classification == FRIEND       # held for friend_hold
    assert res[2].classification == HOSTILE


def test_one_to_one_association():
    """One fix between two people marks only the closer one as friend."""
    iff = IffClassifier(IffParams(min_matches=1))
    res = iff.update([view(1, (0.0, 0.0)), view(2, (1.5, 0.0))], [Fix('SLD-1', (0.4, 0.0), 0.0)], 0.0)
    assert res[1].classification == FRIEND
    assert res[2].classification == UNKNOWN


def test_fix_age_compensated_with_velocity():
    iff = IffClassifier(IffParams(gate=1.0, min_matches=1))
    # The fix is 1 s old, the person walked 2.5 m since.
    res = iff.update([view(1, (2.5, 0.0), v=(2.5, 0.0))], [Fix('SLD-2', (0.0, 0.0), 0.0)], 1.0)
    assert res[1].classification == FRIEND


def test_captured_person_not_matched_and_keeps_class():
    iff = IffClassifier(IffParams(hostile_after=0.0))
    iff.update([view(1, (0.0, 0.0))], [], 0.0)
    assert iff.tracks[1].classification == HOSTILE
    # Captured, a soldier with transponder stands next to him; no soldier track yet.
    res = iff.update([view(1, (0.0, 0.0), NEUTRALIZED)], [Fix('SLD-1', (0.5, 0.0), 1.0)], 1.0)
    assert res[1].classification == HOSTILE


def test_alert_tracks_stay_unknown():
    iff = IffClassifier(IffParams(hostile_after=0.0))
    res = iff.update([view(1, (0.0, 0.0), state=0)], [], 10.0)
    assert res[1].classification == UNKNOWN
    iff.update([view(1, (0.0, 0.0))], [], 11.0)
    res = iff.update([view(1, (0.0, 0.0))], [], 11.5)
    assert res[1].classification == HOSTILE


def test_short_track_that_goes_lost_never_hostile():
    iff = IffClassifier(IffParams(hostile_after=2.0))
    iff.update([view(1, (0.0, 0.0))], [], 0.0)
    iff.update([view(1, (0.0, 0.0), LOST)], [], 1.0)
    assert iff.update([view(1, (0.0, 0.0), LOST)], [], 10.0)[1].classification == UNKNOWN


def test_friend_stays_friend_while_lost_then_needs_new_evidence():
    iff = IffClassifier(IffParams(hostile_after=2.0, friend_hold=3.0, min_matches=1))
    iff.update([view(1, (0.0, 0.0))], [Fix('SLD-1', (0.2, 0.0), 0.0)], 0.0)
    res = iff.update([view(1, (0.0, 0.0), LOST)], [], 20.0)
    assert res[1].classification == FRIEND
    # Re-acquired without transponder: unknown first, hostile after hostile_after.
    assert iff.update([view(1, (0.0, 0.0))], [], 20.5)[1].classification == UNKNOWN
    assert iff.update([view(1, (0.0, 0.0))], [], 23.0)[1].classification == HOSTILE


def test_track_published_hostile_keeps_class_under_new_id():
    iff = IffClassifier(IffParams(hostile_after=2.0))
    res = iff.update([TrackView(7, TRACKED, (0.0, 0.0), (0.0, 0.0), HOSTILE)], [], 50.0)
    assert res[7].classification == HOSTILE


def test_alert_track_takes_hostile_class_after_merge():
    iff = IffClassifier(IffParams(hostile_after=2.0))
    iff.update([view(2, (30.5, 26.0), state=0)], [], 0.0)
    res = iff.update([TrackView(2, TRACKED, (25.0, 15.0), (0.0, -2.0), HOSTILE)], [], 10.0)
    assert res[2].classification == HOSTILE


def test_single_match_of_a_passer_by_is_not_enough():
    """An intruder running past a soldier whose own track is not seen gets one
    match at most; the soldier walking with his track gets one every second."""
    iff = IffClassifier(IffParams(min_matches=2, hostile_after=2.0))
    res = iff.update([view(1, (1.0, 0.0), v=(2.5, 0.0))], [Fix('SLD-2', (0.0, 0.0), 0.0)], 0.0)
    assert res[1].classification == UNKNOWN
    res = iff.update([view(1, (3.5, 0.0), v=(2.5, 0.0))], [Fix('SLD-2', (0.0, 0.0), 1.0)], 1.0)
    assert res[1].classification == UNKNOWN
    # The single match delays the decision, it does not make him a friend.
    assert iff.update([view(1, (6.0, 0.0), v=(2.5, 0.0))], [], 2.5)[1].classification == UNKNOWN
    assert iff.update([view(1, (12.0, 0.0), v=(2.5, 0.0))], [], 5.0)[1].classification == HOSTILE
    soldier = IffClassifier(IffParams(min_matches=2))
    soldier.update([view(5, (0.0, 0.0))], [Fix('SLD-1', (0.3, 0.2), 0.0)], 0.0)
    res = soldier.update([view(5, (0.0, 0.0))], [Fix('SLD-1', (-0.4, 0.5), 1.0)], 1.0)
    assert res[5].classification == FRIEND
