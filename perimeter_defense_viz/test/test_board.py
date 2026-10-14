"""Contact register of the status board."""

from perimeter_defense_viz.board import (ALERT, DETAINED, DISPATCHED, ESCORTING, FRIEND, HOSTILE,
                                         NEUTRALIZED, TRACKED, UNKNOWN, ContactRegister, TrackInfo,
                                         format_elapsed)


def inside(p):
    return 2.0 < p[0] < 38.0 and 2.0 < p[1] < 26.0


def info(tid, state, cls=UNKNOWN, xy=(30.5, 25.0), v=(0.0, -1.3)):
    return TrackInfo(tid, state, cls, xy, v)


def test_breach_contact_life_cycle_to_detention():
    reg = ContactRegister(inside, (20.5, 9.2))
    reg.add_report('BREACH NORTH', (30.5, 26.0), 0.0)
    reg.update([info(1, ALERT, xy=(30.5, 26.0))], {}, {}, 0.5)
    [e] = reg.rows(6)
    assert e.via == 'BREACH NORTH' and e.status == 'ALERT'
    reg.update([info(1, TRACKED, HOSTILE)], {'LUP-2': (7, 1)}, {}, 3.0)
    assert e.status == 'TRACKED' and e.robot == 'LUP-2 FOLLOW' and e.speed > 1.0
    reg.update([info(1, NEUTRALIZED, HOSTILE)], {'LUP-2': (8, 1)}, {1: ('SLD-3', DISPATCHED)}, 9.0)
    assert e.status == 'CAPTURED' and e.custody_text == 'SLD-3 EN ROUTE' and e.speed == 0.0
    reg.update([info(1, NEUTRALIZED, HOSTILE)], {'LUP-2': (0, 0)}, {1: ('SLD-3', ESCORTING)}, 15.0)
    assert e.status == 'ESCORTED' and e.robot == ''
    # The track is deleted once the person is inside the building; the entry stays.
    reg.update([], {}, {1: ('SLD-3', DETAINED)}, 60.0)
    assert reg.rows(6)[0].status == 'DETAINED'
    assert reg.counters() == {'ACTIVE': 0, 'CAPTURED': 0, 'DETAINED': 1}


def test_friends_and_outside_tracks_not_registered():
    reg = ContactRegister(inside, (20.5, 9.2))
    reg.update([info(1, TRACKED, FRIEND, xy=(10.0, 10.0)),
                info(2, TRACKED, UNKNOWN, xy=(30.0, 27.5))], {}, {}, 1.0)
    assert reg.rows(6) == []
    reg.update([info(3, TRACKED, UNKNOWN, xy=(10.0, 10.0))], {}, {}, 2.0)
    assert reg.rows(6) == []                    # waiting for the IFF
    reg.update([info(3, TRACKED, HOSTILE, xy=(10.0, 10.0))], {}, {}, 2.5)
    assert reg.rows(6)[0].via == 'LIDAR'
    # Recognised as a soldier later: removed from the board.
    reg.update([info(3, TRACKED, FRIEND, xy=(10.0, 10.0))], {}, {}, 3.0)
    assert reg.rows(6) == []


def test_breach_without_lidar_contact_stays_registered():
    reg = ContactRegister(inside, (20.5, 9.2))
    reg.add_report('BREACH NORTH', (30.5, 26.0), 0.0)
    reg.update([info(1, ALERT, xy=(30.5, 26.0)), info(2, TRACKED, HOSTILE, xy=(20.0, 20.0))], {}, {}, 1.0)
    reg.update([], {}, {}, 30.0)
    rows = reg.rows(6)
    assert [e.track_id for e in rows] == [1, 2]
    assert [e.status for e in rows] == ['NO CONTACT', 'NO CONTACT']
    assert rows[0].via == 'BREACH NORTH'


def test_spot_report_source():
    reg = ContactRegister(inside, (20.5, 9.2))
    reg.add_report('SPOT SLD-2', (24.0, 15.0), 10.0)
    reg.update([info(5, ALERT, xy=(24.5, 15.5))], {}, {}, 11.0)
    assert reg.rows(6)[0].via == 'SPOT SLD-2'


def test_format_elapsed():
    assert format_elapsed(75.4) == '01:15'


def test_reacquired_contact_keeps_its_entry():
    reg = ContactRegister(inside, (20.5, 9.2))
    reg.add_report('BREACH NORTH', (30.5, 26.0), 0.0)
    reg.update([info(1, ALERT, xy=(30.5, 26.0))], {}, {}, 0.5)
    reg.update([info(1, TRACKED, HOSTILE, xy=(21.0, 9.6))], {}, {}, 20.0)
    reg.update([], {}, {}, 25.0)                       # deleted by the tracker
    reg.update([info(7, TRACKED, HOSTILE, xy=(21.2, 9.5))], {}, {}, 27.0)
    [e] = reg.rows(6)
    assert e.track_id == 7 and e.label == 'T7 (T1)' and e.via == 'BREACH NORTH'
    assert e.status == 'TRACKED'
