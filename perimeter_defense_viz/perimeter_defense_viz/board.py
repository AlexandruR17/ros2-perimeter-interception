"""Contact register shown on the status board next to the map.

Every person the defense registers inside the perimeter (fence breach alert,
soldier spot report, or hostile lidar track inside the fence) gets an entry that
stays on the board for the whole exercise, from the first report to
detention. Only defense information is used: tracks, alerts, robot roles
and the custody reports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

XY = Tuple[float, float]

# Track states and classes (perimeter_defense_msgs/Track).
ALERT, TRACKED, LOST, NEUTRALIZED = 0, 1, 2, 3
UNKNOWN, FRIEND, HOSTILE = 0, 1, 2
# Custody states (perimeter_defense_msgs/Custody).
DISPATCHED, ESCORTING, DETAINED = 0, 1, 2

ROLE_NAMES = {0: 'PATROL', 1: 'INTERCEPT', 2: 'GUARD', 3: 'FLANK', 4: 'INVESTIGATE',
              5: 'SEARCH', 6: 'BLOCK', 7: 'FOLLOW', 8: 'HOLD', 9: 'ESCORT'}
CLASS_NAMES = {UNKNOWN: 'UNKNOWN', FRIEND: 'FRIEND', HOSTILE: 'HOSTILE'}


@dataclass
class TrackInfo:
    id: int
    state: int
    classification: int
    position: XY
    velocity: XY


@dataclass
class Report:
    source: str      # 'BREACH NORTH', 'SPOT SLD-2'
    position: XY
    stamp: float


@dataclass
class Entry:
    track_id: int
    registered: float
    via: str
    classification: int = UNKNOWN
    state: int = ALERT
    position: XY = (0.0, 0.0)
    speed: float = 0.0
    live: bool = True
    custody: Optional[Tuple[str, int]] = None   # (escort call sign, custody state)
    robot: str = ''                              # 'LUP-2 FOLLOW'
    last_seen: float = 0.0
    previous: List[int] = field(default_factory=list)   # earlier track ids of this person

    @property
    def label(self) -> str:
        return f'T{self.track_id}' + (f' (T{self.previous[0]})' if self.previous else '')

    @property
    def status(self) -> str:
        if self.custody and self.custody[1] == DETAINED:
            return 'DETAINED'
        if self.custody and self.custody[1] == ESCORTING:
            return 'ESCORTED'
        if self.state == NEUTRALIZED or self.custody:
            return 'CAPTURED'
        if not self.live:
            return 'NO CONTACT'
        return {ALERT: 'ALERT', TRACKED: 'TRACKED', LOST: 'LOST'}.get(self.state, '?')

    @property
    def custody_text(self) -> str:
        if not self.custody:
            return '-'
        escort, state = self.custody
        return {DISPATCHED: f'{escort} EN ROUTE', ESCORTING: f'{escort} ESCORT',
                DETAINED: f'BY {escort}'}[state]

    @property
    def active(self) -> bool:
        """Still a threat inside the base (not captured, not detained, not gone)."""
        return self.status in ('ALERT', 'TRACKED', 'LOST')


@dataclass
class ContactRegister:
    inside: object                      # callable(XY) -> bool, inside the fence
    objective: XY
    report_radius: float = 5.0          # m, report to track matching
    report_window: float = 30.0         # s
    reacquire_radius: float = 4.0       # m, new hostile track that continues a lost contact
    reacquire_window: float = 20.0      # s
    entries: Dict[int, Entry] = field(default_factory=dict)
    reports: List[Report] = field(default_factory=list)

    def add_report(self, source: str, position: XY, stamp: float) -> None:
        self.reports.append(Report(source, position, stamp))

    def _via(self, position: XY, now: float) -> str:
        recent = [r for r in self.reports if now - r.stamp <= self.report_window]
        best = min(recent, key=lambda r: math.dist(r.position, position), default=None)
        if best is not None and math.dist(best.position, position) <= self.report_radius:
            return best.source
        return 'LIDAR'

    def update(self, tracks: Sequence[TrackInfo], roles: Dict[str, Tuple[int, int]],
               custody: Dict[int, Tuple[str, int]], now: float) -> None:
        """roles: robot call sign -> (role, track id); custody: track id -> (escort, state)."""
        live = set()
        for t in tracks:
            e = self.entries.get(t.id)
            if t.classification == FRIEND:
                # Own soldier recognised by the IFF: not an infiltrated contact.
                self.entries.pop(t.id, None)
                continue
            if e is None:
                # Registered from a fence or spot report alert, or a lidar track
                # inside the fence once the IFF found no transponder (an own
                # soldier is never shown, even for the second the IFF needs).
                confirmed = t.classification == HOSTILE or t.state == NEUTRALIZED
                if not (t.state == ALERT or (confirmed and self.inside(t.position))):
                    continue
                e = self._reacquired(t, now) or Entry(t.id, now, self._via(t.position, now))
                self.entries[t.id] = e
            live.add(t.id)
            e.classification, e.state, e.position = t.classification, t.state, t.position
            e.speed = math.hypot(*t.velocity) if t.state != NEUTRALIZED else 0.0
            e.last_seen = now
        for tid, e in self.entries.items():
            e.live = tid in live
            if tid in custody:
                e.custody = custody[tid]
            e.robot = ''
            for robot, (role, track) in sorted(roles.items()):
                if track == tid and role != 0:
                    e.robot = f'{robot} {ROLE_NAMES.get(role, "?")}'

    def _reacquired(self, t: TrackInfo, now: float) -> Optional[Entry]:
        """A new hostile track where a contact was just lost is the same person:
        the tracker lost him (stopped behind an obstacle) and found him again."""
        lost = [e for e in self.entries.values()
                if e.status in ('LOST', 'NO CONTACT') and now - e.last_seen <= self.reacquire_window
                and math.dist(e.position, t.position) <= self.reacquire_radius]
        best = min(lost, key=lambda e: math.dist(e.position, t.position), default=None)
        if best is None:
            return None
        del self.entries[best.track_id]
        best.previous.insert(0, best.track_id)
        best.track_id = t.id
        return best

    def rows(self, limit: int) -> List[Entry]:
        """Most recent entries, oldest first."""
        ordered = sorted(self.entries.values(), key=lambda e: e.registered)
        return ordered[-limit:]

    def counters(self) -> Dict[str, int]:
        es = list(self.entries.values())
        return {'ACTIVE': sum(e.active for e in es),
                'CAPTURED': sum(e.status in ('CAPTURED', 'ESCORTED') for e in es),
                'DETAINED': sum(e.status == 'DETAINED' for e in es)}

    def distance_to_objective(self, e: Entry) -> float:
        return math.dist(e.position, self.objective)


def format_elapsed(seconds: float) -> str:
    s = max(0, int(seconds))
    return f'{s // 60:02d}:{s % 60:02d}'
