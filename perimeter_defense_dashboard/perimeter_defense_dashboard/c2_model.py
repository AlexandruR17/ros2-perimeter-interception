"""C2 picture for the web dashboard (ROS independent).

Builds the JSON-friendly state the browser draws: robots, tracks, contacts,
threat level and the event log. Event texts are operator-facing and in
Romanian, like the rest of the dashboard. Battery and radio link of the robots are
simulated here (marked as such in the UI); everything else is defense
information (tracks, roles, alerts, custody), never simulator ground truth.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

XY = Tuple[float, float]

ROLE_NAMES = {0: 'PATROL', 1: 'INTERCEPT', 2: 'GUARD', 3: 'FLANK', 4: 'INVESTIGATE',
              5: 'SEARCH', 6: 'BLOCK', 7: 'FOLLOW', 8: 'HOLD', 9: 'ESCORT'}
TRACK_STATES = {0: 'ALERT', 1: 'TRACKED', 2: 'LOST', 3: 'NEUTRALIZED'}
CLASSES = {0: 'UNKNOWN', 1: 'FRIEND', 2: 'HOSTILE'}
SEVERITIES = {0: 'INFO', 1: 'WARNING', 2: 'CRITICAL'}
ROLE_RO = {'PATROL': 'PATRULARE', 'INTERCEPT': 'INTERCEPTARE', 'GUARD': 'PAZĂ', 'FLANK': 'FLANC',
           'INVESTIGATE': 'INVESTIGARE', 'SEARCH': 'CĂUTARE', 'BLOCK': 'BLOCARE',
           'FOLLOW': 'URMĂRIRE', 'HOLD': 'IMOBILIZARE', 'ESCORT': 'ESCORTĂ'}


@dataclass
class BatteryModel:
    """Simulated battery: drains with distance driven and time switched on."""

    level: float = 100.0
    per_metre: float = 0.06       # % per metre driven
    per_second: float = 0.002     # % per second (electronics, sensors)
    last: Optional[XY] = None

    def update(self, pos: XY, dt: float) -> float:
        if self.last is not None:
            self.level -= self.per_metre * math.dist(pos, self.last)
        self.level -= self.per_second * max(dt, 0.0)
        self.level = max(self.level, 0.0)
        self.last = pos
        return self.level


def link_quality(pos: XY, command_post: XY, blocked: int) -> float:
    """Simulated radio link quality in %: distance to the command post antenna
    and buildings in the way."""
    q = 100.0 - 1.2 * math.dist(pos, command_post) - 12.0 * blocked
    return float(min(100.0, max(5.0, q)))


def threat_level(contacts: Sequence[dict]) -> str:
    """Provisional threat level (the threat assessor node comes in M5).

    RED: a hostile contact is active inside the base; ORANGE: an unconfirmed
    alert; GREEN: nothing active.
    """
    active = [c for c in contacts if c['status'] in ('ALERT', 'TRACKED', 'LOST')]
    if any(c['status'] in ('TRACKED', 'LOST') for c in active):
        return 'RED'
    if active:
        return 'ORANGE'
    return 'GREEN'


def covariance_2x2(cov16: Sequence[float]) -> List[float]:
    """[pxx, pxy, pyy] from the row-major 4x4 covariance of a track."""
    if len(cov16) < 6:
        return [0.0, 0.0, 0.0]
    return [round(float(cov16[0]), 4), round(float(cov16[1]), 4), round(float(cov16[5]), 4)]


def downsample(points: Sequence[XY], max_points: int) -> List[List[float]]:
    if not points:
        return []
    step = max(1, math.ceil(len(points) / max_points))
    pts = list(points[::step])
    if pts[-1] != points[-1]:
        pts.append(points[-1])
    return [[round(x, 2), round(y, 2)] for x, y in pts]


@dataclass
class EventLog:
    """Numbered events (alerts, role changes, custody) kept for late viewers."""

    capacity: int = 200
    events: Deque[dict] = field(default_factory=deque)
    next_id: int = 1

    def add(self, t: float, kind: str, severity: str, text: str,
            pos: Optional[XY] = None, track: int = 0) -> dict:
        ev = {'id': self.next_id, 't': round(t, 2), 'kind': kind, 'severity': severity,
              'text': text, 'track': track,
              'pos': [round(pos[0], 2), round(pos[1], 2)] if pos else None}
        self.next_id += 1
        self.events.append(ev)
        while len(self.events) > self.capacity:
            self.events.popleft()
        return ev

    def recent(self, n: int) -> List[dict]:
        return list(self.events)[-n:]


@dataclass
class ChangeWatch:
    """Turns state snapshots into events: robot roles, custody, new tracks."""

    roles: Dict[str, Tuple[str, int]] = field(default_factory=dict)
    custody: Dict[int, int] = field(default_factory=dict)
    seen_tracks: Dict[int, str] = field(default_factory=dict)

    def role(self, log: EventLog, t: float, callsign: str, role: str, track: int) -> None:
        if self.roles.get(callsign) == (role, track):
            return
        first = callsign not in self.roles
        self.roles[callsign] = (role, track)
        if first and role == 'PATROL':
            return
        text = f'{callsign}: {ROLE_RO.get(role, role)}' + (f' T{track}' if track else '')
        log.add(t, 'ROLE', 'INFO', text, track=track)

    def custody_state(self, log: EventLog, t: float, track: int, escort: str, state: int,
                      pos: Optional[XY]) -> None:
        if self.custody.get(track) == state:
            return
        self.custody[track] = state
        text = {0: f'T{track}: {escort} în drum spre persoana capturată',
                1: f'T{track}: escortat de {escort} spre arest',
                2: f'T{track}: reținut în arest ({escort})'}.get(state, f'T{track}: custodie {state}')
        log.add(t, 'CUSTODY', 'INFO', text, pos, track)

    def track(self, log: EventLog, t: float, tid: int, state: str, cls: str, pos: XY) -> None:
        key = f'{state}/{cls}'
        prev = self.seen_tracks.get(tid)
        if prev == key:
            return
        self.seen_tracks[tid] = key
        if cls == 'FRIEND':
            return
        if state == 'TRACKED' and cls == 'HOSTILE' and (prev is None or not prev.endswith('HOSTILE')):
            log.add(t, 'TRACK', 'CRITICAL', f'T{tid}: contact OSTIL urmărit', pos, tid)
        elif state == 'LOST' and prev and prev.startswith('TRACKED') and cls == 'HOSTILE':
            log.add(t, 'TRACK', 'WARNING', f'T{tid}: contact pierdut, predicția continuă', pos, tid)
        elif state == 'NEUTRALIZED' and (prev is None or not prev.startswith('NEUTRALIZED')):
            log.add(t, 'TRACK', 'INFO', f'T{tid}: capturat', pos, tid)
