"""Identification friend or foe from transponders.

Own personnel carry transponders that report a noisy position once per
second. Each round, the recent transponder fixes are associated one-to-one
with the lidar tracks (Hungarian algorithm on the distance, with a gate);
the fix age is compensated with the track velocity.

Classification of a track:
  FRIEND   a transponder matched it min_matches times in a row (one fix of a
           soldier standing next to an intruder running past is not enough)
           and within the last friend_hold seconds, or
           it was FRIEND when the lidar lost it (no new evidence while LOST)
  HOSTILE  tracked by lidar continuously for hostile_after seconds without a
           transponder (a short clutter track that goes LOST never qualifies)
           (any match, even a single one, restarts that time after friend_hold:
           an own soldier must never be declared hostile)
  UNKNOWN  otherwise (fence or spot report only, or too recent to decide)
Captured (neutralized) tracks keep their class and take no part in the
association, so a soldier escorting a detainee is not confused with him.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from perimeter_defense.core.association import associate

XY = Tuple[float, float]

UNKNOWN = 0
FRIEND = 1
HOSTILE = 2

ALERT = 0
TRACKED = 1
LOST = 2
NEUTRALIZED = 3


@dataclass
class IffParams:
    gate: float = 2.0            # m, max distance between a fix and a track
    min_matches: int = 2         # consecutive matches before a track is FRIEND
    max_fix_age: float = 1.5     # s, older fixes are ignored
    friend_hold: float = 3.0     # s a track stays FRIEND after its last match
    hostile_after: float = 2.0   # s of lidar tracking without transponder -> HOSTILE


@dataclass
class TrackView:
    """What the IFF needs from a track."""

    id: int
    state: int
    position: XY
    velocity: XY
    classification: int = UNKNOWN    # as published by the tracker


@dataclass
class Fix:
    transponder: str
    position: XY
    stamp: float


@dataclass
class TrackIff:
    classification: int = UNKNOWN
    transponder: str = ''
    last_match: float = -math.inf
    matches: int = 0
    lidar_since: Optional[float] = None     # start of the current lidar tracking
    last_state: int = ALERT


@dataclass
class IffClassifier:
    params: IffParams = field(default_factory=IffParams)
    tracks: Dict[int, TrackIff] = field(default_factory=dict)

    def update(self, tracks: Sequence[TrackView], fixes: Sequence[Fix], now: float) -> Dict[int, TrackIff]:
        """Associate the fixes with the tracks and update every track's class."""
        p = self.params
        live = {t.id for t in tracks}
        self.tracks = {k: v for k, v in self.tracks.items() if k in live}
        for t in tracks:
            if t.id not in self.tracks and t.classification == HOSTILE:
                # An alert track that took over a hostile lidar track keeps its class.
                self.tracks[t.id] = TrackIff(HOSTILE, lidar_since=now - p.hostile_after,
                                             last_state=TRACKED)
            info = self.tracks.setdefault(t.id, TrackIff())
            if t.classification == HOSTILE and info.classification == UNKNOWN:
                # The tracker merged a hostile lidar track into this alert track.
                info.classification, info.lidar_since = HOSTILE, now - p.hostile_after
            if t.state == TRACKED and (info.lidar_since is None or info.last_state != TRACKED):
                info.lidar_since = now      # first seen, or re-acquired after LOST
            elif t.state != TRACKED:
                info.lidar_since = None     # the time without transponder must be continuous
            info.last_state = t.state

        # Only lidar tracks can be matched: fence and spot alerts are too coarse,
        # captured persons keep their class.
        candidates = [t for t in tracks if t.state in (TRACKED, LOST)]
        fresh = latest_fixes([f for f in fixes if now - f.stamp <= p.max_fix_age])
        if candidates and fresh:
            cost = np.array([[_fix_distance(t, f, now) for f in fresh] for t in candidates])
            matches, _, _ = associate(cost, p.gate)
            for i, j in matches:
                info = self.tracks[candidates[i].id]
                tp = fresh[j].transponder
                if now - info.last_match > p.friend_hold or tp != info.transponder:
                    info.matches = 0
                info.last_match, info.transponder = now, tp
                info.matches += 1

        for t in tracks:
            info = self.tracks[t.id]
            if t.state == NEUTRALIZED:
                continue
            if now - info.last_match <= p.friend_hold and (
                    info.matches >= p.min_matches or info.classification == FRIEND):
                info.classification = FRIEND
            elif t.state == LOST and info.classification == FRIEND:
                pass
            elif (info.lidar_since is not None
                  and now - max(info.lidar_since, info.last_match + p.friend_hold) >= p.hostile_after):
                info.classification, info.transponder = HOSTILE, ''
            elif info.classification == FRIEND:
                info.classification, info.transponder = UNKNOWN, ''
        return self.tracks


def latest_fixes(fixes: Sequence[Fix]) -> List[Fix]:
    """Most recent fix of each transponder."""
    best: Dict[str, Fix] = {}
    for f in fixes:
        if f.transponder not in best or f.stamp > best[f.transponder].stamp:
            best[f.transponder] = f
    return list(best.values())


def _fix_distance(t: TrackView, f: Fix, now: float) -> float:
    age = max(0.0, now - f.stamp)
    x = t.position[0] - t.velocity[0] * age
    y = t.position[1] - t.velocity[1] * age
    return math.hypot(x - f.position[0], y - f.position[1])
