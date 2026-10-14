"""Multi-target tracker: one constant velocity Kalman filter per target.

Life cycle of a track:
  tentative  -> created from an unmatched lidar detection, not published;
                confirmed after confirm_hits detections within confirm_window
  ALERT      -> created from a fence sensor alert only (coarse position)
  TRACKED    -> confirmed by lidar detections
  LOST       -> no detection for lost_timeout; the filter keeps predicting
  NEUTRALIZED-> set externally when a robot reached the target; keeps
                absorbing the detections of that person
Tracks are deleted when LOST for longer than delete_after, when an ALERT is
never confirmed within alert_timeout, or when a tentative track is not
confirmed in time.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from perimeter_defense.core.association import GATE_CHI2_99, associate
from perimeter_defense.core.kalman import ConstantVelocityKF

XY = Tuple[float, float]

TENTATIVE = -1
ALERT = 0
TRACKED = 1
LOST = 2
NEUTRALIZED = 3

UNKNOWN = 0
FRIEND = 1
HOSTILE = 2


@dataclass
class TrackerParams:
    accel_std: float = 1.0            # m/s^2, tuned: 0.18 m/s velocity rms, ~0.85 s walk-to-run response
    meas_std: float = 0.2             # m, lidar cluster centroid
    fence_std: float = 1.5            # m, fence sensor accuracy
    init_vel_std: float = 1.5         # m/s, velocity unknown at creation
    gate_chi2: float = GATE_CHI2_99
    confirm_hits: int = 3
    confirm_window: float = 1.0       # s
    lost_timeout: float = 1.5         # s without detection -> LOST
    delete_after: float = 8.0         # s in LOST -> deleted
    lost_velocity_tau: float = 2.0    # s, velocity decay of LOST tracks (people stop)
    alert_timeout: float = 20.0       # s for an ALERT track to be confirmed
    max_speed: float = 3.5            # m/s, speed clamp for people
    neutralized_keep: float = 10.0    # s a neutralized track stays after its last detection


@dataclass
class Track:
    id: int
    kf: ConstantVelocityKF
    state: int
    first_seen: float
    last_update: float
    hits: int = 1
    classification: int = UNKNOWN
    transponder_id: str = ''
    state_since: float = 0.0
    meta: Dict[str, float] = field(default_factory=dict)

    @property
    def confirmed(self) -> bool:
        return self.state != TENTATIVE


class MultiTargetTracker:
    def __init__(self, params: Optional[TrackerParams] = None) -> None:
        self.p = params or TrackerParams()
        self.tracks: Dict[int, Track] = {}
        self.time: Optional[float] = None
        self._ids = itertools.count(1)

    # -- time ---------------------------------------------------------------
    def predict_to(self, t: float) -> None:
        """Propagate all filters to time t and update the life-cycle states."""
        if self.time is not None and t > self.time:
            dt = t - self.time
            for tr in self.tracks.values():
                if tr.state != NEUTRALIZED:
                    tr.kf.predict(dt)
                    tr.kf.clamp_speed(self.p.max_speed)
                if tr.state == LOST and self.p.lost_velocity_tau > 0.0:
                    tr.kf.x[2:] *= math.exp(-dt / self.p.lost_velocity_tau)
        if self.time is None or t > self.time:
            self.time = t
        self._update_states(self.time)

    def _update_states(self, t: float) -> None:
        for tid in list(self.tracks):
            tr = self.tracks[tid]
            silent = t - tr.last_update
            if tr.state == TENTATIVE and t - tr.first_seen > self.p.confirm_window:
                del self.tracks[tid]
            elif tr.state == ALERT and silent > self.p.alert_timeout:
                del self.tracks[tid]
            elif tr.state == TRACKED and silent > self.p.lost_timeout:
                self._set_state(tr, LOST, t)
            elif tr.state == LOST and t - tr.state_since > self.p.delete_after:
                del self.tracks[tid]
            elif tr.state == NEUTRALIZED and silent > self.p.neutralized_keep:
                del self.tracks[tid]

    @staticmethod
    def _set_state(tr: Track, state: int, t: float) -> None:
        if tr.state != state:
            tr.state = state
            tr.state_since = t

    # -- measurements -------------------------------------------------------
    def update_detections(self, detections: List[XY], t: float) -> List[Tuple[int, int]]:
        """Associate lidar detections with tracks and update them.

        Returns the (track id, detection index) matches.
        """
        self.predict_to(t)
        # Neutralized tracks still take part, so a detained person standing
        # still does not spawn new tracks; their state does not change.
        active = list(self.tracks.values())
        cost = np.array([[tr.kf.mahalanobis2(np.array(z), self.p.meas_std) for z in detections]
                         for tr in active]).reshape(len(active), len(detections))
        matches, _, unmatched = associate(cost, self.p.gate_chi2)
        result = []
        for i, j in matches:
            tr = active[i]
            if tr.state == NEUTRALIZED:
                tr.last_update = t
                result.append((tr.id, j))
                continue
            tr.kf.update(np.array(detections[j]), self.p.meas_std)
            tr.kf.clamp_speed(self.p.max_speed)
            tr.last_update = t
            tr.hits += 1
            if tr.state == TENTATIVE and tr.hits >= self.p.confirm_hits:
                self._set_state(tr, TRACKED, t)
            elif tr.state in (ALERT, LOST):
                self._set_state(tr, TRACKED, t)
            result.append((tr.id, j))
        for j in unmatched:
            tr = self._new_track(detections[j], self.p.meas_std, TENTATIVE, t)
            result.append((tr.id, j))
        return result

    def update_fence_alert(self, position: XY, t: float) -> int:
        """Coarse fence sensor measurement. Updates the closest gated track or
        creates an ALERT track. Returns the track id."""
        self.predict_to(t)
        best, best_d2 = None, self.p.gate_chi2
        for tr in self.tracks.values():
            if tr.state in (NEUTRALIZED, TENTATIVE):
                continue
            d2 = tr.kf.mahalanobis2(np.array(position), self.p.fence_std)
            if d2 <= best_d2:
                best, best_d2 = tr, d2
        if best is not None:
            best.kf.update(np.array(position), self.p.fence_std)
            if best.state == ALERT:
                best.last_update = t
            return best.id
        return self._new_track(position, self.p.fence_std, ALERT, t).id

    def _new_track(self, z: XY, pos_std: float, state: int, t: float) -> Track:
        kf = ConstantVelocityKF(z[0], z[1], pos_std, self.p.init_vel_std, self.p.accel_std)
        tr = Track(id=next(self._ids), kf=kf, state=state, first_seen=t, last_update=t,
                   state_since=t)
        self.tracks[tr.id] = tr
        return tr

    def neutralize(self, track_id: int, t: float) -> bool:
        tr = self.tracks.get(track_id)
        if tr is None:
            return False
        self._set_state(tr, NEUTRALIZED, t)
        tr.kf.x[2:] = 0.0
        return True

    # -- output -------------------------------------------------------------
    def published(self) -> List[Track]:
        """Tracks worth reporting (not tentative), ordered by id."""
        return sorted((tr for tr in self.tracks.values() if tr.confirmed), key=lambda tr: tr.id)

    def speed(self, track_id: int) -> float:
        vx, vy = self.tracks[track_id].kf.velocity
        return math.hypot(vx, vy)
