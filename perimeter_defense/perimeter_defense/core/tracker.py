"""Multi-target tracker: one constant velocity Kalman filter per target.

Life cycle of a track:
  tentative  -> created from an unmatched lidar detection, not published;
                confirmed after confirm_hits detections within confirm_window
  ALERT      -> created from a fence alert or a soldier spot report only
                (coarse position); confirmed when a lidar track within
                alert_merge_radius is classified HOSTILE by the IFF: the
                ALERT track (and its id) takes over the lidar filter. Waiting
                for the IFF keeps an own soldier near the alert from taking
                it over.
  TRACKED    -> confirmed by lidar detections
  LOST       -> no detection for lost_timeout; the filter keeps predicting
  NEUTRALIZED-> set externally when a robot reached the target; keeps
                following the detections of that person (a captured person
                is escorted away, and must not spawn a new track)
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
    max_assoc_distance: float = 3.0   # m, detection to track, on top of the chi-square gate
    alert_merge_radius: float = 8.0   # m, hostile lidar track that confirms an ALERT track
    duplicate_distance: float = 0.7   # m, two confirmed tracks this close follow one person
    captured_merge_distance: float = 1.0  # m, a track this close to a captured person is him


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
        # Neutralized tracks still take part, so a captured person (standing,
        # then escorted) does not spawn new tracks; their state does not change.
        # ALERT tracks have a coarse position and a large covariance that would
        # gate anyone nearby: they are confirmed by new lidar tracks instead.
        active = [tr for tr in self.tracks.values() if tr.state != ALERT]
        cost = np.array([[self._cost(tr, z) for z in detections]
                         for tr in active]).reshape(len(active), len(detections))
        matches, _, unmatched = associate(cost, self.p.gate_chi2)
        result = []
        for i, j in matches:
            tr = active[i]
            tr.kf.update(np.array(detections[j]), self.p.meas_std)
            tr.kf.clamp_speed(self.p.max_speed)
            tr.last_update = t
            if tr.state == NEUTRALIZED:
                result.append((tr.id, j))
                continue
            tr.hits += 1
            if tr.state == TENTATIVE and tr.hits >= self.p.confirm_hits:
                self._set_state(tr, TRACKED, t)
            elif tr.state == LOST:
                self._set_state(tr, TRACKED, t)
            result.append((tr.id, j))
        for j in unmatched:
            tr = self._new_track(detections[j], self.p.meas_std, TENTATIVE, t)
            result.append((tr.id, j))
        self._merge_duplicates()
        return result

    def _merge_duplicates(self) -> None:
        """Two confirmed tracks on one person are merged.

        A LOST track predicted past a person who turned back can be re-created
        as a second track; the one with the freshest detection is kept (then
        the older one). A track that appears on a captured person (missed
        detections while he is escorted) is merged into the captured track,
        which keeps its identity and state.
        """
        live = [tr for tr in self.tracks.values() if tr.state in (TRACKED, LOST, NEUTRALIZED)]
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                if a.id not in self.tracks or b.id not in self.tracks:
                    continue
                captured = NEUTRALIZED in (a.state, b.state)
                limit = self.p.captured_merge_distance if captured else self.p.duplicate_distance
                if math.dist(a.kf.position, b.kf.position) > limit or a.state == b.state == NEUTRALIZED:
                    continue
                if captured:
                    keep, drop = (a, b) if a.state == NEUTRALIZED else (b, a)
                    if drop.last_update > keep.last_update:
                        keep.kf, keep.last_update = drop.kf, drop.last_update
                else:
                    keep, drop = sorted((a, b), key=lambda tr: (-tr.last_update, tr.id))
                    if drop.classification == FRIEND and keep.classification != FRIEND:
                        keep.classification, keep.transponder_id = FRIEND, drop.transponder_id
                del self.tracks[drop.id]

    def _cost(self, tr: Track, z: XY) -> float:
        px, py = tr.kf.position
        if math.hypot(z[0] - px, z[1] - py) > self.p.max_assoc_distance:
            return math.inf
        return tr.kf.mahalanobis2(np.array(z), self.p.meas_std)

    def update_fence_alert(self, position: XY, t: float) -> int:
        """Fence sensor measurement (breach centre, fence_std accuracy)."""
        return self.update_coarse(position, self.p.fence_std, t)

    def update_coarse(self, position: XY, std: float, t: float) -> int:
        """Coarse report (fence sensor, soldier sighting). Updates the closest
        gated track or creates an ALERT track. Friendly and captured tracks are
        never matched. Returns the track id."""
        self.predict_to(t)
        best, best_d2 = None, self.p.gate_chi2
        for tr in self.tracks.values():
            if tr.state in (NEUTRALIZED, TENTATIVE) or tr.classification == FRIEND:
                continue
            d2 = tr.kf.mahalanobis2(np.array(position), std)
            if d2 <= best_d2:
                best, best_d2 = tr, d2
        if best is not None:
            # A lidar track is far more accurate than the report: the report
            # only confirms it. Pulling it towards a coarse position would make
            # it grab the detections of another person nearby.
            if best.state != TRACKED:
                best.kf.update(np.array(position), std)
            if best.state == ALERT:
                best.last_update = t
            return best.id
        return self._new_track(position, std, ALERT, t).id

    def set_classification(self, track_id: int, classification: int, transponder: str = '') -> int:
        """Store the IFF class. A lidar track found HOSTILE confirms the closest
        ALERT track, which takes it over. Returns the id the track now has."""
        tr = self.tracks.get(track_id)
        if tr is None:
            return track_id
        tr.classification, tr.transponder_id = classification, transponder
        if classification != HOSTILE or tr.state != TRACKED:
            return track_id
        pos = tr.kf.position
        alerts = [a for a in self.tracks.values() if a.state == ALERT and a.classification != FRIEND]
        best = min(alerts, key=lambda a: math.dist(a.kf.position, pos), default=None)
        if best is None or math.dist(best.kf.position, pos) > self.p.alert_merge_radius:
            return track_id
        best.kf, best.hits, best.last_update = tr.kf, tr.hits, tr.last_update
        best.classification, best.transponder_id = classification, transponder
        self._set_state(best, TRACKED, self.time if self.time is not None else tr.last_update)
        del self.tracks[track_id]
        return best.id

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
