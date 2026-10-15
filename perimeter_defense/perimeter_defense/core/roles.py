"""Role allocation of the interceptor UGVs (M4).

Every period the available interceptors get a role:

  ENGAGE   intercept a threat (the node turns it into INVESTIGATE for an
           alert, INTERCEPT / FOLLOW for a lidar track, SEARCH for a lost one)
  GUARD    stand between the most urgent threat and the objective
  FLANK    cut the retreat: hold the breach the threat came in by
  BLOCK    same position when the threat is only an alert

Predictive mode:
  - threats are ranked by their time to the objective;
  - one robot always GUARDs while a threat is active (if at least two robots
    are available); the others ENGAGE the most urgent threats, the robot to
    threat assignment minimising the total interception time (exhaustive, the
    fleet is small);
  - robots left over FLANK the most urgent tracked threat (or BLOCK for an
    alert).
  - hysteresis: the previous assignment is kept unless the new one is better
    by more than max(switch_abs, switch_rel * cost).
Naive mode: every robot ENGAGEs the threat closest to it, nobody guards.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Sequence, Tuple

ENGAGE, GUARD, FLANK, BLOCK = 'ENGAGE', 'GUARD', 'FLANK', 'BLOCK'

Assignment = Dict[str, Tuple[str, int]]      # robot -> (role, track id; 0 for GUARD)


@dataclass
class ThreatInfo:
    id: int
    tracked: bool               # lidar track (TRACKED or LOST) rather than an alert
    time_to_objective: float    # s, priority: smaller is more urgent


@dataclass
class RoleParams:
    switch_abs: float = 1.0     # s, minimum improvement to change the assignment
    switch_rel: float = 0.15
    infeasible_penalty: float = 30.0   # s added when the robot cannot intercept


def allocate(robots: Sequence[str], threats: Sequence[ThreatInfo],
             engage_time: Callable[[str, int], float],
             guard_time: Callable[[str], float],
             mode: str = 'predictive',
             previous: Optional[Assignment] = None,
             params: Optional[RoleParams] = None) -> Assignment:
    """robots: available interceptors; engage_time(robot, track) and guard_time(robot)
    in seconds (inf when unreachable)."""
    p = params or RoleParams()
    if not robots or not threats:
        return {}
    if mode == 'naive':
        return {r: (ENGAGE, min(threats, key=lambda t: engage_time(r, t.id)).id) for r in robots}

    ranked = sorted(threats, key=lambda t: t.time_to_objective)
    n_engage = 1 if len(robots) == 1 else min(len(ranked), len(robots) - 1)
    targets = ranked[:n_engage]
    best, best_cost = _best_assignment(robots, targets, engage_time, guard_time)

    if previous is not None:
        prev_cost = _cost_of(previous, robots, targets, engage_time, guard_time)
        if prev_cost is not None and prev_cost <= best_cost + max(p.switch_abs, p.switch_rel * best_cost):
            best = {r: v for r, v in previous.items() if v[0] in (ENGAGE, GUARD)}

    out = dict(best)
    rest = [r for r in robots if r not in out]
    lead = ranked[0]
    for r in rest:
        out[r] = (FLANK if lead.tracked else BLOCK, lead.id)
    return out


def _best_assignment(robots: Sequence[str], targets: Sequence[ThreatInfo],
                     engage_time: Callable[[str, int], float],
                     guard_time: Callable[[str], float]) -> Tuple[Assignment, float]:
    best: Assignment = {}
    best_cost = math.inf
    for chosen in itertools.permutations(robots, len(targets)):
        cost = sum(engage_time(r, t.id) for r, t in zip(chosen, targets))
        assignment = {r: (ENGAGE, t.id) for r, t in zip(chosen, targets)}
        rest = [r for r in robots if r not in chosen]
        if rest:
            g = min(rest, key=guard_time)
            assignment[g] = (GUARD, 0)
            # The guard matters less than the interceptions, but a far guard is worse.
            cost += 0.25 * guard_time(g)
        if cost < best_cost:
            best, best_cost = assignment, cost
    return best, best_cost


def _cost_of(previous: Assignment, robots: Sequence[str], targets: Sequence[ThreatInfo],
             engage_time: Callable[[str, int], float],
             guard_time: Callable[[str], float]) -> Optional[float]:
    """Cost of keeping the previous ENGAGE / GUARD roles, None if no longer valid."""
    engaged = {tid: r for r, (role, tid) in previous.items() if role == ENGAGE}
    if set(engaged) != {t.id for t in targets} or any(r not in robots for r in engaged.values()):
        return None
    cost = sum(engage_time(engaged[t.id], t.id) for t in targets)
    guards = [r for r, (role, _) in previous.items() if role == GUARD and r in robots]
    rest = [r for r in robots if r not in engaged.values()]
    if rest:
        if not guards:
            return None
        cost += 0.25 * guard_time(guards[0])
    return cost


def point_along(path: Sequence[Tuple[float, float]], distance: float) -> Tuple[float, float]:
    """Point at a given distance along a polyline (its end if shorter)."""
    left = distance
    for a, b in zip(path, path[1:]):
        seg = math.dist(a, b)
        if seg >= left and seg > 0:
            f = left / seg
            return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
        left -= seg
    return tuple(path[-1])
