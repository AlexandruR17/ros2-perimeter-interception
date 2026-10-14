"""Detection to track association.

Global nearest neighbour with the Hungarian algorithm on Mahalanobis
distances, with chi-square gating. A greedy nearest neighbour can swap
identities when two people pass close to each other (an intruder walking
past a soldier); the optimal one-to-one assignment minimises the total cost
over all pairs and keeps identities stable in those cases.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

# Chi-square 99 % quantile for 2 degrees of freedom.
GATE_CHI2_99 = 9.21
_INFEASIBLE = 1e9


def associate(cost: np.ndarray, gate: float = GATE_CHI2_99
              ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Optimal one-to-one assignment of rows (tracks) to columns (detections).

    cost[i, j] is the squared Mahalanobis distance of detection j to track i.
    Pairs above the gate are never matched.
    Returns (matches, unmatched_rows, unmatched_cols).
    """
    cost = np.asarray(cost, dtype=float)
    n_rows, n_cols = cost.shape if cost.ndim == 2 else (0, 0)
    if n_rows == 0 or n_cols == 0:
        return [], list(range(n_rows)), list(range(n_cols))

    gated = np.where(cost <= gate, cost, _INFEASIBLE)
    rows, cols = linear_sum_assignment(gated)
    matches = [(int(r), int(c)) for r, c in zip(rows, cols) if gated[r, c] < _INFEASIBLE]
    matched_r = {r for r, _ in matches}
    matched_c = {c for _, c in matches}
    return (matches,
            [r for r in range(n_rows) if r not in matched_r],
            [c for c in range(n_cols) if c not in matched_c])


def euclidean_cost(a: Sequence[Tuple[float, float]], b: Sequence[Tuple[float, float]]) -> np.ndarray:
    """Squared Euclidean distance matrix, used for fusing detections and transponders."""
    if not len(a) or not len(b):
        return np.zeros((len(a), len(b)))
    pa = np.asarray(a, dtype=float)[:, None, :]
    pb = np.asarray(b, dtype=float)[None, :, :]
    return np.sum((pa - pb) ** 2, axis=2)
