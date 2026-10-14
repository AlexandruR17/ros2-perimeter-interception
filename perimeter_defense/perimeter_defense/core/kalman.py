"""Constant velocity Kalman filter, state [x, y, vx, vy] in the map frame."""

from __future__ import annotations

from typing import Tuple

import numpy as np

H = np.array([[1.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0]])


class ConstantVelocityKF:
    """Linear Kalman filter with a white-noise acceleration model.

    accel_std is the standard deviation of the unmodelled acceleration
    (m/s^2); people change speed and direction often, so it is fairly large.
    """

    def __init__(self, x: float, y: float, pos_std: float, vel_std: float,
                 accel_std: float, vx: float = 0.0, vy: float = 0.0) -> None:
        self.x = np.array([x, y, vx, vy], dtype=float)
        self.P = np.diag([pos_std ** 2, pos_std ** 2, vel_std ** 2, vel_std ** 2])
        self.accel_std = float(accel_std)

    @staticmethod
    def transition(dt: float) -> np.ndarray:
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        return F

    def process_noise(self, dt: float) -> np.ndarray:
        q = self.accel_std ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        return q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                             [0, dt4 / 4, 0, dt3 / 2],
                             [dt3 / 2, 0, dt2, 0],
                             [0, dt3 / 2, 0, dt2]])

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        F = self.transition(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.process_noise(dt)

    def predicted(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """State and covariance after dt, without changing the filter."""
        F = self.transition(max(dt, 0.0))
        return F @ self.x, F @ self.P @ F.T + self.process_noise(max(dt, 0.0))

    def innovation(self, z: np.ndarray, meas_std: float) -> Tuple[np.ndarray, np.ndarray]:
        """Innovation y = z - Hx and its covariance S."""
        R = np.eye(2) * meas_std ** 2
        y = np.asarray(z, dtype=float) - H @ self.x
        S = H @ self.P @ H.T + R
        return y, S

    def mahalanobis2(self, z: np.ndarray, meas_std: float) -> float:
        y, S = self.innovation(z, meas_std)
        return float(y @ np.linalg.solve(S, y))

    def update(self, z: np.ndarray, meas_std: float) -> None:
        y, S = self.innovation(z, meas_std)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        # Joseph form keeps P symmetric and positive definite.
        I_KH = np.eye(4) - K @ H
        R = np.eye(2) * meas_std ** 2
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def clamp_speed(self, max_speed: float) -> None:
        """Limit the estimated speed to what a person can do."""
        speed = float(np.hypot(self.x[2], self.x[3]))
        if speed > max_speed > 0.0:
            self.x[2:] *= max_speed / speed

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2]), float(self.x[3])

    def position_std(self) -> float:
        """Radius of the 1-sigma position uncertainty (largest axis)."""
        return float(np.sqrt(np.max(np.linalg.eigvalsh(self.P[:2, :2]))))
